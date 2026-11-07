"""语义实例合并：把指同一物体的重叠实例合成一个，保留来源可追溯。

触发时机：每批关联结束时检查本批变化的实例与其空间候选。
保留首次出现最早的实例 ID；被合并的 ID 记入保留实例的
merged_instance_ids，由 MemoryService 负责持久化删除、别名解析与观察事件改指。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Mapping, Optional, Tuple

from contracts.spatial_memory import SemanticInstance, InstanceConfirmationState, InstanceLifecycleState, InstanceDynamicState
from contracts.frame_semantics import frame_ids_semantically_equal
from services.memory.instance_evidence import (accumulate_support_voxels, support_extent, fuse_position,
    primary_label, cross_label_geometry_compatible, record_support_evidence,
    cooccurrence_evidence, merge_cooccurrence_metadata, remember_coobserved_instances,
    observation_source_index, historical_coobserved_ids, same_frame_distinct_same_label)
from services.memory.object_extent import resolve_object_extent
from services.memory.spatial_candidate_index import SpatialCandidateIndex
import math
import numpy as np


@dataclass(frozen=True)
class InstanceMerge:
    survivor_id: str
    merged_ids: Tuple[str, ...]
    reason: str


class InstanceConsolidator:
    """按几何重合与标签规则找出应合并的实例组。

    同标签沿空间关联规则；不同标签使用小方包含率、逐轴尺寸比与永久共现否决。
    地图域、已知地图资产与坐标系不一致的不比较。
    """

    def __init__(self, *, config) -> None:
        self._config = config

    def find_merges(
        self,
        instances: Mapping[str, SemanticInstance],
        *,
        candidate_ids: Optional[Iterable[str]] = None,
    ) -> Tuple[InstanceMerge, ...]:
        """candidate_ids 为空表示检查全部实例；结果与字典遍历顺序无关。"""
        from services.memory.instance_association_service import InstanceAssociationService
        # 复用关联的范围和误差规则；不调用模型推理，也不保留跨批可变索引。
        policy=InstanceAssociationService(text_embedder=None,association_config=self._config)
        bounds={key:policy._spatial_bounds(value) for key,value in instances.items()
                if value.lifecycle_state != InstanceLifecycleState.REMOVED}
        index=SpatialCandidateIndex((key,value) for key,value in bounds.items() if value is not None)
        candidates=set(instances) if candidate_ids is None else set(candidate_ids)
        parent={key:key for key in instances}
        members={key:{key} for key in instances}
        def root(key):
            while parent[key]!=key:
                key=parent[key]
            return key
        def compatible(a,b):
            if a.map_name!=b.map_name or not frame_ids_semantically_equal(a.pose.frame_id,b.pose.frame_id):
                return False
            aa,bb=a.metadata.get("map_asset_id"),b.metadata.get("map_asset_id")
            if aa and bb and aa!=bb:
                return False
            if a.confirmation_state==b.confirmation_state==InstanceConfirmationState.TENTATIVE:
                return False
            if same_frame_distinct_same_label(a,b,self._config):
                return False
            if cooccurrence_evidence(a,b,self._config) is not None:
                return False
            if policy._conflicting_appearance(a,b) or policy._geometry_fit(a,b) is None:
                return False
            return (a.label.strip().lower()==b.label.strip().lower()
                    or cross_label_geometry_compatible(a,b,self._config))
        pairs=set()
        for key in sorted(candidates):
            if bounds.get(key) is not None:
                pairs.update(tuple(sorted((key,other))) for other in index.query(bounds[key]) if other!=key)
        for a,b in sorted(pairs):
            ra,rb=root(a),root(b)
            if ra==rb:
                continue
            # 分组合并要求组间两两相容，避免A邻B、B邻C的链式误并。
            if not all(compatible(instances[x],instances[y]) for x in members[ra] for y in members[rb]):
                continue
            parent[rb]=ra
            members[ra].update(members.pop(rb))
        merges=[]
        for keys in members.values():
            if len(keys)<2:
                continue
            confirmed=[key for key in keys if instances[key].confirmation_state==InstanceConfirmationState.CONFIRMED]
            survivor=min(confirmed,key=lambda key:(instances[key].first_seen_ts,key))
            merges.append(InstanceMerge(survivor,tuple(sorted(keys-{survivor})),"spatial_identity_overlap"))
        return tuple(sorted(merges,key=lambda item:item.survivor_id))

    def apply(
        self,
        instances: Mapping[str, SemanticInstance],
        merges: Iterable[InstanceMerge],
    ) -> Dict[str, SemanticInstance]:
        """返回合并后的实例字典：保留方累积观察次数、标签票数、体素、证据引用与别名。"""
        merges = tuple(merges)
        if not merges:
            return dict(instances)
        result=dict(instances)
        source_index=observation_source_index(instances.values())
        for merge in merges:
            current=result[merge.survivor_id]
            records=[current]+[result[key] for key in merge.merged_ids]
            # 合并窗口会截断旧样本；先将与组外实例的旧共现证明迁移到永久元数据。
            records=[remember_coobserved_instances(record,historical_coobserved_ids(record,source_index,self._config))
                     for record in records]
            current=records[0]
            evidence=record_support_evidence(current,self._config)
            pose=current.pose
            weight=evidence.position_weight if evidence is not None and evidence.position_weight>0 else current.observation_count/0.005**2
            votes={}
            for record in records:
                ev=record_support_evidence(record,self._config)
                for label,score in (ev.label_votes if ev is not None and ev.label_votes else {record.label:record.confidence*record.observation_count}).items():
                    votes[label]=votes.get(label,0.)+score
                if record is current:
                    continue
                if ev is not None and ev.position_weight>0:
                    sigma=math.sqrt(1/ev.position_weight)
                else:
                    sigma=0.005/math.sqrt(max(1,record.observation_count))
                pose,weight,_=fuse_position(pose,weight,record.pose,sigma)
                points=((np.asarray(ev.voxel_keys).reshape(-1,3)+0.5)*ev.voxel_size_m
                        if ev is not None and ev.voxel_keys else np.empty((0,3)))
                evidence=accumulate_support_voxels(evidence,points,frame_id=pose.frame_id,
                    voxel_size_m=self._config.support_voxel_size_m,max_voxels=self._config.support_max_voxels)
            if evidence is None:
                evidence=accumulate_support_voxels(None,np.empty((0,3)),frame_id=pose.frame_id,
                    voxel_size_m=self._config.support_voxel_size_m,max_voxels=self._config.support_max_voxels)
            label=primary_label(votes,current.label)
            latest=max(records,key=lambda r:(r.last_seen_ts,r.instance_id))
            if latest.dynamic_state in (InstanceDynamicState.DYNAMIC,InstanceDynamicState.QUASI_STATIC):
                pose=latest.pose
                weight=latest.support_evidence.position_weight if latest.support_evidence else 1/0.005**2
            evidence=evidence.model_copy(update={"label_votes":votes,"position_weight":weight,"position_sigma_m":math.sqrt(1/weight)})
            def union(field):
                return list(dict.fromkeys(value for record in records for value in getattr(record,field)))
            metadata=dict(latest.metadata)
            metadata.update(merge_cooccurrence_metadata(records))
            names={key:value for record in records for key,value in (record.metadata.get("label_display_names") or {record.label:record.display_name}).items()}
            metadata["label_display_names"]=names
            updates={"label":label,"display_name":names.get(label,label),"pose":pose,"metadata":metadata,
                "first_seen_ts":min(r.first_seen_ts for r in records),"last_seen_ts":max(r.last_seen_ts for r in records),
                "observation_count":sum(r.observation_count for r in records),"confidence":max(r.confidence for r in records),
                "support_evidence":evidence,"merged_instance_ids":list(dict.fromkeys(union("merged_instance_ids")+list(merge.merged_ids))),
                "semantic_labels":list(dict.fromkeys(union("semantic_labels")+[r.label for r in records])),
                "visual_labels":list(dict.fromkeys(union("visual_labels")+[r.label for r in records])),
                "vision_tags":union("vision_tags"),"artifact_ids":union("artifact_ids"),
                "supporting_observation_event_ids":union("supporting_observation_event_ids"),
                "last_observation_event_id":latest.last_observation_event_id,
                "observation_window":sorted((s for r in records for s in r.observation_window),key=lambda s:s.timestamp)[-12:]}
            detector=next((resolve_object_extent(r) for r in sorted(records,key=lambda r:r.last_seen_ts,reverse=True)
                           if resolve_object_extent(r) is not None and resolve_object_extent(r)["source"]=="detection3d"),None)
            extent=detector or support_extent(evidence,pose=pose)
            if extent is not None:
                if detector:
                    source=next(r for r in records if resolve_object_extent(r)==detector)
                    extent=dict(detector,center_offset_m={k:detector["center_offset_m"][k]+getattr(source.pose.position,k)-getattr(pose.position,k) for k in "xyz"})
                metadata["object_extent"]=extent
            result[merge.survivor_id]=current.model_copy(update=updates)
            for key in merge.merged_ids:
                result.pop(key)
        return result
