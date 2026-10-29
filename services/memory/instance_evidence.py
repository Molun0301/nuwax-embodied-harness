"""语义实例的测量误差、体素证据、位置融合和标签投票。"""
from __future__ import annotations

import math
from types import SimpleNamespace
from typing import Dict, Mapping, Optional, Tuple
import numpy as np

from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.geometry import Pose
from contracts.spatial_memory import InstanceSupportEvidence

ADMISSION_AUTOMATIC = "automatic"
ADMISSION_EXPLICIT = "explicit"


def measurement_sigma_m(attributes: Mapping[str, object]) -> Optional[float]:
    """读取测量误差；旧可靠投影使用距离误差下限，回退位置没有测量权重。"""
    if attributes.get("projection_method") == "current_pose_fallback":
        return None
    for key in ("measurement_sigma_m", "triangulation_residual_m"):
        value = attributes.get(key)
        if type(value) in (int,float) and math.isfinite(value) and value > 0:
            return float(value)
    depth = attributes.get("depth_m")
    depth = float(depth) if type(depth) in (int,float) and math.isfinite(depth) and depth > 0 else 0.0
    floor = 0.005 + 0.002*depth*depth
    angle = attributes.get("ray_cone_half_angle_rad")
    if depth and type(angle) in (int,float) and math.isfinite(angle) and 0 < angle < math.pi/2:
        return max(floor,depth*math.tan(angle)/math.sqrt(3.0))
    return floor


def observation_support_points(attributes: Mapping[str, object], *, frame_id: str) -> Optional[np.ndarray]:
    voxels = attributes.get("support_voxels_map")
    if isinstance(voxels,dict) and voxels.get("frame_id") and frame_ids_semantically_equal(voxels["frame_id"],frame_id):
        keys=np.asarray(voxels.get("voxel_keys") or [],dtype=np.int64).reshape(-1,3)
        if len(keys):
            return (keys+0.5)*float(voxels["voxel_size_m"])
    payload = attributes.get("support_points_map") or attributes.get("map_points")
    if not isinstance(payload, dict) or not payload.get("frame_id") or not frame_ids_semantically_equal(payload["frame_id"],frame_id):
        return None
    points = []
    for point in payload.get("points") or ():
        try:
            xyz = [float(point[k]) for k in "xyz"] if isinstance(point,dict) else [float(v) for v in point[:3]]
        except (KeyError,TypeError,ValueError):
            continue
        if len(xyz)==3 and all(math.isfinite(v) for v in xyz):
            points.append(xyz)
    return np.asarray(points,dtype=np.float64).reshape(-1,3) if points else None


def accumulate_support_voxels(evidence: Optional[InstanceSupportEvidence], points: np.ndarray, *,
                              frame_id: str, voxel_size_m: float, max_voxels: int) -> InstanceSupportEvidence:
    keys = set()
    compatible = evidence is not None and frame_ids_semantically_equal(evidence.frame_id,frame_id)
    if compatible and evidence.voxel_keys:
        old = np.asarray(evidence.voxel_keys,dtype=np.int64).reshape(-1,3)
        if evidence.voxel_size_m != voxel_size_m:
            old = np.floor((old+0.5)*evidence.voxel_size_m/voxel_size_m).astype(np.int64)
        keys.update(map(tuple,old.tolist()))
    points = np.asarray(points,dtype=np.float64).reshape(-1,3)
    points = points[np.isfinite(points).all(axis=1)]
    keys.update(map(tuple,np.floor(points/voxel_size_m).astype(np.int64).tolist()))
    # 稳定散列蓄水池：重复和输入顺序不改变结果，避免截取字典序前缀使范围偏向一侧。
    def rank(key):
        x,y,z = key
        return ((x*73856093) ^ (y*19349663) ^ (z*83492791)) & 0xffffffffffffffff, key
    if len(keys)>max_voxels:
        keys = set(sorted(keys,key=rank)[:max_voxels])
    updates = {"frame_id":frame_id,"voxel_size_m":voxel_size_m,
               "voxel_keys":[v for key in sorted(keys) for v in key]}
    if compatible:
        return evidence.model_copy(update=updates)
    return InstanceSupportEvidence(**updates)


def support_extent(evidence: Optional[InstanceSupportEvidence], *, pose: Pose) -> Optional[Dict[str, object]]:
    if evidence is None or not evidence.voxel_keys or not frame_ids_semantically_equal(evidence.frame_id,pose.frame_id):
        return None
    points = (np.asarray(evidence.voxel_keys).reshape(-1,3)+0.5)*evidence.voxel_size_m
    low, high = np.quantile(points,[0.05,0.95],axis=0)
    low -= evidence.voxel_size_m/2
    high += evidence.voxel_size_m/2
    center = (low+high)/2
    return {"frame_id":pose.frame_id,"size_m":dict(zip("xyz",(high-low).tolist())),
            "center_offset_m":{axis:float(center[i])-getattr(pose.position,axis) for i,axis in enumerate("xyz")},
            "orientation":{"x":0.,"y":0.,"z":0.,"w":1.},"source":"observed_points",
            "source_observation_id":""}


def fuse_position(current: Pose, current_weight: float, observation: Pose, observation_sigma_m: float) -> Tuple[Pose,float,float]:
    weight = 1.0/max(1e-12,observation_sigma_m**2)
    total = max(0.,current_weight)+weight
    position = current.position.model_copy(update={axis:(getattr(current.position,axis)*max(0.,current_weight)+getattr(observation.position,axis)*weight)/total for axis in "xyz"})
    return observation.model_copy(update={"position":position}), total, math.sqrt(1/total)


def update_label_votes(votes: Optional[Mapping[str,float]], label: str, score: float) -> Dict[str,float]:
    result = dict(votes or {})
    label = label.strip().lower()
    if label and math.isfinite(score) and score>0:
        result[label] = result.get(label,0.)+score
    return result


def primary_label(votes: Optional[Mapping[str,float]], fallback: str) -> str:
    return min(votes,key=lambda k:(-votes[k],k)) if votes else fallback


def support_overlap(left: InstanceSupportEvidence, right: InstanceSupportEvidence) -> Tuple[float,float]:
    if not left.voxel_keys or not right.voxel_keys or not frame_ids_semantically_equal(left.frame_id,right.frame_id):
        return 0.,0.
    a=(np.asarray(left.voxel_keys).reshape(-1,3)+0.5)*left.voxel_size_m
    b=(np.asarray(right.voxel_keys).reshape(-1,3)+0.5)*right.voxel_size_m
    tolerance=(left.voxel_size_m+right.voxel_size_m)/2 + 1e-10
    # 分块避免候选间制造无界三维数组。
    def fraction(x,y):
        return sum(np.count_nonzero(np.any(np.max(np.abs(chunk[:,None,:]-y[None,:,:]),axis=2)<=tolerance,axis=1))
                   for chunk in np.array_split(x,max(1,int(np.ceil(len(x)/64.))))) / float(len(x))
    return fraction(a,b),fraction(b,a)


def record_support_evidence(record,config):
    """旧实例仍可能保留原始地图支持点；比较和合并时复用，不能当作没有几何证据。"""
    evidence=getattr(record,"support_evidence",None)
    if evidence is not None:
        return evidence
    attributes={**(getattr(record,"metadata",None) or {}),**record.attributes}
    points=observation_support_points(attributes,frame_id=record.pose.frame_id)
    if points is None:
        return None
    return accumulate_support_voxels(None,points,frame_id=record.pose.frame_id,
        voxel_size_m=config.support_voxel_size_m,max_voxels=config.support_max_voxels)


# 伙伴及独立来源有界；容量饱和只作诊断，不能代替两源分离证明。
COOBSERVED_INSTANCE_IDS = "coobserved_instance_ids"
COOBSERVATION_SATURATED = "coobservation_saturated"
MAX_COOBSERVED_INSTANCES = 256


def instance_identity_ids(record):
    """合并前后的标识属于同一身份，旧伙伴引用仍然有效。"""
    instance_id=getattr(record,"instance_id",None)
    return {instance_id,*getattr(record,"merged_instance_ids",())} if instance_id else set()


def observation_source_key(record):
    source=getattr(record,"source_observation_id",None)
    if source:
        return str(source),str(record.camera_id or "")
    return None


def observation_window_sources(record):
    """只读现有窗口，兼容尚未写入永久共现伙伴的旧实例。"""
    keys=set()
    for sample in getattr(record,"observation_window",()):
        source=sample.metadata.get("source_observation_id")
        if source:
            keys.add((str(source),str(sample.metadata.get("camera_id") or "")))
    return keys


def observation_source_index(records):
    """窗口来源的倒排索引，用于在窗口滚动或合并裁剪前迁移旧共现证明。"""
    result={}
    for record in records:
        for source in observation_window_sources(record):
            result.setdefault(source,{})[record.instance_id]=record
    return result


COOBSERVATION_SOURCES = "coobservation_sources"


def measurement_geometry_fit(left,right,gate_sigma):
    """关联与共现共用逐轴门限和浮点边界；返回匹配评分，明确分开时为None。"""
    if left is None or right is None:
        return None
    residuals=[]
    for axis in range(3):
        delta=abs(left[0][axis]-right[0][axis])
        gate=.5*max(left[1][axis],right[1][axis])+gate_sigma*math.hypot(left[2],right[2])
        if delta>gate and not math.isclose(delta,gate,rel_tol=1e-12,abs_tol=0.):
            return None
        residuals.append(delta/gate if gate>0. else 0.)
    return 1.-max(residuals)


def historical_coobserved_ids(record,source_index,config=None):
    """窗口滚动前迁移已出现的分离来源；一次证据也保留，但不足以否决。"""
    others={key:other for source in observation_window_sources(record)
            for key,other in source_index.get(source,{}).items() if key not in instance_identity_ids(record)}
    result={}
    for key,other in others.items():
        proof=cooccurrence_sources(record,other,config)
        if proof["sources"] or proof["legacy"] or proof["known"]:
            result[key]=proof["sources"]
    return result


def cooccurrence_sources(left,right,config=None):
    """不同身份在独立源中的明确分离证据；旧无来源伙伴仅作一次弱证据。"""
    left_ids,right_ids=instance_identity_ids(left),instance_identity_ids(right)
    result={"sources":[],"legacy":False,"known":False}
    if not left_ids or not right_ids or left_ids & right_ids:
        return result
    sources=set()
    for record,ids in ((left,right_ids),(right,left_ids)):
        metadata=record.metadata
        stored=metadata.get(COOBSERVATION_SOURCES) or {}
        for peer in ids:
            values=stored.get(peer) or ()
            sources.update(values)
            if peer in (metadata.get(COOBSERVED_INSTANCE_IDS) or ()):
                result["known"]=True
                if peer not in stored:
                    result["legacy"]=True
    if config is None:
        from core.settings import MemoryInstanceAssociationConfig
        config=MemoryInstanceAssociationConfig()
    from services.memory.instance_association_service import InstanceAssociationService
    for source,camera in sorted(observation_window_sources(left) & observation_window_sources(right)):
        def sample(record):
            return next(item for item in reversed(record.observation_window)
                if item.metadata.get("source_observation_id")==source
                and (item.metadata.get("camera_id") or "")==camera)
        a,b=sample(left),sample(right)
        if "frame_geometry" not in a.metadata or "frame_geometry" not in b.metadata:
            result["legacy"]=True
            continue
        separated=False
        evaluated=False
        for ga in a.metadata["frame_geometry"] or ():
            for gb in b.metadata["frame_geometry"] or ():
                if ga is None or gb is None:
                    continue
                ra=SimpleNamespace(pose=Pose.model_validate(ga["pose"]),attributes=ga["attributes"],metadata=ga["attributes"])
                rb=SimpleNamespace(pose=Pose.model_validate(gb["pose"]),attributes=gb["attributes"],metadata=gb["attributes"])
                ag,bg=InstanceAssociationService._spatial_geometry(ra),InstanceAssociationService._spatial_geometry(rb)
                if ag is None or bg is None or not frame_ids_semantically_equal(ra.pose.frame_id,rb.pose.frame_id):
                    continue
                evaluated=True
                if measurement_geometry_fit(ag,bg,config.uncertainty_gate_sigma) is None:
                    separated=True
        if separated:
            sources.add(source)
        elif evaluated:
            # 同源精化可撤销该源的分离票，不将一次错误主体估计永久留在账上。
            sources.discard(source)
    result["sources"]=sorted(sources)[:2]
    return result


def cooccurrence_evidence(left,right,config=None):
    """至少两个可识别的独立源都明确分离才否决；旧来源未知不能证明独立。"""
    proof=cooccurrence_sources(left,right,config)
    if len(proof["sources"])>=2:
        return {"reason":"coobserved_instances","source_observation_ids":proof["sources"],
                "independent_source_count":len(proof["sources"])}
    return None


def same_frame_distinct_same_label(left,right,config):
    """保留最新同帧同标签的一对一分配；不产生永久或历史共现否决。"""
    if instance_identity_ids(left) & instance_identity_ids(right):
        return False
    lm,rm=left.metadata,right.metadata
    source=lm.get("last_source_observation_id")
    camera=lm.get("last_camera_id") or ""
    label=lm.get("last_observation_label")
    if not source or source!=rm.get("last_source_observation_id") or camera!=(rm.get("last_camera_id") or ""):
        return False
    if not label or label!=rm.get("last_observation_label"):
        return False
    def geometries(record):
        return next((sample.metadata.get("frame_geometry") or () for sample in reversed(record.observation_window)
            if sample.metadata.get("source_observation_id")==source
            and (sample.metadata.get("camera_id") or "")==camera),())
    for ga in geometries(left):
        for gb in geometries(right):
            if ga is None or gb is None:
                continue
            details=geometry_overlap_diagnostics(
                SimpleNamespace(pose=Pose.model_validate(ga["pose"]),attributes=ga["attributes"]),
                SimpleNamespace(pose=Pose.model_validate(gb["pose"]),attributes=gb["attributes"]),config)
            if details["reason"] in ("cross_label_size_ratio","cross_label_containment"):
                return True
    return False


def _bounded_cooccurrence_metadata(peers,sources,saturated=False):
    # 优先保留两源证明；溢出只作诊断，不凭容量饱和制造未知身份的否决。
    keys=sorted(peers,key=lambda key:(-len(sources.get(key) or ()),key))
    saturated=saturated or len(keys)>MAX_COOBSERVED_INSTANCES
    keys=keys[:MAX_COOBSERVED_INSTANCES]
    result={COOBSERVED_INSTANCE_IDS:keys,COOBSERVATION_SOURCES:{key:sorted(set(sources[key]))[:2]
            for key in keys if key in sources}}
    if saturated:
        result[COOBSERVATION_SATURATED]=True
    return result


def remember_coobserved_instances(instance,peer_ids):
    """复用元数据，按伙伴保存至多两个真实源标识；纯旧ID输入视为弱证据。"""
    peers=set(instance.metadata.get(COOBSERVED_INSTANCE_IDS) or ())
    sources={key:set(values) for key,values in (instance.metadata.get(COOBSERVATION_SOURCES) or {}).items()}
    peers.update(peer_ids)
    if isinstance(peer_ids,Mapping):
        for key,values in peer_ids.items():
            sources[key]=set(values)
    peers.difference_update(instance_identity_ids(instance))
    metadata={**instance.metadata,**_bounded_cooccurrence_metadata(peers,sources,
              bool(instance.metadata.get(COOBSERVATION_SATURATED)))}
    return instance.model_copy(update={"metadata":metadata})


def merge_cooccurrence_metadata(records):
    """并集保留真实独立来源和旧弱伙伴，旧伙伴ID仍可通过实例别名匹配。"""
    peers=set();sources={};own_ids=set();saturated=False
    for record in records:
        peers.update(record.metadata.get(COOBSERVED_INSTANCE_IDS) or ())
        for key,values in (record.metadata.get(COOBSERVATION_SOURCES) or {}).items():
            sources.setdefault(key,set()).update(values)
        own_ids.update(instance_identity_ids(record))
        saturated=saturated or bool(record.metadata.get(COOBSERVATION_SATURATED))
    peers.difference_update(own_ids)
    return _bounded_cooccurrence_metadata(peers,sources,saturated)


def cross_label_geometry_diagnostics(left,right,config):
    """关联和合并共用的小方包含、逐轴尺寸比与永久共现否决，返回实际数值。"""
    veto=cooccurrence_evidence(left,right,config)
    if veto is not None:
        return {"compatible":False,**veto}
    return geometry_overlap_diagnostics(left,right,config)


def geometry_overlap_diagnostics(left,right,config):
    """纯三维小方包含与逐轴尺寸比；帧内去重先于身份和共现判定。"""
    from services.memory.object_extent import resolve_object_extent
    a,b=resolve_object_extent(left),resolve_object_extent(right)
    if a is None or b is None:
        return {"compatible":False,"reason":"object_extent_missing"}
    if not frame_ids_semantically_equal(a["frame_id"],b["frame_id"]):
        return {"compatible":False,"reason":"extent_frame_mismatch"}
    def bounds(record,extent):
        q=extent["orientation"];x,y,z,w=(q[k] for k in "xyzw")
        r=np.array(((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),(2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),(2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y))))
        size=np.maximum(np.abs(r) @ np.array([extent["size_m"][k] for k in "xyz"]),1e-6)
        center=np.array([getattr(record.pose.position,k)+extent["center_offset_m"][k] for k in "xyz"])
        return center-size/2,center+size/2,size
    lo,hi,sa=bounds(left,a);ro,rh,sb=bounds(right,b)
    ratios=np.maximum(sa,sb)/np.minimum(sa,sb)
    result={"compatible":False,"size_ratio_xyz":ratios.tolist(),"max_size_ratio":float(ratios.max()),
            "size_ratio_limit":config.cross_label_max_size_ratio,"containment_limit":config.cross_label_min_overlap}
    if result["max_size_ratio"]>config.cross_label_max_size_ratio:
        return {**result,"reason":"cross_label_size_ratio"}
    ea=record_support_evidence(left,config);eb=record_support_evidence(right,config)
    if ea is not None and eb is not None and ea.voxel_keys and eb.voxel_keys:
        overlaps=support_overlap(ea,eb)
        measures=(len(ea.voxel_keys)//3,len(eb.voxel_keys)//3)
        basis="support_voxels"
    else:
        intersection=np.maximum(0.,np.minimum(hi,rh)-np.maximum(lo,ro))
        volume=float(np.prod(intersection))
        measures=(float(np.prod(sa)),float(np.prod(sb)))
        overlaps=(volume/measures[0],volume/measures[1])
        basis="measured_bounds_volume"
    smaller=0 if measures[0]<=measures[1] else 1
    containment=float(overlaps[smaller])
    compatible=containment>=config.cross_label_min_overlap
    return {**result,"compatible":compatible,"reason":"compatible" if compatible else "cross_label_containment",
        "overlap_basis":basis,"left_measure":measures[0],"right_measure":measures[1],
        "smaller_side":"left" if smaller==0 else "right","left_overlap":float(overlaps[0]),
        "right_overlap":float(overlaps[1]),"smaller_containment":containment}


def cross_label_geometry_compatible(left,right,config) -> bool:
    """标签不同允许小方被包含；尺寸比及已知共现分别否决。"""
    return cross_label_geometry_diagnostics(left,right,config)["compatible"]
