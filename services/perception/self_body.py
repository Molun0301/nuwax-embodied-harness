"""机器人自体检测过滤：求机器人自身在当前画面中的区域，标记主要落在其中的检测。

区域来源有两种：按 URDF 与同刻关节角做正向运动学，把各连杆凸包投到相机画面；
以及每台相机手填的静态忽略多边形。被标记的检测保留在检测叠加层（灰显），
不进入投影、关联和记忆；落在区域内的点也不作为任何检测的几何支持。
"""
from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from datetime import datetime, timezone
import json
import logging
from pathlib import Path
from typing import Callable, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np
import cv2
from functools import cached_property

from services.localization.frame_tree_ops import resolve_frame_transform

from contracts.geometry import FrameTree
from contracts.image import CameraInfo, ImageFrame
from contracts.perception import Detection2D
from core.robot_kinematics import RobotKinematicModel, RobotKinematicsError, load_robot_kinematic_model

LOGGER = logging.getLogger(__name__)

SELF_BODY_KEY = "self_body"
SELF_BODY_COVERAGE_KEY = "self_body_coverage"
SELF_BODY_REGION_METADATA_KEY = "self_body_region"
SELF_BODY_STATUS_METADATA_KEY = "self_body_filter_status"

Polygon = Tuple[Tuple[float, float], ...]


def is_self_body(attributes: Optional[Mapping[str, object]]) -> bool:
    """检测属性是否已被判为机器人自身；只认显式布尔真值。"""

    return bool(attributes) and attributes.get(SELF_BODY_KEY) is True


@dataclass(frozen=True)
class JointStateSample:
    """一次关节状态读数；timestamp 为源采集时间。"""

    timestamp: datetime
    positions: Mapping[str, float]


@dataclass(frozen=True)
class SelfBodyRegion:
    """一帧图像中的机器人自体区域（像素多边形，已含膨胀），可写入感知上下文元数据。"""

    camera_id: str
    image_frame_id: str
    width_px: int
    height_px: int
    polygons_px: Tuple[Polygon, ...]
    sources: Tuple[str, ...] = ()
    joint_age_sec: Optional[float] = None
    ground_base_z_m: Optional[float] = None
    ground_status: str = "ground_height_unavailable"
    static_polygon_count: int = 0

    def to_metadata(self) -> Dict[str, object]:
        return {
            "camera_id": self.camera_id,
            "image_frame_id": self.image_frame_id,
            "width_px": int(self.width_px),
            "height_px": int(self.height_px),
            "polygons_px": [[[round(float(u), 2), round(float(v), 2)] for u, v in polygon] for polygon in self.polygons_px],
            "sources": list(self.sources),
            "ground_base_z_m": self.ground_base_z_m,
            "ground_status": self.ground_status,
            "static_polygon_count": self.static_polygon_count,
            "joint_age_sec": None if self.joint_age_sec is None else round(float(self.joint_age_sec), 4),
        }

    @classmethod
    def from_metadata(cls, value: object) -> Optional["SelfBodyRegion"]:
        if not isinstance(value, Mapping):
            return None
        try:
            polygons = tuple(
                tuple((float(point[0]), float(point[1])) for point in polygon)
                for polygon in value.get("polygons_px") or ()
            )
            joint_age = value.get("joint_age_sec")
            return cls(
                camera_id=str(value["camera_id"]),
                image_frame_id=str(value["image_frame_id"]),
                width_px=int(value["width_px"]),
                height_px=int(value["height_px"]),
                polygons_px=polygons,
                sources=tuple(str(item) for item in value.get("sources") or ()),
                joint_age_sec=None if joint_age is None else float(joint_age),
                ground_base_z_m=None if value.get("ground_base_z_m") is None else float(value["ground_base_z_m"]),
                ground_status=str(value.get("ground_status") or "ground_height_unavailable"),
                static_polygon_count=int(value.get("static_polygon_count",len(polygons) if value.get("sources")==["static"] else 0)),
            )
        except (KeyError, TypeError, ValueError, IndexError):
            return None

    def contains_pixels(self, pixels: np.ndarray) -> np.ndarray:
        """返回 (N,) 布尔数组：各像素坐标是否落在自体区域内。"""
        pixels = np.asarray(pixels)
        result = np.zeros(len(pixels), dtype=bool)
        valid = np.isfinite(pixels).all(axis=1) & (pixels[:,0] >= 0) & (pixels[:,0] < self.width_px) & (pixels[:,1] >= 0) & (pixels[:,1] < self.height_px)
        indices = (pixels[valid] / self._raster_scale).astype(np.int64)
        result[valid] = self._mask[indices[:,1], indices[:,0]] != 0
        return result

    @cached_property
    def _raster_scale(self):
        return max(1, int(np.ceil(max(self.width_px, self.height_px) / 640.0)))

    @cached_property
    def _link_mask(self):
        result = np.zeros((int(np.ceil(self.height_px/self._raster_scale)),
                           int(np.ceil(self.width_px/self._raster_scale))), dtype=np.uint8)
        # 分别填充再求并集，防止相交多边形被奇偶填充规则误挖空。
        for polygon in self.polygons_px[self.static_polygon_count:]:
            _fill_polygon(result, polygon, self._raster_scale)
        return result

    @cached_property
    def _component_hulls(self):
        # 连通性在膨胀后的像素并集上判断；先裁到画面，避免画外连接误包住环境。
        contours,_ = cv2.findContours(self._link_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        return tuple(cv2.convexHull(contour).reshape(-1,2)*self._raster_scale
                     for contour in contours if len(contour)>=3)

    @cached_property
    def _mask(self):
        result=np.zeros_like(self._link_mask)
        # 手工忽略区保留用户画出的轮廓；连杆部件才填指缝、参与外接框判定。
        for polygon in self.polygons_px[:self.static_polygon_count]:
            _fill_polygon(result,polygon,self._raster_scale)
        for hull in self._component_hulls:
            _fill_polygon(result,hull,self._raster_scale)
        return result

    @cached_property
    def _candidate_boxes(self):
        hulls=self._component_hulls
        groups=[(i,) for i in range(len(hulls))]+list(combinations(range(len(hulls)),2))
        if len(hulls)>2:
            groups.append(tuple(range(len(hulls))))
        boxes=[]
        for group in groups:
            points=np.concatenate([hulls[i] for i in group])
            low=np.maximum(points.min(axis=0),0.)
            high=np.minimum(points.max(axis=0)+self._raster_scale,(self.width_px,self.height_px))
            boxes.append((low,high))
        return tuple(boxes)

    def bbox_iou_of_detection(self,detection):
        """检测框与单部件、两部件及全部部件的画面内外接框的最大交并比。"""
        b=detection.bbox
        low=np.maximum((b.x_px,b.y_px),0.)
        high=np.minimum((b.x_px+b.width_px,b.y_px+b.height_px),(self.width_px,self.height_px))
        area=float(np.prod(np.maximum(high-low,0.)))
        best=0.
        for a,z in self._candidate_boxes:
            intersection=float(np.prod(np.maximum(np.minimum(high,z)-np.maximum(low,a),0.)))
            union=area+float(np.prod(z-a))-intersection
            if union>0:
                best=max(best,intersection/union)
        return best

    def coverage_of_detection(self, detection: Detection2D) -> float:
        """检测掩码或框与区域并集的面积比；所有查询复用同一张有界栅格。"""
        polygon = detection.segmentation.polygon_px if detection.segmentation is not None else ()
        if not polygon:
            polygon = detection.attributes.get("mask_polygon_px") or detection.attributes.get("segmentation_polygon_px") or ()
        if not polygon:
            b = detection.bbox
            x, y, w, h = b.x_px, b.y_px, b.width_px, b.height_px
            polygon = ((x,y),(x+w,y),(x+w,y+h),(x,y+h))
        detection_mask = np.zeros_like(self._mask)
        _fill_polygon(detection_mask, polygon, self._raster_scale)
        area = np.count_nonzero(detection_mask)
        return float(np.count_nonzero(detection_mask & self._mask)/area) if area else 0.0


def _fill_polygon(mask, polygon, scale):
    if len(polygon) >= 3:
        points = np.asarray(polygon, dtype=np.float64) / scale
        # 近平面可能产生很大的投影值；限制转换范围，图像边界由填充器裁剪。
        points = np.clip(np.rint(points), -1000000, 1000000).astype(np.int32)
        cv2.fillPoly(mask, [points], 1)


@dataclass(frozen=True)
class SelfBodyFilterResult:
    detections_2d: Tuple[Detection2D, ...]
    region: Optional[SelfBodyRegion]
    metadata: Dict[str, object]


class SelfBodyFilter:
    """自体过滤接口：只标记检测，不删除；下游按 is_self_body 跳过。"""

    def apply(
        self,
        detections_2d: Tuple[Detection2D, ...],
        *,
        image_frame: ImageFrame,
        camera_info: Optional[CameraInfo],
        frame_tree: Optional[FrameTree],
    ) -> SelfBodyFilterResult:
        raise NotImplementedError

    def status(self) -> Dict[str, object]:
        raise NotImplementedError


class DisabledSelfBodyFilter(SelfBodyFilter):
    """未启用或无法装配时的明确状态；检测原样通过并报告原因。"""

    def __init__(self, reason: str) -> None:
        self._reason = str(reason or "disabled")

    def apply(self, detections_2d, *, image_frame, camera_info, frame_tree) -> SelfBodyFilterResult:
        return SelfBodyFilterResult(
            detections_2d=tuple(detections_2d), region=None,
            metadata={SELF_BODY_STATUS_METADATA_KEY: self._reason},
        )

    def status(self) -> Dict[str, object]:
        return {"enabled": False, "status": self._reason}


class KinematicSelfBodyFilter(SelfBodyFilter):
    """按运动学模型与静态忽略区域计算自体区域。

    model 为 None 时只使用静态区域；两者都没有时 apply 报告 model_error。
    """

    def __init__(
        self,
        *,
        model: Optional[RobotKinematicModel],
        model_error: Optional[str],
        joint_state_source: Callable[[], Optional[JointStateSample]],
        coverage_threshold: float,
        dilation_px: float,
        max_joint_age_sec: float,
        static_regions: Mapping[str, Tuple[Polygon, ...]],
        bbox_iou_threshold: float = 0.5,
        static_ground_z_m: Optional[float] = None,
    ) -> None:
        self._bbox_iou_threshold = float(bbox_iou_threshold)
        self._static_ground_z_m = static_ground_z_m
        self._last_status = "not_evaluated"
        self._excluded_camera_links = ()
        self._model = model
        self._model_error = model_error
        self._joint_state_source = joint_state_source
        self._coverage_threshold = float(coverage_threshold)
        self._dilation_px = float(dilation_px)
        self._max_joint_age_sec = float(max_joint_age_sec)
        self._static_regions = {str(key): tuple(value) for key, value in static_regions.items()}

    def apply(self, detections_2d, *, image_frame, camera_info, frame_tree) -> SelfBodyFilterResult:
        detections_2d = tuple(detections_2d)
        region, status = self.compute_region(image_frame=image_frame, camera_info=camera_info, frame_tree=frame_tree)
        self._last_status = status
        metadata: Dict[str, object] = {SELF_BODY_STATUS_METADATA_KEY: status}
        metadata["self_body_excluded_camera_links"]=list(self._excluded_camera_links)
        if region is None:
            return SelfBodyFilterResult(detections_2d=detections_2d, region=None, metadata=metadata)
        metadata[SELF_BODY_REGION_METADATA_KEY] = region.to_metadata()
        if not region.polygons_px:
            return SelfBodyFilterResult(detections_2d=detections_2d, region=region, metadata=metadata)
        annotated = []
        flagged = 0
        for detection in detections_2d:
            coverage = region.coverage_of_detection(detection)
            bbox_iou = region.bbox_iou_of_detection(detection)
            self_body = bool(coverage >= self._coverage_threshold or bbox_iou >= self._bbox_iou_threshold)
            flagged += int(self_body)
            annotated.append(detection.model_copy(update={"attributes": {
                **dict(detection.attributes or {}),
                SELF_BODY_KEY: self_body,
                SELF_BODY_COVERAGE_KEY: round(float(coverage), 4),
                "self_body_bbox_iou": round(float(bbox_iou),4),
            }}, deep=False))
        metadata["self_body_detection_count"] = flagged
        return SelfBodyFilterResult(detections_2d=tuple(annotated), region=region, metadata=metadata)

    def compute_region(
        self,
        *,
        image_frame: ImageFrame,
        camera_info: Optional[CameraInfo],
        frame_tree: Optional[FrameTree],
    ) -> Tuple[Optional[SelfBodyRegion], str]:
        """返回 (区域, 状态)。状态为 ok 或具体不可用原因（如 joint_state_missing、
        joint_state_stale、camera_extrinsic_unavailable、model_unavailable:...）。
        运动学不可用但有该相机静态区域时，仍返回只含静态区域的结果并在状态中说明。
        """
        from services.perception.geometry import DetectionGeometryAugmentor
        width, height = image_frame.width_px, image_frame.height_px
        polygons = [tuple((x*width,y*height) for x,y in polygon)
                    for polygon in self._static_regions.get(image_frame.camera_id, ())]
        sources = ["static"] if polygons else []
        static_count=len(polygons)
        age, status = None, "ok"
        excluded=[]
        self._excluded_camera_links=()
        ground_z=self._static_ground_z_m
        ground_status="static_robot_geometry" if ground_z is not None else "ground_height_unavailable"
        transforms=None
        joint_status="joint_state_missing"
        if self._model is not None and self._model.hulls:
            sample=self._joint_state_source()
            if sample is not None:
                image_ts=image_frame.timestamp
                joint_ts=sample.timestamp
                image_ts=image_ts.replace(tzinfo=timezone.utc) if image_ts.tzinfo is None else image_ts
                joint_ts=joint_ts.replace(tzinfo=timezone.utc) if joint_ts.tzinfo is None else joint_ts
                age=abs((image_ts-joint_ts).total_seconds())
                joint_status="joint_state_stale" if age>self._max_joint_age_sec else "ok"
                if joint_status=="ok":
                    try:
                        transforms=self._model.link_transforms(sample.positions)
                    except RobotKinematicsError as exc:
                        joint_status=exc.code
                    else:
                        ground_z=min(float((h.points_local @ transforms[h.link_name][:3,:3].T
                            + transforms[h.link_name][:3,3])[:,2].min()) for h in self._model.hulls)
                        ground_status="kinematic_foot_minimum"
            if transforms is None:
                ground_status += ":"+joint_status
        if self._model is None:
            status="model_unavailable:"+str(self._model_error)
        elif not self._model.hulls:
            status="model_unavailable:collision_geometry_missing"
        elif camera_info is None:
            status="camera_intrinsics_unavailable"
        else:
            status=DetectionGeometryAugmentor._camera_geometry_error(image_frame,camera_info) or joint_status
            if status=="ok":
                extrinsic=resolve_frame_transform(frame_tree,source_frame_id="base",target_frame_id=camera_info.frame_id)
                if extrinsic is None:
                    status="camera_extrinsic_unavailable"
                else:
                    transform=extrinsic.matrix
                    for body_hull in self._model.hulls:
                        camera_from_link=transform @ transforms[body_hull.link_name]
                        rotation,translation=camera_from_link[:3,:3],camera_from_link[:3,3]
                        planes=body_hull.planes_local
                        if planes is not None and len(planes):
                            camera_origin_local=-rotation.T @ translation
                            if np.all(planes[:,:3] @ camera_origin_local <= planes[:,3]+1e-9):
                                excluded.append(body_hull.link_name)
                                continue
                        camera=_clip_near_plane(body_hull.points_local @ rotation.T + translation,0.01)
                        if len(camera)<3:
                            continue
                        pixels=DetectionGeometryAugmentor._project_camera_points(camera,camera_info)
                        pixels=pixels[np.isfinite(pixels).all(axis=1)]
                        if len(pixels)<3:
                            continue
                        hull=cv2.convexHull(pixels.astype(np.float32)).reshape(-1,2)
                        if self._dilation_px>0:
                            angles=np.arange(8)*np.pi/4
                            offsets=np.column_stack((np.cos(angles),np.sin(angles)))*self._dilation_px/np.cos(np.pi/8)
                            hull=cv2.convexHull((hull[:,None,:]+offsets).reshape(-1,2).astype(np.float32)).reshape(-1,2)
                        if hull[:,0].max()<0 or hull[:,1].max()<0 or hull[:,0].min()>width or hull[:,1].min()>height:
                            continue
                        polygons.append(tuple((float(x),float(y)) for x,y in hull))
                    sources.append("kinematics")
                    self._excluded_camera_links=tuple(excluded)
        if status != "ok" and polygons:
            status = "static_only:" + status
        self._last_status = status
        if not sources and ground_z is None:
            return None, status
        return SelfBodyRegion(image_frame.camera_id, image_frame.frame_id, width, height,
                              tuple(polygons), tuple(sources), age,ground_z,ground_status,static_count), status

    def status(self) -> Dict[str, object]:
        return {
            "enabled": True,
            "status": self._last_status,
            "excluded_camera_enclosing_links": list(self._excluded_camera_links),
            "kinematic_model": self._model.source_path if self._model is not None else None,
            "model_error": self._model_error,
            "static_region_cameras": sorted(self._static_regions),
            "coverage_threshold": self._coverage_threshold,
            "bbox_iou_threshold": self._bbox_iou_threshold,
            "dilation_px": self._dilation_px,
            "max_joint_age_sec": self._max_joint_age_sec,
        }


def parse_static_regions(raw: str) -> Dict[str, Tuple[Polygon, ...]]:
    """解析 {"相机标识": [[[x, y], ...], ...]}，坐标为 0—1 归一化值；非法内容抛 ValueError。"""
    if not str(raw or "").strip():
        return {}
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("静态区域相机标识重复")
            result[key] = value
        return result
    value = json.loads(raw, object_pairs_hook=unique)
    if not isinstance(value, dict):
        raise ValueError("静态区域必须为相机标识到多边形列表的对象")
    result = {}
    for camera, polygons in value.items():
        if not isinstance(camera, str) or not camera.strip() or camera != camera.strip() or any(ord(c)<32 for c in camera):
            raise ValueError("静态区域相机标识无效")
        if not isinstance(polygons, list):
            raise ValueError("相机区域必须为多边形列表")
        parsed = []
        for polygon in polygons:
            if not isinstance(polygon, list) or len(polygon) < 3:
                raise ValueError("静态区域多边形至少需要三个点")
            for point in polygon:
                if (not isinstance(point,list) or len(point)!=2 or
                    any(type(v) not in (int,float) or not np.isfinite(v) or not 0<=v<=1 for v in point)):
                    raise ValueError("静态区域坐标必须是0至1的有限数值对")
            array = np.asarray(polygon, dtype=np.float64)
            area = abs(np.dot(array[:,0], np.roll(array[:,1],-1))-np.dot(array[:,1], np.roll(array[:,0],-1))) / 2
            if area <= 1e-12 or len(set(map(tuple,polygon))) != len(polygon):
                raise ValueError("静态区域多边形退化或包含重复点")
            # 任意非相邻边相交均拒绝，避免自交掩码产生含糊的覆盖语义。
            def cross(a,b,c):
                return float((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))
            n = len(array)
            for i in range(n):
                a,b = array[i],array[(i+1)%n]
                for j in range(i+2,n):
                    if (j+1)%n == i:
                        continue
                    c,d = array[j],array[(j+1)%n]
                    if cross(a,b,c)*cross(a,b,d)<=0 and cross(c,d,a)*cross(c,d,b)<=0 and (np.maximum(np.minimum(a,b),np.minimum(c,d)) <= np.minimum(np.maximum(a,b),np.maximum(c,d))).all():
                        raise ValueError("静态区域多边形存在自交")
            parsed.append(tuple(map(tuple,array.tolist())))
        result[camera] = tuple(parsed)
    return result


def _clip_near_plane(points, near):
    front = points[:,2] >= near
    if front.all():
        return points
    if not front.any():
        return np.empty((0,3))
    a, b = points[front], points[~front]
    ratio = (near-a[:,None,2])/(b[None,:,2]-a[:,None,2])
    intersections = a[:,None,:] + ratio[:,:,None]*(b[None,:,:]-a[:,None,:])
    return np.concatenate((a,intersections.reshape(-1,3)))


def robot_joint_state_source(provider_owner) -> Callable[[], Optional[JointStateSample]]:
    """从机器人提供器读取最近一次关节状态；读不到时返回 None。"""

    def read() -> Optional[JointStateSample]:
        providers = getattr(provider_owner, "providers", None)
        getter = getattr(providers, "get_robot_state", None)
        if getter is None:
            return None
        try:
            state = getter()
        except Exception:  # 提供器暂不可用属于输入缺失，由过滤状态报告。
            return None
        joints = getattr(state, "joints", None) or ()
        if not joints:
            return None
        timestamp = getattr(state, "timestamp", None)
        if not isinstance(timestamp, datetime):
            return None
        if timestamp.tzinfo is None:
            timestamp = timestamp.replace(tzinfo=timezone.utc)
        return JointStateSample(
            timestamp=timestamp,
            positions={str(joint.name): float(joint.position_rad) for joint in joints},
        )

    return read


def build_self_body_filter(
    config,
    *,
    robot_geometry,
    base_dir,
    joint_state_source: Callable[[], Optional[JointStateSample]],
    robot_name: str = "",
) -> SelfBodyFilter:
    """按感知自体过滤设置与机器人模型设置装配；任何装配失败都转为带原因的状态。

    config 为 core.settings.PerceptionSelfFilterConfig，robot_geometry 为
    core.robot_geometry.RobotGeometryConfig。
    """

    if not getattr(config, "enabled", False):
        return DisabledSelfBodyFilter("disabled_by_config")
    try:
        static_regions = parse_static_regions(getattr(config, "static_regions_json", ""))
    except ValueError as exc:
        LOGGER.warning("自体过滤静态区域设置无效：%s", exc)
        return DisabledSelfBodyFilter("static_regions_invalid")
    model = None
    model_error = None
    from core.robot_geometry import resolve_robot_geometry, resolve_robot_model_config
    model_config = resolve_robot_model_config(robot_geometry, robot_name=robot_name, base_dir=base_dir)
    model_path = model_config.model_path
    if not model_path:
        model_error = "model_path_not_configured"
    else:
        path = Path(model_path).expanduser()
        if not path.is_absolute():
            path = Path(base_dir).expanduser() / path
        try:
            model = load_robot_kinematic_model(path, base_link=model_config.model_base_link)
        except RobotKinematicsError as exc:
            model_error = exc.code
            LOGGER.warning("自体过滤无法加载机器人模型 %s：%s", path, exc)
    static_ground=resolve_robot_geometry(robot_geometry,robot_name=robot_name,base_dir=base_dir).z_min_m
    return KinematicSelfBodyFilter(
        model=model,
        model_error=model_error,
        joint_state_source=joint_state_source,
        coverage_threshold=config.coverage_threshold,
        dilation_px=config.dilation_px,
        max_joint_age_sec=config.max_joint_age_sec,
        static_regions=static_regions,
        bbox_iou_threshold=config.bbox_iou_threshold,
        static_ground_z_m=static_ground,
    )


def region_from_detection_metadata(metadata: Optional[Mapping[str, object]]) -> Optional[SelfBodyRegion]:
    """从感知上下文的 detection_metadata 取回同一帧的自体区域。"""

    if not isinstance(metadata, Mapping):
        return None
    return SelfBodyRegion.from_metadata(metadata.get(SELF_BODY_REGION_METADATA_KEY))


def detections_excluding_self(detections: Sequence[Detection2D]) -> Tuple[Detection2D, ...]:
    return tuple(item for item in detections if not is_self_body(item.attributes))
