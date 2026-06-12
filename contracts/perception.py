from __future__ import annotations

from contracts.compat import StrEnum

from pydantic import Field, field_validator

from contracts.base import MetadataDict, TimestampedContract
from contracts.geometry import Pose
from contracts.naming import validate_frame_id, validate_observation_id
from typing import List, Optional, Tuple


class BoundingBox2D(TimestampedContract):
    """二维检测框。"""

    x_px: float = Field(ge=0.0, description="左上角 X。")
    y_px: float = Field(ge=0.0, description="左上角 Y。")
    width_px: float = Field(gt=0.0, description="宽度。")
    height_px: float = Field(gt=0.0, description="高度。")


class SegmentationQuality(TimestampedContract):
    """分割质量指标。"""

    score: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="分割置信度。")
    predicted_iou: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="预测 IoU。")
    stability_score: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="稳定性分数。")
    depth_coverage: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="深度覆盖率。")
    map_projection_coverage: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="地图投影覆盖率。")
    reprojection_error: Optional[float] = Field(default=None, ge=0.0, description="重投影误差。")


class SegmentationMask2D(TimestampedContract):
    """二维分割结果。"""

    polygon_px: Tuple[Tuple[float, float], ...] = Field(default_factory=tuple, description="像素坐标多边形。")
    centroid_uv: Optional[Tuple[float, float]] = Field(default=None, description="像素中心点。")
    bbox_xyxy: Optional[Tuple[float, float, float, float]] = Field(default=None, description="分割外接框。")
    area_px: Optional[float] = Field(default=None, ge=0.0, description="分割面积。")
    area_ratio: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="分割面积占比。")
    contour: MetadataDict = Field(default_factory=dict, description="轮廓附加数据。")
    crop_box: Optional[Tuple[float, float, float, float]] = Field(default=None, description="裁剪框。")
    quality: SegmentationQuality = Field(default_factory=SegmentationQuality, description="分割质量。")
    frame_id: Optional[str] = Field(default=None, description="来源坐标系。")
    camera_id: Optional[str] = Field(default=None, description="来源相机。")
    source: Optional[str] = Field(default=None, description="分割来源。")
    truncation_flags: Tuple[str, ...] = Field(default_factory=tuple, description="截断标记。")
    occlusion_flags: Tuple[str, ...] = Field(default_factory=tuple, description="遮挡标记。")
    attributes: MetadataDict = Field(default_factory=dict, description="附加属性。")


class Detection2D(TimestampedContract):
    """二维检测结果。"""

    label: str = Field(description="检测标签。")
    score: float = Field(ge=0.0, le=1.0, description="置信度。")
    bbox: BoundingBox2D = Field(description="二维检测框。")
    camera_id: Optional[str] = Field(default=None, description="来源相机。")
    track_id: Optional[str] = Field(default=None, description="关联轨迹。")
    segmentation: Optional[SegmentationMask2D] = Field(default=None, description="结构化分割结果。")
    attributes: MetadataDict = Field(default_factory=dict, description="附加属性。")


class Detection3D(TimestampedContract):
    """三维检测结果。"""

    label: str = Field(description="检测标签。")
    score: float = Field(ge=0.0, le=1.0, description="置信度。")
    pose: Pose = Field(description="三维目标位姿。")
    size_x_m: Optional[float] = Field(default=None, gt=0.0, description="X 尺寸。")
    size_y_m: Optional[float] = Field(default=None, gt=0.0, description="Y 尺寸。")
    size_z_m: Optional[float] = Field(default=None, gt=0.0, description="Z 尺寸。")
    attributes: MetadataDict = Field(default_factory=dict, description="附加属性。")


class TrackState(StrEnum):
    """轨迹状态。"""

    TENTATIVE = "tentative"
    TRACKED = "tracked"
    LOST = "lost"
    REMOVED = "removed"


class Track(TimestampedContract):
    """目标轨迹。"""

    track_id: str = Field(description="轨迹标识。")
    label: str = Field(description="目标标签。")
    state: TrackState = Field(default=TrackState.TENTATIVE, description="轨迹状态。")
    score: float = Field(ge=0.0, le=1.0, description="置信度。")
    pose: Optional[Pose] = Field(default=None, description="三维位姿。")
    bbox: Optional[BoundingBox2D] = Field(default=None, description="二维检测框。")
    attributes: MetadataDict = Field(default_factory=dict, description="附加属性。")


class Observation(TimestampedContract):
    """观察结果聚合。"""

    observation_id: str = Field(description="观察标识。")
    frame_id: str = Field(description="观察所属坐标系。")
    summary: Optional[str] = Field(default=None, description="面向上层的中文摘要。")
    detections_2d: List[Detection2D] = Field(default_factory=list, description="二维检测结果。")
    detections_3d: List[Detection3D] = Field(default_factory=list, description="三维检测结果。")
    tracks: List[Track] = Field(default_factory=list, description="轨迹列表。")
    artifact_ids: List[str] = Field(default_factory=list, description="关联制品标识。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("frame_id")
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)

    @field_validator("observation_id")
    @classmethod
    def _validate_observation_id(cls, value: str) -> str:
        return validate_observation_id(value)
