from __future__ import annotations

from contracts.compat import StrEnum

from pydantic import Field, field_validator, model_validator

from contracts.base import ContractModel, MetadataDict, TimestampedContract
from contracts.naming import validate_frame_id
from typing import List, Optional


class ImageEncoding(StrEnum):
    """图像编码格式。"""

    JPEG = "jpeg"
    PNG = "png"
    RGB8 = "rgb8"
    BGR8 = "bgr8"
    MONO8 = "mono8"
    DEPTH16 = "depth16"


class CameraInfo(TimestampedContract):
    """相机内参与基础属性。"""

    camera_id: str = Field(description="相机标识。")
    frame_id: str = Field(description="相机坐标系。")
    width_px: int = Field(ge=1, description="图像宽度。")
    height_px: int = Field(ge=1, description="图像高度。")
    fx: float = Field(description="焦距 X。")
    fy: float = Field(description="焦距 Y。")
    cx: float = Field(description="主点 X。")
    cy: float = Field(description="主点 Y。")
    distortion_model: Optional[str] = Field(default=None, description="畸变模型。")
    distortion_coefficients: List[float] = Field(default_factory=list, description="畸变参数。")

    @field_validator("frame_id")
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)


class ImageFrame(TimestampedContract):
    """图像帧。"""

    camera_id: str = Field(description="相机标识。")
    frame_id: str = Field(description="图像坐标系。")
    width_px: int = Field(ge=1, description="图像宽度。")
    height_px: int = Field(ge=1, description="图像高度。")
    encoding: ImageEncoding = Field(description="图像编码。")
    data: Optional[bytes] = Field(default=None, description="内联图像数据。")
    uri: Optional[str] = Field(default=None, description="外部图像地址。")
    artifact_id: Optional[str] = Field(default=None, description="图像制品标识。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("frame_id")
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)

    @model_validator(mode="after")
    def _ensure_payload(self) -> "ImageFrame":
        if self.data is None and self.uri is None and self.artifact_id is None:
            raise ValueError("ImageFrame 至少需要提供 data、uri 或 artifact_id 之一。")
        return self
