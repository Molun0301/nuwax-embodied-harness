from __future__ import annotations

import base64
from typing import Tuple

import cv2
import numpy as np

from contracts.image import ImageEncoding, ImageFrame
from core.errors import GatewayError


def decode_image_frame_to_bgr(image_frame: ImageFrame) -> np.ndarray:
    """把标准图像帧解码成 BGR 数组，供本地视觉模型复用。"""

    if image_frame.data is None:
        raise GatewayError("当前图像帧未携带内联图像数据，无法交给本地视觉后端。")

    if image_frame.encoding in (ImageEncoding.JPEG, ImageEncoding.PNG):
        buffer = np.frombuffer(image_frame.data, dtype=np.uint8)
        decoded = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if decoded is None:
            raise GatewayError("图像解码失败，无法生成 BGR 帧。")
        return decoded

    if image_frame.encoding == ImageEncoding.BGR8:
        return _reshape_dense_image(image_frame, channels=3)

    if image_frame.encoding == ImageEncoding.RGB8:
        rgb = _reshape_dense_image(image_frame, channels=3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    if image_frame.encoding == ImageEncoding.MONO8:
        mono = _reshape_dense_image(image_frame, channels=1)
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

    if image_frame.encoding == ImageEncoding.DEPTH16:
        depth = np.frombuffer(image_frame.data, dtype=np.uint16)
        expected = image_frame.width_px * image_frame.height_px
        if depth.size != expected:
            raise GatewayError("DEPTH16 图像尺寸与宽高不匹配。")
        depth = depth.reshape((image_frame.height_px, image_frame.width_px))
        normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
        gray = normalized.astype(np.uint8)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    raise GatewayError("暂不支持的图像编码：%s" % image_frame.encoding.value)


def image_frame_to_data_url(image_frame: ImageFrame, *, mime_type: str = "image/jpeg") -> str:
    """把标准图像帧转成适合云端多模态接口的 data URL。"""

    image_bytes, resolved_mime_type = encode_image_frame_bytes(image_frame, mime_type=mime_type)
    encoded = base64.b64encode(image_bytes).decode("ascii")
    return "data:%s;base64,%s" % (resolved_mime_type, encoded)


def encode_image_frame_bytes(
    image_frame: ImageFrame,
    *,
    mime_type: str = "image/jpeg",
) -> Tuple[bytes, str]:
    """把标准图像帧编码成可传输的压缩图像字节。"""

    if image_frame.data is not None:
        if image_frame.encoding == ImageEncoding.JPEG:
            return image_frame.data, "image/jpeg"
        if image_frame.encoding == ImageEncoding.PNG:
            return image_frame.data, "image/png"

    bgr = decode_image_frame_to_bgr(image_frame)
    suffix = ".png" if mime_type == "image/png" else ".jpg"
    ok, encoded = cv2.imencode(suffix, bgr)
    if not ok:
        raise GatewayError("图像重新编码失败。")
    resolved_mime_type = "image/png" if suffix == ".png" else "image/jpeg"
    return bytes(encoded.tobytes()), resolved_mime_type


def _reshape_dense_image(image_frame: ImageFrame, *, channels: int) -> np.ndarray:
    if image_frame.data is None:
        raise GatewayError("图像数据缺失。")
    buffer = np.frombuffer(image_frame.data, dtype=np.uint8)
    expected = image_frame.width_px * image_frame.height_px * channels
    if buffer.size != expected:
        raise GatewayError("图像尺寸与宽高不匹配。")
    if channels == 1:
        return buffer.reshape((image_frame.height_px, image_frame.width_px))
    return buffer.reshape((image_frame.height_px, image_frame.width_px, channels))
