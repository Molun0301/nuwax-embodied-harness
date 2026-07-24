"""OpenCV-backed AprilTag detection for the standalone mapping capability."""

from __future__ import annotations

import base64
import binascii
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

import cv2
import numpy as np


_PNG_MAGIC = b"\x89PNG\r\n\x1a\n"
_JPEG_MAGIC = b"\xff\xd8"
_MAX_IMAGE_PIXELS = 100_000_000
_MAX_ENCODED_BYTES = 32 * 1024 * 1024


class MarkerRequestError(ValueError):
    """The marker request is malformed or cannot be decoded safely."""


@dataclass(frozen=True)
class CameraCalibration:
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    tag_size_m: float


def _finite_float(value: object, field: str) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise MarkerRequestError("%s must be a finite number" % field)
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise MarkerRequestError("%s must be a finite number" % field) from exc
    if not np.isfinite(result):
        raise MarkerRequestError("%s must be a finite number" % field)
    return result


def _decode_bytes(value: object) -> bytes:
    if isinstance(value, str):
        if len(value) > 4 * ((_MAX_ENCODED_BYTES + 2) // 3):
            raise MarkerRequestError("encoded image exceeds the byte limit")
        try:
            data = base64.b64decode(value, validate=True)
        except (binascii.Error, ValueError) as exc:
            raise MarkerRequestError("image data must be valid base64") from exc
    elif isinstance(value, list) and value:
        if len(value) > _MAX_ENCODED_BYTES:
            raise MarkerRequestError("encoded image exceeds the byte limit")
        if any(type(item) is not int or not 0 <= item <= 255 for item in value):
            raise MarkerRequestError("image data contains a non-byte value")
        data = bytes(value)
    else:
        raise MarkerRequestError("image data must be non-empty base64 or a JSON byte array")
    if len(data) > _MAX_ENCODED_BYTES:
        raise MarkerRequestError("encoded image exceeds the byte limit")
    return data


def _positive_dimension(value: object, field: str) -> int:
    if type(value) is not int or value <= 0:
        raise MarkerRequestError("%s must be a positive integer" % field)
    return value


def _png_dimensions(data: bytes) -> tuple[int, int]:
    if len(data) < 24 or not data.startswith(_PNG_MAGIC):
        raise MarkerRequestError("PNG image data has a malformed header")
    if int.from_bytes(data[8:12], "big") != 13 or data[12:16] != b"IHDR":
        raise MarkerRequestError("PNG image data has a malformed IHDR")
    width = int.from_bytes(data[16:20], "big")
    height = int.from_bytes(data[20:24], "big")
    if width == 0 or height == 0:
        raise MarkerRequestError("PNG image dimensions must be positive")
    return width, height


def _jpeg_dimensions(data: bytes) -> tuple[int, int]:
    if len(data) < 4 or not data.startswith(_JPEG_MAGIC):
        raise MarkerRequestError("JPEG image data has a malformed header")
    position = 2
    sof_markers = set(range(0xC0, 0xD0)) - {0xC4, 0xC8, 0xCC}
    while position < len(data):
        if data[position] != 0xFF:
            raise MarkerRequestError("JPEG image data has a malformed marker stream")
        while position < len(data) and data[position] == 0xFF:
            position += 1
        if position >= len(data):
            break
        marker = data[position]
        position += 1
        if marker == 0xDA:
            break
        if marker in {0x01, 0xD8, 0xD9} or 0xD0 <= marker <= 0xD7:
            continue
        if position + 2 > len(data):
            break
        segment_length = int.from_bytes(data[position : position + 2], "big")
        if segment_length < 2 or position + segment_length > len(data):
            raise MarkerRequestError("JPEG image data has a malformed segment")
        if marker in sof_markers:
            if segment_length < 7:
                raise MarkerRequestError("JPEG image data has a malformed SOF segment")
            height = int.from_bytes(data[position + 3 : position + 5], "big")
            width = int.from_bytes(data[position + 5 : position + 7], "big")
            if width == 0 or height == 0:
                raise MarkerRequestError("JPEG image dimensions must be positive")
            return width, height
        position += segment_length
    raise MarkerRequestError("JPEG image data does not contain a valid SOF segment")


def _load_encoded_image(data: bytes, encoding: str) -> np.ndarray:
    if len(data) > _MAX_ENCODED_BYTES:
        raise MarkerRequestError("encoded image exceeds the byte limit")
    if encoding == "png":
        width, height = _png_dimensions(data)
    elif encoding in ("jpeg", "jpg"):
        width, height = _jpeg_dimensions(data)
    else:
        raise MarkerRequestError("encoded image must be PNG or JPEG")
    if width * height > _MAX_IMAGE_PIXELS:
        raise MarkerRequestError("encoded image exceeds the pixel limit")
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.size == 0:
        raise MarkerRequestError("encoded image could not be decoded")
    if image.shape[:2] != (height, width):
        raise MarkerRequestError("decoded image dimensions do not match its header")
    return image


def load_request_image(payload: object) -> np.ndarray:
    """Decode exactly one image path or encoded PNG/JPEG/RGB8 request."""
    if not isinstance(payload, Mapping):
        raise MarkerRequestError("request must be a JSON object")
    image_path = payload.get("image_path")
    image_spec = payload.get("image")
    if (image_path is None) == (image_spec is None):
        raise MarkerRequestError("request must provide exactly one of image_path or image")

    if image_path is not None:
        if not isinstance(image_path, str) or not image_path.strip():
            raise MarkerRequestError("image_path must be a non-empty string")
        path = Path(image_path)
        if not path.is_file():
            raise MarkerRequestError("image_path does not reference a regular file")
        try:
            size = path.stat().st_size
        except OSError as exc:
            raise MarkerRequestError("image_path could not be inspected") from exc
        if size > _MAX_ENCODED_BYTES:
            raise MarkerRequestError("image_path exceeds the byte limit")
        try:
            data = path.read_bytes()
        except OSError as exc:
            raise MarkerRequestError("image_path could not be read") from exc
        if data.startswith(_PNG_MAGIC):
            encoding = "png"
        elif data.startswith(_JPEG_MAGIC):
            encoding = "jpeg"
        else:
            raise MarkerRequestError("image_path must contain a PNG or JPEG image")
        return _load_encoded_image(data, encoding)

    if not isinstance(image_spec, Mapping):
        raise MarkerRequestError("image must be a JSON object")
    encoding = str(image_spec.get("encoding") or "").strip().lower()
    data = _decode_bytes(image_spec.get("data"))
    if encoding in ("png", "jpeg", "jpg"):
        return _load_encoded_image(data, encoding)
    if encoding != "rgb8":
        raise MarkerRequestError("image encoding must be png, jpeg, or rgb8")
    width = _positive_dimension(image_spec.get("width"), "image.width")
    height = _positive_dimension(image_spec.get("height"), "image.height")
    if width * height > _MAX_IMAGE_PIXELS:
        raise MarkerRequestError("RGB8 image exceeds the pixel limit")
    expected = width * height * 3
    if len(data) != expected:
        raise MarkerRequestError("RGB8 image data length must equal width * height * 3")
    rgb = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def parse_calibration(payload: object) -> Optional[CameraCalibration]:
    if not isinstance(payload, Mapping):
        raise MarkerRequestError("request must be a JSON object")
    supplied = [
        payload.get("camera_matrix") is not None,
        payload.get("distortion_coefficients") is not None,
        payload.get("tag_size_m") is not None,
    ]
    if not any(supplied):
        return None
    if not all(supplied):
        raise MarkerRequestError("camera_matrix, distortion_coefficients, and tag_size_m must be supplied together")
    for field in ("camera_matrix", "distortion_coefficients"):
        try:
            values = np.asarray(payload[field], dtype=object)
        except (TypeError, ValueError) as exc:
            raise MarkerRequestError("camera calibration must contain numeric arrays") from exc
        if any(isinstance(value, (bool, np.bool_)) for value in values.flat):
            raise MarkerRequestError("camera calibration must not contain boolean values")
    try:
        camera_matrix = np.asarray(payload["camera_matrix"], dtype=np.float64)
        distortion = np.asarray(payload["distortion_coefficients"], dtype=np.float64).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise MarkerRequestError("camera calibration must contain numeric arrays") from exc
    if camera_matrix.shape != (3, 3) or not np.all(np.isfinite(camera_matrix)):
        raise MarkerRequestError("camera_matrix must be a finite 3x3 array")
    if distortion.size not in (4, 5, 8, 12, 14) or not np.all(np.isfinite(distortion)):
        raise MarkerRequestError("distortion_coefficients must contain 4, 5, 8, 12, or 14 values")
    tag_size_m = _finite_float(payload["tag_size_m"], "tag_size_m")
    if tag_size_m <= 0.0:
        raise MarkerRequestError("tag_size_m must be positive")
    return CameraCalibration(camera_matrix, distortion, tag_size_m)


class AprilTagDetector:
    """Detect `DICT_APRILTAG_36h11` markers and optionally estimate pose."""

    dictionary_name = "DICT_APRILTAG_36h11"

    def __init__(self) -> None:
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        parameters = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    @staticmethod
    def _estimate_pose(corners: np.ndarray, calibration: CameraCalibration) -> Dict[str, List[float]]:
        half = calibration.tag_size_m / 2.0
        object_points = np.asarray(
            [(-half, half, 0.0), (half, half, 0.0), (half, -half, 0.0), (-half, -half, 0.0)],
            dtype=np.float64,
        )
        success, rotation, translation = cv2.solvePnP(
            object_points,
            corners.astype(np.float64),
            calibration.camera_matrix,
            calibration.distortion_coefficients,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success or not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
            raise MarkerRequestError("pose estimation failed")
        return {
            "rotation_vector": [float(value) for value in rotation.reshape(-1)],
            "translation_m": [float(value) for value in translation.reshape(-1)],
        }

    def detect(self, image: np.ndarray, calibration: Optional[CameraCalibration] = None) -> List[Dict[str, Any]]:
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
            raise MarkerRequestError("image must be a uint8 numpy array")
        if image.ndim == 2:
            grayscale = image
        elif image.ndim == 3 and image.shape[2] in (3, 4):
            code = cv2.COLOR_BGR2GRAY if image.shape[2] == 3 else cv2.COLOR_BGRA2GRAY
            grayscale = cv2.cvtColor(image, code)
        else:
            raise MarkerRequestError("image must be grayscale, BGR, or BGRA")
        if grayscale.size == 0 or grayscale.shape[0] * grayscale.shape[1] > _MAX_IMAGE_PIXELS:
            raise MarkerRequestError("image dimensions are invalid")

        corners_list, ids, _rejected = self._detector.detectMarkers(grayscale)
        if ids is None:
            return []
        detections = []
        for marker_id, raw_corners in sorted(zip(ids.reshape(-1), corners_list), key=lambda item: int(item[0])):
            corners = np.asarray(raw_corners, dtype=np.float64).reshape(4, 2)
            detection = {
                "id": int(marker_id),
                "corners": [[float(x), float(y)] for x, y in corners],
                "center": [float(value) for value in corners.mean(axis=0)],
            }
            if calibration is not None:
                detection["pose"] = self._estimate_pose(corners, calibration)
            detections.append(detection)
        return detections


def detect_request(payload: object, detector: Optional[AprilTagDetector] = None) -> Dict[str, Any]:
    image = load_request_image(payload)
    calibration = parse_calibration(payload)
    active_detector = detector if detector is not None else AprilTagDetector()
    return {
        "dictionary": active_detector.dictionary_name,
        "image_size": {"width": int(image.shape[1]), "height": int(image.shape[0])},
        "detections": active_detector.detect(image, calibration),
    }
