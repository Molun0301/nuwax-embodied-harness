#!/usr/bin/env python3
"""One-shot visual query worker backed by the production YOLO-World detector."""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
import json
import math
from pathlib import Path
import re
import sys
from typing import Any, Optional


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from contracts.image import ImageEncoding, ImageFrame  # noqa: E402


_QUERY_PATTERN = re.compile(r"find\s+the\s+['\"](.+?)['\"]", re.IGNORECASE)
_PNG_MAGIC = b"\x89PNG\r\n\x1a\n"
_JPEG_MAGIC = b"\xff\xd8"
MAX_IMAGE_BYTES = 16 * 1024 * 1024
MAX_IMAGE_PIXELS = 16 * 1024 * 1024
MAX_REQUEST_BYTES = 64 * 1024 * 1024


class WorkerRequestError(ValueError):
    """Request does not satisfy the visual-query worker contract."""


class WorkerBackendError(RuntimeError):
    """The production model backend was unavailable or failed."""


def infer_image_encoding(image: bytes, width: int, height: int) -> ImageEncoding:
    """Infer the four encodings accepted by the existing subprocess request."""
    if image.startswith(_PNG_MAGIC):
        return ImageEncoding.PNG
    if image.startswith(_JPEG_MAGIC):
        return ImageEncoding.JPEG
    if len(image) == width * height * 3:
        return ImageEncoding.RGB8
    if len(image) == width * height:
        return ImageEncoding.MONO8
    raise WorkerRequestError("image bytes are neither JPEG/PNG nor dense RGB8/MONO8 for %sx%s" % (width, height))


def _png_dimensions(image: bytes) -> tuple[int, int]:
    if len(image) < 24 or not image.startswith(_PNG_MAGIC):
        raise WorkerRequestError("PNG image has a malformed header")
    if int.from_bytes(image[8:12], "big") != 13 or image[12:16] != b"IHDR":
        raise WorkerRequestError("PNG image has a malformed IHDR")
    width = int.from_bytes(image[16:20], "big")
    height = int.from_bytes(image[20:24], "big")
    if width == 0 or height == 0:
        raise WorkerRequestError("PNG image dimensions must be positive")
    return width, height


def _jpeg_dimensions(image: bytes) -> tuple[int, int]:
    if len(image) < 4 or not image.startswith(_JPEG_MAGIC):
        raise WorkerRequestError("JPEG image has a malformed header")
    position = 2
    sof_markers = set(range(0xC0, 0xD0)) - {0xC4, 0xC8, 0xCC}
    while position < len(image):
        if image[position] != 0xFF:
            raise WorkerRequestError("JPEG image has a malformed marker stream")
        while position < len(image) and image[position] == 0xFF:
            position += 1
        if position >= len(image):
            break
        marker = image[position]
        position += 1
        if marker == 0xDA:
            break
        if marker in {0x01, 0xD8, 0xD9} or 0xD0 <= marker <= 0xD7:
            continue
        if position + 2 > len(image):
            break
        segment_length = int.from_bytes(image[position : position + 2], "big")
        if segment_length < 2 or position + segment_length > len(image):
            raise WorkerRequestError("JPEG image has a malformed segment")
        if marker in sof_markers:
            if segment_length < 7:
                raise WorkerRequestError("JPEG image has a malformed SOF segment")
            height = int.from_bytes(image[position + 3 : position + 5], "big")
            width = int.from_bytes(image[position + 5 : position + 7], "big")
            if width == 0 or height == 0:
                raise WorkerRequestError("JPEG image dimensions must be positive")
            return width, height
        position += segment_length
    raise WorkerRequestError("JPEG image does not contain a valid SOF segment")


def _validate_compressed_dimensions(
    image: bytes,
    encoding: ImageEncoding,
    declared_width: int,
    declared_height: int,
) -> None:
    if encoding == ImageEncoding.PNG:
        actual_width, actual_height = _png_dimensions(image)
    elif encoding == ImageEncoding.JPEG:
        actual_width, actual_height = _jpeg_dimensions(image)
    else:
        return
    actual_pixels = actual_width * actual_height
    if actual_pixels > MAX_IMAGE_PIXELS:
        raise WorkerRequestError(f"image has {actual_pixels} pixels; limit is {MAX_IMAGE_PIXELS}")
    if (actual_width, actual_height) != (declared_width, declared_height):
        raise WorkerRequestError(
            "compressed image dimensions %sx%s do not match declared dimensions %sx%s"
            % (actual_width, actual_height, declared_width, declared_height)
        )


def extract_query(request: dict[str, Any]) -> str:
    """Resolve the dynamic open-vocabulary class from query or the existing prompt."""
    explicit_value = request.get("query")
    if explicit_value is not None and not isinstance(explicit_value, str):
        raise WorkerRequestError("query must be a string")
    explicit = (explicit_value or "").strip()
    if explicit:
        return explicit
    prompt_value = request.get("prompt")
    if not isinstance(prompt_value, str):
        raise WorkerRequestError("prompt must be a string")
    prompt = prompt_value.strip()
    matched = _QUERY_PATTERN.search(prompt)
    if matched is None:
        raise WorkerRequestError("prompt does not contain the requested object class")
    query = matched.group(1).strip()
    if not query:
        raise WorkerRequestError("requested object class is empty")
    return query


def parse_request(payload: object) -> tuple[ImageFrame, str]:
    """Validate JSON payload and build the repository-standard ImageFrame contract."""
    if not isinstance(payload, dict):
        raise WorkerRequestError("request must be a JSON object")
    width = payload.get("width")
    height = payload.get("height")
    if type(width) is not int or type(height) is not int:
        raise WorkerRequestError("width and height must be positive integers")
    if width <= 0 or height <= 0:
        raise WorkerRequestError("width and height must be positive integers")
    pixels = width * height
    if pixels > MAX_IMAGE_PIXELS:
        raise WorkerRequestError(f"image has {pixels} pixels; limit is {MAX_IMAGE_PIXELS}")
    raw_image = payload.get("image")
    if not isinstance(raw_image, list) or not raw_image:
        raise WorkerRequestError("image must be a non-empty JSON byte array")
    if len(raw_image) > MAX_IMAGE_BYTES:
        raise WorkerRequestError(f"image has {len(raw_image)} bytes; limit is {MAX_IMAGE_BYTES}")
    if any(type(value) is not int or not 0 <= value <= 255 for value in raw_image):
        raise WorkerRequestError("image contains a non-byte value")
    image = bytes(raw_image)
    query = extract_query(payload)
    encoding = infer_image_encoding(image, width, height)
    _validate_compressed_dimensions(image, encoding, width, height)
    camera_id = _request_identifier(payload, "camera_id", "visual_query_camera")
    frame_id = _request_identifier(payload, "frame_id", "visual_query_camera")
    frame = ImageFrame(
        camera_id=camera_id,
        frame_id=frame_id,
        width_px=width,
        height_px=height,
        encoding=encoding,
        data=image,
        metadata={
            "source_kind": "nuwax_nav_visual_query_worker",
            "query": query,
            "target_label": query,
            "open_vocabulary_classes": [query],
        },
    )
    return frame, query


def _request_identifier(payload: dict[str, Any], field: str, default: str) -> str:
    if field not in payload:
        return default
    value = payload[field]
    if not isinstance(value, str) or not value.strip():
        raise WorkerRequestError(f"{field} must be a non-empty string when supplied")
    return value


def _validate_backend_args(args: argparse.Namespace) -> None:
    if not isinstance(args.weights, str) or not args.weights.strip():
        raise WorkerRequestError("weights must be a non-empty string")
    for field in ("confidence", "iou"):
        value = getattr(args, field)
        if isinstance(value, bool):
            raise WorkerRequestError(f"{field} must be finite and between 0 and 1")
        try:
            numeric = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise WorkerRequestError(f"{field} must be finite and between 0 and 1") from exc
        if not math.isfinite(numeric) or not 0.0 <= numeric <= 1.0:
            raise WorkerRequestError(f"{field} must be finite and between 0 and 1")
    if type(args.image_size) is not int or args.image_size < 32:
        raise WorkerRequestError("image_size must be an integer greater than or equal to 32")
    if type(args.max_detections) is not int or args.max_detections <= 0:
        raise WorkerRequestError("max_detections must be a positive integer")


def build_backend(args: argparse.Namespace) -> Any:
    """Instantiate the repository's production YOLO-World backend."""
    _validate_backend_args(args)
    from services.perception.detectors import YoloWorldDetectorBackend

    return YoloWorldDetectorBackend(
        name="visual_query_yolo_world",
        weights=args.weights,
        device=args.device,
        confidence_threshold=args.confidence,
        iou_threshold=args.iou,
        image_size=args.image_size,
        max_detections=args.max_detections,
        class_names=("object",),
        dynamic_class_metadata_keys=("open_vocabulary_classes", "query", "target_label"),
        use_worker_process=False,
    )


def select_best_detection(detections: Sequence[Any]) -> Optional[Any]:
    """Return the highest-score detection after validating every result."""
    valid = []
    for detection in detections:
        score = getattr(detection, "score", None)
        bbox = getattr(detection, "bbox", None)
        if bbox is None:
            raise WorkerBackendError("backend returned a detection without a bounding box")
        if type(score) is bool:
            raise WorkerBackendError("backend returned a detection with an invalid score")
        try:
            numeric_score = float(score)
        except (TypeError, ValueError, OverflowError) as exc:
            raise WorkerBackendError("backend returned a detection with an invalid score") from exc
        if not math.isfinite(numeric_score):
            raise WorkerBackendError("backend returned a detection with an invalid score")
        coordinates = []
        for field in ("x_px", "y_px", "width_px", "height_px"):
            value = getattr(bbox, field, None)
            if type(value) is bool:
                raise WorkerBackendError("backend returned an invalid bounding box")
            try:
                coordinates.append(float(value))
            except (TypeError, ValueError, OverflowError) as exc:
                raise WorkerBackendError("backend returned an invalid bounding box") from exc
        if not all(math.isfinite(value) for value in coordinates) or coordinates[2] < 0.0 or coordinates[3] < 0.0:
            raise WorkerBackendError("backend returned an invalid bounding box")
        valid.append((numeric_score, detection))
    return max(valid, key=lambda item: item[0])[1] if valid else None


def query_backend(request: object, backend: Any) -> Optional[dict[str, Any]]:
    """Run one real backend query and return the existing bbox parser schema."""
    frame, query = parse_request(request)
    bundle = backend.detect(frame)
    metadata_value = getattr(bundle, "metadata", None)
    if not isinstance(metadata_value, Mapping):
        raise WorkerBackendError("backend returned invalid metadata")
    metadata = dict(metadata_value)
    if metadata.get("availability") == "unavailable" or metadata.get("stage_status") == "not_ready":
        raise WorkerBackendError(str(metadata.get("error") or metadata.get("reason") or "backend unavailable"))
    detections = getattr(bundle, "detections_2d", None)
    if not isinstance(detections, Sequence) or isinstance(detections, (str, bytes, bytearray)):
        raise WorkerBackendError("backend returned an invalid detections collection")
    detection = select_best_detection(detections)
    if detection is None:
        return None
    bbox = detection.bbox
    x1 = float(bbox.x_px)
    y1 = float(bbox.y_px)
    x2 = x1 + float(bbox.width_px)
    y2 = y1 + float(bbox.height_px)
    if not all(math.isfinite(value) for value in (x1, y1, x2, y2)) or x2 < x1 or y2 < y1:
        raise WorkerBackendError("backend returned an invalid bounding box")
    return {
        "name": str(getattr(detection, "label", None) or query),
        "bbox": [x1, y1, x2, y2],
        "score": float(detection.score),
    }


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", default="yolov8s-worldv2.pt")
    parser.add_argument("--device", default="")
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--max-detections", type=int, default=100)
    parser.add_argument("--preflight", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        if args.preflight:
            backend = build_backend(args)
            json.dump(
                {
                    "status": "ready",
                    "backend": "%s.%s" % (type(backend).__module__, type(backend).__name__),
                    "weights": args.weights,
                    "device": args.device or "auto",
                },
                sys.stdout,
                separators=(",", ":"),
            )
            sys.stdout.write("\n")
            return 0
        request_bytes = sys.stdin.buffer.read(MAX_REQUEST_BYTES + 1)
        if len(request_bytes) > MAX_REQUEST_BYTES:
            raise WorkerRequestError(f"request exceeds {MAX_REQUEST_BYTES} bytes")
        request = json.loads(request_bytes)
        parse_request(request)
        backend = build_backend(args)
        result = query_backend(request, backend)
        if result is None:
            sys.stdout.write("None\n")
        else:
            json.dump(result, sys.stdout, separators=(",", ":"))
            sys.stdout.write("\n")
        return 0
    except Exception as exc:  # noqa: BLE001 - subprocess errors must fail closed.
        sys.stderr.write("visual query worker failed: %s\n" % exc)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
