from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Iterable, List, Optional, Tuple
import urllib.request
import zipfile


PROJECT_ROOT = Path(__file__).resolve().parent.parent

DEFAULT_RELEASE = "v8.4.0"
DEFAULT_REPO = "ultralytics/assets"
DEFAULT_CALIBRATION_ROOT = PROJECT_ROOT / "runtime_data" / "calibration" / "yolo_world"
COCO8_URL = "https://github.com/ultralytics/assets/releases/download/v0.0.0/coco8.zip"
COCO_VAL2017_URL = "http://images.cocodataset.org/zips/val2017.zip"
COCO_CLASSES = (
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush",
)
DEFAULT_CLASSES = (
    "person",
    "door",
    "chair",
    "table",
    "shelf",
    "cabinet",
    "box",
    "bag",
    "backpack",
    "bottle",
    "cup",
    "bench",
    "stairs",
    "charger",
    "dock",
    "traffic cone",
    "fire hydrant",
)


def build_parser() -> argparse.ArgumentParser:
    """构建命令行参数。"""

    parser = argparse.ArgumentParser(description="一键准备 YOLO-Worldv2 权重并导出 TensorRT engine。")
    parser.add_argument("--model", default="yolov8s-worldv2.pt", help="模型名或本地权重路径。")
    parser.add_argument(
        "--weights-output",
        default="runtime_data/models/yolov8s-worldv2.pt",
        help="权重输出路径。",
    )
    parser.add_argument(
        "--engine-output",
        default="runtime_data/models/yolov8s-worldv2.engine",
        help="TensorRT engine 输出路径。",
    )
    parser.add_argument("--release", default=DEFAULT_RELEASE, help="Ultralytics assets release 版本。")
    parser.add_argument("--download-url", default="", help="显式指定权重下载地址。")
    parser.add_argument("--overwrite", action="store_true", help="覆盖已有权重和 engine。")
    parser.add_argument("--download-timeout-sec", type=float, default=60.0, help="单次下载超时时间。")
    parser.add_argument("--download-only", action="store_true", help="只下载/准备权重，不导出 engine。")
    parser.add_argument("--imgsz", type=int, default=640, help="导出输入尺寸。")
    parser.add_argument("--device", default="0", help="导出设备，例如 0、cuda:0。")
    parser.add_argument("--batch", type=int, default=1, help="导出 batch 大小。")
    parser.add_argument("--workspace", type=float, default=None, help="TensorRT workspace，单位 GiB。")
    parser.add_argument("--dynamic", action="store_true", help="导出 dynamic shape engine。")
    parser.add_argument("--simplify", action="store_true", default=False, help="启用 ONNX simplification。Jetson 默认关闭，避免触发 onnxruntime-gpu 自动安装。")
    parser.add_argument("--half", action="store_true", default=True, help="启用 FP16。")
    parser.add_argument("--no-half", action="store_false", dest="half", help="关闭 FP16。")
    parser.add_argument("--int8", action="store_true", default=False, help="启用 INT8 量化。")
    parser.add_argument("--no-int8", action="store_false", dest="int8", help="关闭 INT8，只导出 FP16/FP32 engine。")
    parser.add_argument(
        "--calib-data",
        default="",
        help="INT8 校准数据集 YAML。为空时按 --calib-preset 准备校准数据。",
    )
    parser.add_argument(
        "--calib-preset",
        choices=("none", "coco8", "coco-val2017"),
        default="coco8",
        help="INT8 校准数据预设。coco8 轻量验证，coco-val2017 更适合通用 COCO 校准。",
    )
    parser.add_argument(
        "--calib-root",
        default=str(DEFAULT_CALIBRATION_ROOT),
        help="自动下载校准数据的保存目录。",
    )
    parser.add_argument("--calib-images", default="", help="根据图片目录临时生成 INT8 校准 YAML。")
    parser.add_argument("--calib-fraction", type=float, default=1.0, help="INT8 校准数据使用比例。")
    parser.add_argument("--classes", default=",".join(DEFAULT_CLASSES), help="YOLO-World 导出前固化的开放词汇类别。")
    return parser


def _normalize_path(value: str, *, base_dir: Path = PROJECT_ROOT) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = (base_dir / path).resolve()
    return path


def _parse_classes(value: str) -> Tuple[str, ...]:
    items = value.replace("，", ",").replace("；", ";").replace("|", ",").split(",")
    return tuple(item.strip().lower() for item in items if item.strip())


def _iter_candidate_urls(model_name: str, release: str, explicit_url: str) -> Iterable[str]:
    if explicit_url.strip():
        yield explicit_url.strip()
    yield f"https://github.com/{DEFAULT_REPO}/releases/download/{release}/{Path(model_name).name}"


def _run_external_downloader(command: List[str]) -> bool:
    try:
        completed = subprocess.run(command, check=False)
    except FileNotFoundError:
        return False
    return completed.returncode == 0


def _download_with_wget(url: str, target: Path, timeout_sec: float) -> bool:
    return _run_external_downloader([
        "wget",
        "-4",
        "-O",
        str(target),
        "--timeout=%d" % max(1, int(timeout_sec)),
        "--tries=2",
        url,
    ])


def _download_with_curl(url: str, target: Path, timeout_sec: float) -> bool:
    return _run_external_downloader([
        "curl",
        "-L",
        "-4",
        "--fail",
        "--connect-timeout",
        str(max(1, int(timeout_sec))),
        "-o",
        str(target),
        url,
    ])


def _download_with_urllib(url: str, target: Path, timeout_sec: float) -> bool:
    try:
        with urllib.request.urlopen(url, timeout=timeout_sec) as response:
            with target.open("wb") as file:
                shutil.copyfileobj(response, file)
    except Exception:
        return False
    return True


def _materialize_temp_file(source: Path, *, prefix: str, suffix: str) -> Path:
    temp_file = tempfile.NamedTemporaryFile(prefix=prefix, suffix=suffix, delete=False)
    temp_file.close()
    target = Path(temp_file.name)
    shutil.copy2(str(source), str(target))
    return target


def _download_to_temp(urls: Iterable[str], timeout_sec: float, *, suffix: str) -> Optional[Path]:
    with tempfile.TemporaryDirectory(prefix="nuwax_yolo_world_download_") as temp_dir:
        temp_path = Path(temp_dir) / ("download" + suffix)
        for url in urls:
            if _download_with_wget(url, temp_path, timeout_sec) and temp_path.exists() and temp_path.stat().st_size > 0:
                return _materialize_temp_file(temp_path, prefix="nuwax_yolo_world_download_", suffix=suffix)
            if _download_with_curl(url, temp_path, timeout_sec) and temp_path.exists() and temp_path.stat().st_size > 0:
                return _materialize_temp_file(temp_path, prefix="nuwax_yolo_world_download_", suffix=suffix)
            if _download_with_urllib(url, temp_path, timeout_sec) and temp_path.exists() and temp_path.stat().st_size > 0:
                return _materialize_temp_file(temp_path, prefix="nuwax_yolo_world_download_", suffix=suffix)
    return None


def _ensure_min_bytes(path: Path, min_bytes: int = 100_000) -> None:
    if not path.exists():
        raise SystemExit("文件不存在：%s" % path)
    size = path.stat().st_size
    if size < min_bytes:
        raise SystemExit("文件大小异常，可能未下载完成：%s (%d bytes)" % (path, size))


def _is_original_yolo_world_weight(path: Path) -> bool:
    name = path.name.lower()
    return name.startswith("yolov8") and name.endswith("-world.pt") and "worldv2" not in name


def _prepare_weights(args: argparse.Namespace) -> Path:
    output_path = _normalize_path(args.weights_output)
    if output_path.exists() and not args.overwrite:
        _ensure_min_bytes(output_path)
        print("YOLO-World 权重已存在：%s" % output_path)
        return output_path

    source = Path(args.model).expanduser()
    temp_download: Optional[Path] = None
    if source.exists():
        source_path = source.resolve()
    else:
        temp_download = _download_to_temp(
            _iter_candidate_urls(args.model, args.release, args.download_url),
            timeout_sec=max(1.0, float(args.download_timeout_sec)),
            suffix=".pt",
        )
        if temp_download is None:
            raise SystemExit("未能自动下载 YOLO-World 权重。请使用 --download-url 或先手工放置权重。")
        source_path = temp_download

    output_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(str(source_path), str(output_path))
    _ensure_min_bytes(output_path)
    if temp_download is not None and temp_download.exists():
        temp_download.unlink()
    print("YOLO-World 权重已准备完成：%s" % output_path)
    return output_path


def _write_calibration_yaml(
    image_dir: str,
    output_dir: Path,
    classes: Tuple[str, ...],
    *,
    yaml_name: str = "yolo_world_int8_calibration.yaml",
) -> Path:
    image_path = _normalize_path(image_dir, base_dir=Path.cwd())
    if not image_path.exists() or not image_path.is_dir():
        raise SystemExit("INT8 校准图片目录不存在：%s" % image_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    yaml_path = output_dir / yaml_name
    names = "\n".join("  %d: %s" % (index, label) for index, label in enumerate(classes or DEFAULT_CLASSES))
    yaml_path.write_text(
        "path: %s\ntrain: .\nval: .\nnames:\n%s\n" % (image_path, names),
        encoding="utf-8",
    )
    return yaml_path


def _extract_zip(zip_path: Path, target_dir: Path) -> None:
    target_dir.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(zip_path) as archive:
        archive.extractall(target_dir)


def _download_and_extract_zip(url: str, target_dir: Path, image_dir: Path, timeout_sec: float) -> None:
    if image_dir.exists() and any(image_dir.iterdir()):
        return
    temp_zip = _download_to_temp((url,), timeout_sec=timeout_sec, suffix=".zip")
    if temp_zip is None:
        raise SystemExit("未能自动下载 INT8 校准数据：%s" % url)
    try:
        _extract_zip(temp_zip, target_dir)
    finally:
        if temp_zip.exists():
            temp_zip.unlink()
    if not image_dir.exists() or not any(image_dir.iterdir()):
        raise SystemExit("INT8 校准数据解压后未找到图片目录：%s" % image_dir)


def _prepare_calibration_preset(args: argparse.Namespace, output_dir: Path, classes: Tuple[str, ...]) -> str:
    preset = str(args.calib_preset or "none").strip().lower()
    if preset == "none":
        return ""

    root = _normalize_path(str(args.calib_root), base_dir=PROJECT_ROOT)
    if preset == "coco8":
        dataset_dir = root / "coco8"
        image_dir = dataset_dir / "images" / "val"
        _download_and_extract_zip(
            COCO8_URL,
            root,
            image_dir,
            timeout_sec=max(1.0, float(args.download_timeout_sec)),
        )
        return str(_write_calibration_yaml(
            str(image_dir),
            output_dir,
            COCO_CLASSES,
            yaml_name="coco8_calibration.yaml",
        ))

    if preset == "coco-val2017":
        dataset_dir = root / "coco"
        image_dir = dataset_dir / "val2017"
        _download_and_extract_zip(
            COCO_VAL2017_URL,
            dataset_dir,
            image_dir,
            timeout_sec=max(1.0, float(args.download_timeout_sec)),
        )
        return str(_write_calibration_yaml(
            str(image_dir),
            output_dir,
            COCO_CLASSES,
            yaml_name="coco_val2017_calibration.yaml",
        ))

    raise SystemExit("未知 INT8 校准数据预设：%s" % preset)


def _load_yolo_world_model(weights_path: Path):
    try:
        from ultralytics import YOLOWorld as model_class
    except ImportError:
        try:
            from ultralytics import YOLO as model_class
        except ImportError as exc:
            raise SystemExit("当前环境未安装 ultralytics，无法导出 TensorRT engine。") from exc
    return model_class(str(weights_path))


def _export_engine(args: argparse.Namespace, weights_path: Path) -> Path:
    if _is_original_yolo_world_weight(weights_path):
        raise SystemExit(
            "原版 YOLO-World 权重不支持导出 TensorRT engine，请改用 yolov8s-worldv2.pt。"
        )
    engine_path = _normalize_path(args.engine_output)
    if engine_path.exists() and not args.overwrite:
        _ensure_min_bytes(engine_path)
        print("TensorRT engine 已存在：%s" % engine_path)
        return engine_path

    classes = _parse_classes(args.classes)
    if not classes:
        raise SystemExit("YOLO-World 导出必须提供至少一个开放词汇类别。")

    model = _load_yolo_world_model(weights_path)
    if hasattr(model, "set_classes"):
        model.set_classes(list(classes))

    export_kwargs = {
        "format": "engine",
        "imgsz": max(32, int(args.imgsz)),
        "batch": max(1, int(args.batch)),
        "half": bool(args.half),
        "int8": bool(args.int8),
        "dynamic": bool(args.dynamic),
        "simplify": bool(args.simplify),
    }
    if args.device:
        export_kwargs["device"] = args.device
    if args.workspace is not None:
        export_kwargs["workspace"] = float(args.workspace)
    if args.int8:
        calib_data = str(args.calib_data or "").strip()
        if args.calib_images:
            calib_data = str(_write_calibration_yaml(args.calib_images, engine_path.parent, classes))
        if not calib_data:
            calib_data = _prepare_calibration_preset(args, engine_path.parent, classes)
        if not calib_data:
            raise SystemExit("INT8 导出需要 --calib-data、--calib-images 或 --calib-preset。")
        export_kwargs["data"] = calib_data
        export_kwargs["fraction"] = max(0.0, min(1.0, float(args.calib_fraction)))

    exported_path = Path(model.export(**export_kwargs))
    engine_path.parent.mkdir(parents=True, exist_ok=True)
    if exported_path.resolve() != engine_path.resolve():
        shutil.move(str(exported_path), str(engine_path))
    _ensure_min_bytes(engine_path)

    metadata_path = engine_path.with_suffix(engine_path.suffix + ".metadata.txt")
    metadata_path.write_text(
        "weights=%s\nengine=%s\nclasses=%s\nimgsz=%s\nbatch=%s\nint8=%s\nsimplify=%s\ncalib_data=%s\ncalib_fraction=%s\n"
        % (
            weights_path,
            engine_path,
            ",".join(classes),
            max(32, int(args.imgsz)),
            max(1, int(args.batch)),
            bool(args.int8),
            bool(args.simplify),
            export_kwargs.get("data", ""),
            export_kwargs.get("fraction", ""),
        ),
        encoding="utf-8",
    )
    print("YOLO-World TensorRT engine 导出完成：%s" % engine_path)
    print("导出元数据已写入：%s" % metadata_path)
    return engine_path


def main() -> int:
    """执行一键准备流程。"""

    args = build_parser().parse_args()
    weights_path = _prepare_weights(args)
    if args.download_only:
        return 0
    _export_engine(args, weights_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
