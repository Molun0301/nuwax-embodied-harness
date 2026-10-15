from __future__ import annotations

import argparse
import gc
import json
import os
import tempfile
from pathlib import Path
from typing import Dict, Iterable, Sequence, Tuple


DEFAULT_TEXT_MODEL_ID = "BAAI/bge-m3"
DEFAULT_IMAGE_MODEL_ID = "openai/clip-vit-base-patch32"
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "runtime_data" / "models"
COMPLETION_FILE = ".neh-memory-model-ready.json"
COMPLETION_SCHEMA = 1


def build_parser() -> argparse.ArgumentParser:
    """构建命令行参数。"""

    parser = argparse.ArgumentParser(description="手工下载记忆向量模型到本地目录。")
    parser.add_argument(
        "--output-root",
        default=str(DEFAULT_OUTPUT_ROOT),
        help="模型输出根目录，默认写入 runtime_data/models/。",
    )
    parser.add_argument(
        "--text-model-id",
        default=DEFAULT_TEXT_MODEL_ID,
        help="文本向量模型仓库 ID，默认使用 BAAI/bge-m3。",
    )
    parser.add_argument(
        "--image-model-id",
        default=DEFAULT_IMAGE_MODEL_ID,
        help="图像向量模型仓库 ID，默认使用 openai/clip-vit-base-patch32。",
    )
    parser.add_argument(
        "--text-output-dir",
        default="",
        help="文本向量模型输出目录。未设置时自动写入 output-root 下的固定目录。",
    )
    parser.add_argument(
        "--image-output-dir",
        default="",
        help="图像向量模型输出目录。未设置时自动写入 output-root 下的固定目录。",
    )
    parser.add_argument(
        "--skip-text",
        action="store_true",
        help="跳过文本向量模型下载。",
    )
    parser.add_argument(
        "--skip-image",
        action="store_true",
        help="跳过图像向量模型下载。",
    )
    parser.add_argument(
        "--hf-token",
        default=os.environ.get("HF_TOKEN", ""),
        help="Hugging Face 访问令牌。未设置时复用环境变量 HF_TOKEN。",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=120.0,
        help="下载超时时间，单位秒。",
    )
    parser.add_argument(
        "--force-download",
        action="store_true",
        help="强制重新下载并覆盖目标目录内容。",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="仅离线校验指定的本地模型；不下载或替换自定义模型文件。",
    )
    return parser


def _normalize_path(value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = (Path.cwd() / path).resolve()
    return path


def _repo_id_to_dir_name(repo_id: str) -> str:
    return repo_id.strip().replace("/", "__")


def _resolve_output_dir(output_root: Path, explicit_dir: str, repo_id: str) -> Path:
    if explicit_dir.strip():
        return _normalize_path(explicit_dir)
    return output_root / _repo_id_to_dir_name(repo_id)


def _model_kind(required_files: Sequence[str]) -> str:
    return "text" if "modules.json" in required_files else "image"


def _model_inventory(target_dir: Path) -> Dict[str, dict]:
    """Read file metadata, never model bytes or hashes, including weight shards.

    Hugging Face's resumable-download cache is not a runtime model artifact. The
    receipt records the files validated by the actual model loader, rather than
    guessing a model's format from a fixed filename or minimum weight size.
    """
    files = {}
    visited = set()
    for directory, subdirs, names in os.walk(str(target_dir), followlinks=True):
        base = Path(directory)
        directory_stat = base.stat()
        identity = (directory_stat.st_dev, directory_stat.st_ino)
        if identity in visited:
            subdirs[:] = []
            continue
        visited.add(identity)
        subdirs[:] = sorted(name for name in subdirs if name not in (".cache", ".git", "__pycache__"))
        for name in sorted(names):
            if name == COMPLETION_FILE or name.startswith(".neh-model-ready-"):
                continue
            path = base / name
            if not path.is_file():
                continue
            info = path.stat()
            files[path.relative_to(target_dir).as_posix()] = {
                "size": info.st_size,
                "mtime_ns": info.st_mtime_ns,
            }
    return files


def _looks_prepared(target_dir: Path, required_files: Sequence[str]) -> bool:
    """Only a completed, unchanged local-loader validation proves readiness."""
    if not target_dir.is_dir() or not all((target_dir / name).is_file() for name in required_files):
        return False
    try:
        receipt = json.loads((target_dir / COMPLETION_FILE).read_text(encoding="utf-8"))
        return (
            isinstance(receipt, dict)
            and receipt.get("schema") == COMPLETION_SCHEMA
            and receipt.get("complete") is True
            and receipt.get("kind") == _model_kind(required_files)
            and bool(receipt.get("files"))
            and receipt.get("files") == _model_inventory(target_dir)
        )
    except (OSError, ValueError, TypeError):
        return False


def _validate_local_model(target_dir: Path, required_files: Sequence[str]) -> None:
    """Use the runtime loaders once, offline and on CPU, without an inference run.

    The loaders understand sharded checkpoints, tokenizer files, Sentence
    Transformer module layouts and custom local models. Repeated preparation
    uses the completion receipt instead of loading multi-GB weights again.
    """
    missing = [name for name in required_files if not (target_dir / name).is_file()]
    if missing:
        raise ValueError("缺少必要文件：" + ", ".join(missing))
    offline_keys = ("HF_HUB_OFFLINE", "TRANSFORMERS_OFFLINE")
    previous = {key: os.environ.get(key) for key in offline_keys}
    try:
        for key in offline_keys:
            os.environ[key] = "1"
        if _model_kind(required_files) == "text":
            from sentence_transformers import SentenceTransformer

            model = SentenceTransformer(
                str(target_dir), device="cpu", local_files_only=True, trust_remote_code=False,
            )
        else:
            from transformers import CLIPModel, CLIPProcessor

            CLIPProcessor.from_pretrained(str(target_dir), local_files_only=True)
            model = CLIPModel.from_pretrained(str(target_dir), local_files_only=True)
        del model
    finally:
        for key, value in previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        gc.collect()


def _validate_and_record(target_dir: Path, required_files: Sequence[str], repo_id: str) -> None:
    before = _model_inventory(target_dir)
    _validate_local_model(target_dir, required_files)
    files = _model_inventory(target_dir)
    if before != files:
        raise ValueError("校验期间模型文件发生变化，请在下载完成后重试")
    receipt = {
        "schema": COMPLETION_SCHEMA,
        "complete": True,
        "kind": _model_kind(required_files),
        "repo_id": repo_id,
        "files": files,
    }
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", prefix=".neh-model-ready-", dir=str(target_dir), delete=False,
        ) as stream:
            temporary = Path(stream.name)
            json.dump(receipt, stream, ensure_ascii=False, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(target_dir / COMPLETION_FILE))
    except OSError as exc:
        # A valid pre-existing read-only model remains usable without a receipt.
        print(f"模型离线校验通过，但无法保存完成记录（下次将重新校验）：{exc}")
    finally:
        if temporary is not None and temporary.exists():
            try:
                temporary.unlink()
            except OSError:
                pass


def _iter_download_jobs(args: argparse.Namespace) -> Iterable[Tuple[str, str, Path, Tuple[str, ...]]]:
    output_root = _normalize_path(args.output_root)
    if not args.skip_text:
        yield (
            "文本向量模型",
            args.text_model_id.strip(),
            _resolve_output_dir(output_root, args.text_output_dir, args.text_model_id),
            ("modules.json",),
        )
    if not args.skip_image:
        yield (
            "图像向量模型",
            args.image_model_id.strip(),
            _resolve_output_dir(output_root, args.image_output_dir, args.image_model_id),
            ("config.json", "preprocessor_config.json"),
        )


def _download_snapshot(
    *,
    repo_id: str,
    target_dir: Path,
    token: str,
    timeout_sec: float,
    force_download: bool,
) -> Path:
    try:
        from huggingface_hub import snapshot_download
    except ImportError as exc:
        raise SystemExit(
            "当前环境未安装 huggingface_hub。请先执行 "
            "`python3 -m pip install -r requirements-memory-models.txt`。"
        ) from exc

    os.environ["HF_HUB_DOWNLOAD_TIMEOUT"] = str(max(10, int(timeout_sec)))
    os.environ["HF_HUB_ETAG_TIMEOUT"] = str(max(10, int(timeout_sec)))
    target_dir.parent.mkdir(parents=True, exist_ok=True)

    try:
        snapshot_download(
            repo_id=repo_id,
            local_dir=str(target_dir),
            token=token or None,
            force_download=force_download,
        )
    except TypeError:
        snapshot_download(
            repo_id=repo_id,
            local_dir=str(target_dir),
            token=token or None,
        )
    return target_dir


def _prepare_one_model(
    *,
    label: str,
    repo_id: str,
    target_dir: Path,
    required_files: Sequence[str],
    token: str,
    timeout_sec: float,
    force_download: bool,
    validate_only: bool = False,
) -> Path:
    if not repo_id:
        raise SystemExit(f"{label}缺少仓库 ID。")

    if _looks_prepared(target_dir, required_files) and not force_download:
        print(f"{label}已存在，跳过下载：{target_dir}")
        return target_dir

    if not force_download or validate_only:
        try:
            _validate_and_record(target_dir, required_files, repo_id)
        except ImportError as exc:
            raise SystemExit(f"{label}离线校验缺少模型依赖：{exc}；请安装 requirements-memory-models.txt。") from exc
        except Exception as exc:
            if validate_only:
                raise SystemExit(f"自定义{label}未准备完成：{target_dir}；离线校验失败：{exc}") from exc
            print(f"{label}尚未准备完成，将继续下载：{exc}")
        else:
            print(f"{label}离线校验通过，跳过下载：{target_dir}")
            return target_dir

    # An interrupted refresh must never leave an earlier completion claim.
    receipt_path = target_dir / COMPLETION_FILE
    if receipt_path.exists():
        receipt_path.unlink()

    print(f"开始下载{label}：repo_id={repo_id} target={target_dir}")
    _download_snapshot(
        repo_id=repo_id,
        target_dir=target_dir,
        token=token,
        timeout_sec=timeout_sec,
        force_download=force_download,
    )

    try:
        _validate_and_record(target_dir, required_files, repo_id)
    except Exception as exc:
        raise SystemExit(
            f"{label}下载后离线校验失败：{exc}。目标目录：{target_dir}；"
            "已保留下载文件供重试；文件损坏时可使用 --force-download。"
        ) from exc

    print(f"{label}已准备完成：{target_dir}")
    return target_dir


def main() -> int:
    """执行模型下载流程。"""

    parser = build_parser()
    args = parser.parse_args()
    if args.validate_only and args.force_download:
        parser.error("--validate-only 不能与 --force-download 同时使用。")
    jobs = list(_iter_download_jobs(args))
    if not jobs:
        raise SystemExit("至少保留一个模型下载任务。")

    prepared_paths = {}
    for label, repo_id, target_dir, required_files in jobs:
        prepared_paths[label] = _prepare_one_model(
            label=label,
            repo_id=repo_id,
            target_dir=target_dir,
            required_files=required_files,
            token=args.hf_token.strip(),
            timeout_sec=max(10.0, float(args.timeout_sec)),
            force_download=bool(args.force_download),
            validate_only=bool(args.validate_only),
        )

    print("")
    print("建议把以下配置写入 .env：")
    if "文本向量模型" in prepared_paths:
        print(f"NUWAX_MEMORY_TEXT_EMBEDDING_MODEL={prepared_paths['文本向量模型']}")
    if "图像向量模型" in prepared_paths:
        print(f"NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL={prepared_paths['图像向量模型']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
