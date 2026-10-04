"""Immutable recording-input reuse at the explicit import boundary."""
from pathlib import Path

from core.errors import ArtifactNotFoundError


def recording_artifact(store, *, source_path, kind, mime_type, data, extension, map_name, frame_id, owner):
    """只在显式导入时核对已有字节；持有者在读取租约退出前接管复用制品。"""
    source_path = str(Path(source_path).resolve())
    for ref in store.list_refs(kind=kind):
        if (ref.size_bytes != len(data) or ref.metadata.get("source_path") != source_path
                or ref.metadata.get("map_name") != map_name or ref.metadata.get("frame_id") != frame_id
                or ref.metadata.get("offline_recording") is not True):
            continue
        try:
            with store.read_artifact(ref.artifact_id) as (current, stream):
                if stream.read(len(data) + 1) == data:
                    owner.add_existing_references((current.artifact_id,))
                    return current
        except (ArtifactNotFoundError, FileNotFoundError):
            # 旧侧车或读取路径已失去文件，保留原记录并建立本次真实输入的新身份。
            continue
    return store.save_bytes(kind=kind, mime_type=mime_type, data=data, extension=extension,
        metadata={"map_name": map_name, "frame_id": frame_id, "offline_recording": True,
                  "source_path": source_path})
