"""完整图的私有只读载荷；按固定块复制，避免大图进入单个内存字节串。"""
import hashlib
import mmap
import tempfile

CHUNK_BYTES = 1024 * 1024
MAX_PLANAR_ARCHIVE_BYTES = 2 * 512 * 1024 * 1024 + 16 * 1024 * 1024 + 64 * 1024 + 12


class FrozenPayload:
    def __init__(self, chunks, maximum_bytes=MAX_PLANAR_ARCHIVE_BYTES):
        temporary = tempfile.TemporaryFile(prefix="nuwax-planar-payload-")
        try:
            size = 0
            for chunk in chunks:
                if not isinstance(chunk, bytes) or len(chunk) > CHUNK_BYTES:
                    raise ValueError("二维图载荷分块类型或大小非法")
                size += len(chunk)
                if size > maximum_bytes:
                    raise ValueError("二维图载荷超过预算")
                temporary.write(chunk)
            if not size:
                raise ValueError("二维图载荷为空")
            temporary.flush()
            self._data = mmap.mmap(temporary.fileno(), 0, access=mmap.ACCESS_READ)
            self._size = size
        finally:
            temporary.close()

    def __len__(self):
        return self._size

    def __getitem__(self, key):
        return self._data[key]

    def chunks(self):
        for offset in range(0, self._size, CHUNK_BYTES):
            yield self._data[offset:offset + CHUNK_BYTES]

    def __eq__(self, other):
        if not isinstance(other, (bytes, FrozenPayload)) or len(other) != len(self):
            return False
        return all(self[offset:offset + CHUNK_BYTES] == other[offset:offset + CHUNK_BYTES]
                   for offset in range(0, len(self), CHUNK_BYTES))


def payload_digest(value):
    """Explicit legacy-writer adapter; never called by assigned-identity paths."""
    digest = hashlib.sha256()
    for chunk in payload_chunks(value):
        digest.update(chunk)
    return digest.digest()


def payload_chunks(value):
    for offset in range(0, len(value), CHUNK_BYTES):
        yield value[offset:offset + CHUNK_BYTES]
