"""将BL5无损解码为原地表校验结构；长度和集合上限在分配前检查。"""
import struct


def decode_ground_binary(payload, decode_json, decode_samples):
    view = memoryview(payload)
    offset = 0

    def take(length):
        nonlocal offset
        if length > len(view) - offset:
            raise ValueError("精细地图BL5地表记录截断")
        result = view[offset:offset + length]
        offset += length
        return result

    def numbers(fmt):
        return list(struct.unpack(fmt, take(struct.calcsize(fmt))))

    def count(limit):
        value = numbers("<B")[0]
        if value > limit:
            raise ValueError("精细地图BL5集合超出预算")
        return value

    def samples():
        sources = [decode_json(bytes(take(numbers("<I")[0]))) for _ in range(count(32))]
        values = []
        for _ in range(count(32)):
            index = numbers("<B")[0]
            point = numbers("<3d")
            sequence, stamp, point_index = numbers("<QQI")
            values.append([index, point, sequence, stamp, point_index])
        encoded = dict(sources=sources, values=values)
        decode_samples(encoded)  # 共享原严格来源排序、使用情况和样本索引校验。
        return encoded

    size = numbers("<I")[0]
    if not 1 <= size <= 32768:
        raise ValueError("精细地图BL5地表数量错误")
    result = []
    for _ in range(size):
        anchor, tag = numbers("<HB")
        if tag == 0:
            members = numbers("<" + "H" * count(32))
            origin, normal = numbers("<3d"), numbers("<3d")
            encoded = samples()
            decoded = decode_samples(encoded)
            numeric, groups = [], []
            for source in encoded["sources"]:
                if source != {"kind": "legacy"}:
                    groups.append(dict(source=source, sample_positions=[i for i, sample in enumerate(decoded)
                                                                       if sample["source"] == source]))
            for sample in decoded:
                numeric.append({key: value for key, value in sample.items() if key != "source"})
            data = dict(members=members, plane_origin=origin, normal=normal, samples=numeric)
            if groups:
                data["sample_sources"] = groups
            representation = "legacy_plane"
        elif tag == 1:
            data = dict(anchor=numbers("<3i"), height_m=numbers("<d")[0], occupied_bounds=numbers("<2d"))
            data["members"] = [[numbers("<H")[0], numbers("<8d")] for _ in range(count(32))]
            data["samples"] = samples()
            representation = "heightfield_v1_compact"
        elif tag == 2:
            data, representation = {}, "classified_empty_column"
        else:
            raise ValueError("精细地图BL5地表标签未知")
        result.append([anchor, dict(representation=representation, data=data)])
    if offset != len(view):
        raise ValueError("精细地图BL5地表尾随数据")
    return result
