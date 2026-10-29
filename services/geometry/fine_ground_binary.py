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


def decode_ground_shared(payload, source_identity):
    """BL6：整块共享原始来源描述，返回原逻辑记录及来源表供根边界核验。"""
    view, offset = memoryview(payload), 0

    def take(length):
        nonlocal offset
        if length > len(view) - offset:
            raise ValueError("精细地图BL6记录截断")
        result = view[offset:offset + length]
        offset += length
        return result

    def numbers(fmt):
        return list(struct.unpack(fmt, take(struct.calcsize(fmt))))

    def count(limit):
        value = numbers("<B")[0]
        if value > limit:
            raise ValueError("精细地图BL6集合超过结构上限")
        return value

    def text():
        return bytes(take(numbers("<I")[0])).decode("utf-8")

    mode, source_bytes, source_count = numbers("<BII")
    if mode > 2 or source_bytes > 36 * 1024 * 1024 or source_count > 32768 * 32:
        raise ValueError("精细地图BL6来源上界错误")
    # 先限制来源切片，再读每个字符串，不能越过来源区去引用后续数值载荷。
    remainder = view[offset + source_bytes:]
    view = take(source_bytes)
    offset = 0
    sources, identities = [], []
    for _ in range(source_count):
        tag = numbers("<B")[0]
        if tag == 0:
            source = dict(kind="legacy")
        elif tag == 1:
            backend, runtime = numbers("<iq")
            source = dict(kind="lidar", backend=backend, runtime_generation=runtime, sensor_frame_id=text())
        elif tag == 2:
            runtime, session = numbers("<QQ")
            source = dict(kind="auxiliary", runtime_generation=runtime, session_generation=session,
                          sensor_id=text(), clock_id=text())
        else:
            raise ValueError("精细地图BL6来源标签错误")
        identity = source_identity(source)
        if identities and identities[-1] >= identity:
            raise ValueError("精细地图BL6来源重复或无序")
        sources.append(source)
        identities.append(identity)
    if offset != len(view) or (mode < 2 and (mode == 1) != any(s["kind"] != "legacy" for s in sources)):
        raise ValueError("精细地图BL6来源长度或模式错误")
    view, offset = remainder, 0
    record_count = numbers("<I")[0]
    if not 1 <= record_count <= 32768:
        raise ValueError("精细地图BL6记录数量错误")
    used, records = set(), []

    def samples():
        result, indexes = [], []
        for _ in range(count(32)):
            index, x, y, z, sequence, stamp, point_index = numbers("<I3dQQI")
            if index >= len(sources):
                raise ValueError("精细地图BL6样本来源越界")
            used.add(index)
            indexes.append(index)
            result.append(dict(point=[x, y, z], source_sequence=sequence,
                               source_timestamp_ns=stamp, point_index=point_index))
        return result, indexes

    for _ in range(record_count):
        anchor, tag = numbers("<HB")
        if tag == 0:
            data = dict(members=numbers("<" + "H" * count(32)),
                        plane_origin=numbers("<3d"), normal=numbers("<3d"))
            values, indexes = samples()
            data["samples"] = values
            groups = [dict(source=sources[index], sample_positions=[i for i, value in enumerate(indexes) if value == index])
                      for index in sorted(set(indexes)) if sources[index]["kind"] != "legacy"]
            if groups:
                data["sample_sources"] = groups
            representation = "legacy_plane"
        elif tag == 1 and mode == 2:
            data = dict(anchor=numbers("<3i"), height_m=numbers("<d")[0], occupied_bounds=numbers("<2d"))
            data["members"] = [[numbers("<H")[0], numbers("<8d")] for _ in range(count(32))]
            values, indexes = samples()
            data["samples"] = [dict(source=sources[index], **sample) for index, sample in zip(indexes, values)]
            representation = "heightfield_v1"
        elif tag == 2 and mode == 2:
            data, representation = {}, "classified_empty_column"
        else:
            raise ValueError("精细地图BL6地表标签与模式不符")
        records.append([anchor, dict(representation=representation, data=data)] if mode == 2 else [anchor, data])
    if offset != len(view) or len(used) != len(sources):
        raise ValueError("精细地图BL6尾随数据或未引用来源")
    return mode, records, sources
