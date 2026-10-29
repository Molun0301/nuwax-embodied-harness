"""完整三角网格读取；不执行模型脚本，不跟随外部 COLLADA 资源。"""
from __future__ import annotations

from dataclasses import dataclass
import math
import struct
import warnings
import xml.etree.ElementTree as ET

import numpy as np


class RobotVisualError(ValueError):
    """显示资产失败；错误码用于控制台说明外包盒回退原因。"""

    def __init__(self, message: str, code: str = "invalid_robot_visual"):
        super().__init__(message)
        self.code = code


@dataclass
class MeshPart:
    vertices: np.ndarray
    indices: np.ndarray
    matrix: np.ndarray
    rgba: list | None = None


def xml_root(data: bytes):
    if b"<!DOCTYPE" in data.upper() or b"<!ENTITY" in data.upper():
        raise RobotVisualError("模型不支持文档类型或实体声明。")
    try:
        return ET.fromstring(data)
    except ET.ParseError as exc:
        raise RobotVisualError("模型不是有效 XML（可扩展标记语言）。") from exc


def numbers(text, size=None):
    try:
        values = np.asarray([float(v) for v in (text or "").split()], dtype=np.float64)
    except (ValueError, OverflowError) as exc:
        raise RobotVisualError("模型数值无效。") from exc
    if (size is not None and len(values) != size) or not np.isfinite(values).all():
        raise RobotVisualError("模型数值缺失或非有限。")
    return values


def color(text):
    result = numbers(text, 4)
    if np.any((result < 0) | (result > 1)):
        raise RobotVisualError("材质颜色必须位于 0 到 1。")
    return result.tolist()


def _indices(text):
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("error", DeprecationWarning)
            return np.fromstring(text or "", dtype=np.int64, sep=" ")
    except (ValueError, DeprecationWarning) as exc:
        raise RobotVisualError("DAE 索引必须全部为整数。") from exc


def stl_parts(data: bytes):
    if len(data) >= 84 and 84 + struct.unpack_from("<I", data, 80)[0] * 50 == len(data):
        dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
        vertices = np.frombuffer(data, dtype=dtype, offset=84)["vertices"].reshape(-1, 3).astype(np.float64)
    else:
        try:
            lines = data.decode("utf-8-sig").splitlines()
        except UnicodeError as exc:
            raise RobotVisualError("网格不是有效 STL（三角网格）文件。") from exc
        vertices = np.asarray([numbers(" ".join(parts[1:]), 3) for line in lines
                               if (parts := line.split()) and parts[0] == "vertex"], dtype=np.float64)
        facets = sum(line.strip() == "endfacet" for line in lines)
        meaningful = [line.strip() for line in lines if line.strip()]
        if not meaningful or not meaningful[0].startswith("solid") or not meaningful[-1].startswith("endsolid") or len(vertices) != facets * 3:
            raise RobotVisualError("文本 STL 的三角面结构不完整。")
    if not len(vertices) or not np.isfinite(vertices).all():
        raise RobotVisualError("网格为空或包含非有限坐标。")
    return [MeshPart(vertices, np.arange(len(vertices)).reshape(-1, 3), np.eye(4))]


def _local_ref(value):
    if not value or not value.startswith("#") or len(value) == 1:
        raise RobotVisualError("DAE（COLLADA 三维模型）不支持外部资源引用。", "robot_visual_external_reference")
    return value[1:]


def _triangulate(indices, vertices):
    """耳切法保留凹多边形边界；不把凹多边形扇形展开成错误外观。"""
    if len(indices) == 3:
        return [indices]
    if len(indices) < 3 or len(indices) > 4096:
        raise RobotVisualError("DAE 多边形顶点数无效。")
    pts = vertices[indices]
    normal = np.cross(pts, np.roll(pts, -1, axis=0)).sum(axis=0)
    pts = np.delete(pts, int(np.argmax(np.abs(normal))), axis=1)
    cross = lambda a, b: float(a[0]*b[1]-a[1]*b[0])
    area = sum(cross(a, b) for a, b in zip(pts, np.roll(pts, -1, axis=0)))
    if abs(area) < 1e-20:
        raise RobotVisualError("DAE 多边形退化。")
    sign = 1 if area > 0 else -1
    remaining = list(range(len(indices)))
    result = []
    while len(remaining) > 3:
        for j, b in enumerate(remaining):
            a, c = remaining[j-1], remaining[(j+1) % len(remaining)]
            if sign*cross(pts[b]-pts[a], pts[c]-pts[b]) <= 1e-15:
                continue
            if any(all(sign*cross(pts[v]-pts[u], pts[p]-pts[u]) >= -1e-15
                       for u, v in ((a, b), (b, c), (c, a)))
                   for p in remaining if p not in (a, b, c)):
                continue
            result.append([indices[a], indices[b], indices[c]])
            remaining.pop(j)
            break
        else:
            raise RobotVisualError("DAE 多边形无法完整三角化。")
    result.append([indices[i] for i in remaining])
    return result


def dae_parts(data: bytes):
    root = xml_root(data)
    for node in root.iter():
        node.tag = node.tag.rsplit("}", 1)[-1]
    if root.tag != "COLLADA":
        raise RobotVisualError("DAE 根元素无效。")
    # 外部引用统一拒绝；不尝试下载、打开纹理或访问其他文件。
    for node in root.iter():
        for key in ("url", "source", "target"):
            if key in node.attrib and node.tag not in ("channel",):
                _local_ref(node.get(key))
        if node.tag == "image":
            raise RobotVisualError("DAE 外部纹理不受支持，已回退外包。", "robot_visual_external_reference")
        if node.tag in ("controller", "instance_controller", "animation", "lookat", "skew"):
            raise RobotVisualError("DAE 包含未支持的变形、动画或坐标变换。", "unsupported_robot_visual")
    effects = {}
    for effect in root.findall("./library_effects/effect"):
        diffuse = effect.find(".//diffuse/color")
        if effect.find(".//texture") is not None:
            raise RobotVisualError("DAE 纹理材质不受支持。", "unsupported_robot_visual")
        effects[effect.get("id")] = color(diffuse.text) if diffuse is not None else None
    materials = {}
    for material in root.findall("./library_materials/material"):
        effect = material.find("instance_effect")
        materials[material.get("id")] = effects.get(_local_ref(effect.get("url"))) if effect is not None else None
    geometries = {}
    for geometry in root.findall("./library_geometries/geometry"):
        mesh = geometry.find("mesh")
        if mesh is None:
            raise RobotVisualError("DAE 几何不是三角网格。", "unsupported_robot_visual")
        sources = {}
        for source in mesh.findall("source"):
            array, accessor = source.find("float_array"), source.find("technique_common/accessor")
            if array is None or accessor is None:
                continue
            raw = numbers(array.text)
            stride, offset, count = int(accessor.get("stride", "1")), int(accessor.get("offset", "0")), int(accessor.get("count", "0"))
            if stride < 1 or offset < 0 or count < 0 or offset+stride*count > len(raw):
                raise RobotVisualError("DAE 顶点访问器越界。")
            sources[source.get("id")] = raw[offset:offset+stride*count].reshape(count, stride)
        vertex_sources = {}
        for node in mesh.findall("vertices"):
            pos = node.find("input[@semantic='POSITION']")
            if pos is not None:
                vertex_sources[node.get("id")] = _local_ref(pos.get("source"))
        parts = []
        for primitive in mesh:
            if primitive.tag in ("source", "vertices", "extra"):
                continue
            if primitive.tag not in ("triangles", "polylist", "polygons", "trifans", "tristrips"):
                raise RobotVisualError("DAE 包含未支持的网格图元：" + primitive.tag, "unsupported_robot_visual")
            inputs = primitive.findall("input")
            stride = max((int(i.get("offset", "0")) for i in inputs), default=-1)+1
            pos = next((i for i in inputs if i.get("semantic") in ("VERTEX", "POSITION")), None)
            if pos is None or stride < 1:
                raise RobotVisualError("DAE 三角面缺少位置输入。")
            source_id = _local_ref(pos.get("source"))
            if pos.get("semantic") == "VERTEX":
                source_id = vertex_sources.get(source_id)
            vertices = sources.get(source_id)
            if vertices is None or vertices.shape[1] < 3:
                raise RobotVisualError("DAE 位置源无效。")
            vertices = vertices[:, :3]
            offset = int(pos.get("offset", "0"))
            all_indices = []
            for p in primitive.findall("p"):
                raw = _indices(p.text)
                if len(raw) % stride:
                    raise RobotVisualError("DAE 三角面索引数量无效。")
                indices = raw.reshape(-1, stride)[:, offset]
                if len(indices) and (indices.min() < 0 or indices.max() >= len(vertices)):
                    raise RobotVisualError("DAE 三角面索引越界。")
                if primitive.tag == "triangles":
                    if len(indices) % 3:
                        raise RobotVisualError("DAE 三角面索引不完整。")
                    all_indices.extend(indices.reshape(-1, 3).tolist())
                elif primitive.tag == "polylist":
                    counts = _indices(primitive.findtext("vcount", ""))
                    if np.any(counts < 3) or counts.sum() != len(indices):
                        raise RobotVisualError("DAE 多边形索引不完整。")
                    at = 0
                    for count in counts:
                        all_indices.extend(_triangulate(indices[at:at+count].tolist(), vertices))
                        at += count
                elif primitive.tag == "tristrips":
                    all_indices.extend([[int(indices[i]), int(indices[i+1+i%2]), int(indices[i+2-i%2])]
                                        for i in range(len(indices)-2)])
                elif primitive.tag == "trifans":
                    all_indices.extend([[int(indices[0]), int(indices[i]), int(indices[i+1])]
                                        for i in range(1, len(indices)-1)])
                else:
                    all_indices.extend(_triangulate(indices.tolist(), vertices))
            if primitive.find("ph") is not None:
                raise RobotVisualError("DAE 带孔多边形不受支持。", "unsupported_robot_visual")
            if primitive.tag == "triangles" and int(primitive.get("count", str(len(all_indices)))) != len(all_indices):
                raise RobotVisualError("DAE 声明的三角面数量与实际索引不符。")
            if all_indices:
                parts.append((vertices, np.asarray(all_indices), primitive.get("material")))
        geometries[geometry.get("id")] = parts
    asset = np.eye(4)
    unit = root.find("asset/unit")
    meter = float(unit.get("meter", "1")) if unit is not None else 1.0
    if not math.isfinite(meter) or meter <= 0:
        raise RobotVisualError("DAE 长度单位无效。")
    up = root.findtext("asset/up_axis", "Y_UP").strip()
    if up == "Y_UP":
        asset[:3, :3] = ((1, 0, 0), (0, 0, -1), (0, 1, 0))
    elif up == "X_UP":
        asset[:3, :3] = ((0, 0, -1), (0, 1, 0), (1, 0, 0))
    elif up != "Z_UP":
        raise RobotVisualError("DAE 向上轴无效。")
    asset[:3, :3] *= meter
    result = []
    nodes = {node.get("id"): node for node in root.iter("node") if node.get("id")}

    def visit(node, parent, seen):
        if id(node) in seen or len(seen) > 128:
            raise RobotVisualError("DAE 场景节点循环或过深。")
        seen = seen | {id(node)}
        matrix = parent.copy()
        for transform in node:
            local = np.eye(4)
            if transform.tag == "matrix":
                # COLLADA 规范使用按行书写的矩阵，网络传输再转列主序。
                local = numbers(transform.text, 16).reshape(4, 4)
            elif transform.tag == "translate":
                local[:3, 3] = numbers(transform.text, 3)
            elif transform.tag == "scale":
                local[:3, :3] = np.diag(numbers(transform.text, 3))
            elif transform.tag == "rotate":
                value = numbers(transform.text, 4)
                norm = np.linalg.norm(value[:3])
                if norm == 0:
                    raise RobotVisualError("DAE 旋转轴为空。")
                axis = value[:3]/norm
                x, y, z = axis
                skew = np.array(((0, -z, y), (z, 0, -x), (-y, x, 0)))
                angle = math.radians(value[3])
                local[:3, :3] = math.cos(angle)*np.eye(3)+(1-math.cos(angle))*np.outer(axis, axis)+math.sin(angle)*skew
            else:
                continue
            matrix = matrix @ local
        if not np.isfinite(matrix).all() or not np.allclose(matrix[3], (0, 0, 0, 1)):
            raise RobotVisualError("DAE 场景变换无效。")
        for instance in node.findall("instance_geometry"):
            geometry_id = _local_ref(instance.get("url"))
            if geometry_id not in geometries:
                raise RobotVisualError("DAE 场景引用不存在的几何。")
            bound = {i.get("symbol"): materials.get(_local_ref(i.get("target")))
                     for i in instance.findall(".//instance_material")}
            for vertices, indices, material in geometries[geometry_id]:
                result.append(MeshPart(vertices, indices, matrix, bound.get(material, materials.get(material))))
        for child in node.findall("node"):
            visit(child, matrix, seen)
        for instance in node.findall("instance_node"):
            target = nodes.get(_local_ref(instance.get("url")))
            if target is None:
                raise RobotVisualError("DAE 场景引用不存在的节点。")
            visit(target, matrix, seen)

    instance = root.find("scene/instance_visual_scene")
    scenes = {scene.get("id"): scene for scene in root.findall("./library_visual_scenes/visual_scene")}
    if instance is not None:
        scene = scenes.get(_local_ref(instance.get("url")))
        if scene is None:
            raise RobotVisualError("DAE 缺少所引用的可视场景。")
        for node in scene.findall("node"):
            visit(node, asset, set())
    else:
        for parts in geometries.values():
            result.extend(MeshPart(v, i, asset, materials.get(m)) for v, i, m in parts)
    if not result:
        raise RobotVisualError("DAE 没有可显示的三角面。")
    return result


def primitive_parts(shape):
    """基础几何只细分其曲面；输入网格的所有三角面始终保留。"""
    tag = shape.tag
    if tag == "box":
        size = numbers(shape.get("size"), 3)
        if np.any(size <= 0):
            raise RobotVisualError("盒尺寸必须大于零。")
        vertices = np.array([(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])*size/2
        indices = np.array([(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
                            (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)])
    elif tag in ("cylinder", "sphere"):
        radius = numbers(shape.get("radius"), 1)[0]
        if radius <= 0:
            raise RobotVisualError("曲面半径必须大于零。")
        n = 32
        vertices, indices = [], []
        if tag == "cylinder":
            length = numbers(shape.get("length"), 1)[0]
            if length <= 0:
                raise RobotVisualError("圆柱长度必须大于零。")
            vertices = [(radius*math.cos(i*2*math.pi/n), radius*math.sin(i*2*math.pi/n), z*length/2)
                        for z in (-1, 1) for i in range(n)] + [(0, 0, -length/2), (0, 0, length/2)]
            for i in range(n):
                j = (i+1) % n
                indices.extend(((i, j, j+n), (i, j+n, i+n), (2*n, j, i), (2*n+1, i+n, j+n)))
        else:
            rings = 16
            vertices = [(radius*math.sin(j*math.pi/rings)*math.cos(i*2*math.pi/n),
                         radius*math.sin(j*math.pi/rings)*math.sin(i*2*math.pi/n), radius*math.cos(j*math.pi/rings))
                        for j in range(1, rings) for i in range(n)] + [(0, 0, radius), (0, 0, -radius)]
            for i in range(n):
                k = (i+1) % n
                indices.extend((((rings-1)*n, i, k), ((rings-1)*n+1, (rings-2)*n+k, (rings-2)*n+i)))
                for j in range(rings-2):
                    a, b, c, d = j*n+i, j*n+k, (j+1)*n+i, (j+1)*n+k
                    indices.extend(((a, c, d), (a, d, b)))
        vertices, indices = np.asarray(vertices), np.asarray(indices)
    else:
        raise RobotVisualError("不支持外观几何：" + tag, "unsupported_robot_visual")
    return [MeshPart(vertices, indices, np.eye(4))]
