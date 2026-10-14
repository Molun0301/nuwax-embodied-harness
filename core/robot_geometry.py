"""机器人静态外包解析；不启动驱动、不读取实时姿态，也不发送运动。"""
from __future__ import annotations

from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import struct
from typing import Dict, Mapping, Optional, Tuple
import xml.etree.ElementTree as ET

_GEOMETRY_PREFIX = "NUWAX_ROBOT_"
_DIMENSION_KEYS = tuple(_GEOMETRY_PREFIX + axis + "_M" for axis in ("LENGTH", "WIDTH", "HEIGHT"))
_CENTER_KEYS = tuple(_GEOMETRY_PREFIX + "CENTER_" + axis + "_M" for axis in "XYZ")
ROBOT_GEOMETRY_SETTING_KEYS = frozenset(_DIMENSION_KEYS + _CENTER_KEYS + (
    "NUWAX_ROBOT_MODEL_PATH", "NUWAX_ROBOT_MODEL_BASE_LINK", "NUWAX_ROBOT_MODEL_JOINT_POSITIONS_JSON",
))
_IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
_ZERO = (0.0, 0.0, 0.0)
_MAX_MODEL_BYTES = 8 * 1024 * 1024
_MAX_MESH_BYTES = 64 * 1024 * 1024


class RobotGeometryError(ValueError):
    """带稳定错误码的配置或模型错误。"""

    def __init__(self, message: str, code: str = "invalid_robot_geometry") -> None:
        super().__init__(message)
        self.code = code


def _number(value, label: str, *, positive: bool = False) -> float:
    if isinstance(value, bool):
        raise RobotGeometryError(label + "必须是有限数值。")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise RobotGeometryError(label + "必须是有限数值。") from exc
    if not math.isfinite(result) or (positive and result <= 0):
        raise RobotGeometryError(label + ("必须大于0且有限。" if positive else "必须是有限数值。"))
    return result


def _optional_number(value, label: str, *, positive: bool = False) -> Optional[float]:
    return None if value is None or str(value).strip() == "" else _number(value, label, positive=positive)


def _joint_positions(raw: str) -> Dict[str, float]:
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise RobotGeometryError("参考关节位置包含重复键：" + key)
            result[key] = value
        return result
    try:
        values = json.loads(raw or "{}", object_pairs_hook=unique)
    except (TypeError, ValueError) as exc:
        raise RobotGeometryError("参考关节位置必须是关节名到有限数字的JSON对象。") from exc
    if not isinstance(values, dict):
        raise RobotGeometryError("参考关节位置必须是JSON对象。")
    result = {}
    for name, value in values.items():
        if not isinstance(name, str) or not name.strip() or not isinstance(value, (int, float)):
            raise RobotGeometryError("参考关节名不能为空，位置必须为数字。")
        result[name] = _number(value, "参考关节" + name)
    return result


def normalize_robot_geometry_setting(key: str, value: str) -> str:
    """供现有设置保存入口复用的单键校验；不在保存时打开模型文件。"""
    text = str(value).strip()
    if key not in ROBOT_GEOMETRY_SETTING_KEYS or any(c in text for c in "\r\n\0"):
        raise RobotGeometryError("无效机器人外形设置。")
    if key in _DIMENSION_KEYS + _CENTER_KEYS:
        number = _optional_number(text, key, positive=key in _DIMENSION_KEYS)
        return "" if number is None else str(number)
    if key == "NUWAX_ROBOT_MODEL_JOINT_POSITIONS_JSON":
        return json.dumps(_joint_positions(text), ensure_ascii=False, sort_keys=True, separators=(",", ":")) if text else ""
    return text


@dataclass(frozen=True)
class RobotGeometryConfig:
    """外形用户设置；空尺寸或中心逐轴继承模型/兼容默认。"""

    model_path: str = ""
    model_base_link: str = ""
    model_joint_positions_json: str = ""
    length_m: Optional[float] = None
    width_m: Optional[float] = None
    height_m: Optional[float] = None
    center_x_m: Optional[float] = None
    center_y_m: Optional[float] = None
    center_z_m: Optional[float] = None
    legacy_nav_width_m: Optional[float] = None

    @classmethod
    def from_values(cls, values: Mapping[str, object]) -> "RobotGeometryConfig":
        model_path = str(values.get("NUWAX_ROBOT_MODEL_PATH") or "").strip()
        width = _optional_number(values.get("NUWAX_ROBOT_WIDTH_M"), "机器人宽度", positive=True)
        return cls(
            model_path=model_path,
            model_base_link=str(values.get("NUWAX_ROBOT_MODEL_BASE_LINK") or "").strip(),
            model_joint_positions_json=str(values.get("NUWAX_ROBOT_MODEL_JOINT_POSITIONS_JSON") or "").strip(),
            length_m=_optional_number(values.get("NUWAX_ROBOT_LENGTH_M"), "机器人长度", positive=True),
            width_m=width,
            height_m=_optional_number(values.get("NUWAX_ROBOT_HEIGHT_M"), "机器人高度", positive=True),
            center_x_m=_optional_number(values.get("NUWAX_ROBOT_CENTER_X_M"), "外包中心X"),
            center_y_m=_optional_number(values.get("NUWAX_ROBOT_CENTER_Y_M"), "外包中心Y"),
            center_z_m=_optional_number(values.get("NUWAX_ROBOT_CENTER_Z_M"), "外包中心Z"),
            legacy_nav_width_m=(
                _optional_number(values.get("NUWAX_NAV_ROBOT_WIDTH_M"), "兼容导航宽度", positive=True)
                if not model_path and width is None else None
            ),
        )


@dataclass(frozen=True)
class ResolvedRobotGeometry:
    """机体基准坐标下的静态凸外包；参考关节姿态不等于实时腿部姿态。"""

    footprint_xy_m: Tuple[Tuple[float, float], ...]
    z_min_m: float
    z_max_m: float
    source: str
    model_path: Optional[str] = None
    model_base_link: str = ""
    reference_joint_positions: Tuple[Tuple[str, float], ...] = ()
    warnings: Tuple[str, ...] = ()
    self_collision_primitives: Tuple[dict, ...] = ()
    self_collision_excluded_moving: int = 0
    self_collision_unsupported: int = 0

    @property
    def length_m(self) -> float:
        return max(p[0] for p in self.footprint_xy_m) - min(p[0] for p in self.footprint_xy_m)

    @property
    def width_m(self) -> float:
        return max(p[1] for p in self.footprint_xy_m) - min(p[1] for p in self.footprint_xy_m)

    @property
    def height_m(self) -> float:
        return self.z_max_m - self.z_min_m

    @property
    def center_xyz_m(self) -> Tuple[float, float, float]:
        return tuple((min(p[i] for p in self.footprint_xy_m) + max(p[i] for p in self.footprint_xy_m)) / 2 for i in (0, 1)) + ((self.z_min_m + self.z_max_m) / 2,)

    def to_runtime_dict(self) -> dict:
        """统一运行时物化结果；不读取或生成另一份环境配置。"""
        result = {"footprint_xy_m": [list(point) for point in self.footprint_xy_m],
                  "z_min_m": self.z_min_m, "z_max_m": self.z_max_m, "source": self.source}
        if self.model_path:
            result["model_path"] = self.model_path
        result["self_collision"] = {
            "model_base_link": self.model_base_link,
            "primitives": list(self.self_collision_primitives),
            "excluded_moving_collision_count": self.self_collision_excluded_moving,
            "unsupported_collision_count": self.self_collision_unsupported,
        }
        return result

    def to_runtime_json(self) -> str:
        return json.dumps(self.to_runtime_dict(), ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":"))


def _vector(text: Optional[str], default, label: str):
    if text is None:
        return default
    parts = text.split()
    if len(parts) != 3:
        raise RobotGeometryError(label + "必须包含3个数值。")
    return tuple(_number(p, label) for p in parts)


def _matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _apply(rotation, vector):
    return tuple(sum(rotation[i][j] * vector[j] for j in range(3)) for i in range(3))


def _compose(first, second):
    r1, t1 = first
    r2, t2 = second
    moved = _apply(r1, t2)
    return _matmul(r1, r2), tuple(t1[i] + moved[i] for i in range(3))


def _origin(element):
    origin = element.find("origin")
    if origin is None:
        return _IDENTITY, _ZERO
    xyz = _vector(origin.get("xyz"), _ZERO, "模型平移")
    roll, pitch, yaw = _vector(origin.get("rpy"), _ZERO, "模型欧拉角")
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
            (-sp, cp*sr, cp*cr)), xyz


def _hull(points):
    points = sorted(set((float(x), float(y)) for x, y in points))
    if len(points) < 3 or not all(math.isfinite(v) for point in points for v in point):
        raise RobotGeometryError("模型变换产生了不可表示的外形坐标。")
    span = max(max(p[i] for p in points) - min(p[i] for p in points) for i in (0, 1))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    def half(items):
        result = []
        for point in items:
            while len(result) >= 2 and cross(result[-2], result[-1], point) <= 0:
                result.pop()
            result.append(point)
        return result
    hull = tuple(half(points)[:-1] + half(reversed(points))[:-1])
    if len(hull) < 3 or not math.isfinite(span) or span <= 0:
        raise RobotGeometryError("模型没有有效二维外包面积。")
    # 浮点旋转可能留下几乎同位的顶点。保留有效外支撑线并轻微向外补偿，
    # 不用带容差的弹栈删除真实极值点，以免把长边错误切成斜边。
    supports = []
    for first, second in zip(hull, hull[1:] + hull[:1]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length = math.hypot(dx, dy)
        if length <= span * 1e-12:
            continue
        nx, ny = dy / length, -dx / length
        supports.append((math.atan2(ny, nx), nx, ny,
                         max(nx * first[0] + ny * first[1], nx * second[0] + ny * second[1]) + span * 1e-12))
    supports.sort()
    selected = []
    for support in supports:
        if selected and abs(support[1] * selected[-1][2] - support[2] * selected[-1][1]) <= 1e-12 and support[1] * selected[-1][1] + support[2] * selected[-1][2] > 0:
            continue
        selected.append(support)
    if len(selected) > 2 and abs(selected[-1][1] * selected[0][2] - selected[-1][2] * selected[0][1]) <= 1e-12 and selected[-1][1] * selected[0][1] + selected[-1][2] * selected[0][2] > 0:
        selected.pop()
    if len(selected) < 3:
        raise RobotGeometryError("模型外包退化为线段。")
    result = []
    for first, second in zip(selected, selected[1:] + selected[:1]):
        _, x, y, offset = first
        _, u, v, other = second
        determinant = x * v - y * u
        if determinant <= 1e-14:
            raise RobotGeometryError("模型外包支撑线退化。")
        result.append(((offset * v - y * other) / determinant,
                       (x * other - offset * u) / determinant))
    return tuple(result)


def _read_bounded(path: Path, limit: int, label: str) -> bytes:
    try:
        if not path.is_file():
            raise OSError("不是普通模型文件")
        with path.open("rb") as handle:
            data = handle.read(limit + 1)
    except OSError as exc:
        raise RobotGeometryError(label + "无法读取：" + str(path), "robot_geometry_model_unreadable") from exc
    if len(data) > limit:
        raise RobotGeometryError(label + "超过允许读取大小：" + str(path))
    return data


def _mesh_vertices(element, model_path: Path):
    filename = element.get("filename", "").strip()
    if filename.startswith("package://"):
        parts = filename[len("package://"):].split("/", 1)
        if len(parts) != 2:
            raise RobotGeometryError("无效模型包网格路径：" + filename)
        package = next((p for p in model_path.parents if p.name == parts[0]), None)
        if package is None:
            raise RobotGeometryError("无法在模型祖先目录定位网格包：" + filename)
        path = package / parts[1]
    elif "://" in filename:
        raise RobotGeometryError("仅支持本地相对/绝对网格路径和可定位的模型包路径：" + filename)
    else:
        path = Path(filename).expanduser()
        if not path.is_absolute():
            path = model_path.parent / path
    if not filename:
        raise RobotGeometryError("模型网格缺少filename。")
    suffix = path.suffix.lower()
    if suffix not in (".stl", ".obj"):
        raise RobotGeometryError("不支持该网格格式，请使用STL/OBJ或手填外包：" + str(path), "unsupported_robot_geometry")
    data = _read_bounded(path, _MAX_MESH_BYTES, "网格")
    scale = _vector(element.get("scale"), (1.0, 1.0, 1.0), "网格缩放")
    if any(value == 0 for value in scale):
        raise RobotGeometryError("网格缩放不能为0。")
    points = []
    if suffix == ".stl" and len(data) >= 84 and 84 + struct.unpack_from("<I", data, 80)[0] * 50 == len(data):
        for offset in range(84, len(data), 50):
            points.extend(struct.unpack_from("<fff", data, offset + 12 + vertex * 12) for vertex in range(3))
    else:
        try:
            lines = data.decode("utf-8-sig").splitlines()
        except UnicodeError as exc:
            raise RobotGeometryError("网格不是有效文本或二进制STL。") from exc
        keyword = "v" if suffix == ".obj" else "vertex"
        for line in lines:
            parts = line.split()
            if not parts or parts[0] != keyword:
                continue
            if len(parts) < 4:
                raise RobotGeometryError("网格顶点不完整。")
            point = [_number(v, "网格顶点") for v in parts[1:4]]
            if suffix == ".obj" and len(parts) == 5:
                divisor = _number(parts[4], "网格齐次坐标")
                if divisor == 0:
                    raise RobotGeometryError("网格齐次坐标不能为0。")
                point = [v / divisor for v in point]
            points.append(point)
        if suffix == ".stl":
            meaningful = [line.strip().lower() for line in lines if line.strip()]
            facets = sum(line == "endfacet" for line in meaningful)
            if not meaningful or not meaningful[0].startswith("solid") or not meaningful[-1].startswith("endsolid") or facets * 3 != len(points):
                raise RobotGeometryError("文本STL的三角面结构不完整。")
    if not points or len(points) > 1_000_000:
        raise RobotGeometryError("网格没有顶点或顶点数超过读取预算。")
    return [tuple(_number(point[i], "网格顶点") * scale[i] for i in range(3)) for point in points]


def _geometry_points(geometry, transform, model_path: Path):
    children = list(geometry)
    if len(children) != 1:
        raise RobotGeometryError("每个碰撞/显示几何必须恰好包含一种形状。")
    shape = children[0]
    rotation, center = transform
    if shape.tag == "box":
        size = _vector(shape.get("size"), None, "盒尺寸")
        if size is None or any(value <= 0 for value in size):
            raise RobotGeometryError("模型盒尺寸必须完整且大于0。")
        local = [(x*size[0]/2, y*size[1]/2, z*size[2]/2) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    elif shape.tag == "mesh":
        local = _mesh_vertices(shape, model_path)
    elif shape.tag in ("sphere", "cylinder"):
        radius = _number(shape.get("radius"), "模型半径", positive=True)
        length = _number(shape.get("length"), "圆柱长度", positive=True) if shape.tag == "cylinder" else 0.0
        # 圆形投影使用32个支撑半平面的外接多边形，不把内接采样当作完整轮廓。
        normals = [(math.cos(i*math.tau/32), math.sin(i*math.tau/32)) for i in range(32)]
        def support(normal):
            if shape.tag == "sphere":
                return radius
            local_normal = [sum(rotation[i][j]*normal[i] for i in range(3)) for j in range(3)]
            return radius*math.hypot(local_normal[0], local_normal[1])+length/2*abs(local_normal[2])
        offsets = [center[0]*x+center[1]*y+support((x,y,0)) for x,y in normals]
        xy = []
        for i, (x,y) in enumerate(normals):
            j = (i+1) % len(normals)
            u,v = normals[j]
            det = x*v-y*u
            xy.append(((offsets[i]*v-y*offsets[j])/det, (x*offsets[j]-offsets[i]*u)/det))
        extent = support((0,0,1))
        return xy, center[2]-extent, center[2]+extent
    else:
        raise RobotGeometryError("不支持模型几何：" + str(shape.tag), "unsupported_robot_geometry")
    points = []
    for point in local:
        rotated = _apply(rotation, point)
        points.append(tuple(rotated[i]+center[i] for i in range(3)))
    return [(p[0],p[1]) for p in points], min(p[2] for p in points), max(p[2] for p in points)


def _parse_urdf(path: Path, base_link: str, positions: Dict[str, float], *, source_prefix: str):
    data = _read_bounded(path, _MAX_MODEL_BYTES, "机器人模型")
    if b"<!DOCTYPE" in data.upper() or b"<!ENTITY" in data.upper():
        raise RobotGeometryError("机器人模型不支持文档类型或实体声明。")
    try:
        root = ET.fromstring(data)
    except ET.ParseError as exc:
        raise RobotGeometryError("机器人模型不是有效XML。") from exc
    if root.tag != "robot" or any(isinstance(node.tag, str) and node.tag.startswith("{") for node in root.iter()):
        raise RobotGeometryError("模型必须为已展开的URDF，不执行Xacro或其他脚本。")
    links = {link.get("name"): link for link in root.findall("link")}
    if not links or None in links or "" in links or len(links) != len(root.findall("link")) or len(links) > 4096:
        raise RobotGeometryError("模型连杆名称缺失、重复或数量无效。")
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    if None in joints or "" in joints or len(joints) != len(root.findall("joint")):
        raise RobotGeometryError("模型关节名称缺失或重复。")
    unknown = set(positions) - set(joints)
    if unknown:
        raise RobotGeometryError("参考姿态包含未知关节：" + ",".join(sorted(unknown)))
    children = {}
    edges = {name: [] for name in links}
    for name, joint in joints.items():
        parent, child = joint.find("parent"), joint.find("child")
        parent = parent.get("link") if parent is not None else None
        child = child.get("link") if child is not None else None
        if parent not in links or child not in links or child in children:
            raise RobotGeometryError("关节存在无效连杆引用或重复父连杆：" + name)
        children[child] = parent
        edges[parent].append((child, joint))
    roots = set(links) - set(children)
    if len(roots) != 1:
        raise RobotGeometryError("机器人模型必须是一棵连通的关节树。")
    root_link = next(iter(roots))
    selected = base_link or next((name for name in ("base_link", "base", "trunk") if name in links), root_link)
    if selected not in links:
        raise RobotGeometryError("模型不存在指定机体基准连杆：" + selected)
    values, warnings, resolving = {}, [], set()
    def joint_value(name):
        if name in values:
            return values[name]
        if name in resolving:
            raise RobotGeometryError("模型模仿关节存在循环。")
        resolving.add(name)
        joint = joints[name]
        mimic = joint.find("mimic")
        if len(resolving) > 128:
            raise RobotGeometryError("模型模仿关节链超过解析深度预算。")
        if mimic is not None:
            other = mimic.get("joint")
            if other not in joints:
                raise RobotGeometryError("模仿关节引用无效：" + name)
            value = joint_value(other)*_number(mimic.get("multiplier", "1"), "关节倍率")+_number(mimic.get("offset", "0"), "关节偏移")
            if name in positions and not math.isclose(positions[name], value, rel_tol=1e-12, abs_tol=1e-12):
                raise RobotGeometryError("显式参考位置与模仿关节约束冲突：" + name)
        elif name in positions:
            value = positions[name]
        else:
            value = 0.0
        value = _number(value, "关节参考位置")
        limit = joint.find("limit")
        if limit is not None and joint.get("type") != "continuous":
            lower = _number(limit.get("lower", "-inf"), "关节下限") if limit.get("lower") is not None else -math.inf
            upper = _number(limit.get("upper", "inf"), "关节上限") if limit.get("upper") is not None else math.inf
            if lower > upper:
                raise RobotGeometryError("关节上下限颠倒：" + name)
            if not lower <= value <= upper:
                if name in positions:
                    raise RobotGeometryError("显式参考位置超出关节限位：" + name)
                warnings.append("关节" + name + "的模型零/派生参考位超出运动限位；此结果不是实际站姿。")
        resolving.remove(name)
        values[name] = value
        return value
    transforms = {root_link: (_IDENTITY, _ZERO)}
    pending = [root_link]
    moving = []
    while pending:
        parent = pending.pop()
        for child, joint in edges[parent]:
            if child in transforms:
                raise RobotGeometryError("模型关节树存在循环。")
            name, kind = joint.get("name"), joint.get("type")
            motion = (_IDENTITY, _ZERO)
            if kind == "fixed":
                if name in positions:
                    raise RobotGeometryError("固定关节不能配置参考位置：" + name)
            elif kind in ("revolute", "continuous", "prismatic"):
                moving.append(name)
                value = joint_value(name)
                axis_node = joint.find("axis")
                axis = _vector(axis_node.get("xyz") if axis_node is not None else None, (1.0,0.0,0.0), "关节轴")
                norm = math.hypot(*axis)
                if not math.isfinite(norm) or norm == 0:
                    raise RobotGeometryError("关节轴必须为有限非零向量：" + name)
                axis = tuple(v/norm for v in axis)
                if kind == "prismatic":
                    motion = _IDENTITY, tuple(v*value for v in axis)
                else:
                    x,y,z = axis
                    c,s,k = math.cos(value),math.sin(value),1-math.cos(value)
                    motion = ((c+x*x*k,x*y*k-z*s,x*z*k+y*s),(y*x*k+z*s,c+y*y*k,y*z*k-x*s),(z*x*k-y*s,z*y*k+x*s,c+z*z*k)), _ZERO
            else:
                raise RobotGeometryError("不支持该关节类型的静态标量参考姿态：" + str(kind), "unsupported_robot_geometry")
            transforms[child] = _compose(transforms[parent], _compose(_origin(joint), motion))
            pending.append(child)
    if len(transforms) != len(links):
        raise RobotGeometryError("模型包含断开的连杆或关节循环。")
    base_r, base_t = transforms[selected]
    inverse_r = tuple(tuple(base_r[j][i] for j in range(3)) for i in range(3))
    inverse = inverse_r, _apply(inverse_r, tuple(-v for v in base_t))
    kind = "collision" if any(link.findall("collision") for link in links.values()) else "visual"
    if kind == "visual":
        warnings.append("模型没有碰撞几何，使用显示几何作为明确声明的静态外包后备。")
    xy, lows, highs = [], [], []
    for name, link in links.items():
        for item in link.findall(kind):
            geometry = item.find("geometry")
            if geometry is None:
                raise RobotGeometryError("模型几何描述缺失：" + name)
            points, lo, hi = _geometry_points(geometry, _compose(inverse, _compose(transforms[name], _origin(item))), path)
            xy.extend(points); lows.append(lo); highs.append(hi)
    if not xy:
        raise RobotGeometryError("模型没有可读取的碰撞或显示外形。", "robot_geometry_missing")
    # 自体实体只使用与 body 刚性连接的 collision，不能使用导航凸外包或关节零位。
    fixed_edges = {name: [] for name in links}
    for joint in joints.values():
        if joint.get("type") == "fixed":
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            fixed_edges[parent].append(child)
            fixed_edges[child].append(parent)
    fixed_links, pending = {selected}, [selected]
    while pending:
        for neighbour in fixed_edges[pending.pop()]:
            if neighbour not in fixed_links:
                fixed_links.add(neighbour)
                pending.append(neighbour)
    primitives, excluded_moving, unsupported = [], 0, 0
    for name, link in links.items():
        for item in link.findall("collision"):
            if name not in fixed_links:
                excluded_moving += 1
                continue
            geometry = item.find("geometry")
            shape = list(geometry)[0] if geometry is not None and len(geometry) == 1 else None
            if shape is None or shape.tag not in ("box", "sphere", "cylinder"):
                unsupported += 1
                continue
            rotation, translation = _compose(inverse, _compose(transforms[name], _origin(item)))
            primitive = {"link_name": name, "shape": shape.tag,
                         "translation_m": list(translation),
                         "rotation_row_major": [v for row in rotation for v in row]}
            if shape.tag == "box":
                primitive["size_m"] = list(_vector(shape.get("size"), None, "盒尺寸"))
            else:
                primitive["radius_m"] = _number(shape.get("radius"), "模型半径", positive=True)
                if shape.tag == "cylinder":
                    primitive["length_m"] = _number(shape.get("length"), "圆柱长度", positive=True)
            primitives.append(primitive)
    if moving:
        warnings.append("活动关节使用静态参考姿态，未填写的关节采用模型零位；不表示实时腿部姿态。")
    return ResolvedRobotGeometry(_hull(xy), min(lows), max(highs), source_prefix + "_" + kind,
                                 str(path), selected, tuple(sorted(values.items())), tuple(warnings),
                                 tuple(primitives), excluded_moving, unsupported)


def _box(length, width, height, center, source):
    x,y,z = center
    return ResolvedRobotGeometry(((x-length/2,y-width/2),(x+length/2,y-width/2),
                                  (x+length/2,y+width/2),(x-length/2,y+width/2)), z-height/2, z+height/2, source)


def resolve_robot_geometry(config: RobotGeometryConfig, *, robot_name: str, base_dir) -> ResolvedRobotGeometry:
    """按完整手填、指定模型、驱动默认模型、通用兼容外包解析；不启动设备。"""
    dimensions = tuple(_optional_number(getattr(config, name), name, positive=True) for name in ("length_m","width_m","height_m"))
    centers = tuple(_optional_number(getattr(config, name), name) for name in ("center_x_m","center_y_m","center_z_m"))
    if all(value is not None for value in dimensions):
        result = _box(*dimensions, tuple(0.0 if value is None else value for value in centers), "manual")
    else:
        directory = Path(base_dir).expanduser().resolve()
        name = str(robot_name or "").strip().lower()
        if name == "unitree_go2":
            name = "go2"
        explicit = str(config.model_path or "").strip()
        if explicit:
            if "://" in explicit:
                raise RobotGeometryError("模型路径必须是本地文件，不自动下载远程模型。")
            model = Path(explicit).expanduser()
            if not model.is_absolute():
                model = directory / model
        else:
            model = directory / "drivers" / name / "robot_geometry.urdf" if name and name.replace("_", "").isalnum() else None
            if model is not None and not model.is_file():
                model = None
        if model is not None:
            result = _parse_urdf(model.resolve(), config.model_base_link, _joint_positions(config.model_joint_positions_json),
                                 source_prefix="urdf" if explicit else "robot_default_urdf")
        else:
            # 通用回退沿原0.25米外包半宽与0.6米高度，来源明确，不伪称真实模型。
            result = replace(_box(0.5,0.5,0.6,_ZERO,"legacy_default"), warnings=("当前机器人没有外形模型，使用兼容默认外包；可配置模型或手填真实尺寸。",))
        old_sizes = (result.length_m,result.width_m,result.height_m)
        old_center = result.center_xyz_m
        effective = list(dimensions)
        legacy = not explicit and effective[1] is None and config.legacy_nav_width_m is not None
        if legacy:
            effective[1] = _number(config.legacy_nav_width_m, "兼容导航宽度", positive=True)
        sizes = tuple(old_sizes[i] if effective[i] is None else effective[i] for i in range(3))
        center = tuple(old_center[i] if centers[i] is None else centers[i] for i in range(3))
        if any(v <= 0 or not math.isfinite(v) for v in sizes + old_sizes[:2]):
            raise RobotGeometryError("模型或手填外包必须具有正的有限长宽高。")
        points = tuple(tuple(center[i]+(point[i]-old_center[i])*sizes[i]/old_sizes[i] for i in range(2)) for point in result.footprint_xy_m)
        source = result.source
        if any(v is not None for v in dimensions + centers):
            source += "+manual_override"
        if legacy:
            source += "+legacy_nav_width"
        result = replace(result, footprint_xy_m=points, z_min_m=center[2]-sizes[2]/2,
                         z_max_m=center[2]+sizes[2]/2, source=source)
    result = replace(result, footprint_xy_m=_hull(result.footprint_xy_m))
    points = result.footprint_xy_m
    area_twice = sum(a[0] * b[1] - a[1] * b[0] for a, b in zip(points, points[1:] + points[:1]))
    if not math.isfinite(area_twice) or area_twice <= 1e-12:
        raise RobotGeometryError("机器人二维外包面积或中心偏移不足以可靠表示。")
    if not all(math.isfinite(v) for point in result.footprint_xy_m for v in point) or not all(math.isfinite(v) for v in (result.z_min_m,result.z_max_m)) or result.z_min_m >= result.z_max_m:
        raise RobotGeometryError("解析后的机器人外包无效或超出数值范围。")
    return result
