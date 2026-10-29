"""机器人完整外观资产：安全读取、内容寻址缓存及量化索引二进制。

运动学与导航共用 robot_kinematics；外观独立读取 visual，不使用凸包替代网格。
所有网络矩阵为列主序。网格响应带 Content-Encoding: gzip，浏览器自动解压后，
按 manifest 的字节偏移读取小端 uint16 位置与 uint16/uint32 索引。
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import gzip
import hashlib
import json
import os
from pathlib import Path
import stat
import threading
import time
from typing import Mapping

import numpy as np

from core import robot_geometry as geometry
from core.robot_kinematics import RobotKinematicModel, RobotKinematicsError, load_robot_kinematic_model
from core.robot_visual_mesh import RobotVisualError, color, dae_parts, numbers, primitive_parts, stl_parts, xml_root

_FORMAT = "nuwax-robot-mesh-v1"
_MODEL_LIMIT = 8 * 1024 * 1024
_MESH_LIMIT = 64 * 1024 * 1024
_TOTAL_LIMIT = 256 * 1024 * 1024
_DEFAULT_COLOR = [0.68, 0.72, 0.78, 1.0]


@dataclass(frozen=True)
class RobotVisualAsset:
    asset_id: str
    manifest: dict
    mesh_bytes: bytes
    model: RobotKinematicModel
    reference_positions: dict
    reference_transforms: dict
    navigation_geometry: dict | None
    navigation_geometry_error: dict | None


def _resolve_resource(root: Path, filename: str, *, relative_to: Path | None = None):
    if not filename or "\0" in filename:
        raise RobotVisualError("外观网格路径为空或无效。")
    if filename.startswith("package://"):
        parts = filename[10:].split("/", 1)
        if len(parts) != 2:
            raise RobotVisualError("外观网格包路径无效。")
        candidate = root / parts[1] if parts[0] == root.name else root / parts[0] / parts[1]
    elif "://" in filename or filename.startswith("file:"):
        raise RobotVisualError("外观网格只允许模型目录内本地资源。", "robot_visual_path_forbidden")
    else:
        candidate = Path(filename).expanduser()
        if not candidate.is_absolute():
            candidate = (relative_to or root) / candidate
    try:
        resolved = candidate.resolve()
        resolved.relative_to(root)
    except (ValueError, OSError, RuntimeError) as exc:
        raise RobotVisualError("外观网格引用超出模型目录。", "robot_visual_path_forbidden") from exc
    return resolved


def _safe_read(root: Path, path: Path, limit: int):
    """逐层目录句柄打开，阻止检查之后的符号链接替换越过模型边界。"""
    relative = path.relative_to(root)
    directory = None
    try:
        directory = os.open("/", os.O_RDONLY | os.O_DIRECTORY)
        for part in root.parts[1:]:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=directory)
            os.close(directory)
            directory = child
        for part in relative.parts[:-1]:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=directory)
            os.close(directory)
            directory = child
        descriptor = os.open(relative.parts[-1], os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory)
        with os.fdopen(descriptor, "rb") as handle:
            info = os.fstat(handle.fileno())
            if not stat.S_ISREG(info.st_mode) or info.st_size > limit:
                raise RobotVisualError("模型资源不是普通文件或超过读取预算。")
            data = handle.read(limit + 1)
            if len(data) > limit:
                raise RobotVisualError("模型资源超过读取预算。")
            return data
    except OSError as exc:
        raise RobotVisualError("模型或网格缺失、不可读或包含不安全链接。", "robot_visual_resource_unreadable") from exc
    finally:
        if directory is not None:
            os.close(directory)


def _reference(model, raw):
    values = geometry._joint_positions(raw)
    unknown = set(values) - set(model.joint_elements)
    if unknown:
        raise RobotVisualError("参考姿态含未知关节：" + ",".join(sorted(unknown)))
    positions = {name: values.get(name, 0.0) for name in model.actuated_joint_names}
    computed = {}
    by_name = {joint.name: joint for joint in model.joints}

    def resolve(name):
        if name in computed:
            return computed[name]
        joint = by_name[name]
        if joint.mimic:
            source, multiplier, offset = joint.mimic
            value = resolve(source)*multiplier+offset
            if name in values and not np.isclose(values[name], value, rtol=1e-12, atol=1e-12):
                raise RobotVisualError("参考姿态与模仿关节约束冲突：" + name)
        else:
            value = positions.get(name, 0.0)
        element = model.joint_elements[name]
        limit = element.find("limit")
        if name in values and limit is not None and joint.joint_type != "continuous":
            lower, upper = float(limit.get("lower", "-inf")), float(limit.get("upper", "inf"))
            if not lower <= value <= upper:
                raise RobotVisualError("显式参考姿态超出关节限位：" + name)
        computed[name] = value
        return value

    for name in by_name:
        if name in values and by_name[name].joint_type == "fixed":
            raise RobotVisualError("固定关节不能配置参考位置：" + name)
        resolve(name)
    defaulted = sorted(set(model.actuated_joint_names) - set(values))
    source = "configured_with_zero_defaults" if values and defaulted else "configured" if values else "urdf_zero_default"
    description = ("按配置参考姿态显示" if source == "configured" else
                   "按配置参考姿态显示；未配置关节采用模型零位" if values else
                   "未配置参考关节姿态，采用模型零位；这不是实时关节状态")
    return positions, model.link_transforms(positions), source, defaulted, description


def _matrix_list(matrix):
    return matrix.reshape(-1, order="F").tolist()


def _read_model(base_dir, config):
    if not config.model_path:
        raise RobotVisualError("未配置机器人模型，显示导航外包。", "robot_visual_model_unconfigured")
    model_path = Path(config.model_path).expanduser()
    if not model_path.is_absolute():
        model_path = base_dir / model_path
    root = model_path.parent.resolve()
    path = _resolve_resource(root, str(model_path))
    data = _safe_read(root, path, _MODEL_LIMIT)
    tree = xml_root(data)
    if tree.tag != "robot":
        raise RobotVisualError("机器人模型必须是已展开的 URDF（机器人描述文件）。")
    return root, path, data, tree


class RobotVisualAssetStore:
    """每个网关实例的缓存；相关配置立即生效，文件内容默认每秒校验一次。

    哈希包含 URDF、所有选中网格的完整内容以及机器人外形配置。校验不依赖
    mtime，因此同大小、恢复时间戳的文件替换也会失效。失败绝不返回旧资产。
    """

    def __init__(self, base_dir=None, *, check_interval_s: float = 1.0):
        self.base_dir = Path(base_dir or Path.cwd()).resolve()
        self.check_interval_s = max(0.0, float(check_interval_s))
        self._lock = threading.Lock()
        self._config_key = None
        self._checked_at = 0.0
        self._asset = None
        self._error = None
        self._navigation_key = None
        self._navigation_checked_at = 0.0
        self._navigation = None
        self._navigation_error = None

    def get_asset(self, config: geometry.RobotGeometryConfig | Mapping):
        if not isinstance(config, geometry.RobotGeometryConfig):
            config = geometry.RobotGeometryConfig.from_values(config)
        key = json.dumps(asdict(config), sort_keys=True, separators=(",", ":"))
        with self._lock:
            now = time.monotonic()
            if key == self._config_key and now-self._checked_at < self.check_interval_s:
                if self._error is not None:
                    raise RobotVisualError(*self._error)
                if self._asset is not None:
                    return self._asset
            try:
                asset = self._read_asset(config, key)
            except (RobotVisualError, RobotKinematicsError, geometry.RobotGeometryError, OSError, ValueError, KeyError, IndexError, RecursionError) as exc:
                self._asset = None
                self._navigation_key = None
                self._config_key, self._checked_at = key, time.monotonic()
                self._error = (str(exc), getattr(exc, "code", "invalid_robot_visual"))
                if isinstance(exc, RobotVisualError):
                    raise
                raise RobotVisualError(*self._error) from exc
            self._asset, self._config_key, self._checked_at = asset, key, time.monotonic()
            self._error = None
            return asset

    def get_navigation_geometry(self, config: geometry.RobotGeometryConfig | Mapping):
        """缺外观网格时仍安全读取真实导航外包；不读取导航未采用的缺失外观。"""
        if not isinstance(config, geometry.RobotGeometryConfig):
            config = geometry.RobotGeometryConfig.from_values(config)
        key = json.dumps(asdict(config), sort_keys=True, separators=(",", ":"))
        with self._lock:
            now = time.monotonic()
            if self._asset is not None and key == self._config_key and now-self._checked_at < self.check_interval_s:
                if self._asset.navigation_geometry is not None:
                    return self._asset.navigation_geometry
                error = self._asset.navigation_geometry_error
                raise RobotVisualError(error["message"], error["code"])
            if key == self._navigation_key and now-self._navigation_checked_at < self.check_interval_s:
                if self._navigation_error is not None:
                    raise RobotVisualError(*self._navigation_error)
                return self._navigation
            try:
                root, path, data, tree = _read_model(self.base_dir, config)
                for shape in tree.findall(".//mesh"):
                    _resolve_resource(root, shape.get("filename", ""))
                kind = "collision" if any(link.findall("collision") for link in tree.findall("link")) else "visual"
                resources, size = {path: data}, len(data)
                for shape in tree.findall("./link/"+kind+"/geometry/mesh"):
                    resource = _resolve_resource(root, shape.get("filename", ""))
                    if resource not in resources:
                        resources[resource] = _safe_read(root, resource, _MESH_LIMIT)
                        size += len(resources[resource])
                        if size > _TOTAL_LIMIT:
                            raise RobotVisualError("机器人外观资源超过总读取预算。")
                navigation = geometry.resolve_robot_geometry(config, robot_name="", base_dir=self.base_dir,
                    resource_bytes=resources).to_runtime_dict()
                navigation.pop("model_path", None)
            except (RobotVisualError, geometry.RobotGeometryError, OSError, ValueError, RuntimeError) as exc:
                self._navigation = None
                self._navigation_key, self._navigation_checked_at = key, time.monotonic()
                self._navigation_error = (str(exc), getattr(exc, "code", "invalid_robot_visual"))
                raise RobotVisualError(*self._navigation_error) from exc
            self._navigation, self._navigation_key = navigation, key
            self._navigation_checked_at, self._navigation_error = time.monotonic(), None
            return navigation

    def _read_asset(self, config, config_key):
        root, path, model_data, tree = _read_model(self.base_dir, config)
        chosen = []
        resources = {}
        resource_bytes = len(model_data)
        # 未选中显示的 collision 同样会参与导航外包，先统一审计和内容寻址。
        for shape in tree.findall(".//mesh"):
            resource = _resolve_resource(root, shape.get("filename", ""))
            if resource not in resources:
                resources[resource] = _safe_read(root, resource, _MESH_LIMIT)
                resource_bytes += len(resources[resource])
                if resource_bytes > _TOTAL_LIMIT:
                    raise RobotVisualError("机器人外观资源超过总读取预算。")
        selected_resources = set()
        for link in tree.findall("link"):
            visuals = link.findall("visual")
            for visual in visuals or link.findall("collision"):
                node = visual.find("geometry")
                if node is None or len(node) != 1:
                    raise RobotVisualError("机器人外观几何缺失或无效。")
                shape = node[0]
                resource = None
                if shape.tag == "mesh":
                    resource = _resolve_resource(root, shape.get("filename", ""))
                    if resource.suffix.lower() not in (".stl", ".dae"):
                        raise RobotVisualError("外观网格只支持 STL 与 DAE。", "unsupported_robot_visual")
                    selected_resources.add(resource)
                chosen.append((link.get("name"), visual, shape, resource, not bool(visuals)))
        if not chosen:
            raise RobotVisualError("机器人模型没有可显示的外观或碰撞几何。", "robot_visual_empty")
        digest = hashlib.sha256((_FORMAT+config_key).encode())
        digest.update(model_data)
        for resource, data in sorted(resources.items()):
            name = str(resource.relative_to(root)).encode()
            digest.update(len(name).to_bytes(8, "little"))
            digest.update(name)
            digest.update(len(data).to_bytes(8, "little"))
            digest.update(data)
        asset_id = digest.hexdigest()
        if self._asset is not None and self._asset.asset_id == asset_id:
            return self._asset
        # 独立完整外观加载器负责所有网格；共享运动学不读取导航碰撞凸包。
        model = load_robot_kinematic_model(path, base_link=config.model_base_link, load_hulls=False, urdf_data=model_data)
        reference, transforms, pose_source, defaulted, description = _reference(model, config.model_joint_positions_json)
        global_materials = {node.get("name"): node for node in tree.findall("material")}
        parts_cache, encoded, geometries, visuals, buffer = {}, {}, [], [], bytearray()
        triangles = 0
        max_error = 0.0
        collision_links = set()
        for link, visual, shape, resource, collision in chosen:
            if collision:
                collision_links.add(link)
            material = visual.find("material")
            override = None
            if material is not None:
                if len(material) == 0:
                    material = global_materials.get(material.get("name"), material)
                rgba = material.find("color")
                override = color(rgba.get("rgba")) if rgba is not None else None
                if material.find("texture") is not None:
                    raise RobotVisualError("URDF 纹理材质不受支持。", "unsupported_robot_visual")
            rotation, translation = geometry._origin(visual)
            matrix = np.eye(4)
            matrix[:3, :3], matrix[:3, 3] = rotation, translation
            if resource is not None:
                scale = numbers(shape.get("scale", "1 1 1"), 3)
                if np.any(scale == 0):
                    raise RobotVisualError("外观网格缩放不能为零。")
                matrix[:3, :3] = matrix[:3, :3] @ np.diag(scale)
                cache_key = str(resource)
                if cache_key not in parts_cache:
                    parts_cache[cache_key] = stl_parts(resources[resource]) if resource.suffix.lower() == ".stl" else dae_parts(resources[resource])
            else:
                cache_key = str(sorted(shape.attrib.items()))+shape.tag
                if cache_key not in parts_cache:
                    parts_cache[cache_key] = primitive_parts(shape)
            for index, part in enumerate(parts_cache[cache_key]):
                part_key = (cache_key, index)
                if part_key not in encoded:
                    vertices = part.vertices
                    if not len(vertices) or not np.isfinite(vertices).all():
                        raise RobotVisualError("外观网格为空或包含非有限坐标。")
                    lower = vertices.min(axis=0)
                    scale = np.ptp(vertices, axis=0)/65535.0
                    quantized = np.rint((vertices-lower)/np.where(scale > 0, scale, 1)).astype("<u2")
                    unique, inverse = np.unique(quantized, axis=0, return_inverse=True)
                    indices = inverse[part.indices].reshape(-1)
                    index_type = "uint16" if len(unique) <= 65536 else "uint32"
                    index_data = indices.astype("<u2" if index_type == "uint16" else "<u4").tobytes()
                    while len(buffer) % 4:
                        buffer.append(0)
                    position_offset = len(buffer)
                    buffer.extend(unique.astype("<u2").tobytes())
                    while len(buffer) % 4:
                        buffer.append(0)
                    index_offset = len(buffer)
                    buffer.extend(index_data)
                    geometry_id = len(geometries)
                    encoded[part_key] = geometry_id
                    geometries.append(dict(id=geometry_id, vertex_count=len(unique), index_count=len(indices),
                        position_offset_m=lower.tolist(), position_scale_m=scale.tolist(),
                        position_byte_offset=position_offset, index_byte_offset=index_offset, index_type=index_type))
                geometry_id = encoded[part_key]
                transform = matrix @ part.matrix
                if not np.isfinite(transform).all():
                    raise RobotVisualError("外观几何变换产生非有限数值。")
                # 标准 URDF 单材质覆盖网格颜色；多材质导出保留 DAE 各面分组。
                rgba = override if override is not None and len(visual.findall("material")) == 1 else part.rgba
                visuals.append(dict(link=link, geometry=geometry_id,
                    rgba=rgba or override or _DEFAULT_COLOR.copy(), matrix=_matrix_list(transform)))
                triangles += len(part.indices)
                error = np.linalg.norm(np.abs(transform[:3, :3]) @ (np.asarray(geometries[geometry_id]["position_scale_m"])*0.5))
                max_error = max(max_error, float(error))
        payload = gzip.compress(bytes(buffer), compresslevel=6, mtime=0)
        manifest = dict(format=_FORMAT, asset_id=asset_id, base_link=model.base_link,
            links=list(model.link_elements), actuated_joint_names=list(model.actuated_joint_names),
            geometries=geometries, visuals=visuals,
            reference_transforms={name: _matrix_list(matrix) for name, matrix in transforms.items()},
            reference_pose_source=pose_source, reference_defaulted_joints=defaulted, reference_description=description,
            mesh_bytes=len(payload), raw_mesh_bytes=len(buffer),
            source_mesh_bytes=sum(len(resources[path]) for path in selected_resources),
            all_source_mesh_bytes=sum(map(len, resources.values())),
            triangle_count=triangles, unique_triangle_count=sum(g["index_count"]//3 for g in geometries),
            quantization_max_error_m=max_error, collision_fallback_links=sorted(collision_links))
        navigation, navigation_error = None, None
        try:
            navigation = geometry.resolve_robot_geometry(config, robot_name="", base_dir=self.base_dir,
                resource_bytes={path: model_data, **resources}).to_runtime_dict()
            navigation.pop("model_path", None)
        except geometry.RobotGeometryError as exc:
            navigation_error = {"code": exc.code, "message": str(exc)}
        return RobotVisualAsset(asset_id, manifest, payload, model, reference, transforms, navigation, navigation_error)
