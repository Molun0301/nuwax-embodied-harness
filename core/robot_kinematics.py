"""URDF 运动学与连杆凸包，供感知按实时关节角计算机器人自身在相机中的位置。

与 core/robot_geometry.py 共用同一份模型来源（NUWAX_ROBOT_MODEL_PATH /
NUWAX_ROBOT_MODEL_BASE_LINK）。robot_geometry 按静态参考姿态给导航外包；本模块
在每帧按实际关节角求各连杆在机体基准连杆下的位姿，不读取世界位姿。
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping, Optional, Tuple, Union

import numpy as np


class RobotKinematicsError(ValueError):
    """模型不可用于自体计算；code 进入感知状态，便于区分缺模型与模型非法。"""

    def __init__(self, message: str, code: str = "invalid_robot_kinematics") -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class KinematicJoint:
    """单个 URDF 关节；origin 为父连杆到关节零位的 4×4 齐次变换。"""

    name: str
    joint_type: str
    parent_link: str
    child_link: str
    origin: np.ndarray
    axis: Tuple[float, float, float] = (1.0, 0.0, 0.0)
    # (被模仿关节, 倍率, 偏移)；没有模仿约束时为 None。
    mimic: Optional[Tuple[str, float, float]] = None


@dataclass(frozen=True)
class LinkHull:
    """连杆碰撞几何的凸包顶点，已包含 collision origin，坐标在连杆自身坐标系。"""

    link_name: str
    points_local: np.ndarray
    # 外向单位法向量与平面截距，用于识别包含相机光心的外壳凸包。
    planes_local: Optional[np.ndarray] = None


@dataclass(frozen=True)
class RobotKinematicModel:
    """解析后的运动学树；joints 按父到子的拓扑顺序排列。"""

    source_path: str
    base_link: str
    root_link: str
    joints: Tuple[KinematicJoint, ...]
    hulls: Tuple[LinkHull, ...]
    actuated_joint_names: Tuple[str, ...]
    link_elements: Mapping
    joint_elements: Mapping

    def link_transforms(self, joint_positions: Mapping[str, float]) -> Dict[str, np.ndarray]:
        """返回每个连杆在 base_link 坐标系下的 4×4 位姿。

        joint_positions 必须包含全部 actuated_joint_names（模仿关节由约束计算）；
        缺失时抛出 RobotKinematicsError(code="joint_state_incomplete")，不按零位猜测。
        """
        missing = set(self.actuated_joint_names) - set(joint_positions)
        if missing:
            raise RobotKinematicsError("缺少关节状态：" + ",".join(sorted(missing)), "joint_state_incomplete")
        values, resolving = {}, set()
        by_name = {joint.name: joint for joint in self.joints}
        def value(name):
            if name in values:
                return values[name]
            if name in resolving:
                raise RobotKinematicsError("模仿关节存在循环")
            resolving.add(name)
            joint = by_name[name]
            if joint.joint_type == "fixed":
                result = 0.0
            elif joint.mimic:
                other, multiplier, offset = joint.mimic
                result = value(other) * multiplier + offset
            else:
                raw = joint_positions[name]
                if isinstance(raw, bool):
                    raise RobotKinematicsError("关节状态必须是有限数值", "joint_state_invalid")
                try:
                    result = float(raw)
                except (TypeError,ValueError,OverflowError) as exc:
                    raise RobotKinematicsError("关节状态必须是有限数值", "joint_state_invalid") from exc
            if not np.isfinite(result):
                raise RobotKinematicsError("关节状态必须是有限数值", "joint_state_invalid")
            resolving.remove(name)
            values[name] = result
            return result
        transforms = {self.root_link: np.eye(4)}
        for joint in self.joints:
            motion = np.eye(4)
            angle = value(joint.name)
            axis = np.asarray(joint.axis)
            if joint.joint_type == "prismatic":
                motion[:3, 3] = axis * angle
            elif joint.joint_type in ("revolute", "continuous"):
                x, y, z = axis
                skew = np.array(((0, -z, y), (z, 0, -x), (-y, x, 0)))
                c, sn = np.cos(angle), np.sin(angle)
                motion[:3, :3] = c * np.eye(3) + (1-c) * np.outer(axis, axis) + sn * skew
            transforms[joint.child_link] = transforms[joint.parent_link] @ joint.origin @ motion
        base = transforms[self.base_link]
        inverse = np.eye(4)
        inverse[:3, :3] = base[:3, :3].T
        inverse[:3, 3] = -base[:3, :3].T @ base[:3, 3]
        return {name: inverse @ matrix for name, matrix in transforms.items()}

    def hulls_in_base(self, joint_positions: Mapping[str, float]) -> Tuple[Tuple[str, np.ndarray], ...]:
        """各连杆凸包顶点在 base_link 坐标系下的坐标。"""

        transforms = self.link_transforms(joint_positions)
        result = []
        for hull in self.hulls:
            transform = transforms[hull.link_name]
            result.append((hull.link_name, hull.points_local @ transform[:3, :3].T + transform[:3, 3]))
        return tuple(result)


def load_robot_kinematic_model(
    model_path: Union[str, Path],
    *,
    base_link: str = "",
    max_hull_points: int = 64,
) -> RobotKinematicModel:
    """读取已展开的 URDF，生成运动学树和每个连杆不超过 max_hull_points 的凸包。

    只读取本地文件；网格支持与 robot_geometry 相同的 STL/OBJ，基本形状按外接点集近似。
    """
    from core import robot_geometry as geometry
    path = Path(model_path).expanduser().resolve()
    if not 8 <= max_hull_points <= 64:
        raise RobotKinematicsError("连杆凸包预算必须为8至64点")
    try:
        links, elements, edges, root_link, selected = geometry._read_urdf_description(path, base_link)
        joints, pending = [], [root_link]
        visited = {root_link}
        while pending:
            parent = pending.pop()
            for child, element in edges[parent]:
                if child in visited:
                    raise geometry.RobotGeometryError("模型关节树存在循环")
                visited.add(child)
                kind, name = element.get("type"), element.get("name")
                if kind not in ("fixed", "revolute", "continuous", "prismatic"):
                    raise geometry.RobotGeometryError("不支持的关节类型：" + str(kind), "unsupported_robot_geometry")
                rotation, translation = geometry._origin(element)
                origin = np.eye(4)
                origin[:3, :3], origin[:3, 3] = rotation, translation
                axis_node = element.find("axis")
                axis = np.asarray(geometry._vector(axis_node.get("xyz") if axis_node is not None else None,
                                                   (1., 0., 0.), "关节轴"))
                norm = np.linalg.norm(axis)
                if kind != "fixed" and (not np.isfinite(norm) or norm == 0):
                    raise geometry.RobotGeometryError("关节轴必须为有限非零向量：" + name)
                mimic_node = element.find("mimic")
                mimic = None
                if mimic_node is not None:
                    other = mimic_node.get("joint")
                    if other not in elements:
                        raise geometry.RobotGeometryError("模仿关节引用无效：" + name)
                    mimic = (other, geometry._number(mimic_node.get("multiplier", "1"), "关节倍率"),
                             geometry._number(mimic_node.get("offset", "0"), "关节偏移"))
                joints.append(KinematicJoint(name, kind, parent, child, origin, tuple(axis / norm) if kind != "fixed" else (1.,0.,0.), mimic))
                pending.append(child)
        if len(visited) != len(links):
            raise geometry.RobotGeometryError("模型包含断开的连杆或关节循环")
        # 加载时核对模仿链，不等到运行帧才发现错误。
        by_name = {joint.name: joint for joint in joints}
        for joint in joints:
            seen, current = set(), joint
            while current.mimic:
                if current.name in seen:
                    raise geometry.RobotGeometryError("模型模仿关节存在循环")
                seen.add(current.name)
                current = by_name[current.mimic[0]]
        hulls = []
        for name, link in links.items():
            points = []
            for collision in link.findall("collision"):
                node = collision.find("geometry")
                if node is None or len(node) != 1:
                    raise geometry.RobotGeometryError("模型碰撞几何描述缺失：" + name)
                shape = list(node)[0]
                if shape.tag == "mesh":
                    local = geometry._mesh_vertices(shape, path)
                elif shape.tag == "box":
                    size = geometry._vector(shape.get("size"), None, "盒尺寸")
                    if size is None or min(size) <= 0:
                        raise geometry.RobotGeometryError("模型盒尺寸必须完整且大于0")
                    local = [(x*size[0]/2, y*size[1]/2, z*size[2]/2)
                             for x in (-1,1) for y in (-1,1) for z in (-1,1)]
                elif shape.tag in ("sphere", "cylinder"):
                    radius = geometry._number(shape.get("radius"), "模型半径", positive=True)
                    if shape.tag == "sphere":
                        local = [(x*radius,y*radius,z*radius) for x in (-1,1) for y in (-1,1) for z in (-1,1)]
                    else:
                        length = geometry._number(shape.get("length"), "圆柱长度", positive=True)
                        angle = (np.arange(32)+0.5)*2*np.pi/32
                        ring = np.column_stack((np.cos(angle),np.sin(angle))) * radius/np.cos(np.pi/32)
                        local = [(x,y,z*length/2) for x,y in ring for z in (-1,1)]
                else:
                    raise geometry.RobotGeometryError("不支持模型几何：" + shape.tag, "unsupported_robot_geometry")
                rotation, translation = geometry._origin(collision)
                points.extend(np.asarray(local) @ np.asarray(rotation).T + translation)
            if points:
                vertices = _compress_hull(np.asarray(points), max_hull_points)
                vertices, planes = _convex_planes(vertices)
                hulls.append(LinkHull(name, vertices, planes))
        return RobotKinematicModel(str(path), selected, root_link, tuple(joints), tuple(hulls),
            tuple(j.name for j in joints if j.joint_type != "fixed" and j.mimic is None), links, elements)
    except geometry.RobotGeometryError as exc:
        raise RobotKinematicsError(str(exc), exc.code) from exc


def _compress_hull(points: np.ndarray, limit: int) -> np.ndarray:
    """用均匀方向的极值顶点压缩凸包；六个轴向极值始终保留，不依赖外部几何库。"""
    points = np.unique(points, axis=0)
    if not np.isfinite(points).all():
        raise RobotKinematicsError("模型碰撞几何产生非有限坐标")
    directions = [*np.eye(3), *-np.eye(3)]
    count = limit - len(directions)
    for i in range(count):
        z = 1 - 2*(i+0.5)/count
        radius = np.sqrt(1-z*z)
        angle = i*np.pi*(3-np.sqrt(5))
        directions.append((radius*np.cos(angle),radius*np.sin(angle),z))
    # 分方向求极值，避免为百万顶点网格临时创建巨大矩阵。
    selected = np.unique([np.argmax(points @ direction) for direction in directions])
    result = points[selected]
    result.setflags(write=False)
    return result


def _convex_planes(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """对至多64个支撑点构建增量凸包，去除面内点并返回外向支撑平面。"""
    if len(points)<4:
        return points,np.empty((0,4))
    scale=max(float(np.ptp(points,axis=0).max()),1e-12)
    tolerance=scale*1e-10
    a=0
    b=int(np.argmax(np.linalg.norm(points-points[a],axis=1)))
    line=points[b]-points[a]
    c=int(np.argmax(np.linalg.norm(np.cross(points-points[a],line),axis=1)))
    normal=np.cross(points[b]-points[a],points[c]-points[a])
    norm=float(np.linalg.norm(normal))
    if norm<=tolerance*scale:
        return points,np.empty((0,4))
    d=int(np.argmax(np.abs((points-points[a]) @ (normal/norm))))
    if abs(float((points[d]-points[a]) @ (normal/norm)))<=tolerance:
        return points,np.empty((0,4))
    interior=points[[a,b,c,d]].mean(axis=0)
    def face(i,j,k):
        n=np.cross(points[j]-points[i],points[k]-points[i])
        length=float(np.linalg.norm(n))
        if length<=tolerance*scale:
            return None
        n=n/length
        if float(n @ (interior-points[i]))>0:
            j,k=k,j
            n=-n
        return ((i,j,k),n,float(n @ points[i]))
    faces=[face(a,b,c),face(a,d,b),face(a,c,d),face(b,d,c)]
    for index in range(len(points)):
        if index in (a,b,c,d):
            continue
        visible=[i for i,(_ids,n,offset) in enumerate(faces) if float(n @ points[index])-offset>tolerance]
        if not visible:
            continue
        edges={}
        for i in visible:
            ids=faces[i][0]
            for x,y in ((ids[0],ids[1]),(ids[1],ids[2]),(ids[2],ids[0])):
                key=tuple(sorted((x,y)))
                if key in edges:
                    edges.pop(key)
                else:
                    edges[key]=(x,y)
        visible=set(visible)
        faces=[value for i,value in enumerate(faces) if i not in visible]
        for x,y in edges.values():
            value=face(x,y,index)
            if value is not None:
                faces.append(value)
    vertices=points[sorted({index for ids,_normal,_offset in faces for index in ids})]
    planes=np.asarray([list(normal)+[offset] for _ids,normal,offset in faces])
    vertices.setflags(write=False);planes.setflags(write=False)
    return vertices,planes
