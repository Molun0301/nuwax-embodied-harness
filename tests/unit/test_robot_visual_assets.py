"""完整外观转换、真实坐标变换、缓存失效与模型边界保护。"""
import gzip
import json
import math
import os
from pathlib import Path
import struct
from unittest.mock import patch

import numpy as np
import pytest

from core.robot_geometry import RobotGeometryConfig
from core.robot_kinematics import load_robot_kinematic_model
from core.robot_visual import RobotVisualAssetStore, RobotVisualError


def stl(path, points):
    data = bytearray(b"test".ljust(80, b"\0"))
    data.extend(struct.pack("<I", len(points)//3))
    for triangle in np.asarray(points).reshape(-1, 9):
        data.extend(struct.pack("<12fH", 0, 0, 1, *triangle, 0))
    path.write_bytes(data)


def model(tmp_path, content, joints="", extra_links="", materials=""):
    path = tmp_path / "robot.urdf"
    path.write_text(f'<robot name="test">{materials}<link name="base">{content}</link>{extra_links}{joints}</robot>')
    return path


def load(path, **kwargs):
    return RobotVisualAssetStore(check_interval_s=0).get_asset(RobotGeometryConfig(model_path=str(path), **kwargs))


def points(asset, index=0):
    visual = asset.manifest["visuals"][index]
    g = asset.manifest["geometries"][visual["geometry"]]
    raw = gzip.decompress(asset.mesh_bytes)
    q = np.frombuffer(raw, dtype="<u2", count=g["vertex_count"]*3, offset=g["position_byte_offset"]).reshape(-1, 3)
    v = np.asarray(g["position_offset_m"])+q*np.asarray(g["position_scale_m"])
    indices = np.frombuffer(raw, dtype="<u2" if g["index_type"] == "uint16" else "<u4",
                            count=g["index_count"], offset=g["index_byte_offset"])
    matrix = np.asarray(visual["matrix"]).reshape(4, 4, order="F")
    return v[indices] @ matrix[:3, :3].T + matrix[:3, 3]


def dae(*, scene='', up='Z_UP', unit='1', external='', polygons=False):
    primitive = ('<polylist count="1"><input semantic="VERTEX" source="#vertices" offset="0"/>'
                 '<vcount>5</vcount><p>0 1 2 3 4</p></polylist>' if polygons else
                 '<triangles count="2" material="paint"><input semantic="NORMAL" source="#normals" offset="0"/>'
                 '<input semantic="VERTEX" source="#vertices" offset="1"/><p>0 0 0 1 0 2 0 0 0 2 0 4</p></triangles>')
    return f'''<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema"><asset><unit meter="{unit}"/><up_axis>{up}</up_axis></asset>
      <library_effects><effect id="red"><profile_COMMON><technique><lambert><diffuse><color>1 0.2 0.1 0.8</color></diffuse></lambert></technique></profile_COMMON></effect></library_effects>
      <library_materials><material id="red-material"><instance_effect url="#red"/></material></library_materials>
      <library_geometries><geometry id="mesh"><mesh>
      <source id="positions"><float_array id="arr" count="15">0 0 0 2 0 0 2 2 0 1 1 0 0 2 0</float_array><technique_common><accessor source="#arr" stride="3" count="5"/></technique_common></source>
      <source id="normals"><float_array id="narr" count="3">0 0 1</float_array><technique_common><accessor source="#narr" stride="3" count="1"/></technique_common></source>
      <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>{primitive}</mesh></geometry></library_geometries>
      <library_visual_scenes><visual_scene id="scene"><node id="parent">{scene}<node id="child"><translate>1 2 3</translate><instance_geometry url="#mesh"><bind_material><technique_common><instance_material symbol="paint" target="#red-material"/></technique_common></bind_material></instance_geometry></node></node></visual_scene></library_visual_scenes>
      <scene><instance_visual_scene url="#scene"/></scene>{external}</COLLADA>'''


def test_stl_preserves_all_triangles_quantization_and_shared_vertices(tmp_path):
    expected = np.array([[0, 0, 0], [1.234567, 0, 0], [0, 1, 0],
                         [0, 0, 0], [0, 1, 0], [0.123456, 0.456789, 0.25]])
    stl(tmp_path / "part.stl", expected)
    path = model(tmp_path, '<visual><geometry><mesh filename="part.stl"/></geometry></visual>')
    asset = load(path)
    assert asset.manifest["triangle_count"] == 2
    assert asset.manifest["geometries"][0]["vertex_count"] == 4
    assert asset.manifest["geometries"][0]["index_count"] == 6
    error = np.linalg.norm(points(asset)-expected, axis=1).max()
    assert error <= asset.manifest["quantization_max_error_m"]+1e-12
    assert error < 2e-5
    assert asset.manifest["raw_mesh_bytes"] == len(gzip.decompress(asset.mesh_bytes))
    assert asset.manifest["reference_pose_source"] == "urdf_zero_default"
    assert "不是实时" in asset.manifest["reference_description"]


def test_ascii_stl_urdf_origin_scale_material(tmp_path):
    (tmp_path / "part.stl").write_text('solid x\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid x\n')
    path = model(tmp_path, '<visual><origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/><geometry><mesh filename="part.stl" scale="2 3 4"/></geometry><material name="blue"/></visual>',
                 materials='<material name="blue"><color rgba="0 0 1 0.5"/></material>')
    asset = load(path)
    np.testing.assert_allclose(points(asset), [[1, 2, 3], [1, 4, 3], [-2, 2, 3]], atol=1e-12)
    assert asset.manifest["visuals"][0]["rgba"] == [0, 0, 1, 0.5]


def test_dae_interleaved_indices_nested_matrix_and_material(tmp_path):
    (tmp_path / "part.dae").write_text(dae(scene='<matrix>0 -1 0 5 1 0 0 7 0 0 1 9 0 0 0 1</matrix>'))
    path = model(tmp_path, '<visual><origin xyz="1 0 0"/><geometry><mesh filename="part.dae" scale="2 2 2"/></geometry></visual>')
    asset = load(path)
    assert asset.manifest["visuals"][0]["rgba"] == [1, 0.2, 0.1, 0.8]
    np.testing.assert_allclose(points(asset)[0], [7, 16, 24], atol=1e-10)
    np.testing.assert_allclose(points(asset)[1], [7, 20, 24], atol=1e-10)
    assert asset.manifest["triangle_count"] == 2


def test_dae_concave_polygon_unit_up_axis(tmp_path):
    (tmp_path / "part.dae").write_text(dae(polygons=True, unit="0.001", up="Y_UP"))
    path = model(tmp_path, '<visual><geometry><mesh filename="part.dae"/></geometry></visual>')
    asset = load(path)
    p = points(asset).reshape(-1, 3, 3)
    area = np.linalg.norm(np.cross(p[:, 1]-p[:, 0], p[:, 2]-p[:, 0]), axis=1).sum()/2
    assert asset.manifest["triangle_count"] == 3
    assert area == pytest.approx(3e-6, abs=1e-10)
    assert p[:, :, 1].min() == pytest.approx(-0.003)
    assert p[:, :, 2].min() == pytest.approx(0.002)


def test_visual_selected_and_collision_fallback(tmp_path):
    path = model(tmp_path, '<visual><geometry><box size="1 2 3"/></geometry></visual><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>',
        joints='<joint name="fixed" type="fixed"><parent link="base"/><child link="child"/><origin xyz="2 0 0"/></joint>',
        extra_links='<link name="child"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>')
    asset = load(path)
    assert len(asset.manifest["visuals"]) == 2
    assert asset.manifest["collision_fallback_links"] == ["child"]
    assert asset.model.hulls == ()
    np.testing.assert_allclose(asset.reference_transforms["child"][:3, 3], [2, 0, 0])


def test_missing_visual_mesh_fails_instead_of_silently_dropping_part(tmp_path):
    path = model(tmp_path, '<visual><geometry><mesh filename="missing.stl"/></geometry></visual><collision><geometry><box size="1 1 1"/></geometry></collision>')
    with pytest.raises(RobotVisualError, match="缺失") as error:
        load(path)
    assert error.value.code == "robot_visual_resource_unreadable"


def test_cache_hashes_mesh_content_even_when_size_and_mtime_unchanged(tmp_path):
    mesh = tmp_path / "part.stl"
    stl(mesh, [[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    path = model(tmp_path, '<visual><geometry><mesh filename="part.stl"/></geometry></visual>')
    config = RobotGeometryConfig(model_path=str(path))
    store = RobotVisualAssetStore(check_interval_s=0)
    first = store.get_asset(config)
    assert store.get_asset(config) is first
    before = mesh.stat()
    stl(mesh, [[0, 0, 0], [2, 0, 0], [0, 1, 0]])
    os.utime(mesh, ns=(before.st_atime_ns, before.st_mtime_ns))
    second = store.get_asset(config)
    assert first.asset_id != second.asset_id
    assert points(second)[:, 0].max() == 2
    path.write_text(path.read_text().replace('name="base"', 'name="renamed"'))
    third = store.get_asset(config)
    assert third.asset_id != second.asset_id
    assert third.manifest["base_link"] == "renamed"
    mesh.unlink()
    with pytest.raises(RobotVisualError):
        store.get_asset(config)
    assert store._asset is None


def test_cache_config_change_bypasses_check_interval_and_relative_path(tmp_path):
    path = model(tmp_path, '<visual><geometry><box size="1 1 1"/></geometry></visual>')
    store = RobotVisualAssetStore(base_dir=tmp_path, check_interval_s=60)
    config = RobotGeometryConfig(model_path=path.name)
    first = store.get_asset(config)
    assert store.get_asset(config) is first
    second = store.get_asset(RobotGeometryConfig(model_path=path.name, width_m=0.75))
    assert second.asset_id != first.asset_id


@pytest.mark.parametrize("kind", ["relative", "absolute", "symlink", "package", "url"])
def test_mesh_cannot_read_outside_model_directory(tmp_path, kind):
    directory = tmp_path / "robot"
    directory.mkdir()
    outside = tmp_path / "secret.stl"
    stl(outside, [[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    if kind == "relative":
        filename = "../secret.stl"
    elif kind == "absolute":
        filename = str(outside)
    elif kind == "symlink":
        (directory / "linked.stl").symlink_to(outside)
        filename = "linked.stl"
    elif kind == "package":
        filename = "package://robot/../secret.stl"
    else:
        filename = "https://example.invalid/secret.stl"
    path = model(directory, f'<visual><geometry><mesh filename="{filename}"/></geometry></visual>')
    with patch("core.robot_visual._safe_read", wraps=__import__("core.robot_visual", fromlist=["_safe_read"])._safe_read) as read:
        with pytest.raises(RobotVisualError) as error:
            load(path)
        assert error.value.code == "robot_visual_path_forbidden"
        assert all(args[0][1] != outside for args in read.call_args_list)


@pytest.mark.parametrize("replacement", [
    ('url="#mesh"', 'url="../secret.dae#mesh"'),
    ('source="#positions"', 'source="file:///etc/passwd"'),
    ('</COLLADA>', '<library_images><image id="secret"><init_from>../../secret.png</init_from></image></library_images></COLLADA>'),
])
def test_dae_external_references_rejected_without_reading_them(tmp_path, replacement):
    (tmp_path / "part.dae").write_text(dae().replace(*replacement))
    path = model(tmp_path, '<visual><geometry><mesh filename="part.dae"/></geometry></visual>')
    with pytest.raises(RobotVisualError) as error:
        load(path)
    assert error.value.code == "robot_visual_external_reference"


def test_xml_entity_is_rejected(tmp_path):
    path = tmp_path / "robot.urdf"
    path.write_text('<!DOCTYPE robot [<!ENTITY stolen SYSTEM "file:///etc/passwd">]><robot name="test">&stolen;</robot>')
    with pytest.raises(RobotVisualError, match="实体"):
        load(path)


def test_reference_pose_uses_existing_kinematics_and_reports_defaulted_joints(tmp_path):
    shape = '<collision><geometry><box size="0.2 0.3 0.4"/></geometry></collision>'
    path = model(tmp_path, shape,
        joints='<joint name="turn" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-2" upper="2"/></joint>'
               '<joint name="slide" type="prismatic"><parent link="arm"/><child link="hand"/><axis xyz="1 0 0"/></joint>',
        extra_links=f'<link name="arm">{shape}</link><link name="hand">{shape}</link>')
    asset = load(path, model_joint_positions_json=json.dumps({"turn": math.pi/2}))
    expected = load_robot_kinematic_model(path).link_transforms({"turn": math.pi/2, "slide": 0})
    for link, matrix in expected.items():
        np.testing.assert_allclose(asset.reference_transforms[link], matrix)
        np.testing.assert_allclose(np.array(asset.manifest["reference_transforms"][link]).reshape(4, 4, order="F"), matrix)
    assert asset.manifest["reference_pose_source"] == "configured_with_zero_defaults"
    assert asset.manifest["reference_defaulted_joints"] == ["slide"]
    with pytest.raises(RobotVisualError, match="限位"):
        load(path, model_joint_positions_json='{"turn":3}')


def test_unconfigured_model_has_explicit_fallback_reason():
    with pytest.raises(RobotVisualError) as error:
        RobotVisualAssetStore().get_asset({})
    assert error.value.code == "robot_visual_model_unconfigured"


def test_unselected_collision_is_checked_and_participates_in_hash(tmp_path):
    mesh = tmp_path / "collision.stl"
    stl(mesh, [[0, 0, 0], [1, 0, 0], [0, 1, 1]])
    path = model(tmp_path, '<visual><geometry><box size="1 1 1"/></geometry></visual><collision><geometry><mesh filename="collision.stl"/></geometry></collision>')
    config = RobotGeometryConfig(model_path=str(path))
    store = RobotVisualAssetStore(check_interval_s=0)
    first = store.get_asset(config)
    assert first.manifest["source_mesh_bytes"] == 0
    assert first.manifest["all_source_mesh_bytes"] == mesh.stat().st_size
    stl(mesh, [[0, 0, 0], [2, 0, 0], [0, 1, 1]])
    second = store.get_asset(config)
    assert first.asset_id != second.asset_id
    assert first.mesh_bytes == second.mesh_bytes
    assert first.navigation_geometry != second.navigation_geometry
    path.write_text(path.read_text().replace('filename="collision.stl"', 'filename="../secret.stl"'))
    with pytest.raises(RobotVisualError) as error:
        store.get_asset(config)
    assert error.value.code == "robot_visual_path_forbidden"


def test_safe_geometry_snapshot_matches_legacy_and_never_reopens_files(tmp_path):
    from core.robot_geometry import resolve_robot_geometry
    mesh = tmp_path / "part.stl"
    stl(mesh, [[0, 0, 0], [1, 0, 0], [0, 1, 1]])
    path = model(tmp_path, '<collision><geometry><mesh filename="part.stl"/></geometry></collision>')
    config = RobotGeometryConfig(model_path=str(path))
    expected = resolve_robot_geometry(config, robot_name="test", base_dir=tmp_path).to_runtime_dict()
    resources = {path: path.read_bytes(), mesh: mesh.read_bytes()}
    with patch("core.robot_geometry._read_bounded", side_effect=AssertionError("快照路径禁止再读文件")):
        actual = resolve_robot_geometry(config, robot_name="test", base_dir=tmp_path, resource_bytes=resources).to_runtime_dict()
        assert actual == expected
        with pytest.raises(ValueError, match="快照缺少网格"):
            resolve_robot_geometry(config, robot_name="test", base_dir=tmp_path, resource_bytes={path: resources[path]})


def test_asset_kinematics_and_navigation_use_the_same_urdf_snapshot(tmp_path):
    import core.robot_visual as module
    path = model(tmp_path, '<collision><geometry><box size="1 2 3"/></geometry></collision>')
    original = module.load_robot_kinematic_model

    def replace_after_safe_read(*args, **kwargs):
        path.write_text('<robot name="changed"><link name="changed"/></robot>')
        return original(*args, **kwargs)

    with patch.object(module, "load_robot_kinematic_model", side_effect=replace_after_safe_read):
        asset = load(path)
    assert asset.model.base_link == "base"
    assert asset.navigation_geometry["self_collision"]["model_base_link"] == "base"
    assert asset.navigation_geometry["z_max_m"] == 1.5


def test_file_replaced_with_symlink_after_resolution_is_not_opened(tmp_path):
    from core.robot_visual import _safe_read
    directory = tmp_path / "robot"
    directory.mkdir()
    path = directory / "part.stl"
    outside = tmp_path / "secret.stl"
    outside.write_text("secret")
    path.symlink_to(outside)
    with pytest.raises(RobotVisualError) as error:
        _safe_read(directory, path, 1024)
    assert error.value.code == "robot_visual_resource_unreadable"


def test_failed_asset_is_rate_limited_and_recovers_after_interval(tmp_path):
    import core.robot_visual as module
    path = model(tmp_path, '<visual><geometry><mesh filename="missing.stl"/></geometry></visual>')
    config = RobotGeometryConfig(model_path=str(path))
    store = RobotVisualAssetStore(check_interval_s=1)
    clock = [10.0]
    with patch.object(module.time, "monotonic", side_effect=lambda: clock[0]), patch.object(module, "_safe_read", wraps=module._safe_read) as read:
        with pytest.raises(RobotVisualError):
            store.get_asset(config)
        initial = read.call_count
        for _ in range(10):
            with pytest.raises(RobotVisualError):
                store.get_asset(config)
        assert read.call_count == initial
        stl(tmp_path / "missing.stl", [[0, 0, 0], [1, 0, 0], [0, 1, 0]])
        clock[0] += 1.01
        asset = store.get_asset(config)
        assert asset.manifest["triangle_count"] == 1
        assert read.call_count > initial


def test_dae_declared_triangle_count_cannot_hide_missing_faces(tmp_path):
    (tmp_path / "part.dae").write_text(dae().replace('triangles count="2"', 'triangles count="3"'))
    path = model(tmp_path, '<visual><geometry><mesh filename="part.dae"/></geometry></visual>')
    with pytest.raises(RobotVisualError, match="数量"):
        load(path)


def test_single_urdf_material_overrides_dae_material(tmp_path):
    (tmp_path / "part.dae").write_text(dae())
    path = model(tmp_path, '<visual><geometry><mesh filename="part.dae"/></geometry><material name="override"><color rgba="0 0 1 1"/></material></visual>')
    asset = load(path)
    assert asset.manifest["visuals"][0]["rgba"] == [0, 0, 1, 1]


def test_missing_visual_uses_complete_safe_navigation_collision(tmp_path):
    import core.robot_visual as module
    path = model(tmp_path, '<visual><geometry><mesh filename="missing.stl"/></geometry></visual>'
                 '<collision><origin xyz="1 2 3"/><geometry><box size="2 4 6"/></geometry></collision>')
    store = RobotVisualAssetStore(check_interval_s=60)
    config = RobotGeometryConfig(model_path=str(path))
    with pytest.raises(RobotVisualError):
        store.get_asset(config)
    with patch.object(module, "_safe_read", wraps=module._safe_read) as read:
        navigation = store.get_navigation_geometry(config)
        assert navigation["z_min_m"] == 0
        assert navigation["z_max_m"] == 6
        np.testing.assert_allclose(np.min(navigation["footprint_xy_m"], axis=0), [0, 0], atol=1e-10)
        np.testing.assert_allclose(np.max(navigation["footprint_xy_m"], axis=0), [2, 4], atol=1e-10)
        assert navigation["source"] == "urdf_collision"
        assert "model_path" not in navigation
        initial = read.call_count
        assert store.get_navigation_geometry(config) is navigation
        assert read.call_count == initial == 1


def test_missing_navigation_mesh_cannot_produce_an_invented_box(tmp_path):
    path = model(tmp_path, '<visual><geometry><mesh filename="missing.stl"/></geometry></visual>'
                 '<collision><geometry><mesh filename="also-missing.stl"/></geometry></collision>')
    store = RobotVisualAssetStore(check_interval_s=0)
    with pytest.raises(RobotVisualError) as error:
        store.get_navigation_geometry(RobotGeometryConfig(model_path=str(path)))
    assert error.value.code == "robot_visual_resource_unreadable"


def test_navigation_fallback_still_audits_unused_visual_path(tmp_path):
    path = model(tmp_path, '<visual><geometry><mesh filename="../secret.stl"/></geometry></visual>'
                 '<collision><geometry><box size="2 4 6"/></geometry></collision>')
    with pytest.raises(RobotVisualError) as error:
        RobotVisualAssetStore().get_navigation_geometry(RobotGeometryConfig(model_path=str(path)))
    assert error.value.code == "robot_visual_path_forbidden"
