"""配置注册表一致性测试（实施计划 阶段1 验收）。

逻辑用系统 python3 即可跑（纯 AST/字符串，不依赖 pydantic）。
"""

from __future__ import annotations

import re
from pathlib import Path

from core import config_registry as R

_REPO = Path(__file__).resolve().parents[2]


def _code_read_keys() -> set:
    """直接从 settings 源码正则提取所有被读取的 env 键（独立于 registry 的解析口径）。"""

    keys = set()
    for rel in ("core/settings.py", "drivers/go2/settings.py"):
        src = (_REPO / rel).read_text(encoding="utf-8")
        for m in re.finditer(r'_cfg_(?:str|int|bool|float|csv)(?:_alias)?\(\s*"([A-Z0-9_]+)"', src):
            keys.add(m.group(1))
        # _prefer_platform_str(platform_value, "KEY", default)：键在第二个字面量实参
        for m in re.finditer(r'_prefer_platform_str\(\s*[^,]+,\s*"([A-Z0-9_]+)"', src):
            keys.add(m.group(1))
    return keys


def test_registry_covers_all_read_keys():
    reg_keys = set(R.all_keys())
    code_keys = _code_read_keys()
    aliases = {key for entry in R.runtime_catalog().values() for key in entry.get("aliases", ())}
    missing = code_keys - reg_keys - aliases
    assert not missing, f"registry 缺失代码读取键: {sorted(missing)}"


def test_no_unclassified_misc():
    misc = [f.key for f in R.build_registry() if f.section_id == "misc"]
    assert not misc, f"未归类(misc)键需补分类规则: {misc}"


def test_every_field_has_required_metadata():
    for f in R.build_registry():
        assert re.search(r"[\u4e00-\u9fff]", f.zh_name), f"{f.key} 缺中文名"
        assert f.type in {"str", "int", "bool", "float", "csv"}, f"{f.key} 类型非法: {f.type}"
        assert f.section_title, f"{f.key} 缺板块标题"
        assert f.target_file in {R.TARGET_CORE, R.TARGET_GO2}, f"{f.key} 目标文件非法"


def test_sensitive_keys_present_in_registry():
    reg_keys = set(R.all_keys())
    for k in R.sensitive_keys():
        assert k in reg_keys, f"敏感键 {k} 不在 registry（可能键名已变）"


def test_go2_keys_route_to_data_plane():
    for f in R.build_registry():
        if f.key.startswith("GO2_") and f.target_file == R.TARGET_GO2:
            return
    raise AssertionError("未发现任何路由到 go2_data_plane_env 的 GO2_ 键")


def test_high_frequency_config_keys_have_chinese_names():
    expected = {
        "NUWAX_PERCEPTION_OPENAI_ENABLED": "启用 OpenAI 视觉分析",
        "NUWAX_PERCEPTION_SEGMENTATION_ENABLED": "启用实例分割",
        "NUWAX_PERCEPTION_STREAM_ENABLED": "启用持续视频感知",
        "NUWAX_PERCEPTION_STREAM_AUTO_START": "启动时自动开启视频感知",
    }
    for key, zh_name in expected.items():
        field = R.get_field(key)
        assert field is not None, f"{key} 缺失"
        assert field.zh_name == zh_name


def test_derived_config_names_do_not_expose_bare_suffixes():
    bare_suffixes = {"enabled", "model", "path", "topic", "frame", "timeout", "interval"}
    for field in R.build_registry():
        assert field.zh_name.lower() not in bare_suffixes, f"{field.key} 中文名仍是裸英文后缀"


def test_pointcloud3d_display_config_is_registered_in_display_section():
    expected_types = {
        "NUWAX_GATEWAY_POINTCLOUD3D_SPACING_M": "float",
        "NUWAX_GATEWAY_POINTCLOUD3D_SIZE": "float",
        "NUWAX_GATEWAY_POINTCLOUD3D_COLOR": "str",
        "NUWAX_GATEWAY_POINTCLOUD3D_HEIGHT_COLORS": "csv",
        "NUWAX_GATEWAY_POINTCLOUD3D_GROUND_COLOR": "str",
        "NUWAX_GATEWAY_POINTCLOUD3D_STRUCTURE_COLOR": "str",
        "NUWAX_GATEWAY_POINTCLOUD3D_OBJECT_COLOR": "str",
    }
    for key, field_type in expected_types.items():
        field = R.get_field(key)
        assert field is not None, f"{key} 缺失"
        assert field.type == field_type
        assert field.section_id == "display"
        assert field.target_file == R.TARGET_CORE


def test_pointcloud3d_display_config_loads_and_normalizes_env(monkeypatch):
    from core import settings

    values = {
        "NUWAX_GATEWAY_POINTCLOUD3D_SPACING_M": "0.24",
        "NUWAX_GATEWAY_POINTCLOUD3D_SIZE": "2.25",
        "NUWAX_GATEWAY_POINTCLOUD3D_COLOR": "#ABCDEF",
        "NUWAX_GATEWAY_POINTCLOUD3D_HEIGHT_COLORS": "#010203,#040506,#070809,#0A0B0C,#0D0E0F",
        "NUWAX_GATEWAY_POINTCLOUD3D_GROUND_COLOR": "invalid",
        "NUWAX_GATEWAY_POINTCLOUD3D_STRUCTURE_COLOR": "#445566",
        "NUWAX_GATEWAY_POINTCLOUD3D_OBJECT_COLOR": "#778899",
    }
    for key, value in values.items():
        monkeypatch.setenv(key, value)

    config = settings.load_config().gateway.pointcloud3d

    assert config.spacing_m == 0.24
    assert config.size == 2.25
    assert config.color == "#abcdef"
    assert config.height_colors == ("#010203", "#040506", "#070809", "#0a0b0c", "#0d0e0f")
    assert config.ground_color == "#38bdf8"
    assert config.structure_color == "#445566"
    assert config.object_color == "#778899"
