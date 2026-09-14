"""机器人控制台的统一设置目录。

Python（编程语言）配置默认值取实际加载器源码；跨进程用户设置集中在
config_catalog.json，记录中文名、默认值、消费者、兼容别名和生效范围。
config_descriptions.json 只保留结构化 Python 设置的补充说明。
"""

from __future__ import annotations

import ast
import json
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
from typing import Dict, List, Optional, Tuple

_REPO_ROOT = Path(__file__).resolve().parent.parent
_CORE_SETTINGS = _REPO_ROOT / "core" / "settings.py"
# 仅解析设备配置源码，不在构建页面时导入机器人驱动。
_GO2_SETTINGS = _REPO_ROOT / "drivers" / "go2" / "settings.py"
_DESCRIPTIONS_JSON = _REPO_ROOT / "core" / "config_descriptions.json"

# 目标文件标识：core 主 .env / go2 数据面独立 env（drivers/go2/data_plane.env）。
TARGET_CORE = "core_env"
TARGET_GO2 = "go2_data_plane_env"


@dataclass(frozen=True)
class ConfigField:
    """单个配置项的展示元数据。"""

    key: str
    zh_name: str
    description: str
    type: str  # str | int | bool | float | csv
    section_id: str
    section_title: str
    advanced: bool
    sensitive: bool
    target_file: str
    default_repr: str  # 源码解析出的默认值文本（仅展示，不作运行时来源）
    alias_of: Optional[str] = None
    choices: Tuple[str, ...] = field(default_factory=tuple)
    choice_labels: Tuple[str, ...] = field(default_factory=tuple)
    consumers: Tuple[str, ...] = field(default_factory=tuple)
    apply_scope: str = "重启网关后生效"
    aliases: Tuple[str, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class _RawKey:
    """从源码 AST 解析出的原始键信息（默认值唯一事实源）。"""

    key: str
    type: str
    default_repr: str
    alias_of: Optional[str]
    target_file: str


@lru_cache(maxsize=1)
def runtime_catalog() -> Dict[str, dict]:
    """跨进程用户设置目录；不把部署注入值和测试变量伪装成业务设置。"""
    return json.loads((_REPO_ROOT / "core" / "config_catalog.json").read_text(encoding="utf-8"))


def _node_source(node: ast.AST, source: str) -> str:
    """取 AST 节点对应的源码文本，Python 3.8 兼容。

    优先 ``ast.get_source_segment``（3.8 起可用）；缺失时回退到 ``ast.unparse``
    （3.9+）；都没有则空串。替代直接用 ``ast.unparse``——后者 3.8 不存在，
    实机（Python 3.8）会 AttributeError 导致 build_registry 崩溃。
    """

    seg = ast.get_source_segment(source, node)
    if seg is not None:
        return seg
    unparse = getattr(ast, "unparse", None)
    if unparse is not None:
        return unparse(node)
    return ""


def _parse_settings_keys(path: Path, target_file: str) -> Dict[str, _RawKey]:
    """AST 解析 settings 文件，提取每个 _cfg_* 调用的键/类型/默认值/alias。

    默认值取源码字面量文本（如 ``True`` / ``'/utlidar/robot_odom'`` / 嵌套 _cfg_* 表达式），
    仅用于展示；运行时真实值仍由 settings.py 的 load_config() 决定。
    """

    raw: Dict[str, _RawKey] = {}
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)):
            continue
        fn = node.func.id
        if fn == "_prefer_platform_str":
            # _prefer_platform_str(platform_value, "KEY", default) -> 内部走 _cfg_str("KEY", default)
            args = node.args
            if len(args) >= 2 and isinstance(args[1], ast.Constant) and isinstance(args[1].value, str):
                key = args[1].value
                default_repr = ""
                if len(args) >= 3:
                    try:
                        default_repr = repr(ast.literal_eval(args[2]))
                    except Exception:
                        default_repr = _node_source(args[2], source)
                raw.setdefault(key, _RawKey(key, "str", default_repr, None, target_file))
            continue
        if not fn.startswith("_cfg_"):
            continue
        typ = fn.replace("_cfg_", "").replace("_alias", "")
        is_alias = fn.endswith("_alias")
        args = node.args
        if not args or not isinstance(args[0], ast.Constant) or not isinstance(args[0].value, str):
            continue
        key = args[0].value
        alias_of: Optional[str] = None
        default_idx = 1
        if (
            is_alias
            and len(args) > 1
            and isinstance(args[1], ast.Constant)
            and isinstance(args[1].value, str)
            and args[1].value.isupper()
        ):
            alias_of = args[1].value
            default_idx = 2
        default_repr = ""
        if len(args) > default_idx:
            try:
                default_repr = repr(ast.literal_eval(args[default_idx]))
            except Exception:
                default_repr = _node_source(args[default_idx], source)
        # 同键多次出现以首次为准（settings.py 内通常唯一）。
        raw.setdefault(key, _RawKey(key, typ, default_repr, alias_of, target_file))
    return raw


def _all_raw_keys() -> Dict[str, _RawKey]:
    merged: Dict[str, _RawKey] = {}
    merged.update(_parse_settings_keys(_CORE_SETTINGS, TARGET_CORE))
    for key, rk in _parse_settings_keys(_GO2_SETTINGS, TARGET_GO2).items():
        merged.setdefault(key, rk)
    for key, entry in runtime_catalog().items():
        merged.setdefault(key, _RawKey(key, entry["type"], repr(entry["default"]), None, entry["target_file"]))
    # 旧名称仅供读取与迁移，同一功能只展示一个规范设置。
    for entry in runtime_catalog().values():
        for alias in entry.get("aliases", ()):
            merged.pop(alias, None)
    return merged


# 板块顺序即控制台展示顺序。
SECTIONS: Tuple[Tuple[str, str], ...] = (
    ("robot", "机器人与运行入口"),
    ("gateway", "网关与服务"),
    ("telemetry", "性能监控（遥测）"),
    ("logging", "日志"),
    ("geometry_runtime", "几何引擎·运行时与通信"),
    ("geometry_topics", "几何引擎·话题与坐标系"),
    ("geometry_quality", "几何引擎·点云与质量"),
    ("mapping", "建图"),
    ("nav_stack", "导航与探索"),
    ("lcm", "通信通道与工作区"),
    ("external_geometry", "外部几何栈与桥接"),
    ("perception_detect", "感知·检测与跟踪"),
    ("perception_vision", "感知·分割/云端视觉/延迟投影"),
    ("perception_stream", "感知·关键帧与流"),
    ("runtime_data", "记忆与制品"),
    ("go2_input", "Go2 数据面·输入流"),
    ("go2_official", "Go2 数据面·官方控制"),
    ("go2_video", "Go2 数据面·视频推流"),
    ("relay", "容器中继转发"),
    ("shm_uds", "共享内存与本地套接字"),
    ("misc", "其它"),
)
_SECTION_TITLES = dict(SECTIONS)

# 分类规则：(section_id, 前缀元组)。按列表顺序匹配，**首个命中**生效，故更具体的前缀在前。
_SECTION_RULES: Tuple[Tuple[str, Tuple[str, ...]], ...] = (
    ("telemetry", ("NUWAX_GATEWAY_TELEMETRY",)),
    ("gateway", ("NUWAX_GATEWAY",)),
    ("relay", ("NUWAX_RELAY",)),
    ("logging", ("NUWAX_LOG", "GO2_PROXY_LOG")),
    ("mapping", ("NUWAX_MAPPING", "NUWAX_OSM_", "NUWAX_NOMINATIM_", "NUWAX_GOOGLE_MAPS_")),
    ("nav_stack", ("NUWAX_NAV_", "NUWAX_PERSON_FOLLOW_", "NUWAX_EXPLORE_")),
    ("geometry_runtime", ("NUWAX_GEOMETRY_RUNTIME",)),
    ("lcm", ("NUWAX_LCM", "NUWAX_REAL_LCM_", "NUWAX_NUWAX_WORKSPACE_DIR")),
    ("external_geometry", ("NUWAX_GEOMETRY_EXTERNAL_PROVIDES",)),
    ("geometry_quality", ("NUWAX_GEOMETRY_VOXEL", "NUWAX_GEOMETRY_CROP", "NUWAX_GEOMETRY_MAX_RANGE",
                           "NUWAX_GEOMETRY_TRAVERSABILITY", "NUWAX_GEOMETRY_MAX_", "NUWAX_GEOMETRY_ODOM_COV",
                           "NUWAX_GEOMETRY_TIMESTAMP", "NUWAX_GEOMETRY_POINTCLOUD")),
    ("geometry_topics", ("NUWAX_GEOMETRY_", "NUWAX_MAP_FRAME", "NUWAX_BASE_FRAME")),
    ("perception_detect", ("NUWAX_PERCEPTION_YOLO", "NUWAX_PERCEPTION_BYTETRACK", "NUWAX_PERCEPTION_TRACKER")),
    ("perception_vision", ("NUWAX_PERCEPTION_OPENAI", "NUWAX_PERCEPTION_SEGMENTATION", "NUWAX_PERCEPTION_DEFERRED",
                            "OPENAI_", "ARK_")),
    ("perception_stream", ("NUWAX_PERCEPTION_STREAM", "NUWAX_PERCEPTION_KEYFRAME", "NUWAX_PERCEPTION_BURST",
                            "NUWAX_PERCEPTION",)),
    ("runtime_data", ("NUWAX_MEMORY", "NUWAX_MAP", "NUWAX_ARTIFACT", "NUWAX_RUNTIME", "NUWAX_STATE",
                       "NUWAX_DIAGNOSTIC", "NUWAX_ACTIVE_MAP", "NUWAX_LOCALIZATION_HISTORY",
                       "NUWAX_NAVIGATION_HISTORY", "NUWAX_OBSERVATION_HISTORY", "DASHSCOPE")),
    ("go2_video", ("GO2_VIDEO_STREAM", "GO2_ROBOT_IP", "GO2_WEBRTC_")),
    ("go2_official", ("GO2_OFFICIAL",)),
    ("go2_input", ("GO2_DATA_PLANE", "GO2_DIRECT_DDS", "GO2_DDS", "GO2_IMU")),
    ("robot", ("NUWAX_ACTIVE_ROBOT", "GO2_SDK", "GO2_LOW_LEVEL", "GO2_NETWORK", "GO2_")),
    ("shm_uds",           ("NUWAX_CLOUDMAP_SHM_PATH", "NUWAX_OCCUPANCY_SHM_PATH", "NUWAX_FORBIDDEN_ZONES_PATH", "NUWAX_VIDEO_")),
)

# 敏感键（脱敏不回显）。鉴权已移除，故仅余各类外部服务密钥/令牌。
_SENSITIVE_KEYS = frozenset({
    "NUWAX_RELAY_INCOMING_TOKENS",
    "NUWAX_RELAY_UPSTREAM_TOKEN",
    "NUWAX_PERCEPTION_OPENAI_API_KEY",
    "OPENAI_API_KEY",
    "ARK_API_KEY",
    "NUWAX_MEMORY_EMBEDDING_API_KEY",
    "DASHSCOPE_API_KEY",
    "GO2_WEBRTC_AES_128_KEY",
    "NUWAX_GOOGLE_MAPS_API_KEY",
})

# 常用设置默认展开，其余归入高级设置。
_REGULAR_KEYS = frozenset({
    "NUWAX_ACTIVE_ROBOT", "GO2_DDS_IFACE",
    "NUWAX_ROBOT_MODEL_PATH", "NUWAX_ROBOT_LENGTH_M", "NUWAX_ROBOT_WIDTH_M", "NUWAX_ROBOT_HEIGHT_M",
    "NUWAX_GATEWAY_HOST", "NUWAX_GATEWAY_PORT",
    "NUWAX_GATEWAY_POINTCLOUD3D_SPACING_M", "NUWAX_GATEWAY_POINTCLOUD3D_SIZE",
    "NUWAX_GATEWAY_POINTCLOUD3D_COLOR",
    "NUWAX_GATEWAY_TELEMETRY_ENABLED", "NUWAX_GATEWAY_TELEMETRY_INTERVAL_SEC",
    "NUWAX_LOG_LEVEL",
    "NUWAX_GEOMETRY_ENABLED",
    "NUWAX_GEOMETRY_PIPELINE",
    "NUWAX_GEOMETRY_LIO_BACKEND", "NUWAX_GEOMETRY_MAPPING_BACKEND",
    "NUWAX_GEOMETRY_CORRECTION_BACKEND",
    "NUWAX_MAPPING_PROFILE_NAME", "NUWAX_MAPPING_PROFILE_ENABLED",
    "NUWAX_GEOMETRY_EXTERNAL_ENABLED",
    "NUWAX_PERCEPTION_YOLO_WORLD_ENABLED", "NUWAX_PERCEPTION_YOLO_WORLD_CONFIDENCE",
    "NUWAX_PERCEPTION_STREAM_AUTO_START",
    "NUWAX_PERCEPTION_OPENAI_ENABLED", "NUWAX_PERCEPTION_SEGMENTATION_ENABLED",
    "NUWAX_PERCEPTION_STREAM_ENABLED", "NUWAX_PERCEPTION_STREAM_INTERVAL_SEC",
    "NUWAX_MEMORY_EMBEDDING_API_KEY", "NUWAX_MAP_CATALOG_ROOT",
    "GO2_DATA_PLANE_ENABLED", "GO2_DIRECT_DDS_ENABLED",
    "GO2_OFFICIAL_BACKEND_ENABLED", "GO2_OFFICIAL_OBSTACLE_AVOIDANCE_DEFAULT_ENABLED",
    "GO2_VIDEO_STREAM_ENABLED", "GO2_ROBOT_IP",
    "NUWAX_RELAY_ENABLED",
})

# 枚举型键的可选值（用于前端下拉）。
_CHOICES: Dict[str, Tuple[str, ...]] = {
    "NUWAX_GEOMETRY_PIPELINE": ("lio_3d", "slam_2d"),
    "NUWAX_LOG_LEVEL": ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"),
    "NUWAX_GEOMETRY_LIO_BACKEND": ("device", "fastlio2", "pointlio"),
    "NUWAX_GEOMETRY_MAPPING_BACKEND": ("voxel", "raytracing"),
    "NUWAX_GEOMETRY_CORRECTION_BACKEND": ("none", "library_pgo", "native_pgo", "relocalization"),
    "NUWAX_EXPLORE_ARBITRATION": ("prefer_wavefront",),
}


_CHOICE_LABELS: Dict[str, Tuple[str, ...]] = {
    "NUWAX_GEOMETRY_PIPELINE": ("三维建图、探索与地形导航", "二维建图、探索与平面导航"),
    "NUWAX_GEOMETRY_LIO_BACKEND": ("设备里程计", "快速激光惯性定位（FastLIO2）", "逐点激光惯性定位（Point-LIO）"),
    "NUWAX_GEOMETRY_MAPPING_BACKEND": ("体素窗口", "射线追踪"),
    "NUWAX_GEOMETRY_CORRECTION_BACKEND": ("不启用校正", "库内位姿图优化", "原生位姿图优化", "重定位校正"),
    "NUWAX_LOG_LEVEL": ("调试", "信息", "警告", "错误", "严重错误"),
    "NUWAX_EXPLORE_ARBITRATION": ("优先波前探索",),
}


def _classify_section(key: str) -> str:
    for section_id, prefixes in _SECTION_RULES:
        if key.startswith(prefixes):
            return section_id
    return "misc"


@lru_cache(maxsize=1)
def _env_example_desc() -> Dict[str, str]:
    """读取冻结的描述表 core/config_descriptions.json。

    注意：描述**不**从 .env.example 收割——.env.example 现在是由本 registry 生成的产物，
    若反向收割会形成循环依赖（每次重生成都把注释叠加一次）。描述独立冻结为 JSON，
    源自历史策展模板，可手工增补。
    """

    path = _DESCRIPTIONS_JSON
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return {str(k): str(v) for k, v in data.items() if v}


# 所有配置 key 的显式中文名。
#
# 这里是 Web 配置页展示名的唯一来源；不得用英文 token 拆词兜底。
# 新增/删除配置 key 时，_validate_zh_name_coverage 会强制要求同步维护本表，
# 避免页面再次出现 enabled / backend mode / max level 等机械英文标签。
_ZH_NAMES: Dict[str, str] = {
    'NUWAX_ACTIVE_ROBOT': '当前激活机器人',
    'GO2_SDK_PATH': 'Go2 SDK 路径',
    'GO2_LOW_LEVEL_INTERVAL_SEC': '低级控制周期（秒）',
    'GO2_LOW_LEVEL_KD': '低级控制 KD 增益',
    'GO2_LOW_LEVEL_KP': '低级控制 KP 增益',
    'GO2_LOW_LEVEL_MAX_TORQUE': '低级控制最大扭矩',
    'GO2_LOW_LEVEL_MAX_VELOCITY': '低级控制最大速度',
    'GO2_VOLUME_AUTO_ENABLE_VUI_SWITCH': '自动开启 Go2 VUI 音量开关',
    'NUWAX_GATEWAY_HOST': '网关监听地址',
    'NUWAX_GATEWAY_PORT': '网关端口',
    'NUWAX_GATEWAY_ARTIFACT_DIR': '网关制品目录',
    'NUWAX_GATEWAY_PUBLIC_BASE_URL': '网关公开访问地址',
    'NUWAX_GATEWAY_SSE_KEEPALIVE_SEC': '网关 SSE 保活间隔（秒）',
    'NUWAX_GATEWAY_WS_PING_INTERVAL_SEC': '网关 WebSocket 心跳间隔（秒）',
    'NUWAX_GATEWAY_POINTCLOUD3D_SPACING_M': '3D 点云显示间隔（米）',
    'NUWAX_GATEWAY_POINTCLOUD3D_SIZE': '3D 点云点大小',
    'NUWAX_GATEWAY_POINTCLOUD3D_COLOR': '3D 点云基础颜色',
    'NUWAX_GATEWAY_POINTCLOUD3D_HEIGHT_COLORS': '3D 点云高度渐变颜色',
    'NUWAX_GATEWAY_POINTCLOUD3D_GROUND_COLOR': '3D 点云地面颜色',
    'NUWAX_GATEWAY_POINTCLOUD3D_STRUCTURE_COLOR': '3D 点云结构颜色',
    'NUWAX_GATEWAY_POINTCLOUD3D_OBJECT_COLOR': '3D 点云物体颜色',
    'NUWAX_GATEWAY_TELEMETRY_ENABLED': '启用性能监控',
    'NUWAX_GATEWAY_TELEMETRY_INTERVAL_SEC': '性能监控采集间隔（秒）',
    'NUWAX_GATEWAY_TELEMETRY_INCLUDE_PROCESS_TREE': '纳入网关进程树',
    'NUWAX_GATEWAY_TELEMETRY_PROCESS_KEYWORDS': '遥测进程匹配关键词',
    'NUWAX_GEOMETRY_ENABLED': '启用几何引擎',
    'NUWAX_GEOMETRY_LIO_BACKEND': 'LIO（激光雷达惯性里程计）后端',
    'NUWAX_GEOMETRY_MAPPING_BACKEND': '建图后端',
    'NUWAX_GEOMETRY_CORRECTION_BACKEND': '位姿校正后端',
    'NUWAX_GEOMETRY_CLOUD_MAP_TOPIC': '点云地图话题',
    'NUWAX_GEOMETRY_COST_MAP_TOPIC': '代价地图话题',
    'NUWAX_GEOMETRY_IMU_FRAME': 'IMU 坐标系',
    'NUWAX_GEOMETRY_IMU_TOPIC': 'IMU 话题',
    'NUWAX_GEOMETRY_LIDAR_FRAME': '激光雷达坐标系',
    'NUWAX_GEOMETRY_MIN_TRAVERSABILITY_SCORE': '可通行性最低评分',
    'NUWAX_GEOMETRY_OCCUPANCY_TOPIC': '占据栅格话题',
    'NUWAX_GEOMETRY_ODOM_MAX_AGE_SEC': '里程计最大允许延迟（秒）',
    'NUWAX_GEOMETRY_ODOM_MAX_ANGULAR_COVARIANCE': '里程计最大角度协方差',
    'NUWAX_GEOMETRY_ODOM_MAX_LINEAR_COVARIANCE': '里程计最大线性协方差',
    'NUWAX_GEOMETRY_ODOM_QUALITY_SOURCE': '里程计质量来源',
    'NUWAX_GEOMETRY_ODOM_REQUIRE_COVARIANCE': '要求里程计协方差',
    'NUWAX_GEOMETRY_ODOM_TOPIC': '里程计话题',
    'NUWAX_GEOMETRY_POINT_CLOUD_CROP_FORWARD_MAX_M': '点云前向裁剪最大距离（米）',
    'NUWAX_GEOMETRY_POINT_CLOUD_CROP_FORWARD_MIN_M': '点云前向裁剪最小距离（米）',
    'NUWAX_GEOMETRY_POINT_CLOUD_CROP_LATERAL_M': '点云横向裁剪半宽（米）',
    'NUWAX_GEOMETRY_POINT_CLOUD_CROP_MAX_Z_M': '点云高度裁剪上限（米）',
    'NUWAX_GEOMETRY_POINT_CLOUD_CROP_MIN_Z_M': '点云高度裁剪下限（米）',
    'NUWAX_GEOMETRY_POINT_CLOUD_MAX_AGE_SEC': '点云最大允许延迟（秒）',
    'NUWAX_GEOMETRY_POINT_CLOUD_MAX_POINTS': '点云单帧最大点数',
    'NUWAX_GEOMETRY_POINT_CLOUD_MAX_RANGE_M': '点云最大距离（米）',
    'NUWAX_GEOMETRY_POINT_CLOUD_MIN_RANGE_M': '点云最小距离（米）',
    'NUWAX_GEOMETRY_POINT_CLOUD_PREPROCESS_ENABLED': '启用点云预处理',
    'NUWAX_GEOMETRY_POINT_CLOUD_TOPIC': '点云话题',
    'NUWAX_GEOMETRY_POINT_CLOUD_VOXEL_SIZE_M': '点云体素大小（米）',
    'NUWAX_GEOMETRY_SECONDARY_ODOM_TOPICS': '备用里程计话题',
    'NUWAX_GEOMETRY_TF_MAX_AGE_SEC': 'TF 最大允许延迟（秒）',
    'NUWAX_GEOMETRY_TF_STATIC_TOPIC': '静态 TF 话题',
    'NUWAX_GEOMETRY_TF_TOPIC': 'TF 话题',
    'NUWAX_GEOMETRY_MAX_SENSOR_SKEW_SEC': '最大传感器时间偏差（秒）',
    'NUWAX_GEOMETRY_TIMESTAMP_REQUIRE_MONOTONIC': '要求时间戳单调递增',
    'NUWAX_GEOMETRY_TIMESTAMP_SOURCE': '时间戳来源',
    'NUWAX_GEOMETRY_TRAVERSABILITY_FILTER_ENABLED': '启用可通行性过滤',
    'NUWAX_GEOMETRY_TRAVERSABILITY_MAX_SLOPE_DEG': '可通行最大坡度（度）',
    'NUWAX_GEOMETRY_TRAVERSABILITY_MAX_STEP_HEIGHT_M': '可通行最大台阶高度（米）',
    'NUWAX_GEOMETRY_TRAVERSABILITY_OUTPUT_LAYER': '可通行性输出图层',
    'NUWAX_MAPPING_PROFILE_NAME': 'nuwax-mapping 配置名称',
    'NUWAX_MAPPING_PROFILE_ENABLED': '启用 nuwax-mapping 建图',
    'NUWAX_MAPPING_DEFAULT_PROFILE': '使用默认 nuwax-mapping 配置',
    'NUWAX_MAPPING_DATABASE_ROOT': 'nuwax-mapping 数据库目录',
    'NUWAX_MAPPING_PCD_ROOT': 'nuwax-mapping 点云地图目录',
    'NUWAX_MAPPING_CRATE': 'nuwax-mapping crate 名',
    'NUWAX_MAPPING_OCCUPANCY_PROJECTION': 'nuwax-mapping 占据投影名称',
    'NUWAX_MAPPING_INPUT_CONTRACT': 'nuwax-mapping 输入契约',
    'NUWAX_MAPPING_OUTPUT_CONTRACT': 'nuwax-mapping 输出契约',
    'NUWAX_MAPPING_DATABASE_ASSET_NAME': 'nuwax-mapping 数据库制品名称',
    'NUWAX_MAPPING_PCD_ASSET_NAME': 'nuwax-mapping 点云地图制品名称',
    'NUWAX_MAPPING_OCCUPANCY_LAYER': 'nuwax-mapping 占据栅格图层',
    'NUWAX_MAPPING_COSTMAP_LAYER': 'nuwax-mapping 代价地图图层',
    'NUWAX_MAPPING_TRAVERSABILITY_LAYER': 'nuwax-mapping 可通行性图层',
    'NUWAX_MAPPING_LOCALIZATION_STATUS': 'nuwax-mapping 定位状态字段名',
    'NUWAX_MAPPING_RELOCALIZATION_DIAGNOSTICS': 'nuwax-mapping 重定位诊断字段名',
    'NUWAX_NAV_AUTOSTART': '自动启动导航后端',
    'NUWAX_NAV_GOAL_CHANNEL': '导航适配器目标通道',
    'NUWAX_NAV_CANCEL_CHANNEL': '导航适配器取消通道',
    'NUWAX_NAV_STATE_CHANNEL': '导航适配器状态通道',
    'NUWAX_NAV_COSTMAP_SOURCE': '导航代价地图来源',
    'NUWAX_NAV_COSTMAP_CHANNEL': '导航适配器代价地图通道',
    'NUWAX_NAV_ACTION_NAMESPACE': '导航动作命名空间',
    'NUWAX_NAV_LOOP_CLOSURE_THRESHOLD': '闭环判据计数阈值',
    'NUWAX_NAV_LOOP_CLOSURE_POLL_INTERVAL_SEC': '闭环判据轮询间隔（秒）',
    'NUWAX_NAV_DB_SQLITE_FALLBACK_ENABLED': '启用建图数据库闭环回退',
    'NUWAX_EXPLORE_ARBITRATION': '探索任务选择策略',
    'NUWAX_REAL_LCM_REQUIRED': '通信不可用时停止启动',
    'NUWAX_GEOMETRY_RUNTIME_BINARY': '几何运行程序路径',
    'NUWAX_GEOMETRY_RUNTIME_PORT': '几何远程接口端口',
    'NUWAX_GEOMETRY_RUNTIME_READY_FILE': '几何就绪记录文件',
    'NUWAX_GEOMETRY_RUNTIME_PROCESS_NAME': '几何进程名称',
    'NUWAX_GEOMETRY_RUNTIME_LOG_NAME': '几何日志名称',
    'NUWAX_GEOMETRY_EXTERNAL_ENABLED': '启用几何运行时',
    'NUWAX_GEOMETRY_EXTERNAL_PROVIDES_MAP': '提供地图数据',
    'NUWAX_GEOMETRY_EXTERNAL_PROVIDES_TF': '提供坐标变换',
    'NUWAX_GEOMETRY_EXTERNAL_PROVIDES_NAV': '由几何运行时提供导航',
    'NUWAX_LCM_URL': '机器人通信组播地址',
    'NUWAX_LCM_BACKEND': '机器人通信后端',
    'NUWAX_NUWAX_WORKSPACE_DIR': '原生功能库工作区目录',
    'NUWAX_LCM_CHANNEL_POSE_STAMPED': '定位位姿通道',
    'NUWAX_PERCEPTION_YOLO_WORLD_CONFIDENCE': 'YOLO-World 置信度阈值',
    'NUWAX_PERCEPTION_YOLO_WORLD_ENABLED': '启用 YOLO-World 检测',
    'NUWAX_PERCEPTION_BYTETRACK_HIGH_THRESHOLD': 'ByteTrack 高置信阈值',
    'NUWAX_PERCEPTION_BYTETRACK_IOU': 'ByteTrack 匹配 IOU 阈值',
    'NUWAX_PERCEPTION_BYTETRACK_LOST_TTL_FRAMES': 'ByteTrack 丢失保留帧数',
    'NUWAX_PERCEPTION_BYTETRACK_LOW_THRESHOLD': 'ByteTrack 低置信阈值',
    'NUWAX_PERCEPTION_BYTETRACK_NAME': 'ByteTrack 跟踪器名称',
    'NUWAX_PERCEPTION_BYTETRACK_TRAJECTORY_LIMIT': 'ByteTrack 轨迹保留数量',
    'NUWAX_PERCEPTION_TRACKER_BACKEND': '感知跟踪后端',
    'NUWAX_PERCEPTION_YOLO_WORLD_ALIASES': 'YOLO-World 类别别名',
    'NUWAX_PERCEPTION_YOLO_WORLD_BACKEND_NAME': 'YOLO-World 后端名称',
    'NUWAX_PERCEPTION_YOLO_WORLD_CLASSES': 'YOLO-World 检测类别',
    'NUWAX_PERCEPTION_YOLO_WORLD_DEVICE': 'YOLO-World 推理设备',
    'NUWAX_PERCEPTION_YOLO_WORLD_DYNAMIC_CLASS_KEYS': 'YOLO-World 动态类别元数据键',
    'NUWAX_PERCEPTION_YOLO_WORLD_IMAGE_SIZE': 'YOLO-World 输入图像尺寸',
    'NUWAX_PERCEPTION_YOLO_WORLD_IOU': 'YOLO-World IOU 阈值',
    'NUWAX_PERCEPTION_YOLO_WORLD_MAX_DETECTIONS': 'YOLO-World 最大检测数量',
    'NUWAX_PERCEPTION_YOLO_WORLD_MODEL': 'YOLO-World 模型路径',
    'NUWAX_PERCEPTION_YOLO_WORLD_USE_WORKER': '检测模型使用独立进程',
    'NUWAX_PERCEPTION_OPENAI_ENABLED': '启用 OpenAI 视觉分析',
    'NUWAX_PERCEPTION_SEGMENTATION_ENABLED': '启用实例分割',
    'ARK_API_KEY': 'ARK API 密钥',
    'NUWAX_PERCEPTION_DEFERRED_STRATEGY': '延迟投影策略',
    'NUWAX_PERCEPTION_DEFERRED_XVAL_THRESHOLD_M': '延迟投影交叉校验阈值（米）',
    'NUWAX_PERCEPTION_OPENAI_API_KEY': 'OpenAI API 密钥',
    'NUWAX_PERCEPTION_OPENAI_API_STYLE': 'OpenAI API 类型',
    'NUWAX_PERCEPTION_OPENAI_BACKEND_NAME': 'OpenAI 视觉后端名称',
    'NUWAX_PERCEPTION_OPENAI_BASE_URL': 'OpenAI API 基础地址',
    'NUWAX_PERCEPTION_OPENAI_MAX_TOKENS': 'OpenAI 最大输出令牌数',
    'NUWAX_PERCEPTION_OPENAI_MODEL': 'OpenAI 视觉模型',
    'NUWAX_PERCEPTION_OPENAI_TEMPERATURE': 'OpenAI 温度',
    'NUWAX_PERCEPTION_OPENAI_TIMEOUT_SEC': 'OpenAI 请求超时（秒）',
    'NUWAX_PERCEPTION_SEGMENTATION_ALLOW_BBOX_ONLY_FALLBACK': '允许仅使用检测框兜底',
    'NUWAX_PERCEPTION_SEGMENTATION_BACKEND_KIND': '分割后端类型',
    'NUWAX_PERCEPTION_SEGMENTATION_BACKEND_NAME': '分割后端名称',
    'NUWAX_PERCEPTION_SEGMENTATION_DEPTH_COVERAGE_THRESHOLD': '分割深度覆盖率阈值',
    'NUWAX_PERCEPTION_SEGMENTATION_EMIT_POLYGON': '输出分割多边形',
    'NUWAX_PERCEPTION_SEGMENTATION_EMIT_QUALITY_DETAILS': '输出分割质量详情',
    'NUWAX_PERCEPTION_SEGMENTATION_EMIT_RAW_MASK': '输出原始分割掩码',
    'NUWAX_PERCEPTION_SEGMENTATION_EMIT_RUNTIME_SUMMARY': '输出分割运行摘要',
    'NUWAX_PERCEPTION_SEGMENTATION_LOW_LATENCY_BACKEND_NAME': '分割低延迟后端名称',
    'NUWAX_PERCEPTION_SEGMENTATION_MAP_PROJECTION_COVERAGE_THRESHOLD': '分割地图投影覆盖率阈值',
    'NUWAX_PERCEPTION_SEGMENTATION_POLYGON_SIMPLIFICATION_EPSILON_PX': '分割多边形简化容差（像素）',
    'NUWAX_PERCEPTION_SEGMENTATION_PREDICTED_IOU_THRESHOLD': '分割预测 IOU 阈值',
    'NUWAX_PERCEPTION_SEGMENTATION_PRIMARY_BACKEND_NAME': '分割主后端名称',
    'NUWAX_PERCEPTION_SEGMENTATION_REPROJECTION_ERROR_THRESHOLD_PX': '分割重投影误差阈值（像素）',
    'NUWAX_PERCEPTION_SEGMENTATION_ROUTE': '分割路由策略',
    'NUWAX_PERCEPTION_SEGMENTATION_SCORE_THRESHOLD': '分割分数阈值',
    'NUWAX_PERCEPTION_SEGMENTATION_STABILITY_SCORE_THRESHOLD': '分割稳定性阈值',
    'OPENAI_API_KEY': '通用 OpenAI API 密钥',
    'NUWAX_PERCEPTION_STREAM_AUTO_START': '启动时自动开启视频感知',
    'NUWAX_PERCEPTION_STREAM_ENABLED': '启用持续视频感知',
    'NUWAX_PERCEPTION_STREAM_INTERVAL_SEC': '视频感知间隔（秒）',
    'NUWAX_PERCEPTION_BURST_FRAME_COUNT': '到点复核帧数',
    'NUWAX_PERCEPTION_BURST_INTERVAL_SEC': '到点复核帧间隔（秒）',
    'NUWAX_PERCEPTION_DEFAULT_DETECTOR_BACKEND': '默认检测后端',
    'NUWAX_PERCEPTION_DEFAULT_SCENE_BACKEND': '默认场景后端',
    'NUWAX_PERCEPTION_DEFAULT_SEGMENTATION_BACKEND': '默认分割后端',
    'NUWAX_PERCEPTION_DELAYED_PC_MAX_CONCURRENCY': '延迟点云最大并发数',
    'NUWAX_PERCEPTION_DELAYED_PC_MAX_RANGE_M': '延迟点云最大距离（米）',
    'NUWAX_PERCEPTION_DELAYED_PC_MAX_RETRY': '延迟点云最大重试次数',
    'NUWAX_PERCEPTION_DELAYED_PC_MIN_POINTS': '延迟点云最少点数',
    'NUWAX_PERCEPTION_DELAYED_PC_REFINE_INTERVAL_SEC': '延迟点云细化间隔（秒）',
    'NUWAX_PERCEPTION_HISTORY_LIMIT': '感知历史记录上限',
    'NUWAX_PERCEPTION_KEYFRAME_MIN_INTERVAL_SEC': '关键帧最小间隔（秒）',
    'NUWAX_PERCEPTION_KEYFRAME_REMEMBER_ENABLED': '启用关键帧记忆',
    'NUWAX_PERCEPTION_KEYFRAME_TRANSLATION_M': '关键帧平移阈值（米）',
    'NUWAX_PERCEPTION_KEYFRAME_YAW_DEG': '关键帧转向阈值（度）',
    'NUWAX_PERCEPTION_POSTPROCESS_NMS_ENABLED': '启用检测 NMS 后处理',
    'NUWAX_PERCEPTION_POSTPROCESS_NMS_IOU': '检测 NMS IOU 阈值',
    'NUWAX_PERCEPTION_STREAM_CAMERA_ID': '持续感知相机 ID',
    'NUWAX_PERCEPTION_STREAM_DETECTOR_BACKEND': '持续感知检测后端',
    'NUWAX_PERCEPTION_STREAM_FAILURE_BACKOFF_SEC': '持续感知失败退避（秒）',
    'NUWAX_PERCEPTION_STREAM_STORE_ARTIFACT': '保存持续感知制品',
    'NUWAX_PERCEPTION_STREAM_SKIP_WHEN_LOCALIZATION': '定位模式下暂停自动感知',
    'NUWAX_PERCEPTION_TRIANGULATION_MAX_RESIDUAL_M': '三角化最大残差（米）',
    'NUWAX_PERCEPTION_TRIANGULATION_MIN_PARALLAX_DEG': '三角化最小视差角（度）',
    'NUWAX_PERCEPTION_TRIANGULATION_MIN_RAYS': '三角化最少射线数',
    'NUWAX_MAP_CATALOG_ROOT': '地图目录',
    'NUWAX_MEMORY_EMBEDDING_API_KEY': '记忆嵌入 API 密钥',
    'DASHSCOPE_API_KEY': 'DashScope API 密钥',
    'NUWAX_ACTIVE_MAP_RESTORE_ON_START': '启动时恢复激活地图',
    'NUWAX_ARTIFACT_CLEANUP_BATCH_SIZE': '制品清理批量大小',
    'NUWAX_ARTIFACT_MAX_COUNT': '制品最大数量',
    'NUWAX_ARTIFACT_MAX_TOTAL_BYTES': '制品总大小上限（字节）',
    'NUWAX_ARTIFACT_RETENTION_DAYS': '制品保留天数',
    'NUWAX_DIAGNOSTIC_HISTORY_LIMIT': '诊断历史记录上限',
    'NUWAX_LOCALIZATION_HISTORY_LIMIT': '定位历史记录上限',
    'NUWAX_MAP_AUTO_CREATE_LIBRARY': '自动创建地图库',
    'NUWAX_MAP_DELETE_ALSO_DELETE_LIBRARY': '删除地图时同时删除地图库',
    'NUWAX_MEMORY_DB_PATH': '记忆数据库路径',
    'NUWAX_MEMORY_EMBEDDING_BASE_URL': '记忆嵌入服务基础地址',
    'NUWAX_MEMORY_EMBEDDING_DIM': '记忆嵌入维度',
    'NUWAX_MEMORY_EMBEDDING_MODEL': '记忆嵌入模型',
    'NUWAX_MEMORY_EMBEDDING_TIMEOUT_SEC': '记忆嵌入超时（秒）',
    'NUWAX_MEMORY_HISTORY_LIMIT': '记忆历史记录上限',
    'NUWAX_MEMORY_IMAGE_EMBEDDING_DIM': '图像记忆嵌入维度',
    'NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL': '图像记忆嵌入模型',
    'NUWAX_MEMORY_TEXT_EMBEDDING_DIM': '文本记忆嵌入维度',
    'NUWAX_MEMORY_TEXT_EMBEDDING_MODEL': '文本记忆嵌入模型',
    'NUWAX_NAVIGATION_HISTORY_LIMIT': '导航历史记录上限',
    'NUWAX_OBSERVATION_HISTORY_LIMIT': '观察历史记录上限',
    'NUWAX_STATE_HISTORY_LIMIT': '状态历史记录上限',
    'GO2_VUI_MAX_LEVEL': 'Go2 VUI 最大音量级别',
    'GO2_VUI_MIN_LEVEL': 'Go2 VUI 最小音量级别',
    'GO2_DATA_PLANE_ENABLED': '启用 Go2 数据面',
    'GO2_DDS_IFACE': 'Go2 DDS 网卡',
    'GO2_DIRECT_DDS_ENABLED': '启用 Go2 直连 DDS',
    'GO2_DATA_PLANE_DDS_IFACE': 'Go2 数据面 DDS 网卡',
    'GO2_DIRECT_DDS_LOW_STATE_TOPIC': 'Go2 低状态 DDS 话题',
    'GO2_DIRECT_DDS_MAX_POINTS_PER_SCAN': 'Go2 单次扫描最大点数',
    'GO2_DIRECT_DDS_POINT_CLOUD_ENABLED': '启用 Go2 DDS 点云',
    'GO2_DIRECT_DDS_POINT_CLOUD_XYZ_REGISTERED_TO_ODOM': 'Go2 点云 XYZ 已注册到 odom',
    'GO2_DIRECT_DDS_POINT_CLOUD_TOPIC': 'Go2 DDS 点云话题',
    'GO2_DIRECT_DDS_POSE_TOPIC': 'Go2 姿态 DDS 话题',
    'GO2_DIRECT_DDS_READ_TIMEOUT_SEC': 'Go2 DDS 读取超时（秒）',
    'GO2_IMU_GYRO_DEADBAND_RPS': 'Go2 IMU 陀螺仪死区（弧度/秒）',
    'GO2_OFFICIAL_BACKEND_ENABLED': '启用 Go2 官方控制后端',
    'GO2_OFFICIAL_OBSTACLE_AVOIDANCE_DEFAULT_ENABLED': '默认启用 Go2 官方避障',
    'GO2_OFFICIAL_CLIENT_TIMEOUT_SEC': 'Go2 官方客户端超时（秒）',
    'GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS': 'Go2 官方最大线速度（米/秒）',
    'GO2_OFFICIAL_MAX_YAW_RATE_RPS': 'Go2 官方最大偏航角速度（弧度/秒）',
    'GO2_OFFICIAL_YAW_RATE_DEADBAND_RPS': 'Go2 官方偏航角速度死区（弧度/秒）',
    'GO2_OFFICIAL_SERVICES_AUTO_START': '自动启动 Go2 官方服务',
    'GO2_OFFICIAL_SERVICE_NAMES': 'Go2 官方服务名称',
    'GO2_VIDEO_STREAM_ENABLED': '启用视频推流',
    'GO2_ROBOT_IP': 'Go2 本体 WebRTC 信令 IPv4 地址',
    'GO2_WEBRTC_AES_128_KEY': 'Go2 WebRTC AES-128 密钥',
    'GO2_VIDEO_STREAM_RECONNECT_SEC': 'Go2 视频重连间隔（秒）',
    'NUWAX_RELAY_ENABLED': '启用中继转发',
    'NUWAX_RELAY_HOST': '中继监听地址',
    'NUWAX_RELAY_INCOMING_TOKENS': '中继入站令牌',
    'NUWAX_RELAY_PORT': '中继端口',
    'NUWAX_RELAY_UPSTREAM_BASE_URL': '中继上游网关地址',
    'NUWAX_RELAY_UPSTREAM_TOKEN': '中继上游令牌',
}


def _zh_name_for_key(key: str) -> str:
    """返回配置 key 的显式中文名；缺失时失败，不做英文拆词兜底。"""

    try:
        return runtime_catalog().get(key, {}).get("zh_name") or _ZH_NAMES[key]
    except KeyError as exc:
        raise KeyError(f"配置键缺少中文名映射: {key}") from exc


def _validate_zh_name_coverage(raw: Dict[str, _RawKey]) -> None:
    """校验显式中文名与 settings 中实际配置 key 的覆盖关系。

    - ``missing``（raw 中有但 _ZH_NAMES 中没有）= 严格 fail，面板会出现机械英文标签。
    - ``stale``（_ZH_NAMES 中有但 raw 中没有）= 容许。多版本/多环境漂移时
      settings.py 的 _cfg_* 引用可能与本仓库不同步（例如 _ZH_NAMES 已经为
      ``NUWAX_GEOMETRY_EXTERNAL_FRONTIER_TOPIC`` 备好中文名，但本地 settings.py
      暂未引用），此时不应让本地 Web 面板因孤儿映射直接 500。
    - ``empty``（中文名为空）= 严格 fail。
    """

    actual_keys = set(raw)
    mapped_keys = set(_ZH_NAMES) | set(runtime_catalog())
    missing = sorted(actual_keys - mapped_keys)
    empty = sorted(key for key, name in _ZH_NAMES.items() if not name.strip())
    if not missing and not empty:
        return

    parts: List[str] = []
    if missing:
        parts.append("缺少中文名映射: " + ", ".join(missing))
    if empty:
        parts.append("中文名映射为空: " + ", ".join(empty))
    raise RuntimeError("配置中文名映射与配置键不一致；" + "；".join(parts))


@lru_cache(maxsize=1)
def build_registry() -> Tuple[ConfigField, ...]:
    """组装完整注册表（板块顺序 → 板块内常规先于高级 → 键名）。"""

    raw = _all_raw_keys()
    _validate_zh_name_coverage(raw)
    descs = _env_example_desc()
    fields: List[ConfigField] = []
    for key, rk in raw.items():
        entry = runtime_catalog().get(key, {})
        section_id = entry.get("section_id", _classify_section(key))
        consumers = entry.get("consumers", [
            "drivers/go2/settings.py" if rk.target_file == TARGET_GO2 else "core/settings.py"
        ])
        fields.append(ConfigField(
            key=key,
            zh_name=_zh_name_for_key(key),
            description=entry.get("description", descs.get(key, "")),
            type=rk.type,
            section_id=section_id,
            section_title=_SECTION_TITLES.get(section_id, "其它"),
            advanced=key not in _REGULAR_KEYS,
            sensitive=key in _SENSITIVE_KEYS,
            target_file=rk.target_file,
            default_repr=rk.default_repr,
            alias_of=rk.alias_of,
            choices=tuple(entry.get("choices", _CHOICES.get(key, ()))),
            choice_labels=tuple(entry.get("choice_labels", _CHOICE_LABELS.get(key, ()))),
            consumers=tuple(consumers),
            aliases=tuple(entry.get("aliases", ())),
            apply_scope=entry.get("apply_scope", "重启机器人数据面及网关后生效" if rk.target_file == TARGET_GO2 else "重启网关后生效"),
        ))
    section_order = {sid: i for i, (sid, _t) in enumerate(SECTIONS)}
    fields.sort(key=lambda f: (section_order.get(f.section_id, 999), f.advanced, f.key))
    return tuple(fields)


def all_keys() -> Tuple[str, ...]:
    return tuple(f.key for f in build_registry())


def get_field(key: str) -> Optional[ConfigField]:
    for f in build_registry():
        if f.key == key:
            return f
    return None


def sensitive_keys() -> frozenset:
    return _SENSITIVE_KEYS
