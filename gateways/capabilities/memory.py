"""记忆与语义地图能力；所有入口共享同一能力表。"""
from __future__ import annotations

from contracts.geometry import Pose, Quaternion, Vector3
from contracts.memory import MemoryPayloadFilter
from core.capability import Capability


SCHEMAS = {'get_navigation_semantic_context': {'type': 'object',
                                     'properties': {},
                                     'required': [],
                                     'additionalProperties': False},
 'resolve_navigation_candidates': {'type': 'object',
                                   'properties': {'query': {'type': 'string',
                                                            'description': '要定位的地点、区域或物品的名称或自然语言描述。'},
                                                  'camera_id': {'type': 'string',
                                                                'description': '随候选诊断信息返回的相机标识。留空时不记录该标识。'},
                                                  'similarity_threshold': {'type': 'number',
                                                                           'minimum': 0.0,
                                                                           'maximum': 1.0,
                                                                           'default': 0.25,
                                                                           'description': '候选结果的最低相似度（0～1）。留空时为 0.25。'},
                                                  'limit': {'type': 'integer', 'minimum': 1, 'default': 5,
                                                            'description': '最多返回的结果数。留空时最多返回 5 条。'},
                                                  'map_name': {'type': 'string',
                                                               'description': '候选所属的地图名称。留空时优先使用当前语义地图。'},
                                                  'max_age_sec': {'type': 'number', 'minimum': 0.0,
                                                                  'description': '只保留此秒数内的记录。留空或填 0 时不限制记录时间。'}},
                                   'required': ['query'],
                                   'additionalProperties': False},
 'list_memory_libraries': {'type': 'object', 'properties': {}, 'required': [], 'additionalProperties': False},
 'delete_memory_library': {'type': 'object',
                           'properties': {'library_name': {'type': 'string', 'minLength': 1, 'pattern': r'\S',
                                                           'description': '要删除的记忆库名称。'}},
                           'required': ['library_name'], 'additionalProperties': False},
 'tag_location': {'type': 'object',
                  'properties': {'name': {'type': 'string',
                                          'description': '为当前位置标记的地点名称。'},
                                 'aliases': {'type': 'array',
                                             'description': '地点的其他叫法。留空时不添加别名。'},
                                 'description': {'type': 'string',
                                                 'description': '地点的文字描述。留空时不填写自定义描述。'},
                                 'camera_id': {'type': 'string',
                                               'description': '记录地点时使用的相机标识。留空时使用默认相机。'},
                                 'auto_create_memory': {'type': 'boolean', 'default': True,
                                                        'description': '是否同时生成场景记忆。留空时生成。'}},
                  'required': ['name'],
                  'additionalProperties': False},
 'query_location': {'type': 'object',
                    'properties': {'query': {'type': 'string',
                                             'description': '用于检索已标记地点的名称、别名或描述。'},
                                   'similarity_threshold': {'type': 'number',
                                                            'minimum': 0.0,
                                                            'maximum': 1.0,
                                                            'default': 0.55,
                                                            'description': '地点候选的最低相似度（0～1）。留空时为 0.55。'},
                                   'limit': {'type': 'integer', 'minimum': 1, 'default': 5,
                                             'description': '最多返回的结果数。留空时最多返回 5 条。'},
                                   'map_name': {'type': 'string',
                                                'description': '按地图名称筛选。留空时不加地图名称筛选条件。'},
                                   'camera_id': {'type': 'string',
                                                 'description': '按来源相机标识筛选。留空时不限相机。'},
                                   'semantic_labels': {'type': 'array',
                                                       'description': '语义标签列表，匹配任一标签即可。留空时不限语义标签。'},
                                   'visual_labels': {'type': 'array',
                                                     'description': '视觉检测标签列表，匹配任一标签即可。留空时不限视觉标签。'},
                                   'vision_tags': {'type': 'array',
                                                   'description': '视觉标签列表，匹配任一标签即可。留空时不限此项。'},
                                   'topo_node_id': {'type': 'string',
                                                    'description': '关联的拓扑节点标识。留空时不限节点。'},
                                   'max_age_sec': {'type': 'number', 'minimum': 0.0,
                                                   'description': '只保留此秒数内的记录。留空或填 0 时不限制记录时间。'},
                                   'near_frame_id': {'type': 'string',
                                                     'description': '距离筛选参考点的坐标系。提供参考坐标时留空使用地图坐标系 map。'},
                                   'near_x': {'type': 'number',
                                              'description': '距离筛选参考点的 X 坐标（米）。使用参考点时留空取 0。'},
                                   'near_y': {'type': 'number',
                                              'description': '距离筛选参考点的 Y 坐标（米）。使用参考点时留空取 0。'},
                                   'near_z': {'type': 'number', 'default': 0.0,
                                              'description': '距离筛选参考点的 Z 坐标（米）。留空时为 0。'},
                                   'max_distance_m': {'type': 'number', 'minimum': 0.0,
                                                      'description': '与参考点的最大距离（米）。留空或填 0 时不限制距离。'}},
                    'required': ['query'],
                    'additionalProperties': False},
 'query_semantic_memory': {'type': 'object',
                           'properties': {'query': {'type': 'string',
                                                    'description': '用于检索场景记忆、物品和观察记录的自然语言描述。'},
                                          'similarity_threshold': {'type': 'number',
                                                                   'minimum': 0.0,
                                                                   'maximum': 1.0,
                                                                   'default': 0.25,
                                                                   'description': '候选结果的最低相似度（0～1）。留空时为 0.25。'},
                                          'limit': {'type': 'integer', 'minimum': 1, 'default': 5,
                                                    'description': '最多返回的结果数。留空时最多返回 5 条。'},
                                          'map_name': {'type': 'string',
                                                       'description': '按地图名称筛选。留空时不加地图名称筛选条件。'},
                                          'camera_id': {'type': 'string',
                                                        'description': '按来源相机标识筛选。留空时不限相机。'},
                                          'linked_location_id': {'type': 'string',
                                                                 'description': '关联的命名地点标识。留空时不限地点。'},
                                          'semantic_labels': {'type': 'array',
                                                              'description': '语义标签列表，匹配任一标签即可。留空时不限语义标签。'},
                                          'visual_labels': {'type': 'array',
                                                            'description': '视觉检测标签列表，匹配任一标签即可。留空时不限视觉标签。'},
                                          'vision_tags': {'type': 'array',
                                                          'description': '视觉标签列表，匹配任一标签即可。留空时不限此项。'},
                                          'topo_node_id': {'type': 'string',
                                                           'description': '关联的拓扑节点标识。留空时不限节点。'},
                                          'max_age_sec': {'type': 'number', 'minimum': 0.0,
                                                          'description': '只保留此秒数内的记录。留空或填 0 时不限制记录时间。'},
                                          'near_frame_id': {'type': 'string',
                                                            'description': '距离筛选参考点的坐标系。提供参考坐标时留空使用地图坐标系 map。'},
                                          'near_x': {'type': 'number',
                                                     'description': '距离筛选参考点的 X 坐标（米）。使用参考点时留空取 0。'},
                                          'near_y': {'type': 'number',
                                                     'description': '距离筛选参考点的 Y 坐标（米）。使用参考点时留空取 0。'},
                                          'near_z': {'type': 'number', 'default': 0.0,
                                                     'description': '距离筛选参考点的 Z 坐标（米）。留空时为 0。'},
                                          'max_distance_m': {'type': 'number', 'minimum': 0.0,
                                                             'description': '与参考点的最大距离（米）。留空或填 0 时不限制距离。'},
                                          'include_removed': {'type': 'boolean', 'default': False,
                                                              'description': '是否返回已被搬走（removed）的物品实例。留空时为 false，查历史原位置时填 true。'}},
                           'required': ['query'],
                           'additionalProperties': False},
 'list_scene_spaces': {'type': 'object',
                       'properties': {'limit': {'type': 'integer', 'minimum': 1, 'default': 20,
                                                'description': '最多返回的场景空间数量。留空时最多返回 20 个。'},
                                      'scene_space_id': {'type': 'string',
                                                         'description': '场景空间标识。留空时不限空间。'},
                                      'scene_space_ids': {'type': 'array', 'items': {'type': 'string'},
                                                          'description': '场景空间标识列表，匹配任一标识即可。留空时不限列表。'},
                                      'map_name': {'type': 'string',
                                                   'description': '按地图名称筛选。留空时不加地图名称筛选条件。'},
                                      'review_status': {'type': 'string',
                                                        'description': '按单个审核状态筛选。留空时不限此项。'},
                                      'review_statuses': {'type': 'array', 'items': {'type': 'string'},
                                                          'description': '审核状态列表，匹配任一状态即可。留空时不限列表。'}},
                       'required': [],
                       'additionalProperties': False},
 'get_scene_space': {'type': 'object',
                     'properties': {'scene_space_id': {'type': 'string',
                                                       'description': '要读取的场景空间标识。'}},
                     'required': ['scene_space_id'],
                     'additionalProperties': False},
 'query_scene_spaces': {'type': 'object',
                        'properties': {'query': {'type': 'string',
                                                 'description': '用于检索场景空间的名称或自然语言描述。'},
                                       'similarity_threshold': {'type': 'number',
                                                                'minimum': 0.0,
                                                                'maximum': 1.0,
                                                                'default': 0.25,
                                                                'description': '候选结果的最低相似度（0～1）。留空时为 0.25。'},
                                       'limit': {'type': 'integer', 'minimum': 1, 'default': 5,
                                                 'description': '最多返回的结果数。留空时最多返回 5 条。'},
                                       'map_name': {'type': 'string',
                                                    'description': '按地图名称筛选。留空时不加地图名称筛选条件。'},
                                       'camera_id': {'type': 'string',
                                                     'description': '按来源相机标识筛选。留空时不限相机。'},
                                       'semantic_labels': {'type': 'array',
                                                           'description': '语义标签列表，匹配任一标签即可。留空时不限语义标签。'},
                                       'visual_labels': {'type': 'array',
                                                         'description': '视觉检测标签列表，匹配任一标签即可。留空时不限视觉标签。'},
                                       'vision_tags': {'type': 'array',
                                                       'description': '视觉标签列表，匹配任一标签即可。留空时不限此项。'},
                                       'topo_node_id': {'type': 'string',
                                                        'description': '关联的拓扑节点标识。留空时不限节点。'},
                                       'anchor_id': {'type': 'string',
                                                     'description': '关联的空间锚点标识。留空时不限锚点。'},
                                       'scene_space_id': {'type': 'string',
                                                          'description': '场景空间标识。留空时不限空间。'},
                                       'scene_space_ids': {'type': 'array', 'items': {'type': 'string'},
                                                           'description': '场景空间标识列表，匹配任一标识即可。留空时不限列表。'},
                                       'review_status': {'type': 'string',
                                                         'description': '按单个审核状态筛选。留空时不限此项。'},
                                       'review_statuses': {'type': 'array', 'items': {'type': 'string'},
                                                           'description': '审核状态列表，匹配任一状态即可。留空时不限列表。'},
                                       'max_age_sec': {'type': 'number', 'minimum': 0.0,
                                                       'description': '只保留此秒数内的记录。留空或填 0 时不限制记录时间。'},
                                       'near_frame_id': {'type': 'string',
                                                         'description': '距离筛选参考点的坐标系。提供参考坐标时留空使用地图坐标系 map。'},
                                       'near_x': {'type': 'number',
                                                  'description': '距离筛选参考点的 X 坐标（米）。使用参考点时留空取 0。'},
                                       'near_y': {'type': 'number',
                                                  'description': '距离筛选参考点的 Y 坐标（米）。使用参考点时留空取 0。'},
                                       'near_z': {'type': 'number', 'default': 0.0,
                                                  'description': '距离筛选参考点的 Z 坐标（米）。留空时为 0。'},
                                       'max_distance_m': {'type': 'number', 'minimum': 0.0,
                                                          'description': '与参考点的最大距离（米）。留空或填 0 时不限制距离。'},
                                       'include_non_direct_navigation_candidates': {'type': 'boolean',
                                                                                    'default': False,
                                                                                    'description': '是否附带仅供参考的导航候选。留空时不附带此类候选。'}},
                        'required': ['query'],
                        'additionalProperties': False},
 'review_scene_space': {'type': 'object',
                        'properties': {'scene_space_id': {'type': 'string',
                                                          'description': '待审核的场景空间标识。'},
                                       'primary_label': {'type': 'string',
                                                         'description': '人工确认的主名称。留空时保留原名称。'},
                                       'human_aliases': {'type': 'array', 'items': {'type': 'string'},
                                                         'description': '审核后的人类别名列表。留空时保留原列表，传空列表可清除别名。'},
                                       'review_status': {'type': 'string',
                                                         'description': '更新后的审核状态。留空时保留原状态。'},
                                       'attributes': {'type': 'object',
                                                      'description': '需要合并到场景空间的属性。留空时保留原属性。'},
                                       'metadata': {'type': 'object',
                                                    'description': '需要合并到场景空间的附加信息。留空时不提供自定义信息。'}},
                        'required': ['scene_space_id'],
                        'additionalProperties': False},
 'remember_current_scene': {'type': 'object',
                            'properties': {'title': {'type': 'string',
                                                     'description': '记忆条目的标题。留空时根据采集的观察自动命名。'},
                                           'camera_id': {'type': 'string',
                                                         'description': '采集场景时使用的相机标识。留空时使用默认相机。'},
                                           'summary': {'type': 'string',
                                                       'description': '自定义场景摘要。留空时使用观察摘要。'}},
                            'required': [],
                            'additionalProperties': False},
 'list_semantic_instances': {'type': 'object',
                             'properties': {'library_name': {'type': 'string',
                                                             'description': '要读取的记忆库名称。留空时使用当前库。'}, 'label': {'type': 'string',
                                                                                          'description': '用于筛选实例的物品标签。留空时不限标签。'}},
                             'required': [],
                             'additionalProperties': False},
 'get_scene_mapping_status': {'type': 'object',
                              'properties': {},
                              'required': [],
                              'additionalProperties': False},
 'start_scene_mapping': {'type': 'object', 'properties': {}, 'required': [], 'additionalProperties': False},
 'stop_scene_mapping': {'type': 'object', 'properties': {}, 'required': [], 'additionalProperties': False},
 'refresh_semantic_map': {'type': 'object',
                          'properties': {'apply_labels': {'type': 'boolean',
                                                          'description': '是否自动写入达到门槛的候选名称。留空时沿用语义地图自动写入设置。'},
                                         'timeout_sec': {'type': 'number', 'minimum': 1, 'maximum': 1800,
                                                         'description': '刷新任务的总时限（秒）。留空时为 300 秒。'}},
                          'required': [],
                          'additionalProperties': False},
 'get_semantic_map': {'type': 'object', 'properties': {}, 'required': [], 'additionalProperties': False},
 'get_scene_graph': {'type': 'object',
                     'properties': {'query': {'type': 'string', 'maxLength': 200,
                                              'description': '用于筛选节点的关键词。留空时不限关键词。'},
                                    'node_id': {'type': 'string',
                                                'description': '展开邻域时的起点节点标识。留空时不限制邻域。'},
                                    'hops': {'type': 'integer', 'minimum': 0, 'maximum': 4,
                                             'description': '从起点向外展开的连接层数。留空时为 1 层，0 表示只看起点。'},
                                    'kinds': {'type': 'array',
                                              'items': {'type': 'string',
                                                        'enum': ['region',
                                                                 'passage',
                                                                 'object',
                                                                 'observation']},
                                              'maxItems': 4,
                                              'description': '要保留的节点类型列表。留空时不限类型。'},
                                    'limit': {'type': 'integer', 'minimum': 1, 'maximum': 500,
                                              'description': '最多返回的节点数量。留空时最多返回 100 个。'},
                                    'expected_graph_revision': {'type': 'string',
                                                                'description': '预期的关系图版本。留空时直接使用当前版本。'}},
                     'required': [],
                     'additionalProperties': False},
 'get_scene_graph_node': {'type': 'object',
                          'properties': {'node_id': {'type': 'string',
                                                     'description': '要读取的关系图节点标识。'},
                                         'expected_graph_revision': {'type': 'string',
                                                                     'description': '预期的关系图版本。留空时直接使用当前版本。'}},
                          'required': ['node_id'],
                          'additionalProperties': False},
 'plan_scene_graph_route': {'type': 'object',
                            'properties': {'from_node_id': {'type': 'string',
                                                            'description': '路线起点的节点标识。'},
                                           'to_node_id': {'type': 'string',
                                                          'description': '路线终点的节点标识。'},
                                           'expected_graph_revision': {'type': 'string', 'minLength': 1,
                                                                       'description': '规划所依据的关系图版本，应使用查询返回的版本。'}},
                            'required': ['from_node_id', 'to_node_id', 'expected_graph_revision'],
                            'additionalProperties': False},
 'render_scene_graph_image': {'type': 'object',
                              'properties': {'node_id': {'type': 'string',
                                                         'description': '局部图的中心节点标识。留空时渲染全图。'},
                                             'expected_graph_revision': {'type': 'string',
                                                                         'description': '预期的关系图版本。留空时直接使用当前版本。'},
                                             'width': {'type': 'integer', 'minimum': 512, 'maximum': 2048,
                                                       'description': '输出图片的宽度（像素）。留空时为 1280 像素。'}},
                              'required': [],
                              'additionalProperties': False},
 'locate_current_scene_space': {'type': 'object',
                                'properties': {},
                                'required': [],
                                'additionalProperties': False},
 'update_scene_space_label': {'type': 'object',
                              'properties': {'scene_space_id': {'type': 'string',
                                                                'description': '待改名的场景空间标识。'},
                                             'map_name': {'type': 'string',
                                                          'description': '该场景空间所属地图的名称。'},
                                             'expected_revision': {'type': 'integer', 'minimum': 1,
                                                                   'description': '当前场景空间的修订号，应使用读取空间时返回的值。'},
                                             'label': {'type': 'string', 'minLength': 1, 'maxLength': 120,
                                                       'description': '要写入的新名称。'}},
                              'required': ['scene_space_id', 'map_name', 'expected_revision', 'label'],
                              'additionalProperties': False}}

DESCRIPTIONS = {'get_navigation_semantic_context': '读取当前地图绑定的语义区域、拓扑节点、场景空间候选和证据摘要。',
 'resolve_navigation_candidates': '按自然语言查询只返回记忆和语义空间导航候选及证据，不提交导航任务。',
 'list_memory_libraries': '列出当前可用的命名记忆库及其摘要，供上层选择正确记忆。',
 'tag_location': '把当前位姿与最新观察信息写入命名地点记忆，可供后续命名导航复用。',
 'query_location': '按名称、别名或描述查询已标记地点。',
 'query_semantic_memory': '按自然语言检索近期场景记忆、物品实例和观察。默认不返回已被搬走（removed）的物品实例；查历史原位置时传 include_removed=true。',
 'list_scene_spaces': '列出当前记忆库内的场景空间，可按地图名称、审核状态或空间标识筛选。',
 'get_scene_space': '按场景空间标识读取单个场景空间及其证据。',
 'query_scene_spaces': '按自然语言和过滤条件检索场景空间，只返回候选、分数和证据。',
 'review_scene_space': '人工审核或纠偏场景空间标签、别名、状态和属性，不创建导航决策。',
 'remember_current_scene': '把当前观察或感知结果写入语义记忆。',
 'list_semantic_instances': '返回记忆库中已持久化的语义实例列表（含 pose/frame_id/confidence/is_fallback）。',
 'get_scene_mapping_status': '读取语义后台及最近处理结果，查询不触发模型加载。',
 'start_scene_mapping': '启动受管后台，基于当前地图和已记录观察自动产生区域与标签候选。',
 'stop_scene_mapping': '请求后台停止，迟到结果拒收；stop_pending表示仍在等待在途调用退出。',
 'refresh_semantic_map': '异步提取几何区域并用配置模型评估；不会启动机器人运动。',
 'get_semantic_map': '读取当前地图的区域边界、标签、候选和版本；过期几何明确标注。',
 'get_scene_graph': '检索区域、连接口、物品和观察节点；按节点展开邻域，返回图版本和明确截断信息。',
 'get_scene_graph_node': '读取节点坐标、精确范围、邻接关系和可用图像引用；图像需通过制品接口实际读取。',
 'plan_scene_graph_route': '按最少连接口数量查询区域链及出入口位置；基于二维几何连接，实际机器人通行未检查，不执行运动。',
 'render_scene_graph_image': '将同一图版本渲染为带区域编号和关系的地图图片，返回制品和编号对照；可指定节点取得局部图。',
 'locate_current_scene_space': '按已有定位和精确区域边界判断归属，未知和歧义分别返回。',
 'update_scene_space_label': '按预期修订设置区域名称并保护该显式修改，不伪造人工审核或导航就绪。'}


def _text(args, key):
    value = args.get(key)
    return value.strip() or None if value is not None else None


def _strings(args, plural, singular=None):
    values = [str(item).strip() for item in args.get(plural, []) if str(item or "").strip()]
    if singular is not None and _text(args, singular):
        values.insert(0, _text(args, singular))
    return list(dict.fromkeys(values))


def _payload_filter(args):
    near_pose = None
    if any(key in args for key in ("near_frame_id", "near_x", "near_y")):
        near_pose = Pose(frame_id=args.get("near_frame_id") or "map",
            position=Vector3(x=args.get("near_x", 0), y=args.get("near_y", 0), z=args.get("near_z", 0)),
            orientation=Quaternion(w=1))
    return MemoryPayloadFilter(map_name=_text(args, "map_name"),
        linked_location_id=_text(args, "linked_location_id"),
        camera_ids=[args["camera_id"].strip()] if _text(args, "camera_id") else [],
        semantic_labels_any=_strings(args, "semantic_labels"), visual_labels_any=_strings(args, "visual_labels"),
        vision_tags_any=_strings(args, "vision_tags"), topo_node_ids=_strings(args, "topo_node_ids", "topo_node_id"),
        anchor_ids=_strings(args, "anchor_ids", "anchor_id"),
        scene_space_ids=_strings(args, "scene_space_ids", "scene_space_id"),
        review_statuses=_strings(args, "review_statuses", "review_status"),
        max_age_sec=args.get("max_age_sec") or None, near_pose=near_pose,
        max_distance_m=args.get("max_distance_m") or None)


def list_memory_libraries(platform, args):
    return {"libraries": platform.memory.list_memory_libraries(), "memory_summary": platform.memory.get_summary()}


def delete_memory_library(platform, args):
    result = platform.memory.delete_library(args["library_name"])
    return {"delete_result": result, "libraries": platform.memory.list_memory_libraries(),
            "memory_summary": platform.memory.get_summary()}


def _current_observation_inputs(platform, *, camera_id):
    observation = platform.observation.capture_observation(
        camera_id=camera_id, requested_by="memory_command")
    image = observation.image_artifact
    return {"map_name": observation.metadata["map_name"], "observation_pose": platform.geometry_state.pose(),
            "observation_context": observation,
            "image_bytes": platform.artifacts.read_bytes(image.artifact_id) if image is not None else None}


def remember_sample(platform, sample, *, title=None, summary=None, tags=None, metadata=None):
    """显式传递观察时的地图、位姿、坐标树和图片，不重新采集。"""
    sample = platform.perception.video_runtime.complete_sample(sample)
    return platform.memory.remember_current_scene(map_name=sample.map_name,
        title=title, camera_id=sample.context.camera_id, summary_override=summary,
        tags=tags, metadata=metadata, perception_context=sample.context,
        observation_pose=sample.pose, frame_tree=sample.frame_tree, image_bytes=sample.image_bytes)


def save_sample_photo(platform, sample):
    """复核可读取这一帧图片；业务观察在复核后写入。"""
    artifact = sample.context.image_artifact
    if artifact is not None and sample.image_bytes is not None:
        return platform.memory.save_photo(artifact.artifact_id, sample.image_bytes,
                                         map_name=sample.map_name, mime_type=artifact.mime_type)
    return None


def tag_location(platform, args):
    inputs = _current_observation_inputs(platform, camera_id=_text(args, "camera_id"))
    values = platform.memory.tag_location(args["name"].strip(), **inputs, aliases=args.get("aliases"),
        description=_text(args, "description"), camera_id=inputs["observation_context"].camera_id,
        auto_create_memory=args.get("auto_create_memory", True), attach_latest_perception=False,
        attach_latest_observation=False, metadata={"source": "tag_location"})
    return {"tagged_location": values[0], "semantic_memory": values[1]}


def remember_current_scene(platform, args):
    inputs = _current_observation_inputs(platform, camera_id=_text(args, "camera_id"))
    return {"semantic_memory": platform.memory.remember_current_scene(
        **inputs, title=_text(args, "title"), camera_id=inputs["observation_context"].camera_id,
        summary_override=_text(args, "summary"), metadata={"source": "remember_current_scene"})}


def query_location(platform, args):
    return {"query_result": platform.memory.query_location(args["query"].strip(),
        similarity_threshold=args.get("similarity_threshold", .55), limit=args.get("limit", 5),
        payload_filter=_payload_filter(args))}


def query_semantic_memory(platform, args):
    return {"query_result": platform.memory.query_semantic_memory(args["query"].strip(),
        similarity_threshold=args.get("similarity_threshold", .25), limit=args.get("limit", 5),
        payload_filter=_payload_filter(args), include_removed=args.get("include_removed", False))}


def list_scene_spaces(platform, args):
    return {"scene_spaces": platform.memory.list_scene_spaces(limit=args.get("limit", 20),
        payload_filter=_payload_filter(args)), "memory_summary": platform.memory.get_summary()}


def get_scene_space(platform, args):
    return {"scene_space": platform.memory.get_scene_space(args["scene_space_id"].strip())}


def query_scene_spaces(platform, args):
    return {"query_result": platform.memory.query_scene_spaces(args["query"].strip(),
        similarity_threshold=args.get("similarity_threshold", .25), limit=args.get("limit", 5),
        payload_filter=_payload_filter(args),
        include_non_direct_navigation_candidates=args.get("include_non_direct_navigation_candidates", False))}


def review_scene_space(platform, args):
    return {"scene_space": platform.memory.review_scene_space(args["scene_space_id"].strip(),
        map_name=platform.memory.current_library,
        primary_label=_text(args, "primary_label") if "primary_label" in args else None,
        human_aliases=args.get("human_aliases"), review_status=_text(args, "review_status"),
        attributes=args.get("attributes"), metadata={**args.get("metadata", {}), "requested_by": "unknown"})}


def list_semantic_instances(platform, args):
    return {"instances": platform.memory.list_semantic_instances(
        library_name=args["library_name"].strip() if "library_name" in args else None,
        label=_text(args, "label"))}


def get_navigation_semantic_context(platform, args):
    return {"navigation_semantic_context": platform.memory.get_navigation_semantic_context()}


def resolve_navigation_candidates(platform, args):
    return {"navigation_candidates": platform.memory.resolve_navigation_candidates(args["query"].strip(),
        camera_id=_text(args, "camera_id"), similarity_threshold=args.get("similarity_threshold", .25),
        max_age_sec=args.get("max_age_sec") or None, map_name=_text(args, "map_name"), limit=args.get("limit", 5))}


def get_scene_mapping_status(platform, args):
    return platform.scene_mapping.status()


def start_scene_mapping(platform, args):
    return platform.scene_mapping.start()


def stop_scene_mapping(platform, args):
    return platform.scene_mapping.stop()


def refresh_semantic_map(platform, ctx, args):
    ctx.check()
    ctx.progress(0, "semantic_mapping", "正在提取区域及核对观察证据。")
    result = platform.scene_mapping.run_once(apply_labels=args.get("apply_labels"), cancel_check=ctx.check,
                                            expected_capture=args["_scene_mapping_capture"])
    ctx.check()
    return result


def prepare_semantic_map(platform, args):
    if not platform.scene_mapping.config.enabled:
        from core.errors import GatewayError
        raise GatewayError("请先在设置中启用语义建图。", error_code="scene_mapping_disabled", http_status=409)
    args["_scene_mapping_capture"] = platform.scene_mapping.capture()


def get_semantic_map(platform, args):
    return platform.scene_mapping.get_map()


def get_scene_graph(platform, args):
    return platform.scene_mapping.graph.query(**args)


def get_scene_graph_node(platform, args):
    return platform.scene_mapping.graph.node(**args)


def plan_scene_graph_route(platform, args):
    return platform.scene_mapping.graph.route(**args)


def render_scene_graph_image(platform, args):
    return platform.scene_mapping.graph.image(**args)


def locate_current_scene_space(platform, args):
    return platform.scene_mapping.locate()


def update_scene_space_label(platform, args):
    return {"scene_space": platform.scene_mapping.update_label(**args, requested_by="unknown")}


# 参数合法性由 invoke 按这份能力表统一检查。
from contracts.spatial_memory import SceneSpaceReviewStatus

for name in ("tag_location", "query_location", "query_semantic_memory", "get_scene_space",
             "query_scene_spaces", "review_scene_space", "resolve_navigation_candidates"):
    for field in SCHEMAS[name]["required"]:
        if SCHEMAS[name]["properties"][field].get("type") == "string":
            SCHEMAS[name]["properties"][field].update(minLength=1, pattern=r"\S")
for value in SCHEMAS.values():
    for name in ("review_status", "review_statuses"):
        if name in value["properties"]:
            target = value["properties"][name]
            if name == "review_statuses":
                target = target["items"]
            target["enum"] = [status.value for status in SceneSpaceReviewStatus]

SCHEMAS["refresh_semantic_map"]["properties"]["timeout_sec"]["default"] = 300
DESCRIPTIONS["delete_memory_library"] = "删除记忆库；当前库立即重建为空库并继续记录。"

CAPABILITIES = [
    Capability(name, DESCRIPTIONS[name], input_schema, "task" if name == "refresh_semantic_map" else
        "action" if name in {"tag_location", "remember_current_scene", "review_scene_space", "delete_memory_library",
                            "start_scene_mapping", "stop_scene_mapping", "render_scene_graph_image",
                            "update_scene_space_label"} else "query",
        globals()[name], agent_tool=name not in {"review_scene_space", "list_semantic_instances"},
        timeout_sec=300 if name == "refresh_semantic_map" else None,
        normalize_names=tuple(key for key in ("map_name", "library_name") if key in input_schema["properties"]),
        before_submit=prepare_semantic_map if name == "refresh_semantic_map" else None)
    for name, input_schema in SCHEMAS.items()
]
