#!/usr/bin/env python3
"""通过公共网关验证空间关系图发现、查询、路径和图片，不发出机器人运动。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen
import uuid


def run(base_url: str, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    def request(path, data=None):
        raw = json.dumps(data).encode() if data is not None else None
        with urlopen(Request(base_url.rstrip('/') + path, data=raw, headers={'Content-Type': 'application/json'}), timeout=30) as response:
            return json.load(response)
    def capability(name, arguments):
        result = request('/api/capabilities/' + name + '/invoke', {'arguments': arguments})
        return result.get('result', result)
    tools = request('/api/agent/tools')['tools']
    names = ['get_scene_graph', 'get_scene_graph_node', 'plan_scene_graph_route', 'render_scene_graph_image']
    for name in names:
        found = next(item for item in tools if item['descriptor']['name'] == name)
        assert found['supported'] and found['runnable']
    mcp = request('/mcp', {'jsonrpc': '2.0', 'id': 1, 'method': 'tools/list', 'params': {}})
    assert all(name in json.dumps(mcp) for name in names)
    graph = capability('get_scene_graph', {'limit': 500, 'kinds': ['region', 'passage', 'object']})
    revision = graph['graph_revision']
    regions = [n for n in graph['nodes'] if n['kind'] == 'region']
    assert len(regions) >= 2
    detail = capability('get_scene_graph_node', {'node_id': regions[0]['node_id'], 'expected_graph_revision': revision})
    assert detail['node']['properties']['boundary']['polygon_points']
    route = capability('plan_scene_graph_route', {'from_node_id': regions[0]['node_id'], 'to_node_id': regions[-1]['node_id'], 'expected_graph_revision': revision})
    assert route['graph_revision'] == revision and not route['executable'] and not route['navigation_checked']
    invalid_status = None
    try:
        capability('plan_scene_graph_route', {'from_node_id': regions[0]['node_id'], 'to_node_id': regions[-1]['node_id'], 'expected_graph_revision': 'old'})
    except HTTPError as exc:
        invalid_status = exc.code
    assert invalid_status == 409
    request_id = 'scene_graph_smoke_' + uuid.uuid4().hex
    receipt = request('/api/agent/invoke', {'request_id': request_id, 'tool_name': 'get_scene_graph', 'arguments': {'node_id': regions[0]['node_id'], 'hops': 2, 'limit': 50}})
    assert receipt['delivery_state'] == 'recorded' and receipt['error'] is None
    full = request('/api/agent/calls/' + request_id + '/result')
    assert revision in json.dumps(full)
    image = capability('render_scene_graph_image', {'expected_graph_revision': revision, 'width': 1280})
    metadata = request('/artifacts/' + image['artifact']['artifact_id'] + '/meta')
    assert revision in json.dumps(metadata) and image['image_is_new_observation'] is False
    times = []
    for _ in range(5):
        start = time.perf_counter()
        current = capability('get_scene_graph', {'limit': 100})
        times.append(time.perf_counter() - start)
        assert current['graph_revision'] == revision
    report = {'base_url': base_url, 'tools_discovered': names, 'mcp_discovery': True, 'agent_actual_call': receipt,
              'graph_revision': revision, 'coverage': graph['coverage'], 'route': route,
              'stale_revision_status': invalid_status, 'image': image,
              'warm_query_median_ms': round(statistics.median(times) * 1000, 2),
              'query_samples_ms': [round(t * 1000, 2) for t in times], 'robot_motion_requested': False}
    (output_dir / '公共接口验证.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    (output_dir / '实录关系图.json').write_text(json.dumps(graph, ensure_ascii=False, indent=2) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-url', default='http://127.0.0.1:18792')
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    result = run(args.base_url, args.output_dir)
    print(json.dumps({key: result[key] for key in ('coverage', 'stale_revision_status', 'warm_query_median_ms', 'robot_motion_requested')}, ensure_ascii=False, indent=2))
