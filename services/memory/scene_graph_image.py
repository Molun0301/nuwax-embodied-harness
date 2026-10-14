"""从同一关系图快照渲染语义地图图片，保留编号、坐标映射与来源。"""
from __future__ import annotations

import io
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from contracts.occupancy_cells import PackedOccupancyCells
from core.errors import GatewayError
from services.memory.room_segmentation import _local


def _font(size):
    for path, chinese in (("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", True),
                          ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", False)):
        if Path(path).is_file():
            return ImageFont.truetype(path, size), chinese
    return ImageFont.load_default(), False


def _display_cells(occupancy, labels, size):
    """Sample only presentation pixels, using Pillow's exact nearest-neighbor choices."""
    height, width = occupancy.shape
    if size[0] >= width and size[1] >= height:
        return occupancy, labels
    # Let Pillow choose indices: hand-written ratios can differ at rounding boundaries.
    cols = np.asarray(Image.fromarray(np.arange(width, dtype=np.int32)[None, :]).resize(
        (size[0], 1), Image.Resampling.NEAREST))[0]
    rows = np.asarray(Image.fromarray(np.arange(height, dtype=np.int32)[:, None]).resize(
        (1, size[1]), Image.Resampling.NEAREST))[:, 0]
    # The renderer flips vertically after colorization; sample in that same orientation.
    rows = height - 1 - rows[::-1]
    selection = np.ix_(rows, cols)
    return occupancy[selection], labels[selection]


def render_scene_graph_image(index, snapshot, *, node_id=None, width=1280):
    if not isinstance(width, int) or isinstance(width, bool) or not 512 <= width <= 2048:
        raise GatewayError("图像宽度应为512至2048像素。", error_code="invalid_params", http_status=422)
    grid, geometry = snapshot.occupancy_grid, index.geometry
    height = max(512, min(1536, round(width * grid.height / grid.width) + 140))
    left, bottom, right, top = 0, 0, grid.width, grid.height
    focus = index.nodes.get(node_id)
    focus_region = focus.node_id if focus and focus.kind == "region" else focus.properties.get("region_id") if focus else None
    if focus and focus.position is None and focus_region is None:
        raise GatewayError("该节点没有可定位的地图位置，无法生成局部图。", error_code="scene_graph_position_unknown", http_status=409)
    if focus_region in geometry.region_ids:
        label = geometry.region_ids.index(focus_region) + 1
        rows, cols = np.where(geometry.labels == label)
        if len(rows):
            padding = max(10, round(2.0 / grid.resolution_m))
            left, right = max(0, int(cols.min()) - padding), min(grid.width, int(cols.max()) + padding + 1)
            bottom, top = max(0, int(rows.min()) - padding), min(grid.height, int(rows.max()) + padding + 1)
    elif focus and focus.position:
        cx, cy = _local(focus.position.x, focus.position.y, geometry.origin, geometry.resolution)
        padding = max(10, round(5.0 / grid.resolution_m))
        left, right = max(0, int(cx) - padding), min(grid.width, int(cx) + padding + 1)
        bottom, top = max(0, int(cy) - padding), min(grid.height, int(cy) + padding + 1)
    if right <= left or top <= bottom:
        raise GatewayError("所选节点没有当前地图内可渲染的位置。", error_code="scene_graph_position_unknown", http_status=409)
    data = np.frombuffer(grid.data.raw, dtype=np.int8) if isinstance(grid.data, PackedOccupancyCells) else np.asarray(grid.data, dtype=np.int8)
    occupancy = data.reshape(grid.height, grid.width)[bottom:top, left:right]
    labels = geometry.labels[bottom:top, left:right]
    scale = min((width - 48) / (right - left), (height - 144) / (top - bottom))
    draw_width, draw_height = max(1, round((right - left) * scale)), max(1, round((top - bottom) * scale))
    occupancy, labels = _display_cells(occupancy, labels, (draw_width, draw_height))
    palette = np.array([[236, 239, 242], [161, 211, 192], [169, 196, 224], [198, 180, 220],
                        [224, 206, 166], [187, 210, 171], [176, 184, 218]], dtype=np.uint8)
    raster = np.empty((*occupancy.shape, 3), dtype=np.uint8)
    raster[:] = [220, 224, 229]
    raster[occupancy == 0] = [247, 248, 249]
    raster[occupancy > 0] = [55, 62, 69]
    assigned = labels > 0
    raster[assigned] = palette[(labels[assigned] - 1) % 6 + 1]
    x0, y0 = (width - draw_width) // 2, 68 + (height - 144 - draw_height) // 2
    canvas = Image.new("RGB", (width, height), "#ffffff")
    canvas.paste(Image.fromarray(raster[::-1]).resize((draw_width, draw_height), Image.Resampling.NEAREST), (x0, y0))
    draw = ImageDraw.Draw(canvas)
    font, chinese = _font(16)
    title_font, _ = _font(22)
    title = "空间语义地图" if chinese else "Spatial semantic map"
    draw.text((24, 16), title, fill="#172027", font=title_font)
    draw.text((24, 45), "Graph " + index.snapshot.graph_revision[:16] + "  |  " + snapshot.frame_id, fill="#56616b", font=font)

    def project(position):
        gx, gy = _local(position.x, position.y, geometry.origin, geometry.resolution)
        return x0 + (gx - left) * draw_width / (right - left), y0 + (top - gy) * draw_height / (top - bottom)

    for relation in index.snapshot.edges:
        if relation.kind != "access":
            continue
        a, b = index.nodes[relation.source], index.nodes[relation.target]
        if a.position and b.position:
            p, q = project(a.position), project(b.position)
            if all(x0 <= pnt[0] <= x0 + draw_width and y0 <= pnt[1] <= y0 + draw_height for pnt in (p, q)):
                # 点线只表示拓扑关系，图片下方明确不表示实际运动路径。
                distance = max(abs(q[0] - p[0]), abs(q[1] - p[1]))
                steps = max(1, round(distance / 5))
                for i in range(0, steps, 2):
                    u, v = i / steps, min(1, (i + 1) / steps)
                    draw.line((p[0] + (q[0] - p[0]) * u, p[1] + (q[1] - p[1]) * u,
                               p[0] + (q[0] - p[0]) * v, p[1] + (q[1] - p[1]) * v), fill="#56616b", width=1)
    legend, boxes = [], []
    ordered = sorted(index.nodes.values(), key=lambda n: (n.kind != "passage", n.node_id != node_id, n.kind != "region", n.node_id))
    for node in ordered:
        if node.position is None or node.kind == "observation":
            continue
        x, y = project(node.position)
        if not x0 <= x <= x0 + draw_width or not y0 <= y <= y0 + draw_height:
            continue
        code = node.properties.get("display_code") or "O%d" % (len(legend) + 1)
        legend.append({"code": code, "node_id": node.node_id, "name": node.name,
                       "kind": node.kind, "pixel": [round(x, 2), round(y, 2)]})
        if node.kind == "passage":
            draw.ellipse((x - 3, y - 3, x + 3, y + 3), fill="#d28b39", outline="#ffffff")
            continue
        text = code + (" " + node.name[:12] if chinese and (node.node_id == node_id or node.properties.get("has_label")) else "")
        bbox = draw.textbbox((0, 0), text, font=font)
        tw, th = bbox[2] - bbox[0] + 8, bbox[3] - bbox[1] + 8
        box = (max(x0, min(x - tw / 2, x0 + draw_width - tw)), max(y0, y - th / 2))
        rect = (box[0], box[1], box[0] + tw, box[1] + th)
        if node.node_id != node_id and any(rect[0] < b[2] + 3 and rect[2] > b[0] - 3 and rect[1] < b[3] + 3 and rect[3] > b[1] - 3 for b in boxes):
            continue
        boxes.append(rect)
        draw.rectangle(rect, fill="#ffffff", outline="#263743" if node.node_id == node_id else "#aeb8c0")
        draw.text((rect[0] + 4, rect[1] + 4 - bbox[1]), text, fill="#172027", font=font)
    note = "虚线表示区域关系；非运动轨迹。连接口尚未检查实际通行。" if chinese else "Dashed links: topology only. Robot passage not checked."
    draw.text((24, height - 52), note, fill="#56616b", font=font)
    scale_x = draw_width / (right - left)
    scale_y = draw_height / (top - bottom)
    draw.text((24, height - 28), "R: region   P: passage   O: object   |   %.3f m/cell" % grid.resolution_m, fill="#56616b", font=font)
    stream = io.BytesIO()
    canvas.save(stream, format="PNG")
    return {"png": stream.getvalue(), "width": width, "height": height, "legend": legend,
            "coordinate_mapping": {"frame_id": snapshot.frame_id, "grid_origin": geometry.origin,
                                   "resolution_m": geometry.resolution, "crop_cells": [left, bottom, right, top],
                                   "pixel_origin": [x0, y0], "pixels_per_cell": [scale_x, scale_y], "image_y_points_down": True},
            "map_snapshot_timestamp": snapshot.timestamp.isoformat(), "navigation_checked": False}
