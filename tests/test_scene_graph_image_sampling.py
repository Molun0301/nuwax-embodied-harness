"""Display sampling must preserve the original full-resolution RGB renderer exactly."""
from datetime import datetime, timezone
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image

from contracts.occupancy_cells import PackedOccupancyCells
from services.memory import scene_graph_image as renderer
from services.memory.room_segmentation import _world


def _legacy_image(occupancy, labels, size):
    palette = np.array([[236, 239, 242], [161, 211, 192], [169, 196, 224], [198, 180, 220],
                        [224, 206, 166], [187, 210, 171], [176, 184, 218]], dtype=np.uint8)
    raster = np.empty((*occupancy.shape, 3), dtype=np.uint8)
    raster[:] = [220, 224, 229]
    raster[occupancy == 0] = [247, 248, 249]
    raster[occupancy > 0] = [55, 62, 69]
    assigned = labels > 0
    raster[assigned] = palette[(labels[assigned] - 1) % 6 + 1]
    return Image.fromarray(raster[::-1]).resize(size, Image.Resampling.NEAREST)


def test_randomized_sampling_matches_original_pixels():
    rng = np.random.default_rng(56123)
    shapes = [(1, 2049), (2049, 1), (29, 47), (1001, 1999), (64, 64)]
    shapes += [tuple(rng.integers(1, 200, size=2)) for _ in range(100)]
    for height, width in shapes:
        # Non-contiguous crops, occupied/unknown cells, unlabeled cells and palette wraparound.
        occupancy = rng.choice(np.array([-1, 0, 1, 40, 100], dtype=np.int8), (height + 4, width + 4))[2:-2, 2:-2]
        labels = rng.integers(0, 20, size=(height + 4, width + 4), dtype=np.int32)[2:-2, 2:-2]
        for size in [(1, 1), (13, 7), (width, height), (width + 1, height + 1), (47, 29)]:
            sampled = renderer._display_cells(occupancy, labels, size)
            actual = _legacy_image(*sampled, size)
            expected = _legacy_image(occupancy, labels, size)
            assert actual.tobytes() == expected.tobytes(), (occupancy.shape, size)


def _scene(height, width, packed=True):
    rows, cols = np.indices((height, width), dtype=np.int32)
    occupancy = np.array([-1, 0, 1, 100], dtype=np.int8)[(rows * 3 + cols * 7) % 4]
    labels = ((rows // 19 + cols // 23) % 10).astype(np.int32)
    labels[(rows < height // 3) | (cols < width // 4)] = 0
    data = PackedOccupancyCells(occupancy.tobytes()) if packed else occupancy.ravel().tolist()
    grid = SimpleNamespace(height=height, width=width, resolution_m=0.2, data=data)
    origin = [2.0, -3.0, 0.0, 0.37]
    geometry = SimpleNamespace(labels=labels, region_ids=['region-%d' % i for i in range(1, 10)],
                               origin=origin, resolution=0.2)
    nodes = {}
    for node_id, kind, x, y, properties in [
        ('region-1', 'region', width * 0.5, height * 0.5, {'display_code': 'R1', 'has_label': True}),
        ('object-1', 'object', width * 0.48, height * 0.49, {'display_code': 'O1'}),
        ('object-2', 'object', width * 0.7, height * 0.6, {'region_id': 'region-1'}),
        ('passage-1', 'passage', width * 0.52, height * 0.53, {'display_code': 'P1'}),
        ('observation-1', 'observation', width * 0.5, height * 0.5, {}),
    ]:
        wx, wy = _world(x, y, origin, 0.2)
        nodes[node_id] = SimpleNamespace(node_id=node_id, kind=kind, name=node_id,
                                        position=SimpleNamespace(x=wx, y=wy), properties=properties)
    edges = [SimpleNamespace(kind='access', source='region-1', target='passage-1')]
    index = SimpleNamespace(geometry=geometry, nodes=nodes,
                            snapshot=SimpleNamespace(graph_revision='1234567890abcdef', edges=edges))
    snapshot = SimpleNamespace(occupancy_grid=grid, frame_id='map', timestamp=datetime(2026, 1, 2, tzinfo=timezone.utc))
    return index, snapshot


@pytest.mark.parametrize('shape', [(37, 61), (701, 1301), (1201, 301), (1, 2051), (2051, 1)])
@pytest.mark.parametrize('width', [512, 1280, 2048])
@pytest.mark.parametrize('focus', [None, 'region-1', 'object-1', 'object-2'])
def test_full_png_and_metadata_match_original(monkeypatch, shape, width, focus):
    index, snapshot = _scene(*shape, packed=width != 512)
    labels = index.geometry.labels.copy()
    actual = renderer.render_scene_graph_image(index, snapshot, node_id=focus, width=width)
    # Bypassing display sampling restores the old full-resolution colorize-then-resize path.
    monkeypatch.setattr(renderer, '_display_cells', lambda occupancy, labels, size: (occupancy, labels))
    expected = renderer.render_scene_graph_image(index, snapshot, node_id=focus, width=width)
    assert actual == expected
    np.testing.assert_array_equal(index.geometry.labels, labels)


def test_small_maps_keep_original_views():
    occupancy = np.zeros((37, 61), dtype=np.int8)
    labels = np.zeros_like(occupancy, dtype=np.int32)
    sampled_occupancy, sampled_labels = renderer._display_cells(occupancy, labels, (610, 370))
    assert sampled_occupancy is occupancy
    assert sampled_labels is labels
