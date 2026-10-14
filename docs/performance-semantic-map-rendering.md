# Semantic-map presentation sampling

Measured 2026-10-13 in the existing cloud checkout with Python 3.8.10,
NumPy 1.24.4 and Pillow 10.4.0. This change affects image presentation only;
physical occupancy, region geometry, topology, coordinate mapping and navigation
checks are unchanged.

## Change

Previously the renderer colorized every cell in the selected crop into RGB,
flipped it, then resized it with Pillow NEAREST. Large crops allocated and
colorized millions of pixels that were never displayed.

The renderer now computes the same display dimensions first. For downscaled
crops, it resizes one-dimensional integer index images with Pillow NEAREST,
selects those exact source cells, and then colorizes the selected cells.
Sampling accounts for the renderer's vertical flip. Using Pillow itself avoids
introducing a hand-written nearest-neighbor rounding convention. Small maps
that only need upscaling retain their original array views and colorization path.

## Measurements

Each number below is the median of **three fresh processes**, not five. Each
process constructed the same deterministic packed occupancy map and int32
region labels, then rendered once at width 1280. PNG encoding was timed inside
`Image.Image.save`; non-PNG time is total rendering minus that interval. Peak
RSS is the whole process high-water mark, including imports and input data,
not an isolated allocation measurement. Timed runs were not concurrent with
another worker's benchmarks.

| Map | Total before → after | Non-PNG before → after | PNG before → after | Peak RSS before → after |
| --- | --- | --- | --- | --- |
| 128 × 128 | 48.1 → 49.5 ms | 15.9 → 16.4 ms | 32.2 → 33.2 ms | 138.4 → 137.6 MiB |
| 2048 × 2048 | 317.4 → 152.0 ms | 278.6 → 114.3 ms | 38.8 → 37.7 ms | 233.2 → 185.2 MiB |
| 4096 × 4096 | 1105.0 → 156.6 ms | 1061.9 → 116.1 ms | 43.2 → 40.4 ms | 503.5 → 246.2 MiB |

The large-map case was 7.1× faster overall and used 51.1% less peak RSS.
The small-map control was **3.1% slower** in this three-process sample; it does
not establish a small-map improvement or a statistically significant regression.
Its new work is one helper call and a dimension check, without sampling copies.
The patterned fixtures make these reproducible renderer microbenchmarks, not
claims about every real map, machine or full endpoint latency. No focus-crop
region scan or PNG codec optimization is claimed.

## Equivalence verification

`tests/test_scene_graph_image_sampling.py` contains:

- 525 deterministic raster comparisons covering random and non-contiguous crops,
  unknown/free/occupied cells, region palette cycling, one-cell axes, downscaling,
  upscaling and unchanged dimensions
- 60 complete render comparisons checking PNG bytes and every returned metadata
  field against the original colorize-before-resize path, restored by bypassing
  the sampling helper
- One small-map identity check ensuring original array views are retained

Full-render cases cover widths 512/1280/2048, packed and list occupancy,
rotated map origins, whole-map/region/object focus, region-associated objects,
passage markers, access lines, observations and overlapping labels. The complete
PNG comparisons include whatever fonts are available in the test environment;
they do not separately certify every platform/font/Pillow version. The full
renderer comparison shares unchanged overlay code, so an additional manual
check compared 36 renders against the separately saved original source; all
PNG bytes and metadata matched. This is tested equivalence, not an exhaustive
proof over all possible inputs.

Validation: 139 tests passed across the new test module and
`tests/unit/test_scene_mapping_service.py`; `compileall` and `git diff --check`
passed. Ruff could not run because the workspace's Python environment has no
ruff module or executable. No dependencies were installed for this change.

## Evidence

The cloud workspace's sibling directory
`neh-evidence/semantic-render-20261013/` retains the original renderer,
benchmark script, all 18 raw process measurements and validation log. Those
files are outside the repository and are not required at runtime. The benchmark
script accepts `before|after` and a square-map dimension; it loads the
retained baseline beside the script. Run it with the repository on `PYTHONPATH`
using the stated toolchain.
