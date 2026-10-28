# Rebuilding a small native validation environment

This is the reproducible x86_64 Linux setup verified on 2026-10-05 with GCC 14.2,
Rust 1.88.0 and Python 3.12. Keep dependencies outside the checkout, restore them
in stages, and retain their build directories. Do not run an all-module installer
or rebuild the SDK for each test. These commands do not change project sources,
lockfiles, shell profiles or system packages. The validated Geometry lane is the
named durable V3 snapshot below; do not assume every primary/WIP dependency graph
is identical.

## Pinned inputs

Use official releases/registries. Verify published checksums where available;
record exact commit/tag, archive size and local digest otherwise. These checks
verify installer inputs; they are **not application hashing or object identity**.

| Input | Version/source |
| --- | --- |
| Rust compiler, Cargo, host stdlib | `1.88.0`, `x86_64-unknown-linux-gnu`; `https://static.rust-lang.org/dist/2025-06-26/{rustc,cargo,rust-std}-1.88.0-x86_64-unknown-linux-gnu.tar.xz` (three URLs; each has `.sha256`) |
| CMake | [3.31.6 Linux x86_64 release](https://github.com/Kitware/CMake/releases/tag/v3.31.6), archive `cmake-3.31.6-linux-x86_64.tar.gz`; verify its entry in `cmake-3.31.6-SHA-256.txt` |
| protoc | [29.3](https://github.com/protocolbuffers/protobuf/releases/tag/v29.3), `protoc-29.3-linux-x86_64.zip`, including `include/` |
| LCM | [1.5.2](https://github.com/lcm-proj/lcm/releases/tag/v1.5.2); the verified [PyPI wheel](https://pypi.org/project/lcm/1.5.2/) supplies headers, libraries and `bin/lcm-gen` under its installation prefix |
| Eigen | [3.4.0 headers](https://gitlab.com/libeigen/eigen/-/releases/3.4.0); verified existing headers may be reused |
| Boost | [1.83.0 archive](https://archives.boost.io/release/1.83.0/source/boost_1_83_0.tar.bz2), **exact version required** |
| oneTBB | [commit 8b829acc65569019edb896c5150d427f288e8aba](https://codeload.github.com/oneapi-src/oneTBB/tar.gz/8b829acc65569019edb896c5150d427f288e8aba), version 2021.11.0 |
| glog | [0.6.0](https://codeload.github.com/google/glog/tar.gz/refs/tags/v0.6.0) |
| Ceres | [commit 85331393dc0dff09f6fb9903ab0c4bfa3e134b01](https://codeload.github.com/ceres-solver/ceres-solver/tar.gz/85331393dc0dff09f6fb9903ab0c4bfa3e134b01), version 2.2.0 |
| LZ4 | [1.10.0](https://codeload.github.com/lz4/lz4/tar.gz/refs/tags/v1.10.0) |
| FLANN | [1.9.2](https://codeload.github.com/flann-lib/flann/tar.gz/refs/tags/1.9.2) |
| PCL | [1.14.1](https://codeload.github.com/PointCloudLibrary/pcl/tar.gz/refs/tags/pcl-1.14.1) |
| GTSAM | [commit 1a9792a7ede244850a413739557635b606f295c0](https://codeload.github.com/borglab/gtsam/tar.gz/1a9792a7ede244850a413739557635b606f295c0), **4.3a1**, not the 4.2 release |
| libjpeg-turbo | [official 2.1.5.1 amd64 package](https://github.com/libjpeg-turbo/libjpeg-turbo/releases/download/2.1.5.1/libjpeg-turbo-official_2.1.5.1_amd64.deb) |

The repository-pinned archive SHA-256 values are:

- Boost: `6478edfe2f3305127cffe8caf73ea0176c53769f4bf1585be237eb30798c3b8e`
- GTSAM: `62f9624662e54b6f5aff03784504f790389bed85d2dee1648ba839bc78df6407`

protoc/libjpeg-turbo release metadata did not supply independent digests in this
restoration. Their official asset identity, size, archive integrity and reported
version/linkage were checked; a locally calculated digest is not a vendor checksum.

## Prefixes and small prerequisites

Host prerequisites: GCC/G++ (verified 14.2), GNU Make, pkg-config, Git, Bash,
Python 3.12 for the wheel route, curl and archive tools (tar, xz/bzip2, unzip,
dpkg-deb). Keep working glibc/libstdc++/OpenMP and GLib runtime libraries, plus
zlib development headers/library. Check actual discovery/linkage before installing
anything missing; do not assume runtime-only libraries provide development files.

Use Bash. Set absolute paths for `REPO` (a clean, verified source snapshot), `BASE`
(a writable dependency/cache root), `LCM_PREFIX` (contains `include/lcm`, `lib`,
`bin/lcm-gen`) and `EIGEN_INCLUDE_DIR` (contains `Eigen/Core`). Inspect actual
versions before reuse; do not point metadata at missing or differently versioned
libraries. Existing runtime-only Boost/TBB packages are insufficient.

```bash
set -euo pipefail
SDK="$BASE/sdk"; SRC="$BASE/src"; BUILD="$BASE/build"; META="$BASE/metadata"
TOOLS="$BASE/tools"; RUST="$TOOLS/rust-1.88.0"
mkdir -p "$SDK" "$SRC" "$BUILD" "$META/lib/pkgconfig" "$META/share/eigen3/cmake"
# Extract verified archives into SRC/TOOLS, checking archive paths first.
# Install each Rust component with its install.sh --prefix="$RUST" --disable-ldconfig.
export PATH="$RUST/bin:$TOOLS/cmake-3.31.6-linux-x86_64/bin:$TOOLS/protoc-29.3/bin:$LCM_PREFIX/bin:$PATH"
export PROTOC="$TOOLS/protoc-29.3/bin/protoc"
export PROTOC_INCLUDE="$TOOLS/protoc-29.3/include"
export LCM_GEN_EXECUTABLE="$LCM_PREFIX/bin/lcm-gen"
rustc --version; cargo --version; cmake --version; "$PROTOC" --version
"$LCM_GEN_EXECUTABLE" --version # must be exactly: lcm-gen 1.5.2
```

When a reused LCM wheel lacks `lcm.pc`, supply truthful local discovery metadata:

```bash
cat > "$META/lib/pkgconfig/lcm.pc" <<EOF_PC
prefix=$LCM_PREFIX
libdir=\${prefix}/lib
includedir=\${prefix}/include
Name: lcm
Description: Verified LCM headers and shared library
Version: 1.5.2
Libs: -L\${libdir} -llcm
Cflags: -I\${includedir}
EOF_PC
```

An ordinary Eigen installation can supply its own CMake/pkg-config files. For
verified header-only reuse, the local `Eigen3Config.cmake` must expose
`Eigen3_VERSION`, `EIGEN3_VERSION`, `EIGEN3_VERSION_STRING` as `3.4.0`,
`EIGEN3_INCLUDE_DIR`/`EIGEN3_INCLUDE_DIRS` as the actual header directory, and an
imported interface target `Eigen3::Eigen` with that include directory. Supply a
matching `Eigen3ConfigVersion.cmake` (CMake's `write_basic_package_version_file`,
version 3.4.0, `AnyNewerVersion`, `ARCH_INDEPENDENT`) and `eigen3.pc` version 3.4.0
with `Cflags: -I<actual-header-directory>`. Validate header macros and a small
compile/link check; metadata alone is not evidence of a usable SDK.

Save a prefix-local environment file with canonical values; source it once in a
fresh shell. Do not append prefixes repeatedly or change flags between warm runs.

```bash
export CMAKE_PREFIX_PATH="$SDK:$META"
export PKG_CONFIG_PATH="$SDK/lib/pkgconfig:$META/lib/pkgconfig:$LCM_PREFIX/lib/pkgconfig"
export LIBRARY_PATH="$SDK/lib:$SDK/libjpeg-package/opt/libjpeg-turbo/lib64"
export LD_LIBRARY_PATH="$SDK/lib:$SDK/libjpeg-package/opt/libjpeg-turbo/lib64:$LCM_PREFIX/lib"
export CARGO_HOME="$BASE/cargo-home" CARGO_TARGET_DIR="$BASE/cargo-target"
export CARGO_BUILD_JOBS=2 CMAKE_BUILD_PARALLEL_LEVEL=2
export CARGO_PROFILE_DEV_DEBUG=0 CARGO_PROFILE_TEST_DEBUG=0
pkg-config --modversion lcm eigen3
```

## Build only the necessary SDK

Build sequentially in a fresh shell with no inherited `CFLAGS`, `CXXFLAGS`,
`CPPFLAGS` or `LDFLAGS`. AVX/SSE switches do not remove inherited compiler flags;
inspect the generated flags before building. The helper retains source/build
directories; reconfigure only when needed. Names are extracted archive directories.

```bash
unset CFLAGS CXXFLAGS CPPFLAGS LDFLAGS
cmake_install() {
  local source=$1 name=$2; shift 2
  cmake -S "$source" -B "$BUILD/$name" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$SDK" -DCMAKE_INSTALL_LIBDIR=lib \
    -DCMAKE_PREFIX_PATH="$SDK;$META" -DBUILD_SHARED_LIBS=ON "$@"
  cmake --build "$BUILD/$name" --parallel 2
  cmake --install "$BUILD/$name"
}
(cd "$SRC/boost_1_83_0" && ./bootstrap.sh --with-toolset=gcc --prefix="$SDK" &&
 ./b2 --build-dir="$BUILD/boost" --prefix="$SDK" -j2 toolset=gcc variant=release \
   link=shared runtime-link=shared threading=multi cxxstd=17 --layout=system \
   --with-serialization --with-filesystem --with-thread --with-system \
   --with-chrono --with-atomic --with-date_time --with-graph \
   --with-program_options --with-random --with-timer --with-iostreams install)
cmake_install "$SRC/oneTBB-8b829acc65569019edb896c5150d427f288e8aba" tbb \
  -DTBB_TEST=OFF -DTBB_EXAMPLES=OFF -DTBB_STRICT=OFF -DTBB_DISABLE_HWLOC_AUTOMATIC_SEARCH=ON
cmake_install "$SRC/glog-0.6.0" glog \
  -DBUILD_TESTING=OFF -DWITH_GFLAGS=OFF -DWITH_GTEST=OFF -DWITH_UNWIND=OFF
cmake_install "$SRC/ceres-solver-85331393dc0dff09f6fb9903ab0c4bfa3e134b01" ceres \
  -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF \
  -DSUITESPARSE=OFF -DCXSPARSE=OFF -DEIGENSPARSE=ON -DUSE_CUDA=OFF \
  -DGFLAGS=OFF -DLAPACK=OFF -DMINIGLOG=OFF -DEXPORT_BUILD_DIR=OFF \
  -DCMAKE_INSTALL_RPATH="$SDK/lib"
cmake_install "$SRC/lz4-1.10.0/build/cmake" lz4 -DBUILD_STATIC_LIBS=OFF -DLZ4_BUILD_CLI=OFF
cmake_install "$SRC/flann-1.9.2" flann -DCMAKE_INSTALL_RPATH="$SDK/lib" \
  -DBUILD_C_BINDINGS=ON -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_MATLAB_BINDINGS=OFF \
  -DBUILD_CUDA_LIB=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_DOC=OFF \
  -DUSE_MPI=OFF -DCMAKE_DISABLE_FIND_PACKAGE_HDF5=ON
```

Boost iostreams is required by PCL's global configuration even with PCL IO off.
FLANN's real C wrapper is required because its supplied `flann.pc` links both
`flann` and `flann_cpp`; do not fake the library or remove requirements to hide a
link failure. Boost iostreams reused available zlib/zstd support; do not add
optional compression packages silently. No new gflags/SuiteSparse/HDF5/CUDA
installation is needed here.

For PCL, enable only `common kdtree octree search sample_consensus filters 2d
features registration`; `2d` is a real dependency of `features`.

```bash
pcl_flags=()
for c in common kdtree octree search sample_consensus filters 2d features registration; do
  pcl_flags+=("-DBUILD_$c=ON")
done
for c in geometry io ml segmentation visualization surface keypoints tracking recognition \
         stereo apps benchmarks outofcore examples people simulation global_tests tools; do
  pcl_flags+=("-DBUILD_$c=OFF")
done
cmake_install "$SRC/pcl-pcl-1.14.1" pcl "${pcl_flags[@]}" \
  -DCMAKE_CXX_STANDARD=17 -DPCL_SHARED_LIBS=ON -DPCL_ONLY_CORE_POINT_TYPES=ON \
  -DPCL_ENABLE_AVX=OFF -DPCL_ENABLE_SSE=OFF -DPCL_ENABLE_MARCHNATIVE=OFF \
  -DWITH_LIBUSB=OFF -DWITH_PNG=OFF -DWITH_QHULL=OFF -DWITH_CUDA=OFF \
  -DWITH_VTK=OFF -DWITH_QT=NO -DWITH_PCAP=OFF -DWITH_OPENGL=OFF \
  -DWITH_GLEW=OFF -DWITH_OPENNI=OFF -DWITH_OPENNI2=OFF -DWITH_DOCS=OFF \
  -DCMAKE_INSTALL_RPATH="$SDK/lib"
# Extract the official package locally; do not install it globally/run package scripts.
dpkg-deb -x "$BASE/downloads/libjpeg-turbo-official_2.1.5.1_amd64.deb" "$SDK/libjpeg-package"
GTSAM_SRC="$SRC/gtsam-1a9792a7ede244850a413739557635b606f295c0"
cmake -DNUWAX_GTSAM_SOURCE_DIR="$GTSAM_SRC" \
  -P "$REPO/services/nuwax-workspace/crates/nuwax-gtsam/native/eigen_compat.cmake"
cmake_install "$GTSAM_SRC" gtsam -DBUILD_SHARED_LIBS=OFF \
  -DCMAKE_CXX_STANDARD=17 -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TIMING_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_UNSTABLE_BUILD_PYTHON=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_WITH_TBB=OFF -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_USE_SYSTEM_METIS=OFF \
  -DGTSAM_BUILD_METIS_EXECUTABLES=OFF -DCMAKE_CXX_FLAGS=-Wno-error=array-bounds
```

**ABI requirement:** PCL's defaults produced C++14 + AVX2 libraries while native
consumers use C++17 without AVX. The mismatch caused an ICP crash/double free.
Rebuilding PCL as C++17 with explicit AVX/SSE extensions off fixed the unchanged
consumer; baseline x86_64 SSE2 and O3 remain. Do not add ad hoc consumer AVX flags.
Core-only precompilation was checked against actual `PointXYZ`/`PointNormal` APIs;
this is not a full general-purpose PCL SDK.

The locked `turbojpeg-sys 0.1.0` uses pregenerated bindings and simply links
`-lturbojpeg` by default. It does not need a bundled rebuild, NASM or libclang on
this official-binary route; newer crate environment variables do not apply.

## Preflight, then bounded validation

Before compiling Geometry, configure its actual native GTSAM, PGO and SLAM source
directories into separate `$BUILD/preflight-*` outputs. Pass the verified prefix,
`LCM_GEN_EXECUTABLE` and `FETCHCONTENT_FULLY_DISCONNECTED=ON`. Require both GTSAM
consumers' `gtsam-library.txt` to resolve to the **same installed** `libgtsam.a`,
`libmetis-gtsam.a`, `libcephes-gtsam.a`; there must be no `_deps` fallback. Check
`find_package(GTSAM 4.2 REQUIRED)` succeeds with the pinned 4.3a1 exports. The two
small project-owned wrappers may compile separately; the upstream SDK must not.

```bash
cd "$REPO"
# Select a clean verified V3 source snapshot containing the intended tests.
cargo test --locked --manifest-path services/nuwax-workspace/Cargo.toml \
  -p nuwax-map-chunks --lib
# Keep this value stable: two Cargo build scripts must not spawn four C++ compilers.
export CMAKE_BUILD_PARALLEL_LEVEL=1
unset RUSTFLAGS CARGO_ENCODED_RUSTFLAGS RUSTC_WRAPPER RUSTC_WORKSPACE_WRAPPER
cargo test --locked --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml \
  --bin nuwax-geometry-runtime --no-run -vv
# Repeat the identical no-run command to check warm reuse; never cargo clean.
```

Record source identity, actual flags, emitted binary, unchanged source/lockfile,
SDK paths and `ldd` results. Stop on a new dependency, SDK fallback/rebuild, ABI
failure or resource overrun; do not patch project code to make setup appear green.
Audit/select tests separately before running Geometry: **no-run success is not
behavioral, transport, robot or performance acceptance**.

## Measured evidence and limits

- Primary [260b460d](https://github.com/Molun0301/nuwax-embodied-harness/commit/260b460d9d92952c6b94c9e1396c66f016b64f40)
  compiled map-chunks with **0 library tests**. V3 checkpoint projection
  [947d9cb7](https://github.com/Molun0301/nuwax-embodied-harness/commit/947d9cb73894d92320bd2f7c175a3fb3ed0c71e9)
  ran **40 passed, 0 failed, 0 ignored**. These are distinct source snapshots,
  not a promise that every branch has 40 tests.
- Clean durable V3 [e69f9af4](https://github.com/Molun0301/nuwax-embodied-harness/commit/e69f9af4a3b8b67ad3c5ea3e3247ebbc73948738),
  verified tree `fa4c24311a5d87bf09cd103ff66d3f4f2edf8269`, was the Geometry
  no-run baseline: **3m53s** initially, **0.29s** warm;
  zero warm Rust/C++ compilation or C++ relinking, unchanged sources/lockfiles.
  No Geometry tests were executed in that validation.
- SDK smoke checks passed Boost/TBB/Ceres optimization, voxel/kd-tree/normals/ICP
  and a GTSAM pose graph (error 58.75 to approximately 8.9e-28).
- SDK source archives: **263.5 MiB**; one failed FLANN transfer was unmeasured
  (reserving its entire 33 MiB archive still stays under the **450 MiB** envelope).
  Installed SDK approximately **284 MiB**, retained source/build/archive cache
  **1.9 GiB**; SDK work stayed under a **12 GiB** peak allowance.
- First Geometry compile added **19.7 MiB** crate archives and **2.48 GiB**
  Cargo cache/target. Enforce separate **150 MiB / 4 GiB** growth limits and inspect
  logs during initial builds. SDK builds used jobs=2; Geometry used Cargo2/CMake1.

Timings are observations on one host, not guarantees. Keep downloaded provenance,
configuration/build logs and reusable caches outside Git; commit no binaries,
archives, vendor trees, credentials or machine-specific environment paths.
