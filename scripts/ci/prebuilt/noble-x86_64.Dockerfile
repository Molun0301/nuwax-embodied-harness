# x86预编译使用与目标设备相同的Ubuntu24系统库。
ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE}
ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC LC_ALL=C.UTF-8
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl git python3 python3-dev python3-venv build-essential \
    pkg-config cmake make patch patchelf zstd unzip binutils protobuf-compiler \
    liblcm-dev liblcm-bin libglib2.0-dev libssl-dev libeigen3-dev libpcl-dev \
    libgoogle-glog-dev libgflags-dev libboost-dev libboost-serialization-dev \
    libboost-filesystem-dev libboost-thread-dev libboost-system-dev \
    libboost-chrono-dev libboost-atomic-dev \
    && rm -rf /var/lib/apt/lists/*
# 固定PGO wire V3宿主生成器；只构建此工具，不安装或替换目标LCM运行库。
RUN curl -fsSL --retry 3 https://codeload.github.com/lcm-proj/lcm/tar.gz/bac6b256144b25e8e41a172d1a45f4581e49717d -o /tmp/lcm-1.5.2.tar.gz \
    && echo '3c6634d089d69f6be5a7dfd80c2379beb65c6bbf3a2ec852d8ba37f644ddeeb4  /tmp/lcm-1.5.2.tar.gz' | sha256sum -c - \
    && mkdir /tmp/lcm-1.5.2 && tar -xzf /tmp/lcm-1.5.2.tar.gz --strip-components=1 -C /tmp/lcm-1.5.2 \
    && cmake -S /tmp/lcm-1.5.2 -B /tmp/lcm-1.5.2-build -DCMAKE_BUILD_TYPE=Release \
       -DLCM_ENABLE_PYTHON=OFF -DLCM_ENABLE_JAVA=OFF -DLCM_ENABLE_LUA=OFF \
       -DLCM_ENABLE_GO=OFF -DLCM_ENABLE_TESTS=OFF -DLCM_ENABLE_EXAMPLES=OFF \
    && cmake --build /tmp/lcm-1.5.2-build --target lcm-gen --parallel 2 \
    && mkdir -p /opt/lcm-gen-1.5.2/bin /opt/lcm-gen-1.5.2/share \
    && cp /tmp/lcm-1.5.2-build/lcmgen/lcm-gen /opt/lcm-gen-1.5.2/bin/lcm-gen \
    && cp /tmp/lcm-1.5.2/COPYING /opt/lcm-gen-1.5.2/share/COPYING \
    && /opt/lcm-gen-1.5.2/bin/lcm-gen --version | grep -Fx 'lcm-gen 1.5.2' \
    && rm -rf /tmp/lcm-1.5.2 /tmp/lcm-1.5.2-build /tmp/lcm-1.5.2.tar.gz
ENV LCM_GEN_EXECUTABLE=/opt/lcm-gen-1.5.2/bin/lcm-gen
RUN useradd --create-home --uid 1001 builder
USER builder
ENV RUSTUP_HOME=/home/builder/.rustup CARGO_HOME=/home/builder/.cargo \
    PATH=/home/builder/.cargo/bin:$PATH CMAKE_GENERATOR="Unix Makefiles" \
    CMAKE_EXPORT_COMPILE_COMMANDS=ON
RUN curl -fsSL --retry 3 https://sh.rustup.rs -o /tmp/rustup.sh \
    && sh /tmp/rustup.sh -y --profile minimal --default-toolchain 1.88.0 --no-modify-path \
    && rm /tmp/rustup.sh
WORKDIR /workspace
