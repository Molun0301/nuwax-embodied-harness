# Go2使用Ubuntu20系统ABI，默认gcc9；只有C++20依赖显式使用gcc10。
ARG BASE_IMAGE=ubuntu:20.04
FROM ${BASE_IMAGE}
ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC LC_ALL=C.UTF-8
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl git python3 python3-dev python3-venv build-essential \
    gcc-10 g++-10 pkg-config cmake make patch patchelf zstd unzip binutils \
    liblcm-dev=1.3.1+repack1-2.3 liblcm-bin=1.3.1+repack1-2.3 \
    libglib2.0-dev libssl-dev libeigen3-dev=3.3.7-2 \
    libpcl-dev=1.10.0+dfsg-5ubuntu1 libgoogle-glog-dev libgflags-dev libboost1.71-dev=1.71.0-6ubuntu6 \
    libboost-serialization-dev libboost-filesystem-dev libboost-thread-dev \
    libboost-system-dev libboost-chrono-dev libboost-atomic-dev \
    && rm -rf /var/lib/apt/lists/*
# protoc3.6不能处理当前协议的optional字段，采用官方固定静态工具及历史已验摘要。
RUN curl -fsSL --retry 3 https://github.com/protocolbuffers/protobuf/releases/download/v25.3/protoc-25.3-linux-aarch_64.zip -o /tmp/protoc.zip \
    && echo '9eae1f20f70cccc912d1c318c3929b86aebf5afd4b0f32c196ef682c222ed5ae  /tmp/protoc.zip' | sha256sum -c - \
    && unzip -q /tmp/protoc.zip -d /opt/protoc && rm /tmp/protoc.zip \
    && /opt/protoc/bin/protoc --version | grep -Fx 'libprotoc 25.3'
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
    PATH=/opt/protoc/bin:/home/builder/.cargo/bin:$PATH \
    PROTOC=/opt/protoc/bin/protoc PROTOC_INCLUDE=/opt/protoc/include \
    NUWAX_CXX20=g++-10 CMAKE_GENERATOR="Unix Makefiles" CMAKE_EXPORT_COMPILE_COMMANDS=ON
RUN curl -fsSL --retry 3 https://sh.rustup.rs -o /tmp/rustup.sh \
    && sh /tmp/rustup.sh -y --profile minimal --default-toolchain 1.88.0 --no-modify-path \
    && rm /tmp/rustup.sh
WORKDIR /workspace
