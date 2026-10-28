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
