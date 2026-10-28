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
RUN useradd --create-home --uid 1001 builder
USER builder
ENV RUSTUP_HOME=/home/builder/.rustup CARGO_HOME=/home/builder/.cargo \
    PATH=/home/builder/.cargo/bin:$PATH CMAKE_GENERATOR="Unix Makefiles" \
    CMAKE_EXPORT_COMPILE_COMMANDS=ON
RUN curl -fsSL --retry 3 https://sh.rustup.rs -o /tmp/rustup.sh \
    && sh /tmp/rustup.sh -y --profile minimal --default-toolchain 1.88.0 --no-modify-path \
    && rm /tmp/rustup.sh
WORKDIR /workspace
