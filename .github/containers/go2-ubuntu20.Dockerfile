# 官方Ubuntu20.04多架构镜像；固定索引摘要，ARM托管机执行真正aarch64构建。
FROM ubuntu:20.04@sha256:8feb4d8ca5354def3d8fce243717141ce31e2c428701f6682bd2fafe15388214
ARG NEH_PACKAGES
ARG NEH_UID=1001
ARG NEH_GID=1001
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
RUN apt-get update && apt-get install -y --no-install-recommends ${NEH_PACKAGES} && rm -rf /var/lib/apt/lists/*
# Ubuntu20.04的protoc3.6不支持现有协议的optional字段；使用官方固定ARM编译器。
RUN test "$(uname -m)" = aarch64 \
    && curl --fail --location --retry 3 https://github.com/protocolbuffers/protobuf/releases/download/v25.3/protoc-25.3-linux-aarch_64.zip -o /tmp/neh-protoc.zip \
    && echo '9eae1f20f70cccc912d1c318c3929b86aebf5afd4b0f32c196ef682c222ed5ae  /tmp/neh-protoc.zip' | sha256sum --check \
    && /usr/bin/python3 -m zipfile -e /tmp/neh-protoc.zip /opt/neh-protoc \
    && chmod 755 /opt/neh-protoc/bin/protoc \
    && test "$(/opt/neh-protoc/bin/protoc --version)" = 'libprotoc 25.3' \
    && printf 'syntax = "proto3"; message Probe { optional string value = 1; }\n' > /tmp/neh-protoc-check.proto \
    && /opt/neh-protoc/bin/protoc --experimental_allow_proto3_optional --proto_path=/tmp --descriptor_set_out=/tmp/neh-protoc-check.pb /tmp/neh-protoc-check.proto \
    && rm /tmp/neh-protoc.zip /tmp/neh-protoc-check.proto /tmp/neh-protoc-check.pb
ENV PROTOC=/opt/neh-protoc/bin/protoc
ENV PROTOC_INCLUDE=/opt/neh-protoc/include
# Ubuntu20.04随附的旧pip不识别现代manylinux标签；25.0.1仍支持Python3.8。
RUN /usr/bin/python3 -m pip install --disable-pip-version-check --no-cache-dir --index-url https://pypi.org/simple pip==25.0.1
RUN groupadd --gid ${NEH_GID} nehbuild && useradd --uid ${NEH_UID} --gid ${NEH_GID} --create-home --shell /bin/bash nehbuild
USER nehbuild
WORKDIR /home/nehbuild
