# 官方Ubuntu20.04多架构镜像；固定索引摘要，ARM托管机执行真正aarch64构建。
FROM ubuntu:20.04@sha256:8feb4d8ca5354def3d8fce243717141ce31e2c428701f6682bd2fafe15388214
ARG NEH_PACKAGES
ARG NEH_UID=1001
ARG NEH_GID=1001
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
RUN apt-get update && apt-get install -y --no-install-recommends ${NEH_PACKAGES} && rm -rf /var/lib/apt/lists/*
# Ubuntu20.04随附的旧pip不识别现代manylinux标签；25.0.1仍支持Python3.8。
RUN /usr/bin/python3 -m pip install --disable-pip-version-check --no-cache-dir --index-url https://pypi.org/simple pip==25.0.1
RUN groupadd --gid ${NEH_GID} nehbuild && useradd --uid ${NEH_UID} --gid ${NEH_GID} --create-home --shell /bin/bash nehbuild
USER nehbuild
WORKDIR /home/nehbuild
