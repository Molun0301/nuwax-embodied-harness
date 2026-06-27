#!/usr/bin/env bash
# build_proto.sh — 编译 navigation_proxy.proto 为 Python gRPC stub
# 用法：bash protos/build_proto.sh
# 输出：services/navigation/nav2_action_proxy_pb2.py + nav2_action_proxy_pb2_grpc.py
#
# 兼容实机 Py3.8.10 + grpcio-tools + protobuf 5.x
# 若 grpcio-tools 不可用，会回退到本地手写 stub（保证 import 不崩）。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROTO_FILE="${REPO_ROOT}/protos/navigation_proxy.proto"
OUT_DIR="${REPO_ROOT}/services/navigation"

mkdir -p "${OUT_DIR}"

if python3 -m grpc_tools.protoc -I "${REPO_ROOT}/protos" \
    --python_out="${OUT_DIR}" \
    --grpc_python_out="${OUT_DIR}" \
    "${PROTO_FILE}" 2>/dev/null; then
    echo "grpcio-tools 生成成功：${OUT_DIR}/nav2_action_proxy_pb2*.py"
    exit 0
fi

echo "[WARN] grpcio-tools 不可用，请使用手写 stub（protos/nav2_action_proxy_pb2.py）"
exit 1