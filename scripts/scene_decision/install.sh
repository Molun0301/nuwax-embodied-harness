#!/usr/bin/env bash
set -euo pipefail

# 模型依赖仅安装进独立环境，平台与机器人环境不参与。
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
scene_env="${NUWAX_SCENE_DECISION_VENV:-${HOME}/.local/share/nuwax/scene-decision/venv}"
python3 -m venv "$scene_env"
"$scene_env/bin/python" -m pip install --timeout 180 --retries 2 torch==2.8.0 torchvision==0.23.0 --index-url https://download.pytorch.org/whl/cu128
"$scene_env/bin/python" -m pip install -r "$root_dir/requirements/scene-decision.in"
"$scene_env/bin/python" "$root_dir/scripts/scene_decision/download_model.py"
echo "独立模型环境已安装：$scene_env"
