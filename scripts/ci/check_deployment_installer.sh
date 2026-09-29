#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${REPO_ROOT}"

bash -n \
  install.sh \
  scripts/deploy_installer/entrypoint.sh \
  scripts/install_motion_guard_deployment.sh \
  scripts/systemd/check_motion_guard_python.sh

python3 -m compileall -q -f install.py scripts/deploy_installer scripts/module_installer
ruff check scripts/deploy_installer install.py
# 保留与现有实机解释器兼容的 Optional 注解，不自动改为联合类型语法。
ruff check --ignore UP045 scripts/module_installer
python3 install.py --help >/dev/null
python3 install.py install --modules core --robot customrobot --json | python3 -c '
import json, sys
plan = json.load(sys.stdin)["plan"]
assert plan["modules"] == ["core"]
assert plan["models"] == [] and plan["builds"] == []
'
python3 install.py install --modules pointlio --robot customrobot --json | python3 -c '
import json, sys
plan = json.load(sys.stdin)["plan"]
assert plan["modules"] == ["core", "geometry", "pointlio"]
assert "libgoogle-glog-dev" in plan["system_packages"]
'
plan_file="$(mktemp -t neh-install-plan.XXXXXX)"
trap 'rm -f "${plan_file}"' EXIT
python3 install.py plan --profile go2-jetson --release-id ci-contract --go2-robot-ip 192.0.2.10 >"${plan_file}"

grep -q 'rust_build' "${plan_file}"
grep -q 'nuwax-person-follow' "${plan_file}"
grep -q '/api/health/full' "${plan_file}"

python3 install.py install --preset map2d --robot customrobot --json | python3 -c '
import json, sys
plan = json.load(sys.stdin)["plan"]
assert plan["modules"] == ["core", "geometry", "slam2d"]
assert plan["models"] == []
'

# 测试按项目约定仅存在于开发机；远端检查不依赖未入仓的文件。
if [[ "${1:-}" == "--with-tests" ]]; then
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
    tests/deployment \
    tests/unit/test_runtime_deployment_config_contract.py \
    tests/unit/test_motion_guard_deployment.py \
    tests/unit/test_systemd_runtime_readiness.py
elif [[ $# -gt 0 ]]; then
  echo "用法：$0 [--with-tests]" >&2
  exit 2
fi
bash scripts/install_motion_guard_deployment.sh check-sources

echo "deployment installer checks: PASS"
