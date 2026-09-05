#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${REPO_ROOT}"

bash -n \
  install.sh \
  scripts/install_motion_guard_deployment.sh \
  scripts/systemd/check_motion_guard_python.sh

python3 -m compileall -q -f install.py scripts/deploy_installer
ruff check scripts/deploy_installer install.py
python3 install.py --help >/dev/null
python3 install.py plan --profile go2-jetson --release-id ci-contract --go2-robot-ip 192.0.2.10 >/tmp/nuwax-install-plan.txt

grep -q 'rust_build' /tmp/nuwax-install-plan.txt
grep -q 'nuwax-person-follow' /tmp/nuwax-install-plan.txt
grep -q '/api/health/full' /tmp/nuwax-install-plan.txt

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
