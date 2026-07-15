#!/usr/bin/env bash
set -euo pipefail

LCM_URL="${1:-${NUWAX_LCM_URL:-udpm://239.255.76.67:7667?ttl=1}}"

fail() {
  echo "LCM multicast preflight failed: $*" >&2
  exit 1
}

[[ "${LCM_URL}" == udpm://* ]] \
  || fail "NUWAX_LCM_URL must use udpm://, got ${LCM_URL}"
[[ "${LCM_URL}" =~ [\?\&]ttl=1($|\&) ]] \
  || fail "production NUWAX_LCM_URL must declare ttl=1, got ${LCM_URL}"

provider="${LCM_URL#udpm://}"
group="${provider%%:*}"
[[ "${group}" =~ ^(22[4-9]|23[0-9])\. ]] \
  || fail "LCM destination is not an IPv4 multicast group: ${group}"

command -v ip >/dev/null 2>&1 || fail "iproute2 is required for route validation"
route="$(ip route get "${group}" 2>&1)" \
  || fail "no route to multicast group ${group}: ${route:-unknown error}"
iface="$(awk '{for (i=1; i<=NF; i++) if ($i == "dev" && i < NF) {print $(i+1); exit}}' <<<"${route}")"
[[ -n "${iface}" ]] || fail "route does not name an output interface: ${route}"

link="$(ip link show dev "${iface}" 2>&1)" \
  || fail "cannot inspect multicast interface ${iface}: ${link:-unknown error}"
if [[ "${link}" != *MULTICAST* ]]; then
  if [[ "${iface}" == "lo" ]]; then
    fail "loopback route selected but lo lacks MULTICAST; enable it with 'ip link set lo multicast on'"
  fi
  fail "route interface ${iface} lacks MULTICAST capability: ${link}"
fi

echo "LCM multicast route ready: group=${group} interface=${iface} url=${LCM_URL}"
