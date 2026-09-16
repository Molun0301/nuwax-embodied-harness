#!/usr/bin/env bash
set -euo pipefail

mode=check
case "${1:-}" in
  --prepare) mode=prepare; shift ;;
  --validate-config) mode=validate; shift ;;
esac
(( $# <= 1 )) || { echo 'LCM preflight expects one URL' >&2; exit 1; }
LCM_URL="${1:-${NUWAX_LCM_URL:-udpm://239.255.76.67:7667?ttl=0}}"
# 此标记只用于当前组的本项目路由；准备时使用add，不能覆盖已有用户路由。
route_protocol=242

report_error() {
  echo "LCM multicast preflight failed: $*" >&2
  return 1
}
fail() { report_error "$@" || true; exit 1; }

[[ "${LCM_URL}" == udpm://* && "${LCM_URL}" != *[[:space:]]* ]] || fail "NUWAX_LCM_URL must use udpm:// without whitespace"
provider="${LCM_URL#udpm://}"
authority="${provider%%[?]*}"
if [[ "${authority}" == *:* ]]; then
  group="${authority%%:*}"
  port="${authority#*:}"
else
  group="${authority}"
  port=7667
fi
IFS=. read -r -a octets <<<"${group}"
[[ ${#octets[@]} -eq 4 ]] || fail "invalid multicast IPv4 address: ${group}"
for octet in "${octets[@]}"; do
  [[ "${octet}" =~ ^(0|[1-9][0-9]{0,2})$ ]] || fail "invalid multicast IPv4 address: ${group}"
  (( octet <= 255 )) || fail "invalid multicast IPv4 address: ${group}"
done
(( octets[0] >= 224 && octets[0] <= 239 )) || fail "LCM destination is not an IPv4 multicast group: ${group}"
[[ "${port}" =~ ^[1-9][0-9]{0,4}$ ]] && (( port <= 65535 )) || fail "invalid LCM port: ${port}"
ttl=0
ttl_seen=false
if [[ "${provider}" == *'?'* ]]; then
  query="${provider#*[?]}"
  IFS='&' read -r -a arguments <<<"${query}"
  for argument in "${arguments[@]}"; do
    [[ "${argument}" == ttl=* ]] || continue
    [[ "${ttl_seen}" == false ]] || fail 'duplicate ttl option'
    value="${argument#ttl=}"
    [[ "${value}" =~ ^[0-9]{1,3}$ ]] || fail "invalid LCM ttl: ${value}"
    ttl=$((10#${value}))
    (( ttl <= 255 )) || fail "invalid LCM ttl: ${value}"
    ttl_seen=true
  done
fi
if [[ "${mode}" == validate ]]; then
  echo "LCM multicast configuration ready: group=${group} ttl=${ttl}"
  exit 0
fi
command -v ip >/dev/null 2>&1 || fail 'iproute2 is required for route validation'

route_ready() {
  local route iface link source_address loopback_addresses exact_routes explicit_source=false flags route_line selected_owned
  route="$(ip -4 route get "${group}" 2>&1)" || { report_error "no route to multicast group ${group}: ${route}"; return 1; }
  iface="$(awk '{for (i=1; i<=NF; i++) if ($i == "dev" && i < NF) {print $(i+1); exit}}' <<<"${route}")"
  [[ -n "${iface}" ]] || { report_error "route does not name an output interface: ${route}"; return 1; }
  if (( ttl == 0 )) && [[ "${iface}" != lo ]]; then
    report_error "本机LCM路由必须指向lo；请先以管理员身份运行本脚本 --prepare"
    return 1
  fi
  if (( ttl == 0 )); then
    source_address="$(awk '{for (i=1; i<=NF; i++) if ($i == "src" && i < NF) {print $(i+1); exit}}' <<<"${route}")"
    loopback_addresses="$(ip -o -4 address show dev lo)" || return 1
    exact_routes="$(ip -4 route show exact "${group}/32")" || return 1
    while IFS= read -r route_line; do
      if [[ " ${route_line} " == *' dev lo '* && " ${route_line} " == *" src ${source_address} "* ]]; then explicit_source=true; fi
    done <<<"${exact_routes}"
    if [[ "${explicit_source}" != true ]]; then
      report_error '本机LCM需要具有明确lo源地址的同组/32路由，避免物理地址变化改变报文来源'
      return 1
    fi
    if ! awk -v expected="${source_address}" '{for (i=1;i<NF;i++) if ($i=="inet") {split($(i+1),parts,"/"); if(parts[1]==expected) found=1}} END {exit !found}' <<<"${loopback_addresses}"; then
      report_error "本机LCM源地址必须属于lo，不能借用物理网卡地址: ${source_address}"
      return 1
    fi
  fi
  if (( ttl > 0 )) && [[ "${iface}" == lo ]]; then
    selected_owned="$(ip -4 route show exact "${group}/32" proto "${route_protocol}")" || return 1
    if [[ -n "${selected_owned}" ]]; then
      report_error '外部LCM仍指向项目本机路由；请沿完整停止和网络准备流程切换'
      return 1
    fi
  fi
  link="$(ip link show dev "${iface}" 2>&1)" || { report_error "cannot inspect multicast interface ${iface}: ${link}"; return 1; }
  flags="${link#*<}"; flags="${flags%%>*}"
  [[ ",${flags}," == *,MULTICAST,* ]] || { report_error "route interface ${iface} lacks MULTICAST capability"; return 1; }
  [[ ",${flags}," == *,UP,* ]] || { report_error "route interface ${iface} is not UP"; return 1; }
  echo "LCM multicast route ready: group=${group} interface=${iface} url=${LCM_URL}"
}

if [[ "${mode}" == prepare ]]; then
  exact="$(ip -4 route show exact "${group}/32")"
  owned="$(ip -4 route show exact "${group}/32" proto "${route_protocol}")"
  if (( ttl == 0 )); then
    if [[ -n "${exact}" && "${exact}" != "${owned}" ]]; then
      # 已有用户回环路由可直接复用，但不能取得其所有权或日后将它删除。
      while IFS= read -r route_line; do
        [[ " ${route_line} " == *' dev lo '* ]] || fail "已有同组精确路由不属于本项目，拒绝覆盖: ${route_line}"
      done <<<"${exact}"
    fi
    [[ "${EUID}" -eq 0 ]] || fail 'local multicast preparation requires root'
    ip link set dev lo up multicast on
    if [[ -z "${exact}" ]]; then
      ip -4 route add "${group}/32" dev lo src 127.0.0.1 proto "${route_protocol}"
      if ! route_ready; then
        # 只回收本次新增的路由；已有配置和其它组完全不参与回滚。
        ip -4 route del "${group}/32" dev lo proto "${route_protocol}" || true
        exit 1
      fi
      exit 0
    fi
  elif [[ -n "${owned}" ]]; then
    # 显式非零TTL恢复宿主机原有选路，只移除当前组由本项目创建的回环项。
    [[ "${EUID}" -eq 0 ]] || fail 'managed multicast route removal requires root'
    while IFS= read -r route_line; do
      [[ " ${route_line} " == *' dev lo '* ]] || fail "项目路由标记被其它接口占用: ${route_line}"
    done <<<"${owned}"
    ip -4 route del "${group}/32" dev lo proto "${route_protocol}"
    if ! route_ready; then
      ip -4 route add "${group}/32" dev lo src 127.0.0.1 proto "${route_protocol}" || true
      exit 1
    fi
    exit 0
  fi
fi
route_ready
