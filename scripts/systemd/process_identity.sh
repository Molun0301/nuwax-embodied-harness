#!/usr/bin/env bash

# Shared process identity readers for ready-file publishers and consumers.
# Callers own error reporting; every failure is represented by a non-zero exit.

nuwax_read_boot_id() {
  local boot_id=""
  IFS= read -r boot_id < /proc/sys/kernel/random/boot_id || return 1
  [[ "${boot_id}" =~ ^[0-9A-Fa-f]{8}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{12}$ ]] \
    || return 1
  printf '%s\n' "${boot_id,,}"
}

nuwax_read_proc_starttime() {
  local pid="${1:-}"
  local stat_line=""
  local fields_after_comm=""
  local -a fields=()
  local starttime=""

  [[ "${pid}" =~ ^[1-9][0-9]*$ ]] || return 1
  IFS= read -r stat_line < "/proc/${pid}/stat" || return 1
  [[ "${stat_line}" == *") "* ]] || return 1

  # Field 2 (`comm`) is parenthesized and may contain spaces or `)`. Strip
  # through the final `) `; field 3 is then array index 0, so field 22 is 19.
  fields_after_comm="${stat_line##*) }"
  read -r -a fields <<< "${fields_after_comm}" || return 1
  [[ "${#fields[@]}" -gt 19 ]] || return 1
  starttime="${fields[19]}"
  [[ "${starttime}" =~ ^[1-9][0-9]*$ ]] || return 1
  printf '%s\n' "${starttime}"
}
