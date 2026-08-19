set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

default:
  @just --list

make_cmd := if os_family() == "windows" { "mingw32-make" } else { "make" }
default_robot := "infantry"
default_jobs := "12"
default_pyocd_target := "STM32H723VG"
default_pyocd_connect := "under-reset"
local_robot_file := ".robot_type"
local_vscode_robot_header := ".vscode/robot_type.local.h"

_requirements := "scripts/requirement.txt"

[private]
_ensure-py:
  @root="{{justfile_directory()}}"; \
  venv_dir="${PY_VENV_DIR:-${root}/.venv}"; \
  if [[ -n "${VIRTUAL_ENV:-}" && -x "${VIRTUAL_ENV}/bin/python" ]]; then py="${VIRTUAL_ENV}/bin/python"; \
  elif [[ -x "${venv_dir}/bin/python" ]]; then py="${venv_dir}/bin/python"; \
  else \
    if ! command -v python3 >/dev/null 2>&1; then \
      echo "[py-env] python3 not found. Please install Python 3 first." >&2; exit 2; \
    fi; \
    echo "[py-env] creating virtual environment: ${venv_dir}" >&2; \
    python3 -m venv "${venv_dir}"; \
    py="${venv_dir}/bin/python"; \
  fi; \
  if ! "${py}" -c 'import serial, websockets, pyocd' >/dev/null 2>&1; then \
    echo "[py-env] installing/updating requirements from {{_requirements}}" >&2; \
    "${py}" -m ensurepip --upgrade >/dev/null 2>&1 || true; \
    "${py}" -m pip install --upgrade pip >&2; \
    "${py}" -m pip install -r "${root}/{{_requirements}}" >&2; \
  fi; \
  echo "${py}"

[private]
_py script *args:
  @py="$(just _ensure-py)"; \
  root="{{justfile_directory()}}"; \
  exec "${py}" "${root}/{{script}}" {{args}}

# Show all available recipes.
help:
  @just --list

[private]
_resolve-robot requested="":
  @requested="{{requested}}"; \
  robot_file="{{justfile_directory()}}/{{local_robot_file}}"; \
  configured=""; \
  if [[ -f "${robot_file}" ]]; then \
    configured="$(tr -d '\r' < "${robot_file}" | sed -n 's/^[[:space:]]*//; s/[[:space:]]*$//; /^[[:space:]]*$/d; 1p')"; \
  fi; \
  if [[ -n "${configured}" ]]; then \
    case "${configured}" in \
      infantry|damiao|hero|sentry|sentry_swerve) ;; \
      *) echo "Invalid robot type '${configured}' in {{local_robot_file}}. Use: infantry|damiao|hero|sentry|sentry_swerve." >&2; exit 2 ;; \
    esac; \
  fi; \
  if [[ -z "${requested}" ]]; then \
    if [[ -n "${configured}" ]]; then \
      echo "${configured}"; \
    else \
      echo "{{default_robot}}"; \
    fi; \
    exit 0; \
  fi; \
  case "${requested}" in \
    infantry|damiao|hero|sentry|sentry_swerve) ;; \
    *) echo "Unsupported robot type '${requested}'. Use: infantry|damiao|hero|sentry|sentry_swerve." >&2; exit 2 ;; \
  esac; \
  if [[ -n "${configured}" && "${requested}" != "${configured}" ]]; then \
    echo "Robot type '${requested}' conflicts with local robot '${configured}' from {{local_robot_file}}." >&2; \
    exit 2; \
  fi; \
  echo "${requested}"

[private]
_sync-vscode-robot robot="":
  @resolved_robot="$(just _resolve-robot "{{robot}}")"; \
  header="{{justfile_directory()}}/{{local_vscode_robot_header}}"; \
  tmp_header="${header}.tmp"; \
  mkdir -p "$(dirname "${header}")"; \
  printf '#pragma once\n#define ROBOT_TYPE_%s\n' "${resolved_robot}" > "${tmp_header}"; \
  if [[ -f "${header}" ]] && cmp -s "${tmp_header}" "${header}"; then \
    rm -f "${tmp_header}"; \
  else \
    mv "${tmp_header}" "${header}"; \
  fi

[private]
_print-robot robot="":
  @resolved_robot="$(just _resolve-robot "{{robot}}")"; \
  echo "[robot] using ${resolved_robot}"

# Sync VSCode IntelliSense robot define from `.robot_type`.
sync-vscode-robot robot="":
  @just _sync-vscode-robot "{{robot}}"

# Build firmware with robot type and parallel jobs.
build robot="" jobs=default_jobs:
  @resolved_robot="$(just _resolve-robot "{{robot}}")"; \
  just _print-robot "${resolved_robot}"; \
  just _sync-vscode-robot "${resolved_robot}"; \
  {{make_cmd}} -j{{jobs}} ROBOT_TYPE="${resolved_robot}"

# Clean build artifacts.
clean:
  {{make_cmd}} clean

# Clean then rebuild firmware.
rebuild robot="" jobs=default_jobs:
  @resolved_robot="$(just _resolve-robot "{{robot}}")"; \
  just _sync-vscode-robot "${resolved_robot}"; \
  {{make_cmd}} clean && {{make_cmd}} -j{{jobs}} ROBOT_TYPE="${resolved_robot}"

# Run upstream Makefile help.
make-help:
  {{make_cmd}} help

# Flash with DFU-util.
flash-dfu robot="":
  @resolved_robot="$(just _resolve-robot "{{robot}}")"; \
  just _print-robot "${resolved_robot}"; \
  just _sync-vscode-robot "${resolved_robot}"; \
  {{make_cmd}} ROBOT_TYPE="${resolved_robot}" flash_dfu

# Flash with pyOCD using any supported probe.
flash-pyocd robot="" target=default_pyocd_target serial="" freq="" connect=default_pyocd_connect:
  @resolved_robot="$(just _resolve-robot "{{robot}}")"; \
  just _print-robot "${resolved_robot}"; \
  just _sync-vscode-robot "${resolved_robot}"; \
  {{make_cmd}} ROBOT_TYPE="${resolved_robot}" PYOCD_TARGET={{target}} DAP_SERIAL={{serial}} PYOCD_FREQUENCY={{freq}} PYOCD_CONNECT={{connect}} flash_pyocd

# Backward-compatible aliases.
flash-dap robot="" target=default_pyocd_target serial="" freq="" connect=default_pyocd_connect:
  just flash-pyocd "{{robot}}" "{{target}}" "{{serial}}" "{{freq}}" "{{connect}}"

flash-stlink robot="" target=default_pyocd_target serial="" freq="" connect=default_pyocd_connect:
  just flash-pyocd "{{robot}}" "{{target}}" "{{serial}}" "{{freq}}" "{{connect}}"

# Flash with JLinkExe.
flash-jlink robot="" serial="" speed="4000":
  @resolved_robot="$(just _resolve-robot "{{robot}}")"; \
  just _print-robot "${resolved_robot}"; \
  just _sync-vscode-robot "${resolved_robot}"; \
  {{make_cmd}} ROBOT_TYPE="${resolved_robot}" JLINK_SERIAL="{{serial}}" JLINK_SPEED="{{speed}}" flash_jlink

# Unified flash entrypoint.
# Usage:
#   just flash
#   just flash pyocd
#   just flash --method pyocd --robot hero --serial 69655005
#   just flash dap --freq 100k --connect pre-reset
flash *args:
  @method="auto"; robot=""; target="{{default_pyocd_target}}"; serial=""; freq=""; connect="{{default_pyocd_connect}}"; \
  set -- {{args}}; \
  while [[ "$#" -gt 0 ]]; do \
    case "$1" in \
      --method) [[ "$#" -ge 2 ]] || { echo "Missing value for --method" >&2; exit 2; }; method="$2"; shift 2 ;; \
      --robot) [[ "$#" -ge 2 ]] || { echo "Missing value for --robot" >&2; exit 2; }; robot="$2"; shift 2 ;; \
      --target) [[ "$#" -ge 2 ]] || { echo "Missing value for --target" >&2; exit 2; }; target="$2"; shift 2 ;; \
      --serial) [[ "$#" -ge 2 ]] || { echo "Missing value for --serial" >&2; exit 2; }; serial="$2"; shift 2 ;; \
      --freq|--frequency) [[ "$#" -ge 2 ]] || { echo "Missing value for --freq" >&2; exit 2; }; freq="$2"; shift 2 ;; \
      --connect) [[ "$#" -ge 2 ]] || { echo "Missing value for --connect" >&2; exit 2; }; connect="$2"; shift 2 ;; \
      auto|dfu|pyocd|dap|stlink|dap-openocd|jlink) method="$1"; shift ;; \
      halt|pre-reset|under-reset|attach) connect="$1"; shift ;; \
      infantry|damiao|hero|sentry|sentry_swerve) robot="$1"; shift ;; \
      *) echo "Unsupported flash arg: $1" >&2; exit 2 ;; \
    esac; \
  done; \
  if [[ "${method}" == "dap-openocd" ]]; then \
    method="pyocd"; \
  fi; \
  if [[ "$method" == "auto" ]]; then \
    probe_lines=""; \
    if command -v pyocd >/dev/null 2>&1; then \
      probe_lines="$(pyocd list 2>/dev/null | awk '/^[[:space:]]*[0-9]+[[:space:]]+/ {print}')"; \
      if [[ -n "$probe_lines" ]]; then \
        non_jlink_lines="$(printf '%s\n' "$probe_lines" | grep -Evi 'j-?link' || true)"; \
        if [[ -n "$non_jlink_lines" ]]; then \
          method="pyocd"; \
        fi; \
      fi; \
    fi; \
    if [[ "$method" == "auto" ]] && command -v JLinkExe >/dev/null 2>&1; then \
      jlink_output="$(printf 'ShowEmuList USB\nexit\n' | JLinkExe -NoGui 1 2>&1 || true)"; \
      if printf '%s\n' "$jlink_output" | grep -Eq 'J-Link\\[[0-9]+\\]:'; then \
        method="jlink"; \
      fi; \
    fi; \
    if [[ "$method" == "auto" ]] && command -v dfu-util >/dev/null 2>&1 && dfu-util -l 2>/dev/null | grep -q 'Found DFU:'; then \
      method="dfu"; \
    fi; \
    if [[ "$method" == "auto" ]]; then \
      echo "No supported flash transport detected. Checked: pyOCD probes, J-Link, DFU." >&2; \
      exit 2; \
    fi; \
  fi; \
  if [[ "${method}" == "pyocd" || "${method}" == "dap" || "${method}" == "stlink" ]]; then \
    busy_logger_pids="$(pgrep -f 'scripts/dashboard/rtt_(dashboard|ws_bridge)\.py' || true)"; \
    safe_logger_pids=""; \
    for pid in ${busy_logger_pids}; do \
      if [[ "${pid}" != "$$" && "${pid}" != "${PPID:-0}" ]]; then \
        safe_logger_pids+=" ${pid}"; \
      fi; \
    done; \
    safe_logger_pids="${safe_logger_pids# }"; \
    if [[ -n "${safe_logger_pids}" ]]; then \
      echo "[flash] stopping RTT logger process(es) to free CMSIS-DAP probe: ${safe_logger_pids}"; \
      kill -TERM ${safe_logger_pids} || true; \
      sleep 1; \
    fi; \
  fi; \
  resolved_robot="$(just _resolve-robot "${robot}")"; \
  just _print-robot "${resolved_robot}"; \
  just _sync-vscode-robot "${resolved_robot}"; \
  case "${method}" in \
    dfu) {{make_cmd}} ROBOT_TYPE="${resolved_robot}" flash_dfu ;; \
    pyocd|dap|stlink) {{make_cmd}} ROBOT_TYPE="${resolved_robot}" PYOCD_TARGET="${target}" DAP_SERIAL="${serial}" PYOCD_FREQUENCY="${freq}" PYOCD_CONNECT="${connect}" flash_pyocd ;; \
    jlink) {{make_cmd}} ROBOT_TYPE="${resolved_robot}" JLINK_SERIAL="${serial}" JLINK_SPEED="${freq:-4000}" flash_jlink ;; \
    *) echo "Unsupported method='${method}'. Use: auto|dfu|pyocd|jlink (aliases: dap, stlink, dap-openocd -> pyocd)" >&2; exit 2 ;; \
  esac

# Ensure python env/deps, then run RTT logger.
# Ensure python env/deps, then run RTT dashboard CLI.
# Extra args pass-through: just logger-cli --mode split --serial 69655005
logger-cli *args:
  just _py scripts/dashboard/rtt_dashboard.py --mode dashboard --channel 1 --log-channel 0 --connect normal --poll-ms 20 --refresh-ms 100 {{args}}

# Ensure python env/deps, then run RTT websocket logger bridge.
# Extra args pass-through: just logger --ws-port 9000 --serial 69655005
logger *args:
  just _py scripts/dashboard/rtt_ws_bridge.py --dashboard-channel 1 --log-channel 0 --ws-host 127.0.0.1 --ws-port 8765 --http-host 127.0.0.1 --http-port 8080 {{args}}

# Ensure python env/deps, then run interactive vision serial tool.
vision *args:
  just _py scripts/vision_tool.py {{args}}

# Short aliases for daily use.
b *args:
  just build {{args}}

f *args:
  just flash {{args}}

lc *args:
  just logger-cli {{args}}

lg *args:
  just logger {{args}}

# Install/repair python virtualenv and requirements.
py-bootstrap:
  @just _ensure-py >/dev/null
  @echo "[py-env] ready"

# Print python interpreter selected by env checker.
py-which:
  @just _ensure-py
