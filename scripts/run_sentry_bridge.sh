#!/usr/bin/env bash
set -euo pipefail

LOG_DIR="/home/nyu/sentry_planner/logs/autostart"
mkdir -p "$LOG_DIR"

export PATH="/home/nyu/.local/bin:/usr/local/bin:/usr/bin:/bin:$PATH"
unset BASH_ENV || true
unset ZDOTDIR || true
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

rm -f /tmp/nyush-rm-sentry-bridge-ttyACM*.lock >/dev/null 2>&1 || true
rm -f /tmp/nyush-rm-sentry-vision /tmp/nyush-rm-sentry-radar >/dev/null 2>&1 || true

set +u
. /opt/ros/humble/setup.bash
. /home/nyu/sentry_planner/install/setup.bash
set -u

cd /home/nyu/Codespace/nyush-rm-control
exec just sentry-bridge ${SENTRY_BRIDGE_ARGS:-}
