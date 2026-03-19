#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SENTRY_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$SENTRY_ROOT/rm_vision_ws/install/setup.bash"
source "$SENTRY_ROOT/rm_decision_ws/install/setup.bash"

python3 "$SCRIPT_DIR/test_behavior_chain.py" "$@"
