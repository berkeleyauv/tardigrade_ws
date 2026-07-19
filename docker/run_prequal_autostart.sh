#!/usr/bin/env bash
set -euo pipefail

env_file="${TARDIGRADE_ENV_FILE:-/etc/tardigrade/prequal.env}"
if [[ ! -r "$env_file" ]]; then
  echo "Missing configuration: $env_file" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$env_file"

PREQUAL_MODE="${PREQUAL_MODE:-tether}"
case "$PREQUAL_MODE" in
  auto)
    ;;
  tether)
    echo "PREQUAL_MODE=tether: automatic prequal launch is disabled."
    echo "Set PREQUAL_MODE=auto and start this service when the robot is ready."
    exit 0
    ;;
  *)
    echo "Invalid PREQUAL_MODE='$PREQUAL_MODE'; expected 'auto' or 'tether'." >&2
    exit 2
    ;;
esac

: "${WORKSPACE:?Set WORKSPACE in $env_file}"
: "${ESP_PORT:?Set ESP_PORT in $env_file}"
: "${VECTORNAV_PORT:?Set VECTORNAV_PORT in $env_file}"

IMAGE="${IMAGE:-tardigrade-foxy}"
NAME="${NAME:-tardigrade-prequal}"
DRY_RUN="${DRY_RUN:-true}"
STARTUP_DELAY_SEC="${STARTUP_DELAY_SEC:-60.0}"
FORWARD_COMMAND="${FORWARD_COMMAND:-0.20}"
DESCENT_COMMAND="${DESCENT_COMMAND:-0.20}"
DESCENT_DURATION_SEC="${DESCENT_DURATION_SEC:-8.0}"
OUTBOUND_DURATION_SEC="${OUTBOUND_DURATION_SEC:-40.0}"
RETURN_DURATION_SEC="${RETURN_DURATION_SEC:-40.0}"
YAW_KP="${YAW_KP:-0.4}"
YAW_KD="${YAW_KD:-0.1}"
MAX_YAW_COMMAND="${MAX_YAW_COMMAND:-0.20}"

docker rm -f "$NAME" >/dev/null 2>&1 || true

exec docker run --rm \
  --name "$NAME" \
  --network host \
  --ipc host \
  --privileged \
  -v "$WORKSPACE:/ws" \
  -v /dev:/dev \
  -v /dev/bus/usb:/dev/bus/usb \
  "$IMAGE" \
  ros2 launch tardigrade_bringup prequal_autonomy.launch.py \
    vectornav_port:="$VECTORNAV_PORT" \
    esp_port:="$ESP_PORT" \
    dry_run:="$DRY_RUN" \
    startup_delay_sec:="$STARTUP_DELAY_SEC" \
    forward_command:="$FORWARD_COMMAND" \
    descent_command:="$DESCENT_COMMAND" \
    descent_duration_sec:="$DESCENT_DURATION_SEC" \
    outbound_duration_sec:="$OUTBOUND_DURATION_SEC" \
    return_duration_sec:="$RETURN_DURATION_SEC" \
    yaw_kp:="$YAW_KP" \
    yaw_kd:="$YAW_KD" \
    max_yaw_command:="$MAX_YAW_COMMAND"
