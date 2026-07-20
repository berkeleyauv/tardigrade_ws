#!/usr/bin/env bash
set -euo pipefail

env_file="${TARDIGRADE_ENV_FILE:-/etc/tardigrade/qual.env}"
if [[ ! -r "$env_file" ]]; then
  echo "Missing configuration: $env_file" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$env_file"

: "${WORKSPACE:?Set WORKSPACE in $env_file}"
: "${ESP_PORT:?Set ESP_PORT in $env_file}"
: "${VECTORNAV_PORT:?Set VECTORNAV_PORT in $env_file}"

IMAGE="${IMAGE:-tardigrade-foxy}"
NAME="${NAME:-tardigrade-qual}"
DRY_RUN="${DRY_RUN:-true}"
STARTUP_DELAY_SEC="${STARTUP_DELAY_SEC:-15.0}"
TARGET_DEPTH_M="${TARGET_DEPTH_M:-1.5}"
FORWARD_DISTANCE_M="${FORWARD_DISTANCE_M:-20.0}"
FORWARD_COMMAND="${FORWARD_COMMAND:-0.20}"
YAW_KP="${YAW_KP:-0.4}"
YAW_KD="${YAW_KD:-0.1}"
MAX_YAW_COMMAND="${MAX_YAW_COMMAND:-0.20}"

runtime_args=()
if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; then
  runtime_args+=(--runtime nvidia)
fi

docker rm -f "$NAME" >/dev/null 2>&1 || true

exec docker run --rm \
  --name "$NAME" \
  --network host \
  --ipc host \
  --privileged \
  "${runtime_args[@]}" \
  -v "$WORKSPACE:/ws" \
  -v /dev:/dev \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /tmp/argus_socket:/tmp/argus_socket \
  -v /usr/local/zed:/usr/local/zed \
  -v /usr/local/cuda:/usr/local/cuda \
  -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ZED_DIR=/usr/local/zed \
  -e CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda \
  -e CMAKE_PREFIX_PATH=/usr/local/zed \
  -e LD_LIBRARY_PATH=/usr/local/zed/lib:/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu \
  "$IMAGE" \
  ros2 launch tardigrade_bringup qual_autonomy.launch.py \
    vectornav_port:="$VECTORNAV_PORT" \
    esp_port:="$ESP_PORT" \
    dry_run:="$DRY_RUN" \
    startup_delay_sec:="$STARTUP_DELAY_SEC" \
    target_depth_m:="$TARGET_DEPTH_M" \
    forward_distance_m:="$FORWARD_DISTANCE_M" \
    forward_command:="$FORWARD_COMMAND" \
    yaw_kp:="$YAW_KP" \
    yaw_kd:="$YAW_KD" \
    max_yaw_command:="$MAX_YAW_COMMAND"
