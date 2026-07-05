#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${WORKSPACE:-}" ]]; then
  if [[ -n "${SUDO_USER:-}" && "$SUDO_USER" != "root" ]]; then
    WORKSPACE="$(getent passwd "$SUDO_USER" | cut -d: -f6)/Developer/tardigrade_ws"
  else
    WORKSPACE="$HOME/Developer/tardigrade_ws"
  fi
fi
IMAGE="${IMAGE:-tardigrade-foxy}"
NAME="${NAME:-tardigrade-foxy}"

docker stop "$NAME" >/dev/null 2>&1 || true
docker rm "$NAME" >/dev/null 2>&1 || true

docker run -it --rm \
  --name "$NAME" \
  --network host \
  --ipc host \
  --privileged \
  -v "$WORKSPACE:/ws" \
  -v /dev:/dev \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /usr/local/zed:/usr/local/zed \
  -v /usr/local/cuda:/usr/local/cuda \
  -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro \
  -e ZED_DIR=/usr/local/zed \
  -e CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda \
  -e CMAKE_PREFIX_PATH=/usr/local/zed \
  -e LD_LIBRARY_PATH=/usr/local/zed/lib:/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu \
  -e LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu \
  "$IMAGE"
