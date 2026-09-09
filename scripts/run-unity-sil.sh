#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
sim_player="${SIM_PLAYER:-$repo_root/../tardigrade_unity_world/Builds/Linux/TardigradeSim.x86_64}"

if [[ ! -x "$sim_player" ]]; then
  echo "Unity player not found or not executable: $sim_player"
  echo "Build it first or set SIM_PLAYER to the standalone executable."
  exit 1
fi

cd "$repo_root"
docker compose -f docker/compose.yaml up -d tardigrade
cleanup() {
  docker compose -f docker/compose.yaml exec -T tardigrade pkill -f unity_sil.launch.py >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM
docker compose -f docker/compose.yaml exec -d tardigrade bash -lc \
  'source /opt/ros/foxy/setup.bash && source /ws/install/setup.bash && ros2 launch tardigrade_bringup unity_sil.launch.py'
"$sim_player" "$@"
