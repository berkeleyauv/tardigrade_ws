#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
cd "$repo_root"

mode="dev"
build_image=false
rebuild_image=false
detached=false

usage() {
  cat <<'EOF'
Usage: ./docker-build.sh [options]

Options:
  --jetson       Start the Jetson hardware container.
  --build        Build the Docker image before starting the container.
  --rebuild      Build the Docker image with --no-cache before starting.
  --detached     Start the container in the background.
  -h, --help     Show this help.

Environment:
  WORKSPACE      Host workspace path to mount at /ws. Defaults to this repo.
  IMAGE          Docker image name. Defaults to tardigrade-foxy.
  NAME           Container name. Defaults to tardigrade-foxy.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --jetson)
      mode="jetson"
      ;;
    --build)
      build_image=true
      ;;
    --rebuild)
      build_image=true
      rebuild_image=true
      ;;
    --detached|-d)
      detached=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      usage
      exit 1
      ;;
  esac
  shift
done

export WORKSPACE="${WORKSPACE:-$repo_root}"
export IMAGE="${IMAGE:-tardigrade-foxy}"
export NAME="${NAME:-tardigrade-foxy}"

compose_files=(-f docker/compose.yaml)
if [[ "$mode" == "jetson" ]]; then
  compose_files+=(-f docker/compose.jetson.yaml)
fi

if [[ "$build_image" == true ]]; then
  build_args=("${compose_files[@]}" build)
  if [[ "$rebuild_image" == true ]]; then
    build_args+=(--no-cache)
  fi
  docker compose "${build_args[@]}"
fi

run_args=("${compose_files[@]}")
if [[ "$detached" == true ]]; then
  run_args+=(up -d)
else
  run_args+=(run --rm tardigrade)
fi

docker compose "${run_args[@]}"

