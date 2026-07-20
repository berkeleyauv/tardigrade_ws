#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
cd "$repo_root"

clean=false
hardware=false
debug=false
package=""

usage() {
  cat <<'EOF'
Usage: ./build.sh [options]

Options:
  --clean          Remove build/install/log before building.
  --hardware       Build hardware packages, including ZED SDK packages.
  --pkg PACKAGE    Build only one package.
  --debug          Build with RelWithDebInfo.
  -h, --help       Show this help.

Default:
  Builds the local development workspace and skips ZED SDK packages that only
  build on the Jetson or a machine with the Stereolabs SDK installed.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean)
      clean=true
      ;;
    --hardware)
      hardware=true
      ;;
    --pkg)
      if [[ -z "${2:-}" ]]; then
        echo "Missing package name after --pkg"
        exit 1
      fi
      package="$2"
      shift
      ;;
    --debug)
      debug=true
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

if [[ "$clean" == true ]]; then
  rm -rf build install log
fi

build_cmd=(colcon build --symlink-install)

if [[ -n "$package" ]]; then
  build_cmd+=(--packages-select "$package")
else
  packages_to_skip=()
  if [[ "$hardware" == false ]]; then
    packages_to_skip+=(zed_components zed_wrapper zed_ros2)
  fi
  if [[ "${#packages_to_skip[@]}" -gt 0 ]]; then
    build_cmd+=(--packages-skip "${packages_to_skip[@]}")
  fi
fi

if [[ "$debug" == true ]]; then
  build_cmd+=(--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo)
fi

"${build_cmd[@]}"
