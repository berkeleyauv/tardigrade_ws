#!/usr/bin/env bash

export PYTHONWARNINGS="ignore:easy_install command is deprecated"
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=1

source /opt/ros/foxy/setup.bash

if [[ -f /ws/install/setup.bash ]]; then
  source /ws/install/setup.bash
fi

alias build-ws="/ws/build.sh"
alias build-hw="/ws/build.sh --hardware"
alias clean-build="/ws/build.sh --clean"
alias mock="ros2 launch tardigrade_bringup mock.launch.py"
alias status="ros2 topic echo /tardigrade/status"
alias fg="ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py"
