#!/usr/bin/env bash
set -eo pipefail

folder=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$folder/../.." && pwd)
cd "$root"

load_ros_environment() {
    source /opt/ros/melodic/setup.bash
    source devel/setup.bash
}
load_ros_environment

export CONTROL_RESEARCH_DEVEL="$root/devel"
exec python3 "$folder/trial.py" "$@"
