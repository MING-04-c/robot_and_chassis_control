#!/usr/bin/env bash
# Launch one ROS 2 command and collect system/process resource data beside it.
# Usage:
#   ./profile_ros_command.sh /tmp/full_stack 120 -- ros2 launch ...
set -euo pipefail

if (( $# < 4 )) || [[ "$3" != "--" ]]; then
  echo "Usage: $0 PREFIX DURATION_SECONDS -- COMMAND [ARG ...]" >&2
  exit 2
fi

prefix=$1
duration=$2
shift 3
if [[ ! "$duration" =~ ^[0-9]+$ ]] || (( duration == 0 )); then
  echo "DURATION_SECONDS must be a positive integer" >&2
  exit 2
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p -- "$(dirname -- "$prefix")"

# ROS 2 otherwise tries to create logs under ~/.ros, which may be read-only on
# a service account or in a container.
ros_log_dir=${ROS_LOG_DIR:-"${prefix}.roslog"}
mkdir -p -- "$ros_log_dir"

"$script_dir/profile_ros_load.sh" "$prefix" "$duration" &
profiler_pid=$!

set +e
ROS_LOG_DIR="$ros_log_dir" setsid --wait -- "$@" >"${prefix}.launch.log" 2>&1 &
command_pid=$!
wait "$command_pid"
command_status=$?
set -e

# The sampler exits on its own after DURATION_SECONDS. If ROS exits early, stop
# it now so the result files are complete and the wrapper can return the launch
# status.
if kill -0 "$profiler_pid" 2>/dev/null; then
  kill "$profiler_pid" 2>/dev/null || true
  wait "$profiler_pid" 2>/dev/null || true
fi

if [[ -x "$script_dir/summarize_ros_profile.py" && -f "${prefix}.ps.txt" ]]; then
  "$script_dir/summarize_ros_profile.py" "$prefix" > "${prefix}.summary.csv" || true
fi

echo "ROS command exit status: $command_status"
echo "Results: ${prefix}.{meta.txt,pidstat.txt,mpstat.txt,vmstat.txt,ps.txt,summary.csv,thermal.csv,launch.log}"
exit "$command_status"
