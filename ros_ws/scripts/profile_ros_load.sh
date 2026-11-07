#!/usr/bin/env bash
# Read-only Raspberry Pi resource sampler for a separately launched ROS 2 run.
# Usage: ./profile_ros_load.sh [/tmp/robot_load] [duration_seconds]
# A duration of 0 (the default) samples until interrupted.
set -euo pipefail

prefix=${1:-/tmp/robot_load}
duration=${2:-0}
if [[ ! "$duration" =~ ^[0-9]+$ ]]; then
  echo "duration_seconds must be a nonnegative integer" >&2
  exit 2
fi
parent=$(dirname -- "$prefix")
mkdir -p -- "$parent"

for utility in pidstat mpstat vmstat; do
  if ! command -v "$utility" >/dev/null 2>&1; then
    echo "Missing '$utility'. Install sysstat (pidstat/mpstat) and procps (vmstat), then retry." >&2
    exit 1
  fi
done

{
  date --iso-8601=seconds
  uname -a
  printf '\nCPU count: '
  getconf _NPROCESSORS_ONLN
  printf '\nMemory and swap:\n'
  free -h
  printf '\n/tmp filesystem:\n'
  df -h "$parent"
  printf '\nInitial process snapshot (CPU/RSS/thread count; command line truncated to 240 chars):\n'
  ps -eo pid,ppid,nlwp,pcpu,pmem,rss,stat,comm,args --sort=-pcpu \
    | awk 'NR <= 31 { print substr($0, 1, 240) }'
  printf '\nInitial pressure stall information:\n'
  for resource in cpu memory io; do
    if [[ -r "/proc/pressure/$resource" ]]; then
      printf '[%s]\n' "$resource"
      cat "/proc/pressure/$resource"
    fi
  done
} > "${prefix}.meta.txt"

pidstat -h -u -r -w -d -p ALL 1 > "${prefix}.pidstat.txt" 2>&1 &
pidstat_pid=$!
mpstat -P ALL 1 > "${prefix}.mpstat.txt" 2>&1 &
mpstat_pid=$!
vmstat -w -t 1 > "${prefix}.vmstat.txt" 2>&1 &
vmstat_pid=$!
(
  while true; do
    printf '\n# %s\n' "$(date --iso-8601=seconds)"
    ps -eo pid,ppid,nlwp,pcpu,pmem,rss,stat,comm,args --sort=-pcpu \
      | awk 'NR <= 81 { print substr($0, 1, 320) }'
    sleep 1
  done
) > "${prefix}.ps.txt" 2>&1 &
ps_pid=$!

cleanup() {
  trap - EXIT INT TERM
  kill "$pidstat_pid" "$mpstat_pid" "$vmstat_pid" "$ps_pid" 2>/dev/null || true
  wait "$pidstat_pid" "$mpstat_pid" "$vmstat_pid" "$ps_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM

printf 'timestamp,temp_c,arm_clock_hz,throttled\n' > "${prefix}.thermal.csv"
if (( duration > 0 )); then
  echo "Sampling system resources for ${duration}s to ${prefix}.{meta.txt,pidstat.txt,mpstat.txt,vmstat.txt,ps.txt,thermal.csv}."
else
  echo "Sampling system resources to ${prefix}.{meta.txt,pidstat.txt,mpstat.txt,vmstat.txt,ps.txt,thermal.csv}; press Ctrl+C to stop."
fi

elapsed=0
while true; do
  timestamp=$(date --iso-8601=seconds)
  temp_c=NA
  if [[ -r /sys/class/thermal/thermal_zone0/temp ]]; then
    temp_c=$(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp)
  fi
  arm_clock_hz=NA
  throttled=NA
  if command -v vcgencmd >/dev/null 2>&1; then
    if clock_output=$(vcgencmd measure_clock arm 2>/dev/null); then
      arm_clock_hz=$(sed -nE 's/^frequency\(48\)=([0-9]+)$/\1/p' <<< "$clock_output")
    fi
    if throttle_output=$(vcgencmd get_throttled 2>/dev/null); then
      throttled=$(sed -nE 's/^throttled=(0x[[:xdigit:]]+)$/\1/p' <<< "$throttle_output")
    fi
    arm_clock_hz=${arm_clock_hz:-NA}
    throttled=${throttled:-NA}
  fi
  printf '%s,%s,%s,%s\n' "$timestamp" "$temp_c" "$arm_clock_hz" "$throttled" \
    >> "${prefix}.thermal.csv"
  ((elapsed += 1))
  if (( duration > 0 && elapsed >= duration )); then
    break
  fi
  sleep 1
done

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ -x "$script_dir/summarize_ros_profile.py" && -f "${prefix}.ps.txt" ]]; then
  "$script_dir/summarize_ros_profile.py" "$prefix" > "${prefix}.summary.csv" || true
fi
