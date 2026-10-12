# Source this file after /opt/ros/<your_distro>/setup.bash.
robot_ws_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export LD_LIBRARY_PATH="$robot_ws_dir/local/elite_sdk/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
source "$robot_ws_dir/install/setup.bash"
unset robot_ws_dir
