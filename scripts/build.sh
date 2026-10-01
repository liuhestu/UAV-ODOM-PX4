#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ "${ROS_VERSION:-1}" != 1 || "${CMAKE_PREFIX_PATH:-}" == *'/opt/ros/humble'* ]]; then
  echo '请使用未加载 ROS2 Humble 的终端构建 ROS1 工作区。' >&2
  exit 1
fi
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo '需要本机已有的 /opt/ros/noetic/setup.bash。' >&2
  exit 1
fi
set +u
source /opt/ros/noetic/setup.bash
set -u
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
export PATH="${HOME}/.local/bin:${PATH}"
command -v catkin >/dev/null || { echo '请先安装 catkin_tools。' >&2; exit 1; }
cd "$workspace_dir"
catkin config --workspace "$workspace_dir" --init --extend /opt/ros/noetic \
  --source-space src --build-space build --devel-space devel --log-space logs \
  --link-devel --no-install -j2 -p1
catkin build --workspace "$workspace_dir" -j2 -p1 "$@"
printf '\n构建完成，当前终端加载：\nsource %q\n' "$workspace_dir/devel/setup.bash"
