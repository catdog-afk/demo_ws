#!/usr/bin/env bash
# 无需编译或加载 ROS 环境，只清理能确认属于本工作区的仿真节点。
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec 9>"$workspace_dir/.demo.lock"
flock -x 9
python3 "$workspace_dir/scripts/demo_processes.py" stop --workspace "$workspace_dir"
rm -f -- "$workspace_dir/.demo.pid" "$workspace_dir/.demo.instance"
