#!/usr/bin/env bash
# 先清理本工程旧会话，再启动当前仓库的 install。
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
pid_file="$workspace_dir/.demo.pid"
instance_file="$workspace_dir/.demo.instance"
helper="$workspace_dir/scripts/demo_processes.py"

if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$workspace_dir/install/setup.bash" ]]; then
    echo '请安装 ROS2 Humble，并先在当前仓库运行 colcon build --symlink-install。' >&2
    exit 1
fi

# 仅锁启动和清理事务，运行期间释放；子节点与tee不能继承锁FD。
exec 9>"$workspace_dir/.demo.lock"
flock -x 9
python3 "$helper" stop --workspace "$workspace_dir"
rm -f -- "$pid_file" "$instance_file"

set +u
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
set -u
cd -- "$workspace_dir"
demo_instance="$(python3 -c 'import uuid; print(uuid.uuid4().hex)')"
mkdir -p -- "$workspace_dir/results"
terminal_log="$workspace_dir/results/launch_$(date +%Y%m%d_%H%M%S)_$$.txt"
echo '>> 启动 A2 演示（Ctrl+C 会清理本轮全部节点）...'
echo ">> 完整终端输出保存到：$terminal_log"
echo ">> ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}；重复服务也可能来自同域其他主机。"

cleanup() {
    # 再次Ctrl+C不能打断核验，退出清理只处理自己的实例。
    trap '' INT TERM HUP
    flock -x 9
    if python3 "$helper" stop --workspace "$workspace_dir" --instance "$demo_instance"; then
        if [[ -f "$instance_file" && "$(cat -- "$instance_file")" == "$demo_instance" ]]; then
            rm -f -- "$pid_file" "$instance_file"
        fi
    else
        echo '清理未完成，保留实例信息；请运行 bash stop_demo.sh 查看。' >&2
    fi
    flock -u 9
}

# launcher先退出时，仍凭继承的工程和实例标签查找存活子节点。
env A2_DEMO_WORKSPACE="$workspace_dir" A2_DEMO_INSTANCE="$demo_instance" \
    setsid ros2 launch --noninteractive machining_demo demo.launch.py "$@" \
    9>&- > >(tee "$terminal_log" 9>&-) 2>&1 &
demo_pid=$!
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
printf '%s\n' "$demo_pid" > "$pid_file"
printf '%s\n' "$demo_instance" > "$instance_file"
flock -u 9
wait "$demo_pid"
