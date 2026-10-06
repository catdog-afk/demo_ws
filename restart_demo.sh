#!/usr/bin/env bash
# 重启由本脚本管理的 A2 演示；从任意目录调用，使用当前仓库的 install。
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
pid_file="$workspace_dir/.demo.pid"

if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$workspace_dir/install/setup.bash" ]]; then
    echo '请安装 ROS2 Humble，并先在当前仓库运行 colcon build --symlink-install。' >&2
    exit 1
fi

if [[ -f "$pid_file" ]]; then
    old_pid="$(cat -- "$pid_file")"
    if [[ "$old_pid" =~ ^[0-9]+$ ]] && kill -0 "$old_pid" 2>/dev/null; then
        old_command="$(ps -p "$old_pid" -o args=)"
        old_group="$(ps -p "$old_pid" -o pgid= | tr -d ' ')"
        if [[ "$old_command" != *'ros2 launch machining_demo demo.launch.py'* || "$old_group" != "$old_pid" ]]; then
            echo '保存的进程标识与 A2 演示不匹配，未终止任何进程。' >&2
            exit 1
        fi
        echo '>> 结束此前由本脚本启动的 A2 演示...'
        # SIGINT 触发 ROS launch 对所有子节点的完整退出流程。
        kill -INT "$old_pid" 2>/dev/null || true
        for attempt in {1..100}; do
            kill -0 "$old_pid" 2>/dev/null || break
            sleep 0.2
        done
        if kill -0 "$old_pid" 2>/dev/null; then
            echo '旧演示仍未退出，保留进程标识并停止启动新实例。' >&2
            exit 1
        fi
    fi
    rm -f -- "$pid_file"
fi

set +u
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
set -u
cd -- "$workspace_dir"
echo '>> 启动 A2 演示（Ctrl+C 可退出）...'
setsid ros2 launch machining_demo demo.launch.py "$@" &
demo_pid=$!
printf '%s\n' "$demo_pid" > "$pid_file"

cleanup() {
    kill -INT "$demo_pid" 2>/dev/null || true
    for attempt in {1..100}; do
        kill -0 "$demo_pid" 2>/dev/null || break
        sleep 0.2
    done
    if kill -0 "$demo_pid" 2>/dev/null; then
        echo '演示仍在退出，保留进程标识；等待退出日志结束后再启动。' >&2
        return
    fi
    if [[ -f "$pid_file" && "$(cat -- "$pid_file")" == "$demo_pid" ]]; then
        rm -f -- "$pid_file"
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
wait "$demo_pid"
