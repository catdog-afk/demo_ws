#!/bin/bash
# A2 演示一键重启：先彻底清理旧实例，再启动新的（避免多实例冲突导致"取料失败"）
# 用法: bash ~/demo_ws/restart_demo.sh [可选 launch 参数]
#   例: bash ~/demo_ws/restart_demo.sh machining_sec:=15.0 machining_timeout:=8.0

echo ">> 清理旧演示进程..."
pkill -9 -f "ros2 launch machining_dem[o]" 2>/dev/null || true
pkill -9 -f "install/machining_dem[o]" 2>/dev/null || true
pkill -9 -f "moveit_ros_move_grou[p]" 2>/dev/null || true
pkill -9 -f "ros2_control_nod[e]" 2>/dev/null || true
pkill -9 -f "controller_manage[r]" 2>/dev/null || true
pkill -9 -f "robot_state_publishe[r]" 2>/dev/null || true
pkill -9 -f "rviz[2]" 2>/dev/null || true
sleep 3

echo ">> 启动演示（Ctrl+C 可退出）..."
source /opt/ros/humble/setup.bash
source ~/demo_ws/install/setup.bash
ros2 launch machining_demo demo.launch.py "$@"
