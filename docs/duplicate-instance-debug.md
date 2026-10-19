# 10 月 19 日 15:14 重复动作服务中止

## 本轮日志证明了什么

日志提交 `33920a2` 新增同轮 CSV 和摘要：

- `results/task_log_20261019_151424_865840_49d441d6a2e84be39ba9bb8fbc763624.csv`
- `results/summary_20261019_151424_865840_49d441d6a2e84be39ba9bb8fbc763624.txt`

15:14:29.877 已成功识别三个工件，坐标分别为
`(0.4622, 0.1164, 0.0620)`、`(0.4408, 0.2429, 0.0620)`、
`(0.4561, 0.3948, 0.0620)` 米。
15:14:29.929，在夹爪张开、发送运动目标前发现：

| 动作状态话题 | 检测结果 |
| --- | --- |
| `/move_action/_action/status` | 两个 `/move_group` 发布端 |
| `/execute_trajectory/_action/status` | 两个 `/move_group` 发布端 |
| `/panda_arm_controller/follow_joint_trajectory/_action/status` | 两个 `/panda_arm_controller` 发布端 |

15:14:29.951 任务进入 `ABORTED`，本轮时长 5.1 秒。
这次中止由重复服务保护触发，图像定位已经成功。
CSV 没有主机地址和操作系统进程清单，不能据此断定两个发布端都在本机。
同名 ROS 节点可以属于不同实例，不能通过按节点名去重绕过保护。

## 本次修改

旧 `restart_demo.sh` 主要检查保存的启动器 PID；启动器已退出、子节点仍存活时，
存在遗漏旧节点、随后再启动一套的窗口。本次修改关闭这一窗口，但尚不能证明
用户本轮重复端点的唯一来源就是该窗口。

- 新启动给整套节点继承工程路径与随机实例标签。退出、重启时检查整个独立会话，
  不以启动器是否存活作为清理完成的依据。
- 兼容能核对到本工作区节点路径的旧版独立会话。只有同用户、同工程、有独立会话证据，
  且会话内全部成员均为已知仿真节点时才清理；归属不明则停止清理并拒绝新启动。
- 发送信号前复核 PID、启动时间、会话、进程组、程序入口及标签；使用 Linux pidfd
  绑定进程身份，避免 PID 被复用后信号发给其他进程。
- 清理依次使用 SIGINT（等待 8 秒）、SIGTERM（4 秒）、SIGKILL（2 秒），期间持续检查
  该会话的晚生子节点。已退出的僵尸不计为运行中的节点。
- 启动与清理共用文件锁；退出清理只处理自己的实例，防止旧终端退出时结束刚启动的新实例。
  子节点和日志转存进程不继承启动锁。
- 新增 `bash stop_demo.sh`，无需编译或加载 ROS 环境即可关闭已确认归属的本工程实例。
- 保留重复动作服务保护，错误信息补充发布端 GID 和 `ROS_DOMAIN_ID`，便于后续区分端点。

## Ubuntu 更新与复测

先在原演示终端按一次 Ctrl+C，等退出结束。在工程目录执行：

```bash
cd ~/demo_ws
git switch debug-pr2-fix
git pull --ff-only origin debug-pr2-fix
bash stop_demo.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
bash restart_demo.sh repeat_cycle:=false
```

`stop_demo.sh` 可以在重新编译之前使用。若脚本报归属不明，保留其输出，先确认报出的具体 PID
属于哪个启动终端；不要忽略错误继续另开仿真。此前直接用 `ros2 launch` 启动、且与终端共享
会话的旧实例，需要回原终端关闭，脚本不会批量结束同名的其他 ROS 工程进程。

复测一轮完成后，可另开终端，加载同一工程环境，再调用：

```bash
source /opt/ros/humble/setup.bash
source ~/demo_ws/install/setup.bash
ros2 service call /task/start demo_interfaces/srv/StartTask "{slot: 0}"
```

关闭整套演示可在启动终端按 Ctrl+C，或另开终端运行：

```bash
cd ~/demo_ws
bash stop_demo.sh
```

## 清理本机后仍有重复服务

先确认本机是否还有其他工作区启动的仿真；不要关闭不属于本次演示的工程。
ROS 2 同域节点可以相互发现，另一台主机也可能贡献同名发布端。
可选择一个未被其他演示使用的域，例如 42，再从关闭状态启动：

```bash
ROS_DOMAIN_ID=42 bash restart_demo.sh repeat_cycle:=false
```

与之通信的其他终端也必须设置相同域后再运行 ROS 命令：

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/demo_ws/install/setup.bash
ros2 service call /task/start demo_interfaces/srv/StartTask "{slot: 0}"
```

脚本默认保留用户已有域配置，不自动改变 ROS 域，也不能清理其他主机的进程。
域隔离原理及取值限制见 [ROS 2 Humble 官方 Domain ID 说明](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html)。

如仍中止，请提交同轮 CSV、摘要以及 `results/launch_日期_时间_PID.txt` 完整终端输出。
CSV/摘要默认在 `~/demo_ws/results/`；启动脚本的终端输出在脚本所在工作区的 `results/`。

## 验证范围

回归测试覆盖会话归属、未知成员拒绝、进程身份变化、实例隔离、晚生子节点等情况。
另有两项 Linux 真进程测试，分别验证正常节点退出和启动器退出后的孤儿清理。
这些检查验证进程管理与离线控制逻辑；ROS/DDS 发现、MoveIt 规划执行和 RViz 连续演示
仍需在 Ubuntu + ROS2 Humble 环境复测。
