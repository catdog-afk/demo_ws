# 10 月 19 日 14:56 图像过期导致抓取停止

日志提交 `d53ac00` 新增一轮失败记录：
`task_log_20261019_145634_078618_2bc72d4f23db47329cc1789ad1b9525c.csv`。

## 日志可以确认的事实

- 14:56:39.072 首次成功识别三个工件，工件0为 `(0.4622,0.1164,0.0620)`。
- 14:56:39.330 夹爪张开完成，14:56:43.277 预抓取点执行完成。
- 14:56:43.301 复核返回“相机图像缺失或过期”，14:56:43.348 任务 ABORTED。
- 本轮没有开始下降，没有执行抓取附着，也没有返回颜色、轮廓或物料间距错误。

这次已证实的是复核时没有可接受的新鲜图像。现有文件没有相机查询时长、DDS 状态、
相机节点终端输出或实际帧龄，因此不能唯一确认断流发生在相机查询、图像发布、接收还是处理阶段。

## 找到的代码问题与修改

1. 旧程序只对“图像时间戳没有更新”短暂等待；当识别服务返回“缺帧/过期”时却立即失败。
   新服务用 `retryable` 标记仅输入暂不可用的情况，机械臂最多等待 2 秒获取新图像。
   重新得到新鲜、同帧、同坐标系的完整观测，才继续复核位置变化和下降。
2. 虚拟相机的场景查询没有超时处理，一直未完成的请求会让所有后续 tick 直接返回。
   现在超过 0.8 秒取消本地等待并重新查询；已完成但延迟过长的结果也丢弃。
   取消客户端等待不等于取消远端 MoveIt 服务，代码没有把旧结果改时间后继续使用。
3. 原始图像、识别图与接收端统一使用 Best Effort、Keep Last 深度1，减少旧图排队和可靠重传等待。
   这是图像数据通道的调整，不是本轮 DDS 故障的已证实根因。
4. 图像采集帧龄改为先减整数纳秒再换算秒，避免浮点相减将同一采集时刻误判为未来。
5. 缺帧和过期日志分开，过期时保留收到间隔和采集帧龄。
   相机查询超时、延迟结果丢弃、服务不可用及成像恢复发布到 `/camera/status`，记录器写入同轮 CSV。

## 保留的拒绝条件

1 秒的新鲜度上限没有放宽，不返回固定槽位作为后备。
相机一直不恢复则等待超时中止，不能无限等或继续使用过期坐标。
错坐标系、未来时间戳、轮廓异常、同色歧义、缺物料或间距不足不会按断流重试。
预抓取点后的位移复核和原有碰撞/执行检查仍生效。

## 构建与复测

本次 **LocateWorkpieces 服务定义增加 retryable 字段**。先停止旧仿真，重新编译接口包和 Python 包，
再重新加载环境、启动新进程。不能只更新 Python 文件而继续用旧接口或旧节点。

```bash
cd ~/demo_ws
git pull --ff-only origin debug-pr2-fix
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
bash restart_demo.sh repeat_cycle:=false
```

先确认一轮完整成功，再重复运行。相机仍报错时一起上传新的 CSV、摘要和 `results/launch_*.txt`。
可在已加载环境的另一个终端观察：

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/status
```

新增离线测试覆盖靠近后短暂断帧恢复、永久断帧停止、轮廓错误不重试、采集时刻相等、
查询挂起恢复、延迟结果丢弃和相机状态记录。它们不验证真实 DDS 调度或 Ubuntu 机器负载。
本次实际断流的根因及改动后的集成效果仍需新仿真日志确认。

接口依据：[ROS2 Humble QoS 文档](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)、
[rclpy QoS 实现](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/qos.py)。
