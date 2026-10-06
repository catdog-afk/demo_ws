# PR2 连续运行故障分析

## 上传日志中的事实

分支 `debug-pr2-fix` 的日志提交 `b2b7992`。以下时间来自用户 Ubuntu 仿真生成的文件，
范围为 2026-10-05 23:00 之后；既有的较早记录未用作本次改动的验收证据。

| 本轮开始时间 | 结果 | 最后成功运动 | 失败阶段 |
| --- | --- | --- | --- |
| 10-05 23:48:52 | DONE，第 1 轮 | 回零位 | 无 |
| 10-05 23:49:40 | ABORTED，第 2 轮 | 预抓取点(槽位0) | 抓取点(槽位0) |
| 10-05 23:55:52 | ABORTED，重新启动后的第 1 轮 | 预抓取点(槽位0) | 抓取点(槽位0) |
| 10-06 00:31:01 | DONE，第 1 轮 | 回零位 | 无 |
| 10-06 00:32:05 | ABORTED，第 2 轮 | 放置点、夹爪张开及工件分离 | 退出加工台 |

失败的抓取下降没有执行完成；取料前张开夹爪已成功。另一轮取料全部成功，退出加工台失败。
因此不能把问题简单归为“第二轮重复张开夹爪失败”或任务状态没有复位。
失败的 CSV 没有 MoveIt 返回码，无法仅靠现有证据区分碰撞、目标无有效 IK 或其他规划问题。

## 代码问题与修改

- 原 `_joint_goal` 对手指和旋转关节都使用 `0.01` 容差。手指关节单位为米，
  张开目标 35mm 可被 25mm 满足；旧程序没有保证抓取前的开度。现在手指容差为 0.5mm。
- 原程序在靠近工件的每一步重新用 OMPL 求一个位姿目标；几何上相近的目标可能得到不同关节解。
  下降、提起和放置后的退出现在沿当前状态进行 2mm 步长的笛卡尔插值，保留碰撞检查和关节跳变检测。
  只有返回成功、完成比例为 100% 且非空的轨迹才发送执行。长距离转移继续使用 OMPL。
- 原程序在动作结果到达后立即读取缓存关节。现在发送下一段请求前等待所有规划关节的新反馈，
  最多 2 秒，无完整反馈则停止发送运动目标。
- 原运动错误只写入终端，服务响应和 CSV 丢失阶段与错误码。现在服务响应和 `/arm/status`
  同时保存具体错误；失败时查询当前碰撞对并记录当前关节、附着工件和加工台工件。
- 原重启脚本使用 SIGTERM 结束 launch；改为给 launch 发 SIGINT，让它管理子进程退出。
  等待 20 秒后仍未退出就停止重启，保留进程标识，不并行启动第二个受脚本管理的实例。

前两项是对抓取附近运动不确定性的修正，尚未获得新版本的 ROS2 仿真证据。
当前日志不足以证明它们是上述全部报错的唯一根因，也不能证明用户当时存在残留进程。

## 接口依据

- [MoveIt Humble GetCartesianPath](https://github.com/moveit/moveit_msgs/blob/humble/srv/GetCartesianPath.srv)：
  指定位姿路点、碰撞检查、步长、关节跳变与速度参数；响应包含路径完成比例和错误码。
- [MoveIt Humble CartesianPathService](https://github.com/moveit/moveit2/blob/humble/moveit_ros/move_group/src/default_capabilities/cartesian_path_service_capability.cpp)：
  从当前规划场景合并请求状态，调用 IK 插值、碰撞检查和时间参数化。
- [MoveIt Humble ExecuteTrajectory](https://github.com/moveit/moveit_msgs/blob/humble/action/ExecuteTrajectory.action)：
  执行规划服务返回的轨迹并检查执行错误码。
- [MoveIt Humble GetStateValidity](https://github.com/moveit/moveit_msgs/blob/humble/srv/GetStateValidity.srv)：
  诊断当前状态及碰撞对象；诊断失败不会覆盖原动作失败原因。
- [ROS2 Humble LaunchService](https://github.com/ros2/launch/blob/humble/launch/launch/launch_service.py)：
  SIGINT 触发 launch 的关闭流程。

## 验证边界

离线回归测试覆盖原节点状态机、两轮动作编排、夹爪容差、完整/部分/异常直线路径、
执行拒绝与超时、关节反馈新鲜度以及具体失败信息记录。
这些测试替换 ROS 传输，不验证 Panda IK、实际碰撞检测、DDS 或控制器执行。
Ubuntu 22.04 + ROS2 Humble 的复测步骤在 README“连续循环失败的排查与复测”中；
需要至少 5 个连续成功周期及三个槽位的实际运行日志才能进一步确认仿真效果。
