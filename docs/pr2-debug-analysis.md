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
- [ROS2 Humble ExecuteLocal](https://github.com/ros2/launch/blob/humble/launch/launch/actions/execute_local.py)：
  SIGINT 关闭事件在非交互模式下主动向子节点传递信号。重启脚本在独立会话中运行 launch，
  因此显式传入 `--noninteractive`，而不依赖子节点收到终端广播的 Ctrl+C。

## 验证边界

离线回归测试覆盖原节点状态机、两轮动作编排、夹爪容差、完整/部分/异常直线路径、
执行拒绝与超时、关节反馈新鲜度以及具体失败信息记录。
这些测试替换 ROS 传输，不验证 Panda IK、实际碰撞检测、DDS 或控制器执行。
Ubuntu 22.04 + ROS2 Humble 的复测步骤在 README“连续循环失败的排查与复测”中；
需要至少 5 个连续成功周期及三个槽位的实际运行日志才能进一步确认仿真效果。

## RViz 双影与卡顿的后续修正

用户报告取消 `RobotModel` 后双影消失，但动作显示变得卡顿。该现象与重复机器人显示层、
场景状态发布和渲染更新过慢相符，仍需实测消息频率来确认剩余卡顿的来源。

原配置同时启用 RobotModel、PlanningScene 机器人和半透明的 3x 轨迹预览。
现在默认仅显示 PlanningScene 机器人及附着工件。原 Scene Display Time 为 0.2 秒，
改为 0.02 秒；move_group 的场景发布上限显式设为 30Hz（`scene_update_hz` 可调整）。

原 RViz 节点使用 `moveit_config.to_dict()`，其中含有开启场景发布的参数。
根据 Humble PlanningSceneMonitor 源码，RViz 内部监视器会读取这些参数并可能发布到
与其订阅的 `/monitored_planning_scene` 相同的话题。现在只给 RViz 模型、语义、运动学和
规划显示参数，由 move_group 发布场景。是否曾在用户机器形成重复发布需话题信息确认。

依据：[MoveIt Humble 场景发布实现](https://github.com/moveit/moveit2/blob/humble/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp)、
[MoveIt Humble 场景显示实现](https://github.com/moveit/moveit2/blob/humble/moveit_ros/visualization/planning_scene_rviz_plugin/src/planning_scene_display.cpp)。
README 给出了实际消息频率和发布者的查看命令；本地没有 ROS2/RViz，未确认真实渲染效果。

## 10 月 6 日下午上传的日志

日志提交 `c51a95b` 新增 27 轮记录：23 轮 DONE、3 轮 ABORTED、1 轮 INTERRUPTED。
13:39 开始的一组任务连续完成 17 轮；13:50:27 的下一轮在关闭节点时标记 INTERRUPTED，
这条记录说明任务运行期间仿真退出，不等同于规划失败。

| 开始时间 | 失败阶段 | 直线路径比例 | 起始关节4 | 距模型软下限的余量 |
| --- | --- | --- | --- | --- |
| 13:51:01 | 取回抓取点 | 58.33% | -3.07180 | 0.00000 rad |
| 13:59:08 | 放置点 | 72.73% | -3.06997 | 0.00183 rad |
| 14:06:13 | 取回抓取点 | 9.09% | -3.06135 | 0.01045 rad |

这三次起始状态查询均为有效、未发现碰撞。该检查只覆盖起始状态，不排除路径中间的碰撞。
GetCartesianPath 的 SUCCESS 表示计算服务正常返回，部分路径仍不可执行。
路径可能因 IK、限位、碰撞或相对跳变过滤截断；当前证据不能唯一证明哪一个条件首先触发。
但极小的关节4限位余量是三次失败的共同特征，因此本次调整加工台点位并处理短轨迹的跳变筛选。

### 点位与轨迹修改

- 加工台从 `(0.10, -0.25, 0.05)` 移至 `(0.40, -0.25, 0.05)`；工件中心从
  `(0.10, -0.25, 0.122)` 同步为 `(0.40, -0.25, 0.122)`。所有放置/取回点从加工台几何计算。
  箱体保持位于桌面内且不与料盘相交，保留原有尺寸、工件间隙和颜色。
- 仅约 2cm 的直线运动不再按平均步长的 2 倍截断。完整轨迹在执行前检查各关节绝对变化，
  当前状态到首点、相邻点之间的变化均不得超过 0.20 rad，同时拒绝缺失关节和非有限数值。
  MoveIt 碰撞检查、100% 路径条件、执行结果检查保持生效。
- 不完整路径记录其未执行末端的关节位置，供后续确认截断原因；不执行不完整路径、不伪造 DONE。

### 离线运动学检查及限制

使用[官方 Panda URDF](https://github.com/moveit/moveit_resources/blob/ros2/panda_description/urdf/panda.urdf)
的关节变换和软限位，对三个失败关节快照做正运动学，得到的手部位置分别为
`(0.100443, -0.249483, 0.242109)`、`(0.099173, -0.250397, 0.241704)`、
`(0.099288, -0.250097, 0.241339)`，与原加工台预抓取点吻合。

采用 Jacobian 阻尼最小二乘、关节软限位约束，从这三个快照和 HOME 共四种种子求解新的
预抓取点，再按 2mm 采样下降到抓取点。新 x=0.40 的四条运动学路径均完成全部 11 个采样，
关节4最低余量分别为 0.584841、0.583939、0.587967、0.576526 rad。
原 x=0.10 的日志种子也存在可达的运动学路径，但余量只有约 0.004～0.009 rad，
说明原点位并非完全不可达，而是当前关节解的限位余量很小。

这个检查不使用 MoveIt KDL，也没有碰撞模型；它支持向外移动点位的选择，不能代替实机或
ROS2 仿真验证。离线回归覆盖几何位置同步、箱体不相交、完整轨迹的关节跳变拒绝、
首点与反馈一致性、无效轨迹拒绝和部分路径诊断；新布局的实际运行仍需用户复测。
