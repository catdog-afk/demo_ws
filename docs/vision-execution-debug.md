# 10 月 19 日抓取执行失败分析

日志提交 `6900bd9` 新增 18 轮记录：16 轮 DONE、2 轮 ABORTED。
其中 17 轮为 10 月 6 日记录，最新一轮为 10 月 19 日视觉版本。

## 最新一轮：视觉定位正常，执行被中止

`task_log_20261019_144045_510229_75f34c7242a1488990e8b257e948438c.csv`：

- 14:40:50.624 首次定位工件0为 `(0.4622,0.1164,0.0620)`，三个工件完整。
- 14:40:54.632 完成预抓取点运动，14:40:54.654 的复核坐标与首次完全一致。
- 14:40:54.676 开始直线路径，14:40:55.331 返回 `PREEMPTED (-7)`。
- 当前状态检查有效，未检测到起点碰撞。工件尚未附着，任务在 PICK 中止。

这不是图像缺失、定位不一致或部分路径拒绝。代码已经通过路径完整性条件，
失败来自 ExecuteTrajectory 返回的执行状态。但 CSV 不包含控制器输出、取消请求来源或
进程清单，不能证明一定是残留进程、用户取消或 MoveIt 内部竞争。

Humble 的 [ExecuteTrajectory 实现](https://github.com/moveit/moveit2/blob/humble/moveit_ros/move_group/src/default_capabilities/execute_trajectory_action_capability.cpp)
将轨迹执行管理器的 PREEMPTED 状态映射为 -7。
[MoveGroup 实现](https://github.com/moveit/moveit2/blob/humble/moveit_ros/move_group/src/default_capabilities/move_action_capability.cpp)
的组合规划执行也使用同一执行管理器。因此混用两个执行入口增加了状态交接复杂度，
但源码依据不能单独证明本次一定因交接而失败。

## 较早一轮：回零规划无效

`task_log_20261006_151259_814098_3185c320c21b49beac313b66c42591e9.csv`：
前 12 轮成功，第 13 轮放回并退出料盘后，回零返回 `INVALID_MOTION_PLAN (-2)`。
当时没有附着工件、加工台没有工件、当前状态查询有效。
缺少完整规划器日志，不能确定哪一个轨迹验证条件失败。
另四轮 10 月 6 日记录包含 TIMEOUT 后继续等待并 DONE，支持超时处理流程已实际运行。

## 修改及约束

1. MoveGroup 改为仅规划，所有普通、夹爪、回零与笛卡尔轨迹都经 ExecuteTrajectory 执行。
   这消除本应用混合执行入口，未调整碰撞容差或将 PREEMPTED 当作成功。
2. 只有尚未执行的规划返回 -1/-2，才刷新关节状态并允许重规划一次。
   失败执行和 PREEMPTED 不自动重试，避免意外继续运动。
3. ROS 动作终态必须为 SUCCEEDED(4)，且 MoveIt 错误码为 SUCCESS，才判为成功。
   失败保留最后反馈、ROS终态、goal_id及动作状态话题发布节点。
4. 动作状态话题发现重复服务端时，不再发送运动目标。
   这能阻止已发现的重复服务，不证明没有其他控制客户端或发现延迟。
5. `restart_demo.sh` 自动用 tee 保存 `results/launch_日期_时间_PID.txt`，保留原 Ctrl+C 转发逻辑。
   再报错时将该文件与同轮 CSV、摘要一起提交，才能追踪控制器取消原因。

规划时避碰、笛卡尔完整性和执行结果检查仍然生效。ExecuteTrajectory 不等同于
MoveGroup 组合执行的动态场景重规划监视；本项目仅用于既定工位和启动前摆放的物料，
不能据此宣称已支持运动过程中移动障碍物的动态避障。

离线回归验证入口分离、规划重试边界、PREEMPTED 不重试、终态检查和重复服务拒绝。
尚未在 ROS2 环境重现本次抢占，也未证明这些改动已消除用户机器上的执行失败。
