# 10 月 19 日 16:34 取回路径碰撞分析

## 本轮证据

日志提交 `4043a47` 包含 `results/launch_20261019_163211_169319.txt`，以及四轮配套 CSV、摘要。

| 开始时间 | 结果 | 时长 | 补充信息 |
| --- | --- | --- | --- |
| 16:32:16 | DONE，第 1 轮 | 38.4 秒 | 完整取放、加工、放回与回零 |
| 16:32:58 | DONE，第 2 轮 | 32.6 秒 | 托盘预抓取、加工后预抓取各一次无效规划，重试恢复 |
| 16:33:34 | DONE，第 3 轮 | 31.2 秒 | 加工后预抓取一次无效规划，重试恢复 |
| 16:34:08 | ABORTED，第 4 轮 | 20.3 秒 | 加工后预抓取两次规划均无效 |

均为槽位 0。这四轮视觉定位及抓取前复核均成功，未出现重复动作服务保护或 PREEMPTED。
日志未包含启动脚本打印的域值，不能单凭本文件断定实际使用的 ROS 域编号。

最后一轮已完成取料、上料、退出加工台和加工。
16:34:28.279，取回预抓取规划得到 55 点路径，第 4、49、50 点被 MoveIt 判为碰撞，
碰撞对象为世界中的 `workpiece_0` 与 `panda_leftfinger`。
16:34:28.432 重试后，57 点路径中的第 50 点再次发生相同碰撞，返回
`INVALID_MOTION_PLAN (-2)`。两条无效路径均未执行。

两次诊断的当前关节状态有效、无碰撞，说明当前静止状态和生成的整条运动路径需要分别判断。
终端第 1905–1928 行给出了路径碰撞的明确证据；仅看 CSV 中的“当前状态有效”会遗漏这一点。
第四轮上料预放置还曾出现附着工件与加工台的路径碰撞，重新规划后成功。

## 修改依据与实现

`_place()` 末尾已经沿直线退出到加工台预抓取位置，随后等待加工。
等待期间本应用没有发出机械臂移动指令。旧 `_retrieve()` 却对完全相同的预抓取位姿
再次请求全局 OMPL 规划。七轴臂的同一个末端位姿可对应不同关节姿态；重新采样目标 IK
可能生成绕行路径，增加手指靠近工件的机会。日志没有保存全部规划关节轨迹，因此
不能断言最后两条无效路径一定发生了 IK 分支切换。

现在取回预抓取改用已有的 `_goto_linear()`，从当前状态做局部直线对齐，随后才下降取料。
该过程继续检查新关节反馈、避碰、路径完整率以及当前状态到轨迹首点/相邻点的关节跳变。
路径不完整或执行失败时停止，既不切回全局绕行，也不将工件标记为已取回。

此外，启动日志第 74–76 行显示 OMPL 的 TOTG 使用默认 `path_tolerance=0.1`、
`resample_dt=0.1`。TOTG 会拟合并重采样路径，允许新点偏离原始几何路径。
这可以解释“找到路径但后处理后验证无效”的一种机制，但日志不足以证明其为唯一原因。
本次保留原适配器及 MoveIt 最终路径验证，将 OMPL 参数调整为：

| 参数 | 原默认值 | 新值 | 含义 |
| --- | --- | --- | --- |
| `ompl.path_tolerance` | 0.1 | 0.001 | 收紧关节空间路径拟合容差；旋转关节为 rad 量级，不能换算成固定毫米误差 |
| `ompl.resample_dt` | 0.1 秒 | 0.02 秒 | 加密重采样后的轨迹时间点 |

参数仅调整 OMPL 管线。Humble 的 GetCartesianPath 服务内部自行使用 TOTG，
不读取这两个 OMPL 参数。局部直线改动减少冗余目标求解，不能据此声称消除所有后处理偏离。
本次不改变允许碰撞矩阵、抓取点位或执行失败判定，无效规划仍不能执行。

依据为 [Humble TOTG 官方说明](https://moveit.picknik.ai/humble/doc/examples/time_parameterization/time_parameterization_tutorial.html)、
[Humble 参数读取实现](https://github.com/moveit/moveit2/blob/humble/moveit_ros/planning/planning_request_adapter_plugins/src/add_time_optimal_parameterization.cpp)
及 [Humble Cartesian 服务实现](https://github.com/moveit/moveit2/blob/humble/moveit_ros/move_group/src/default_capabilities/cartesian_path_service_capability.cpp)。

## 更新和复测

在原终端按 Ctrl+C，等待退出后，逐行执行：

```bash
cd ~/demo_ws
git pull --ff-only origin debug-pr2-fix
bash stop_demo.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=42
bash restart_demo.sh
```

观察至少连续 5 轮 DONE，重点核对“取回预抓取点”改为直线规划。
参数可在另一个同域终端核对：

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
ros2 param get /move_group ompl.path_tolerance
ros2 param get /move_group ompl.resample_dt
```

预期分别为 0.001、0.02。之后分别检查槽位 1、2，不能用槽位 0 的成功代替全部槽位验收。
若仍中止，请继续提交同轮 CSV、摘要和完整 `launch_*.txt`；中止后的工件可能留在加工台，
不要直接重复调用 start，应退出并重启以恢复初始场景。

## 测试范围

新增离线回归通过实际控制器的 Cartesian 服务与执行回调替身，验证取回流程使用局部直线，
以及预抓取失败、下降失败、附着失败和提起失败时的工件归属与动作边界。
原有部分路径拒绝、绝对关节步长限制、重复服务保护和执行终态检查继续测试。
离线测试不验证真实 Panda IK、TOTG 几何路径、碰撞结果或 DDS；连续运行效果仍需 Ubuntu 仿真验证。

终端还出现 RViz 轨迹预览 `No robot state or robot model loaded` 及图像订阅 QoS 不兼容告警。
这些提示与本轮规划碰撞分别处理；前三轮同样存在预览报错但任务完成，现有证据不支持将其作为
第四轮中止的直接原因。
