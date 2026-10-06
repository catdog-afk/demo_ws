# ROS 结课项目：A2 模拟机床上下料

> 工业场景：机械臂为加工工位取放工件 —— 从料盘取料，放入模拟加工台，等待加工结束后取回。

## 一、演示与运行

### 环境要求

- Ubuntu 22.04 + ROS2 Humble
- 依赖安装（一次性）：

```bash
sudo apt install ros-humble-moveit ros-humble-moveit-visual-tools \
    ros-humble-rviz-visual-tools \
    ros-humble-moveit-resources-panda-description \
    ros-humble-moveit-resources-panda-moveit-config \
    ros-humble-ros2-control ros-humble-ros2-controllers
```

### 编译

```bash
cd ~/demo_ws
colcon build --symlink-install
source install/setup.bash
```

### 一键启动完整演示

```bash
ros2 launch machining_demo demo.launch.py
```

启动后 RViz 自动打开，任务 **5 秒后自动开始**，依次展示：
机床忙碌（初始 10s）→ 等待空闲 → 取料 → 上料 → 加工（5s，显示进度）→ 取回 → 放回原槽位 → 回零成功 → 完成。
**每轮完成后停留 3 秒，自动开始下一轮，无需重新启动终端或 RViz。** 默认重复加工
`slot` 指定的工件，每轮独立保存 CSV 和摘要到 `~/demo_ws/results/`，状态标注显示已完成的轮数。

只演示一轮时使用：

```bash
ros2 launch machining_demo demo.launch.py repeat_cycle:=false
```

单轮完成后仿真继续运行，可调用 `/task/start` 开始下一轮。在连续运行的轮间停留期间调用
`/task/cancel` 会停止后续循环并保持 `IDLE`，之后可用 `/task/start` 继续。

### 场景颜色

桌面为深灰色、料盘为蓝色、加工台为浅灰色。槽位 0/1/2 的工件分别为橙色、黄色、紫色，
取料、夹爪附着、上料和放回后保持颜色一致。RViz 默认视角调整为较近的俯视，便于观察工件。

> ⚠️ **一次只能运行一个演示实例**。重复启动会导致控制器重复加载、
> `/move_action` 动作名冲突，机械臂报"取料失败"。
> 推荐从第一次启动就使用当前仓库的 `bash restart_demo.sh`。脚本只清理自己管理的演示进程组，
> 再启动当前仓库的构建结果；可附加 launch 参数，如 `bash restart_demo.sh machining_sec:=15.0 machining_timeout:=8.0`。
> 手动 `ros2 launch` 启动的旧实例，需要先在原终端按 Ctrl+C 退出。
> 按一次 Ctrl+C 后等待进程退出日志结束并回到命令提示符，再启动下一次。
> 脚本使用 `--noninteractive`，让独立会话中的 launch 主动向子节点传递退出信号。
> 另外注意：电脑休眠唤醒后 ROS 进程会失去 DDS 发现能力（服务调用卡在
> "waiting for service"），此时同样用 restart_demo.sh 重启即可。

关闭后怀疑进程残留时，可先查看操作系统进程（此命令只查看，不结束进程）：

```bash
ps -eo pid,ppid,stat,args | grep -E '[m]achining_demo|[m]ove_group|[r]os2_control_node|[r]obot_state_publisher|[r]viz2'
```

输出可能包含其他 ROS 工程使用的公共节点，应结合命令路径和父进程确认归属。
`ros2 node list` 的显示也可能受 DDS 发现和 CLI 缓存影响，不能单独作为残留进程的证明。
确认进程属于已停止的本演示后，可以对具体 PID 发 `kill -INT PID`；若启动器还在，优先对其 PID 发信号。
脚本只能管理由它启动的实例；首次改用脚本前，仍需退出此前手动启动的仿真。

### 常用参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `auto_start` | true | 是否自动开始演示 |
| `slot` | 0 | 取料槽位 0~2 |
| `auto_start_delay` | 5.0 | 自动启动延迟（秒） |
| `repeat_cycle` | true | 成功完成后连续运行；设为 false 时单轮运行 |
| `repeat_delay` | 3.0 | 每轮完成后的停留时间（秒），必须大于 0 |
| `initial_busy_sec` | 10.0 | 机床初始忙碌时长，便于观察忙碌等待阶段 |
| `machining_sec` | 5.0 | 模拟加工时长 |
| `machining_timeout` | 8.0 | 加工超时阈值（拓展：超时提示） |
| `timeout_retry_once` | true | 超时后延长一次等待窗口，不重新开始加工 |
| `station_timeout` | 30.0 | 等待场景、服务、工位 READY 的超时阈值 |
| `arm_call_timeout` | 660.0 | 整个取放动作服务的超时阈值，包含多段规划与执行 |
| `use_rviz` | true | 是否启动 RViz |
| `results_dir` | ~/demo_ws/results | 运行记录输出目录 |

演示"加工超时警告"（条件变化场景）：

```bash
ros2 launch machining_demo demo.launch.py machining_sec:=15.0 machining_timeout:=8.0
```

（等待加工 8 秒时触发超时警告，警告停留至少 1 秒；延长一次等待窗口后正常完成。）

演示“持续超时中止”：

```bash
ros2 launch machining_demo demo.launch.py machining_sec:=30.0 machining_timeout:=4.0
```

第二次超时后进入 `ABORTED`。执行过程中取消或中止后不自动启动新任务，保留工件位置供检查，
应先退出原演示再重新启动。`/machine/reset` 在工件仍在位或正在加工时拒绝复位，避免伪造工位空闲。
`/task/cancel` 在当前动作结束并确认占用更新后生效；服务请求超时不会停止远端机械臂运动。

### 手动控制（可选）

```bash
ros2 service call /task/start demo_interfaces/srv/StartTask "{slot: 1}"   # 启动槽位 1 任务
ros2 service call /task/cancel demo_interfaces/srv/CancelTask "{}"       # 取消任务
ros2 service call /machine/reset demo_interfaces/srv/ResetMachine "{}"   # 复位机床
ros2 topic echo /task/state                                              # 查看任务状态
```

### 自动化测试

离线回归测试（Python 3.10+，不要求安装 ROS）：

```bash
python -m unittest discover -s tests -v
```

测试执行真实节点回调，替换 ROS 消息、时钟与服务传输，覆盖状态机、超时、取消、
工件坐标与逐轮记录。该测试不验证 DDS 通信、碰撞检测或实际 MoveIt 轨迹。

ROS 集成验收需要两个终端；第一个终端禁止自动启动，第二个终端发起本次任务：

```bash
# 终端 1
ros2 launch machining_demo demo.launch.py auto_start:=false repeat_cycle:=false
```

```bash
# 终端 2（先 source install/setup.bash）
ros2 run machining_demo demo_test --ros-args -p slot:=1
```

测试必须观察本轮全部核心阶段，且找到本轮新生成、槽位匹配的 CSV 和摘要，
不会使用仓库自带的旧日志判定成功。自定义 `results_dir` 时，第二个终端须传相同的 `-p output_dir:=...`。

超时后完成场景：终端 1 增加 `machining_sec:=15.0 machining_timeout:=8.0`，
终端 2 增加 `-p require_timeout:=true`。
持续超时场景：终端 1 增加 `machining_sec:=30.0 machining_timeout:=4.0`，
终端 2 增加 `-p expected_final:=ABORTED -p require_timeout:=true`。

每次条件变化前退出并重新启动演示；正常任务完成后可在同一实例重复运行不同槽位。
连续运行方式使用默认启动命令；集成验收使用单轮模式，便于核对本次任务的结束状态和日志。
GitHub Actions 自动执行离线回归测试，ROS/MoveIt 集成验收需在 Ubuntu 22.04 + ROS2 Humble 上另行执行。

### 连续循环失败的排查与复测

`debug-pr2-fix` 中上传的 10 月 5 日 23 点之后记录显示：两轮在预抓取成功后的下降阶段中止，
另一次在放下工件后的退出加工台阶段中止。旧 CSV 只记录了“动作执行失败”，没有 MoveIt 返回码，
因此这些记录能定位失败阶段，不能确认唯一根因。

控制代码现将夹爪直线关节的目标容差从 10mm 改为 0.5mm；靠近工件的下降和退出改为
`/compute_cartesian_path` 直线规划，再通过 `/execute_trajectory` 执行。直线路径保持碰撞检查，
完整路径才允许执行；路径不完整时直接中止，不执行部分路径。长距离转移仍使用 OMPL。
每段运动前等待新的完整关节反馈，失败时 CSV 和摘要保留动作名称、返回码或直线路径完成比例，
CSV 还记录当时关节位置和可查询到的碰撞对象。

复测前在原终端按 Ctrl+C，等待退出完成，再更新代码、编译。建议保存完整终端输出：

```bash
ros2 launch machining_demo demo.launch.py 2>&1 | tee ~/pick_failure.log
```

连续观察至少 5 轮 `DONE`，再用 `slot:=1`、`slot:=2` 分别启动检查另外两个槽位。
若仍出现 `ABORTED`，保留本次 `summary_*.txt`、`task_log_*.csv` 以及 `~/pick_failure.log`。
这些修改通过离线控制逻辑测试，真实 IK、碰撞和连续运动效果仍需在 ROS2 仿真环境复测。
重启脚本使用 SIGINT 请求 launch 退出，并等待旧实例结束；若未能退出则保留进程标识、拒绝启动新实例。

### 录制演示视频

```bash
ffmpeg -f x11grab -video_size 1920x1080 -framerate 30 -i :0.0 -c:v libx264 -preset ultrafast demo_run.mp4
```

## 二、系统结构与 ROS 通信关系

```
                          ┌──────────────┐  /machine/status(MachineStatus) ┌──────────────┐
                          │ machine_     │◄─────────────────────────────────│  task_       │
                          │ simulator    │  /machine/workpiece_present      │  manager     │
                          │ (角色C 工位)  │◄─────────────────────────────────│ (角色D 状态机) │
                          └──────┬───────┘                                  └──────┬───────┘
                                 │ /machine/status                              │ /task/state(TaskState)
                                 ▼                                              │
                          ┌──────────────┐  /arm/command(ArmCommand)      ┌──────▼───────┐
                          │  arm_        │◄────────────────────────────────│  result_     │
                          │  controller  │  /tray/set_occupancy           │  recorder    │
                          │ (角色B MoveIt)│◄────────────────────────────────│ (角色D 记录)  │
                          └──────┬───────┘                                 └──────────────┘
                                 │ 规划场景(桌面/料盘/工件/加工台)                 ▲
                          ┌──────▼───────┐  /tray/occupancy               /task/state
                          │  scene_      │──────────────────────────────────┘
                          │  manager     │  静态TF: table/tray/machine
                          │ (角色A 场景)  │
                          └──────────────┘

        底层：move_group（MoveIt 运动规划 / /move_action 动作 / /apply_planning_scene 服务）
              + ros2_control（mock_components 硬件模拟 + 关节轨迹控制器） + RViz2 可视化
```

**话题 / 服务接口一览**（自定义接口包 `demo_interfaces`）：

| 接口 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/task/state` | 话题 TaskState | task_manager → 全局 | 任务状态机状态（10 种状态 + 槽位 + 说明文字，保留最新状态） |
| `/machine/status` | 话题 MachineStatus | machine → task/recorder | 工位信号（READY/BUSY/MACHINING/DONE + 进度） |
| `/machine/workpiece_present` | 话题 Bool | arm → machine | 工件在位信号 |
| `/tray/occupancy` | 话题 TrayOccupancy | scene → 全局 | 料盘槽位占用 |
| `/arm/status` | 话题 String | arm → recorder | 机械臂执行反馈 |
| `/arm/command` | 服务 ArmCommand | task → arm | 动作指令 pick/place/retrieve/return/home/open/close |
| `/task/start` `/task/cancel` | 服务 | 外部 → task | 启动/取消任务 |
| `/tray/set_occupancy` | 服务 | task → scene | 更新槽位占用 |
| `/machine/reset` | 服务 | 外部 → machine | 复位机床（测试用） |

**任务状态机**：

```
IDLE → WAIT_STATION(场景/服务/工位就绪等待) → PICK(取料) → PLACE(上料) → WAIT_MACHINING(等加工)
     → RETRIEVE(取回) → RETURN(放回、确认料盘占用、回零) → DONE → IDLE
     DONE --repeat_cycle=true，停留 repeat_delay 秒--> IDLE → WAIT_STATION（下一轮）
     DONE --取消后续循环--> IDLE（保持仿真，可手动启动）
     执行中 --取消/异常--> ABORTED（停止后续自动循环）
     WAIT_MACHINING --首次超时--> TIMEOUT(警告) → WAIT_MACHINING(延长一次等待)
     WAIT_MACHINING --第二次超时--> ABORTED
```

## 三、自行完成的核心逻辑

1. **任务流程状态机**（task_manager.py）——任务状态切换、工位忙碌等待、加工超时提示与重试、取消/异常处理；
2. **工位信号模拟**（machine_simulator.py）——机床状态机（忙碌→就绪→加工→完成）与进度发布；
3. **上下料动作流程**（arm_controller.py）——取料/上料/取回/放回的完整动作编排，工件附着与分离；
4. **运行记录**（result_recorder.py）——每轮任务独立生成带槽位的 CSV 与摘要，执行中即时刷新，关闭节点时保留未完成证据；
5. **验收规则与回归测试**（evidence.py、tests/）——校验 A2 核心阶段完整性，验证忙碌、信号中断、超时和取消等条件变化。

## 四、复用的开源资源（来源声明）

| 资源 | 来源 | 用途 |
|------|------|------|
| Franka Emika Panda 机械臂 URDF/SRDF/运动学配置 | [moveit_resources](https://github.com/moveit/moveit_resources)（ROS2 apt 包 moveit_resources_panda_description / moveit_resources_panda_moveit_config） | 机器人模型、规划组、KDL 逆运动学求解器 |
| MoveIt 2 运动规划框架 | [MoveIt](https://moveit.picknik.ai/)（ros-humble-moveit） | 运动规划、碰撞检测、工件附着 |
| ros2_control + mock_components | [ros2_control](https://control.ros.org/)（官方 demo 同款） | 硬件模拟、关节轨迹执行、关节状态发布 |
| RViz2 | ROS2 官方 | 可视化 |

## 五、成员分工对应模块

| 角色 | 主要工作 | 对应代码 |
|------|----------|----------|
| A 仿真与平台 | 模型、场景、启动配置 | scene_manager.py、demo.launch.py、machining_demo.rviz |
| B 动作与执行 | 机械臂运动、执行状态反馈 | arm_controller.py（/arm/status、/arm/command） |
| C 感知与交互 | 工位信号处理 | machine_simulator.py（/machine/status） |
| D 任务与记录 | 任务流程、取消/异常、日志 | task_manager.py、result_recorder.py |
| E 集成与测试 | 接口联调、测试脚本、可视化 | demo_test.py、README 参数表格 |

## 六、目录结构

```
demo_ws/
├── README.md                  # 本文件（启动说明）
├── results/                   # 运行记录输出（task_log_*.csv + summary_*.txt）
└── src/
    ├── demo_interfaces/       # 自定义话题/服务接口（3 msg + 5 srv）
    └── machining_demo/        # 主功能包
        ├── machining_demo/    # 5 个功能节点 + 共享布局/验收规则 + 集成测试脚本
        ├── launch/demo.launch.py
        └── rviz/machining_demo.rviz
```

## 七、PPT A2 验收对应关系

依据课程作业 PPT 第 3 页最低技术要求、第 6 页 A2 题目及第 14 页汇报要求：

| 要求 | 实现与验收证据 |
|------|----------------|
| 从料盘取料、上料、等待加工、取回 | `PICK → PLACE → WAIT_MACHINING → RETRIEVE → RETURN → DONE`，回到原槽位并确认回零 |
| 展示各阶段状态 | `/task/state`、`/machine/status`、RViz 状态标注和每轮 CSV |
| 工位信号、机械臂和夹爪控制 | 独立 machine/arm/task 节点，工件在位 Bool，MoveGroup 与夹爪规划组 |
| 至少两个 ROS 功能节点，有自行开发逻辑 | 五个功能节点、自定义 msg/srv、任务状态机、工位模拟和结果记录 |
| 从启动到结束可复现、保存运行记录 | 一键 launch、逐轮 CSV/摘要、正常和超时的集成验收命令 |
| 补充一次条件变化或失败处理 | 忙碌等待、加工超时提示、持续超时中止及任务取消 |
| 注明开源来源及本人工作 | 第三、四、五节；实际成员姓名和贡献由小组填写 |

建议视频连续展示默认完整循环，再展示一次超时条件变化。源码已提供，最终汇报 PPT 和
视频仍需小组结合实际 Ubuntu 仿真运行录制。仓库既有 `results/` 为历史数据，不代表本次修改的仿真验收结果。
