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
机床忙碌（3s）→ 等待空闲 → 取料 → 上料 → 加工（5s，显示进度）→ 取回 → 放回料盘 → 完成。
运行记录自动保存到 `~/demo_ws/results/`。

> ⚠️ **一次只能运行一个演示实例**。重复启动会导致控制器重复加载、
> `/move_action` 动作名冲突，机械臂报"取料失败"。
> 重启请用：`bash ~/demo_ws/restart_demo.sh`（自动清理旧实例后再启动，
> 可附加 launch 参数，如 `bash ~/demo_ws/restart_demo.sh machining_sec:=15.0 machining_timeout:=8.0`）。
> 另外注意：电脑休眠唤醒后 ROS 进程会失去 DDS 发现能力（服务调用卡在
> "waiting for service"），此时同样用 restart_demo.sh 重启即可。

### 常用参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `auto_start` | true | 是否自动开始演示 |
| `slot` | 0 | 取料槽位 0~2 |
| `initial_busy_sec` | 3.0 | 机床初始忙碌时长（拓展：忙碌等待） |
| `machining_sec` | 5.0 | 模拟加工时长 |
| `machining_timeout` | 8.0 | 加工超时阈值（拓展：超时提示） |
| `use_rviz` | true | 是否启动 RViz |
| `results_dir` | ~/demo_ws/results | 运行记录输出目录 |

演示"加工超时警告"（条件变化场景）：

```bash
ros2 launch machining_demo demo.launch.py machining_sec:=15.0 machining_timeout:=8.0
```

（加工 8 秒时触发超时警告，任务等待重试一次后正常完成。）

### 手动控制（可选）

```bash
ros2 service call /task/start demo_interfaces/srv/StartTask "{slot: 1}"   # 启动槽位 1 任务
ros2 service call /task/cancel demo_interfaces/srv/CancelTask "{}"       # 取消任务
ros2 service call /machine/reset demo_interfaces/srv/ResetMachine "{}"   # 复位机床
ros2 topic echo /task/state                                              # 查看任务状态
```

### 自动化测试（正常流程跑通校验）

```bash
ros2 run machining_demo demo_test
```

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
| `/task/state` | 话题 TaskState | task_manager → 全局 | 任务状态机状态（9 种状态 + 说明文字） |
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
IDLE → WAIT_STATION(工位忙碌等待) → PICK(取料) → PLACE(上料) → WAIT_MACHINING(等加工)
     → RETRIEVE(取回) → RETURN(放回) → DONE → IDLE
     任意状态 --取消/异常--> ABORTED
     WAIT_MACHINING --超时--> TIMEOUT(警告, 重试一次) --> ABORTED
```

## 三、自行完成的核心逻辑

1. **任务流程状态机**（task_manager.py）——任务状态切换、工位忙碌等待、加工超时提示与重试、取消/异常处理；
2. **工位信号模拟**（machine_simulator.py）——机床状态机（忙碌→就绪→加工→完成）与进度发布；
3. **上下料动作流程**（arm_controller.py）——取料/上料/取回/放回的完整动作编排，工件附着与分离；
4. **运行记录**（result_recorder.py）——全流程事件 CSV 记录 + 摘要文件，保证"演示可复现、结果可保存"。

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
        ├── machining_demo/    # 5 个功能节点 + 共享布局 layout.py + 测试脚本
        ├── launch/demo.launch.py
        └── rviz/machining_demo.rviz
```
