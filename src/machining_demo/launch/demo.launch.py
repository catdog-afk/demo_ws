"""A2 模拟机床上下料：一键启动全部节点。

启动内容（与 moveit_resources_panda 官方 demo 一致的底层架构）：
    1. move_group（MoveIt 运动规划，配置来自 moveit_resources_panda_moveit_config）
    2. ros2_control + mock_components（硬件模拟）+ 控制器加载
       （joint_state_broadcaster / panda_arm_controller / panda_hand_controller）
    3. robot_state_publisher（机器人 TF）+ world 静态 TF
    4. rviz2（机器人模型 / 规划场景 / 轨迹 / 状态标注）
    5. 五个功能节点：scene_manager / arm_controller / machine_simulator /
       task_manager / result_recorder

常用参数：
    auto_start:=true         是否自动开始演示（默认 true，5 秒后自动启动）
    slot:=0                  取料槽位 0~2
    initial_busy_sec:=10.0   机床初始忙碌时长（演示"忙碌等待"拓展）
    machining_sec:=5.0       加工时长
    machining_timeout:=8.0   加工超时阈值（演示"超时提示"拓展）
    use_rviz:=true           是否启动 RViz
    results_dir:=...         运行记录输出目录（默认 ~/demo_ws/results）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    def parameter(name, value_type):
        return ParameterValue(LaunchConfiguration(name), value_type=value_type)

    # ---- MoveIt 配置（复用官方 panda 配置包）----
    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .robot_description(
            file_path='config/panda.urdf.xacro',
            mappings={'ros2_control_hardware_type': 'mock_components'})
        .robot_description_semantic(file_path='config/panda.srdf')
        .trajectory_execution(
            file_path='config/gripper_moveit_controllers.yaml')
        .planning_pipelines(
            pipelines=['ompl', 'chomp', 'pilz_industrial_motion_planner'])
        .to_moveit_configs()
    )

    # ---- move_group（运动规划服务 + /move_action 动作 + /apply_planning_scene）----
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    # ---- ros2_control（mock_components 硬件模拟）----
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[moveit_config.robot_description, os.path.join(
            get_package_share_directory('moveit_resources_panda_moveit_config'),
            'config', 'ros2_controllers.yaml')],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
        output='screen',
    )

    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '-c', '/controller_manager'],
            output='screen',
        )
        for name in ('joint_state_broadcaster', 'panda_arm_controller',
                     'panda_hand_controller')
    ]

    # ---- TF ----
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                   'world', 'panda_link0'],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    # ---- RViz ----
    # 必须传入 robot_description / robot_description_semantic 参数：
    # PlanningScene 与 Trajectory 插件需要 SRDF 才能加载机器人模型，
    # 否则 Status: Error 且场景物体（料盘/机床/工件）不会显示
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(
            get_package_share_directory('machining_demo'),
            'rviz', 'machining_demo.rviz')],
        parameters=[moveit_config.to_dict()],
        output='screen',
    )

    # ---- 功能节点 ----
    scene_manager = Node(
        package='machining_demo', executable='scene_manager', output='screen')
    machine_simulator = Node(
        package='machining_demo', executable='machine_simulator',
        output='screen',
        parameters=[{
            'initial_busy_sec': parameter('initial_busy_sec', float),
            'machining_sec': parameter('machining_sec', float),
        }])
    arm_controller = Node(
        package='machining_demo', executable='arm_controller', output='screen')
    task_manager = Node(
        package='machining_demo', executable='task_manager', output='screen',
        parameters=[{
            'auto_start': parameter('auto_start', bool),
            'auto_start_delay': parameter('auto_start_delay', float),
            'slot': parameter('slot', int),
            'machining_timeout': parameter('machining_timeout', float),
            'station_timeout': parameter('station_timeout', float),
            'arm_call_timeout': parameter('arm_call_timeout', float),
            'timeout_retry_once': parameter('timeout_retry_once', bool),
        }])
    result_recorder = Node(
        package='machining_demo', executable='result_recorder',
        output='screen',
        parameters=[{'output_dir': LaunchConfiguration('results_dir')}])

    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='true',
                              description='是否自动开始演示'),
        DeclareLaunchArgument('slot', default_value='0',
                              description='取料槽位 0~2'),
        DeclareLaunchArgument('auto_start_delay', default_value='5.0',
                              description='自动启动延迟（秒）'),
        DeclareLaunchArgument('initial_busy_sec', default_value='10.0',
                              description='机床初始忙碌时长（秒）'),
        DeclareLaunchArgument('machining_sec', default_value='5.0',
                              description='加工时长（秒）'),
        DeclareLaunchArgument('machining_timeout', default_value='8.0',
                              description='加工超时阈值（秒）'),
        DeclareLaunchArgument('station_timeout', default_value='30.0',
                              description='等待工位/场景/服务就绪的超时（秒）'),
        DeclareLaunchArgument('arm_call_timeout', default_value='660.0',
                              description='完整取放动作服务超时（秒）'),
        DeclareLaunchArgument('timeout_retry_once', default_value='true',
                              description='加工超时后允许延长一次等待窗口'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否启动 RViz'),
        DeclareLaunchArgument('results_dir',
                              default_value=os.path.expanduser(
                                  '~/demo_ws/results'),
                              description='运行记录输出目录'),
        move_group_node,
        ros2_control_node,
        *controller_spawners,
        static_tf_node,
        robot_state_publisher,
        rviz_node,
        scene_manager,
        machine_simulator,
        arm_controller,
        task_manager,
        result_recorder,
    ])
