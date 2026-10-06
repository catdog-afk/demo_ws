"""角色B·动作与执行：机械臂控制节点。

通过 move_group 的标准 ROS 接口实现运动控制（不使用 moveit_commander）：
    - /move_action 动作（MoveGroup action）：运动规划 + 轨迹执行
    - /compute_cartesian_path + /execute_trajectory：抓取附近的直线进退
    - /check_state_validity：失败后的当前状态碰撞诊断
    - /apply_planning_scene 服务：场景更新（工件附着 / 分离）
    - /joint_states 话题：获取当前关节状态作为规划起点

提供服务 /arm/command（demo_interfaces/srv/ArmCommand），command 取值：
    home     回零位姿态
    open     夹爪张开
    close    夹爪闭合
    pick     从料盘槽位取料（工件附着到末端）
    place    将工件放到加工台（分离工件）
    retrieve 从加工台取回工件（附着）
    return   将工件放回料盘槽位（分离）

发布话题：
    /arm/status                    执行状态反馈（String）
    /machine/workpiece_present     加工台工件在位信号（Bool，机床节点订阅）

执行链：move_group -> MoveItSimpleControllerManager -> ros2_control
        （mock_components 硬件模拟，joint_state_broadcaster 发布关节状态）
"""
import time
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (MotionPlanRequest, PlanningOptions, Constraints,
                             PositionConstraint, OrientationConstraint,
                             JointConstraint, BoundingVolume, RobotState,
                             PlanningScene, AttachedCollisionObject,
                             CollisionObject, MoveItErrorCodes)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetStateValidity

from demo_interfaces.srv import ArmCommand

from machining_demo import layout
from machining_demo.scene_colors import object_color

# 规划组关节
ARM_JOINTS = ['panda_joint%d' % i for i in range(1, 8)]
FINGER_JOINTS = ['panda_finger_joint1', 'panda_finger_joint2']

# 命名姿态（panda.srdf 中的 group_state）
HOME_JOINTS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
HAND_OPEN = [0.035, 0.035]
HAND_CLOSE = [0.0, 0.0]

# 仅用于抓取附近的短直线段：拒绝任何一步超过约 11.5 度的关节变化。
# 在服务返回的时间参数化轨迹上检查，也包含当前状态到轨迹首点。
MAX_LINEAR_JOINT_STEP = 0.20

# MoveGroup 动作返回码
MOVEIT_ERROR_SUCCESS = 1


def make_pose(x, y, z, quat=layout.GRASP_QUAT):
    """构造世界坐标系（panda_link0）下的位姿消息。"""
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
    return p


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('velocity_scale', 0.5)

        # 接口
        # 重要：动作/服务客户端必须使用独立回调组。
        # 默认回调组是互斥的，/arm/command 服务回调阻塞等待 future 时，
        # 同组客户端的回调会被禁止执行，导致 future 永远无法完成（死锁）。
        self._clients_group = MutuallyExclusiveCallbackGroup()
        self._action_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self._clients_group)
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
            callback_group=self._clients_group)
        self._cartesian_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path',
            callback_group=self._clients_group)
        self._validity_client = self.create_client(
            GetStateValidity, '/check_state_validity',
            callback_group=self._clients_group)
        self._scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene',
            callback_group=self._clients_group)
        self.pub_status = self.create_publisher(String, '/arm/status', 10)
        self.pub_workpiece = self.create_publisher(
            Bool, '/machine/workpiece_present', 10)
        self.srv = self.create_service(ArmCommand, '/arm/command', self._cmd_cb)

        # 最新关节状态（/joint_states 订阅，多线程保护）
        # 同样使用独立回调组：服务回调阻塞期间仍需持续更新关节状态
        self._joint_positions = {}
        self._joint_received = {}
        self._joint_lock = threading.Lock()
        self._joint_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10,
                                 callback_group=self._joint_group)

        # 当前附着工件 / 加工台上的工件（用于 place/retrieve 跟踪）
        self._attached_name = None
        self._machine_workpiece = None
        self._last_error = ''

        self.get_logger().info('机械臂控制节点就绪，服务 /arm/command 已创建')

    # ---------- 关节状态 ----------
    def _js_cb(self, msg):
        received = time.monotonic()
        with self._joint_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos
                self._joint_received[name] = received

    def _wait_ready(self):
        """等待 move_group 动作服务与关节状态就绪。"""
        deadline = time.monotonic() + 60.0
        while rclpy.ok() and time.monotonic() < deadline:
            with self._joint_lock:
                arm_ready = all(j in self._joint_positions
                                for j in ARM_JOINTS + FINGER_JOINTS)
            if (self._action_client.server_is_ready() and
                    self._execute_client.server_is_ready() and
                    self._cartesian_client.service_is_ready() and
                    self._scene_client.service_is_ready() and arm_ready):
                return True
            time.sleep(0.2)
        self.get_logger().error('等待 move_group / joint_states 就绪超时')
        return False

    def _current_state(self):
        """以当前关节状态构造规划起点（is_diff=True，与当前场景合并）。"""
        rs = RobotState()
        rs.is_diff = True
        js = JointState()
        with self._joint_lock:
            js.name = [j for j in ARM_JOINTS + FINGER_JOINTS
                       if j in self._joint_positions]
            js.position = [self._joint_positions[j] for j in js.name]
        rs.joint_state = js
        return rs

    def _motion_start_state(self, name):
        """等待本次请求之后的完整反馈，避免沿用上一段运动结束前的状态。"""
        since = time.monotonic()
        deadline = since + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            with self._joint_lock:
                fresh = all(self._joint_received.get(j, 0.0) >= since
                            for j in ARM_JOINTS + FINGER_JOINTS)
            if fresh:
                return self._current_state()
            time.sleep(0.02)
        self._failure('%s：未收到新的完整关节反馈，不发送运动目标' % name)
        return None

    # ---------- 服务回调 ----------
    def _cmd_cb(self, request, response):
        command = request.command
        slot = request.slot
        self._last_error = ''
        self.get_logger().info('收到动作指令: %s (slot=%d)' % (command, slot))
        if not self._wait_ready():
            response.success = False
            response.message = 'move_group 未就绪'
            return response
        try:
            if command == 'home':
                ok = self._home()
            elif command == 'open':
                ok = self._gripper(open_=True)
            elif command == 'close':
                ok = self._gripper(open_=False)
            elif command == 'pick':
                ok = self._pick(slot)
            elif command == 'place':
                ok = self._place()
            elif command == 'retrieve':
                ok = self._retrieve()
            elif command == 'return':
                ok = self._return(slot)
            else:
                response.success = False
                response.message = '未知指令: ' + command
                return response
            response.success = ok
            response.message = 'ok' if ok else (self._last_error or '动作执行失败')
        except Exception as e:  # 规划失败等异常统一处理
            self._failure('指令 %s 执行异常: %s' % (command, e))
            response.success = False
            response.message = '异常: %s' % e
        return response

    # ---------- 运动规划与执行（move_group 动作） ----------
    def _pose_goal(self, pose):
        """末端位姿目标约束（link: panda_hand，坐标系 panda_link0）。"""
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = 'panda_link0'
        pc.link_name = layout.EE_LINK
        pc.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]  # 位置容差 1mm
        bv = BoundingVolume()
        bv.primitives = [sphere]
        # 关键：约束球体必须通过 primitive_poses 摆放到目标点，
        # target_point_offset 保持 0。MoveIt 的语义是"连杆上的点
        # (link_pose * target_point_offset) 必须落在约束区域内"——
        # 若把目标坐标填进 offset 而球体留在原点，机械臂会走到
        # 目标点的镜像位置（实测抓取点 x 反号，偏差约 900mm）
        region_pose = Pose()
        region_pose.position.x = pose.position.x
        region_pose.position.y = pose.position.y
        region_pose.position.z = pose.position.z
        region_pose.orientation.w = 1.0
        bv.primitive_poses = [region_pose]
        pc.constraint_region = bv
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        c.position_constraints = [pc]

        oc = OrientationConstraint()
        oc.header.frame_id = 'panda_link0'
        oc.link_name = layout.EE_LINK
        oc.weight = 1.0
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.02
        oc.absolute_y_axis_tolerance = 0.02
        oc.absolute_z_axis_tolerance = 0.02
        c.orientation_constraints = [oc]
        return c

    def _joint_goal(self, joints, values):
        """关节角目标约束。"""
        c = Constraints()
        for name, val in zip(joints, values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            # 手指为直线关节，单位是米；旧值 0.01 允许少张开 10mm。
            # 料宽仅 40mm，必须保证实际张开程度，降低抓取/分离时接触风险。
            tolerance = 0.0005 if name in FINGER_JOINTS else 0.01
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    def _plan_execute(self, group, goal_constraints, name):
        """向 move_group 发送规划+执行请求（阻塞至完成）。"""
        if not rclpy.ok():
            self._failure('%s：节点正在退出，不再发送运动目标' % name)
            return False
        start_state = self._motion_start_state(name)
        if start_state is None:
            return False
        self._status('规划中: %s' % name)
        req = MotionPlanRequest()
        req.workspace_parameters.header.frame_id = 'panda_link0'
        req.workspace_parameters.min_corner.x = -0.8
        req.workspace_parameters.min_corner.y = -0.8
        req.workspace_parameters.min_corner.z = -0.3
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 0.8
        req.workspace_parameters.max_corner.z = 1.2
        req.start_state = start_state
        req.goal_constraints = [goal_constraints]
        req.group_name = group
        req.num_planning_attempts = 3
        req.allowed_planning_time = self.get_parameter('planning_time').value
        req.max_velocity_scaling_factor = self.get_parameter(
            'velocity_scale').value
        req.max_acceleration_scaling_factor = 0.3
        req.pipeline_id = 'ompl'

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions(replan=True)

        return self._execute_action(self._action_client, goal, name)

    def _execute_action(self, client, goal, name):
        """统一检查 MoveGroup / ExecuteTrajectory 的接受、超时和执行结果。"""
        if not rclpy.ok():
            self._failure('%s：节点正在退出，不再发送运动目标' % name)
            return False
        future = client.send_goal_async(goal)
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done() or not future.result().accepted:
            self._failure('%s：运动目标未接受或响应超时' % name)
            return False
        goal_handle = future.result()

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + 90.0
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not result_future.done():
            self._failure('%s：动作超时或节点正在退出' % name)
            if rclpy.ok():
                goal_handle.cancel_goal_async()
            return False
        result = result_future.result().result
        if result.error_code.val == MOVEIT_ERROR_SUCCESS:
            self._status('执行完成: %s' % name)
            return True
        code = result.error_code.val
        self._diagnose_state()
        self._failure('%s：规划/执行失败 %s' % (name, self._error_code(code)))
        return False

    @staticmethod
    def _error_code(code):
        code_name = next((key for key in dir(MoveItErrorCodes)
                          if key.isupper() and getattr(MoveItErrorCodes, key) == code), 'UNKNOWN')
        return '%s (%d)' % (code_name, code)

    def _diagnose_state(self):
        """失败时记录实际关节和当前碰撞对；诊断服务不可用不影响原失败原因。"""
        with self._joint_lock:
            joints = ', '.join('%s=%.5f' % (j, self._joint_positions[j])
                               for j in ARM_JOINTS + FINGER_JOINTS
                               if j in self._joint_positions)
        self._status('失败现场：%s；附着=%s；加工台工件=%s' %
                     (joints, self._attached_name, self._machine_workpiece))
        if not rclpy.ok() or not self._validity_client.service_is_ready():
            return
        try:
            request = GetStateValidity.Request()
            request.robot_state = self._current_state()
            request.group_name = layout.ARM_GROUP
            future = self._validity_client.call_async(request)
            deadline = time.monotonic() + 1.0
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if not future.done():
                future.cancel()
                self._status('失败现场：状态碰撞查询超时')
                return
            result = future.result()
            contacts = ', '.join('%s/%s' % (c.contact_body_1, c.contact_body_2)
                                 for c in result.contacts[:10]) or '无'
            self._status('失败现场：当前状态有效=%s；碰撞对=%s' % (result.valid, contacts))
        except Exception as exc:
            self._status('失败现场：状态碰撞查询异常 %s' % exc)

    def _goto_pose(self, pose, name):
        return self._plan_execute(layout.ARM_GROUP, self._pose_goal(pose), name)

    def _goto_linear(self, pose, name):
        """抓取附近沿直线进退，保留当前 IK 分支；完整且避碰的轨迹才执行。"""
        start_state = self._motion_start_state(name)
        if start_state is None:
            return False
        self._status('直线规划中: %s' % name)
        request = GetCartesianPath.Request()
        request.header.frame_id = 'panda_link0'
        request.start_state = start_state
        request.group_name = layout.ARM_GROUP
        request.link_name = layout.EE_LINK
        request.waypoints = [pose]
        request.max_step = 0.002
        # 约 2cm 的短路径不采用相对平均步长阈值；首次纠正姿态等正常变化
        # 也可能被相对阈值截断。完整路径另用绝对关节变化上限检查。
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = self.get_parameter('velocity_scale').value
        request.max_acceleration_scaling_factor = 0.3
        future = self._cartesian_client.call_async(request)
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            self._failure('%s：直线路径查询超时或节点正在退出' % name)
            return False
        result = future.result()
        fraction = result.fraction
        if (result.error_code.val != MOVEIT_ERROR_SUCCESS or
                not math.isfinite(fraction) or abs(fraction - 1.0) > 1e-6):
            partial = result.solution.joint_trajectory
            if partial.points:
                joints = ', '.join('%s=%.5f' % (j, value) for j, value in
                                   zip(partial.joint_names, partial.points[-1].positions))
                self._status('直线路径末端（未执行）：%s' % joints)
            self._diagnose_state()
            self._failure('%s：直线路径不完整 (%.2f%%)，%s，不执行部分轨迹' %
                          (name, fraction * 100.0, self._error_code(result.error_code.val)))
            return False
        trajectory = result.solution
        if not trajectory.joint_trajectory.points:
            self._failure('%s：直线规划返回空轨迹' % name)
            return False
        if not self._check_linear_jumps(start_state, trajectory, name):
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        return self._execute_action(self._execute_client, goal, name)

    def _check_linear_jumps(self, start_state, trajectory, name):
        """绝对步长检查：不允许靠切换 IK 分支完成短直线运动。"""
        jt = trajectory.joint_trajectory
        names = list(jt.joint_names)
        start = dict(zip(start_state.joint_state.name, start_state.joint_state.position))
        if (len(names) != len(ARM_JOINTS) or set(names) != set(ARM_JOINTS) or
                any(j not in start for j in names)):
            self._failure('%s：直线轨迹关节不完整，不执行' % name)
            return False
        previous = [start[j] for j in names]
        if not all(math.isfinite(value) for value in previous):
            self._failure('%s：起始关节反馈包含无效数值，不执行' % name)
            return False
        for index, point in enumerate(jt.points):
            values = list(point.positions)
            if len(values) != len(names) or not all(math.isfinite(value) for value in values):
                self._failure('%s：直线轨迹第 %d 点包含无效关节值，不执行' % (name, index))
                return False
            jumps = [abs(value - old) for value, old in zip(values, previous)]
            largest = max(jumps)
            if largest > MAX_LINEAR_JOINT_STEP:
                joint = names[jumps.index(largest)]
                self._failure('%s：直线轨迹 %s 跳变 %.4f rad，超过 %.2f rad，不执行' %
                              (name, joint, largest, MAX_LINEAR_JOINT_STEP))
                return False
            previous = values
        return True

    # ---------- 场景更新（工件附着/分离） ----------
    def _apply_scene(self, scene):
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._scene_client.call_async(req)
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done() or not future.result().success:
            self._failure('工件附着/分离：场景更新失败或响应超时')
            return False
        return True

    def _attach_workpiece(self, name):
        """将工件碰撞物体附着到末端执行器（panda_hand）。"""
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        aco = AttachedCollisionObject()
        aco.link_name = layout.EE_LINK
        aco.touch_links = layout.TOUCH_LINKS
        co = CollisionObject()
        co.id = name
        # 附着物体的位姿定义在夹爪（panda_hand）坐标系下：
        # 若写成 panda_link0，MoveIt 会把箱子放到基座原点附近，导致自碰撞
        co.header.frame_id = layout.EE_LINK
        co.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [layout.WORKPIECE_SIZE] * 3
        co.primitives = [box]
        attach_pose = Pose()
        attach_pose.position.z = float(layout.ATTACH_Z_OFFSET)
        attach_pose.orientation.w = 1.0
        co.primitive_poses = [attach_pose]
        co.pose.orientation.w = 1.0
        aco.object = co
        scene.robot_state.attached_collision_objects = [aco]
        scene.object_colors = [object_color(name)]
        return self._apply_scene(scene)

    def _detach_workpiece(self, name, world_xyz):
        """分离工件并将其移动到世界坐标系下的新位置。"""
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        # 从末端分离
        aco = AttachedCollisionObject()
        aco.link_name = layout.EE_LINK
        remove_obj = CollisionObject()
        remove_obj.id = name
        remove_obj.operation = CollisionObject.REMOVE
        aco.object = remove_obj
        scene.robot_state.attached_collision_objects = [aco]
        # 在世界中重新添加
        co = CollisionObject()
        co.id = name
        co.header.frame_id = 'panda_link0'
        co.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [layout.WORKPIECE_SIZE] * 3
        co.primitives = [box]
        pose = Pose()
        pose.position.x = world_xyz[0]
        pose.position.y = world_xyz[1]
        pose.position.z = world_xyz[2]
        pose.orientation.w = 1.0
        co.primitive_poses = [pose]
        co.pose.orientation.w = 1.0
        scene.world.collision_objects = [co]
        scene.object_colors = [object_color(name)]
        return self._apply_scene(scene)

    # ---------- 基础动作 ----------
    def _status(self, text):
        self.get_logger().info(text)
        if rclpy.ok():
            self.pub_status.publish(String(data=text))

    def _failure(self, text):
        """同时保存服务响应与 CSV 反馈，避免丢失失败动作和 MoveIt 返回码。"""
        self._last_error = text
        self.get_logger().error(text)
        if rclpy.ok():
            self.pub_status.publish(String(data=text))

    def _home(self):
        return self._plan_execute(layout.ARM_GROUP,
                                  self._joint_goal(ARM_JOINTS, HOME_JOINTS),
                                  '回零位(ready)')

    def _gripper(self, open_):
        target = HAND_OPEN if open_ else HAND_CLOSE
        name = '夹爪张开' if open_ else '夹爪闭合'
        self._status(name)
        return self._plan_execute(layout.HAND_GROUP,
                                  self._joint_goal(FINGER_JOINTS, target),
                                  name)

    # ---------- 上下料动作 ----------
    def _pick(self, slot):
        if slot >= layout.NUM_SLOTS:
            self.get_logger().error('槽位 %d 超出范围' % slot)
            return False
        if self._attached_name is not None or self._machine_workpiece is not None:
            self._failure('已有工件在夹爪或加工台，不能重复取料')
            return False
        x, y, z = layout.slot_xyz(slot)
        name = layout.workpiece_name(slot)
        approach = make_pose(x, y, z + layout.APPROACH_HEIGHT)
        grasp = make_pose(x, y, z + layout.GRASP_Z_OFFSET)

        self._status('取料：槽位 %d -> 预抓取点' % slot)
        # 上一轮结束时夹爪可能仍闭合，先张开再靠近工件。
        if not self._gripper(open_=True):
            return False
        if not self._goto_pose(approach, '预抓取点(槽位%d)' % slot):
            return False
        if not self._goto_linear(grasp, '抓取点(槽位%d)' % slot):
            return False
        # 先附着再闭合：附着后 touch_links 自动允许手指与工件接触，
        # 闭合时手指扫过工件才不会被碰撞检测拒绝
        if not self._attach_workpiece(name):
            return False
        self._attached_name = name
        self._status('工件 %s 已附着到末端' % name)
        if not self._gripper(open_=False):
            return False
        if not self._goto_linear(approach, '提起工件'):
            return False
        self.pub_workpiece.publish(Bool(data=False))
        return True

    def _place(self):
        if self._attached_name is None:
            self.get_logger().error('末端没有附着工件，无法执行 place')
            return False
        name = self._attached_name
        x, y, z = layout.machine_xyz()
        approach = make_pose(x, y, z + layout.APPROACH_HEIGHT)
        place = make_pose(x, y, z + layout.GRASP_Z_OFFSET)

        if not self._goto_pose(approach, '上料预放置点'):
            return False
        if not self._goto_linear(place, '放置点'):
            return False
        if not self._gripper(open_=True):
            return False
        # 分离工件并移动到加工台位置
        if not self._detach_workpiece(name, layout.machine_xyz()):
            return False
        self._attached_name = None
        self._machine_workpiece = name
        self._status('工件已放入加工台，开始等待加工')
        if not self._goto_linear(approach, '退出加工台'):
            return False
        self.pub_workpiece.publish(Bool(data=True))
        return True

    def _retrieve(self):
        if self._machine_workpiece is None:
            self.get_logger().error('加工台上没有工件，无法执行 retrieve')
            return False
        name = self._machine_workpiece
        x, y, z = layout.machine_xyz()
        approach = make_pose(x, y, z + layout.APPROACH_HEIGHT)
        grasp = make_pose(x, y, z + layout.GRASP_Z_OFFSET)

        if not self._goto_pose(approach, '取回预抓取点'):
            return False
        if not self._goto_linear(grasp, '取回抓取点'):
            return False
        # MoveIt 附着操作同时移除世界物体，避免两次更新之间丢失工件。
        if not self._attach_workpiece(name):
            return False
        self._attached_name = name
        self._machine_workpiece = None
        self._status('工件已从加工台取回并附着')
        if not self._gripper(open_=False):
            return False
        if not self._goto_linear(approach, '取回提起'):
            return False
        self.pub_workpiece.publish(Bool(data=False))
        return True

    def _return(self, slot):
        if slot >= layout.NUM_SLOTS:
            self.get_logger().error('槽位 %d 超出范围' % slot)
            return False
        if self._attached_name is None:
            self.get_logger().error('末端没有附着工件，无法执行 return')
            return False
        name = self._attached_name
        x, y, z = layout.slot_xyz(slot)
        approach = make_pose(x, y, z + layout.APPROACH_HEIGHT)
        grasp = make_pose(x, y, z + layout.GRASP_Z_OFFSET)

        if not self._goto_pose(approach, '放回预放置点(槽位%d)' % slot):
            return False
        if not self._goto_linear(grasp, '放回点(槽位%d)' % slot):
            return False
        if not self._gripper(open_=True):
            return False
        if not self._detach_workpiece(name, layout.slot_xyz(slot)):
            return False
        self._attached_name = None
        self._status('工件已放回料盘槽位 %d' % slot)
        if not self._goto_linear(approach, '退出料盘'):
            return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    # 多线程执行器：服务回调中阻塞等待动作结果时，其他回调仍可被处理
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
