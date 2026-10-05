"""角色B·动作与执行：机械臂控制节点。

通过 move_group 的标准 ROS 接口实现运动控制（不使用 moveit_commander）：
    - /move_action 动作（MoveGroup action）：运动规划 + 轨迹执行
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
                             CollisionObject)
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import ApplyPlanningScene

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
        self._joint_lock = threading.Lock()
        self._joint_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10,
                                 callback_group=self._joint_group)

        # 当前附着工件 / 加工台上的工件（用于 place/retrieve 跟踪）
        self._attached_name = None
        self._machine_workpiece = None

        self.get_logger().info('机械臂控制节点就绪，服务 /arm/command 已创建')

    # ---------- 关节状态 ----------
    def _js_cb(self, msg):
        with self._joint_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos

    def _wait_ready(self):
        """等待 move_group 动作服务与关节状态就绪。"""
        deadline = time.monotonic() + 60.0
        while rclpy.ok() and time.monotonic() < deadline:
            with self._joint_lock:
                arm_ready = all(j in self._joint_positions
                                for j in ARM_JOINTS + FINGER_JOINTS)
            if (self._action_client.server_is_ready() and
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

    # ---------- 服务回调 ----------
    def _cmd_cb(self, request, response):
        command = request.command
        slot = request.slot
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
            response.message = 'ok' if ok else '动作执行失败'
        except Exception as e:  # 规划失败等异常统一处理
            self.get_logger().error('指令 %s 执行异常: %s' % (command, e))
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
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    def _plan_execute(self, group, goal_constraints, name):
        """向 move_group 发送规划+执行请求（阻塞至完成）。"""
        self._status('规划中: %s' % name)
        req = MotionPlanRequest()
        req.workspace_parameters.header.frame_id = 'panda_link0'
        req.workspace_parameters.min_corner.x = -0.8
        req.workspace_parameters.min_corner.y = -0.8
        req.workspace_parameters.min_corner.z = -0.3
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 0.8
        req.workspace_parameters.max_corner.z = 1.2
        req.start_state = self._current_state()
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

        future = self._action_client.send_goal_async(goal)
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done() or not future.result().accepted:
            self.get_logger().error('move_group 未接受目标: %s' % name)
            return False
        goal_handle = future.result()

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + 90.0
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not result_future.done():
            self.get_logger().error('动作超时: %s' % name)
            goal_handle.cancel_goal_async()
            return False
        result = result_future.result().result
        if result.error_code.val == MOVEIT_ERROR_SUCCESS:
            self._status('执行完成: %s' % name)
            return True
        self.get_logger().error('规划/执行失败(%s): %s'
                                % (name, result.error_code.val))
        return False

    def _goto_pose(self, pose, name):
        return self._plan_execute(layout.ARM_GROUP, self._pose_goal(pose), name)

    # ---------- 场景更新（工件附着/分离） ----------
    def _apply_scene(self, scene):
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._scene_client.call_async(req)
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done() or not future.result().success:
            self.get_logger().error('场景更新失败')
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
            self.get_logger().error('已有工件在夹爪或加工台，不能重复取料')
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
        if not self._goto_pose(grasp, '抓取点(槽位%d)' % slot):
            return False
        # 先附着再闭合：附着后 touch_links 自动允许手指与工件接触，
        # 闭合时手指扫过工件才不会被碰撞检测拒绝
        if not self._attach_workpiece(name):
            return False
        self._attached_name = name
        self._status('工件 %s 已附着到末端' % name)
        if not self._gripper(open_=False):
            return False
        if not self._goto_pose(approach, '提起工件'):
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
        if not self._goto_pose(place, '放置点'):
            return False
        if not self._gripper(open_=True):
            return False
        # 分离工件并移动到加工台位置
        if not self._detach_workpiece(name, layout.machine_xyz()):
            return False
        self._attached_name = None
        self._machine_workpiece = name
        self._status('工件已放入加工台，开始等待加工')
        if not self._goto_pose(approach, '退出加工台'):
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
        if not self._goto_pose(grasp, '取回抓取点'):
            return False
        # MoveIt 附着操作同时移除世界物体，避免两次更新之间丢失工件。
        if not self._attach_workpiece(name):
            return False
        self._attached_name = name
        self._machine_workpiece = None
        self._status('工件已从加工台取回并附着')
        if not self._gripper(open_=False):
            return False
        if not self._goto_pose(approach, '取回提起'):
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
        if not self._goto_pose(grasp, '放回点(槽位%d)' % slot):
            return False
        if not self._gripper(open_=True):
            return False
        if not self._detach_workpiece(name, layout.slot_xyz(slot)):
            return False
        self._attached_name = None
        self._status('工件已放回料盘槽位 %d' % slot)
        if not self._goto_pose(approach, '退出料盘'):
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
