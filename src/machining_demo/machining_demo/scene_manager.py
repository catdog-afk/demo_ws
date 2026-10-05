"""角色A·仿真与平台：场景搭建节点。

功能：
    1. 通过 move_group 的 /apply_planning_scene 服务，向 MoveIt 规划场景添加
       桌面、料盘、加工台与工件碰撞物体；
    2. 发布 table / tray / machine 静态 TF 坐标变换；
    3. 维护并发布料盘槽位占用状态（/tray/occupancy），提供设置服务。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose

from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene

from demo_interfaces.msg import TrayOccupancy
from demo_interfaces.srv import SetOccupancy

from machining_demo import layout


class SceneManager(Node):
    def __init__(self):
        super().__init__('scene_manager')
        # 规划场景服务客户端（move_group 提供）
        self.scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        if not self.scene_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error('等待 /apply_planning_scene 服务超时')
            raise RuntimeError('/apply_planning_scene 不可用')

        # 静态 TF 广播器
        self._tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_tfs()

        # 料盘占用状态
        self.occupied = [True] * layout.NUM_SLOTS
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_occupancy = self.create_publisher(TrayOccupancy, '/tray/occupancy', latched)

        # 初始化期间主动处理服务响应；场景成功应用后才暴露占用服务。
        self._build_scene()
        self.srv_occupancy = self.create_service(
            SetOccupancy, '/tray/set_occupancy', self._set_occupancy_cb)
        self.create_timer(1.0, self._publish_occupancy)

        self._publish_occupancy()

    # ---------- 静态 TF ----------
    def _publish_static_tfs(self):
        tfs = []
        for child, xyz in (('table', layout.TABLE['pos']),
                           ('tray', layout.TRAY['pos']),
                           ('machine', layout.MACHINE['pos'])):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'panda_link0'
            t.child_frame_id = child
            t.transform.translation.x = xyz[0]
            t.transform.translation.y = xyz[1]
            t.transform.translation.z = xyz[2]
            t.transform.rotation.w = 1.0
            tfs.append(t)
        self._tf_broadcaster.sendTransform(tfs)
        self.get_logger().info('已发布静态 TF：table / tray / machine')

    # ---------- 规划场景 ----------
    def _box(self, name, size, xyz):
        """构造一个盒状碰撞物体（规划场景坐标：panda_link0）。"""
        co = CollisionObject()
        co.id = name
        co.header.frame_id = 'panda_link0'
        co.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(s) for s in size]
        co.primitives = [box]
        pose = Pose()
        pose.position.x = xyz[0]
        pose.position.y = xyz[1]
        pose.position.z = xyz[2]
        pose.orientation.w = 1.0
        co.primitive_poses = [pose]
        # primitive_poses 相对于 co.pose；对象坐标系必须保持单位变换。
        co.pose.orientation.w = 1.0
        return co

    def _apply(self, scene):
        """调用 /apply_planning_scene 服务更新规划场景。"""
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            future.cancel()
            self.get_logger().error('apply_planning_scene 调用失败')
            return False
        return future.result().success

    def _build_scene(self):
        self.get_logger().info('正在构建规划场景（桌面 / 料盘 / 加工台 / 工件）...')
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [
            self._box(layout.TABLE['name'], layout.TABLE['size'],
                      layout.TABLE['pos']),
            self._box(layout.TRAY['name'], layout.TRAY['size'],
                      layout.TRAY['pos']),
            self._box(layout.MACHINE['name'], layout.MACHINE['size'],
                      layout.MACHINE['pos']),
        ]
        for slot in range(layout.NUM_SLOTS):
            scene.world.collision_objects.append(
                self._box(layout.workpiece_name(slot),
                          (layout.WORKPIECE_SIZE,) * 3,
                          layout.slot_xyz(slot)))
        if self._apply(scene):
            self.get_logger().info('场景构建完成：桌面 + 料盘(3 个工件) + 加工台')
        else:
            raise RuntimeError('场景构建失败，禁止启动上下料任务')

    # ---------- 料盘占用 ----------
    def _set_occupancy_cb(self, request, response):
        if request.slot >= layout.NUM_SLOTS:
            response.success = False
            response.message = '槽位编号超出范围（0~%d）' % (layout.NUM_SLOTS - 1)
            return response
        self.occupied[request.slot] = request.occupied
        self._publish_occupancy()
        self.get_logger().info('槽位 %d 更新为 %s'
                               % (request.slot,
                                  '占用' if request.occupied else '空闲'))
        response.success = True
        response.message = 'ok'
        return response

    def _publish_occupancy(self):
        msg = TrayOccupancy()
        msg.num_slots = layout.NUM_SLOTS
        msg.occupied = list(self.occupied)
        self.pub_occupancy.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SceneManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
