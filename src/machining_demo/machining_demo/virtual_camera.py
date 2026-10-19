"""独立的简化虚拟相机，仅本节点读取 MoveIt 世界物体。无机器人遮挡/光照物理。"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from machining_demo import layout
from machining_demo.vision_geometry import WIDTH, HEIGHT, FOCAL, CX, CY, FRAME
from machining_demo.vision_image import render


class VirtualCamera(Node):
    def __init__(self):
        super().__init__('virtual_camera')
        self.client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 1)
        self.info = self.create_publisher(CameraInfo, '/camera/camera_info', 1)
        self.bridge = CvBridge()
        self.future = None
        self.create_timer(0.2, self.tick)

    def tick(self):
        if self.future is not None:
            if not self.future.done():
                return
            try:
                scene = self.future.result().scene
                objects = []
                for obj in scene.world.collision_objects:
                    if obj.id not in layout.OBJECT_COLORS or obj.header.frame_id not in ('panda_link0', 'world', ''):
                        continue
                    if len(obj.primitives) != 1 or len(obj.primitive_poses) != 1:
                        continue
                    pose, base = obj.primitive_poses[0], obj.pose
                    # 本仿真仅轴对齐箱体，拒绝旋转而非绘制错误的真值。
                    if any(abs(v) > 1e-6 for v in (base.orientation.x, base.orientation.y,
                                                  base.orientation.z, pose.orientation.x,
                                                  pose.orientation.y, pose.orientation.z)):
                        continue
                    xyz = tuple(getattr(base.position, a) + getattr(pose.position, a) for a in ('x', 'y', 'z'))
                    objects.append((obj.id, xyz, obj.primitives[0].dimensions))
                msg = self.bridge.cv2_to_imgmsg(render(objects), encoding='bgr8')
                msg.header.frame_id = FRAME
                msg.header.stamp = self.requested_at
                self.pub.publish(msg)
                info = CameraInfo()
                info.header = msg.header
                info.width, info.height = WIDTH, HEIGHT
                info.distortion_model = 'plumb_bob'
                info.d = [0.0] * 5
                info.k = [FOCAL, 0.0, CX, 0.0, FOCAL, CY, 0.0, 0.0, 1.0]
                info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                info.p = [FOCAL, 0.0, CX, 0.0, 0.0, FOCAL, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
                self.info.publish(info)
            except Exception as exc:
                self.get_logger().warning('虚拟相机未成像: %s' % exc)
            self.future = None
        if self.client.service_is_ready():
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            self.requested_at = self.get_clock().now().to_msg()
            self.future = self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
