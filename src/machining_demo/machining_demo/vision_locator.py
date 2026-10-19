"""像素识别服务：不订阅场景物体位姿，不读取槽位坐标。"""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from demo_interfaces.srv import LocateWorkpieces
from machining_demo.vision_geometry import FRAME
from machining_demo.vision_image import detect


class VisionLocator(Node):
    def __init__(self):
        super().__init__('vision_locator')
        self.bridge = CvBridge()
        self.latest = None
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 1)
        self.pub = self.create_publisher(Image, '/vision/annotated', 1)
        self.create_service(LocateWorkpieces, '/vision/locate_workpieces', self.locate)

    def image_cb(self, msg):
        # 保留原始时间戳，不能在缓存被重新读取时刷新时间。
        self.latest = (msg, time.monotonic())
        try:
            _, annotated = detect(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
            out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out.header = msg.header
            self.pub.publish(out)
        except Exception:
            # 无完整检测时发布本帧原图，不把旧检测框留在当前画面。
            self.pub.publish(msg)

    def locate(self, request, response):
        try:
            if self.latest is None or time.monotonic() - self.latest[1] > 1.0:
                raise ValueError('相机图像缺失或过期，不使用预设坐标代替')
            msg = self.latest[0]
            age = (self.get_clock().now().nanoseconds / 1e9 -
                   msg.header.stamp.sec - msg.header.stamp.nanosec / 1e9)
            if msg.header.frame_id != FRAME or not 0 <= age <= 1.0:
                raise ValueError('相机坐标系或采集时间无效')
            centers, annotated = detect(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
            for x, y, z in centers:
                pose = PoseStamped()
                pose.header.frame_id = 'panda_link0'
                pose.header.stamp = msg.header.stamp
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
                pose.pose.orientation.w = 1.0
                response.centers.append(pose)
            out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out.header = msg.header
            self.pub.publish(out)
            response.success, response.message = True, '三件物料已从图像定位'
        except Exception as exc:
            response.success, response.message = False, str(exc)
            response.centers = []
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VisionLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
