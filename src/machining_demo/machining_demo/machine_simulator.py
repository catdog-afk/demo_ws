"""角色C·感知与交互：模拟机床（加工台）工位信号节点。

状态机：
    BUSY（初始占用一段时间，模拟换型/维护）
      -> READY（空闲就绪，等待上料）
      -> MACHINING（检测到工件在位，模拟加工并发布进度）
      -> DONE（加工完成，等待取回）
      -> READY（工件被取走后，开始下一循环）

发布话题：
    /machine/status            工位状态（MachineStatus 消息，10Hz）
    /machine/status_marker     RViz 文本标注（机床上方显示当前状态）
订阅话题：
    /machine/workpiece_present 工件在位信号（Bool，机械臂节点发布）
服务：
    /machine/reset             复位为就绪状态（测试用）
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

from demo_interfaces.msg import MachineStatus
from demo_interfaces.srv import ResetMachine

from machining_demo import layout

# 状态常量（与 MachineStatus.msg 保持一致）
READY, BUSY, MACHINING, DONE = 0, 1, 2, 3


class MachineSimulator(Node):
    def __init__(self):
        super().__init__('machine_simulator')
        self.declare_parameter('initial_busy_sec', 3.0)
        self.declare_parameter('machining_sec', 5.0)
        self.declare_parameter('publish_rate', 10.0)

        self.state = BUSY
        self.progress = 0.0
        self.workpiece_present = False
        self._state_start = self.get_clock().now()
        self._last_status = ''

        self.pub_status = self.create_publisher(
            MachineStatus, '/machine/status', 10)
        self.pub_marker = self.create_publisher(
            Marker, '/machine/status_marker', 10)
        self.create_subscription(
            Bool, '/machine/workpiece_present', self._workpiece_cb, 10)
        self.create_service(ResetMachine, '/machine/reset', self._reset_cb)

        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._tick)
        self._state_start = self.get_clock().now()
        self._log('机床初始化完成，初始状态 BUSY（模拟工位占用 %.0f 秒）',
                  self.get_parameter('initial_busy_sec').value)

    def _log(self, text, *args):
        self.get_logger().info(text % args if args else text)
        self._last_status = text % args if args else text

    # ---------- 回调 ----------
    def _workpiece_cb(self, msg):
        if msg.data and not self.workpiece_present:
            self._log('检测到工件上料到位')
        elif not msg.data and self.workpiece_present:
            self._log('工件已取走')
        self.workpiece_present = msg.data

    def _reset_cb(self, request, response):
        self.state = READY
        self.progress = 0.0
        self.workpiece_present = False
        self._state_start = self.get_clock().now()
        self._log('机床已复位为 READY')
        response.success = True
        response.message = 'ok'
        return response

    # ---------- 状态机 ----------
    def _tick(self):
        now = self.get_clock().now()
        elapsed = (now - self._state_start).nanoseconds / 1e9

        if self.state == BUSY:
            if elapsed >= self.get_parameter('initial_busy_sec').value:
                self.state = READY
                self._state_start = now
                self._log('工位空闲就绪，等待上料')
        elif self.state == READY:
            if self.workpiece_present:
                self.state = MACHINING
                self.progress = 0.0
                self._state_start = now
                self._log('开始加工（预计 %.0f 秒）',
                          self.get_parameter('machining_sec').value)
        elif self.state == MACHINING:
            total = self.get_parameter('machining_sec').value
            self.progress = min(100.0, 100.0 * elapsed / total)
            if elapsed >= total:
                self.state = DONE
                self.progress = 100.0
                self._state_start = now
                self._log('加工完成，工件可取回')
        elif self.state == DONE:
            if not self.workpiece_present:
                self.state = READY
                self._state_start = now
                self._log('工件取走，工位重新就绪')

        self._publish_status()

    # ---------- 发布 ----------
    def _publish_status(self):
        msg = MachineStatus()
        msg.state = self.state
        msg.progress = self.progress
        msg.detail = layout.MACHINE_STATE_NAMES[self.state]
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_status.publish(msg)

        marker = Marker()
        marker.header.frame_id = 'panda_link0'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'machine_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        x, y = layout.MACHINE['pos'][:2]
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.45
        marker.scale.z = 0.07
        marker.color.a = 1.0
        if self.state == BUSY:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.7, 0.0
            text = '机床: 忙碌中...'
        elif self.state == READY:
            marker.color.r, marker.color.g, marker.color.b = 0.3, 1.0, 0.3
            text = '机床: 就绪'
        elif self.state == MACHINING:
            marker.color.r, marker.color.g, marker.color.b = 0.3, 0.7, 1.0
            text = '机床: 加工中 %.0f%%' % self.progress
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.3
            text = '机床: 加工完成'
        marker.text = text
        self.pub_marker.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = MachineSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
