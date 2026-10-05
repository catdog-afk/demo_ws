"""A2 异步任务流程：确认工位、取料、上料、加工、取回、放回并回零。"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker
from demo_interfaces.msg import TaskState, MachineStatus, TrayOccupancy
from demo_interfaces.srv import StartTask, CancelTask, ArmCommand, SetOccupancy
from machining_demo import layout

IDLE, PICK, PLACE, WAIT_MACHINING, RETRIEVE, RETURN = 0, 1, 2, 3, 4, 5
DONE, WAIT_STATION, TIMEOUT, ABORTED = 6, 7, 8, 9
STATE_COLOR = {
    IDLE: (0.7, 0.7, 0.7), PICK: (0.3, 0.7, 1.0), PLACE: (0.3, 0.7, 1.0),
    WAIT_MACHINING: (0.3, 1.0, 1.0), RETRIEVE: (0.5, 1.0, 0.3),
    RETURN: (0.5, 1.0, 0.3), DONE: (1.0, 1.0, 0.3),
    WAIT_STATION: (1.0, 0.7, 0.0), TIMEOUT: (1.0, 0.3, 0.0),
    ABORTED: (1.0, 0.3, 0.3),
}


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        defaults = {
            'auto_start': True, 'auto_start_delay': 5.0, 'slot': 0,
            'repeat_cycle': True, 'repeat_delay': 3.0,
            'machining_timeout': 8.0, 'timeout_retry_once': True,
            'arm_call_timeout': 660.0, 'station_timeout': 30.0,
            'signal_timeout': 2.0, 'occupancy_timeout': 5.0, 'loop_rate': 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in ('auto_start_delay', 'machining_timeout', 'arm_call_timeout',
                     'station_timeout', 'signal_timeout', 'occupancy_timeout',
                     'loop_rate', 'repeat_delay'):
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError('%s 必须是大于 0 的有限数值' % name)
        self.slot = self.get_parameter('slot').value
        if not 0 <= self.slot < layout.NUM_SLOTS:
            raise ValueError('slot 必须在 0~%d 范围内' % (layout.NUM_SLOTS - 1))
        self.state, self.detail = IDLE, ''
        self.machine_status = None
        self._machine_received = None
        self._tray_occupied = None
        self.cancel_requested = False
        self._arm_future = self._arm_deadline = None
        self._occ_future = self._occ_deadline = self._next_after_occ = None
        self._wait_start = self._station_start = self._warning_start = None
        self._timeout_retried = self._return_home = False
        self._next_cycle_at = None
        self._completed_cycles = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.create_publisher(TaskState, '/task/state', latched)
        self.pub_marker = self.create_publisher(Marker, '/task/status_marker', latched)
        self.create_subscription(MachineStatus, '/machine/status', self._machine_cb, 10)
        self.create_subscription(TrayOccupancy, '/tray/occupancy', self._tray_cb, latched)
        self.create_service(StartTask, '/task/start', self._start_cb)
        self.create_service(CancelTask, '/task/cancel', self._cancel_cb)
        self._arm_client = self.create_client(ArmCommand, '/arm/command')
        self._occ_client = self.create_client(SetOccupancy, '/tray/set_occupancy')
        self.create_timer(1.0 / self.get_parameter('loop_rate').value, self._tick)
        self._set_state(IDLE, '任务状态机就绪')
        self._auto_timer = None
        if self.get_parameter('auto_start').value:
            self._auto_timer = self.create_timer(
                self.get_parameter('auto_start_delay').value, self._auto_start)

    def _seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _machine_cb(self, msg):
        self.machine_status = msg
        self._machine_received = self._seconds()

    def _tray_cb(self, msg):
        if msg.num_slots == layout.NUM_SLOTS and len(msg.occupied) == layout.NUM_SLOTS:
            self._tray_occupied = list(msg.occupied)

    def _machine_fresh(self):
        return (self._machine_received is not None and
                self._seconds() - self._machine_received <=
                self.get_parameter('signal_timeout').value)

    def _auto_start(self):
        if self._auto_timer is not None:
            self.destroy_timer(self._auto_timer)
            self._auto_timer = None
        if self.state == IDLE:
            self._begin_task()

    def _start_cb(self, request, response):
        if self.state != IDLE:
            response.success = False
            response.message = '任务未处于 IDLE；中止后请检查场景并重启演示'
        elif not 0 <= request.slot < layout.NUM_SLOTS:
            response.success = False
            response.message = '槽位编号超出范围（0~%d）' % (layout.NUM_SLOTS - 1)
        elif self._tray_occupied is not None and not self._tray_occupied[request.slot]:
            response.success = False
            response.message = '目标槽位没有工件'
        else:
            self.slot = request.slot
            self._begin_task()
            response.success = True
            response.message = '任务已启动，槽位 %d' % self.slot
        return response

    def _cancel_cb(self, request, response):
        if self.state in (IDLE, DONE) and self._next_cycle_at is not None:
            self._next_cycle_at = None
            self._set_state(IDLE, '连续运行已停止，可调用 /task/start 再次启动')
            response.success = True
            response.message = '已取消后续自动循环，仿真保持运行'
            return response
        response.success = self.state not in (IDLE, DONE, ABORTED)
        if response.success:
            self.cancel_requested = True
            response.message = '已请求取消，当前动作结束并确认占用更新后中止'
        else:
            response.message = '当前没有可取消的任务'
        return response

    def _begin_task(self):
        # 手动启动时也撤销初次自动启动计时，避免产生额外任务。
        if self._auto_timer is not None:
            self.destroy_timer(self._auto_timer)
            self._auto_timer = None
        self._next_cycle_at = None
        self.cancel_requested = self._timeout_retried = self._return_home = False
        self._station_start = self._seconds()
        self._set_state(WAIT_STATION, '槽位 %d：等待场景、服务及工位 READY' % self.slot)

    def _set_state(self, state, detail):
        if state == DONE:
            self._completed_cycles += 1
            detail = '第 %d 轮完成：%s' % (self._completed_cycles, detail)
            if self.get_parameter('repeat_cycle').value:
                delay = self.get_parameter('repeat_delay').value
                self._next_cycle_at = self._seconds() + delay
                detail += '；%.1f 秒后自动开始下一轮' % delay
        elif state == ABORTED:
            self._next_cycle_at = None
        self.state, self.detail = state, detail
        self.get_logger().info('[任务状态] %s - %s' % (layout.TASK_STATE_NAMES[state], detail))
        msg = TaskState()
        msg.state, msg.slot, msg.detail = state, self.slot, detail
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_state.publish(msg)
        self._publish_marker()

    def _publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'panda_link0'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id = 'task_status', 0
        marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = 0.25, 0.45, 0.55
        marker.scale.z, marker.color.a = 0.07, 1.0
        marker.color.r, marker.color.g, marker.color.b = STATE_COLOR[self.state]
        marker.text = '%s: %s' % (layout.TASK_STATE_NAMES[self.state], self.detail)
        self.pub_marker.publish(marker)

    def _advance(self, state, detail):
        if self.cancel_requested:
            self.cancel_requested = False
            self._set_state(ABORTED, '任务已取消，请检查工件位置后重启演示')
        else:
            self._set_state(state, detail)

    def _arm_call(self, command):
        if not self._arm_client.service_is_ready():
            self._set_state(ABORTED, '机械臂服务不可用')
            return
        req = ArmCommand.Request()
        req.command, req.slot = command, self.slot
        self._arm_future = self._arm_client.call_async(req)
        self._arm_deadline = self._seconds() + self.get_parameter('arm_call_timeout').value

    def _poll(self, future, deadline, label):
        if not future.done():
            if self._seconds() <= deadline:
                return None
            future.cancel()
            return False, '%s超时；服务取消不等于停止机械臂，请检查现场并重启' % label
        try:
            result = future.result()
            return result.success, result.message
        except Exception as exc:
            return False, '%s异常: %s' % (label, exc)

    def _set_occupancy(self, occupied, next_state, detail):
        if not self._occ_client.service_is_ready():
            self._set_state(ABORTED, '料盘占用服务不可用')
            return
        req = SetOccupancy.Request()
        req.slot, req.occupied = self.slot, occupied
        self._occ_future = self._occ_client.call_async(req)
        self._occ_deadline = self._seconds() + self.get_parameter('occupancy_timeout').value
        self._next_after_occ = (next_state, detail)

    def _tick(self):
        if self.state == IDLE:
            if self._next_cycle_at is not None:
                if not self.get_parameter('repeat_cycle').value:
                    self._next_cycle_at = None
                elif self._seconds() >= self._next_cycle_at:
                    self._begin_task()
            return
        if self.state == ABORTED:
            return
        if self._occ_future is not None:
            result = self._poll(self._occ_future, self._occ_deadline, '料盘占用更新')
            if result is not None:
                self._occ_future = None
                if result[0]:
                    self._advance(*self._next_after_occ)
                else:
                    self._set_state(ABORTED, result[1])
            return
        if self.cancel_requested and self._arm_future is None:
            self._advance(ABORTED, '任务已取消')
            return
        if self.state == WAIT_STATION:
            if self._seconds() - self._station_start > self.get_parameter('station_timeout').value:
                self._set_state(ABORTED, '等待工位/场景/服务就绪超时')
            elif (self._machine_fresh() and self.machine_status.state == MachineStatus.READY
                  and self._tray_occupied is not None
                  and self._arm_client.service_is_ready() and self._occ_client.service_is_ready()):
                if self._tray_occupied[self.slot]:
                    self._advance(PICK, '工位就绪，从槽位 %d 取料' % self.slot)
                else:
                    self._set_state(ABORTED, '目标槽位没有工件')
            return
        if self.state in (WAIT_MACHINING, TIMEOUT):
            if not self._machine_fresh():
                self._set_state(ABORTED, '工位信号丢失或过期')
            elif self.machine_status.state == MachineStatus.DONE and self.state != TIMEOUT:
                self._advance(RETRIEVE, '加工完成，取回工件')
            elif self.state == TIMEOUT:
                # 留出一秒，确保视频中的超时提示可以看见。
                if self._seconds() - self._warning_start >= 1.0:
                    self._advance(WAIT_MACHINING, '超时后继续等待一次；不重新启动加工')
            elif self._seconds() - self._wait_start >= self.get_parameter('machining_timeout').value:
                if self.get_parameter('timeout_retry_once').value and not self._timeout_retried:
                    self._timeout_retried = True
                    self._warning_start = self._wait_start = self._seconds()
                    self._set_state(TIMEOUT, '加工超时警告：延长一次等待窗口')
                else:
                    self._set_state(ABORTED, '加工超时，任务中止')
            return
        if self.state == DONE:
            if not self.get_parameter('repeat_cycle').value:
                self._next_cycle_at = None
            if self._next_cycle_at is not None and self._seconds() < self._next_cycle_at:
                return
            self._set_state(IDLE, '循环结束，工件已放回且机械臂已回零')
            return
        commands = {PICK: 'pick', PLACE: 'place', RETRIEVE: 'retrieve',
                    RETURN: 'home' if self._return_home else 'return'}
        if self._arm_future is None:
            if self.state == PLACE and (not self._machine_fresh() or
                                        self.machine_status.state != MachineStatus.READY):
                self._set_state(ABORTED, '上料前工位未就绪，请检查工位和工件位置')
            else:
                self._arm_call(commands[self.state])
            return
        result = self._poll(self._arm_future, self._arm_deadline, '机械臂动作')
        if result is None:
            return
        self._arm_future = None
        if not result[0]:
            self._set_state(ABORTED, '%s失败: %s' % (commands[self.state], result[1]))
        elif self.state == PICK:
            self._set_occupancy(False, PLACE, '取料完成，准备上料')
        elif self.state == PLACE:
            self._wait_start = self._seconds()
            self._advance(WAIT_MACHINING, '上料完成，等待工位加工结束信号')
        elif self.state == RETRIEVE:
            self._advance(RETURN, '取回完成，放回原料盘槽位')
        elif self.state == RETURN:
            if self._return_home:
                self._advance(DONE, '工件已放回，机械臂回零成功，上下料循环完成')
            else:
                self._return_home = True
                self._set_occupancy(True, RETURN, '工件已放回，确认料盘占用后回零')


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
