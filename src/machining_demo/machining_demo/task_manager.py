"""角色D·任务与记录：任务流程状态机节点（自行开发的核心逻辑）。

状态机：
    IDLE
      -> WAIT_STATION   （工位忙碌，等待空闲 —— 拓展：忙碌等待）
      -> PICK           （料盘取料）
      -> PLACE          （上料到加工台）
      -> WAIT_MACHINING （等待加工完成；超时 -> TIMEOUT 警告 —— 拓展：超时提示）
      -> RETRIEVE       （取回工件）
      -> RETURN         （放回料盘）
      -> DONE           （单次循环完成）
      -> IDLE
    任意状态可被取消 -> ABORTED（取消/异常处理）

发布话题：
    /task/state          任务状态（TaskState 消息）
    /task/status_marker  RViz 文本标注（显示当前阶段）
提供服务：
    /task/start          启动任务（StartTask，指定槽位）
    /task/cancel         取消任务（CancelTask）
调用服务：
    /arm/command         机械臂动作指令
    /tray/set_occupancy  料盘占用更新
订阅话题：
    /machine/status      机床工位状态
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

from demo_interfaces.msg import TaskState, MachineStatus
from demo_interfaces.srv import (StartTask, CancelTask, ArmCommand,
                                 SetOccupancy)

from machining_demo import layout

# 状态常量（与 TaskState.msg 保持一致）
IDLE, PICK, PLACE, WAIT_MACHINING, RETRIEVE, RETURN = 0, 1, 2, 3, 4, 5
DONE, WAIT_STATION, TIMEOUT, ABORTED = 6, 7, 8, 9

# 状态显示文字（用于 RViz 文本标注）
STATE_TEXT = {
    IDLE: '空闲，等待任务启动',
    PICK: '取料：从料盘抓取工件',
    PLACE: '上料：放入加工台',
    WAIT_MACHINING: '等待加工完成...',
    RETRIEVE: '取回：从加工台取回工件',
    RETURN: '放回：将工件放回料盘',
    DONE: '任务完成！上下料循环结束',
    WAIT_STATION: '工位忙碌，等待空闲...',
    TIMEOUT: '加工超时警告！',
    ABORTED: '任务中止',
}

# RViz 文本标注颜色（RGB）
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
        self.declare_parameter('auto_start', True)
        self.declare_parameter('auto_start_delay', 5.0)
        self.declare_parameter('machining_timeout', 8.0)
        self.declare_parameter('timeout_retry_once', True)
        self.declare_parameter('arm_call_timeout', 60.0)
        self.declare_parameter('loop_rate', 10.0)

        self.state = IDLE
        self.slot = 0
        self.detail = ''
        self.machine_status = MachineStatus()  # 最新机床状态
        self.cancel_requested = False
        self._arm_future = None               # 进行中的机械臂服务调用
        self._arm_deadline = None
        self._occ_future = None
        self._wait_start = None               # WAIT_MACHINING 开始时刻
        self._timeout_retried = False

        # 话题
        self.pub_state = self.create_publisher(TaskState, '/task/state', 10)
        self.pub_marker = self.create_publisher(Marker, '/task/status_marker', 10)
        self.create_subscription(MachineStatus, '/machine/status',
                                 self._machine_cb, 10)

        # 服务
        self.create_service(StartTask, '/task/start', self._start_cb)
        self.create_service(CancelTask, '/task/cancel', self._cancel_cb)

        # 客户端
        self._arm_client = self.create_client(ArmCommand, '/arm/command')
        self._occ_client = self.create_client(SetOccupancy, '/tray/set_occupancy')

        rate = self.get_parameter('loop_rate').value
        self.create_timer(1.0 / rate, self._tick)

        self._set_state(IDLE, '任务状态机就绪')
        self._auto_timer = None
        if self.get_parameter('auto_start').value:
            delay = self.get_parameter('auto_start_delay').value
            self._auto_timer = self.create_timer(delay, self._auto_start)
            self.get_logger().info('%.0f 秒后自动启动演示任务' % delay)

    # ---------- 回调 ----------
    def _machine_cb(self, msg):
        self.machine_status = msg

    def _auto_start(self):
        if self._auto_timer is not None:
            self.destroy_timer(self._auto_timer)
            self._auto_timer = None
        if self.state == IDLE:
            self._begin_task()

    def _start_cb(self, request, response):
        if self.state != IDLE:
            response.success = False
            response.message = '任务正在执行中，无法重复启动'
            return response
        if request.slot >= layout.NUM_SLOTS:
            response.success = False
            response.message = '槽位编号超出范围（0~%d）' % (layout.NUM_SLOTS - 1)
            return response
        self.slot = request.slot
        self._begin_task()
        response.success = True
        response.message = '任务已启动，槽位 %d' % self.slot
        return response

    def _cancel_cb(self, request, response):
        if self.state == IDLE or self.state in (DONE, ABORTED):
            response.success = False
            response.message = '当前没有可取消的任务'
            return response
        self.cancel_requested = True
        self.get_logger().warn('收到取消请求，当前阶段结束后中止')
        response.success = True
        response.message = '已请求取消'
        return response

    # ---------- 状态机 ----------
    def _begin_task(self):
        self.cancel_requested = False
        self._timeout_retried = False
        self._set_state(PICK, '任务启动，目标槽位 %d' % self.slot)

    def _set_state(self, state, detail):
        self.state = state
        self.detail = detail
        self.get_logger().info('[任务状态] %s - %s'
                               % (layout.TASK_STATE_NAMES[state], detail))
        msg = TaskState()
        msg.state = state
        msg.slot = self.slot
        msg.detail = detail
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_state.publish(msg)
        self._publish_marker()

    def _publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'panda_link0'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'task_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.25
        marker.pose.position.y = 0.45
        marker.pose.position.z = 0.55
        marker.scale.z = 0.09
        marker.color.a = 1.0
        r, g, b = STATE_COLOR.get(self.state, (1.0, 1.0, 1.0))
        marker.color.r, marker.color.g, marker.color.b = r, g, b
        marker.text = '任务状态: %s - %s' % (
            layout.TASK_STATE_NAMES[self.state], self.detail)
        self.pub_marker.publish(marker)

    # ---------- 机械臂服务调用（异步，带超时） ----------
    def _arm_call(self, command, slot=0):
        req = ArmCommand.Request()
        req.command = command
        req.slot = slot
        self._arm_future = self._arm_client.call_async(req)
        self._arm_deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self.get_parameter('arm_call_timeout').value)

    def _arm_call_done(self):
        """机械臂调用完成或超时，返回 (success, message)。"""
        if self._arm_future is None:
            return True, ''
        if not self._arm_future.done():
            if self.get_clock().now() > self._arm_deadline:
                self._arm_future.cancel()
                self._arm_future = None
                return False, '机械臂调用超时'
            return None  # 仍在执行
        result = self._arm_future.result()
        self._arm_future = None
        return result.success, result.message

    def _set_occupancy(self, slot, occupied):
        req = SetOccupancy.Request()
        req.slot = slot
        req.occupied = occupied
        self._occ_future = self._occ_client.call_async(req)

    # ---------- 主循环 ----------
    def _tick(self):
        if self.cancel_requested and self._arm_future is None:
            self._set_state(ABORTED, '任务已取消')
            self.cancel_requested = False
            return

        # 处理占用更新（非阻塞）
        if self._occ_future is not None and self._occ_future.done():
            self._occ_future = None

        if self.state == PICK:
            if self._arm_future is None:
                # 拓展：工位忙碌则等待
                if self.machine_status.state in (MachineStatus.BUSY,):
                    self._set_state(WAIT_STATION, '工位忙碌，等待空闲后开始取料')
                else:
                    self._arm_call('pick', self.slot)
            else:
                r = self._arm_call_done()
                if r is not None:
                    ok, message = r
                    if ok:
                        self._set_occupancy(self.slot, False)
                        self._set_state(PLACE, '取料完成，准备上料')
                    else:
                        self._set_state(ABORTED, '取料失败: %s' % message)

        elif self.state == WAIT_STATION:
            if self.machine_status.state != MachineStatus.BUSY:
                self._set_state(PICK, '工位已空闲，开始取料')

        elif self.state == PLACE:
            if self._arm_future is None:
                self._arm_call('place')
            else:
                r = self._arm_call_done()
                if r is not None:
                    ok, message = r
                    if ok:
                        self._wait_start = self.get_clock().now()
                        self._set_state(WAIT_MACHINING, '上料完成，等待加工')
                    else:
                        self._set_state(ABORTED, '上料失败: %s' % message)

        elif self.state == WAIT_MACHINING:
            if self.machine_status.state == MachineStatus.DONE:
                self._set_state(RETRIEVE, '加工完成，取回工件')
            else:
                elapsed = (self.get_clock().now() -
                           self._wait_start).nanoseconds / 1e9
                timeout = self.get_parameter('machining_timeout').value
                if elapsed > timeout:
                    self.get_logger().warn('加工超过 %.0f 秒，触发超时警告'
                                           % timeout)
                    if (self.get_parameter('timeout_retry_once').value
                            and not self._timeout_retried):
                        self._timeout_retried = True
                        self._set_state(TIMEOUT, '加工超时警告：继续等待（重试一次）')
                        self._wait_start = self.get_clock().now()
                        self._set_state(WAIT_MACHINING, '超时后继续等待加工')
                    else:
                        self._set_state(ABORTED, '加工超时，任务中止')

        elif self.state == RETRIEVE:
            if self._arm_future is None:
                self._arm_call('retrieve')
            else:
                r = self._arm_call_done()
                if r is not None:
                    ok, message = r
                    if ok:
                        self._set_state(RETURN, '取回完成，放回料盘')
                    else:
                        self._set_state(ABORTED, '取回失败: %s' % message)

        elif self.state == RETURN:
            if self._arm_future is None:
                self._arm_call('return', self.slot)
            else:
                r = self._arm_call_done()
                if r is not None:
                    ok, message = r
                    if ok:
                        self._set_occupancy(self.slot, True)
                        self._arm_call('home')
                        self._set_state(DONE, '工件已放回，任务完成')
                    else:
                        self._set_state(ABORTED, '放回失败: %s' % message)

        elif self.state == DONE:
            if self._arm_future is None:
                self._set_state(IDLE, '循环结束，等待下一次任务')
            else:
                r = self._arm_call_done()
                if r is not None:
                    self._arm_future = None
                    self._set_state(IDLE, '循环结束，等待下一次任务')


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
