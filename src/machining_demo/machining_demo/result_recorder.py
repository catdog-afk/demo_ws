"""逐轮保存 A2 任务证据；执行期间即时刷新 CSV，结束后写入摘要。"""
import csv
import os
from datetime import datetime
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from demo_interfaces.msg import TaskState, MachineStatus
from machining_demo import layout
from machining_demo.evidence import cycle_errors


class ResultRecorder(Node):
    def __init__(self):
        super().__init__('result_recorder')
        self.declare_parameter('output_dir', os.path.expanduser('~/demo_ws/results'))
        self.states_seen = []
        self.active = False
        self.slot = None
        self.start_time = None
        self._stream = self._writer = None
        self._machine_key = None
        self._event_count = 0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TaskState, '/task/state', self._task_cb, latched)
        self.create_subscription(MachineStatus, '/machine/status', self._machine_cb, 10)
        self.create_subscription(String, '/arm/status', self._arm_cb, 10)
        self.get_logger().info('运行记录节点就绪，输出目录: %s' %
                               self.get_parameter('output_dir').value)

    def _begin(self, slot):
        self.slot = slot
        self.start_time = self.get_clock().now()
        self.states_seen, self._event_count, self._machine_key = [], 0, None
        out_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        os.makedirs(out_dir, exist_ok=True)
        stamp = '%s_%s' % (datetime.now().strftime('%Y%m%d_%H%M%S_%f'), uuid4().hex)
        self._csv_path = os.path.join(out_dir, 'task_log_%s.csv' % stamp)
        self._summary_path = os.path.join(out_dir, 'summary_%s.txt' % stamp)
        self._stream = open(self._csv_path, 'x', newline='', encoding='utf-8')
        self._writer = csv.writer(self._stream)
        self._writer.writerow(['time', 'source', 'event', 'detail', 'slot'])
        self._stream.flush()
        self.active = True

    def _append(self, source, event, detail):
        if not self.active:
            return
        time_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self._writer.writerow([time_str, source, event, detail, self.slot])
        self._stream.flush()
        self._event_count += 1

    def _task_cb(self, msg):
        if not self.active:
            # 忽略重复结束状态或上一轮 IDLE；下一轮重新计算起始时刻。
            if msg.state in (TaskState.IDLE, TaskState.DONE, TaskState.ABORTED):
                return
            self._begin(msg.slot)
        if msg.slot != self.slot:
            return
        name = layout.TASK_STATE_NAMES.get(msg.state, str(msg.state))
        if not self.states_seen or self.states_seen[-1] != name:
            self.states_seen.append(name)
        self._append('task', name, msg.detail)
        if msg.state in (TaskState.DONE, TaskState.ABORTED):
            self._finish(name, msg.detail)

    def _machine_cb(self, msg):
        if not self.active:
            return
        progress = int(msg.progress) if msg.state == MachineStatus.MACHINING else None
        key = (msg.state, progress)
        if key == self._machine_key:
            return
        self._machine_key = key
        name = layout.MACHINE_STATE_NAMES.get(msg.state, str(msg.state))
        detail = '加工进度 %d%%' % progress if progress is not None else msg.detail
        self._append('machine', name, detail)

    def _arm_cb(self, msg):
        self._append('arm', 'feedback', msg.data)

    def _finish(self, final_state, detail):
        self._stream.close()
        self._stream = None
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        errors = cycle_errors(self.states_seen)
        summary = '\n'.join([
            '========== A2 运行摘要 ==========',
            '结束状态: %s' % final_state,
            '槽位: %d' % self.slot,
            '说明: %s' % detail,
            '本轮任务时长: %.1f 秒' % elapsed,
            '任务经过的状态: %s' % ' -> '.join(self.states_seen),
            'A2 阶段验收: %s' % ('通过' if not errors else '；'.join(errors)),
            '事件总数: %d' % self._event_count,
            'CSV 记录: %s' % self._csv_path,
        ])
        with open(self._summary_path + '.tmp', 'w', encoding='utf-8') as stream:
            stream.write(summary + '\n')
        os.replace(self._summary_path + '.tmp', self._summary_path)
        self.active = False
        self.get_logger().info('\n%s' % summary)

    def destroy_node(self):
        if self.active:
            self._append('recorder', 'INTERRUPTED', '节点关闭，任务未完成，保留已有证据')
            self._finish('INTERRUPTED', '节点关闭，任务未完成')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ResultRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
