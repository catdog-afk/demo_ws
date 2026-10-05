"""角色D·任务与记录：运行记录节点。

订阅任务状态、机床状态与机械臂执行反馈，将全部事件写入 CSV 运行记录；
任务结束（DONE / ABORTED）时生成摘要文件（summary.txt）并打印。

输出目录（参数 output_dir，由 launch 传入，默认为 ~/demo_ws/results）：
    task_log_YYYYmmdd_HHMMSS.csv   逐条事件记录
    summary_YYYYmmdd_HHMMSS.txt    本次运行摘要
"""
import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from demo_interfaces.msg import TaskState, MachineStatus

from machining_demo import layout


class ResultRecorder(Node):
    def __init__(self):
        super().__init__('result_recorder')
        self.declare_parameter('output_dir', os.path.expanduser('~/demo_ws/results'))

        self.states_seen = []          # 本次任务经过的状态序列
        self.records = []              # 全部事件 (time_str, source, event, detail)
        self.start_time = self.get_clock().now()
        self.finished = False

        self.create_subscription(TaskState, '/task/state', self._task_cb, 10)
        self.create_subscription(MachineStatus, '/machine/status',
                                 self._machine_cb, 10)
        self.create_subscription(String, '/arm/status', self._arm_cb, 10)

        self.get_logger().info('运行记录节点就绪，输出目录: %s'
                               % self.get_parameter('output_dir').value)

    def _append(self, source, event, detail):
        now = self.get_clock().now()
        time_str = datetime.fromtimestamp(now.nanoseconds / 1e9).strftime(
            '%H:%M:%S.%f')[:-3]
        self.records.append((time_str, source, event, detail))

    def _task_cb(self, msg):
        name = layout.TASK_STATE_NAMES.get(msg.state, str(msg.state))
        if not self.states_seen or self.states_seen[-1] != name:
            self.states_seen.append(name)
        self._append('task', name, msg.detail)
        if msg.state in (TaskState.DONE, TaskState.ABORTED) and not self.finished:
            self.finished = True
            self._finish(name, msg.detail)

    def _machine_cb(self, msg):
        name = layout.MACHINE_STATE_NAMES.get(msg.state, str(msg.state))
        if msg.state == MachineStatus.MACHINING:
            self._append('machine', name,
                         '加工进度 %.0f%%' % msg.progress)
        elif msg.state != 0:  # 非 READY 状态记录一次
            self._append('machine', name, msg.detail)

    def _arm_cb(self, msg):
        self._append('arm', 'feedback', msg.data)

    # ---------- 结束处理 ----------
    def _finish(self, final_state, detail):
        out_dir = self.get_parameter('output_dir').value
        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        csv_path = os.path.join(out_dir, 'task_log_%s.csv' % stamp)
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'source', 'event', 'detail'])
            writer.writerows(self.records)

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        lines = [
            '========== 运行摘要 ==========',
            '结束状态: %s' % final_state,
            '说明: %s' % detail,
            '总运行时长: %.1f 秒' % elapsed,
            '任务经过的状态: %s' % ' -> '.join(self.states_seen),
            '事件总数: %d' % len(self.records),
            'CSV 记录: %s' % csv_path,
        ]
        summary = '\n'.join(lines)
        self.get_logger().info('\n%s' % summary)
        summary_path = os.path.join(out_dir, 'summary_%s.txt' % stamp)
        with open(summary_path, 'w', encoding='utf-8') as f:
            f.write(summary + '\n')
        self.get_logger().info('运行记录已保存: %s' % csv_path)
        # 重置，为下一轮任务记录做准备
        self.states_seen = []
        self.records = []
        self.finished = False


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
