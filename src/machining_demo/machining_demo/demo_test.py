"""ROS 集成验收：观察本轮完整阶段，并验证本轮新生成的 CSV 与摘要。

先启动 auto_start:=false，再运行 ros2 run machining_demo demo_test。
expected_final:=ABORTED 可用于持久加工超时场景，require_timeout:=true 验证警告。
"""
import glob
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from demo_interfaces.srv import StartTask
from demo_interfaces.msg import TaskState
from machining_demo import layout
from machining_demo.evidence import cycle_errors, log_errors


class DemoTest(Node):
    def __init__(self):
        super().__init__('demo_test')
        for name, value in {'slot': 0, 'timeout': 300.0, 'expected_final': 'DONE',
                            'require_timeout': False, 'output_dir': os.path.expanduser('~/demo_ws/results')}.items():
            self.declare_parameter(name, value)
        self.transitions = []
        self._observing = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TaskState, '/task/state', self._state_cb, latched)
        self.start_client = self.create_client(StartTask, '/task/start')

    def _state_cb(self, msg):
        if not self._observing or msg.slot != self.get_parameter('slot').value:
            return
        name = layout.TASK_STATE_NAMES.get(msg.state, str(msg.state))
        if name == 'IDLE':
            return
        if not self.transitions or self.transitions[-1] != name:
            self.transitions.append(name)
            self.get_logger().info('[测试] 状态 -> %s' % name)

    def _fail(self, message):
        self.get_logger().error('[测试] 失败：' + message)
        return 1

    def run(self):
        slot = self.get_parameter('slot').value
        expected = self.get_parameter('expected_final').value
        if not 0 <= slot < layout.NUM_SLOTS or expected not in ('DONE', 'ABORTED'):
            return self._fail('slot 或 expected_final 参数无效')
        out_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        # 排除仓库自带的历史结果；本次必须生成新的文件。
        old_logs = set(glob.glob(os.path.join(out_dir, 'task_log_*.csv')))
        if not self.start_client.wait_for_service(timeout_sec=30.0):
            return self._fail('/task/start 服务不可用')
        # 处理初次订阅的历史快照，再开始观察本次请求。
        rclpy.spin_once(self, timeout_sec=0.2)
        self._observing = True
        request = StartTask.Request()
        request.slot = slot
        future = self.start_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            future.cancel()
            return self._fail('启动服务响应超时')
        try:
            response = future.result()
        except Exception as exc:
            return self._fail('启动服务异常: %s' % exc)
        if not response.success:
            return self._fail(response.message + '；测试应使用 auto_start:=false')
        deadline = time.monotonic() + self.get_parameter('timeout').value
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.transitions and self.transitions[-1] in ('DONE', 'ABORTED'):
                break
        if not self.transitions or self.transitions[-1] != expected:
            return self._fail('预期 %s，实际状态 %s' % (expected, self.transitions))
        if expected == 'DONE' and cycle_errors(self.transitions):
            return self._fail('缺少完整 A2 阶段: %s' % self.transitions)
        if self.get_parameter('require_timeout').value and 'TIMEOUT' not in self.transitions:
            return self._fail('本轮没有观察到 TIMEOUT 警告')
        # 记录节点和测试节点异步接收结束状态，等待其刷新并写入摘要。
        deadline = time.monotonic() + 5.0
        last_error = '没有本轮新 CSV 和摘要'
        while rclpy.ok() and time.monotonic() < deadline:
            for path in sorted(set(glob.glob(os.path.join(out_dir, 'task_log_*.csv'))) - old_logs):
                summary = os.path.join(out_dir, os.path.basename(path).replace('task_log_', 'summary_').replace('.csv', '.txt'))
                if not os.path.isfile(summary):
                    continue
                with open(summary, encoding='utf-8') as stream:
                    content = stream.read()
                if ('结束状态: %s\n' % expected not in content or '槽位: %d\n' % slot not in content):
                    continue
                errors = log_errors(path, slot, expected)
                if errors:
                    last_error = '；'.join(errors)
                    continue
                self.get_logger().info('[测试] 通过，记录: %s' % path)
                return 0
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._fail(last_error)


def main(args=None):
    rclpy.init(args=args)
    node = DemoTest()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
