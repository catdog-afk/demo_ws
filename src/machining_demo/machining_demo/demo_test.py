"""角色E·集成与测试：自动化测试脚本。

流程：
    1. 等待任务状态机与启动服务就绪；
    2. 调用 /task/start 启动一次上下料任务；
    3. 订阅 /task/state 记录状态流转；
    4. 等待 DONE / ABORTED 或超时（默认 180 秒）；
    5. 校验运行记录文件已生成并包含 DONE 状态，输出测试结论。

用法：
    ros2 run machining_demo demo_test --ros-args -p slot:=1
"""
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from demo_interfaces.srv import StartTask
from demo_interfaces.msg import TaskState

from machining_demo import layout


class DemoTest(Node):
    def __init__(self):
        super().__init__('demo_test')
        self.declare_parameter('slot', 0)
        self.declare_parameter('timeout', 180.0)
        self.declare_parameter('output_dir',
                               os.path.expanduser('~/demo_ws/results'))

        self.transitions = []
        self.create_subscription(TaskState, '/task/state', self._state_cb, 10)
        self.start_client = self.create_client(StartTask, '/task/start')

    def _state_cb(self, msg):
        name = layout.TASK_STATE_NAMES.get(msg.state, str(msg.state))
        if not self.transitions or self.transitions[-1] != name:
            self.transitions.append(name)
            self.get_logger().info('[测试] 状态 -> %s' % name)

    def wait_service(self, timeout=30.0):
        deadline = time.time() + timeout
        while not self.start_client.wait_for_service(timeout_sec=1.0):
            if time.time() > deadline:
                return False
            self.get_logger().info('[测试] 等待 /task/start 服务...')
        return True

    def run(self):
        if not self.wait_service():
            self.get_logger().error('[测试] 失败：/task/start 服务不可用')
            return 1

        req = StartTask.Request()
        req.slot = self.get_parameter('slot').value
        self.get_logger().info('[测试] 启动任务（槽位 %d）...' % req.slot)
        future = self.start_client.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if future.done():
                resp = future.result()
                if not resp.success:
                    self.get_logger().error('[测试] 失败：%s' % resp.message)
                    return 1
                break

        deadline = time.time() + self.get_parameter('timeout').value
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.transitions and self.transitions[-1] in ('DONE', 'ABORTED'):
                break

        self.get_logger().info('[测试] 状态流转: %s'
                               % ' -> '.join(self.transitions))
        if not self.transitions or self.transitions[-1] != 'DONE':
            self.get_logger().error('[测试] 失败：任务未以 DONE 结束')
            return 1

        # 校验运行记录文件
        out_dir = self.get_parameter('output_dir').value
        logs = sorted(os.path.join(out_dir, f)
                      for f in os.listdir(out_dir)
                      if f.startswith('task_log_'))
        if not logs:
            self.get_logger().error('[测试] 失败：未生成运行记录文件')
            return 1
        latest = logs[-1]
        with open(latest, encoding='utf-8') as f:
            content = f.read()
        if 'DONE' not in content:
            self.get_logger().error('[测试] 失败：记录中缺少 DONE 状态')
            return 1

        self.get_logger().info('[测试] 通过！运行记录: %s' % latest)
        self.get_logger().info('[测试] 完整状态序列: %s'
                               % ' -> '.join(self.transitions))
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = DemoTest()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
