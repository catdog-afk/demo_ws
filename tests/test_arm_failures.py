"""Regression coverage for retaining the failing motion stage and MoveIt code."""
import unittest
from unittest.mock import Mock, patch

from ros_doubles import Node, Future, Data, ns, install
install()
from machining_demo.arm_controller import ArmController


class ArmFailureTests(unittest.TestCase):
    def setUp(self):
        Node.overrides = {}
        self.node = ArmController()
        self.node._action_client = Mock()
        self.node._wait_ready = lambda: True
        self.node._current_state = lambda: ns()
        self.node._motion_start_state = lambda name: ns()
        self.node._validity_client.ready = False

    def request_factory(self):
        return ns(workspace_parameters=ns(header=ns(), min_corner=ns(), max_corner=ns()))

    def test_pick_response_preserves_the_failing_stage(self):
        def failed_pick(slot):
            self.node._failure('抓取点：规划/执行失败 START_STATE_IN_COLLISION (-10)')
            return False
        self.node._pick = failed_pick
        response = self.node._cmd_cb(ns(command='pick', slot=0), ns())
        self.assertFalse(response.success)
        self.assertIn('抓取点', response.message)
        self.assertIn('START_STATE_IN_COLLISION (-10)', response.message)
        self.assertEqual(self.node.pub_status.messages[-1].data, response.message)

    def test_next_command_does_not_reuse_previous_error(self):
        self.node._last_error = 'previous failure'
        self.node._pick = lambda slot: True
        response = self.node._cmd_cb(ns(command='pick', slot=0), ns())
        self.assertTrue(response.success)
        self.assertEqual(response.message, 'ok')
        self.assertEqual(self.node._last_error, '')

    def test_moveit_error_code_is_recorded_in_feedback(self):
        result_future = Future()
        result_future.complete = True
        result_future.value = ns(result=ns(error_code=ns(val=-10)))
        goal_future = Future()
        goal_future.complete = True
        goal_future.value = ns(accepted=True, get_result_async=lambda: result_future)
        self.node._action_client.send_goal_async.return_value = goal_future
        with patch('machining_demo.arm_controller.MotionPlanRequest', side_effect=self.request_factory), \
             patch('machining_demo.arm_controller.MoveGroup', ns(Goal=Data)):
            self.assertFalse(self.node._plan_execute('panda_arm', ns(), '预抓取点'))
        self.assertIn('预抓取点', self.node._last_error)
        self.assertIn('START_STATE_IN_COLLISION (-10)', self.node._last_error)
        self.assertEqual(self.node.pub_status.messages[-1].data, self.node._last_error)

    def test_shutdown_does_not_send_another_motion_goal(self):
        with patch('rclpy.ok', return_value=False):
            self.assertFalse(self.node._plan_execute('panda_arm', ns(), '提起工件'))
        self.node._action_client.send_goal_async.assert_not_called()


if __name__ == '__main__':
    unittest.main()
