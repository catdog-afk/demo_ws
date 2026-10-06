"""Transport regressions for short Cartesian moves, freshness and gripper units.

The doubles exercise real controller code; they do not validate Panda IK/collisions.
"""
import unittest
from unittest.mock import Mock, patch

from ros_doubles import Node, Future, Data, ns, install
install()
from machining_demo.arm_controller import ArmController, ARM_JOINTS, FINGER_JOINTS, make_pose
from machining_demo import layout


def completed(value):
    future = Future()
    future.complete, future.value = True, value
    return future


class LinearMotionTests(unittest.TestCase):
    def setUp(self):
        Node.overrides = {}
        self.node = ArmController()
        self.node._execute_client = Mock()
        self.node._validity_client.ready = False
        self.start = ns(is_diff=True)
        self.node._motion_start_state = Mock(return_value=self.start)
        self.trajectory = ns(joint_trajectory=ns(points=[ns()]))
        self.node._cartesian_client.call_async = Mock(return_value=completed(
            ns(error_code=ns(val=1), fraction=1.0, solution=self.trajectory)))
        self.goal_handle = ns(accepted=True, get_result_async=lambda: completed(
            ns(result=ns(error_code=ns(val=1)))))
        self.node._execute_client.send_goal_async.return_value = completed(self.goal_handle)

    def test_full_collision_checked_path_is_sent_to_execution(self):
        pose = make_pose(0.45, 0.17, 0.162)
        self.assertTrue(self.node._goto_linear(pose, '抓取点'))
        request = self.node._cartesian_client.call_async.call_args.args[0]
        self.assertIs(request.start_state, self.start)
        self.assertTrue(request.avoid_collisions)
        self.assertEqual(request.header.frame_id, 'panda_link0')
        self.assertEqual(request.link_name, 'panda_hand')
        self.assertEqual(request.group_name, 'panda_arm')
        self.assertEqual(request.waypoints, [pose])
        self.assertEqual(request.max_step, 0.002)
        self.assertGreater(request.jump_threshold, 0.0)
        self.assertEqual(request.max_velocity_scaling_factor, 0.5)
        goal = self.node._execute_client.send_goal_async.call_args.args[0]
        self.assertIs(goal.trajectory, self.trajectory)
        self.assertEqual(self.node.pub_status.messages[-1].data, '执行完成: 抓取点')

    def test_partial_or_nonfinite_path_is_never_executed(self):
        for fraction in (0.0, 0.95, 1.1, float('nan'), float('inf')):
            with self.subTest(fraction=fraction):
                self.node._cartesian_client.call_async.return_value = completed(
                    ns(error_code=ns(val=1), fraction=fraction, solution=self.trajectory))
                self.assertFalse(self.node._goto_linear(make_pose(0, 0, 0), '退出加工台'))
                self.assertIn('退出加工台', self.node._last_error)
                self.assertIn('不执行部分轨迹', self.node._last_error)
        self.node._execute_client.send_goal_async.assert_not_called()

    def test_cartesian_error_and_empty_path_are_not_success(self):
        for code, trajectory in ((-1, self.trajectory), (1, ns(joint_trajectory=ns(points=[])))):
            self.node._cartesian_client.call_async.return_value = completed(
                ns(error_code=ns(val=code), fraction=1.0, solution=trajectory))
            self.assertFalse(self.node._goto_linear(make_pose(0, 0, 0), '提起工件'))
        self.node._execute_client.send_goal_async.assert_not_called()

    def test_execution_error_is_preserved(self):
        self.goal_handle.get_result_async = lambda: completed(ns(result=ns(error_code=ns(val=-4))))
        self.assertFalse(self.node._goto_linear(make_pose(0, 0, 0), '放置点'))
        self.assertIn('放置点', self.node._last_error)
        self.assertIn('CONTROL_FAILED (-4)', self.node._last_error)

    def test_rejected_execution_goal_is_not_success(self):
        self.goal_handle.accepted = False
        self.assertFalse(self.node._goto_linear(make_pose(0, 0, 0), '抓取点'))
        self.assertIn('未接受', self.node._last_error)

    def test_missing_joint_feedback_prevents_planning(self):
        self.node._motion_start_state.return_value = None
        self.assertFalse(self.node._goto_linear(make_pose(0, 0, 0), '抓取点'))
        self.node._cartesian_client.call_async.assert_not_called()
        self.node._execute_client.send_goal_async.assert_not_called()

    def test_planning_timeout_is_recorded_and_never_executes(self):
        pending = Future()
        self.node._cartesian_client.call_async.return_value = pending
        with patch('machining_demo.arm_controller.time.monotonic', side_effect=(0.0, 11.0)):
            self.assertFalse(self.node._goto_linear(make_pose(0, 0, 0), '抓取点'))
        self.assertTrue(pending.cancelled)
        self.assertIn('查询超时', self.node._last_error)
        self.node._execute_client.send_goal_async.assert_not_called()

    def test_execution_timeout_cancels_the_goal(self):
        self.goal_handle.cancel_goal_async = Mock()
        self.goal_handle.get_result_async = lambda: Future()
        with patch('machining_demo.arm_controller.time.monotonic', side_effect=(0.0, 1.0, 100.0)):
            self.assertFalse(self.node._execute_action(self.node._execute_client, ns(), '提起工件'))
        self.goal_handle.cancel_goal_async.assert_called_once()
        self.assertIn('动作超时', self.node._last_error)

    def test_collision_diagnostic_records_contacts_without_masking_failure(self):
        self.node._validity_client.ready = True
        self.node._current_state = lambda: self.start
        self.node._validity_client.call_async = Mock(return_value=completed(
            ns(valid=False, contacts=[ns(contact_body_1='panda_leftfinger', contact_body_2='workpiece_0')])))
        self.node._diagnose_state()
        self.assertIn('panda_leftfinger/workpiece_0', self.node.pub_status.messages[-1].data)


class JointFeedbackTests(unittest.TestCase):
    def setUp(self):
        Node.overrides = {}
        self.node = ArmController()

    def feedback(self, names):
        self.node._js_cb(ns(name=names, position=[0.035] * len(names)))

    def test_old_or_incomplete_feedback_cannot_start_motion(self):
        for names in (ARM_JOINTS + FINGER_JOINTS, ARM_JOINTS):
            self.node._joint_received.clear()
            with patch('machining_demo.arm_controller.time.monotonic', return_value=1.0):
                self.feedback(names)
            with patch('machining_demo.arm_controller.time.monotonic', side_effect=(2.0, 2.1, 4.1)), \
                 patch('machining_demo.arm_controller.time.sleep'):
                self.assertIsNone(self.node._motion_start_state('抓取点'))
        self.assertIn('完整关节反馈', self.node._last_error)

    def test_next_complete_feedback_is_used(self):
        self.node._current_state = Mock(return_value=ns(joint_state='new'))
        with patch('machining_demo.arm_controller.time.monotonic', side_effect=(2.0, 2.1, 2.2, 2.3)), \
             patch('machining_demo.arm_controller.time.sleep', side_effect=lambda seconds: self.feedback(ARM_JOINTS + FINGER_JOINTS)):
            result = self.node._motion_start_state('抓取点')
        self.assertEqual(result.joint_state, 'new')

    def test_finger_tolerance_is_half_millimeter_not_ten_millimeters(self):
        goal = self.node._joint_goal(FINGER_JOINTS, [0.035, 0.035])
        self.assertTrue(all(c.tolerance_below <= 0.0005 for c in goal.joint_constraints))
        # 即使两个手指都位于允许的最小开度，仍能包住 40mm 工件。
        self.assertGreater(sum(c.position - c.tolerance_below for c in goal.joint_constraints),
                           layout.WORKPIECE_SIZE)


class ArmCycleTests(unittest.TestCase):
    def test_two_cycles_use_linear_contact_moves_and_clear_workpiece_tracking(self):
        Node.overrides = {}
        node = ArmController()
        motion = []
        node._goto_pose = lambda pose, name: motion.append(('transfer', name)) or True
        node._goto_linear = lambda pose, name: motion.append(('linear', name)) or True
        node._gripper = lambda open_: True
        node._apply_scene = lambda scene: True
        node._home = lambda: True
        for cycle in range(2):
            self.assertTrue(node._pick(0))
            self.assertEqual(node._attached_name, 'workpiece_0')
            self.assertTrue(node._place())
            self.assertIsNone(node._attached_name)
            self.assertEqual(node._machine_workpiece, 'workpiece_0')
            self.assertTrue(node._retrieve())
            self.assertIsNone(node._machine_workpiece)
            self.assertTrue(node._return(0))
            self.assertIsNone(node._attached_name)
            self.assertTrue(node._home())
        self.assertEqual([name for kind, name in motion if kind == 'linear'],
                         ['抓取点(槽位0)', '提起工件', '放置点', '退出加工台',
                          '取回抓取点', '取回提起', '放回点(槽位0)', '退出料盘'] * 2)


if __name__ == '__main__':
    unittest.main()
