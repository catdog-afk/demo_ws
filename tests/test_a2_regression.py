import csv
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from ros_doubles import Node, Future, Client, MachineStatus, TaskState, TrayOccupancy, ns, install
install()
from machining_demo.task_manager import TaskManager, WAIT_STATION, PICK, PLACE, WAIT_MACHINING, RETRIEVE, RETURN, DONE, IDLE, ABORTED, TIMEOUT
from machining_demo.machine_simulator import MachineSimulator
from machining_demo.result_recorder import ResultRecorder
from machining_demo.scene_manager import SceneManager
from machining_demo.arm_controller import ArmController
from machining_demo.evidence import cycle_errors, log_errors
from machining_demo import layout


class TaskTests(unittest.TestCase):
    def setUp(self):
        Node.overrides = {'auto_start': False, 'repeat_cycle': False, 'slot': 2}
        self.node = TaskManager()

    def ready(self, state=MachineStatus.READY):
        self.node._machine_cb(MachineStatus(state))
        self.node._tray_cb(TrayOccupancy())

    def start(self):
        self.ready()
        self.node._begin_task()
        self.node._tick()
        self.assertEqual(self.node.state, PICK)

    def finish_arm(self, success=True):
        self.node._tick()
        self.node._arm_future.resolve(success)
        self.node._tick()

    def finish_occupancy(self, success=True):
        self.node._occ_future.resolve(success)
        self.node._tick()

    def loaded(self):
        self.start()
        self.finish_arm()
        self.finish_occupancy()
        self.finish_arm()
        self.assertEqual(self.node.state, WAIT_MACHINING)

    def returned(self):
        self.loaded()
        self.ready(MachineStatus.DONE)
        self.node._tick()
        self.finish_arm()
        self.finish_arm()
        self.finish_occupancy()

    def test_complete_cycle_uses_requested_slot_and_waits_for_home(self):
        self.returned()
        self.assertEqual(self.node.state, RETURN)
        self.assertNotIn(DONE, [msg.state for msg in self.node.pub_state.messages])
        self.finish_arm()
        self.assertEqual(self.node.state, DONE)
        self.node._tick()
        self.assertEqual(self.node.state, IDLE)
        self.assertEqual([call.command for call in self.node._arm_client.calls],
                         ['pick', 'place', 'retrieve', 'return', 'home'])
        self.assertTrue(all(call.slot == 2 for call in self.node._arm_client.calls))
        self.assertEqual([call.occupied for call in self.node._occ_client.calls], [False, True])

    def test_machine_unknown_or_busy_never_starts_pick(self):
        self.node._begin_task()
        self.node._tick()
        self.assertEqual(self.node.state, WAIT_STATION)
        self.ready(MachineStatus.BUSY)
        self.node._tick()
        self.assertFalse(self.node._arm_client.calls)
        self.ready()
        self.node._tick()
        self.assertEqual(self.node.state, PICK)

    def test_existing_machining_or_done_is_not_ready(self):
        for state in (MachineStatus.MACHINING, MachineStatus.DONE):
            self.ready(state)
            self.node._begin_task()
            self.node._tick()
            self.assertEqual(self.node.state, WAIT_STATION)

    def test_station_timeout_is_bounded(self):
        self.node._begin_task()
        self.node.clock.advance(31)
        self.node._tick()
        self.assertEqual(self.node.state, ABORTED)

    def test_scene_and_services_are_required(self):
        self.node._machine_cb(MachineStatus())
        self.node._begin_task()
        self.node._tick()
        self.assertEqual(self.node.state, WAIT_STATION)
        self.node._tray_cb(TrayOccupancy())
        self.node._arm_client.ready = False
        self.node._tick()
        self.assertEqual(self.node.state, WAIT_STATION)

    def test_empty_slot_is_rejected(self):
        self.node._tray_cb(TrayOccupancy([True, True, False]))
        response = self.node._start_cb(ns(slot=2), ns())
        self.assertFalse(response.success)
        self.assertEqual(self.node.state, IDLE)

    def test_invalid_slot_is_rejected(self):
        for slot in (-1, 3, 255):
            self.assertFalse(self.node._start_cb(ns(slot=slot), ns()).success)

    def test_cancel_during_pick_updates_occupancy_without_place(self):
        self.start()
        self.node._tick()
        self.assertTrue(self.node._cancel_cb(ns(), ns()).success)
        self.node._arm_future.resolve()
        self.node._tick()
        self.assertEqual(self.node.state, PICK)
        self.finish_occupancy()
        self.assertEqual(self.node.state, ABORTED)
        self.node._tick()
        self.assertEqual([call.command for call in self.node._arm_client.calls], ['pick'])

    def test_cancel_while_waiting_is_immediate(self):
        self.node._begin_task()
        self.node._cancel_cb(ns(), ns())
        self.node._tick()
        self.assertEqual(self.node.state, ABORTED)
        self.assertFalse(self.node._arm_client.calls)

    def test_cancelled_task_cannot_restart_without_scene_recovery(self):
        self.node.state = ABORTED
        self.assertFalse(self.node._start_cb(ns(slot=0), ns()).success)

    def test_failed_home_cannot_report_done(self):
        self.returned()
        self.finish_arm(False)
        self.assertEqual(self.node.state, ABORTED)
        self.assertNotIn(DONE, [msg.state for msg in self.node.pub_state.messages])

    def test_occupancy_failure_prevents_next_action(self):
        self.start()
        self.finish_arm()
        self.finish_occupancy(False)
        self.assertEqual(self.node.state, ABORTED)
        self.assertEqual(len(self.node._arm_client.calls), 1)

    def test_occupancy_timeout_prevents_next_action(self):
        self.start()
        self.finish_arm()
        future = self.node._occ_future
        self.node.clock.advance(6)
        self.node._tick()
        self.assertEqual(self.node.state, ABORTED)
        self.assertTrue(future.cancelled)

    def test_arm_exception_is_recorded_as_abort(self):
        self.start()
        self.node._tick()
        future = self.node._arm_future
        future.error, future.complete = RuntimeError('transport disconnected'), True
        self.node._tick()
        self.assertEqual(self.node.state, ABORTED)

    def test_arm_timeout_is_not_success(self):
        self.start()
        self.node._tick()
        future = self.node._arm_future
        self.node.clock.advance(661)
        self.node._tick()
        self.assertEqual(self.node.state, ABORTED)
        self.assertTrue(future.cancelled)

    def test_machine_disconnection_is_not_completion(self):
        self.loaded()
        self.node.clock.advance(3)
        self.node._tick()
        self.assertEqual(self.node.state, ABORTED)

    def test_station_becoming_busy_before_place_blocks_loading(self):
        self.start()
        self.finish_arm()
        self.finish_occupancy()
        self.ready(MachineStatus.BUSY)
        self.node._tick()
        self.assertEqual(self.node.state, ABORTED)
        self.assertEqual(len(self.node._arm_client.calls), 1)

    def test_timeout_warning_is_visible_and_allows_completion(self):
        self.loaded()
        self.node.clock.advance(8.1)
        self.ready(MachineStatus.MACHINING)
        self.node._tick()
        self.assertEqual(self.node.state, TIMEOUT)
        self.node.clock.advance(0.5)
        self.ready(MachineStatus.DONE)
        self.node._tick()
        self.assertEqual(self.node.state, TIMEOUT)
        self.node.clock.advance(0.6)
        self.node._tick()
        self.assertEqual(self.node.state, WAIT_MACHINING)
        self.node._tick()
        self.assertEqual(self.node.state, RETRIEVE)

    def test_second_timeout_aborts(self):
        self.loaded()
        for elapsed in (8.1, 1.1, 7.0):
            self.node.clock.advance(elapsed)
            self.ready(MachineStatus.MACHINING)
            self.node._tick()
        self.assertEqual(self.node.state, ABORTED)

    def test_timeout_without_retry_aborts(self):
        self.loaded()
        self.node.params['timeout_retry_once'] = False
        self.node.clock.advance(8.1)
        self.ready(MachineStatus.MACHINING)
        self.node._tick()
        self.assertEqual(self.node.state, ABORTED)

    def test_bad_parameters_fail_early(self):
        for values in ({'slot': 3}, {'machining_timeout': 0.0},
                       {'loop_rate': float('nan')}, {'repeat_delay': 0.0}):
            Node.overrides = values
            with self.assertRaises(ValueError):
                TaskManager()

    def test_continuous_mode_runs_two_complete_cycles_without_relaunch(self):
        self.node.params['repeat_cycle'] = True
        self.returned()
        self.finish_arm()
        self.assertEqual(self.node._completed_cycles, 1)
        self.node._tick()
        self.assertEqual(self.node.state, DONE)
        self.assertEqual(len(self.node._arm_client.calls), 5)
        self.node.clock.advance(3.1)
        self.ready()
        self.node._tick()  # 保留完成状态至轮间间隔结束
        self.node._tick()  # 自动进入下一轮，未调用 start 服务
        self.assertEqual(self.node.state, WAIT_STATION)
        self.node._tick()
        self.finish_arm()
        self.finish_occupancy()
        self.finish_arm()
        self.ready(MachineStatus.DONE)
        self.node._tick()
        self.finish_arm()
        self.finish_arm()
        self.finish_occupancy()
        self.finish_arm()
        self.assertEqual(self.node._completed_cycles, 2)
        self.assertEqual(self.node.state, DONE)
        self.assertEqual([call.command for call in self.node._arm_client.calls],
                         ['pick', 'place', 'retrieve', 'return', 'home'] * 2)
        self.assertTrue(all(call.slot == 2 for call in self.node._arm_client.calls))

    def test_cancel_between_cycles_keeps_simulation_idle(self):
        self.node.params['repeat_cycle'] = True
        self.returned()
        self.finish_arm()
        self.assertTrue(self.node._cancel_cb(ns(), ns()).success)
        self.assertEqual(self.node.state, IDLE)
        self.node.clock.advance(100)
        self.ready()
        self.node._tick()
        self.assertEqual(self.node.state, IDLE)
        self.assertEqual(len(self.node._arm_client.calls), 5)
        self.assertTrue(self.node._start_cb(ns(slot=1), ns()).success)
        self.assertEqual(self.node.state, WAIT_STATION)

    def test_cancel_active_continuous_task_does_not_restart(self):
        self.node.params['repeat_cycle'] = True
        self.start()
        self.node._cancel_cb(ns(), ns())
        self.node._tick()
        self.node.clock.advance(100)
        self.ready()
        self.node._tick()
        self.assertEqual(self.node.state, ABORTED)
        self.assertIsNone(self.node._next_cycle_at)
        self.assertFalse(self.node._arm_client.calls)

    def test_failed_continuous_cycle_does_not_restart(self):
        self.node.params['repeat_cycle'] = True
        self.returned()
        self.finish_arm(False)
        self.node.clock.advance(100)
        self.ready()
        self.node._tick()
        self.assertEqual(self.node.state, ABORTED)
        self.assertEqual(self.node._completed_cycles, 0)
        self.assertIsNone(self.node._next_cycle_at)

    def test_next_cycle_waits_for_machine_ready(self):
        self.node.params['repeat_cycle'] = True
        self.returned()
        self.finish_arm()
        self.node.clock.advance(3.1)
        self.ready(MachineStatus.BUSY)
        self.node._tick()
        self.node._tick()
        self.node._tick()
        self.assertEqual(self.node.state, WAIT_STATION)
        self.assertEqual(len(self.node._arm_client.calls), 5)
        self.ready()
        self.node._tick()
        self.assertEqual(self.node.state, PICK)

    def test_single_cycle_can_start_again_without_relaunch(self):
        self.returned()
        self.finish_arm()
        self.node._tick()
        self.node.clock.advance(100)
        self.ready()
        self.node._tick()
        self.assertEqual(self.node.state, IDLE)
        self.assertTrue(self.node._start_cb(ns(slot=0), ns()).success)

    def test_manual_start_disables_pending_initial_auto_start(self):
        Node.overrides = {'auto_start': True, 'repeat_cycle': False}
        self.node = TaskManager()
        self.assertIsNotNone(self.node._auto_timer)
        self.node._start_cb(ns(slot=0), ns())
        self.assertIsNone(self.node._auto_timer)


class MachineTests(unittest.TestCase):
    def setUp(self):
        Node.overrides = {'initial_busy_sec': 1.0, 'machining_sec': 5.0}
        self.node = MachineSimulator()

    def test_machine_cycle_and_progress_reset(self):
        self.node.clock.advance(1.1)
        self.node._tick()
        self.assertEqual(self.node.state, MachineStatus.READY)
        self.node._workpiece_cb(ns(data=True))
        self.node._tick()
        self.assertEqual(self.node.state, MachineStatus.MACHINING)
        self.node.clock.advance(5)
        self.node._tick()
        self.assertEqual(self.node.state, MachineStatus.DONE)
        self.node._workpiece_cb(ns(data=False))
        self.node._tick()
        self.assertEqual((self.node.state, self.node.progress), (MachineStatus.READY, 0.0))

    def test_loaded_machine_reset_is_rejected(self):
        self.node.workpiece_present = True
        self.assertFalse(self.node._reset_cb(ns(), ns()).success)
        self.assertTrue(self.node.workpiece_present)

    def test_missing_part_does_not_finish_machining(self):
        self.node.state = MachineStatus.MACHINING
        self.node.clock.advance(100)
        self.node._tick()
        self.assertEqual(self.node.state, MachineStatus.READY)
        self.assertEqual(self.node.progress, 0.0)

    def test_zero_machining_duration_is_rejected(self):
        Node.overrides = {'machining_sec': 0.0}
        with self.assertRaises(ValueError):
            MachineSimulator()


class GeometryTests(unittest.TestCase):
    def test_scene_box_has_one_world_transform(self):
        node = SceneManager.__new__(SceneManager)
        obj = node._box('box', (0.04,) * 3, (0.45, 0.17, 0.062))
        self.assertEqual(obj.pose.position.x, 0.0)
        self.assertEqual(obj.pose.orientation.w, 1.0)
        self.assertEqual(obj.primitive_poses[0].position.x, 0.45)

    def test_attached_and_detached_objects_do_not_double_offsets(self):
        node = ArmController.__new__(ArmController)
        scenes = []
        node._apply_scene = lambda scene: scenes.append(scene) or True
        self.assertTrue(node._attach_workpiece('workpiece_1'))
        self.assertTrue(node._detach_workpiece('workpiece_1', (0.10, -0.25, 0.122)))
        attached = scenes[0].robot_state.attached_collision_objects[0].object
        detached = scenes[1].world.collision_objects[0]
        self.assertEqual(attached.pose.position.z, 0.0)
        self.assertEqual(attached.primitive_poses[0].position.z, 0.10)
        self.assertEqual(detached.pose.position.z, 0.0)
        self.assertEqual(detached.primitive_poses[0].position.z, 0.122)
        for scene in scenes:
            color = scene.object_colors[0]
            self.assertEqual(color.id, 'workpiece_1')
            self.assertEqual((color.color.r, color.color.g, color.color.b, color.color.a),
                             layout.OBJECT_COLORS['workpiece_1'])

    def test_initial_scene_assigns_a_distinct_color_to_every_object(self):
        node = SceneManager.__new__(SceneManager)
        scenes = []
        node.get_logger = lambda: ns(info=lambda message: None)
        node._apply = lambda scene: scenes.append(scene) or True
        node._build_scene()
        scene = scenes[0]
        self.assertEqual({obj.id for obj in scene.world.collision_objects},
                         {color.id for color in scene.object_colors})
        self.assertEqual(len(scene.object_colors), 6)
        rgba_values = {(color.color.r, color.color.g, color.color.b, color.color.a)
                       for color in scene.object_colors}
        self.assertEqual(len(rgba_values), 6)

    def test_scene_initialization_spins_service_response(self):
        node = SceneManager.__new__(SceneManager)
        node.scene_client = Client()
        node.get_logger = lambda: ns(error=lambda message: None)
        with patch('rclpy.spin_until_future_complete', side_effect=lambda node, future, timeout_sec: future.resolve()) as spin:
            self.assertTrue(node._apply(ns()))
        spin.assert_called_once()

    def test_pick_opens_gripper_before_approaching(self):
        node = ArmController.__new__(ArmController)
        node._attached_name = node._machine_workpiece = None
        commands = []
        node._status = lambda message: None
        node._gripper = lambda open_: commands.append(('gripper', open_)) or True
        node._goto_pose = lambda pose, name: commands.append(('motion', name)) or False
        self.assertFalse(node._pick(0))
        self.assertEqual(commands[0], ('gripper', True))


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        Node.overrides = {'output_dir': self.directory.name}
        self.node = ResultRecorder()

    def tearDown(self):
        self.node.destroy_node()
        self.directory.cleanup()

    def cycle(self, slot):
        for state in (WAIT_STATION, PICK, PLACE, WAIT_MACHINING, RETRIEVE, RETURN, DONE):
            self.node._task_cb(TaskState(state, slot, 'stage'))
            self.node.clock.advance(1)

    def test_two_cycles_have_distinct_logs_and_durations(self):
        self.cycle(0)
        self.node.clock.advance(1000)
        self.cycle(2)
        logs = sorted(Path(self.directory.name).glob('task_log_*.csv'))
        self.assertEqual(len(logs), 2)
        observed_slots = set()
        for path in logs:
            with open(path, encoding='utf-8', newline='') as stream:
                slot = int(next(csv.DictReader(stream))['slot'])
            observed_slots.add(slot)
            self.assertEqual(log_errors(path, slot), [])
        self.assertEqual(observed_slots, {0, 2})
        for path in Path(self.directory.name).glob('summary_*.txt'):
            self.assertIn('本轮任务时长: 6.0 秒', path.read_text(encoding='utf-8'))

    def test_logs_are_flushed_before_completion(self):
        self.node._task_cb(TaskState(PICK, 1))
        path = next(Path(self.directory.name).glob('*.csv'))
        self.assertIn('PICK', path.read_text(encoding='utf-8'))
        self.assertFalse(list(Path(self.directory.name).glob('summary_*.txt')))

    def test_duplicate_terminal_messages_do_not_create_extra_logs(self):
        self.cycle(0)
        self.node._task_cb(TaskState(DONE))
        self.node._task_cb(TaskState(IDLE))
        self.assertEqual(len(list(Path(self.directory.name).glob('*.csv'))), 1)

    def test_shutdown_preserves_incomplete_evidence(self):
        self.node._task_cb(TaskState(PICK, 0))
        self.node.destroy_node()
        summary = next(Path(self.directory.name).glob('summary_*.txt')).read_text(encoding='utf-8')
        self.assertIn('结束状态: INTERRUPTED', summary)
        self.assertNotIn('A2 阶段验收: 通过', summary)

    def test_ready_and_machine_changes_are_recorded_without_flooding(self):
        self.node._task_cb(TaskState(PICK))
        for state in (MachineStatus.READY, MachineStatus.READY, MachineStatus.MACHINING):
            self.node._machine_cb(MachineStatus(state))
        with open(self.node._csv_path, encoding='utf-8', newline='') as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual([row['event'] for row in rows if row['source'] == 'machine'], ['READY', 'MACHINING'])


class EvidenceTests(unittest.TestCase):
    def test_done_alone_is_not_accepted(self):
        self.assertTrue(cycle_errors(['DONE']))

    def test_out_of_order_stages_are_not_accepted(self):
        self.assertTrue(cycle_errors(['PICK', 'PLACE', 'RETRIEVE', 'WAIT_MACHINING', 'RETURN', 'DONE']))

    def test_warning_and_repeat_wait_are_accepted(self):
        self.assertEqual(cycle_errors(['WAIT_STATION', 'PICK', 'PLACE', 'WAIT_MACHINING', 'TIMEOUT',
                                       'WAIT_MACHINING', 'RETRIEVE', 'RETURN', 'DONE']), [])


if __name__ == '__main__':
    unittest.main()
