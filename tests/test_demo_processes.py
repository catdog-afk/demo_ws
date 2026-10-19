"""Process selection safety without ROS, /proc access, or sending signals."""
from dataclasses import FrozenInstanceError, replace
from contextlib import redirect_stdout
from enum import IntEnum
import importlib.util
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch


WORKSPACE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'demo_processes_under_test', WORKSPACE / 'scripts' / 'demo_processes.py')
PROCESS_MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = PROCESS_MODULE
SPEC.loader.exec_module(PROCESS_MODULE)
Process = PROCESS_MODULE.Process
SelectionError = PROCESS_MODULE.SelectionError
select_targets = PROCESS_MODULE.select_targets
same_identity = PROCESS_MODULE.same_identity

UID = 1000
OWN_SID = 9000
INSTANCE_A = 'a' * 32
INSTANCE_B = 'b' * 32


def process(pid=401, sid=400, name='move_group', tagged=True, **changes):
    values = dict(
        pid=pid, uid=UID, pgid=sid, sid=sid, start_ticks=10000 + pid,
        state='S', argv=('/opt/ros/humble/lib/example/' + name,),
        cwd=str(WORKSPACE), workspace_tag=str(WORKSPACE) if tagged else '',
        instance_tag=INSTANCE_A if tagged else '')
    values.update(changes)
    return Process(**values)


def project_node(pid=402, sid=400, name='arm_controller', tagged=False, **changes):
    executable = WORKSPACE / 'install' / 'machining_demo' / 'lib' / 'machining_demo' / name
    changes.setdefault('argv', ('/usr/bin/python3', str(executable)))
    return process(pid=pid, sid=sid, tagged=tagged, **changes)


class ProcessSelectionTests(unittest.TestCase):
    def select(self, processes, **kwargs):
        return select_targets(processes, str(WORKSPACE), UID, OWN_SID, **kwargs)

    def test_tagged_orphans_selected_after_launch_leader_is_gone(self):
        nodes = [process(pid=401), project_node(pid=402, tagged=True),
                 process(pid=403, name='ros2_control_node')]
        self.assertNotIn(400, [node.pid for node in nodes])
        self.assertEqual({p.pid for p in self.select(nodes)}, {401, 402, 403})

    def test_legacy_orphans_selected_from_project_executable_anchor(self):
        nodes = [process(tagged=False), project_node(),
                 process(pid=403, name='rviz2', tagged=False)]
        self.assertEqual({p.pid for p in self.select(nodes, legacy_pid=400)},
                         {401, 402, 403})

    def test_all_seven_project_nodes_are_valid_legacy_anchors(self):
        for name in ('scene_manager', 'arm_controller', 'machine_simulator',
                     'task_manager', 'result_recorder', 'virtual_camera', 'vision_locator'):
            with self.subTest(name=name):
                node = project_node(name=name)
                self.assertEqual(self.select([node]), [node])

    def test_legacy_launch_in_workspace_anchors_dedicated_session(self):
        for flags in ((), ('--noninteractive',)):
            with self.subTest(flags=flags):
                launcher = process(pid=400, tagged=False, argv=(
                    '/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch',
                    *flags, 'machining_demo', 'demo.launch.py'))
                child = process(tagged=False)
                self.assertEqual({p.pid for p in self.select([launcher, child])},
                                 {400, 401})

    def test_legacy_launch_from_other_cwd_is_not_ownership_proof(self):
        launcher = process(pid=400, tagged=False,
                           cwd=str(WORKSPACE.parent / 'other_project'), argv=(
                               '/opt/ros/humble/bin/ros2', 'launch',
                               'machining_demo', 'demo.launch.py'))
        self.assertEqual(self.select([launcher, process(tagged=False)]), [])

    def test_untagged_global_ros_processes_are_not_selected_by_name(self):
        self.assertEqual(self.select([
            process(tagged=False), process(pid=402, tagged=False, name='rviz2')]), [])

    def test_same_named_nodes_from_another_workspace_are_untouched(self):
        foreign = str(WORKSPACE.parent / 'another_demo_ws')
        nodes = [process(workspace_tag=foreign),
                 process(pid=402, workspace_tag=foreign, name='ros2_control_node')]
        self.assertEqual(self.select(nodes), [])

    def test_project_path_prefix_does_not_match_another_workspace(self):
        foreign = Path(str(WORKSPACE) + '_other')
        node = project_node(argv=(str(foreign / 'install' / 'machining_demo' /
                                    'lib' / 'machining_demo' / 'arm_controller'),))
        self.assertEqual(self.select([node]), [])

    def test_only_current_uid_is_selected(self):
        foreign_user = process(uid=UID + 1)
        self.assertEqual(self.select([foreign_user]), [])

    def test_workspace_tag_without_valid_instance_blocks_restart(self):
        for invalid in ('', 'a' * 31, 'a' * 33, 'g' * 32, '../other'):
            with self.subTest(instance=invalid), self.assertRaises(SelectionError):
                self.select([process(instance_tag=invalid)])

    def test_tagged_process_outside_dedicated_group_blocks_restart(self):
        with self.assertRaises(SelectionError):
            self.select([process(pgid=499)])

    def test_own_session_is_never_selected(self):
        own = process(sid=OWN_SID)
        self.assertEqual(self.select([own]), [])

    def test_unknown_live_member_blocks_entire_owned_session(self):
        unknown = process(pid=402, name='bash', tagged=False)
        with self.assertRaises(SelectionError):
            self.select([process(), unknown])

    def test_arbitrary_tool_argument_is_not_a_ros_executable(self):
        unknown = process(pid=402, tagged=False,
                          argv=('/usr/bin/cat', '/opt/ros/humble/lib/example/move_group'))
        with self.assertRaises(SelectionError):
            self.select([process(), unknown])

    def test_arbitrary_tool_argument_cannot_establish_legacy_ownership(self):
        node_path = WORKSPACE / 'install' / 'machining_demo' / 'lib' / 'machining_demo' / 'arm_controller'
        unknown = process(tagged=False, argv=('/usr/bin/cat', str(node_path)))
        self.assertEqual(self.select([unknown]), [])

    def test_foreign_workspace_member_blocks_entire_owned_session(self):
        foreign = process(pid=402,
                          workspace_tag=str(WORKSPACE.parent / 'other_project'))
        with self.assertRaises(SelectionError):
            self.select([process(), foreign])

    def test_anchor_does_not_authorize_unknown_member_of_legacy_session(self):
        unknown = process(pid=403, name='unrelated_application', tagged=False)
        with self.assertRaises(SelectionError):
            self.select([project_node(), unknown])

    def test_zombies_and_dead_members_are_ignored(self):
        live = process()
        zombies = [process(pid=402, name='unrelated_application', state='Z'),
                   process(pid=403, name='bash', state='X',
                           workspace_tag=str(WORKSPACE.parent / 'other_project'))]
        self.assertEqual(self.select([live, *zombies]), [live])
        self.assertEqual(self.select(zombies), [])

    def test_cleanup_of_old_instance_does_not_select_new_instance(self):
        old = process()
        new = process(pid=601, sid=600, instance_tag=INSTANCE_B)
        self.assertEqual(self.select([old, new], instance=INSTANCE_A), [old])
        self.assertEqual(self.select([new], instance=INSTANCE_A), [])

    def test_instance_specific_cleanup_does_not_fall_back_to_legacy_session(self):
        old_legacy = project_node(pid=602, sid=600)
        current = process()
        self.assertEqual(self.select([current, old_legacy], instance=INSTANCE_A),
                         [current])

    def test_missing_legacy_anchor_blocks_restart_when_saved_session_survives(self):
        with self.assertRaises(SelectionError):
            self.select([process(tagged=False)], legacy_pid=400)

    def test_known_ros_roles_in_owned_session_are_selected(self):
        names = ('move_group', 'ros2_control_node', 'robot_state_publisher',
                 'static_transform_publisher', 'rviz2', 'spawner', 'xacro')
        nodes = [process(pid=401 + i, name=name) for i, name in enumerate(names)]
        self.assertEqual({p.pid for p in self.select(nodes)}, {p.pid for p in nodes})


class ProcessIdentityTests(unittest.TestCase):
    def test_pidfd_does_not_signal_a_reused_pid(self):
        from unittest.mock import patch
        expected = process()
        actual = replace(expected, start_ticks=expected.start_ticks+1)
        with patch.object(PROCESS_MODULE.os,'pidfd_open',return_value=90,create=True), \
                patch.object(PROCESS_MODULE.signal,'pidfd_send_signal',create=True) as send, \
                patch.object(PROCESS_MODULE,'read_process',return_value=actual), \
                patch.object(PROCESS_MODULE.os,'close') as close:
            PROCESS_MODULE.signal_checked(expected,PROCESS_MODULE.signal.SIGINT)
        send.assert_not_called()
        close.assert_called_once_with(90)

    def test_exec_change_is_rechecked_after_pidfd_open(self):
        from unittest.mock import patch
        expected = project_node(tagged=True)
        actual = replace(expected,argv=('/bin/bash',))
        with patch.object(PROCESS_MODULE.os,'pidfd_open',return_value=90,create=True), \
                patch.object(PROCESS_MODULE.signal,'pidfd_send_signal',create=True) as send, \
                patch.object(PROCESS_MODULE,'read_process',return_value=actual), \
                patch.object(PROCESS_MODULE.os,'close'):
            with self.assertRaises(SelectionError):
                PROCESS_MODULE.signal_checked(expected,PROCESS_MODULE.signal.SIGINT)
        send.assert_not_called()

    def test_process_record_is_immutable(self):
        node = process()
        with self.assertRaises(FrozenInstanceError):
            node.pid = 500

    def test_pid_reuse_is_not_the_same_identity(self):
        expected = process()
        reused = replace(expected, start_ticks=expected.start_ticks + 1)
        self.assertFalse(same_identity(expected, reused))

    def test_all_identity_fields_are_revalidated(self):
        expected = process()
        for field in ('pid', 'uid', 'pgid', 'sid', 'start_ticks'):
            with self.subTest(field=field):
                actual = replace(expected, **{field: getattr(expected, field) + 1})
                self.assertFalse(same_identity(expected, actual))

    def test_status_changes_do_not_change_process_identity(self):
        expected = process()
        self.assertTrue(same_identity(expected, replace(expected, state='R')))


class StopWorkspaceTests(unittest.TestCase):
    def run_stop(self, scan, send):
        clock = [0.0]

        class Signals(IntEnum):
            SIGINT = 2
            SIGTERM = 15
            SIGKILL = 9

        def sleep(seconds):
            clock[0] += seconds

        with patch.object(PROCESS_MODULE.os, 'getuid', return_value=UID, create=True), \
                patch.object(PROCESS_MODULE.os, 'getsid', return_value=OWN_SID, create=True), \
                patch.object(PROCESS_MODULE, 'signal', Signals), \
                patch.object(PROCESS_MODULE, 'scan_processes', side_effect=scan), \
                patch.object(PROCESS_MODULE, 'signal_checked', side_effect=send), \
                patch.object(PROCESS_MODULE.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(PROCESS_MODULE.time, 'sleep', side_effect=sleep), \
                redirect_stdout(io.StringIO()):
            PROCESS_MODULE.stop_workspace(str(WORKSPACE), legacy_pid=400)

    def test_legacy_child_born_during_shutdown_is_not_left_behind(self):
        anchor = project_node()
        first = process(tagged=False)
        late = process(pid=403, tagged=False, name='robot_state_publisher')
        scans = [0]
        signals = []

        def scan():
            scans[0] += 1
            if any(pid == late.pid for pid, _ in signals):
                return []
            if scans[0] == 1:
                return [anchor, first]
            if scans[0] == 2:
                return [first, late]
            return [late]

        self.run_stop(scan, lambda target, sig: signals.append((target.pid, sig)))
        self.assertIn(late.pid, [pid for pid, _ in signals],
                      'A late legacy child must be stopped before reporting an empty session.')

    def test_known_identity_cannot_bypass_current_ownership_validation(self):
        anchor = project_node()
        first = process(tagged=False)
        changes = ({'argv': ('/bin/bash',)},
                   {'workspace_tag': str(WORKSPACE.parent / 'other_project')})
        for change in changes:
            with self.subTest(change=change):
                changed = replace(first, **change)
                scans = [0]
                signals = []

                def scan():
                    scans[0] += 1
                    return [anchor, first] if scans[0] == 1 else [changed]

                with self.assertRaises(SelectionError):
                    self.run_stop(scan, lambda target, sig: signals.append((target.pid, sig)))
                self.assertEqual(len(signals), 2,
                                 'Only initial verified targets may receive SIGINT; '
                                 'changed ownership must stop further signals.')


if __name__ == '__main__':
    unittest.main()
