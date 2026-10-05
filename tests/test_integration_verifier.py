import csv
import itertools
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from ros_doubles import Node, install
install()
from machining_demo.demo_test import DemoTest
from machining_demo.evidence import REQUIRED_STAGES


class IntegrationVerifierTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        Node.overrides = {'output_dir': self.directory.name, 'slot': 1}
        self.node = DemoTest()

    def tearDown(self):
        self.directory.cleanup()

    def make_log(self, stamp, stages=REQUIRED_STAGES, slot=1, final='DONE'):
        root = Path(self.directory.name)
        with open(root / ('task_log_%s.csv' % stamp), 'w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time', 'source', 'event', 'detail', 'slot'])
            for stage in stages:
                writer.writerow(['12:00:00', 'task', stage, 'stage', slot])
        (root / ('summary_%s.txt' % stamp)).write_text(
            '结束状态: %s\n槽位: %d\n' % (final, slot), encoding='utf-8')

    def run_verifier(self, new_log=None, transitions=REQUIRED_STAGES):
        def service_response(node, future, timeout_sec):
            future.resolve()
            self.node.transitions = list(transitions)
            if new_log:
                new_log()
        with patch('rclpy.spin_once'), \
             patch('rclpy.spin_until_future_complete', side_effect=service_response), \
             patch('time.monotonic', side_effect=itertools.count()):
            return self.node.run()

    def test_historical_done_log_cannot_pass_current_task(self):
        self.make_log('historical')
        self.assertEqual(self.run_verifier(), 1)

    def test_new_complete_matching_log_passes(self):
        self.assertEqual(self.run_verifier(lambda: self.make_log('current')), 0)

    def test_new_done_only_log_is_rejected(self):
        self.assertEqual(self.run_verifier(lambda: self.make_log('current', stages=['DONE'])), 1)

    def test_new_log_for_another_slot_is_rejected(self):
        self.assertEqual(self.run_verifier(lambda: self.make_log('current', slot=2)), 1)

    def test_warning_must_be_observed_when_requested(self):
        self.node.params['require_timeout'] = True
        self.assertEqual(self.run_verifier(lambda: self.make_log('current')), 1)

    def test_expected_abort_requires_abort_in_both_log_and_summary(self):
        self.node.params['expected_final'] = 'ABORTED'
        self.assertEqual(self.run_verifier(lambda: self.make_log('current', final='ABORTED'),
                                          transitions=['WAIT_STATION', 'ABORTED']), 1)
        self.assertEqual(self.run_verifier(lambda: self.make_log('next', stages=['WAIT_STATION', 'ABORTED'],
                                                                final='ABORTED'),
                                          transitions=['WAIT_STATION', 'ABORTED']), 0)


if __name__ == '__main__':
    unittest.main()
