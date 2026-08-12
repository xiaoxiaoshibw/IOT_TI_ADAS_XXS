#!/usr/bin/env python3
"""Standard-library regression tests for owned process cleanup and run_id propagation."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid
from types import SimpleNamespace
import unittest
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import orchestrator


class CleanupTest(unittest.TestCase):
    def test_cleanup_stops_only_owned_process_group(self):
        args = SimpleNamespace(scenario='baseline')
        run = orchestrator.Run(args)
        temporary = tempfile.TemporaryDirectory(prefix='local_3m_cleanup_')
        self.addCleanup(temporary.cleanup)
        run.log_dir = Path(temporary.name)
        run.report_path = run.log_dir / 'report.json'
        run.write_initial_report()
        external = subprocess.Popen(['sleep', '60'], start_new_session=True)
        def stop_external():
            if external.poll() is None:
                os.killpg(external.pid, signal.SIGKILL)
            external.wait(timeout=1.0)
        self.addCleanup(stop_external)
        owned = run.start('owned_test', ['sleep', '60'])
        run.cleanup()
        self.assertIsNotNone(owned.poll())
        self.assertIsNone(external.poll())


class RunIdPropagationTest(unittest.TestCase):
    """P0.C: 编排器必须为整轮运行生成一次 UUID v4,日志目录名不能充当 run_id。"""

    def test_omitted_run_id_generates_uuid_v4(self):
        args = SimpleNamespace(scenario='baseline', run_id='')
        run = orchestrator.Run(args)
        self.assertTrue(orchestrator.is_canonical_uuid_v4(run.run_id))
        self.assertEqual(run.args.run_id, run.run_id)

    def test_explicit_run_id_is_validated_as_uuid_v4(self):
        accepted = str(uuid.uuid4())
        args = SimpleNamespace(scenario='baseline', run_id=accepted)
        run = orchestrator.Run(args)
        self.assertEqual(run.run_id, accepted)

    def test_invalid_explicit_run_id_is_rejected(self):
        for bad in ('not-a-uuid', 'aaaaaaaa-bbbb-1ccc-8ddd-eeeeeeeeeeee',
                    'AAAAAAAA-BBBB-4CCC-8DDD-EEEEEEEEEEEE'):
            args = SimpleNamespace(scenario='baseline', run_id=bad)
            with self.assertRaises(SystemExit):
                orchestrator.Run(args)
        # 空字符串则自动生成 UUID,不算拒绝路径。

    def test_initial_report_uses_propagated_run_id(self):
        rid = str(uuid.uuid4())
        args = SimpleNamespace(scenario='baseline', run_id=rid)
        run = orchestrator.Run(args)
        temporary = tempfile.TemporaryDirectory(prefix='local_3m_runid_')
        self.addCleanup(temporary.cleanup)
        run.log_dir = Path(temporary.name)
        run.report_path = run.log_dir / 'report.json'
        run.write_initial_report()
        import json
        report = json.loads(run.report_path.read_text(encoding='utf-8'))
        self.assertEqual(report['run_id'], rid)
        # 日志目录名不能出现在 report.run_id
        self.assertNotIn(run.log_dir.name, report['run_id'])

    def test_uuid_validator_accepts_canonical_form(self):
        for _ in range(8):
            self.assertTrue(orchestrator.is_canonical_uuid_v4(str(uuid.uuid4())))
        self.assertFalse(orchestrator.is_canonical_uuid_v4(
            str(uuid.uuid4()).upper()))


if __name__ == '__main__':
    unittest.main()
