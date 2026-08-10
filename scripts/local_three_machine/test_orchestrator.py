#!/usr/bin/env python3
"""Standard-library regression tests for owned process cleanup."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
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


if __name__ == '__main__':
    unittest.main()
