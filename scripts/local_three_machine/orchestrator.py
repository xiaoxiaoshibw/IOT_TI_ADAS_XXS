#!/usr/bin/env python3
"""Own and verify one local PC/Orin/MCU HIL process topology."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
LOG_ROOT = ROOT / 'logs' / 'local_three_machine'
CRITICAL_PATTERNS = (
    'local_three_machine_pc', 'mcu_sil_runner.py', 'can_gateway_node',
    'global_planner_node', 'trajectory_planner_node', '/adas_gui/adas_gui')


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--gui-offscreen', action='store_true')
    parser.add_argument('--scenario', choices=['baseline', 'acc', 'aeb', 'overtake'],
                        default='baseline')
    parser.add_argument('--duration', type=int, default=0)
    parser.add_argument('--keep-running', action='store_true')
    parser.add_argument('--fault-test', action='store_true')
    parser.add_argument('--dangerous-fault-test', action='store_true')
    parser.add_argument('--navigation-test', action='store_true')
    return parser.parse_args()


def process_snapshot():
    result = subprocess.run(['ps', '-eo', 'pid=,args='], text=True,
                            stdout=subprocess.PIPE, check=True)
    ignored = {os.getpid()}
    parent = os.getppid()
    while parent > 1 and parent not in ignored:
        ignored.add(parent)
        try:
            parent = int(Path('/proc/%d/stat' % parent).read_text().split()[3])
        except (OSError, ValueError, IndexError):
            break
    matches = []
    for line in result.stdout.splitlines():
        fields = line.strip().split(maxsplit=1)
        if len(fields) != 2 or int(fields[0]) in ignored:
            continue
        if any(pattern in fields[1] for pattern in CRITICAL_PATTERNS):
            # Ignore the shell/editor command that happens to contain source text.
            if 'rg ' in fields[1] or 'apply_patch' in fields[1]:
                continue
            matches.append(line.strip())
    return matches


class Run:
    def __init__(self, args):
        self.args = args
        stamp = time.strftime('%Y%m%d_%H%M%S') + '_%d' % os.getpid()
        self.log_dir = LOG_ROOT / stamp
        self.log_dir.mkdir(parents=True)
        self.report_path = self.log_dir / 'report.json'
        self.processes = {}
        self.logs = {}
        self.cleaned = False

    def write_initial_report(self):
        report = {
            'schema_version': 1, 'overall': 'RUNNING',
            'run_id': self.log_dir.name, 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
            'can_interface': 'vcan0', 'scenario': self.args.scenario,
            'log_dir': str(self.log_dir), 'checks': [], 'processes': {},
        }
        self.write_report(report)

    def read_report(self):
        try:
            return json.loads(self.report_path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError):
            return {'schema_version': 1, 'checks': []}

    def write_report(self, report):
        temporary = self.report_path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n',
                             encoding='utf-8')
        temporary.replace(self.report_path)

    def add_check(self, name, ok, detail):
        report = self.read_report()
        report.setdefault('checks', []).append({
            'name': name, 'status': 'PASS' if ok else 'FAIL', 'detail': detail})
        report['overall'] = ('FAIL' if any(item['status'] == 'FAIL'
                                          for item in report['checks']) else 'PASS')
        self.write_report(report)
        print('[%s] %s%s' % ('PASS' if ok else 'FAIL', name,
                             '' if not detail else ': ' + detail), flush=True)

    def start(self, role, command, extra_env=None):
        log_path = self.log_dir / ('%s.log' % role)
        stream = open(log_path, 'w', encoding='utf-8')
        env = os.environ.copy()
        if extra_env:
            env.update(extra_env)
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        self.processes[role] = process
        self.logs[role] = stream
        report = self.read_report()
        report.setdefault('processes', {})[role] = {
            'pid': process.pid, 'log': str(log_path), 'command': command}
        self.write_report(report)
        return process

    def alive(self, role):
        process = self.processes.get(role)
        return process is not None and process.poll() is None

    def wait_alive(self, roles, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            failed = [role for role in roles if not self.alive(role)]
            if not failed:
                return True
            # A process that exited will never recover.
            if any(self.processes[role].poll() is not None for role in failed):
                return False
            time.sleep(0.1)
        return False

    def cleanup(self):
        if self.cleaned:
            return
        self.cleaned = True
        for process in self.processes.values():
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and any(
                process.poll() is None for process in self.processes.values()):
            time.sleep(0.05)
        for process in self.processes.values():
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                pass
        for stream in self.logs.values():
            stream.close()
        survivors = [role for role, process in self.processes.items()
                     if process.poll() is None]
        self.add_check('Cleanup', not survivors,
                       'owned process groups stopped' if not survivors else
                       'survivors=' + ','.join(survivors))

    def stop(self, role, timeout=5.0):
        process = self.processes[role]
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=1.0)

    def group_process(self, pgid, executable):
        result = subprocess.run(['ps', '-eo', 'pid=,pgid=,args='], text=True,
                                stdout=subprocess.PIPE, check=True)
        for line in result.stdout.splitlines():
            fields = line.strip().split(maxsplit=2)
            if len(fields) == 3 and int(fields[1]) == pgid and executable in fields[2]:
                return int(fields[0])
        return None


def interface_ready():
    result = subprocess.run(['ip', 'link', 'show', 'vcan0'],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return result.returncode == 0 and 'UP' in result.stdout.split('\n', 1)[0]


def main():
    args = parse_args()
    if args.duration < 0:
        print('--duration must be non-negative', file=sys.stderr)
        return 2
    if args.dangerous_fault_test:
        args.fault_test = True
    if args.check:
        args.fault_test = True if not args.fault_test else args.fault_test
        args.navigation_test = True if not args.navigation_test else args.navigation_test
    run = Run(args)
    run.write_initial_report()
    print('logs=%s' % run.log_dir, flush=True)

    if os.environ.get('ROS_DOMAIN_ID') != '145':
        run.add_check('Preflight', False, 'ROS_DOMAIN_ID must be explicitly 145')
        return 1
    conflicts = process_snapshot()
    if conflicts:
        run.add_check('Preflight', False, 'existing critical processes: ' + '; '.join(conflicts))
        return 1
    if not interface_ready():
        detail = ('vcan0 is absent/down. Run: sudo modprobe vcan && '
                  'sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up')
        run.add_check('Preflight', False, detail)
        print(detail, file=sys.stderr)
        return 1
    run.add_check('Preflight', True, 'vcan0 up; no conflicting managed processes')

    overlay = '' if args.scenario == 'baseline' else args.scenario + '_scenario.yaml'
    try:
        mcu = run.start('mcu', [
            sys.executable, str(ROOT / 'adas_mcu/tools/mcu_sil_runner.py'),
            '--interface', 'vcan0'])
        pc = run.start('pc', [
            sys.executable, str(ROOT / 'scripts/local_three_machine/pc_companion.py'),
            '--interface', 'vcan0'])
        orin_command = [
            'ros2', 'launch', 'adas_launch', 'local_three_machine.launch.py',
        ]
        if overlay:
            orin_command.append('scenario:=' + overlay)
        orin = run.start('orin', orin_command)
        gui_expected = args.gui or args.gui_offscreen
        if gui_expected:
            gui_binary = (ROOT / 'adas_bridge_pc/carla_ros2_bridge/ws/install' /
                          'adas_gui/lib/adas_gui/adas_gui')
            if not gui_binary.is_file():
                raise RuntimeError('GUI is not built; rerun with --build')
            # Bypass start_gui.sh: that production-HIL entry intentionally binds
            # CycloneDDS to the physical Orin link. Local HIL keeps the inherited
            # RMW default and only uses the explicitly scoped domain 145.
            gui_command = [str(gui_binary)]
            gui_env = {'ADAS_GUI_MODE': 'sil'}
            if args.gui_offscreen:
                gui_env['QT_QPA_PLATFORM'] = 'offscreen'
                gui_command += ['--screenshot', str(run.log_dir / 'gui.png')]
            run.start('gui', gui_command, gui_env)

        time.sleep(0.5)
        duplicate_gui_rejected = True
        if gui_expected and run.alive('gui'):
            duplicate = subprocess.run(
                [str(gui_binary), '--screenshot', str(run.log_dir / 'duplicate.png')],
                cwd=ROOT, env={**os.environ, 'QT_QPA_PLATFORM': 'offscreen',
                               'ADAS_GUI_MODE': 'sil'},
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                timeout=5.0)
            duplicate_gui_rejected = duplicate.returncode == 2
            (run.log_dir / 'gui_duplicate.log').write_text(
                duplicate.stdout, encoding='utf-8')
        process_ok = run.wait_alive(['mcu', 'pc', 'orin'], timeout=2.0)
        run.add_check('PC process', run.alive('pc'), 'pid=%d' % pc.pid)
        run.add_check('MCU host runner', run.alive('mcu'), 'pid=%d' % mcu.pid)
        if not process_ok:
            raise RuntimeError('a managed role exited during startup')

        if args.gui_offscreen:
            try:
                run.processes['gui'].wait(timeout=12.0)
            except subprocess.TimeoutExpired:
                raise RuntimeError('offscreen GUI screenshot timed out')
            screenshot = run.log_dir / 'gui.png'
            gui_ok = (run.processes['gui'].returncode == 0 and screenshot.exists() and
                      duplicate_gui_rejected)
            run.add_check('Single GUI instance', gui_ok,
                          'offscreen screenshot=%s gui_rc=%s duplicate_rc=%s' %
                          (screenshot, run.processes['gui'].returncode,
                           duplicate.returncode))
        elif args.gui:
            run.add_check('Single GUI instance',
                          run.alive('gui') and duplicate_gui_rejected,
                          'one owned GUI pid=%d' % run.processes['gui'].pid)
        else:
            run.add_check('Single GUI instance', True, 'GUI disabled; no external instance')

        rc = 0
        if args.check:
            command = [
                sys.executable, str(ROOT / 'scripts/local_three_machine/checker.py'),
                '--interface', 'vcan0', '--report', str(run.report_path),
                '--mcu-pid', str(mcu.pid)]
            if args.fault_test:
                command.append('--fault-test')
            if args.dangerous_fault_test:
                command.append('--dangerous-fault-test')
            if args.navigation_test:
                command.append('--navigation-test')
            with open(run.log_dir / 'check.log', 'w', encoding='utf-8') as check_log:
                check = subprocess.run(command, cwd=ROOT, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True)
                check_log.write(check.stdout)
                print(check.stdout, end='')
                rc = check.returncode
            if rc == 0:
                recovery_detail = []
                planner_pid = run.group_process(orin.pid, 'global_planner_node')
                planner_down = planner_pid is not None
                if planner_pid is not None:
                    os.kill(planner_pid, signal.SIGTERM)
                    deadline = time.monotonic() + 4.0
                    while time.monotonic() < deadline:
                        try:
                            os.kill(planner_pid, 0)
                        except ProcessLookupError:
                            break
                        time.sleep(0.05)
                    probe = subprocess.run([
                        sys.executable, str(ROOT / 'scripts/local_three_machine/probe.py'),
                        '--nav', 'unavailable', '--timeout', '1.5'],
                        cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                        text=True)
                    planner_down &= probe.returncode == 0
                    recovery_detail.append('planner_down=' + probe.stdout.strip())
                run.stop('orin')
                orin_recovery = run.start('orin_recovery', orin_command)
                probe = subprocess.run([
                    sys.executable, str(ROOT / 'scripts/local_three_machine/probe.py'),
                    '--nav', 'available', '--timeout', '30'], cwd=ROOT,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                planner_recovered = probe.returncode == 0 and orin_recovery.poll() is None
                recovery_detail.append('planner_recovered=' + probe.stdout.strip())

                run.stop('pc')
                probe = subprocess.run([
                    sys.executable, str(ROOT / 'scripts/local_three_machine/probe.py'),
                    '--fault', 'timeout', '--timeout', '1.5'], cwd=ROOT,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                pc_down = probe.returncode == 0
                recovery_detail.append('pc_down=' + probe.stdout.strip())
                pc_recovery = run.start('pc_recovery', [
                    sys.executable,
                    str(ROOT / 'scripts/local_three_machine/pc_companion.py'),
                    '--interface', 'vcan0'])
                probe = subprocess.run([
                    sys.executable, str(ROOT / 'scripts/local_three_machine/probe.py'),
                    '--fault', 'success', '--timeout', '5'], cwd=ROOT,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                pc_recovered = probe.returncode == 0 and pc_recovery.poll() is None
                recovery_detail.append('pc_recovered=' + probe.stdout.strip())
                recovery_ok = planner_down and planner_recovered and pc_down and pc_recovered
                run.add_check('Timeout recovery', recovery_ok, '; '.join(recovery_detail))
                if not recovery_ok:
                    rc = 1

            # checker owns the canonical checks; preserve process metadata around it.
            report = run.read_report()
            report['processes'] = {
                role: {'pid': process.pid, 'log': str(run.log_dir / (role + '.log'))}
                for role, process in run.processes.items()}
            report['run_id'] = run.log_dir.name
            report['ros_domain_id'] = '145'
            report['can_interface'] = 'vcan0'
            report['scenario'] = args.scenario
            report['log_dir'] = str(run.log_dir)
            run.write_report(report)

        if args.duration:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                if not all(run.alive(role) for role in ('mcu', 'pc', 'orin')):
                    rc = 1
                    break
                time.sleep(0.2)
        elif not args.check or args.keep_running:
            print('local three-machine HIL running; Ctrl-C to stop', flush=True)
            while all(run.alive(role) for role in ('mcu', 'pc', 'orin')):
                time.sleep(0.5)
            rc = 1
        return rc
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        run.add_check('Runtime', False, str(error))
        return 1
    finally:
        # --keep-running means stay in the foreground after checks; it never
        # transfers ownership. Ctrl-C, timeout, launch failure and normal exit
        # must all tear down exactly the process groups created by this run.
        run.cleanup()


if __name__ == '__main__':
    raise SystemExit(main())
