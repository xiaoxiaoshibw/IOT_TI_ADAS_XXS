#!/usr/bin/env python3
"""Own and verify one local PC/Orin/MCU HIL process topology."""

import argparse
import fcntl
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
    'global_planner_node', 'trajectory_planner_node', '/adas_gui/adas_gui',
    'CarlaUE4', 'adas_carla_bridge/lib/adas_carla_bridge/bridge_node')


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
    parser.add_argument('--carla', action='store_true',
                        help='run a real CARLA server and CARLA PC bridge')
    parser.add_argument('--carla-root', default=os.environ.get('CARLA_ROOT', ''))
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
            'simulation': ('carla' if getattr(self.args, 'carla', False)
                           else 'software_vehicle'),
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

    def start(self, role, command, extra_env=None, cwd=ROOT, remove_env=()):
        log_path = self.log_dir / ('%s.log' % role)
        stream = open(log_path, 'w', encoding='utf-8')
        env = os.environ.copy()
        for name in remove_env:
            env.pop(name, None)
        if extra_env:
            env.update(extra_env)
        process = subprocess.Popen(command, cwd=cwd, env=env, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        self.processes[role] = process
        self.logs[role] = stream
        report = self.read_report()
        report.setdefault('processes', {})[role] = {
            'pid': process.pid, 'log': str(log_path), 'command': command,
            'cwd': str(cwd)}
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


def wait_gui_lock(process, timeout=5.0):
    path = '/tmp/adas_gui.lock'
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and process.poll() is None:
        fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
        try:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                fcntl.flock(fd, fcntl.LOCK_UN)
            except BlockingIOError:
                return True
        finally:
            os.close(fd)
        time.sleep(0.02)
    return False


def find_carla_root(explicit):
    candidates = [Path(explicit)] if explicit else []
    candidates += [Path.home() / 'CARLA_0.9.16', Path.home() / '程序' / 'CARLA_0.9.16']
    for candidate in candidates:
        if candidate and (candidate / 'CarlaUE4.sh').is_file():
            return candidate.resolve()
    return None


def wait_carla_ready(process, timeout=90.0):
    try:
        import carla
    except ImportError:
        return False, 'PythonAPI module carla is not importable'
    deadline = time.monotonic() + timeout
    last = 'not contacted'
    while time.monotonic() < deadline:
        if process.poll() is not None:
            return False, 'CARLA exited rc=%s' % process.returncode
        try:
            # A CARLA rpc client that timed out while UE was booting can retain
            # a stale connection. Use a fresh client for each readiness probe.
            client = carla.Client('127.0.0.1', 2000)
            client.set_timeout(3.0)
            world = client.get_world()
            return True, '%s frame=%d' % (world.get_map().name,
                                          world.get_snapshot().frame)
        except RuntimeError as error:
            last = str(error)
        time.sleep(0.25)
    return False, 'CARLA readiness timeout: ' + last


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
        if args.carla:
            carla_root = find_carla_root(args.carla_root)
            if carla_root is None:
                raise RuntimeError(
                    'CARLA 0.9.16 not found; set CARLA_ROOT to its installation directory')
            carla_command = [str(carla_root / 'CarlaUE4.sh'), '-nosound',
                             '-quality-level=Low', '-carla-port=2000']
            # A normal --gui run keeps the native CARLA window visible. CI and
            # no-GUI runs still execute full CARLA physics/rendering offscreen.
            if not args.gui:
                carla_command.append('-RenderOffScreen')
            carla_process = run.start(
                'carla', carla_command, cwd=carla_root,
                remove_env=('LD_LIBRARY_PATH', 'PYTHONPATH', 'QT_PLUGIN_PATH',
                            'QML2_IMPORT_PATH'))
            carla_ready, carla_detail = wait_carla_ready(carla_process)
            run.add_check('CARLA server', carla_ready, carla_detail)
            if not carla_ready:
                raise RuntimeError(carla_detail)

            bridge_binary = (ROOT / 'adas_bridge_pc/carla_ros2_bridge/ws/install' /
                             'adas_carla_bridge/lib/adas_carla_bridge/bridge_node')
            if not bridge_binary.is_file():
                raise RuntimeError('CARLA bridge is not built; rerun with --build')
            scenario = {'baseline': 'lka', 'acc': 'acc', 'aeb': 'aeb',
                        'overtake': 'overtake'}[args.scenario]
            pc_command = [
                str(bridge_binary), '--scenario', scenario,
                '--control-source', 'can', '--can-transport', 'socketcan',
                '--can-interface', 'vcan0', '--carla-host', '127.0.0.1',
                '--carla-port', '2000', '--town', 'Town04', '--duration', '0',
                '--log-dir', str(run.log_dir / 'carla_csv')]
            pc = run.start('pc', pc_command)
            orin_command = [
                'ros2', 'launch', 'adas_launch',
                'local_three_machine_carla.launch.py']
        else:
            pc_command = [
                sys.executable,
                str(ROOT / 'scripts/local_three_machine/pc_companion.py'),
                '--interface', 'vcan0']
            pc = run.start('pc', pc_command)
            orin_command = [
                'ros2', 'launch', 'adas_launch', 'local_three_machine.launch.py']
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
            gui_env = {
                'ADAS_LOCAL_THREE_MACHINE': '1',
                'XDG_CONFIG_HOME': str(run.log_dir / 'gui_config'),
            }
            if args.carla:
                gui_env['CARLA_ROOT'] = str(carla_root)
                gui_env['ADAS_GUI_SCENARIO'] = scenario
                gui_env['ADAS_GUI_CONTROL_SOURCE'] = 'can'
            else:
                gui_env['ADAS_GUI_MODE'] = 'sil'
            if args.gui_offscreen:
                gui_env['QT_QPA_PLATFORM'] = 'offscreen'
                gui_command += ['--screenshot', str(run.log_dir / 'gui.png'),
                                '--screenshot-delay-ms',
                                '8000' if args.carla else '1500']
            run.start('gui', gui_command, gui_env)

        duplicate_gui_rejected = True
        if gui_expected and run.alive('gui'):
            if not wait_gui_lock(run.processes['gui']):
                raise RuntimeError('GUI did not acquire its single-instance lock')
            duplicate = subprocess.run(
                [str(gui_binary), '--screenshot', str(run.log_dir / 'duplicate.png')],
                cwd=ROOT, env={**os.environ, 'QT_QPA_PLATFORM': 'offscreen',
                               'ADAS_GUI_MODE': 'sil'},
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                timeout=5.0)
            duplicate_gui_rejected = duplicate.returncode == 2
            (run.log_dir / 'gui_duplicate.log').write_text(
                duplicate.stdout, encoding='utf-8')
        managed_roles = ['mcu', 'pc', 'orin'] + (['carla'] if args.carla else [])
        process_ok = run.wait_alive(managed_roles, timeout=2.0)
        run.add_check('PC process', run.alive('pc'), 'pid=%d' % pc.pid)
        run.add_check('MCU host runner', run.alive('mcu'), 'pid=%d' % mcu.pid)
        if not process_ok:
            raise RuntimeError('a managed role exited during startup')

        if args.gui_offscreen:
            try:
                run.processes['gui'].wait(timeout=20.0)
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
                '--mcu-pid', str(mcu.pid), '--scenario', args.scenario]
            if args.carla:
                command.append('--carla')
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
                pc_recovery = run.start('pc_recovery', pc_command)
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
                if not all(run.alive(role) for role in managed_roles):
                    rc = 1
                    break
                time.sleep(0.2)
        elif not args.check or args.keep_running:
            print('local three-machine HIL running; Ctrl-C to stop', flush=True)
            while all(run.alive(role) for role in managed_roles):
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
