#!/usr/bin/env python3
"""Scoped Linux shutdown of this workspace's A2 demo; no ROS dependency."""
import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import re
import signal
import sys
import time

PROJECT_NODES = frozenset(('scene_manager', 'arm_controller', 'machine_simulator',
    'task_manager', 'result_recorder', 'virtual_camera', 'vision_locator'))
ROS_NODES = PROJECT_NODES | frozenset(('move_group', 'ros2_control_node',
    'robot_state_publisher', 'static_transform_publisher', 'rviz2', 'spawner', 'xacro'))


class SelectionError(RuntimeError):
    pass


@dataclass(frozen=True)
class Process:
    pid: int
    uid: int
    pgid: int
    sid: int
    start_ticks: int
    state: str
    argv: tuple
    cwd: str
    workspace_tag: str = ''
    instance_tag: str = ''


def normalized(value):
    return os.path.normcase(os.path.realpath(value))


def role(process):
    # Only executable/interpreter-script positions, not arbitrary arguments.
    if not process.argv:
        return None
    executable = Path(process.argv[0]).name
    names = [executable]
    if re.fullmatch(r'python(?:3(?:\.\d+)?)?', executable) and len(process.argv) > 1:
        names.append(Path(process.argv[1]).name)
    for name in names:
        if name in ROS_NODES:
            return name
    if ('ros2' in names and 'launch' in process.argv and
            'machining_demo' in process.argv and 'demo.launch.py' in process.argv):
        return 'ros2_launch'
    return None


def project_anchor(process, workspace):
    node_role = role(process)
    if node_role == 'ros2_launch':
        return bool(process.cwd) and normalized(process.cwd) == workspace
    if node_role not in PROJECT_NODES:
        return False
    directory = Path(workspace) / 'install' / 'machining_demo' / 'lib' / 'machining_demo'
    return any(Path(arg).is_absolute() and Path(arg).name == node_role and
               normalized(str(Path(arg).parent)) == normalized(str(directory))
               for arg in process.argv[:2])


def same_identity(expected, actual):
    return actual is not None and all(getattr(expected, key) == getattr(actual, key)
        for key in ('pid', 'uid', 'pgid', 'sid', 'start_ticks'))


def select_targets(processes, workspace, uid, own_sid, instance=None, legacy_pid=None):
    workspace = normalized(workspace)
    live = [p for p in processes if p.state not in ('Z', 'X')]
    sessions = set()
    for p in live:
        if p.uid != uid or p.sid == own_sid:
            continue
        tagged = bool(p.workspace_tag) and normalized(p.workspace_tag) == workspace
        if tagged and (instance is None or p.instance_tag == instance):
            if not re.fullmatch(r'[0-9a-f]{32}', p.instance_tag) or p.sid != p.pgid:
                raise SelectionError('项目标签不完整或不是独立会话：PID %d，未自动结束' % p.pid)
            sessions.add(p.sid)
        if instance is None and p.sid == p.pgid and project_anchor(p, workspace):
            sessions.add(p.sid)
    if instance is None and legacy_pid is not None:
        remaining = [p for p in live if p.sid == legacy_pid and p.uid == uid and p.sid != own_sid]
        if remaining and legacy_pid not in sessions:
            raise SelectionError('保存的会话 %d 仍有进程，但无法确认属于本工程，未自动结束' % legacy_pid)
    return session_targets(live, sessions, workspace, uid, instance)


def session_targets(live, sessions, workspace, uid, instance):
    targets = []
    for p in live:
        if p.sid not in sessions:
            continue
        if (p.uid != uid or p.pgid != p.sid or role(p) is None or
                (p.workspace_tag and normalized(p.workspace_tag) != workspace) or
                (instance is not None and p.instance_tag != instance)):
            raise SelectionError('会话 %d 含归属不明的进程 PID %d，拒绝清理和重启' % (p.sid, p.pid))
        targets.append(p)
    return targets


def read_process(pid, proc_root=Path('/proc')):
    folder = Path(proc_root) / str(pid)
    try:
        uid = folder.stat().st_uid
        stat = (folder / 'stat').read_text()
        # Field 3 follows comm's last ')'; comm can contain spaces/parentheses.
        fields = stat.rsplit(')', 1)[1].split()
        state, pgid, sid, started = fields[0], int(fields[2]), int(fields[3]), int(fields[19])
        if state in ('Z', 'X'):
            return Process(pid, uid, pgid, sid, started, state, (), '')
        argv = tuple(value.decode(errors='replace') for value in
                     (folder / 'cmdline').read_bytes().split(b'\0') if value)
        base = Process(pid, uid, pgid, sid, started, state, argv, '')
        # Do not inspect unrelated applications' environment.
        if role(base) is None:
            return base
        cwd = os.readlink(folder / 'cwd')
        tags = {}
        for item in (folder / 'environ').read_bytes().split(b'\0'):
            key, sep, value = item.partition(b'=')
            if sep and key in (b'A2_DEMO_WORKSPACE', b'A2_DEMO_INSTANCE'):
                tags[key] = value.decode(errors='replace')
        return Process(pid, uid, pgid, sid, started, state, argv, cwd,
                       tags.get(b'A2_DEMO_WORKSPACE', ''), tags.get(b'A2_DEMO_INSTANCE', ''))
    except (FileNotFoundError, ProcessLookupError):
        return None
    except (PermissionError, OSError, ValueError, IndexError) as exc:
        raise SelectionError('无法核对进程 %s 的身份，停止操作：%s' % (pid, exc)) from exc


def scan_processes():
    uid = os.getuid()
    records = []
    for folder in Path('/proc').iterdir():
        if not folder.name.isdecimal():
            continue
        try:
            if folder.stat().st_uid != uid:
                continue
        except FileNotFoundError:
            continue
        process = read_process(int(folder.name))
        if process is not None:
            records.append(process)
    return records


def signal_checked(expected, sig):
    """Pin PID with pidfd before checking birth time, avoiding reused-PID signals."""
    fd = None
    try:
        if hasattr(os, 'pidfd_open') and hasattr(signal, 'pidfd_send_signal'):
            fd = os.pidfd_open(expected.pid)
        else:
            raise SelectionError('当前 Linux/Python 不支持 pidfd，请使用 Ubuntu 22.04 的 python3')
        actual = read_process(expected.pid)
        if same_identity(expected, actual) and actual.state not in ('Z', 'X'):
            def entrypoint(p):
                interpreter = bool(p.argv) and re.fullmatch(r'python(?:3(?:\.\d+)?)?', Path(p.argv[0]).name)
                return p.argv[:2] if interpreter else p.argv[:1]
            if (role(actual) != role(expected) or entrypoint(actual) != entrypoint(expected) or
                    actual.workspace_tag != expected.workspace_tag or actual.instance_tag != expected.instance_tag):
                raise SelectionError('PID %d 的程序或工程标签已改变，未发送信号' % expected.pid)
            signal.pidfd_send_signal(fd, sig)
    except ProcessLookupError:
        pass
    finally:
        if fd is not None:
            os.close(fd)


def stop_workspace(workspace, instance=None, legacy_pid=None):
    uid, own_sid = os.getuid(), os.getsid(0)
    known = {}
    proven_sessions = {}

    def remaining():
        records = scan_processes()
        selected = select_targets(records, workspace, uid, own_sid, instance, legacy_pid)
        by_pid = {p.pid: p for p in records}
        for p in selected:
            if p.sid not in proven_sessions:
                leader = by_pid.get(p.sid)
                proven_sessions[p.sid] = leader.start_ticks if leader is not None else None
        # Keep checking the entire proven session even if its last anchor exits.
        # A newly observed leader with that numeric SID means identity is ambiguous.
        for sid, leader_birth in proven_sessions.items():
            leader = by_pid.get(sid)
            if leader is not None and leader.state not in ('Z', 'X') and leader.start_ticks != leader_birth:
                raise SelectionError('会话编号 %d 身份已改变，拒绝继续清理' % sid)
        selected = session_targets([p for p in records if p.state not in ('Z', 'X')],
                                    set(proven_sessions), normalized(workspace), uid, instance)
        # Retain verified identities after their last project-path anchor exits.
        for p in selected:
            known[(p.pid, p.start_ticks)] = p
        return [p for p in known.values() if same_identity(p, by_pid.get(p.pid))
                and by_pid[p.pid].state not in ('Z', 'X')]

    targets = remaining()
    legacy_pid = None  # PID file is only a discovery hint, not later authority.
    if not targets:
        print('本工程没有可确认的残留仿真进程。')
        return
    print('清理本工程进程：' + ', '.join('%d(%s)' % (p.pid, role(p)) for p in targets))
    for sig, seconds in ((signal.SIGINT, 8.0), (signal.SIGTERM, 4.0), (signal.SIGKILL, 2.0)):
        print('发送 %s，等待本工程节点退出。' % sig.name, flush=True)
        for p in targets:
            signal_checked(p, sig)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            targets = remaining()
            if not targets:
                print('本工程仿真节点已退出。')
                return
            time.sleep(0.2)
    raise SelectionError('仍有节点未退出，拒绝启动第二套：%s' % ','.join(str(p.pid) for p in targets))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('stop', 'status'))
    parser.add_argument('--workspace', required=True)
    parser.add_argument('--instance')
    args = parser.parse_args()
    if not sys.platform.startswith('linux'):
        parser.error('此工具仅用于 Ubuntu/Linux，不会在 Windows 上发送信号。')
    try:
        workspace = normalized(args.workspace)
        pid_file = Path(workspace) / '.demo.pid'
        saved = pid_file.read_text().strip() if pid_file.exists() else ''
        legacy_pid = int(saved) if saved.isdecimal() and args.instance is None else None
        if args.action == 'status':
            targets = select_targets(scan_processes(), workspace, os.getuid(), os.getsid(0),
                                     args.instance, legacy_pid)
            for p in targets:
                print('PID=%d SID=%d start=%d node=%s' % (p.pid, p.sid, p.start_ticks, role(p)))
            if not targets:
                print('本工程没有可确认的残留仿真进程。')
        else:
            stop_workspace(workspace, args.instance, legacy_pid)
    except (SelectionError, PermissionError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
