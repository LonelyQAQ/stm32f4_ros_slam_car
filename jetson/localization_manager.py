#!/usr/bin/env python3
"""AMCL + map_server 静态地图定位：启停与就绪等待

与 Cartographer 的分工：
  * 建图：Cartographer 发 map→odom；
  * 定位：本模块启 map_server + AMCL，AMCL 发 map→odom，地图不再增长；
  * odom→base_link 始终由 EKF 发，base_link→laser_frame 由雷达 launch 发。
"""
import os
import signal
import subprocess
import time

from slam_manager import stop_cartographer
from webui_state import S

AMCL_PARAMS = '/home/nvidia/amcl_tmini.yaml'
LOG = '/tmp/localization_web.log'
PATTERNS = ('localization_launch.py', 'lifecycle_manager_localization',
            'map_server', 'amcl')


def _pids(pattern):
    r = subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True)
    return [int(x) for x in r.stdout.split() if x.isdigit()]


def _all_stopped():
    return not any(_pids(pat) for pat in PATTERNS)


def stop_localization():
    """停 map_server/AMCL/lifecycle，并确认进程真正退出后再返回。"""
    with S.lock:
        p = getattr(S, 'loc_proc', None)
        S.loc_proc = None
    if p is not None and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except Exception:
            p.terminate()
    for pat in PATTERNS:
        for pid in _pids(pat):
            try:
                os.kill(pid, signal.SIGINT)
            except Exception:
                pass
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if _all_stopped():
            return True
        time.sleep(0.2)
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pat in PATTERNS:
            for pid in _pids(pat):
                try:
                    os.kill(pid, sig)
                except Exception:
                    pass
        time.sleep(0.8)
        if _all_stopped():
            return True
    return _all_stopped()


def start_localization(map_yaml):
    """启动 map_server + AMCL；返回 (proc, error)。"""
    if not os.path.isfile(map_yaml):
        return None, '找不到地图 yaml: %s' % map_yaml
    if not os.path.isfile(AMCL_PARAMS):
        return None, '找不到 AMCL 参数: %s' % AMCL_PARAMS
    if not stop_localization():
        return None, '无法清理旧的 map_server/AMCL 进程'
    stop_cartographer()
    logf = open(LOG, 'ab', buffering=0)
    cmd = ('source /opt/ros/humble/setup.bash && '
           'ros2 launch nav2_bringup localization_launch.py '
           f'map:={map_yaml} params_file:={AMCL_PARAMS} use_sim_time:=false')
    try:
        p = subprocess.Popen(['bash', '-lc', cmd], stdout=logf, stderr=logf,
                             start_new_session=True)
    finally:
        logf.close()
    with S.lock:
        S.loc_proc = p
    S.note('启动 AMCL/map_server (pid %d): %s' % (p.pid, map_yaml))
    return p, ''


_STATE_CLIENTS = {}
LIFECYCLE_ACTIVE = 3


def _state_id(node, name, timeout=1.0):
    """查 lifecycle 当前状态 id（走服务，别用 ros2 CLI，那个要 2 s 起）。"""
    from lifecycle_msgs.srv import GetState

    key = name + '/get_state'
    cli = _STATE_CLIENTS.get(key)
    if cli is None:
        cli = node.create_client(GetState, key)
        _STATE_CLIENTS[key] = cli
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    fut = cli.call_async(GetState.Request())
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < timeout:
        time.sleep(0.02)
    if not fut.done():
        return None
    try:
        return int(fut.result().current_state.id)
    except Exception:
        return None


def _proc_alive(pattern):
    return subprocess.run(['pgrep', '-f', pattern],
                          capture_output=True).returncode == 0


def is_active(node=None):
    """AMCL + map_server 生命周期都处于 active。"""
    if node is not None:
        a = _state_id(node, '/amcl')
        b = _state_id(node, '/map_server')
        if a is not None and b is not None:
            return a == LIFECYCLE_ACTIVE and b == LIFECYCLE_ACTIVE
    return (_proc_alive('nav2_amcl/amcl')
            and _proc_alive('nav2_map_server/map_server'))


def wait_ready(node, timeout=15.0):
    """等待当前定位会话的 map_server/AMCL active，且静态地图已到 webui。"""
    deadline = time.monotonic() + timeout
    next_check = 0.0
    active = False
    while time.monotonic() < deadline:
        with S.lock:
            localizing = getattr(S, 'localizing', False)
            have_map = S.map_meta is not None
        now = time.monotonic()
        if localizing and have_map:
            if now >= next_check:
                active = is_active(node)
                next_check = now + 0.5
            if active:
                return True
        time.sleep(0.2)
    return False
