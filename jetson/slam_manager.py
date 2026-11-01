#!/usr/bin/env python3
"""Cartographer 建图管理：启停 / 结束本图并入库（finish_trajectory + write_state + 降噪）

与 slam_manager 旧版的差别：建图引擎从 slam_toolbox 换成 cartographer，
"暂停/恢复"没有对应服务，改成"结束本图"（入地图库）。
"""
import os
import signal
import subprocess
import time

import numpy as np

import map_tools
from webui_state import S

LAUNCH = os.path.expanduser('~/cartographer_tmini.launch.py')
MAPPING_LUA = 'cartographer_tmini.lua'
CARTO_PATTERNS = ('cartographer_tmini.launch.py',
                  'cartographer_occupancy_grid_node',
                  'cartographer_node')


def _alive():
    with S.lock:
        p = S.proc
    return p is not None and p.poll() is None


def _spawn(extra):
    logf = open('/tmp/carto_web.log', 'ab', buffering=0)
    cmd = ('source /opt/ros/humble/setup.bash && '
           'source ~/ydlidar_ws/install/setup.bash && '
           'ros2 launch %s %s' % (LAUNCH, extra))
    p = subprocess.Popen(['bash', '-lc', cmd], stdout=logf, stderr=logf,
                         start_new_session=True)
    with S.lock:
        S.proc = p
        S.paused = False
    return p


def slam_start():
    """新建一张图（建图模式）"""
    if _alive():
        return 'Cartographer 已在运行，先「结束本图」再新建'
    p = _spawn('config:=%s' % MAPPING_LUA)
    S.note('启动 Cartographer 建图 (pid %d)' % p.pid)
    return '正在启动 Cartographer（约 5~8 s）'


def _kill():
    """停掉 Cartographer 的 launch、cartographer_node 和所有栅格节点。

    旧实现只 pkill cartographer_node，会留下 launch 和 occupancy_grid_node，
    导致 /map 有两个发布者、地图看起来“又长了一张”。
    """
    with S.lock:
        p = S.proc
        S.proc = None
    killed = False
    if p is not None and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except Exception:
            p.terminate()
        killed = True
    for pat in CARTO_PATTERNS:
        r = subprocess.run(['pkill', '-INT', '-f', pat], capture_output=True)
        killed = killed or r.returncode == 0
    time.sleep(0.8)
    for pat in CARTO_PATTERNS:
        r = subprocess.run(['pgrep', '-f', pat], capture_output=True)
        if r.returncode == 0:
            subprocess.run(['pkill', '-TERM', '-f', pat], capture_output=True)
    time.sleep(0.5)
    for pat in CARTO_PATTERNS:
        r = subprocess.run(['pgrep', '-f', pat], capture_output=True)
        if r.returncode == 0:
            subprocess.run(['pkill', '-KILL', '-f', pat], capture_output=True)
    return killed


def stop_cartographer():
    return _kill()


def slam_stop():
    if _kill():
        S.note('已停止 Cartographer')
        return '已停止 Cartographer（本图未保存）'
    return 'Cartographer 未在运行'


# --------------------------------------------------------------------------
# 结束本图：finish_trajectory → write_state → 导出 pgm → 降噪 → meta
# --------------------------------------------------------------------------
def _call(node, client, req, timeout=8.0):
    if not client.service_is_ready():
        if not client.wait_for_service(timeout_sec=3.0):
            return None, '服务不可用：slam_toolbox/cartographer 没在跑？'
    fut = client.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        time.sleep(0.02)
    if not fut.done():
        return None, '服务超时'
    return fut.result(), None


def finish_map(node):
    """结束本图并入库。返回 (map_id, 说明) 或抛异常。"""
    from cartographer_ros_msgs.srv import FinishTrajectory, WriteState

    grid_msg = node.last_map_msg()
    if grid_msg is None:
        return None, '还没有地图数据（Cartographer 起了吗？）'

    map_id = map_tools.new_map_id()
    d = map_tools.map_dir(map_id)
    os.makedirs(d, exist_ok=True)

    # 1) 结束轨迹 + 写 pbstream（失败不阻断，pgm 仍可保存）
    pbstream = ''
    try:
        res, err = _call(node, node.cli_finish, FinishTrajectory.Request(trajectory_id=0))
        if err:
            S.note('finish_trajectory 失败: %s' % err)
        res, err = _call(node, node.cli_write_state,
                         WriteState.Request(filename=os.path.join(d, 'map.pbstream')))
        if err:
            S.note('write_state 失败: %s' % err)
        else:
            pbstream = os.path.join(d, 'map.pbstream')
    except Exception as e:
        S.note('Cartographer 服务调用异常: %s' % e)

    # 2) 导出原始栅格
    res_m = grid_msg.info.resolution
    ox = grid_msg.info.origin.position.x
    oy = grid_msg.info.origin.position.y
    raw = map_tools.occupancy_to_pgm(grid_msg)
    map_tools.save_pgm(os.path.join(d, 'raw.pgm'), raw, res_m, [ox, oy])

    # 3) 降噪
    clean, stats = map_tools.morph_open_close(raw)
    map_tools.save_pgm(os.path.join(d, 'map.pgm'), clean, res_m, [ox, oy])
    map_tools.save_preview(os.path.join(d, 'preview.png'), raw, clean)

    # 4) 元数据
    with S.lock:
        pose = S.pose
        scan_hz = S.scan_hz
    meta = {
        'map_id': map_id,
        'created': time.strftime('%Y-%m-%d %H:%M:%S'),
        'resolution': round(float(res_m), 4),
        'origin': [round(float(ox), 3), round(float(oy), 3), 0.0],
        'size': [int(grid_msg.info.width), int(grid_msg.info.height)],
        'occupied_raw': int(np.sum(raw == map_tools.OCC)),
        'occupied_clean': int(stats['occupied_after']),
        'denoise': {'removed': stats['removed'], 'added': stats['added'],
                    'iter': map_tools.MORPH_ITER,
                    'noise_min_neighbors': map_tools.NOISE_MIN_NEIGHBORS,
                    'gap_min_neighbors': map_tools.GAP_MIN_NEIGHBORS},
        'pbstream': bool(pbstream),
        'scan_hz': round(float(scan_hz), 2),
        'pose_at_finish': [round(float(v), 3) for v in pose[:3]],
        'state': 'ready',
    }
    map_tools.write_meta(map_id, meta)
    S.note('地图已入库: %s（障碍 %d→%d）' %
           (map_id, meta['occupied_raw'], meta['occupied_clean']))
    return map_id, '建图完成：%s（障碍 %d → 降噪后 %d）' % (
        map_id, meta['occupied_raw'], meta['occupied_clean'])
