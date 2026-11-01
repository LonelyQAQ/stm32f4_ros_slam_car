#!/usr/bin/env python3
"""Web 控制台共享状态（webui / nav_manager / slam_manager 共用）"""
import threading
import time
import uuid


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.map_ver = 0
        self.map_meta = None      # dict(w,h,res,ox,oy,rle)
        self.map_last_msg = None  # 最近一帧 OccupancyGrid，入库时导出 PGM 用
        self.map_session = uuid.uuid4().hex
        self.map_stamp = None
        self.map_seen_t = 0.0
        self.last_scan_msg = None
        self.last_scan_t = 0.0
        self.scan_ver = 0
        self.scan_pts = []        # 世界坐标系下的 [x,y] 展平
        self.pose = (0.0, 0.0, 0.0, 0.0, 0.0)   # x,y,th,v,w（odom）
        self.map_pose = None                     # x,y,th（map→base_link）
        self.map_pose_t = 0.0                    # map_pose 最近更新时间
        self.imu = (0.0, 0.0, 0.0)             # roll,pitch,yaw (deg)
        self.imu_zero = [0.0, 0.0, 0.0]
        self.cmd = [0.0, 0.0]
        self.cmd_t = 0.0
        self.scan_hz = 0.0
        self.scan_count = 0
        self.paused = False
        self.proc = None
        self.loc_proc = None
        self.localizing = False
        self.map_static = False
        self.map_id = None
        self.reloc_busy = False                 # 全局重定位进行中
        self.loc_quality = None                 # dict(inlier, mean, ok, t)
        self.log = []
        self.mode = 'manual'
        self.nav = {'active': False, 'status': 'idle', 'goal': None,
                    'distance': None, 'eta': None, 'source': None}
        self.safety = {'blocked': [], 'min': {}, 'reason': '', 'stale': True}
        self.home = {'x': 0.0, 'y': 0.0, 'yaw': 0.0,
                     'source': 'map_origin', 'stamp': 0}
        self.home_dist = None                    # 当前位姿到原点的距离（m）
        self.path = []
        self.path_ver = 0

    def note(self, msg):
        with self.lock:
            self.log.append((time.time(), msg))
            self.log = self.log[-30:]


S = State()


def rle_encode(data):
    """int8 数组 → [[value, count], ...]"""
    out = []
    if not data:
        return out
    cur = data[0]
    n = 1
    for v in data[1:]:
        if v == cur and n < 65535:
            n += 1
        else:
            out.append([cur, n])
            cur = v
            n = 1
    out.append([cur, n])
    return out
