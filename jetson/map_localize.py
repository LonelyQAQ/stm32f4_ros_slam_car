#!/usr/bin/env python3
"""单帧激光对静态地图做全局定位（扫描 ↔ 地图边缘拟合）

流程：
  1. 把 /scan 从 laser_frame 换到 base_link（雷达安装 yaw=π）；
  2. 对静态地图做距离场（到最近障碍的距离）；
  3. 在地图自由格上粗搜 (x, y, yaw)，用扫描点到障碍的平均距离/内点率评分；
  4. 对前几名做 0.1 m/5°、0.05 m/2° 精修；
  5. 返回最佳位姿，交给 AMCL 作为初始位姿。

用于“上电后车在地图任意位置”的粗定位；AMCL 再负责连续跟踪。
"""
import math

import numpy as np

import map_tools

LASER_YAW = math.pi          # base_link→laser_frame 静态 TF
POS_STRIDE = 2               # 自由格抽稀，2 格 = 0.1 m
THETA_STEP = math.radians(10.0)
SCAN_STRIDE = 4              # 约 440 点 → 110 点（点太少会锁到相似房间）
MIN_POINTS = 40
MAX_DIST = 0.5               # 评分时距离截断
INLIER_DIST = 0.15
MIN_INLIER = 0.55            # 低于它认为全局定位不可信
MAX_MEAN = 0.12              # 平均最近距离上限（m），配合 MIN_INLIER 判可信
SQRT2 = math.sqrt(2.0)


def _distance_field(occ):
    """二值障碍图 → 到最近障碍的栅格距离（8 邻域 Chamfer）"""
    h, w = occ.shape
    inf = 1.0e9
    d = np.where(occ, 0.0, inf)
    for y in range(h):
        for x in range(w):
            v = d[y, x]
            if v == 0.0:
                continue
            if y > 0:
                v = min(v, d[y - 1, x] + 1.0)
                if x > 0:
                    v = min(v, d[y - 1, x - 1] + SQRT2)
                if x + 1 < w:
                    v = min(v, d[y - 1, x + 1] + SQRT2)
            if x > 0:
                v = min(v, d[y, x - 1] + 1.0)
            d[y, x] = v
    for y in range(h - 1, -1, -1):
        for x in range(w - 1, -1, -1):
            v = d[y, x]
            if v == 0.0:
                continue
            if y + 1 < h:
                v = min(v, d[y + 1, x] + 1.0)
                if x > 0:
                    v = min(v, d[y + 1, x - 1] + SQRT2)
                if x + 1 < w:
                    v = min(v, d[y + 1, x + 1] + SQRT2)
            if x + 1 < w:
                v = min(v, d[y, x + 1] + 1.0)
            d[y, x] = v
    return d


def _scan_base(scan):
    if scan is None:
        return None
    rmin = max(0.1, float(scan.range_min))
    rmax = min(6.0, float(scan.range_max))
    pts = []
    for i in range(0, len(scan.ranges), SCAN_STRIDE):
        r = scan.ranges[i]
        if not math.isfinite(r) or r < rmin or r > rmax:
            continue
        a = scan.angle_min + i * scan.angle_increment + LASER_YAW
        pts.append((r * math.cos(a), r * math.sin(a)))
    if len(pts) < MIN_POINTS:
        return None
    return np.asarray(pts, dtype=np.float64)


def _grid(map_msg):
    grid = map_tools.occupancy_to_pgm(map_msg)
    res = float(map_msg.info.resolution)
    ox = float(map_msg.info.origin.position.x)
    oy = float(map_msg.info.origin.position.y)
    h, w = grid.shape
    dist = _distance_field(grid == map_tools.OCC) * res
    return grid, dist, res, ox, oy, h, w


def _scores(pts, dist, res, ox, oy, h, w):
    """pts: M×N×2 map 坐标。返回 score/inlier/mean，长度 M。"""
    col = np.floor((pts[:, :, 0] - ox) / res).astype(np.int32)
    row = (h - 1 - np.floor((pts[:, :, 1] - oy) / res)).astype(np.int32)
    valid = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    d = np.full(pts.shape[:2], MAX_DIST, dtype=np.float64)
    d[valid] = dist[row[valid], col[valid]]
    np.minimum(d, MAX_DIST, out=d)
    inlier = np.mean(d <= INLIER_DIST, axis=1)
    mean = np.mean(d, axis=1)
    return inlier - 0.2 * mean, inlier, mean


def _rot(pts, th):
    c, s = math.cos(th), math.sin(th)
    return np.column_stack([c * pts[:, 0] - s * pts[:, 1],
                            s * pts[:, 0] + c * pts[:, 1]])


def global_localize(map_msg, scan):
    """返回 dict(x,y,yaw,inlier,mean_dist,score)；失败返回 None。"""
    if map_msg is None or scan is None:
        return None
    base = _scan_base(scan)
    if base is None:
        return None
    grid, dist, res, ox, oy, h, w = _grid(map_msg)
    ys, xs = np.nonzero(grid == map_tools.FREE)
    sel = (ys % POS_STRIDE == 0) & (xs % POS_STRIDE == 0)
    ys, xs = ys[sel], xs[sel]
    if len(ys) == 0:
        return None
    cand = np.column_stack([ox + (xs + 0.5) * res,
                            oy + (h - 1 - ys + 0.5) * res])

    best = None
    for th in np.arange(-math.pi, math.pi, THETA_STEP):
        pts = _rot(base, float(th))[None, :, :] + cand[:, None, :]
        score, inlier, mean = _scores(pts, dist, res, ox, oy, h, w)
        i = int(np.argmax(score))
        if best is None or score[i] > best[0]:
            best = (float(score[i]), float(inlier[i]), float(mean[i]),
                    float(cand[i, 0]), float(cand[i, 1]), float(th))
    if best is None:
        return None

    for step_m, step_th in ((0.10, math.radians(5.0)),
                            (0.05, math.radians(2.0))):
        bx, by, bth = best[3], best[4], best[5]
        local = best
        for dx in (-step_m, 0.0, step_m):
            for dy in (-step_m, 0.0, step_m):
                for dth in (-step_th, 0.0, step_th):
                    th = bth + dth
                    pts = _rot(base, th) + np.array([bx + dx, by + dy])
                    score, inlier, mean = _scores(pts[None, :, :],
                                                  dist, res, ox, oy, h, w)
                    if float(score[0]) > local[0]:
                        local = (float(score[0]), float(inlier[0]),
                                 float(mean[0]), float(bx + dx),
                                 float(by + dy), float(th))
        best = local
    return {'x': best[3], 'y': best[4], 'yaw': best[5],
            'inlier': best[1], 'mean_dist': best[2], 'score': best[0]}
