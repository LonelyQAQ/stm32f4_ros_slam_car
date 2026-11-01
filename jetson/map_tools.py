#!/usr/bin/env python3
"""地图库：保存 / 形态学降噪 / 拟合性检查 / 列表 / 删除

目录结构（~/maps/<map_id>/）：
    raw.pgm  raw.yaml      建图原始产物
    map.pgm  map.yaml      降噪后成品（导航用这个）
    map.pbstream           Cartographer 位姿图（后续定位/续建用）
    preview.png            降噪前后对比
    meta.json              元数据 + 拟合检查结果

PGM 取值按 map_server 约定：0=障碍、254=空闲、205=未知。
"""
import json
import math
import os
import re
import shutil
import time

import numpy as np
from PIL import Image

MAPS_DIR = os.path.expanduser('~/maps')

# 栅格三态
OCC, FREE, UNKNOWN = 0, 254, 205

# 降噪参数（写进 meta.json，便于回溯）
MORPH_ITER = 1
NOISE_MIN_NEIGHBORS = 1   # 8 邻域障碍数 ≤ 它 → 判为孤立噪点（墙有 2 个以上邻居，不会误删）
GAP_MIN_NEIGHBORS = 6     # 8 邻域障碍数 ≥ 它 → 判为裂缝补齐（墙角通常 3~5，不会被糊死）

# 拟合性检查判据
FIT_INLIER_MIN = 0.60
FIT_DIST_MAX = 0.15      # m


# --------------------------------------------------------------------------
# 读写
# --------------------------------------------------------------------------
def new_map_id():
    return time.strftime('%Y%m%d_%H%M%S')


def map_dir(map_id):
    return os.path.join(MAPS_DIR, map_id)


def save_pgm(path, grid, resolution, origin):
    """grid: uint8 数组，行序自上而下（与 PGM 一致）；origin=[x,y]"""
    h, w = grid.shape
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(grid.tobytes())
    yaml_path = os.path.splitext(path)[0] + '.yaml'
    with open(yaml_path, 'w') as f:
        f.write('image: %s\n' % os.path.basename(path))
        f.write('resolution: %.6f\n' % resolution)
        f.write('origin: [%.6f, %.6f, 0.000000]\n' % (origin[0], origin[1]))
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


def load_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    m = re.match(rb'P5\s+(\d+)\s+(\d+)\s+255\s', data)
    if not m:
        raise ValueError('不是 P5 PGM: %s' % path)
    w, h = int(m.group(1)), int(m.group(2))
    grid = np.frombuffer(data[m.end():m.end() + w * h], dtype=np.uint8).reshape((h, w))
    return grid


def occupancy_to_pgm(msg):
    """nav_msgs/OccupancyGrid → (uint8 网格, 已是 PGM 行序)"""
    w, h = msg.info.width, msg.info.height
    arr = np.array(msg.data, dtype=np.int16).reshape((h, w))
    grid = np.full((h, w), UNKNOWN, dtype=np.uint8)
    grid[arr == 0] = FREE
    grid[(arr > 0) & (arr < 50)] = FREE          # 0..49 视为空闲
    grid[arr >= 50] = OCC
    # OccupancyGrid 行序自下而上，PGM 自上而下
    return np.flipud(grid)


# --------------------------------------------------------------------------
# 形态学降噪（不依赖 scipy/opencv）
# --------------------------------------------------------------------------
def _neighbors(mask):
    """返回 8 邻域 + 自身的布尔栈 (9, h, w)"""
    pad = np.pad(mask, 1, mode='constant', constant_values=False)
    h, w = mask.shape
    out = np.empty((9, h, w), dtype=bool)
    k = 0
    for dy in (0, 1, 2):
        for dx in (0, 1, 2):
            out[k] = pad[dy:dy + h, dx:dx + w]
            k += 1
    return out


def morph_open_close(grid):
    """形态学降噪：去孤立噪点（开运算的等价效果）+ 补小裂缝（闭运算）。

    为什么不直接做 3×3 腐蚀：单线雷达扫出来的墙只有 1 格厚，标准 8 邻域腐蚀会把整堵墙
    当噪点删光（实测 434 个障碍格全变 0）。这里改用两个局部判据：
      * 去噪点：障碍格的 8 邻域里障碍数 ≤ NOISE_MIN_NEIGHBORS → 判为孤立点删掉
      * 补裂缝：空闲格的 8 邻域里障碍数 ≥ GAP_MIN_NEIGHBORS → 判为墙上的缝，补成障碍
    两者都只在"已知"区域内生效，未知区保持未知。
    """
    occ = grid == OCC
    free = grid == FREE
    h, w = grid.shape

    for _ in range(MORPH_ITER):
        nb = _neighbors(occ)
        n_occ = nb.sum(axis=0) - 1          # 去掉自身

        # 去孤立点：邻域几乎没有邻居的障碍格
        speckle = occ & (n_occ <= NOISE_MIN_NEIGHBORS)
        occ_new = occ & ~speckle

        # 补裂缝：被障碍包围的空闲格
        nb = _neighbors(occ_new)
        n_occ2 = nb.sum(axis=0)
        gap = free & (n_occ2 >= GAP_MIN_NEIGHBORS)
        occ_new = occ_new | gap

        occ = occ_new

    out = grid.copy()
    out[:] = np.where(free, FREE, UNKNOWN)
    out[occ] = OCC
    removed = int(np.sum((grid == OCC) & ~occ))
    added = int(np.sum((grid != OCC) & occ))
    return out, {'removed': removed, 'added': added,
                 'occupied_after': int(np.sum(occ))}


def save_preview(path, raw_grid, clean_grid):
    """左原图右成品"""
    def to_rgb(g):
        img = np.zeros((g.shape[0], g.shape[1], 3), dtype=np.uint8)
        img[g == FREE] = (250, 250, 250)
        img[g == UNKNOWN] = (128, 128, 128)
        img[g == OCC] = (0, 0, 0)
        return img

    gap = 8
    h, w = raw_grid.shape
    canvas = np.full((h, w * 2 + gap, 3), 200, dtype=np.uint8)
    canvas[:, :w] = to_rgb(raw_grid)
    canvas[:, w + gap:] = to_rgb(clean_grid)
    Image.fromarray(canvas).save(path)


# --------------------------------------------------------------------------
# 拟合性检查：当前扫描点 vs 地图障碍单元
# --------------------------------------------------------------------------
def fit_check(grid, resolution, origin, pts_map, inlier_dist=FIT_DIST_MAX):
    """pts_map: Nx2 世界坐标点（map 系）。返回 dict。"""
    if pts_map is None or len(pts_map) == 0:
        return {'ok': False, 'reason': 'no_scan'}

    occ_yx = np.argwhere(grid == OCC)          # (n, 2) = (row, col)
    if len(occ_yx) == 0:
        return {'ok': False, 'reason': 'empty_map'}

    # 栅格坐标：col = (x-ox)/res, row = (oy + h*res - y)/res  (PGM 行序自上而下)
    h = grid.shape[0]
    pts = np.asarray(pts_map, dtype=np.float64)
    col = (pts[:, 0] - origin[0]) / resolution
    row = h - 1 - (pts[:, 1] - origin[1]) / resolution

    occ_col = occ_yx[:, 1].astype(np.float64)
    occ_row = occ_yx[:, 0].astype(np.float64)

    # 分块计算最近障碍距离（避免一次性 N*M 内存）
    best = np.full(len(pts), np.inf)
    chunk = max(1, int(2e6 // max(1, len(occ_col))))
    for i in range(0, len(pts), chunk):
        dc = col[i:i + chunk, None] - occ_col[None, :]
        dr = row[i:i + chunk, None] - occ_row[None, :]
        best[i:i + chunk] = np.sqrt((dc * dc + dr * dr).min(axis=1))

    dist_m = best * resolution
    inlier = float(np.mean(dist_m <= inlier_dist))
    mean_d = float(np.mean(dist_m))
    passed = inlier >= FIT_INLIER_MIN and mean_d <= FIT_DIST_MAX
    return {
        'ok': True,
        'inlier_ratio': round(inlier, 3),
        'mean_dist_m': round(mean_d, 3),
        'points': int(len(pts)),
        'passed': bool(passed),
    }


# --------------------------------------------------------------------------
# 地图库管理
# --------------------------------------------------------------------------
def list_maps():
    out = []
    if not os.path.isdir(MAPS_DIR):
        return out
    for name in sorted(os.listdir(MAPS_DIR), reverse=True):
        meta_path = os.path.join(MAPS_DIR, name, 'meta.json')
        if not os.path.isfile(meta_path):
            continue
        try:
            with open(meta_path) as f:
                meta = json.load(f)
        except Exception:
            continue
        out.append(meta)
    return out


def write_meta(map_id, meta):
    d = map_dir(map_id)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, 'meta.json'), 'w') as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)


def delete_map(map_id):
    d = map_dir(map_id)
    if not os.path.isdir(d):
        return False, '地图不存在'
    if not re.fullmatch(r'\d{8}_\d{6}', map_id):
        return False, '地图 id 非法'
    shutil.rmtree(d)
    return True, '已删除 %s' % map_id
