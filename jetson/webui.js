const cv = document.getElementById('view');
const ctx = cv.getContext('2d');
let mapData = null;          // {w,h,res,ox,oy,img:canvas}
let scan = [];               // 传感器系下的 [x,y,...]
let path = [];               // map 系路径 [x,y,...]
let st = {
  pose: [0, 0, 0, 0, 0], map_pose: null,
  imu: [0, 0, 0], imu_zero: [0, 0, 0], cmd: [0, 0],
  scan_hz: 0, paused: false, slam_running: false, log: [], have_map: false,
  mode: 'manual', localizing: false, map_static: false, map_id: null,
  nav: { active: false, status: 'idle', goal: null, distance: null },
  safety: { blocked: [], min: {}, stale: true },
  home: { x: 0, y: 0, yaw: 0, source: 'map_origin' },
};
let view = { cx: 0, cy: 0, scale: 70 };   // 屏幕中心的世界坐标 + 每米像素
let drag = null;
let fitted = false;

function resize() {
  const r = cv.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  cv.width = Math.max(1, Math.round(r.width * dpr));
  cv.height = Math.max(1, Math.round(r.height * dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  draw();
}
window.addEventListener('resize', resize);

function w2s(x, y) {
  const r = cv.getBoundingClientRect();
  return [r.width / 2 + (x - view.cx) * view.scale, r.height / 2 - (y - view.cy) * view.scale];
}

function buildMapImage(m) {
  const c = document.createElement('canvas');
  c.width = m.w; c.height = m.h;
  const g = c.getContext('2d');
  const img = g.createImageData(m.w, m.h);
  const d = img.data;
  let i = 0;
  for (const [v, n] of m.rle) {
    let r, gg, b;
    if (v < 0) { r = 36; gg = 40; b = 48; }            // 未知
    else if (v >= 50) { r = 12; gg = 14; b = 18; }     // 障碍
    else { r = 225; gg = 228; b = 234; }               // 空闲
    for (let k = 0; k < n; k++, i++) {
      d[i * 4] = r; d[i * 4 + 1] = gg; d[i * 4 + 2] = b; d[i * 4 + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  m.img = c;
  return m;
}

function draw() {
  const r = cv.getBoundingClientRect();
  ctx.clearRect(0, 0, r.width, r.height);
  ctx.fillStyle = '#10131a';
  ctx.fillRect(0, 0, r.width, r.height);

  // 网格
  let step = 1;
  while (step * view.scale < 40) step *= 2;
  ctx.strokeStyle = '#1b2029';
  ctx.lineWidth = 1;
  const x0 = view.cx - r.width / 2 / view.scale, x1 = view.cx + r.width / 2 / view.scale;
  const y0 = view.cy - r.height / 2 / view.scale, y1 = view.cy + r.height / 2 / view.scale;
  for (let x = Math.floor(x0 / step) * step; x <= x1; x += step) {
    const [sx] = w2s(x, 0); ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, r.height); ctx.stroke();
  }
  for (let y = Math.floor(y0 / step) * step; y <= y1; y += step) {
    const [, sy] = w2s(0, y); ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(r.width, sy); ctx.stroke();
  }

  // 地图
  if (mapData && mapData.img) {
    const m = mapData;
    const [sx, syTop] = w2s(m.ox, m.oy + m.h * m.res);
    const w = m.w * m.res * view.scale, h = m.h * m.res * view.scale;
    ctx.imageSmoothingEnabled = false;
    // OccupancyGrid 行 0 在 map 底部；画布行 0 在顶部，这里翻转回来
    ctx.save();
    ctx.translate(sx, syTop + h);
    ctx.scale(1, -1);
    ctx.drawImage(m.img, 0, 0, w, h);
    ctx.restore();
  }

  // Nav2 路径
  if (path.length >= 4) {
    ctx.strokeStyle = 'rgba(72,213,151,0.9)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < path.length; i += 2) {
      const [ax, ay] = w2s(path[i], path[i + 1]);
      if (i === 0) ctx.moveTo(ax, ay); else ctx.lineTo(ax, ay);
    }
    ctx.stroke();
  }

  // 扫描点已由后端按 TF 变换到 map 系，这里直接画，不再乘位姿
  const p = st.map_pose || st.pose;
  const [px, py, th] = p;

  // 扫描点已经由后端按 map→laser_frame TF 变换到 map 系
  ctx.fillStyle = '#ff5c5c';
  for (let i = 0; i < scan.length; i += 2) {
    const x = scan[i];
    const y = scan[i + 1];
    const [ax, ay] = w2s(x, y);
    ctx.fillRect(ax - 1, ay - 1, 2, 2);
  }

  // 导航目标
  const goal = st.nav && st.nav.goal;
  if (goal && goal.length >= 2) {
    const [gx, gy] = w2s(goal[0], goal[1]);
    ctx.strokeStyle = '#ffb020';
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(gx, gy, 9, 0, Math.PI * 2); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(gx - 13, gy); ctx.lineTo(gx + 13, gy);
    ctx.moveTo(gx, gy - 13); ctx.lineTo(gx, gy + 13); ctx.stroke();
  }

  // 机器人：三角 + 朝向线
  {
    const [ax, ay] = w2s(px, py);
    ctx.save();
    ctx.translate(ax, ay);
    ctx.rotate(-th);
    ctx.beginPath();
    ctx.moveTo(16, 0); ctx.lineTo(-10, 9); ctx.lineTo(-10, -9); ctx.closePath();
    ctx.fillStyle = '#4fa3ff'; ctx.fill();
    ctx.strokeStyle = '#cfe4ff'; ctx.lineWidth = 1; ctx.stroke();
    ctx.restore();
  }
}

// ---------- 事件流 ----------
let es = null;
let esTimer = null;
function connect() {
  if (es) {                      // 必须先关旧连接，否则每次重连都会多占一条 TCP
    try { es.close(); } catch (e) {}
    es = null;
  }
  es = new EventSource('/stream');
  es.addEventListener('map', e => {
    mapData = buildMapImage(JSON.parse(e.data));
    if (!fitted) { fitMap(); fitted = true; }
    draw();
  });
  es.addEventListener('scan', e => {
    scan = JSON.parse(e.data).pts;
    draw();
  });
  es.addEventListener('path', e => {
    path = JSON.parse(e.data).pts || [];
    draw();
  });
  es.addEventListener('state', e => {
    Object.assign(st, JSON.parse(e.data));
    updateHud();
    draw();
  });
  es.onerror = () => {
    if (es) {
      try { es.close(); } catch (e) {}
      es = null;
    }
    if (esTimer) return;         // 同一时间只允许一个重连计时器
    esTimer = setTimeout(() => { esTimer = null; connect(); }, 2000);
  };
}

function fitMap() {
  if (!mapData) return;
  const r = cv.getBoundingClientRect();
  const mw = mapData.w * mapData.res, mh = mapData.h * mapData.res;
  view.scale = Math.min(r.width / mw, r.height / mh) * 0.92;
  view.cx = mapData.ox + mw / 2;
  view.cy = mapData.oy + mh / 2;
}

function updateHud() {
  const p = st.map_pose || st.pose;
  const [x, y, th] = p;
  const v = st.pose[3] || 0, w = st.pose[4] || 0;
  document.getElementById('pose').textContent =
      `x ${x.toFixed(2)}  y ${y.toFixed(2)}  θ ${(th * 57.2958).toFixed(1)}°`;
  document.getElementById('spd').textContent = `${v.toFixed(2)} m/s  ${w.toFixed(2)} rad/s`;
  const z = st.imu_zero, a = st.imu;
  document.getElementById('imu').textContent =
      `r ${(a[0] - z[0]).toFixed(1)}  p ${(a[1] - z[1]).toFixed(1)}  y ${(a[2] - z[2]).toFixed(1)}`;
  document.getElementById('imuAbs').textContent =
      `${a[0].toFixed(1)} / ${a[1].toFixed(1)} / ${a[2].toFixed(1)}`;
  document.getElementById('dScan').textContent = `${(scan.length / 2) | 0} 点 · ${st.scan_hz.toFixed(1)} Hz`;
  document.getElementById('dMap').textContent = mapData ? `${mapData.w}×${mapData.h} @ ${mapData.res.toFixed(2)} m` : '无';
  document.getElementById('dCmd').textContent = `${st.cmd[0].toFixed(2)}, ${st.cmd[1].toFixed(2)}`;

  if (st.maps) refreshMaps(st.maps);
  const slam = document.getElementById('slamState');
  const q = st.loc_quality;
  const qtxt = (st.localizing && q)
      ? ` · 匹配 ${q.inlier.toFixed(2)}/${q.mean.toFixed(2)}m`
      : '';
  const qcls = (q && !q.ok) ? 'warn' : 'on';
  slam.innerHTML = st.localizing
      ? `<span class="dot ${qcls}"></span>定位中${st.map_id ? ' · ' + st.map_id : ''}${qtxt}${st.reloc_busy ? ' · 重定位中' : ''}`
      : st.slam_running
          ? `<span class="dot ${st.paused ? 'warn' : 'on'}"></span>${st.paused ? '已暂停' : '建图中'}`
          : '<span class="dot"></span>未运行';

  const nav = st.nav || {};
  const status = nav.active ? (nav.status || '执行中') : (nav.status || '空闲');
  document.getElementById('navState').textContent = status;
  document.getElementById('navDist').textContent =
      (nav.distance != null) ? `${nav.distance.toFixed(2)} m` : '-';
  const home = st.home || { x: 0, y: 0 };
  document.getElementById('homePose').textContent =
      home.valid
          ? `x ${home.x.toFixed(2)} y ${home.y.toFixed(2)}`
          : `未设置${home.invalid_reason ? '（' + home.invalid_reason + '）' : ''}`;
  document.getElementById('homeDist').textContent =
      (home.valid && st.home_dist != null) ? `${(st.home_dist * 100).toFixed(1)} cm` : '-';
  const he = nav.home_error;
  document.getElementById('homeErr').textContent =
      he ? `${(he.distance * 100).toFixed(1)} cm / ${he.yaw.toFixed(1)}°`
           + (he.jump != null ? ` · 停车后跳变 ${(he.jump * 100).toFixed(1)} cm` : '')
         : '-';
  const safety = st.safety || { blocked: [], stale: true };
  const blocked = safety.blocked || [];
  const comp = safety.blocked_components || {};
  const flags = [comp.linear_x ? 'v' : '', comp.angular_z ? 'w' : '']
      .filter(Boolean).join('/');
  document.getElementById('safetyState').textContent =
      blocked.length
          ? `受阻 ${blocked.join('/')}${flags ? ' [' + flags + ']' : ''}`
          : (safety.stale ? '无雷达' : '正常');

  const modeBtn = document.getElementById('bMode');
  modeBtn.textContent = st.mode === 'manual' ? '手动模式' : '自动模式';
  modeBtn.classList.toggle('warn', st.mode !== 'manual');
  document.getElementById('hint').textContent =
      st.mode === 'manual'
          ? 'WASD 行驶 · 滚轮缩放 · 拖动平移 · 点击地图设导航目标'
          : '自动导航中 · WASD 已禁用 · 点击地图可改目标 · 急停随时有效';

  const log = document.getElementById('log');
  log.textContent = (st.log || []).map(l => {
    const d = new Date(l[0] * 1000);
    return `${d.toTimeString().slice(0, 8)}  ${l[1]}`;
  }).join('\n') || '-';
}

// ---------- 指令 ----------
async function post(obj) {
  try {
    const r = await fetch('/api', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(obj),
    });
    const j = await r.json();
    const t = document.getElementById('toast');
    t.style.color = j.ok ? 'var(--ok)' : '#ff7b7b';
    t.textContent = j.msg || '';
    return j;
  } catch (e) {
    document.getElementById('toast').textContent = '请求失败: ' + e;
    return { ok: false };
  }
}

// ---------- 键盘 & 按钮 ----------
const keys = new Set();
let lastSent = 0, sentNonZero = false;

function speeds() {
  return [parseFloat(document.getElementById('lin').value),
          parseFloat(document.getElementById('ang').value)];
}

function tickControl() {
  if (st.mode !== 'manual') return;     // 自动模式由 Nav2 控制
  const [lv, av] = speeds();
  let v = 0, w = 0;
  if (keys.has('w')) v += lv;
  if (keys.has('s')) v -= lv;
  if (keys.has('a')) w += av;
  if (keys.has('d')) w -= av;
  const now = performance.now();
  if (v || w) {
    if (now - lastSent > 100) { post({ cmd: 'vel', v, w }); lastSent = now; sentNonZero = true; }
  } else if (sentNonZero) {
    post({ cmd: 'stop' }); sentNonZero = false;
  }
}
setInterval(tickControl, 50);

window.addEventListener('keydown', e => {
  const k = e.key.toLowerCase();
  if ('wasd'.includes(k)) { keys.add(k); e.preventDefault(); }
});
window.addEventListener('keyup', e => {
  const k = e.key.toLowerCase();
  if ('wasd'.includes(k)) { keys.delete(k); e.preventDefault(); }
});
window.addEventListener('blur', () => keys.clear());

function hold(id, k) {
  const el = document.getElementById(id);
  const on = e => { e.preventDefault(); keys.add(k); };
  const off = e => { e.preventDefault(); keys.delete(k); };
  el.addEventListener('mousedown', on); el.addEventListener('mouseup', off);
  el.addEventListener('mouseleave', off);
  el.addEventListener('touchstart', on); el.addEventListener('touchend', off);
}
hold('bF', 'w'); hold('bB', 's'); hold('bL', 'a'); hold('bR', 'd');

document.getElementById('bStop').onclick = () => { keys.clear(); post({ cmd: 'estop' }); };
document.getElementById('bStart').onclick = () => post({ cmd: 'slam_start' });
document.getElementById('bFinish').onclick = () => post({ cmd: 'map_finish' });
document.getElementById('bKill').onclick = () => post({ cmd: 'slam_stop' });
document.getElementById('bUseMap').onclick = () => post({ cmd: 'map_use', map_id: currentMap() });
document.getElementById('bReloc').onclick = () => post({ cmd: 'map_relocalize' });
let initPoseMode = false;
const bInitPose = document.getElementById('bInitPose');
bInitPose.onclick = () => {
  initPoseMode = !initPoseMode;
  bInitPose.classList.toggle('warn', initPoseMode);
  document.getElementById('toast').textContent =
      initPoseMode ? '点击地图设置 AMCL 初始位姿' : '';
};
document.getElementById('bCheck').onclick = () => post({ cmd: 'map_check', map_id: currentMap() });
document.getElementById('bDelMap').onclick = () => {
  const id = currentMap();
  if (id && confirm('删除地图 ' + id + '？不可恢复')) post({ cmd: 'map_delete', map_id: id });
};
function currentMap() {
  const el = document.getElementById('mapSel');
  return el && el.value ? el.value : '';
}
let mapsSig = '';
function refreshMaps(list) {
  const el = document.getElementById('mapSel');
  if (!el) return;
  const sig = (list || []).map(m => m.map_id).join(',');
  if (sig === mapsSig) return;          // 列表没变就不重建，避免冲掉用户选择
  mapsSig = sig;
  const keep = el.value;
  el.innerHTML = '';
  (list || []).forEach(m => {
    const o = document.createElement('option');
    o.value = m.map_id;
    const fit = m.fit_check ? (m.fit_check.passed ? '✓' : '✗') : '—';
    o.textContent = `${m.map_id}  ${m.size[0]}×${m.size[1]}  障碍${m.occupied_clean}  校验${fit}`;
    el.appendChild(o);
  });
  if (keep) el.value = keep;
  if (!el.options.length) {
    const o = document.createElement('option');
    o.value = ''; o.textContent = '（暂无地图）';
    el.appendChild(o);
  }
}
document.getElementById('bImuZero').onclick = () => post({ cmd: 'imu_zero' });
document.getElementById('bImuClear').onclick = () => post({ cmd: 'imu_clear' });

document.getElementById('bMode').onclick = () => {
  keys.clear();
  post({ cmd: 'mode', mode: st.mode === 'manual' ? 'auto' : 'manual' });
};
document.getElementById('bHome').onclick = () => post({ cmd: 'nav_home' });
document.getElementById('bSetHome').onclick = () => post({ cmd: 'set_home' });
document.getElementById('bNavCancel').onclick = () => post({ cmd: 'nav_cancel' });
document.getElementById('bNavStart').onclick = () => post({ cmd: 'nav_start' });
document.getElementById('bNavStop').onclick = () => post({ cmd: 'nav_stop' });

document.getElementById('lin').oninput = e => document.getElementById('linv').textContent = (+e.target.value).toFixed(2);
document.getElementById('ang').oninput = e => document.getElementById('angv').textContent = (+e.target.value).toFixed(2);

// ---------- 缩放 / 平移 / 点击设目标 ----------
cv.addEventListener('wheel', e => {
  e.preventDefault();
  const f = Math.exp(-e.deltaY / 400);
  view.scale = Math.min(400, Math.max(8, view.scale * f));
  draw();
}, { passive: false });

cv.addEventListener('mousedown', e => {
  drag = { x: e.clientX, y: e.clientY, cx: view.cx, cy: view.cy, moved: false };
  cv.style.cursor = 'grabbing';
});
window.addEventListener('mousemove', e => {
  if (!drag) return;
  if (Math.abs(e.clientX - drag.x) > 4 || Math.abs(e.clientY - drag.y) > 4) drag.moved = true;
  view.cx = drag.cx - (e.clientX - drag.x) / view.scale;
  view.cy = drag.cy + (e.clientY - drag.y) / view.scale;
  draw();
});
window.addEventListener('mouseup', e => {
  if (drag && !drag.moved && e.target === cv && mapData) {
    const r = cv.getBoundingClientRect();
    const sx = e.clientX - r.left, sy = e.clientY - r.top;
    const x = view.cx + (sx - r.width / 2) / view.scale;
    const y = view.cy - (sy - r.height / 2) / view.scale;
    const p = st.map_pose || st.pose;
    if (initPoseMode) {
      if (st.map_pose) {
        post({ cmd: 'initial_pose', x, y, yaw: st.map_pose[2] });
      } else {
        document.getElementById('toast').textContent = '地图位姿不可用，无法设初始位姿';
      }
      initPoseMode = false;
      bInitPose.classList.remove('warn');
    } else {
      const yaw = Math.atan2(y - p[1], x - p[0]);
      post({ cmd: 'nav_goal', x, y, yaw });
    }
  }
  drag = null;
  cv.style.cursor = 'grab';
});

resize();
connect();
