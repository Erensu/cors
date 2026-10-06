/**
 * 通用工具：状态配色、格式化、提示条
 */
(function (global) {
  'use strict';

  /**
   * 解状态 → 语义分类
   * 覆盖两套来源：
   *  - 基线解算：engine.c 的 SOL_STR（FIX/FLOAT/PPP/...）
   *  - 定位报文：solution.c nmea_solq 的 quality（4=FIX 5=FLOAT 3=PPP）
   */
  const LEVELS = {
    FIX: 'fix',
    FIXDEG: 'fixdeg',
    FLOAT: 'float',
    PPP: 'ppp',
    DGPS: 'dgps',
    SINGLE: 'single',
    SBAS: 'sbas',
    DR: 'dr',
    NONE: 'none'
  };

  const LABELS = {
    fix: '固定解',
    fixdeg: '固定解(降级)',
    float: '浮点解',
    ppp: 'PPP',
    dgps: '差分',
    single: '单点',
    sbas: 'SBAS',
    dr: '航位推算',
    none: '无解'
  };

  /**
   * 挂载 canvas 到容器。
   *
   * HTML 里的绘图区写的是 <div id="xxx" class="chart"></div>（便于用 CSS 控制
   * 尺寸与占位），而绘图模块需要 canvas 元素。这里在容器内动态插入一个
   * canvas 并返回，三个图表模块共用，避免在 HTML 里重复维护。
   *
   * 若容器本身就是 canvas 则直接复用。
   *
   * @param {string|HTMLElement} container 容器元素 **或** 其 id
   * @returns {HTMLCanvasElement|null}
   *
   * 两种入参都支持是必需的：抽屉里的曲线容器是 innerHTML 动态生成的，
   * 没有稳定的 id 可查（每次重开都是新节点），只能直接传元素。
   * 而主面板的趋势图/网图/天穹图容器在 HTML 里写死，仍按 id 查。
   */
  function mountCanvas(container) {
    const box = (typeof container === 'string')
      ? document.getElementById(container)
      : container;
    if (!box) return null;

    // 已经是 canvas，直接用
    if (box.tagName === 'CANVAS') return box;

    const cv = document.createElement('canvas');
    cv.style.display = 'block';
    cv.style.width = '100%';
    cv.style.height = '100%';
    box.appendChild(cv);
    return cv;
  }

  function levelOf(stat) {
    if (!stat) return null;
    return LEVELS[String(stat).toUpperCase()] || 'none';
  }

  function labelOf(stat) {
    const lv = levelOf(stat);
    return lv ? LABELS[lv] : '—';
  }

  /** 状态配色（用于图表与徽标） */
  const COLORS = {
    fix: '#16a34a',
    fixdeg: '#0891b2',
    float: '#f59e0b',
    ppp: '#8b5cf6',
    dgps: '#0ea5e9',
    sbas: '#a855f7',
    single: '#64748b',
    dr: '#6b7280',
    none: '#cbd5e1'
  };

  function colorOf(stat) {
    const lv = levelOf(stat);
    return COLORS[lv] || COLORS.none;
  }

  /** #rrggbb → rgba(r,g,b,a)，用于光晕等半透明叠加 */
  function hexToRgba(hex, alpha) {
    if (!hex) return 'rgba(0,0,0,0)';
    let h = String(hex).replace('#', '');
    if (h.length === 3) h = h.split('').map((c) => c + c).join('');
    const n = parseInt(h, 16);
    if (!Number.isFinite(n)) return 'rgba(0,0,0,0)';
    const r = (n >> 16) & 255;
    const g = (n >> 8) & 255;
    const b = n & 255;
    return `rgba(${r},${g},${b},${alpha === undefined ? 1 : alpha})`;
  }

  /** HTML 转义，防止站点名里的特殊字符破坏结构 */
  function esc(s) {
    if (s === null || s === undefined) return '';
    return String(s)
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#39;');
  }

  /** 数字格式化 */
  function num(v, digits) {
    if (v === null || v === undefined || !isFinite(v)) return '—';
    return Number(v).toFixed(digits === undefined ? 2 : digits);
  }

  /** 相对时间 */
  function ago(ts) {
    if (!ts) return '—';
    const s = Math.floor((Date.now() - ts) / 1000);
    if (s < 60) return `${s}秒前`;
    if (s < 3600) return `${Math.floor(s / 60)}分钟前`;
    return `${Math.floor(s / 3600)}小时前`;
  }

  /** 时钟文本 */
  function clock(ts) {
    if (!ts) return '—';
    const d = new Date(ts);
    const p = (n) => String(n).padStart(2, '0');
    return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
  }

  /** 提示条 */
  function toast(title, msg, kind) {
    const wrap = document.getElementById('toasts');
    if (!wrap) return;
    const el = document.createElement('div');
    el.className = 'toast ' + (kind || '');
    el.innerHTML = `<b>${esc(title)}</b><span>${esc(msg || '')}</span>`;
    wrap.appendChild(el);
    setTimeout(() => {
      el.style.opacity = '0';
      el.style.transition = 'opacity .25s';
      setTimeout(() => el.remove(), 250);
    }, kind === 'err' ? 6000 : 3500);
  }

  /** 确认对话（替代原生 confirm，保证风格统一） */
  function confirmDialog(title, message) {
    return new Promise((resolve) => {
      const mask = document.getElementById('mask');
      const box = document.createElement('div');

      box.className = 'modal-box';
      box.style.width = 'min(420px,100%)';
      box.innerHTML = `
        <div class="modal-head"><h3>${esc(title)}</h3></div>
        <div class="modal-body">
          <p style="margin:0;line-height:1.7;color:var(--text-2)">${esc(message)}</p>
          <div class="modal-foot">
            <button class="btn ghost" data-act="cancel">取消</button>
            <button class="btn danger" data-act="ok">确认删除</button>
          </div>
        </div>`;

      mask.hidden = false;
      mask.innerHTML = '';
      mask.appendChild(box);

      const done = (v) => {
        mask.hidden = true;
        mask.innerHTML = '';
        resolve(v);
      };
      box.querySelector('[data-act="cancel"]').onclick = () => done(false);
      box.querySelector('[data-act="ok"]').onclick = () => done(true);
      mask.onclick = (e) => {
        if (e.target === mask) done(false);
      };
    });
  }

  /**
   * 两经纬度点间的大圆距离（km），Haversine 公式
   *
   * 用于三角网边长标注。与引擎侧 norm(pos2pos()) 的球面距离在
   * 站点尺度（几到几百公里）内差异可忽略，足够标注用。
   */
  function haversineKm(lat1, lon1, lat2, lon2) {
    const R = 6371.0088; // 地球平均半径，IUGG 推荐值
    const toRad = Math.PI / 180;
    const dLat = (lat2 - lat1) * toRad;
    const dLon = (lon2 - lon1) * toRad;
    const a =
      Math.sin(dLat / 2) ** 2 +
      Math.cos(lat1 * toRad) * Math.cos(lat2 * toRad) * Math.sin(dLon / 2) ** 2;
    return 2 * R * Math.asin(Math.min(1, Math.sqrt(a)));
  }

  global.MCORS = global.MCORS || {};
  global.MCORS.util = {
    levelOf, labelOf, colorOf, hexToRgba, COLORS, LABELS,
    esc, num, ago, clock, toast, confirmDialog, mountCanvas,
    haversineKm
  };
})(window);