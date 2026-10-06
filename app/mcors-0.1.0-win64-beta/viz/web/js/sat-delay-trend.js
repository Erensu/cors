/**
 * 逐星双差延迟曲线图（原生 Canvas 折线图，零依赖）
 *
 * 用于基线详情抽屉里「点击某颗卫星的延迟数值 → 看该星历史走势」。
 * 与 trend.js（定位质量趋势）是同类但不同的两件事：
 *   trend.js  指标固定为 DOP/星数，画在主面板的固定 canvas 上；
 *   本模块    指标为逐星三路延迟，画在抽屉里**动态创建**的容器中。
 * 故独立成文件，不去给 trend.js 塞参数 —— 那会让它的主面板路径也变复杂。
 *
 * 三路延迟的物理含义不同，故 Y 轴不共用同一量程，硬叠加会掩盖量级差异：
 *   atmo 大气延迟合计（≈ 对流层 + 电离层），量级最大
 *   trpd 双差对流层延迟，量级中等
 *   iond 双差电离层延迟，**有正有负**且跳变最剧烈，是排障的主要看点
 */
(function (global) {
  'use strict';

  const U = global.MCORS.util;

  /* 指标定义。decimals 由实测量级定：
   * 大气/对流层是厘米级（0.0x~1.x m），2 位小数足以看出毫米级抖动；
   * 电离层残差常在 ±0.5m 内摆动，3 位小数才看得出正负交替。 */
  const METRICS = {
    atmo: { label: '双差大气延迟', short: '大气', field: 'atmo', color: '#0891b2', decimals: 2 },
    trpd: { label: '双差对流层延迟', short: '对流层', field: 'trpd', color: '#16a34a', decimals: 2 },
    iond: { label: '双差电离层延迟', short: '电离层', field: 'iond', color: '#dc2626', decimals: 3 }
  };

  class SatDelayChart {
    /**
     * @param {HTMLElement} box  容器元素（内部会创建 canvas）
     * @param {object} [opts]    { onMetric }
     */
    constructor(box, opts) {
      this.box = box;
      this.opts = opts || {};
      if (!box) return;

      this.canvas = U.mountCanvas(box);
      if (!this.canvas) return;
      this.ctx = this.canvas.getContext('2d');

      this.metric = 'iond';   // 默认看电离层：跳变最剧烈，排障价值最高
      this.series = [];        // [{ts, atmo, trpd, iond, el}]
      this.satName = '';
      this.blName = '';

      this._onResize = () => { this.resize(); this.render(); };
      global.addEventListener('resize', this._onResize);
    }

    destroy() {
      if (this._onResize) global.removeEventListener('resize', this._onResize);
      this._onResize = null;
    }

    resize() {
      const c = this.canvas;
      if (!c) return;
      /* 抽屉刚展开时宽度还没稳定，直接量会得到 0。
       * 兜底给 600×160，下一帧 resize 会纠正。 */
      const rect = c.getBoundingClientRect();
      const dpr = global.devicePixelRatio || 1;
      const w = Math.max(240, rect.width || this.box.clientWidth || 600);
      const h = Math.max(120, rect.height || this.box.clientHeight || 160);
      c.width = Math.round(w * dpr);
      c.height = Math.round(h * dpr);
      if (this.ctx && this.ctx.setTransform) this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      this.w = w;
      this.h = h;
    }

    setSat(satName, blName) {
      this.satName = satName || '';
      this.blName = blName || '';
    }

    setMetric(m) {
      if (METRICS[m]) { this.metric = m; this.render(); }
    }

    /** @param {Array<{ts:number,atmo:number,trpd:number,iond:number}>} rows */
    setData(rows) {
      this.series = Array.isArray(rows) ? rows : [];
      this.resize();
      this.render();
    }

    /* ------------------------------------------------------------ 渲染 */

    render() {
      const ctx = this.ctx;
      if (!ctx) return;
      const w = this.w || 600;
      const h = this.h || 160;
      ctx.clearRect(0, 0, w, h);

      const m = METRICS[this.metric] || METRICS.iond;
      const padL = 52, padR = 12, padT = 26, padB = 22;
      const plotW = w - padL - padR;
      const plotH = h - padT - padB;

      /* 该指标的有限值。**先过滤再取量程**：缺失值（null）若参与
       * min/max 计算，min 会变成 0 或 -Infinity，Y 轴范围直接失真。 */
      const vals = [];
      for (const p of this.series) {
        const v = p ? p[m.field] : null;
        if (typeof v === 'number' && Number.isFinite(v)) vals.push(v);
      }

      if (!vals.length) {
        this._renderEmpty(ctx, w, h, m);
        return;
      }

      let lo = Math.min.apply(null, vals);
      let hi = Math.max.apply(null, vals);
      if (lo === hi) { lo -= 0.5; hi += 0.5; }   // 单值/常量：给个量程否则线贴边

      /* 零轴：延迟有正有负，0 是「有无偏差」的分界，必须画出来。
       * 只有当 0 落在量程内才画 —— 否则会在画布外，无意义。 */
      const hasZero = lo <= 0 && hi >= 0;

      const yOf = (v) => padT + plotH - ((v - lo) / (hi - lo)) * plotH;
      const n = this.series.length;
      /* X 轴按**历元序号**均分而非按 ts 间隔：引擎可能丢帧（重连/超时），
       * 按时间轴均分会把断档拉成一条斜线，看着像连续演化，实则没有数据。 */
      const xOf = (i) => padL + (n <= 1 ? plotW / 2 : (i / (n - 1)) * plotW);

      this._grid(ctx, padL, padT, plotW, plotH, lo, hi, m, hasZero);
      this._curve(ctx, xOf, yOf, m, hasZero, lo, padT + plotH);
      this._labels(ctx, m, vals, padL, padT, plotW, w, h);
    }

    _renderEmpty(ctx, w, h, m) {
      ctx.fillStyle = '#8b97a6';
      ctx.font = '12px -apple-system, "Segoe UI", sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      const msg = this.series.length
        ? `${m.label}：暂无有效历元`
        : '正在累积历史数据，请稍候…';
      ctx.fillText(msg, w / 2, h / 2 - 8);
      ctx.font = '11px -apple-system, "Segoe UI", sans-serif';
      ctx.fillStyle = '#a8b2c0';
      ctx.fillText(
        this.satName ? `${this.blName} · ${this.satName}` : '',
        w / 2, h / 2 + 10);
      ctx.textAlign = 'left';
    }

    _grid(ctx, padL, padT, plotW, plotH, lo, hi, m, hasZero) {
      const rows = 4;
      ctx.strokeStyle = 'rgba(140,150,165,0.18)';
      ctx.lineWidth = 1;
      ctx.fillStyle = '#8b97a6';
      ctx.font = '10px ui-monospace, Consolas, monospace';
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      for (let i = 0; i <= rows; i++) {
        const v = lo + ((hi - lo) * i) / rows;
        const y = padT + plotH - (plotH * i) / rows;
        ctx.beginPath();
        ctx.moveTo(padL, y);
        ctx.lineTo(padL + plotW, y);
        ctx.stroke();
        ctx.fillText(v.toFixed(m.decimals), padL - 6, y);
      }
      /* Y 轴边框 */
      ctx.strokeStyle = 'rgba(140,150,165,0.35)';
      ctx.beginPath();
      ctx.moveTo(padL, padT);
      ctx.lineTo(padL, padT + plotH);
      ctx.stroke();
      /* X 轴基线 */
      ctx.beginPath();
      ctx.moveTo(padL, padT + plotH);
      ctx.lineTo(padL + plotW, padT + plotH);
      ctx.stroke();

      if (hasZero) {
        const y = padT + plotH - ((0 - lo) / (hi - lo)) * plotH;
        ctx.save();
        ctx.setLineDash([3, 3]);
        ctx.strokeStyle = U.hexToRgba(m.color, 0.55);
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(padL, y);
        ctx.lineTo(padL + plotW, y);
        ctx.stroke();
        ctx.restore();
      }
    }

    _curve(ctx, xOf, yOf, m, hasZero, lo, baseY) {
      const n = this.series.length;
      /* 连续段索引：null 会把折线断开（不跨过缺失点连线），
       * 否则「中间没数据」会被画成一条直线，看着像有值。 */
      const segs = [];
      let cur = [];
      for (let i = 0; i < n; i++) {
        const v = this.series[i] ? this.series[i][m.field] : null;
        if (typeof v === 'number' && Number.isFinite(v)) {
          cur.push({ x: xOf(i), y: yOf(v) });
        } else if (cur.length) {
          segs.push(cur); cur = [];
        }
      }
      if (cur.length) segs.push(cur);

      /* 面积填充（仅当量程含 0 时才有物理意义：从 0 填到曲线） */
      if (hasZero) {
        const y0 = yOf(0);
        for (const sgm of segs) {
          if (sgm.length < 2) continue;
          ctx.beginPath();
          ctx.moveTo(sgm[0].x, y0);
          for (const p of sgm) ctx.lineTo(p.x, p.y);
          ctx.lineTo(sgm[sgm.length - 1].x, y0);
          ctx.closePath();
          ctx.fillStyle = U.hexToRgba(m.color, 0.10);
          ctx.fill();
        }
      }

      ctx.strokeStyle = m.color;
      ctx.lineWidth = 1.6;
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      for (const sgm of segs) {
        ctx.beginPath();
        if (sgm.length === 1) {
          /* 单点不能画线（moveTo+lineTo 同点不可见），补一个向下的短竖线，
           * 与 trend.js 的处理一致：宁可显示"这里有个值"，也不要装作没数据。 */
          ctx.moveTo(sgm[0].x, sgm[0].y);
          ctx.lineTo(sgm[0].x, baseY);
        } else {
          ctx.moveTo(sgm[0].x, sgm[0].y);
          for (let i = 1; i < sgm.length; i++) ctx.lineTo(sgm[i].x, sgm[i].y);
        }
        ctx.stroke();
      }

      /* 末端点：标出最新值的位置 */
      for (const sgm of segs) {
        const p = sgm[sgm.length - 1];
        ctx.beginPath();
        ctx.arc(p.x, p.y, 2.6, 0, Math.PI * 2);
        ctx.fillStyle = m.color;
        ctx.fill();
      }
    }

    _labels(ctx, m, vals, padL, padT, plotW, w, h) {
      /* 标题 + 当前值 */
      ctx.textAlign = 'left';
      ctx.textBaseline = 'alphabetic';
      ctx.font = '600 11.5px -apple-system, "Segoe UI", sans-serif';
      ctx.fillStyle = m.color;
      const cur = vals[vals.length - 1];
      let title = m.label;
      if (this.satName) title = `${this.satName} · ${m.short}`;
      ctx.fillText(title, padL, 15);
      ctx.font = '11px ui-monospace, Consolas, monospace';
      ctx.fillStyle = '#6b7686';
      const stats = `当前 ${cur.toFixed(m.decimals)}  最小 ${Math.min.apply(null, vals).toFixed(m.decimals)}  最大 ${Math.max.apply(null, vals).toFixed(m.decimals)}`;
      ctx.textAlign = 'right';
      ctx.fillText(stats, w - 12, 15);

      /* X 轴说明 */
      ctx.font = '10px -apple-system, "Segoe UI", sans-serif';
      ctx.fillStyle = '#8b97a6';
      ctx.textAlign = 'left';
      ctx.fillText('← 较早', padL, h - 7);
      ctx.textAlign = 'right';
      ctx.fillText(`较近 →（${this.series.length} 历元）`, w - 12, h - 7);
      ctx.textAlign = 'left';
    }
  }

  /* ------------------------------------------------------------------ */

  /**
   * 从采集层的 satHistory 里取出某基线某卫星的曲线。
   *
   * key 形如 `BL:base→rover|PRN`（与 collector/index.js 的 recordSatHistory
   * 严格对应）。做**前缀匹配**而非精确拼 key，因为调用方通常只知道卫星名。
   *
   * 键里**没有频点后缀**（曾有 `PRNf<freq>`，已去掉）：引擎 solution.c 输出
   * 逐星行时三路延迟硬编码取 `[0]` 索引，与频点无关 —— 实测同一颗星的
   * f0/f1/f2 数值完全相同。按频点分组只会得到几条一模一样的曲线。
   *
   * @param {Object} satHistory 采集层 satHistory 全量
   * @param {string} baseName
   * @param {string} roverName
   * @param {string} satName    如 'G05'
   * @returns {Array<object>} 匹配到的曲线（正常为 0 或 1 条）
   */
  function seriesForSat(satHistory, baseName, roverName, satName) {
    if (!satHistory || !baseName || !roverName || !satName) return [];
    const bl = `${baseName}→${roverName}`;
    const prefix = `BL:${bl}|`;
    const out = [];
    for (const k of Object.keys(satHistory)) {
      if (!k.startsWith(prefix)) continue;
      /* 从**右**切出卫星名，而不是用正则从左往右找 `|`。
       * 站名来自引擎（NTRIP mountpoint），理论上可含 `|`；若站名里带了
       * `|`，从左找会把「站名的一段 + 卫星名」整段当成卫星名，
       * 于是每颗星都匹配不上、曲线永远空。 */
      const prn = k.slice(prefix.length);
      if (!prn || prn === k) continue;
      if (prn !== satName) continue;
      const rows = satHistory[k];
      if (!rows || !rows.length) continue;
      out.push({ key: k, prn, rows });
    }
    return out;
  }

  global.MCORS.SatDelayChart = SatDelayChart;
  global.MCORS.satDelay = {
    METRICS,
    seriesForSat
  };
})(window);
