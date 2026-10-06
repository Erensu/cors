/**
 * 定位质量趋势图（原生 Canvas 折线图，零依赖）
 * 支持 HDOP 与参与星数两种指标切换
 */
(function (global) {
  'use strict';

  const U = global.MCORS.util;

  class TrendChart {
    constructor(canvasId, selectId) {
      this.canvas = U.mountCanvas(canvasId);
      if (!this.canvas) return;
      this.ctx = this.canvas.getContext('2d');

      this.series = [];        // [{t, v}]
      this.metric = 'hdop';

      /* 指标定义集中在此，避免渲染各处重复三元判断。
       * dop 类指标从 0 起（视觉更诚实），星数类按数据范围留 2 的余量。
       * decimals：DOP 常见值域 0.5~30，1 位小数会把波动全抹平
       * （实测引擎 GGA 的 HDOP 长期写死 1.0，曲线曾是一条直线）。 */
      this.metrics = {
        hdop:     { label: 'HDOP',     field: 'hdop',    color: '#2563eb', dop: true,  decimals: 2 },
        pdop:     { label: 'PDOP',     field: 'pdop',    color: '#7c3aed', dop: true,  decimals: 2 },
        hdopGsa:  { label: 'HDOP(GSA)', field: 'hdopGsa', color: '#0891b2', dop: true,  decimals: 2 },
        vdop:     { label: 'VDOP',     field: 'vdop',    color: '#db2777', dop: true,  decimals: 2 },
        numSat:   { label: '参与解算星数', field: 'numSat', color: '#16a34a', dop: false, decimals: 0 }
      };
      this.label = this.metrics[this.metric].label;

      this.sel = document.getElementById(selectId);
      if (this.sel) {
        this.sel.onchange = () => {
          this.metric = this.sel.value;
          const m = this.metrics[this.metric];
          this.label = m ? m.label : this.metric;
          this.render();
        };
      }

      window.addEventListener('resize', () => {
        this._resize();
        this.render();
      });

      this._resize();
    }

    _resize() {
      const c = this.canvas;
      const rect = c.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      c.width = Math.max(1, Math.round(rect.width * dpr));
      c.height = Math.max(1, Math.round(rect.height * dpr));
      this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      this.w = rect.width;
      this.h = rect.height;
    }

    setData(points) {
      this.series = points || [];
      this.render();
    }

    render() {
      const ctx = this.ctx;
      if (!ctx) return;
      ctx.clearRect(0, 0, this.w, this.h);

      /* padT 从 14 增到 40：顶部腾出一行放「数值摘要」。
       * 用户反馈趋势图偏空 —— 折线本身信息量有限（只看得到形状），
       * 补一行统计量（当前/最小/最大/平均/波动）能让「这段数据到底
       * 稳不稳」一眼可判，比只看曲线更实用。 */
      const padL = 42, padR = 14, padT = 40, padB = 26;
      const plotW = this.w - padL - padR;
      const plotH = this.h - padT - padB;

      if (!this.series.length) {
        ctx.fillStyle = '#8b97a6';
        ctx.font = '13px -apple-system, sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('暂无趋势数据', this.w / 2, this.h / 2);
        return;
      }

      const M = this.metrics[this.metric] || this.metrics.hdop;
      const field = M.field;
      const vals = this.series
        .map((p) => p[field])
        .filter((v) => v !== null && v !== undefined && isFinite(v));

      if (!vals.length) {
        ctx.fillStyle = '#8b97a6';
        ctx.font = '13px -apple-system, sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('暂无有效数据', this.w / 2, this.h / 2);
        return;
      }

      let min = Math.min(...vals);
      let max = Math.max(...vals);

      if (M.dop) {
        /* DOP 是**有意义的绝对量**（0=理想几何，1~2=良好，>5=很差），
         * 纵轴从 0 起能如实表达"这个值算不算好"。
         *
         * 但一刀切从 0 起会有两个问题：
         *   1) 数据集中在高位时（如 VDOP 常年 1.2~1.5）曲线被压在顶部，
         *      80% 画布留白，波动完全看不出来；
         *   2) 数据本身跨度很小时（如 HDOP 在 0.8~1.0 抖动），
         *      从 0 起会把 0.2 的抖动压成一条直线。
         *
         * 故改为**按数据实际跨度留白**，下限仍不小于 0：
         * 0.8~1.0 → 轴范围约 0.76~1.04，0.2 的抖动占满画布，
         * 变化清晰可辨；同时因下限被 Math.max(0, ...) 兜住，
         * 不会把"0=理想"这个语义丢掉。
         * 最小跨度 0.2：小于此值多为引擎写死的常量（如 GGA 的 HDOP=1.0），
         * 放大也是噪声。 */
        const span = Math.max(max - min, 0.2);
        const pad = span * 0.15;
        min = Math.max(0, min - pad);
        max = max + pad;
      } else {
        /* 整数指标（参与解算星数）。
         * 原先固定留 ±2 的余量，恒定值（如一直是 12 颗）会得到
         * 10~14 的轴、占用率 0%，曲线压成一条直线看不出任何信息。
         * 改为按实际跨度留白：跨度为 0 时给 ±1（画出上下两条边界，
         * 让"平线在中间"可读），有波动时留 1 颗余量即可。 */
        const span = max - min;
        const pad = span > 0 ? Math.max(1, span * 0.15) : 1;
        min = Math.max(0, Math.floor(min - pad));
        max = Math.ceil(max + pad);
      }
      if (max - min < 1e-9) max = min + 1;

      /* 刻度步长：对数档位取整（1/2/2.5/5 × 10^n），
       * 让每个格子的数值都是"好看"的读数。
       * 目标 8 格 —— 原先 4 格太粗，HDOP 0.05 级的抖动跨不到半格，
       * 曲线看着像直线。8 格配 2.5 的步长档，最小可分辨 0.02。 */
      const rawStep = (max - min) / 8;
      const mag = Math.pow(10, Math.floor(Math.log10(rawStep)));
      const norm = rawStep / mag;
      let step = (norm <= 1 ? 1 : norm <= 2 ? 2 : norm <= 2.5 ? 2.5 : norm <= 5 ? 5 : 10) * mag;
      /* 整数指标的步长必须是整数：否则会出现"11.5 颗星"这种
       * 不存在的读数（实测星数恒为 12 时步长算成 0.5，标签出 11/12/13
       * 混排）。这里向上取整到 1 的倍数。 */
      if (!M.dop) step = Math.max(1, Math.round(step));
      min = Math.floor(min / step) * step;
      max = Math.ceil(max / step) * step;

      // 步长变了，格数要跟着变（原来固定 4，改完步长对不上会画歪）
      const ticks = Math.max(2, Math.round((max - min) / step));

      // Y 轴刻度
      ctx.strokeStyle = '#eef1f5';
      ctx.fillStyle = '#8b97a6';
      ctx.lineWidth = 1;
      ctx.font = '10px ui-monospace, monospace';
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';

      for (let i = 0; i <= ticks; i++) {
        const v = min + step * i;
        const y = padT + plotH - (plotH * i) / ticks;
        ctx.beginPath();
        ctx.moveTo(padL, y);
        ctx.lineTo(padL + plotW, y);
        ctx.stroke();
        /* 小数位跟着步长走：步长 0.02 时若仍固定 2 位，
         * 相邻格子会显示成同一个数（0.98/1.00/1.02 之外还有 1.019），
         * 用户读不出差异。 */
        const dec = M.dop ? Math.min(3, Math.max(M.decimals, Math.ceil(-Math.log10(step)))) : 0;
        ctx.fillText(M.dop ? v.toFixed(dec) : String(Math.round(v)), padL - 6, y);
      }

      /* 次网格线：每个主格再细分 2 份，用更淡的灰色。
       * 只在纵向够高时画 —— 格子高度不足 14px 时次网格会挤成一团灰带，
       * 反而更难读（实测 8 主格 + 次格在 324px 高的面板里最合适）。 */
      const cellH = plotH / ticks;
      if (cellH >= 14) {
        ctx.strokeStyle = '#f6f8fa';
        ctx.beginPath();
        for (let i = 0; i < ticks; i++) {
          for (let k = 1; k < 2; k++) {
            const y = padT + plotH - (plotH * (i + k / 2)) / ticks;
            ctx.moveTo(padL, y);
            ctx.lineTo(padL + plotW, y);
          }
        }
        ctx.stroke();
      }

      // X 轴时间标签（首尾各一个）
      const first = this.series[0].ts;
      const last = this.series[this.series.length - 1].ts;
      ctx.fillStyle = '#8b97a6';
      ctx.textBaseline = 'top';
      ctx.textAlign = 'left';
      ctx.fillText(U.clock(first), padL, padT + plotH + 7);
      ctx.textAlign = 'right';
      ctx.fillText(U.clock(last), padL + plotW, padT + plotH + 7);

      const n = this.series.length;
      const xAt = (i) => padL + (n === 1 ? plotW / 2 : (plotW * i) / (n - 1));
      const yAt = (v) => padT + plotH - ((v - min) / (max - min)) * plotH;

      // 面积填充
      const valid = [];
      for (let i = 0; i < n; i++) {
        const v = this.series[i][field];
        if (v !== null && v !== undefined && isFinite(v)) valid.push({ x: xAt(i), y: yAt(v) });
      }

      if (valid.length >= 1) {
        const color = M.color;

        const grad = ctx.createLinearGradient(0, padT, 0, padT + plotH);
        grad.addColorStop(0, U.hexToRgba(color, 0.22));
        grad.addColorStop(1, U.hexToRgba(color, 0.01));

        /* 面积填充与折线需要 ≥2 个点。
         * 但刚启动时每组只有 1 个点，若此处直接跳过，
         * 面板在积累到第2 个点之前**完全空白** —— 用户看到的就是
         *「没有曲线」。故单点时只画标记点，让面板从第一帧起就有内容。 */
        if (valid.length > 1) {
          ctx.beginPath();
          ctx.moveTo(valid[0].x, padT + plotH);
          valid.forEach((p) => ctx.lineTo(p.x, p.y));
          ctx.lineTo(valid[valid.length - 1].x, padT + plotH);
          ctx.closePath();
          ctx.fillStyle = grad;
          ctx.fill();

          // 折线
          ctx.beginPath();
          valid.forEach((p, i) => (i === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y)));
          ctx.strokeStyle = color;
          ctx.lineWidth = 2;
          ctx.lineJoin = 'round';
          ctx.lineCap = 'round';
          ctx.stroke();
        } else {
          /* 单点：从该点向下画一条淡竖线到基线，
           * 让「只有一个样本」在视觉上也可读。 */
          ctx.beginPath();
          ctx.moveTo(valid[0].x, valid[0].y);
          ctx.lineTo(valid[0].x, padT + plotH);
          ctx.strokeStyle = U.hexToRgba(color, 0.35);
          ctx.lineWidth = 1;
          ctx.setLineDash([3, 3]);
          ctx.stroke();
          ctx.setLineDash([]);
        }

        // 末端点
        const lastP = valid[valid.length - 1];
        ctx.beginPath();
        ctx.arc(lastP.x, lastP.y, 3.5, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        /* 当前值标注：取最后一个**有效**点。
         * 原先直接取 series[n-1]，末尾若是 null 会显示成 NaN；
         * 这里改为在 valid 里找对应的原始索引，取最后一个有限值。 */
        let curRaw = null;
        for (let i = n - 1; i >= 0; i--) {
          const v = this.series[i][field];
          if (v !== null && v !== undefined && isFinite(v)) { curRaw = v; break; }
        }
        const curVal = M.dop
          ? Number(curRaw).toFixed(M.decimals)
          : String(Math.round(Number(curRaw)));
        const label = `${this.label} ${curVal}`;
        ctx.font = '600 11px -apple-system, sans-serif';
        const tw = ctx.measureText(label).width;
        let lx = lastP.x - tw - 12;
        if (lx < padL) lx = lastP.x + 8;
        ctx.fillStyle = color;
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';
        ctx.fillText(label, lx, Math.max(padT + 7, lastP.y));
      }

      /* 顶部数值摘要条。
       *
       * 折线只能看出"形状"，判断不了"这段数据稳不稳"。补统计量后，
       * 「平均 0.71、波动 0.37」这类结论不用眼睛估。
       *
       * 波动 = max - min，是判断质量抖动的核心指标：
       * DOP 波动大意味着几何条件在剧烈变化（卫星进出、遮挡），
       * 比单看当前值更有诊断价值。
       *
       * 布局：指标名（带色点）在左，五个统计项依次排开，用竖线分隔。
       * 颜色跟当前指标一致，与折线颜色呼应。 */
      const cur = Number(vals[vals.length - 1]);
      const sum = vals.reduce((a, b) => a + b, 0);
      const avg = sum / vals.length;
      const dmin = Math.min(...vals);
      const dmax = Math.max(...vals);
      const fmt = (v) => (M.dop ? Number(v).toFixed(M.decimals) : String(Math.round(v)));

      ctx.textBaseline = 'middle';
      let sx = padL;
      const sy = 14;

      // 指标名 + 色点
      ctx.beginPath();
      ctx.arc(sx + 3.5, sy, 3.5, 0, Math.PI * 2);
      ctx.fillStyle = M.color;
      ctx.fill();
      sx += 12;
      ctx.fillStyle = '#5a6675';
      ctx.font = '600 11px -apple-system, sans-serif';
      ctx.textAlign = 'left';
      ctx.fillText(this.label, sx, sy);
      sx += ctx.measureText(this.label).width + 10;

      /* 统计项：标签浅灰、数值深色，便于快速扫读数值 */
      const items = M.dop
        ? [['当前', fmt(cur)], ['最小', fmt(dmin)], ['最大', fmt(dmax)],
           ['平均', fmt(avg)], ['波动', fmt(dmax - dmin)]]
        : [['当前', fmt(cur)], ['最小', fmt(dmin)], ['最大', fmt(dmax)],
           ['平均', fmt(avg)]];

      ctx.font = '10.5px -apple-system, sans-serif';
      for (const [k, v] of items) {
        // 分隔线（除首项外）
        ctx.strokeStyle = '#e3e8ee';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(sx, sy - 6);
        ctx.lineTo(sx, sy + 6);
        ctx.stroke();
        sx += 8;

        ctx.fillStyle = '#8b97a6';
        ctx.fillText(k, sx, sy);
        sx += ctx.measureText(k).width + 3;

        ctx.fillStyle = '#334155';
        ctx.font = '600 10.5px ui-monospace, monospace';
        ctx.fillText(v, sx, sy);
        sx += ctx.measureText(v).width + 10;
        ctx.font = '10.5px -apple-system, sans-serif';
      }
    }
  }

  global.MCORS.TrendChart = TrendChart;
})(window);