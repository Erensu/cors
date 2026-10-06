/**
 * 卫星天空图（极坐标）
 *
 * 数据来源：NMEA GSV 报文中的方位角/仰角/SNR，
 * 以及 GSA 中的参与解算卫星列表（用于区分"可见"与"参与解算"）。
 */
(function (global) {
  'use strict';

  const U = global.MCORS.util;

  /** 星座配色 */
  const SYS_COLORS = {
    GPS: '#2563eb',
    GLO: '#dc2626',
    GAL: '#16a34a',
    BDS: '#8b5cf6',
    QZS: '#f59e0b',
    IRN: '#0ea5e9',
    SBS: '#64748b',
    MIX: '#64748b'
  };

  const SYS_NAMES = {
    GPS: 'GPS', GLO: 'GLONASS', GAL: 'Galileo',
    BDS: '北斗', QZS: 'QZSS', IRN: 'IRNSS', SBS: 'SBAS'
  };

  class SkyPlot {
    constructor(canvasId, selectId) {
      this.canvas = U.mountCanvas(canvasId);
      if (!this.canvas) return;
      this.ctx = this.canvas.getContext('2d');

      this.sats = [];
      this.usedSet = new Set();
      this.title = '';
      this.quality = null;
      this.hdop = null;
      this.pdop = null;
      this.vdop = null;
      this.numSat = null;
      this.caps = null;

      this.sel = document.getElementById(selectId);
      if (this.sel) {
        this.sel.onchange = () => {
          this.onSelect && this.onSelect(this.sel.value);
        };
      }

      this._bind();
      this._resize();
      this.render();
    }

    _bind() {
      const c = this.canvas;

      let hover = null;
      c.addEventListener('mousemove', (e) => {
        const r = c.getBoundingClientRect();
        const mx = e.clientX - r.left;
        const my = e.clientY - r.top;
        const hit = this.hitPoints.find(
          (p) => Math.hypot(p.x - mx, p.y - my) < 11
        );
        const newHover = hit ? hit.sat : null;
        if (newHover !== hover) {
          hover = newHover;
          c.style.cursor = hover ? 'help' : 'default';
          this.hoverSat = hover;
          this.render();
        }
        if (hover) {
          this._tooltip(hover, mx, my);
        } else {
          this._hideTooltip();
        }
      });

      c.addEventListener('mouseleave', () => {
        this.hoverSat = null;
        this._hideTooltip();
        this.render();
      });

      window.addEventListener('resize', () => {
        this._resize();
        this.render();
      });
    }

    _tooltip(sat, mx, my) {
      let el = document.getElementById('skyTip');
      if (!el) {
        el = document.createElement('div');
        el.id = 'skyTip';
        Object.assign(el.style, {
          position: 'fixed',
          zIndex: '400',
          background: 'rgba(28,37,48,.94)',
          color: '#fff',
          padding: '7px 10px',
          borderRadius: '6px',
          fontSize: '12px',
          pointerEvents: 'none',
          lineHeight: '1.6',
          whiteSpace: 'nowrap',
          boxShadow: '0 4px 14px rgba(0,0,0,.2)'
        });
        document.body.appendChild(el);
      }
      const used = this.usedSet.has(sat.label);
      el.innerHTML =
        `<b>${sat.label}</b> · ${SYS_NAMES[sat.sys] || sat.sys}<br>` +
        `仰角 ${U.num(sat.el, 1)}°　方位 ${U.num(sat.az, 1)}°<br>` +
        `SNR ${U.num(sat.snr, 0)} dB-Hz<br>` +
        (used ? '<span style="color:#86efac">参与解算</span>' : '<span style="color:#cbd5e1">未参与解算</span>');
      el.style.display = 'block';
      const rect = this.canvas.getBoundingClientRect();
      let x = rect.left + mx + 14;
      let y = rect.top + my - 10;
      if (x + 170 > window.innerWidth) x = rect.left + mx - 175;
      el.style.left = x + 'px';
      el.style.top = y + 'px';
    }

    _hideTooltip() {
      const el = document.getElementById('skyTip');
      if (el) el.style.display = 'none';
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

    /** 更新数据 */
    setData(payload, caps) {
      if (caps) this.caps = caps;
      if (!payload) {
        this.sats = [];
        this.usedSet = new Set();
        this.title = '';
        this.quality = null;
      } else {
        this.sats = payload.sats || [];
        this.usedSet = new Set(payload.usedSats || []);
        this.title = payload.title || '';
        this.quality = payload.qualityLabel || null;
        this.hdop = payload.hdop;
        this.pdop = payload.pdop;
        this.vdop = payload.vdop;
        this.numSat = payload.numSat;
      }
      this.render();
    }

    /** 同步站点下拉选项 */
    setOptions(stations, current) {
      if (!this.sel) return;
      const prev = current || this.sel.value;
      this.sel.innerHTML = '<option value="">选择站点</option>';
      (stations || []).forEach((s) => {
        const o = document.createElement('option');
        o.value = s.name;
        o.textContent = s.name + (s.cnName ? ` (${s.cnName})` : '');
        this.sel.appendChild(o);
      });
      if (prev && this.sel.querySelector(`option[value="${CSS.escape(prev)}"]`)) {
        this.sel.value = prev;
      }
    }

    render() {
      const ctx = this.ctx;
      if (!ctx) return;
      ctx.clearRect(0, 0, this.w, this.h);

      /* 布局改为「圆在左、分系统条形图在右」。
       *
       * 用户反馈天空图偏空，且与基站信息对调位置后希望利用多出的宽度。
       * 圆径本来由**高度**决定（R = min(宽, 高/2-26) - 12），
       * 居中摆放时右侧一大片空白纯浪费；左对齐后右侧可放
       * 各星座的可见/参与星数条形图 —— 这是判断「哪个系统在拖后腿」
       * 最直接的信息，原界面要切换到别处才能看到。
       *
       * 条形图区宽取 min(240, 38%)：够放「BDS ▓▓▓▓▓ 6/9」这样的行，
       * 又不至于挤压圆图。 */
      const barW = Math.min(240, this.w * 0.38);
      const skyW = this.w - barW - 20;          // 圆可用宽（留 20 间隔）
      const R = Math.min(skyW / 2, this.h / 2 - 26) - 12;
      /* 左边距 22px：方位标注 W/E/N/S 画在圆外侧，12px 时
       * 「W」会被画布左边缘裁掉（实测）。 */
      const cx = 22 + R;                         // 圆左对齐
      const cy = this.h / 2 + 6;

      this._drawBackground(ctx, cx, cy, R);

      if (!this.sats.length) {
        // 区分"上游没给观测值"与"有数据但当前无可见星"
        let msg = '暂无卫星数据';
        if (this.caps && this.caps.code === 'EPH_ONLY') {
          msg = '上游仅推星历，无观测值';
        } else if (this.caps && this.caps.code === 'NO_MONITOR') {
          msg = '未连接 monitor 服务';
        } else if (this.caps && this.caps.code === 'NO_FRAMES') {
          msg = '已连接但无数据帧';
        } else if (this.caps && this.caps.code === 'MOCK') {
          msg = 'mock 模式 · 暂无卫星数据';
        }
        ctx.fillStyle = '#8b97a6';
        ctx.font = '13px -apple-system, sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(msg, cx, cy);
        return;
      }

      this.hitPoints = [];

      // 仰角圈：30/60/90
      ctx.strokeStyle = '#e8ecf2';
      ctx.fillStyle = '#a8b3c0';
      ctx.lineWidth = 1;
      ctx.font = '10px ui-monospace, monospace';

      for (let el = 0; el <= 90; el += 30) {
        if (el === 0) continue;
        const r = R * (1 - el / 90);
        ctx.beginPath();
        ctx.arc(cx, cy, r, 0, Math.PI * 2);
        ctx.stroke();
        ctx.textAlign = 'left';
        ctx.textBaseline = 'bottom';
        ctx.fillText(el + '°', cx + 3, cy - r + 11);
      }

      // 方位刻度线
      ctx.strokeStyle = '#eef1f5';
      for (let az = 0; az < 360; az += 30) {
        const rad = ((az - 90) * Math.PI) / 180;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + R * Math.cos(rad), cy + R * Math.sin(rad));
        ctx.stroke();
      }

      // 方位标签（N/E/S/W）
      ctx.fillStyle = '#7b8794';
      ctx.font = '600 11.5px -apple-system, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      const dirs = [['N', 0], ['E', 90], ['S', 180], ['W', 270]];
      for (const [t, az] of dirs) {
        const rad = ((az - 90) * Math.PI) / 180;
        const r = R + 15;
        ctx.fillText(t, cx + r * Math.cos(rad), cy + r * Math.sin(rad));
      }

      // 绘制卫星
      for (const sat of this.sats) {
        if (!Number.isFinite(sat.az) || !Number.isFinite(sat.el)) continue;

        // 仰角 <=0 或 >=90 的卫星不画在地平线上
        if (sat.el < 0 || sat.el > 90) continue;

        const r = R * (1 - sat.el / 90);
        const rad = ((sat.az - 90) * Math.PI) / 180;
        const x = cx + r * Math.cos(rad);
        const y = cy + r * Math.sin(rad);

        const used = this.usedSet.has(sat.label);
        const color = SYS_COLORS[sat.sys] || SYS_COLORS.MIX;

        // 卫星点：参与解算的实心+外环，未参与的半透明
        ctx.beginPath();
        ctx.arc(x, y, used ? 6 : 4.5, 0, Math.PI * 2);
        ctx.fillStyle = used ? color : U.hexToRgba(color, 0.42);
        ctx.fill();

        ctx.strokeStyle = used ? '#fff' : U.hexToRgba(color, 0.6);
        ctx.lineWidth = used ? 1.6 : 1;
        ctx.stroke();

        // 悬停高亮
        if (this.hoverSat && this.hoverSat.label === sat.label) {
          ctx.beginPath();
          ctx.arc(x, y, 9, 0, Math.PI * 2);
          ctx.strokeStyle = '#111827';
          ctx.lineWidth = 1.5;
          ctx.stroke();
        }

        // PRN 标签
        ctx.font = '9.5px ui-monospace, monospace';
        ctx.fillStyle = used ? '#1c2530' : '#8b97a6';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(String(sat.prn), x, y - 10);

        this.hitPoints.push({ x, y, sat });
      }

      this._drawHeader(ctx);

      /* 分系统星数条形图（右侧空白区）。
       * 放在 render 而非 _drawHeader 里：barW 是本函数的局部变量，
       * _drawHeader 看不到它（此前放在那边会抛
       * ReferenceError: barW is not defined，异常中断 render，
       * 连带底部图例一起不画）。条形图本也不属于"头部信息"，
       * 由 render 统一调度更合理。 */
      this._drawSysBars(ctx, this.w - barW - 4);
    }

    /**
     * 绘制天顶圆底色与地平圈。
     *
     * 极坐标约定：圆心为天顶，半径 R 对应仰角 0°（地平线），
     * 半径 0 对应仰角 90°（天顶）。
     */
    _drawBackground(ctx, cx, cy, R) {
      // 天顶圆（外缘即地平圈）淡色填充，衬托卫星点
      ctx.beginPath();
      ctx.arc(cx, cy, R, 0, Math.PI * 2);
      ctx.fillStyle = '#f8fafc';
      ctx.fill();

      // 地平圈
      ctx.beginPath();
      ctx.arc(cx, cy, R, 0, Math.PI * 2);
      ctx.strokeStyle = '#d4dbe4';
      ctx.lineWidth = 1.2;
      ctx.stroke();

      // 天顶十字参考线
      ctx.strokeStyle = '#eef1f5';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(cx - R, cy);
      ctx.lineTo(cx + R, cy);
      ctx.moveTo(cx, cy - R);
      ctx.lineTo(cx, cy + R);
      ctx.stroke();
    }

    /* 顶部信息条与底部图例都按**整幅画布**居中，不按圆心。
     * 圆为了给右侧条形图腾位已左对齐（cx≈121），若这些面板级文字
     * 跟着圆心居中，长内容（如「A003 · RTK 固定解 | 可见 12 | PDOP…」）
     * 会从左侧溢出画布被裁掉 —— 实测确实发生了。
     * 它们描述的是整个面板，本就该相对面板居中。
     *
     * 注：cx 参数已不再需要，保留签名仅为向后兼容调用处。 */
    _drawHeader(ctx) {
      // 顶部信息条
      const parts = [];
      if (this.title) parts.push(this.title);
      if (this.quality) parts.push(this.quality);
      if (this.numSat !== null && this.numSat !== undefined) parts.push(`可见 ${this.numSat}`);
      /* DOP 保留 2 位小数：1 位会把 0.9 / 1.4 这类差异抹平成同一个值。
       * 同时给出 PDOP（GSA 的三维精度），它比 HDOP 更能反映解算质量。 */
      /* 三个 DOP 一起展示：PDOP（三维综合）、HDOP（水平）、
       * VDOP（垂直 —— 高程解算质量的直接指标，固定解高程最怕它变大）。 */
      if (this.pdop !== null && this.pdop !== undefined) parts.push(`PDOP ${U.num(this.pdop, 2)}`);
      if (this.hdop !== null && this.hdop !== undefined) parts.push(`HDOP ${U.num(this.hdop, 2)}`);
      if (this.vdop !== null && this.vdop !== undefined) parts.push(`VDOP ${U.num(this.vdop, 2)}`);

      /* 顶部信息行。
       * 注意这里是**条件绘制**而不是 `if (!parts.length) return;` ——
       * 原来的提前 return 会在没有 DOP 数据（引擎未解算）时直接结束
       * render，把后面的分系统条形图和底部图例一起丢掉。
       * 实测现象：天空图右侧空白、底部图例消失，但单独手动调用
       * _drawSysBars 又完全正常（因为绕过了这个 return）。 */
      if (parts.length) {
        ctx.fillStyle = '#5a6675';
        ctx.font = '11.5px -apple-system, sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'top';
        ctx.fillText(parts.join('　|　'), this.w / 2, 4);
      }

      // 底部图例（实际出现过的星座），同样改为条件绘制
      const sysList = [...new Set(this.sats.map((s) => s.sys))].filter(Boolean);
      if (sysList.length) {
        let totalW = 0;
        ctx.font = '10.5px -apple-system, sans-serif';
        for (const s of sysList) totalW += ctx.measureText(SYS_NAMES[s] || s).width + 20;

        let x = this.w / 2 - totalW / 2;
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';
        for (const s of sysList) {
          const label = SYS_NAMES[s] || s;
          ctx.beginPath();
          ctx.arc(x + 4, this.h - 10, 4, 0, Math.PI * 2);
          ctx.fillStyle = SYS_COLORS[s] || SYS_COLORS.MIX;
          ctx.fill();
          ctx.fillStyle = '#5a6675';
          ctx.fillText(label, x + 12, this.h - 10);
          x += ctx.measureText(label).width + 20;
        }
      }
    }

    /**
     * 右侧「各系统星数」条形图。
     *
     * 每个星座一行，双段条形：
     *   浅色 = 可见星数（轨道全长按最大可见数归一）
     *   实色 = 参与解算星数
     * 右侧标注「参与/可见」。
     *
     * 为什么值得画：定位质量差时，第一个要问的是"是哪个系统星不够"。
     * 原界面只有圆图上的点，得靠数点判断；条形图把
     * 「BDS 9 可见 / 6 参与」这类信息直接摊开，
     * 一眼能看出是某系统整体缺星，还是有星但没被采用。
     */
    _drawSysBars(ctx, x0) {
      const sats = this.sats;
      if (!sats || !sats.length) return;

      const stat = new Map();
      for (const s of sats) {
        const k = s.sys || 'MIX';
        if (!stat.has(k)) stat.set(k, { vis: 0, used: 0 });
        const o = stat.get(k);
        o.vis++;
        if (this.usedSet.has(s.label)) o.used++;
      }
      if (!stat.size) return;

      // 可见数降序：星最多的系统排最上，符合扫读习惯
      const rows = [...stat.entries()].sort((a, b) => b[1].vis - a[1].vis);
      const maxVis = Math.max(...rows.map((r) => r[1].vis), 1);

      const labelW = 36, valW = 40;
      const trackX = x0 + labelW + 6;
      const trackW = Math.max(40, this.w - 14 - valW - trackX);

      const rowH = 26;
      const startY = (this.h - rows.length * rowH) / 2;

      // 小标题
      ctx.font = '600 10.5px -apple-system, sans-serif';
      ctx.fillStyle = '#8b97a6';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'bottom';
      ctx.fillText('各系统星数（参与/可见）', x0, startY - 6);

      rows.forEach(([sys, o], i) => {
        const cy = startY + i * rowH + rowH / 2;
        const color = SYS_COLORS[sys] || SYS_COLORS.MIX;

        // 系统名
        ctx.font = '600 11px -apple-system, sans-serif';
        ctx.fillStyle = '#5a6675';
        ctx.textAlign = 'right';
        ctx.textBaseline = 'middle';
        ctx.fillText(SYS_NAMES[sys] || sys, x0 + labelW, cy);

        // 轨道（可见数）
        const fullW = (o.vis / maxVis) * trackW;
        const usedW = (o.used / maxVis) * trackW;
        const bh = 11, by = cy - bh / 2;

        ctx.fillStyle = '#eef1f5';
        ctx.fillRect(trackX, by, fullW, bh);

        // 参与数（实色）；用低透明度表示"有星未用"的部分
        if (usedW > 0) {
          ctx.fillStyle = color;
          ctx.fillRect(trackX, by, usedW, bh);
        }

        /* 数值：两者不等时把"未参与"标出来 —— 有星没被采用
         * 往往意味着该星高度角过低或被剔除，是排查的重点。 */
        /* 只有**一颗都没参与**才用橙色告警。
         * 不能用「有星未参与」作条件 —— 正常解算里低仰角星本就不参与，
         * 那种判据会让每个系统常年橙色，提示失去意义。
         * 全未参与才是真异常（系统级故障或该星座被整体剔除）。 */
        ctx.font = '600 10.5px ui-monospace, monospace';
        ctx.textAlign = 'left';
        ctx.fillStyle = (o.used === 0 && o.vis > 0) ? '#b45309' : '#334155';
        ctx.fillText(o.used + '/' + o.vis, trackX + trackW + 6, cy);
      });
    }
  }

  global.MCORS.SkyPlot = SkyPlot;
  global.MCORS.SYS_COLORS = SYS_COLORS;
})(window);