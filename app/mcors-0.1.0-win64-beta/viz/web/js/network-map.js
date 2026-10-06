/**
 * 基站网图
 *
 * 用原生 Canvas 绘制，零外部依赖。实现要点：
 *  - 站网按经纬度做等距圆柱投影，再缩放平移到画布
 *  - **三角网**：对基站做 Delaunay 三角剖分（见 delaunay.js），
 *    这对应引擎侧 src/dtrignet/dtrignet.c 的站网结构（Shewchuk triangle.c）
 *  - 基线：引擎实际在解的基线，以连线+状态色叠加在三角网之上
 *  - 站点可点击选中，选中态在信息面板中联动
 *  - 支持拖拽平移与滚轮缩放，便于查看密集区域
 *
 * 为什么三角网要在「平面」上算：
 *   Delaunay 的数学基础是欧氏平面。若直接拿经纬度当平面坐标，
 *   经度方向的尺度会随纬度被高估（赤道处 1° 经度约 111km，
 *   北纬 60° 处只有 55km），三角剖分会被拉歪。
 *   因此先做等距圆柱投影：x = lon·cos(φ₀)，y = lat，φ₀ 取站网中心纬度。
 */
(function (global) {
  'use strict';

  const U = global.MCORS.util;
  const D = global.MCORS.delaunay;

  /* 虚拟站（VRS 虚拟参考点）的专用色。
   *
   * 选紫色的理由：util.js 的 COLORS 里 8 种解状态色
   * （fix绿 / fixdeg青 / float橙 / ppp紫 / dgps蓝 / sbas紫 / single灰 / dr灰）
   * 已被占用，其中 ppp 与 sbas 也在紫色系。故取偏冷的**蓝紫 #7c3aed**，
   * 并靠「菱形 + 虚线光环」的形状差异兜底 —— 即便颜色与 ppp 接近，
   * 形状仍能区分，不会混淆。 */
  const VRS_COLOR = '#7c3aed';

  class NetworkMap {
    constructor(canvasId) {
      this.canvas = U.mountCanvas(canvasId);
      if (!this.canvas) return;
      this.ctx = this.canvas.getContext('2d');

      this.stations = [];
      this.baselines = [];
      this.showLabels = true;
      this.showCnNames = false;
      this.showBaselines = true;
      // 三角网开关：站网形态视图 / 仅基线视图
      this.showTrinet = true;
      // 三角网面片底纹（填色）与网形边线
      this.showTrinetFill = true;

      this.trinet = { triangles: [], edges: [] };

      this.selected = null;   // 选中的站名
      this.onSelect = null;   // 选中回调

      // 视图变换
      this.scale = 1;
      this.offsetX = 0;
      this.offsetY = 0;
      this.baseScale = 1;
      this.baseOffsetX = 0;
      this.baseOffsetY = 0;
      this.autoFit = true;

      this.hitPoints = [];   // 命中测试用

      this._bindEvents();
      this._resize();
    }

    _bindEvents() {
      const c = this.canvas;

      c.addEventListener('mousedown', (e) => {
        this.dragging = true;
        this.dragMoved = false;
        this.lastX = e.offsetX;
        this.lastY = e.offsetY;
      });

      window.addEventListener('mouseup', (e) => {
        if (!this.dragging) return;
        this.dragging = false;
        // 拖动幅度小则视为点击
        if (!this.dragMoved && this.canvas.contains(e.target)) {
          const r = this.canvas.getBoundingClientRect();
          const mx = e.clientX - r.left;
          const my = e.clientY - r.top;
          this._handleClick(mx, my);
        }
      });

      c.addEventListener('mousemove', (e) => {
        /* 悬停提示：颜色之外再给一条文字通道，
         * 否则色弱用户无法确认某条线究竟是什么解状态。 */
        if (!this.dragging) {
          this._updateHover(e.offsetX, e.offsetY);
          return;
        }
        const dx = e.offsetX - this.lastX;
        const dy = e.offsetY - this.lastY;
        if (Math.abs(dx) > 2 || Math.abs(dy) > 2) this.dragMoved = true;
        this.offsetX += dx;
        this.offsetY += dy;
        this.lastX = e.offsetX;
        this.lastY = e.offsetY;
        this.autoFit = false;
        this.render();
      });

      c.addEventListener(
        'wheel',
        (e) => {
          e.preventDefault();
          const factor = e.deltaY < 0 ? 1.12 : 1 / 1.12;
          const rect = this.canvas.getBoundingClientRect();
          const mx = e.offsetX;
          const my = e.offsetY;
          // 以鼠标位置为锚点缩放
          const worldX = (mx - this.offsetX) / this.scale;
          const worldY = (my - this.offsetY) / this.scale;
          this.scale = Math.max(0.2, Math.min(60, this.scale * factor));
          this.offsetX = mx - worldX * this.scale;
          this.offsetY = my - worldY * this.scale;
          this.autoFit = false;
          this.render();
        },
        { passive: false }
      );

      window.addEventListener('resize', () => {
        this._resize();
        /* 画布尺寸变了，baseOffsetX/Y（= w/2, h/2）与 baseScale
         * （依赖 w、h）都必须重算，否则视野会偏。
         * 这是必要的重算，与 setData 里被跳过的那些不是一回事。 */
        this._computeBase();
        this.render();
      });
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
    setData(stations, baselines, trinet) {
      this._allStations = (stations || []).filter(
        (s) => Number.isFinite(s.lat) && Number.isFinite(s.lon)
      );
      this.stations = this._allStations;
      this.baselines = baselines || [];
      // 引擎侧权威三角网（来自 showtrinet）。有它就不再自己重算 ——
      // 引擎用的是考虑业务约束的 triangle.c，与自己按坐标跑 Delaunay
      // 得到的网形可能不同，运维判断应以引擎为准。
      this._engineTrinet = trinet && trinet.nodes && trinet.nodes.length ? trinet : null;

      /* 视野稳定性：只在站点集合**真正变化**时重算。
       *
       * 原来每次 setData（WS 每次推送，1~3s 一次）都调 _computeBase()，
       * 而它按 minLat/maxLat 算全局适配。问题有两层：
       *   1) 运行期 addsource/delsource 会改变站点集合 → 视野突变，
       *      用户正看着的站网图会「跳一下」，基线随之重绘，
       *      观感就是"基线跳来跳去"；
       *   2) 即便站点集合不变，浮点累加误差也会让 scale 每次略有差异，
       *      同样表现为持续抖动。
       * 故用「站名+坐标的签名」判断是否需要重算：
       * 数据没变就复用上次的 baseScale/center，完全不碰视野。
       * 用户点「适配视野」仍走 fitView()，可随时手动重算。 */
      if (this._geoSignature() !== this._lastGeoSig) {
        this._lastGeoSig = this._geoSignature();
        this._computeBase();
      }
      this._buildTrinet();
      this._renderStateLegend();
      this._renderVrsLegend();
      this.render();
    }

    /**
     * 渲染虚拟站图例（菱形紫块 + 计数）。
     *
     * 与基线状态图例同一原则：**没有样本就不占位**。
     * 当前站网没有虚拟站时整行隐藏，避免出现「虚拟站 0」这种
     * 让人以为「功能坏了」的提示。
     */
    _renderVrsLegend() {
      const box = document.getElementById('netVrsLegend');
      if (!box) return;
      const n = this.stations.filter((s) => s.isVirtual).length;
      box.hidden = n === 0;
      if (!n) return;
      const cnt = document.getElementById('netVrsCount');
      if (cnt) cnt.textContent = String(n);
      /* 附带说明有多少条基线挂在虚拟站上 —— 这才是运维真正关心的：
       * 虚拟站本身不算问题，「几条基线依赖虚拟参考」才是。 */
      const vNames = new Set(
        this.stations.filter((s) => s.isVirtual).map((s) => s.name)
      );
      const blVirtual = this.baselines.filter(
        (b) => vNames.has(b.baseName) || vNames.has(b.roverName)
      ).length;
      box.title = box.getAttribute('title') +
        (blVirtual ? `\n当前有 ${blVirtual} 条基线以虚拟站为端点（图中标 V·）。` : '');
    }

    /** 当前站点集合的几何签名（站名与经纬度，与顺序无关）。
     *  站点增删或坐标变化会改变签名 → 触发视野重算。 */
    _geoSignature() {
      if (!this._allStations.length) return 'empty';
      /* 保留 6 位小数：远小于 GPS 量级，又足以吸收浮点噪声。
       * 排序让返回顺序（引擎每轮返回顺序可能变）不影响判断。 */
      const parts = this._allStations.map((s) =>
        s.name + '@' + s.lon.toFixed(6) + ',' + s.lat.toFixed(6)
      );
      parts.sort();
      return parts.join(';');
    }

    /**
     * 渲染基线解状态图例。
     *
     * 只列**当前站网实际存在**的状态，并复用 baselineStyle() 的
     * 颜色/线宽/虚实 —— 图例里的线段与图上的连线长得一样，
     * 对应关系不必靠文字解释。
     *
     * 静态列举全部 9 种状态有两个问题：
     *   1) 当前全是固定解时，6 个色块里 5 个没有样本，
     *      让人误以为「有状态但没画出来」；
     *   2) 状态集合会随引擎版本变化（solstr[] 增项），静态列表必然过期。
     */
    _renderStateLegend() {
      const box = document.getElementById('blStateLegend');
      if (!box) return;
      const states = NetworkMap.presentStates(this.baselines);
      if (!states.length) {
        box.textContent = '（暂无基线）';
        return;
      }
      /* 固定解排最前（最常见也最重要），其余按条数降序 */
      states.sort((a, b) => {
        if (a.level === 'fix') return -1;
        if (b.level === 'fix') return 1;
        return b.count - a.count;
      });
      box.innerHTML = states
        .map((st) => {
          const style = NetworkMap.baselineStyle(st.level);
          const cls = style.dash.length ? ' class="dashed"' : '';
          const tip = st.label + '：' + st.count + ' 条基线，线宽 ' + style.width + 'px' +
            (style.dash.length ? '，虚线表示解不可信' : '');
          return '<span class="bl-lg" title="' + U.esc(tip) + '">' +
            '<i' + cls + ' style="--c:' + style.color + ';--w:' + style.width +
            'px;--a:' + style.alpha + '"></i>' +
            '<b>' + U.esc(st.label) + '</b><s>' + st.count + '</s></span>';
        })
        .join('');
    }

    /**
     * 构建站网三角剖分
     *
     * 优先用引擎侧权威拓扑（showtrinet）；引擎不可用或尚未建网时，
     * 回退到本地 Delaunay（投影到局部平面，用 cos φ₀ 修正经度）。
     */
    _buildTrinet() {
      this.trinet = { triangles: [], edges: [] };
      this.trinetEdges = [];
      this.trinetTris = [];
      this.trinetSource = 'none';

      if (this._engineTrinet) {
        this._buildTrinetFromEngine();
        return;
      }
      this._buildTrinetLocal();
    }

    /**
     * 用引擎快照建图。
     *
     * 引擎给的是 srcId 拓扑 + ECEF 转出的经纬度；本图已有一份来自 conf 的
     * 站点列表（带中文名、行政区划）。此处按坐标就近把两者对齐，
     * 这样三角形能直接复用已计算好的画布坐标。
     */
    _buildTrinetFromEngine() {
      const et = this._engineTrinet;
      if (!et.nodes || et.nodes.length < 3) return;

      // srcId -> 本图站点：优先按名字精确匹配，其次按坐标就近
      const byName = new Map(this.stations.map((s) => [s.name, s]));
      const byId = new Map();
      const used = new Set();

      for (const n of et.nodes) {
        if (!n.hasPos) continue;
        let hit = byName.get(n.name) || null;
        if (!hit) {
          // 名字对不上时按坐标找最近的未占用站点（ECEF 转出的经纬度
          // 与 conf 里的值同源，容差 1e-4 度约 10 米已很宽松）
          let best = null;
          let bestD = Infinity;
          for (const s of this.stations) {
            if (used.has(s.name)) continue;
            const d = Math.hypot(s.lat - n.lat, s.lon - n.lon);
            if (d < bestD) {
              bestD = d;
              best = s;
            }
          }
          if (best && bestD < 1e-4) hit = best;
        }
        if (hit) {
          byId.set(n.srcId, hit);
          used.add(hit.name);
        }
      }

      if (byId.size < 3) {
        // 对齐失败就不硬画，退回本地算法，避免画出错位的网
        this.trinetSource = 'fallback';
        this._buildTrinetLocal();
        return;
      }

      const edges = [];
      for (const e of et.edges) {
        const A = byId.get(e.a);
        const B = byId.get(e.b);
        if (A && B) edges.push([A, B]);
      }
      const tris = [];
      for (const t of et.triangles) {
        const A = byId.get(t.v[0]);
        const B = byId.get(t.v[1]);
        const C = byId.get(t.v[2]);
        if (A && B && C) tris.push([A, B, C]);
      }

      this.trinetEdges = edges;
      this.trinetTris = tris;
      this.trinet = { triangles: tris, edges };
      this.trinetSource = 'engine';
      this.trinetWarnings = (et.warnings || []).slice();
    }

    /** 本地 Delaunay 兜底 */
    _buildTrinetLocal() {
      if (!D) return;

      const pts = this.stations.map((s, idx) => ({
        i: idx,
        x: s.lon * Math.cos((this.centerLat * Math.PI) / 180),
        y: s.lat
      }));

      const r = D.triangulate(pts);
      this.trinet = r;
      // 边按两端站名缓存，绘制时直接取坐标
      this.trinetEdges = r.edges.map(([a, b]) => [this.stations[a], this.stations[b]]);
      this.trinetTris = r.triangles.map(([a, b, c]) => [
        this.stations[a], this.stations[b], this.stations[c]
      ]);
      if (this.trinetSource === 'none') this.trinetSource = 'local';
    }

    /** 切换三角网显示 */
    setShowTrinet(on) {
      this.showTrinet = !!on;
      this.render();
    }

    setShowTrinetFill(on) {
      this.showTrinetFill = !!on;
      this.render();
    }

    /** 仅显示在线站点参与构网（离线站会让网形失真） */
    setTrinetOnlineOnly(on) {
      this.trinetOnlineOnly = !!on;
      if (on && this._allStations) {
        this.stations = this._allStations.filter((s) => s.online === true);
        if (this.stations.length < 3) this.stations = this._allStations;
        this._computeBase();
        this._buildTrinet();
        this.render();
      } else if (this._allStations) {
        this.stations = this._allStations;
        this._computeBase();
        this._buildTrinet();
        this.render();
      }
    }

    /** 计算基准投影参数 */
    _computeBase() {
      if (!this.stations.length) {
        this.baseScale = 1;
        this.baseOffsetX = this.w / 2;
        this.baseOffsetY = this.h / 2;
        return;
      }
      let minLat = Infinity, maxLat = -Infinity;
      let minLon = Infinity, maxLon = -Infinity;
      for (const s of this.stations) {
        if (s.lat < minLat) minLat = s.lat;
        if (s.lat > maxLat) maxLat = s.lat;
        if (s.lon < minLon) minLon = s.lon;
        if (s.lon > maxLon) maxLon = s.lon;
      }

      // 单站或共点时给个默认跨度
      if (maxLat - minLat < 1e-6) { minLat -= 0.05; maxLat += 0.05; }
      if (maxLon - minLon < 1e-6) { minLon -= 0.05; maxLon += 0.05; }

      const pad = 60;
      const spanX = maxLon - minLon;
      const spanY = maxLat - minLat;
      this.baseScale = Math.min(
        (this.w - pad * 2) / spanX,
        (this.h - pad * 2) / spanY
      );
      this.centerLon = (minLon + maxLon) / 2;
      this.centerLat = (minLat + maxLat) / 2;
      this.baseOffsetX = this.w / 2;
      this.baseOffsetY = this.h / 2;

      this.spanX = spanX;
      this.spanY = spanY;
      this.minLat = minLat;
      this.maxLat = maxLat;
      this.minLon = minLon;
      this.maxLon = maxLon;
    }

    /** 经纬度 → 画布坐标（等距圆柱投影） */
    project(lon, lat) {
      const s = this.baseScale * this.scale;
      const x = (lon - this.centerLon) * s + this.offsetX;
      //纬度方向上翻转，使北在上
      const y = -(lat - this.centerLat) * s + this.offsetY;
      return { x, y };
    }

    _applyView() {
      if (this.autoFit) {
        this.scale = 1;
        this.offsetX = this.baseOffsetX;
        this.offsetY = this.baseOffsetY;
      }
    }

    _handleClick(mx, my) {
      let best = null;
      let bestD = 14; // 命中半径
      for (const p of this.hitPoints) {
        const d = Math.hypot(p.x - mx, p.y - my);
        if (d < bestD) {
          bestD = d;
          best = p;
        }
      }
      this.selected = best ? best.name : null;
      if (this.onSelect) this.onSelect(this.selected);
      this.render();
    }

    render() {
      const ctx = this.ctx;
      if (!ctx) return;
      this._applyView();

      ctx.clearRect(0, 0, this.w, this.h);

      if (!this.stations.length) {
        ctx.fillStyle = '#8b97a6';
        ctx.font = '13px -apple-system, sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('暂无带经纬度的基站数据', this.w / 2, this.h / 2);
        this.hitPoints = [];
        return;
      }

      this._drawGrid(ctx);
      if (this.showTrinet) this._drawTrinet(ctx);
      if (this.showBaselines) this._drawBaselines(ctx);
      this._drawStations(ctx);
    }

    /**
     * 绘制站网三角网
     *
     * 视觉分层（自下而上）：面片底纹 → 网形边线。
     * 边线颜色按两端站的在线情况取：
     *   两端均在线 → 实线深色；否则虚线浅色，一眼看出失效链路。
     */
    _drawTrinet(ctx) {
      const tris = this.trinetTris || [];
      const edges = this.trinetEdges || [];
      if (!edges.length) return;

      // ── 面片底纹 ──────────────────────────────────────────
      // 用极浅的填充让网形结构可辨，但不至于压过站点与基线
      if (this.showTrinetFill && tris.length) {
        ctx.save();
        ctx.beginPath();
        for (const [a, b, c] of tris) {
          const p1 = this.project(a.lon, a.lat);
          const p2 = this.project(b.lon, b.lat);
          const p3 = this.project(c.lon, c.lat);
          ctx.moveTo(p1.x, p1.y);
          ctx.lineTo(p2.x, p2.y);
          ctx.lineTo(p3.x, p3.y);
          ctx.closePath();
        }
        ctx.fillStyle = 'rgba(99,102,241,.055)';
        ctx.fill();
        ctx.restore();
      }

      // ── 网形边线 ──────────────────────────────────────────
      ctx.save();
      ctx.lineWidth = 1.2;

      for (const [a, b] of edges) {
        if (!a || !b) continue;
        const p1 = this.project(a.lon, a.lat);
        const p2 = this.project(b.lon, b.lat);
        const bothOnline = a.online === true && b.online === true;

        ctx.setLineDash(bothOnline ? [] : [4, 3]);
        ctx.strokeStyle = bothOnline ? 'rgba(99,102,241,.42)' : 'rgba(148,163,184,.38)';

        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
      }

      ctx.setLineDash([]);

      // ── 放大后标注三角形边长 ───────────────────────────────
      // 只在缩放足够大时显示，避免密集区文字糊成一片
      if (this.scale > 2.2) {
        ctx.font = '10px ui-monospace, monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        for (const [a, b] of edges) {
          if (!a || !b) continue;
          const km = U.haversineKm(a.lat, a.lon, b.lat, b.lon);
          if (!Number.isFinite(km)) continue;
          const p1 = this.project(a.lon, a.lat);
          const p2 = this.project(b.lon, b.lat);
          // 只标注足够长的边，短边标注会互相遮挡
          if (Math.hypot(p2.x - p1.x, p2.y - p1.y) < 70) continue;
          const mx = (p1.x + p2.x) / 2;
          const my = (p1.y + p2.y) / 2;
          const txt = km >= 100 ? km.toFixed(0) + 'km' : km.toFixed(1) + 'km';
          const tw = ctx.measureText(txt).width;
          ctx.fillStyle = 'rgba(255,255,255,.82)';
          ctx.fillRect(mx - tw / 2 - 2, my - 6, tw + 4, 12);
          ctx.fillStyle = 'rgba(79,70,229,.78)';
          ctx.fillText(txt, mx, my);
        }
      }
      ctx.restore();
    }

    _drawGrid(ctx) {
      const s = this.baseScale * this.scale;
      // 依据当前缩放选择合适的网格间距（度）
      const targetPx = 90;
      let stepDeg = Math.pow(10, Math.round(Math.log10(targetPx / s)));
      const candidates = [stepDeg, stepDeg * 2, stepDeg * 5, stepDeg * 10];
      // 选一个屏幕上间距 >= 60px 的
      let step = candidates.find((c) => c * s >= 60) || candidates[candidates.length - 1];

      const startLon = this.centerLon - this.w / 2 / s;
      const endLon = this.centerLon + this.w / 2 / s;
      const startLat = this.centerLat + this.h / 2 / s;
      const endLat = this.centerLat - this.h / 2 / s;

      ctx.strokeStyle = '#eef1f5';
      ctx.fillStyle = '#b8c2ce';
      ctx.lineWidth = 1;
      ctx.font = '10px ui-monospace, monospace';

      const from = (v) => Math.ceil(v / step) * step;

      ctx.textAlign = 'left';
      ctx.textBaseline = 'top';
      for (let lon = from(startLon); lon <= endLon; lon += step) {
        const { x } = this.project(lon, this.centerLat);
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, this.h);
        ctx.stroke();
        ctx.fillText(lon.toFixed(step < 0.01 ? 3 : step < 1 ? 2 : 0) + '°E', x + 3, 3);
      }

      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      for (let lat = from(endLat); lat <= startLat; lat += step) {
        const { y } = this.project(this.centerLon, lat);
        if (y < -20 || y > this.h + 20) continue;
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(this.w, y);
        ctx.stroke();
        ctx.fillText(lat.toFixed(step < 0.01 ? 3 : step < 1 ? 2 : 0) + '°N', 3, y - 5);
      }
    }

    _drawBaselines(ctx) {
      const byName = new Map(this.stations.map((s) => [s.name, s]));

      /* 具体的线宽/透明度/虚线由 baselineStyle() 按解状态给出，
       * 循环开始前不再统一设置，避免覆盖。 */

      /* 长度标签的可见性判断。
       *
       * 原实现是 `this.scale > 1.6`，但 autoFit（fitView，默认开启）会把
       * scale 强制重置为 1（见 _applyView），所以默认视图下**永远不显示**。
       * 固定阈值也不合理：站网疏密程度不同，需要的缩放倍数也不同。
       *
       * 改为按**画布空间**判断：先算出所有标签位置，
       * 若彼此不重叠（或重叠数量很少）就画出来。这样默认视图即可见，
       * 放大后也不会糊成一片。 */
      const labels = [];
      const hits = [];

      for (const b of this.baselines) {
        const bn = b.baseName || U.esc(String(b.baseSrcId));
        const rn = b.roverName || U.esc(String(b.roverSrcId));
        const bs = byName.get(bn);
        const rs = byName.get(rn);
        if (!bs || !rs) continue;

        const p1 = this.project(bs.lon, bs.lat);
        const p2 = this.project(rs.lon, rs.lat);

        /* 颜色 = 解状态（沿用全站统一的 COLORS 映射，与基线表徽标、
         * 徽标 CSS 变量同源，保证「图上线的颜色」与「表里徽标的颜色」
         * 指向同一个视觉语言）。
         *
         * 额外用**线宽 + 虚线**承载第二通道：只靠颜色区分对色弱用户
         * 不友好，也让「固定解 vs 降级固定解」这类相近色更难分辨。
         * 线宽按解质量递减，劣质解用虚线表示不可靠。 */
        const style = NetworkMap.baselineStyle(b.stat);
        ctx.strokeStyle = style.color;
        ctx.lineWidth = style.width;
        ctx.setLineDash(style.dash);
        ctx.globalAlpha = style.alpha;
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
        ctx.setLineDash([]);

        /* 记录线段几何供悬停命中判定。网图区域不大，线性扫描足够，
         * 不必建空间索引。 */
        hits.push({
          x1: p1.x, y1: p1.y, x2: p2.x, y2: p2.y,
          label: bn + ' \u2192 ' + rn,
          stat: b.stat,
          statLabel: U.labelOf(b.stat),
          km: b.baselineKm
        });

        // 收集基线中点的长度标签（画完线之后再统一绘制）
        if (Number.isFinite(b.baselineKm)) {
          /* 基线任一端是虚拟站时，在长度标签前加「V」角标。
           *
           * 虚拟参考点参与解算意味着这条基线是「虚拟基线」——
           * 它的长度与普通实体基线不是一回事（VRS 常按距离半径生成）。
           * 标出来可避免运维把虚拟基线的长度当作实体站间距来评估网型。 */
          const vVirtual = !!(bs.isVirtual || rs.isVirtual);
          labels.push({
            x: (p1.x + p2.x) / 2,
            y: (p1.y + p2.y) / 2,
            text: (vVirtual ? 'V·' : '') + b.baselineKm.toFixed(0) + 'km',
            km: b.baselineKm,
            color: NetworkMap.baselineStyle(b.stat).color,
            vVirtual
          });
        }
      }
      ctx.globalAlpha = 1;
      this._drawBaselineLabels(ctx, labels);
      this._blHits = hits;
    }

    /**
     * 找出鼠标位置命中的基线（到线段的距离最小且在阈值内）。
     * @returns {object|null}
     */
    hitBaseline(mx, my) {
      const hits = this._blHits || [];
      let best = null;
      let bestD = 7;          /* 命中半径(px)：比线宽略大，兼顾好点 */
      for (const h of hits) {
        const dx = h.x2 - h.x1;
        const dy = h.y2 - h.y1;
        const len2 = dx * dx + dy * dy;
        let t = len2 ? ((mx - h.x1) * dx + (my - h.y1) * dy) / len2 : 0;
        t = t < 0 ? 0 : t > 1 ? 1 : t;
        const px = h.x1 + t * dx;
        const py = h.y1 + t * dy;
        const d = Math.hypot(mx - px, my - py);
        if (d < bestD) { bestD = d; best = h; }
      }
      return best;
    }

    /**
     * 绘制基线长度标签，带碰撞 avoidance。
     *
     * 标签互相压住时优先保留较长的（信息量更大），
     * 且同一位置只留一个，避免密集区文字糊成一片。
     */
    _drawBaselineLabels(ctx, labels) {
      if (!labels || !labels.length) return;

      ctx.font = '600 11px ui-monospace, monospace';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';

      /* 按长度降序：先画长基线，短基线遇到冲突就让位。
       * 长度是标签里最有信息量的部分，优先保证长基线可读。
       *
       * 注意必须用结构化的 L.km 排序，不能 parseFloat(L.text)：
       * 虚拟基线的文本带 'V·' 前缀（如 'V·21km'），
       * parseFloat 会得到 NaN，NaN 参与 sort 比较会让整个排序失效
       *（NaN 参与比较恒为 false，数组顺序退化成原序）。 */
      const sorted = labels.slice().sort((a, b) => {
        const na = Number.isFinite(a.km) ? a.km : parseFloat(a.text);
        const nb = Number.isFinite(b.km) ? b.km : parseFloat(b.text);
        return (nb || 0) - (na || 0);
      });

      const placed = [];
      const MIN_DX = 24;   // 标签最小水平间距(px)
      const MIN_DY = 12;   // 标签最小垂直间距(px)

      for (const L of sorted) {
        const tw = ctx.measureText(L.text).width;
        const halfW = tw / 2 + 3;
        let clash = false;

        for (const p of placed) {
          if (Math.abs(p.x - L.x) < (p.halfW + halfW) && Math.abs(p.y - L.y) < MIN_DY) {
            clash = true;
            break;
          }
        }
        if (clash) continue;   // 冲突则不画，保证可读性

        ctx.fillStyle = 'rgba(255,255,255,.94)';
        ctx.fillRect(L.x - halfW, L.y - 7.5, halfW * 2, 15);
        /* 虚拟基线的标签加紫框：与虚拟站节点的菱形+紫边呼应，
         * 让「这条基线挂在虚拟点上」在标签上也一眼可辨。 */
        if (L.vVirtual) {
          ctx.strokeStyle = VRS_COLOR;
          ctx.lineWidth = 1;
          ctx.strokeRect(L.x - halfW + 0.5, L.y - 7, halfW * 2 - 1, 14);
        }
        ctx.fillStyle = L.color;
        ctx.fillText(L.text, L.x, L.y);
        placed.push({ x: L.x, y: L.y, halfW });
      }
    }

    _drawStations(ctx) {
      this.hitPoints = [];

      /* 虚拟站着重显示：先画普通站，再画虚拟站。
       *
       * 为什么要着重：虚拟站（VRS 虚拟参考点）是引擎按基线解算需求
       * 动态生成的参考点，不是物理基站 —— 运维看站网图时若与真实基站
       * 长得一样，会误判「这里有7 个实体站」。故用**形状 + 颜色 + 光环**
       * 三重区分：紫色菱形 + 虚线光环 + 「V」角标，一眼可辨。
       *
       * 绘制顺序放在同一循环里按虚拟站后置即可，但选中环要画在最上层，
       * 所以拆成两轮：先所有真实站，再所有虚拟站，最后统一画选中环。 */
      const ordered = this.stations.slice().sort(
        (a, b) => (a.isVirtual ? 1 : 0) - (b.isVirtual ? 1 : 0)
      );

      for (const s of ordered) {
        const { x, y } = this.project(s.lon, s.lat);
        const isSel = this.selected === s.name;
        const online = s.online === true;
        const stat = s.stat || (online ? null : 'NONE');
        /* 节点半径必须在分支外算：真实站的标签偏移要用它，
         * 虚拟站的标签偏移则用它自己的 VRS_R。写进 else 分支会让
         * 真实站标签处拿不到（ReferenceError: r is not defined）。 */
        const r = isSel ? 7 : 5.5;

        if (s.isVirtual) {
          this._drawVirtualStation(ctx, x, y, isSel, online, s);
        } else {
          const color = stat ? U.colorOf(stat) : '#94a3b8';

          // 离线站点用空心表示
          if (!online) {
            ctx.beginPath();
            ctx.arc(x, y, r, 0, Math.PI * 2);
            ctx.fillStyle = '#fff';
            ctx.fill();
            ctx.strokeStyle = color;
            ctx.lineWidth = 2;
            ctx.stroke();
          } else {
            // 在线站点带光晕
            ctx.beginPath();
            ctx.arc(x, y, r + 3.5, 0, Math.PI * 2);
            ctx.fillStyle = U.hexToRgba(color, 0.16);
            ctx.fill();

            ctx.beginPath();
            ctx.arc(x, y, r, 0, Math.PI * 2);
            ctx.fillStyle = color;
            ctx.fill();
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 1.8;
            ctx.stroke();
          }
        }

        this.hitPoints.push({ x, y, name: s.name });

        // 标签：虚拟站始终显示（不依赖 showLabels），
        // 否则「着重显示」在默认视图下等于没做
        if (s.isVirtual) {
          const txt = s.name;
          ctx.font = '600 11.5px -apple-system, sans-serif';
          const tw = ctx.measureText(txt).width;
          const ty = y - 13;
          ctx.fillStyle = 'rgba(245,243,255,.95)';
          ctx.fillRect(x - tw / 2 - 4, ty - 1, tw + 8, 15);
          ctx.strokeStyle = VRS_COLOR;
          ctx.lineWidth = 1;
          ctx.strokeRect(x - tw / 2 - 4, ty - 1, tw + 8, 15);
          ctx.fillStyle = VRS_COLOR;
          ctx.textAlign = 'center';
          ctx.textBaseline = 'bottom';
          ctx.fillText(txt, x, ty + 12);
        } else if (this.showLabels || isSel) {
          const txt = this.showCnNames && s.nameMatched ? s.cnName || s.name : s.name;
          ctx.font = (isSel ? '600 ' : '') + '11.5px -apple-system, sans-serif';
          const tw = ctx.measureText(txt).width;
          const ty = y - r - 7;
          ctx.fillStyle = 'rgba(255,255,255,.9)';
          ctx.fillRect(x - tw / 2 - 3, ty - 1, tw + 6, 14);
          ctx.fillStyle = isSel ? '#2563eb' : '#1c2530';
          ctx.textAlign = 'center';
          ctx.textBaseline = 'bottom';
          ctx.fillText(txt, x, ty + 12);
        }
      }

      // 选中环统一画在最上层（虚拟站的虚线光环不该被选中环压住）
      for (const s of this.stations) {
        if (this.selected !== s.name) continue;
        const { x, y } = this.project(s.lon, s.lat);
        ctx.beginPath();
        ctx.arc(x, y, (s.isVirtual ? 12 : 7) + 7, 0, Math.PI * 2);
        ctx.strokeStyle = '#2563eb';
        ctx.lineWidth = 2;
        ctx.stroke();
      }
    }

    /**
     * 画一个虚拟站（VRS 虚拟参考点）。
     *
     * 视觉语言与真实基站刻意不同，三重区分：
     *   形状：菱形（真实站是圆）
     *   颜色：紫色（与 8 种解状态色都不撞）
     *   光环：虚线圆（真实站是实心光晕）
     * 在线与否不靠填充实心/空心区分 —— 虚拟站本来就是引擎生成的，
     * 「连不在线」对它没有运维含义，故统一实心，只用虚线光环表示存在。
     *
     * @param {number} x 画布 X
     * @param {number} y 画布 Y
     * @param {boolean} isSel 是否选中
     * @param {boolean} online 在线状态（保留形参以便未来按在线态微调样式）
     * @param {object} s 站点对象
     */
    _drawVirtualStation(ctx, x, y, isSel, online, s) {
      const R = isSel ? 9 : 7.5;   // 菱形外接圆半径
      const inner = isSel ? 4.5 : 3.5;

      /* 虚线光环：比真实站的实心光晕更「虚」，语义上「不是实体」。
       * 半径随选中放大，与真实站的选中态保持一致的响应。 */
      ctx.save();
      ctx.setLineDash([4, 3]);
      ctx.beginPath();
      ctx.arc(x, y, R + 5, 0, Math.PI * 2);
      ctx.strokeStyle = U.hexToRgba(VRS_COLOR, isSel ? 0.75 : 0.42);
      ctx.lineWidth = isSel ? 2 : 1.4;
      ctx.stroke();
      ctx.restore();

      /* 外层半透明紫晕，强化存在感 */
      ctx.beginPath();
      ctx.arc(x, y, R + 1.5, 0, Math.PI * 2);
      ctx.fillStyle = U.hexToRgba(VRS_COLOR, 0.14);
      ctx.fill();

      /* 主体：菱形（45° 旋转的方形），一眼区别于圆形基站 */
      ctx.beginPath();
      ctx.moveTo(x, y - R);
      ctx.lineTo(x + R, y);
      ctx.lineTo(x, y + R);
      ctx.lineTo(x - R, y);
      ctx.closePath();
      ctx.fillStyle = VRS_COLOR;
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 1.8;
      ctx.stroke();

      /* 内点：中心小圆点，强化「参考点」语义 */
      ctx.beginPath();
      ctx.arc(x, y, inner * 0.55, 0, Math.PI * 2);
      ctx.fillStyle = '#fff';
      ctx.fill();
    }

    fitView() {
      this.autoFit = true;
      this.render();
    }

    highlight(name) {
      this.selected = name;
      this.render();
    }
  }

  /* ================================================================
   * 基线解状态 → 视觉样式
   *
   * 颜色沿用 theme.js 的 U.colorOf()（与基线表徽标、图例色块同源），
   * 另外用线宽/虚线/透明度承载第二通道：
   *   固定解       粗、实线、不透明      —— 最可信
   *   降级固定解   中、实线              —— 可用但退化
   *   浮点/PPP/差分 中、实线              —— 可用
   *   单点/SBAS/DR 细、实线              —— 精度有限
   *   无解         最细、虚线、半透明    —— 不可用
   *
   * 只靠颜色区分对色弱用户不友好；线宽与虚实是形状通道，
   * 在灰度打印或色觉障碍下仍能分辨。
   * ================================================================ */
  /** 显示/隐藏悬停提示 */
  NetworkMap.prototype._updateHover = function (mx, my) {
    if (!this._hoverEl) {
      const el = document.createElement('div');
      el.className = 'bl-hover';
      el.hidden = true;
      /* 挂在网图容器内（position:relative），便于用绝对定位跟随鼠标 */
      const host = this.canvas && this.canvas.parentElement;
      if (!host) return;
      host.appendChild(el);
      this._hoverEl = el;
    }
    const hit = this.hitBaseline(mx, my);
    if (!hit) {
      if (!this._hoverEl.hidden) {
        this._hoverEl.hidden = true;
        this.canvas.style.cursor = '';
      }
      return;
    }
    const st = NetworkMap.baselineStyle(hit.stat);
    const km = Number.isFinite(hit.km) ? U.num(hit.km, 1) + ' km' : '—';
    this._hoverEl.innerHTML =
      '<div class="bl-hover-t">' + U.esc(hit.label) + '</div>' +
      '<div class="bl-hover-r"><i style="background:' + st.color + '"></i>' +
      '<b>' + U.esc(hit.statLabel) + '</b><s>' + km + '</s></div>';
    this._hoverEl.hidden = false;
    /* 靠近右/下边缘时翻到另一侧，避免提示被裁掉 */
    const hw = this._hoverEl.offsetWidth || 160;
    const left = mx + hw + 14 > this.w ? mx - hw - 12 : mx + 12;
    this._hoverEl.style.left = left + 'px';
    this._hoverEl.style.top = Math.min(my + 12, this.h - 46) + 'px';
    this.canvas.style.cursor = 'pointer';
  };

  NetworkMap.baselineStyle = function (stat) {
    const lv = U.levelOf(stat) || 'none';
    const color = U.colorOf(stat);
    switch (lv) {
      case 'fix':
        return { color, width: 3.0, dash: [], alpha: 0.95 };
      case 'fixdeg':
        return { color, width: 2.4, dash: [], alpha: 0.9 };
      case 'ppp':
      case 'float':
        return { color, width: 2.0, dash: [], alpha: 0.85 };
      case 'dgps':
        return { color, width: 1.8, dash: [], alpha: 0.8 };
      case 'sbas':
      case 'single':
      case 'dr':
        return { color, width: 1.4, dash: [], alpha: 0.7 };
      default:
        /* 无解：虚线 + 半透明，一眼看出这条基线不可信 */
        return { color, width: 1.2, dash: [5, 4], alpha: 0.5 };
    }
  };

  /**
   * 当前站网里实际出现的解状态集合（用于图例只显示有样本的项）。
   * 全部是 FIX 时不必把 6 个色块都摆出来 —— 空色块反而让人误以为
   * 存在其他状态却没画出来。
   */
  NetworkMap.presentStates = function (baselines) {
    const seen = new Map();
    (baselines || []).forEach((b) => {
      const lv = U.levelOf(b.stat) || 'none';
      if (!seen.has(lv)) seen.set(lv, { level: lv, label: U.labelOf(b.stat), count: 0, color: U.colorOf(b.stat) });
      seen.get(lv).count++;
    });
    return [...seen.values()];
  };

  global.MCORS.NetworkMap = NetworkMap;
})(window);