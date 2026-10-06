/**
 * 主控制器：串联各面板，处理数据分发与联动
 */
(function (global) {
  'use strict';

  const U = global.MCORS.util;

  class Dashboard {
    constructor() {
      this.api = new global.MCORS.Api();
      this.snapshot = null;
      this.lastFrameCount = 0;
      this.lastFrameAt = 0;
      this.frameRate = 0;

      // 面板实例
      this.netMap = new global.MCORS.NetworkMap('networkMap');
      this.blPanel = new global.MCORS.BaselinePanel();
      this.stPanel = new global.MCORS.StationPanel();
      this.sky = new global.MCORS.SkyPlot('skyPlot', 'skyStation');
      this.trend = new global.MCORS.TrendChart('trendChart', 'trendMetric');

      this.selectedStation = null;

      this._bindPanels();
      this._bindControls();

      // WebSocket
      this.api.onStatusChange = (ok) => this._renderConn(ok);
      this.api.onSnapshot((s) => this.onSnapshot(s));
      this.api.connect();

      // 命令输出抽屉
      this.drawer = document.getElementById('cmdDrawer');
      this.drawerOut = document.getElementById('cmdOut');
      this.mask = document.getElementById('mask');
      const btnCloseDrawer = document.getElementById('btnCloseDrawer');
      if (btnCloseDrawer) {
        btnCloseDrawer.onclick = () => {
          this.drawer.hidden = true;
          this.mask.hidden = true;
        };
      }
    }

    _bindPanels() {
      // 网图选中 → 联动站点下拉与天空图
      this.netMap.onSelect = (name) => {
        this.selectedStation = name;
        if (name) {
          U.toast('已选中基站', name);
          this._loadSky(name);
        }
      };

      // 基线表格点击 → 联动网图
      this.blPanel.onSelect = (name) => {
        this.selectedStation = name;
        this.netMap.highlight(name);
        this._loadSky(name);
      };
      this.blPanel.onRefresh = () => this._pollBaselines();

      // 站点表格点击 → 联动
      this.stPanel.onSelect = (name) => {
        this.selectedStation = name;
        this.netMap.highlight(name);
        this._loadSky(name);
      };
      this.stPanel.onChanged = () => {
        U.toast('列表已更新', '等待引擎确认…', 'warn');
        // 重新拉取字典
        global.MCORS.rest.post('/api/dict/reload').then(() => {
          this._pollBaselines();
        });
      };

      this.sky.onSelect = (name) => {
        if (!name) return;
        this.selectedStation = name;
        this.netMap.highlight(name);
      };
    }

    _bindControls() {
      const optTrinet = document.getElementById('optShowTrinet');
      const optTrinetFill = document.getElementById('optTrinetFill');
      const optLabels = document.getElementById('optShowLabels');
      const optCn = document.getElementById('optShowLabels2');
      const optBl = document.getElementById('optShowBaselines');
      const btnFit = document.getElementById('btnFitView');

      if (optTrinet) {
        optTrinet.onchange = () => {
          this.netMap.setShowTrinet(optTrinet.checked);
        };
      }
      if (optTrinetFill) {
        optTrinetFill.onchange = () => {
          this.netMap.setShowTrinetFill(optTrinetFill.checked);
        };
      }
      if (optLabels) {
        optLabels.onchange = () => {
          this.netMap.showLabels = optLabels.checked;
          this.netMap.render();
        };
      }
      if (optCn) {
        optCn.onchange = () => {
          this.netMap.showCnNames = optCn.checked;
          this.netMap.render();
        };
      }
      if (optBl) {
        optBl.onchange = () => {
          this.netMap.showBaselines = optBl.checked;
          this.netMap.render();
        };
      }
      if (btnFit) {
        btnFit.onclick = () => this.netMap.fitView();
      }
    }

    /** 请求引擎刷新基线（通过只读命令通道） */
    async _pollBaselines() {
      const r = await global.MCORS.rest.post('/api/console', { cmd: 'showbls' });
      if (r.ok && r.output) {
        U.toast('基线已刷新', '', 'ok');
      }
    }

    /** 加载指定站点的卫星数据 */
    _loadSky(name) {
      if (!this.snapshot || !name) return;
      const sel = document.getElementById('skyStation');
      if (sel && sel.value !== name) {
        const opt = sel.querySelector(`option[value="${CSS.escape(name)}"]`);
        if (opt) sel.value = name;
      }
      this._updateSky(name);
    }

    _updateSky(name) {
      if (!this.snapshot) return;
      const latest = this.snapshot.latest || {};

      // 优先匹配 RTK 帧，其次 PNT
      let payload = null;
      for (const key of Object.keys(latest)) {
        const [type, who] = key.split(':');
        if (who === name && type === 'RTK') { payload = latest[key]; break; }
      }
      if (!payload) {
        for (const key of Object.keys(latest)) {
          const [type, who] = key.split(':');
          if (who === name) { payload = latest[key]; break; }
        }
      }

      if (!payload) {
        this.sky.setData(null, this.snapshot.caps);
        return;
      }

      this.sky.setData({
        sats: payload.sats || [],
        usedSats: this._usedSatsOf(name),
        title: `${name} · ${payload.type}`,
        qualityLabel: payload.qualityLabel,
        hdop: payload.hdop,
        pdop: payload.pdop,
        vdop: payload.vdop,
        numSat: payload.numSat
      }, this.snapshot.caps);
    }

    /** 从 GSA 汇总参与解算卫星 */
    _usedSatsOf(name) {
      const latest = this.snapshot ? this.snapshot.latest || {} : {};
      const used = new Set();
      for (const key of Object.keys(latest)) {
        const [, who] = key.split(':');
        if (who !== name) continue;
        const p = latest[key];
        if (p && p.usedSats) p.usedSats.forEach((s) => used.add(s));
      }
      return [...used];
    }

    onSnapshot(s) {
      this.snapshot = s;

      // 数据来源标识
      const srcTag = document.getElementById('sourceTag');
      if (srcTag) {
        const c = s.caps || {};
        // 有异常时直接显示诊断结论，比笼统的"实时/模拟"更有信息量
        if (c.level === 'error' || c.level === 'warn') {
          srcTag.textContent = c.msg;
          srcTag.style.color = c.level === 'error' ? 'var(--danger)' : 'var(--warn)';
          srcTag.title = c.hint || '';
        } else {
          const isMock = s.source === 'mock';
          srcTag.textContent = isMock
            ? '模拟数据源（引擎未运行）'
            : `实时数据源 · 已运行 ${Math.round(s.uptimeMs / 1000)}s`;
          srcTag.style.color = isMock ? 'var(--warn)' : 'var(--text-3)';
          srcTag.title = '';
        }
      }

      // 顶部统计
      this._renderStats(s);
      this._renderConnSource(s);
      this._renderCapsBanner(s);

      // 各面板
      this.netMap.setData(s.stations, s.baselines, s.trinet);
      this._renderTopoStatus(s);
      /* 名册 + 填入：第 4 参传 trinet 播种名册 —— 基线表与网图一一对应
       * （无向等价：1→2 与 2→1 同一条基线），网图里有的边先显示
       * 「待解算」，$BLSOLS 每帧往对应行填数。播种只收两端都有真实
       * 站名的边（引擎三角网里有无解算输出的幽灵顶点，已在面板内过滤），
       * 行数与行序由此结构性稳定。 */
      this.blPanel.setData(s.baselines, s.caps, s.satHistory, s.trinet);
      this.stPanel.setData(s.stations, s.srcInfoStations);

      // 天空图下拉
      //
      // 旧逻辑只在用户点选后才设 selectedStation，而下拉默认是空值，
      // 于是天空图永远拿不到 payload，恒显「暂无卫星数据」——
      // 即使后端已经推来 10 颗星的可见星与解算星列表。
      // 这里在未选站时自动选中「有实时数据」的第一个站，
      // 用户手动选过之后就尊重用户选择（不覆盖）。
      if (!this.selectedStation) {
        const withData = Object.keys(s.latest || {});
        if (withData.length) {
          const who = withData[0].split(':')[1];
          if (who) this.selectedStation = who;
        }
      }
      this.sky.setOptions(s.stations, this.selectedStation);
      if (this.selectedStation) this._updateSky(this.selectedStation);

      /* 趋势：优先取当前选中站的那一组，取不到则回退到任意一组。
       *
       * 这里的 key 匹配曾长期是「趋势图无曲线」的根因：
       * 采集层原先只广播第一组 RTK:，而选中站由网图/基线表/站点表点击
       * 写入（传的是**站名**，未必等于该组 key 里的 rover 名），
       * 于是 trendKey 拼出来的组在 s.history 里不存在 → setData([])。
       * 现在采集层广播全部组，但保留回退链：选中站没有历史时
       * 退到第一个 RTK: 组，好过整张图空着。 */
      const hist = s.history || {};
      const hk = (x) => hist[`RTK:${x}`];
      let trendKey = this.selectedStation && hk(this.selectedStation)
        ? `RTK:${this.selectedStation}`
        : null;
      if (!trendKey) {
        trendKey = Object.keys(hist).find((k) => k.startsWith('RTK:')) ||
                   Object.keys(hist)[0] || null;
      }
      this.trend.setData(trendKey ? hist[trendKey] : []);
    }

    _renderStats(s) {
      const stations = s.stations || [];
      const baselines = s.baselines || [];

      const online = stations.filter((x) => x.online === true).length;
      const fix = baselines.filter((b) => b.level === 'fix' || b.level === 'fixdeg').length;

      const set = (id, val) => {
        const el = document.getElementById(id);
        if (el) el.querySelector('b').textContent = val;
      };
      set('statStations', stations.length);
      set('statBaselines', baselines.length);
      set('statFix', fix);
      set('statOnline', `${online}/${stations.length}`);

      /* 帧率：采集层每秒推一次快照，用帧数差值估算。
       *
       * 数据源字段是 `solution`（solution.c 的推送流）。
       * 原先这里读的是 `s.monitor` —— 那条通道已被废弃，
       * 字段名对不上会让 frameCount 取到 undefined，
       * 求出的帧率是 NaN，顶栏会直接显示 "NaN"。
       * 故这里既改用正确字段，又对非有限值做兜底，
       * 避免将来再改字段名时又以 NaN 的形式暴露到界面上。 */
      const m = s.solution || {};
      const cur = Number(m.frames);
      if (Number.isFinite(cur) && cur !== this.lastFrameCount && this.lastFrameAt) {
        const dt = (s.ts - this.lastFrameAt) / 1000;
        if (dt > 0) this.frameRate = (cur - this.lastFrameCount) / dt;
        this.lastFrameCount = cur;
        this.lastFrameAt = s.ts;
      } else if (!this.lastFrameAt || !Number.isFinite(this.lastFrameCount)) {
        this.lastFrameCount = Number.isFinite(cur) ? cur : 0;
        this.lastFrameAt = s.ts;
      }
      set('statFrames', Number.isFinite(this.frameRate) ? this.frameRate.toFixed(1) : '—');
    }

    /**
     * 显示三角网拓扑来源。
     *
     * 运维必须能一眼分清「这是引擎算出来的网」还是「前端自己凑的网」——
     * 前者是权威数据，后者只是引擎不可用时的示意，不能用来判断
     * 基线是否按预期组网。
     */
    _renderTopoStatus(s) {
      const el = document.getElementById('netTopoStatus');
      if (!el) return;
      const sw = el.querySelector('.sw');
      const txt = el.querySelector('b');
      const nm = this.netMap;

      const trinet = s.trinet || {};
      const err = trinet.error;

      let label;
      let cls;
      if (nm && nm.trinetSource === 'engine') {
        const n = (nm.trinetTris || []).length;
        label = `引擎拓扑 ${n} 面`;
        cls = 'ok';
      } else if (nm && nm.trinetSource === 'fallback') {
        label = '引擎对齐失败·本地示意';
        cls = 'warn';
      } else if (nm && nm.trinetSource === 'local') {
        label = '本地剖分·示意';
        cls = 'warn';
      } else {
        label = '无拓扑';
        cls = 'off';
      }
      if (err) label += '（链路错误）';

      if (txt) txt.textContent = label;
      el.title =
        '三角网拓扑来源：' +
        (nm && nm.trinetSource ? nm.trinetSource : 'none') +
        (err ? '｜错误: ' + err : '') +
        (nm && nm.trinetWarnings && nm.trinetWarnings.length
          ? '｜告警: ' + nm.trinetWarnings.join('; ')
          : '');
      el.className = 'lg topo-' + cls;

      // 三角形数量同步到统计条，便于与基线数对照
      const tset = document.getElementById('statTrinet');
      if (tset) {
        const b = tset.querySelector('b');
        if (b) b.textContent = (nm && nm.trinetTris ? nm.trinetTris.length : 0);
      }
    }

    _renderConnSource(s) {
      const el = document.getElementById('wsText');
      const dot = document.getElementById('wsDot');
      if (!el || !dot) return;

      if (s.source === 'mock') {
        dot.className = 'dot on';
        el.textContent = '模拟模式';
        return;
      }
      const ok = s.console && s.console.connected;
      dot.className = 'dot ' + (ok ? 'on' : 'off');
      el.textContent = ok ? '引擎已连接' : '引擎未连接';
    }

    _renderConn(wsOk) {
      if (!wsOk) {
        const dot = document.getElementById('wsDot');
        const txt = document.getElementById('wsText');
        if (dot) dot.className = 'dot off';
        if (txt) txt.textContent = '采集层断开';
      }
    }

    /**
     * 数据链路诊断横幅。
     *
     * 实测发现上游 NTRIP 挂点只推星历不含观测值，若不在界面明示，
     * 用户会误判为"引擎坏了"。这里把采集层的诊断结论直接呈现出来。
     */
    _renderCapsBanner(s) {
      const el = document.getElementById('capsBanner');
      if (!el) return;

      const c = s.caps;
      if (!c || c.level === 'ok') {
        el.style.display = 'none';
        el.innerHTML = '';
        return;
      }

      const isMock = c.code === 'MOCK';
      if (isMock) {
        // mock 模式不算异常，用中性提示
        el.style.display = '';
        el.className = 'caps-banner mock';
        el.innerHTML =
          '<span class="caps-tag">模拟</span>' +
          '<span class="caps-msg">当前使用内置模拟数据源，未连接真实引擎。' +
          '启动 cors-engine 后设置 <code>MCORS_SOURCE=live</code> 可切换到实时链路。</span>';
        return;
      }

      el.style.display = '';
      el.className = 'caps-banner ' + (c.level === 'error' ? 'error' : 'warn');
      el.innerHTML =
        `<span class="caps-tag">${global.MCORS.util.esc(c.msg)}</span>` +
        `<span class="caps-msg">${global.MCORS.util.esc(c.hint || '')}</span>` +
        `<span class="caps-meta">已收帧 ${c.frames} · 在线 ${c.online}/${c.total}</span>`;
    }
  }

  window.addEventListener('DOMContentLoaded', () => {
    global.MCORS.dashboard = new Dashboard();
  });
})(window);