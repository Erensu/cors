/**
 * 基站信息面板 + 基站增删交互
 *
 * 写操作会通过采集层下发引擎控制台命令：
 *   新增: addsource <name> <addr> <port> <user> <passwd> <mntpnt> <lat> <lon> <h> <type>
 *   删除: delsource <name>
 * 见 engine.c cmd_add_source / cmd_del_source
 */
(function (global) {
  'use strict';

  const U = global.MCORS.util;

  class StationPanel {
    constructor() {
      this.tbody = document.getElementById('stBody');
      this.search = document.getElementById('stSearch');
      this.data = [];
      this.filter = '';
      this.busy = false;
      this.onSelect = null;

      /* 详情抽屉：站点字段很多（20+），表格只放高频项，
       * 点击行从右侧滑出完整信息（天线/设备/连接等）。 */
      this.drawer = document.getElementById('stDrawer');
      this.drawerTitle = document.getElementById('stDrawerTitle');
      this.drawerSub = document.getElementById('stDrawerSub');
      this.drawerBody = document.getElementById('stDrawerBody');
      this._bindDrawer();

      if (this.search) {
        this.search.oninput = () => {
          this.filter = this.search.value.trim().toLowerCase();
          this.render();
        };
      }

      this._bindModal();
    }

    /**
     * @param {Array} stations        站点字典（conf 台账）列表
     * @param {Array} [srcInfoStations] 引擎 $SRCINFO 的原始行
     */
    setData(stations, srcInfoStations) {
      this.dict = stations || [];
      this.srcInfo = srcInfoStations || [];
      this.data = this._merge();
      this.render();
      // 采集层每 3s 推一次快照，表格重渲染后若抽屉正开着，
      // 需要用新数据刷新内容，否则会一直停留在旧值（例如 srcId 尚未回填时）
      if (this.drawer && !this.drawer.hidden && this._detailName) {
        this.openDetail(this._detailName);
      }
    }

    /**
     * 合并两个数据源，作为表格的最终行集。
     *
     * 为什么要以 $SRCINFO 为准、而不是用 conf 台账：
     *   台账是**人工维护的配置文件**，只含登记过的站。而引擎实际在用的源
     *   可能更多 —— 运行期 addsource 加的源不在台账里，引擎自己生成的
     *   VRS 虚拟参考点更不在。只显示台账会让运维误以为「引擎没在用这些源」。
     *
     * 反之只用 $SRCINFO 也不行：中文名、行政区划、天线高这些是台账独有，
     * 引擎侧没有。故取并集，按站名对齐：
     *   以 $SRCINFO 行的顺序与存在性为准，逐行合并台账的同名记录；
     *   台账里有、$SRCINFO 里没有的站也保留（可能只是本轮未连上），
     *   但标记 offline，避免"凭空消失"。
     */
    _merge() {
      const byName = new Map();
      for (const d of this.dict) byName.set(d.name, d);

      const rows = [];
      const seen = new Set();

      for (const r of this.srcInfo) {
        const d = byName.get(r.name) || {};
        seen.add(r.name);
        rows.push({
          ...d,                                   // 台账字段（cnName/region/antHgt/type...）
          name: r.name,
          /* 引擎侧事实，一律以 $SRCINFO 为准 */
          online: r.connected,
          connState: r.state,
          addr: r.addr,
          port: r.port,
          mountpoint: r.mountpoint,
          srcId: r.staSrcId,
          ecef: r.pos,
          antDes: r.antDes,
          antSno: r.antSno,
          marker: r.marker,
          recType: r.recType,
          recSno: r.recSno,
          recVer: r.recVer,
          isVirtual: r.isVirtual,
          fromEngine: true,
          /* 经纬度优先用台账（人工测绘），缺失时用引擎 ECEF 换算的 */
          lat: Number.isFinite(d.lat) ? d.lat : r.lat,
          lon: Number.isFinite(d.lon) ? d.lon : r.lon,
          height: Number.isFinite(d.height) ? d.height : r.hgt
        });
      }

      /* 台账有、引擎本轮没报的站：保留并标离线。
       * 这类站通常是配置里写了但上游没连上，直接从界面抹掉会让人
       * 以为配置丢了。 */
      for (const d of this.dict) {
        if (seen.has(d.name)) continue;
        rows.push({ ...d, online: false, fromEngine: false });
      }
      return rows;
    }

    _bindModal() {
      const modal = document.getElementById('modalStation');
      const form = document.getElementById('formStation');
      const mask = document.getElementById('mask');
      if (!modal || !form) return;

      const open = () => {
        modal.hidden = false;
        mask.hidden = false;
        form.reset();
        // 默认值沿用常见配置
        form.elements.port.value = 8002;
        form.elements.type.value = 'M';
        setTimeout(() => form.elements.name.focus(), 40);
      };
      const close = () => {
        modal.hidden = true;
        mask.hidden = true;
      };

      const btnAdd = document.getElementById('btnAddStation');
      if (btnAdd) btnAdd.onclick = open;
      const btnClose = document.getElementById('btnCloseModal');
      if (btnClose) btnClose.onclick = close;
      const btnCancel = document.getElementById('btnCancelModal');
      if (btnCancel) btnCancel.onclick = close;

      mask.onclick = (e) => {
        if (e.target === mask) close();
      };

      form.onsubmit = async (e) => {
        e.preventDefault();
        if (this.busy) return;

        const fd = new FormData(form);
        const payload = {
          name: String(fd.get('name') || '').trim(),
          addr: String(fd.get('addr') || '').trim(),
          port: Number(fd.get('port')),
          user: String(fd.get('user') || '').trim(),
          passwd: String(fd.get('passwd') || ''),
          mntpnt: String(fd.get('mntpnt') || '').trim(),
          lat: Number(fd.get('lat')),
          lon: Number(fd.get('lon')),
          height: Number(fd.get('height')),
          type: String(fd.get('type') || 'M').toUpperCase()
        };

        const submit = form.querySelector('button[type="submit"]');
        this.busy = true;
        submit.disabled = true;
        submit.textContent = '下发中…';

        try {
          const r = await global.MCORS.rest.post('/api/station/add', payload);
          if (r.ok) {
            U.toast('基站已创建', `${payload.name} 已下发至引擎`, 'ok');
            close();
            this.onChanged && this.onChanged();
          } else {
            U.toast('创建失败', r.error || r.output || '未知错误', 'err');
          }
        } catch (err) {
          U.toast('请求异常', err.message, 'err');
        } finally {
          this.busy = false;
          submit.disabled = false;
          submit.textContent = '确认新增';
        }
      };
    }

    render() {
      if (!this.tbody) return;

      if (!this.data.length) {
        this.tbody.innerHTML =
          '<tr class="empty"><td colspan="7">暂无基站数据<br><small style="color:var(--text-3)">点击右上角「+ 新增基站」创建</small></td></tr>';
        return;
      }

      const f = this.filter;
      const arr = f
        ? this.data.filter(
            (s) =>
              String(s.name).toLowerCase().includes(f) ||
              String(s.cnName || '').toLowerCase().includes(f) ||
              String(s.region || '').toLowerCase().includes(f) ||
              String(s.mountpoint || '').toLowerCase().includes(f) ||
              String(s.addr || '').toLowerCase().includes(f)
          )
        : this.data;

      if (!arr.length) {
        this.tbody.innerHTML = '<tr class="empty"><td colspan="7">没有匹配的基站</td></tr>';
        return;
      }

      this.tbody.innerHTML = arr
        .map((s) => {
          const online = s.online === true;
          const statCell = online
            ? '<span class="badge fix">已连接</span>'
            : '<span class="badge none">未连接</span>';

          /* runtimeOnly = 运行期 addsource 加的，不在 conf/ntripsources 里。
           * VRS 虚拟站 = 引擎自己生成的参考点（srcId 为 -1），不是真实基站。
           * 两者都必须在名字旁标出来，否则运维会以为台账里"多"了站。 */
          const nameCell = `<b>${U.esc(s.name)}</b>${
            s.nameMatched && s.cnName
              ? `<span class="sub2">${U.esc(s.cnName)}</span>`
              : ''
          }${s.isVirtual
              ? '<span class="rt-tag" title="引擎生成的 VRS 虚拟参考点，不是真实基站（$SRCINFO 中 srcId 为 -1）">虚拟</span>'
              : ''}${
            s.fromEngine === false
              ? '<span class="rt-tag" title="台账中登记，但引擎本轮 $SRCINFO 未上报（多为上游未连上）">未上报</span>'
              : (!s.isVirtual && s.runtimeOnly
                  ? '<span class="rt-tag" title="运行期新增，不在 conf/ntripsources 中；引擎重启后需重新 addsource">运行期</span>'
                  : '')
          }`;

          /* 位置：优先台账测绘坐标，缺失时用引擎 $SRCINFO 的 ECEF 换算值。
           * 经纬度都取不到时显示 —（例如未定位的源）。 */
          const hasPos = Number.isFinite(s.lat) && Number.isFinite(s.lon);
          const posCell = hasPos
            ? `${U.num(s.lat, 5)}, ${U.num(s.lon, 5)}${
                Number.isFinite(s.height) ? `<span class="sub2">${U.num(s.height, 1)} m</span>` : ''
              }`
            : '<span class="na">—</span>';

          const upCell = s.addr
            ? `<span class="mono-sm" title="${U.esc(s.addr)}:${U.esc(s.port)}">${U.esc(s.addr)}<span class="sub2">:${U.esc(s.port)}</span></span>`
            : '<span class="na">—</span>';

          /* 设备：天线型号 / 接收机型号，两行小字。
           * 引擎侧给的是设备型号字符串（如 LEIAR10），字号压小以免撑高行。 */
          const devA = s.antDes || '';
          const devB = s.recType || '';
          const devCell = (devA || devB)
            ? `${devA ? `<span class="sub2">${U.esc(devA)}</span>` : ''}${
                devB ? `<span class="sub2">${U.esc(devB)}</span>` : ''}`
            : '<span class="na">—</span>';

          return `
            <tr data-name="${U.esc(s.name)}">
              <td>${nameCell}</td>
              <td>${posCell}</td>
              <td>${s.mountpoint ? `<span class="mono-sm">${U.esc(s.mountpoint)}</span>` : '<span class="na">—</span>'}</td>
              <td>${upCell}</td>
              <td class="dev-cell">${devCell}</td>
              <td class="st-cell">
                <!-- 内部用 .st-in 包装：td 必须保持 table-cell，否则
                     table-layout:fixed 下该列会脱离列宽算法、按内容收缩
                     （实测表头 121px 而 tbody 只有 16px，徽标被挤没）。
                     包装盒做 flex 容器，兼顾间距控制与行盒高度。 -->
                <span class="st-in"><span class="dot-status ${online ? 'on' : 'off'}"></span>${statCell}</span>
              </td>
              <td>
                <button class="row-del" data-act="del" title="删除该基站"
                        ${this.busy ? 'disabled' : ''}>删除</button>
              </td>
            </tr>`;
        })
        .join('');

      this._bind();
    }

    _bind() {
      this.tbody.querySelectorAll('[data-act="del"]').forEach((btn) => {
        btn.onclick = async (e) => {
          e.stopPropagation();
          const tr = btn.closest('tr');
          const name = tr.dataset.name;

          const ok = await U.confirmDialog(
            '删除基站',
            `确认删除基站 ${name} ？引擎将停止该基站的数据接入，相关基线解算也会终止。此操作不可撤销。`
          );
          if (!ok) return;

          btn.disabled = true;
          const r = await global.MCORS.rest.post('/api/station/del', { name });
          if (r.ok) {
            U.toast('基站已删除', name, 'ok');
            this.onChanged && this.onChanged();
          } else {
            U.toast('删除失败', r.error || r.output || '未知错误', 'err');
            btn.disabled = false;
          }
        };
      });

      this.tbody.querySelectorAll('tr[data-name]').forEach((tr) => {
        tr.style.cursor = 'pointer';
        tr.onclick = (e) => {
          if (e.target.dataset.act === 'del') return;
          // 打开详情抽屉（用户要求：站点信息要详细）
          this.openDetail(tr.dataset.name);
          this.onSelect && this.onSelect(tr.dataset.name);
        };
      });
    }
  }

  /* ================================================================
   * 详情抽屉
   * ================================================================ */
  StationPanel.prototype._bindDrawer = function () {
    const btn = document.getElementById('stDrawerClose');
    if (btn) btn.onclick = () => this.closeDrawer();
    if (this.drawer) {
      // 点击抽屉外部（遮罩区域）关闭
      this.drawer.onclick = (e) => { if (e.target === this.drawer) this.closeDrawer(); };
    }
    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && this.drawer && !this.drawer.hidden) this.closeDrawer();
    });
  };

  StationPanel.prototype.closeDrawer = function () {
    this._detailName = null;
    if (this.drawer) this.drawer.hidden = true;
  };

  /* 数字格式化：null/undefined 显示为占位符 */
  function fmt(v, digits) {
    if (v === null || v === undefined || v === '') return null;
    const n = Number(v);
    if (!Number.isFinite(n)) return String(v);
    return digits === undefined ? String(n) : n.toFixed(digits);
  }

  StationPanel.prototype._kv = function (label, value, mono) {
    const na = value === null || value === undefined || value === '' ||
               value === '—';
    return `<div class="st-kv"><dt>${U.esc(label)}</dt>` +
           `<dd class="${na ? 'na' : (mono ? 'mono' : '')}">` +
           `${na ? '未配置' : U.esc(value)}</dd></div>`;
  };

  StationPanel.prototype._group = function (title) {
    return `<div class="st-grp">${U.esc(title)}</div>`;
  };

  /**
   * 打开某站点的详情。
   * @param {string} name 基站名
   */
  StationPanel.prototype.openDetail = function (name) {
    if (!this.drawer) return;
    const s = this.data.find((x) => x.name === name);
    if (!s) return;

    this._detailName = name;
    this.drawerTitle.textContent = s.name || '—';
    const sub = [];
    if (s.cnName && s.cnName !== s.name) sub.push(s.cnName);
    if (s.typeLabel) sub.push(s.typeLabel);
    if (s.connState) sub.push(s.connState === 'connected' ? '上游已连接' : '上游未连接');
    this.drawerSub.textContent = sub.join(' · ') || '—';

    const ecef = s.ecef;   // 引擎 $SRCINFO 回填的 ECEF，形如 {x,y,z}；也可能是旧格式字符串
    let h = '';

    h += this._group('标识');
    h += this._kv('基站名', s.name, true);
    h += this._kv('来源', s.isVirtual
      ? '引擎生成的 VRS 虚拟参考点（不是真实基站，$SRCINFO 中 srcId 为 -1）'
      : s.fromEngine === false
        ? '台账中登记，但引擎本轮 $SRCINFO 未上报'
        : s.runtimeOnly
          ? '运行期新增（不在 conf/ntripsources，引擎重启后需重新 addsource）'
          : 'conf/ntripsources 配置文件');
    /* ── 以下严格按 solution.c solution_srcinfos() 的输出字段与顺序 ──
     *
     * 引擎注释写明的 17 个字段：
     *   站名，站ID，NTRIP标识，NTRIP IP地址，端口，NTRIP用户，NTRIP密码，
     *   NTRIP挂载点，连接状态，位置 ECEF，站ID，天线类型，天线序号，
     *   Mark名，接收机类型，接收机序号，接收机版本号
     *
     * 刻意**不展示**台账独有字段（中文名/英文名/行政区/省/市/建站年份/
     * 天线高/方位角/厂商/类型说明）：本机 base-stations.info 解析为空，
     * 它们恒为「未设置」—— 一屏全是「未设置」会淹没真正有用的信息。
     * 台账字段中只有「中文名」在站名匹配上时作为副标题保留（见表格渲染）。
     *
     * 引擎报了两个 srcId：ntrip 源 ID（--8s %4d 的 %4d）与
     * 测站 srcId（sta.srcid，%3d），二者不同，故分列两行。 */
    h += this._kv('NTRIP 源ID（站ID）', fmt(s.srcId));
    h += this._kv('NTRIP 标识', s.type);
    h += this._kv('测站 srcId（站ID）', fmt(s.staSrcId));

    h += this._group('坐标');
    h += this._kv('纬度', fmt(s.lat, 6));
    h += this._kv('经度', fmt(s.lon, 6));
    h += this._kv('高程', s.height === null || s.height === undefined ? null
                    : fmt(s.height, 3) + ' m');
    /* ECEF 的形态随来源而异：$SRCINFO 给的是 {x,y,z} 对象，
     * 早期代码把引擎的 "x,y,z" 字符串原样透传。两种都要能显示 ——
     * 若只认字符串，改成对象后这里会显示成 "[object Object]"。
     * 这是引擎上报的**权威位置**，精度高于经纬度。 */
    if (ecef && typeof ecef === 'object') {
      h += this._kv('ECEF-X', fmt(ecef.x, 3) + ' m', true);
      h += this._kv('ECEF-Y', fmt(ecef.y, 3) + ' m', true);
      h += this._kv('ECEF-Z', fmt(ecef.z, 3) + ' m', true);
    } else if (typeof ecef === 'string' && ecef) {
      const p = ecef.split(',');
      h += this._kv('ECEF-X', p[0], true);
      h += this._kv('ECEF-Y', p[1], true);
      h += this._kv('ECEF-Z', p[2], true);
    } else {
      h += this._kv('ECEF 坐标', null);
    }

    h += this._group('上游连接（NTRIP）');
    h += this._kv('NTRIP IP地址', s.addr, true);
    h += this._kv('端口', fmt(s.port));
    /* NTRIP 用户名引擎会输出；**密码刻意不显示** —— 没有排障价值，
     * 却会随 WebSocket 快照进入浏览器内存、开发者工具与截图。
     * 需排障时看引擎输出或 conf/ntripsources。 */
    h += this._kv('NTRIP 用户', s.user);
    h += this._kv('NTRIP 密码', '（不显示）');
    h += this._kv('NTRIP 挂载点', s.mountpoint, true);
    /* 连接状态：$SRCINFO 的 state 字段（connected / disconnected）。
     * 引擎未定位时 pos 为 "---"，但 state 仍会给出，故以 state 为准。 */
    h += this._kv('连接状态', s.state === 'connected' ? '已连接'
                      : s.state === 'disconnected' ? '未连接'
                      : (s.connState === 'connected' ? '已连接'
                         : s.connState === 'disconnected' ? '未连接' : null));

    /* 天线与接收机分两组 —— 现场排障时「换天线」和「换接收机」
     * 是两件独立的事，混在一起会看错。 */
    h += this._group('天线');
    h += this._kv('天线类型', s.antDes);
    h += this._kv('天线序号', s.antSno, true);
    h += this._kv('Mark 名', s.marker);

    h += this._group('接收机');
    h += this._kv('接收机类型', s.recType);
    h += this._kv('接收机序号', s.recSno, true);
    h += this._kv('接收机版本号', s.recVer);

    this.drawerBody.innerHTML = h;
    this.drawer.hidden = false;
  };

  global.MCORS.StationPanel = StationPanel;
})(window);