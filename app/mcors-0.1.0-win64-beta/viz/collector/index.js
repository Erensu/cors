/**
 * mcors 实时监控大屏 · 采集层主程序
 *
 * 职责：
 *   1. 连 src/cors/solution.c 的推送服务(TCP 8080) —— **唯一数据来源**
 *   2. 连 cors-engine 控制台(TCP 9000)           —— **仅**下发控制命令
 *      （增删站 / 增删基线 / 重载配置），不再读取任何状态数据
 *   3. 读 conf/ 站点字典
 *   4. 通过 WebSocket 把结构化 JSON 推给前端大屏
 *   5. 提供 HTTP 接口（静态资源 + REST 控制命令）
 *
 * ⚠ 数据源收敛说明（2026-10-04）：
 *   原先有两条数据通道 —— 控制台轮询（showbls / sourceinfo all / showtrinet）
 *   与 monitor 订阅（TCP 7999，NMEA 实时流）。现两者**全部废弃**，
 *   统一由 solution 流的四类帧供给：
 *     $SRCINFO  → 基站信息 + 站名↔srcId 映射（原 sourceinfo all）
 *     $NET[cors]→ 三角网（原 showtrinet / monitor 的 $NET 帧）
 *     $BLSOLS   → 基线解算信息（原 showbls，但字段与格式完全不同）
 *     $SRCSOLS  → NMEA（原 monitor 的 $RTK 帧）
 *   好处：一次连接、一轮数据原子到达（同轮的四个面板必然来自同一时刻），
 *   且不再需要「轮询命令 + 静默超时切应答」这套脆弱的同步机制。
 *
 * 零外部依赖：WebSocket 握手与帧编码自行实现（见 ws-lite.js）。
 */
const http = require('http');
const fs = require('fs');
const path = require('path');

const cfg = require('./config');
const ConsoleClient = require('./console-client');
const SolutionClient = require('./solution-client');
const { StationDict } = require('./station-dict');
const { SrcIdMap } = require('./bls-parser');
const {
  parseSrcInfo, parseBlsols, parseNet, parseSrcSols
} = require('./solution-parser');
const trinetParser = require('./trinet-parser');
const { createMockEngine } = require('./mock-engine');
const WSLite = require('./ws-lite');

// ---------------------------------------------------------------- 状态存储

const store = {
  source: cfg.web.source,
  startedAt: Date.now(),
  console: { connected: false, lastError: null, lastAt: null },
  solution: {
    connected: false,
    frames: 0,
    byType: { SRCINFO: 0, NET: 0, BLSOLS: 0, SRCSOLS: 0 },
    lastFrameAt: null,
    lastRound: '',
    host: null,
    errors: 0
  },
  stations: [],
  baselines: [],
  dtrigs: [],
  satellites: [],
  // src/cors/solution.c 的 $NET[cors] 提供权威三角网，前端网图优先用它；
  // 引擎不可用时前端回退到 viz/web/js/delaunay.js 本地重算
  trinet: {
    nodes: [],
    edges: [],
    triangles: [],
    warnings: [],
    source: 'engine',
    fetchedAt: null,
    error: null
  },
  latest: new Map(), // 站名 -> 最近一帧的解析结果
  srcIds: new SrcIdMap(),
  // srcid -> 站点名（来自 $NET 顶点名或 $SRCINFO），供三角网做中文名关联
  srcIdIndex: new Map(),
  // 最近一次 $NET 帧的原始 CSV。保留它是为了在站名↔srcId 映射迟到时重建站网图
  trinetCsv: null,
  history: new Map(), // key -> 时序数组（定位质量，key 形如 'RTK:站名'）

  /* 逐星双差延迟的时序缓冲，key 形如 'BL:基线名|卫星名'。
   *
   * 与 history 分开是因为粒度不同：history 是每基线一组，
   * 这里是**每颗卫星**一组（10 基线 × 9 星 ≈ 90 组），且点更轻。
   * 前端点击逐星表格里的延迟值时按 key 取曲线。 */
  satHistory: new Map(),
  // 各曲线最后一次被更新的时间戳（ms），用于「只画最近活跃的基线」与老化清理
  satHistorySeen: new Map(),
  log: []
};

function log(level, msg) {
  const item = { t: Date.now(), level, msg };
  store.log.push(item);
  if (store.log.length > 300) store.log.shift();
  const ts = new Date(item.t).toISOString().slice(11, 19);
  console.log(`[${ts}] ${level.toUpperCase()} ${msg}`);
}

/* ================================================================
 * history 的保留策略
 *
 * 实测（1920x1080、12 条基线）：
 *   history 有 12 组 × 324 点 = 3888 点，单点 81 字节 → 312 KB
 *   而 /api/snapshot 总量 325 KB，即 history 占 **96%**
 *
 * 前端每秒通过 WebSocket 全量收到这份数据，但 main.js 只取
 * **一组**（优先 RTK:<选中站>，否则第一个 RTK:），
 * `bl:` 开头的 12 组从未被前端读取。
 *
 * 这不是理论问题 —— 3 个客户端每秒各收 325 KB，
 * 一小时就是 3.5 GB；JSON.stringify 还在主线程上每秒跑 3 次。
 *
 * 因此：
 *   - API 端（/api/snapshot）保留完整 history，便于排障与离线分析；
 *   - WebSocket 广播时对每组做**窗口截断 + 字段精简**，
 *     趋势图本来就只看最近若干点、也只画 5 个指标。
 * ================================================================ */

/* 趋势图实际只画最近这一段窗口，超出的点前端用不到 */
const TREND_WINDOW = 180;      /* 每组广播点数（约 3 分钟 @1Hz） */

/**
 * 广播用的 history 裁剪：**逐组**截窗口 + 剔无关字段。
 *
 * 为什么不按组数裁剪（这是本函数的原设计，也是「趋势图无曲线」的根因）：
 *   原实现用 TREND_MAX_GROUPS = 1 只发**第一组** RTK:，
 *   而前端 main.js 的 trendKey 是按「当前选中站」拼的 `RTK:${selectedStation}`。
 *   两者只有在「选中站恰好等于第一组」时才一致 ——
 *   用户点网图/基线表/站点表切换站后，选中的是别的站，
 *   s.history[trendKey] 就是 undefined，前端只能 setData([]) 显示「暂无趋势数据」。
 *   实测 mock 下 7 组里只有 1 组能命中，其余 6 组全空。
 *
 * 为什么放开组数后体积可接受（实测 7 组 × 180 点）：
 *   history 单点 JSON 约 62 字节（精简后），7 × 180 × 62 ≈ 76 KB；
 *   而同一份快照里 latest 光是 sats 数组就有 37.8 KB ——
 *   放开 history 的增量远小于已有开销，换来「任意站都能画曲线」，
 *   这笔交易划算。真正的膨胀来自 720 点全量，故仍保留窗口截断。
 *
 * @param {Object<string, Array<object>>} h 完整 history
 * @returns {Object<string, Array<object>>} 裁剪后的 history
 */
function trimHistory(h) {
  if (!h) return {};
  const out = {};
  for (const k of Object.keys(h)) {
    const a = h[k];
    if (!a || !a.length) continue;
    const win = a.length > TREND_WINDOW ? a.slice(-TREND_WINDOW) : a;
    /* 精简到趋势图真正画的 5 个指标 + ts。
     * 丢掉 level（解状态）—— 趋势图不用它，而基线表另有数据源。 */
    out[k] = win.map((p) => ({
      ts: p.ts,
      hdop: p.hdop,
      pdop: p.pdop,
      hdopGsa: p.hdopGsa,
      vdop: p.vdop,
      numSat: p.numSat
    }));
  }
  return out;
}

/**
 * 逐星延迟时序的广播裁剪：逐组截窗口 + 剔无关字段。
 *
 * 与 trimHistory 同思路但更严格：这里的组数是「基线数 × 卫星数」，
 * 实测 10 基线 9 星 = 90 组，故每组压到 satHistoryWindow 点、
 * 且只留 4 个字段（ts + 三路延迟 + 高度角）。
 *
 * 另**剔除单点组**：只有一个点的曲线画不出趋势，只会让前端多画一条
 * 只有一个 marker 的线，徒增视觉噪音。用户刚点开就看到「等待更多历元」
 * 比看到一条孤零零的折线更诚实。
 *
 * @param {Object<string, Array<object>>} h 完整 satHistory
 * @returns {Object<string, Array<object>>} 裁剪后的
 */
function trimSatHistory(h) {
  if (!h) return {};
  const out = {};
  const win = cfg.satHistoryWindow;
  for (const k of Object.keys(h)) {
    const a = h[k];
    if (!a || !a.length) continue;
    if (a.length < 2) continue;
    const w = a.length > win ? a.slice(-win) : a;
    out[k] = w.map((p) => ({
      ts: p.ts, atmo: p.atmo, trpd: p.trpd, iond: p.iond, el: p.el
    }));
  }
  return out;
}

/**
 * 为前端裁剪快照：剥掉它不消费的字段、裁剪 history。
 * 供 broadcast() 使用（WebSocket 专用）；
 * /api/snapshot 仍返回完整数据，便于排障。
 */
function snapshotForPush() {
  const full = snapshot();
  const slim = Object.assign({}, full);
  /* history 广播**所有**组（逐组截窗口 + 精简字段）。
   * 原先只发第一组 RTK:，导致前端按选中站取 key 时几乎必然落空、
   * 趋势图恒显「暂无趋势数据」；放开组数的体积代价可接受，
   * 详见 trimHistory 的注释。 */
  slim.history = trimHistory(full.history);
  /* historyKeys 告诉前端「有哪些组可选」，让它能显示切换器 */
  slim.historyKeys = full.history ? Object.keys(full.history) : [];
  /* satHistory 同理广播，但**每组只留 satHistoryWindow 个点**。
   * 组数 = 基线数 × 卫星数（可达 90+），不截窗口体积会顶到几百 KB，
   * 而逐星延迟看的是最近几分钟的走势，更长的历史前端画不出来。 */
  slim.satHistory = trimSatHistory(full.satHistory);
  /* latest 里的 nmea 数组前端不用（skyplot 只要 sats/usedSats 等），
   * 实测占 latest 的绝大部分，这里剔除 */
  if (full.latest) {
    const l = {};
    for (const k of Object.keys(full.latest)) {
      const v = full.latest[k] || {};
      /* 白名单按 onMonitorFrame 里 payload 的**实际字段**列全
       * （ts/type/base/rover/quality/qualityLabel/level/lat/lon/alt/
       *   hdop/numSat/age/speedKn/courseDeg/utc/pdop/hdopGsa/vdop/
       *   sats/usedSats）。
       * 之前这里凭印象写，漏了 hdopGsa 与 type —— 趋势图的
       * 「HDOP(GSA)」指标会因此拿不到值。改用「除白名单外全剔除」，
       * 漏项时表现为多余字段（无害）而不是缺字段（静默失效）。 */
      l[k] = {
        ts: v.ts, type: v.type, base: v.base, rover: v.rover,
        quality: v.quality, qualityLabel: v.qualityLabel, level: v.level,
        lat: v.lat, lon: v.lon, alt: v.alt,
        hdop: v.hdop, pdop: v.pdop, hdopGsa: v.hdopGsa, vdop: v.vdop,
        numSat: v.numSat, age: v.age,
        speedKn: v.speedKn, courseDeg: v.courseDeg, utc: v.utc,
        sats: v.sats, usedSats: v.usedSats
      };
    }
    slim.latest = l;
  }
  return slim;
}

function pushHistory(key, point) {
  if (!store.history.has(key)) store.history.set(key, []);
  const arr = store.history.get(key);
  arr.push(point);
  const limit = cfg.historyLimit;
  while (arr.length > limit) arr.shift();
}

/**
 * 诊断数据能力。
 *
 * 背景（实测结论，2026-10-03）：
 *   conf/ntripsources 全部 7 个源都指向挂点 RTCM32eph，经真实抓取验证
 *   （130 帧全部通过 CRC24Q 校验）该挂点只推星历报文：
 *     1019 GPS星历 / 1020 GLONASS星历 / 1042 BeiDou星历
 *     1044 / 1045 / 1046 Galileo星历
 *   零 MSM 观测报文（107x/108x/109x/111x）、零 1005 基准站坐标。
 *
 *   这意味着：引擎能正常收流、能分发星历，但拿不到伪距/载波相位观测值，
 *   基线解算无法收敛，天空图也没有可见星。
 *
 *   前端据此区分"没数据"与"有数据但解算不出"，避免展示误导性的空面板。
 */
function diagnoseCapabilities() {
  /* 帧计数改用 solution 流：它每轮必推四类帧，
   * 故 frames>0 代表「引擎在推数据」，语义比旧的 monitor 计数更准确。 */
  const frames = store.solution.frames;
  const dict = store.stationDict;
  const stations = dict ? dict.list() : [];

  // 统计已收到的报文构成
  let withObs = 0;
  for (const p of store.latest.values()) {
    if (!p) continue;
    if (p.sats && p.sats.length) withObs++;
    else if (p.numSat) withObs++;
  }

  const online = stations.filter((s) => s.online).length;
  const isMock = cfg.web.source === 'mock';

  // 判定链条：先看数据形态，再看链路，最后才看解算。
  //
  // EPH_ONLY 必须排在 MOCK 之前 —— 仅星历模式（MCORS_MOCK_EPH_ONLY=1）
  // 正是用来复现真实上游「只推星历」条件的，若被 MOCK 提前截断就验证不到横幅。
  let level = 'ok';
  let code = 'OK';
  let msg = '数据正常';
  let hint = '';

  if (frames > 0 && withObs === 0) {
    level = 'warn';
    code = 'EPH_ONLY';
    msg = isMock ? '仅星历模式（模拟）' : '仅收到星历，无观测值';
    hint =
      '上游 NTRIP 挂点（RTCM32eph）只推星历报文，不含 MSM 观测值。' +
      '基线解算与卫星天空图需要观测值（107x/108x/109x/111x）才能工作。' +
      '星历分发本身不受影响。';
  } else if (isMock) {
    code = 'MOCK';
    msg = '模拟数据模式';
    hint = '当前为 mock 数据源，未连接真实引擎。设置 MCORS_SOURCE=live 可切换到真实链路。';
  } else if (!store.solution.connected) {
    level = 'error';
    code = 'NO_SOLUTION';
    msg = '未连接 solution 数据源';
    hint = `无法连接 solution 推送服务（${cfg.solution.port} 端口）。`
         + '请确认 cors-engine 已启动：该服务由 src/cors/solution.c 提供，'
         + '随引擎 start 一起拉起。';
  } else if (frames === 0) {
    level = 'warn';
    code = 'NO_FRAMES';
    msg = '已连接但无数据帧';
    hint = 'solution 通道已连接但一帧未收到。正常情况下每 1.5s 应到一轮'
         + '（$SRCINFO/$NET/$BLSOLS/$SRCSOLS 各一帧）。';
  } else if (!store.baselines.length) {
    level = 'warn';
    code = 'NO_BASELINE';
    msg = '有观测值但无基线';
    hint = '已收到观测数据，但尚未建立基线。可在下方"基站管理"中配置基线关系。';
  }

  return {
    level,
    code,
    msg,
    hint,
    frames,
    online,
    total: stations.length,
    withObs,
    source: store.source
  };
}

/** 组装推送给前端的完整快照 */
function snapshot() {
  const dict = store.stationDict;
  const stations = dict ? dict.list() : [];

  // 站点在线状态 + 解算状态回填
  const blByRover = new Map();
  for (const b of store.baselines) {
    const prev = blByRover.get(b.roverSrcId);
    // 固定解优先展示
    if (!prev || (prev.level !== 'fix' && b.level === 'fix')) {
      blByRover.set(b.roverSrcId, b);
    }
  }

  const stationFeatures = stations.map((s) => {
    const bl = blByRover.get(s.srcId);
    return {
      ...s,
      nameMatched: s.nameMatched !== false && !!s.cnName,
      solLevel: bl ? bl.level : null,
      solLabel: bl ? bl.label : null,
      stat: bl ? bl.stat : null
    };
  });

  /* 把「虚拟站」并入 stations，供站网图绘制。
   *
   * 背景：stations 来自 conf 站点字典（stationDict.list()），
   * 而虚拟站有两个来源，字典里都装不下：
   *   1) conf/ntripsources 第 10 列类型 ='V' 的源 —— 这类**会**进字典
   *      （normalizeSrcType 认V），但仅当用户显式配置才存在；
   *   2) 引擎 VRS 生成的虚拟参考点（$SRCINFO 中 srcId < 0）——
   *      只出现在 srcInfoStations，字典里永远没有。
   * 实测当前 conf 的 7 个源全是 M/E，无 V，故网图一个虚拟站都画不出来。
   *
   * 做法：字典侧标记 isVirtual（type==='V'），srcInfo 侧按 isVirtual 标记，
   * 两者按站名去重合并进 stations —— 网图因此不必关心虚拟站从哪来。
   * 只在字典里没有同名站时才追加，避免覆盖台账里更完整的字段。 */
  const merged = stationFeatures.slice();
  const byName = new Map(merged.map((s) => [s.name, s]));
  for (const r of stationFeatures) {
    if (r.type === 'V' && !byName.has(r.name)) byName.set(r.name, r);
  }
  for (const r of store.srcInfoStations || []) {
    if (!r || !r.isVirtual) continue;         // 只补虚拟站，真实站仍以字典为准
    if (byName.has(r.name)) {
      /* 字典里已登记同名虚拟站：补上引擎侧的在线状态与精确坐标，
       * 但保留字典的其他字段。 */
      const t = byName.get(r.name);
      t.isVirtual = true;
      if (r.pos && !t.ecef) t.ecef = r.pos;
      if (r.connected) t.online = true;
      continue;
    }
    /* 引擎独有的虚拟站：转成网图可用的最小站点对象。
     * srcInfoStations 已由 applySrcInfo 换算好 lat/lon/hgt（用引擎的
     * 权威 ECEF + Bowring 闭式解），直接复用，不必再转一次。 */
    if (!Number.isFinite(r.lat) || !Number.isFinite(r.lon)) continue;  // 无坐标无法定位，跳过
    const vs = {
      name: r.name,
      /* 字典站有 height，虚拟站没有；给 0 会让「台账高程」列显示 0，
       * 用 null 更诚实 —— 前端 num() 会显示「—」。 */
      height: null,
      type: 'V',
      typeLabel: '虚拟',
      typeDesc: '虚拟参考站',
      isVirtual: true,
      runtimeOnly: true,
      lat: r.lat,
      lon: r.lon,
      hgt: r.hgt,
      ecef: r.pos,
      /* 用 staSrcId（测站 srcId）而非 id（NTRIP 源 id）：
       * 基线关联用的是测站 srcId，保持与字典侧 srcId 同语义。 */
      srcId: r.staSrcId,
      online: r.connected === true,
      cnName: null,
      addr: r.addr || '',
      mntpnt: r.mountpoint || '',
      connState: r.state || null
    };
    byName.set(r.name, vs);
    merged.push(vs);
  }

  return {
    ts: Date.now(),
    source: store.source,
    uptimeMs: Date.now() - store.startedAt,
    /* 数据源状态：solution 流取代了原先的 console 轮询 + monitor 订阅。
     * console 仅剩「控制命令」用途，故其连接状态单独保留以便排障
     * （控制不通只影响增删站，不影响画面数据）。 */
    solution: {
      ...store.solution,
      status: store.solutionClient ? store.solutionClient.status() : null
    },
    console: { ...store.console },
    caps: diagnoseCapabilities(),
    dictStats: dict ? dict.stats() : null,
    // 站名↔srcId 映射的规模，排障用：若为 0 则站网图节点名会退化成数字
    sourceInfoCount: store.sourceInfoCount || 0,
    srcIdMapSize: store.srcIds && store.srcIds.map ? store.srcIds.map.size : 0,
    stations: merged,
    /* $SRCINFO 的原始解析结果。
     *
     * 单独暴露的原因：stations 来自 conf/ 站点字典，只含「台账里登记过」的站；
     * 而 $SRCINFO 是**引擎侧实际在用**的源列表 —— 运行期 addsource 加进来的源
     * 可能不在台账里。两者取并集才能既不漏、也不凭空多。
     * 另外设备信息（天线/接收机型号版本）只有引擎侧有，也在这份数据里。 */
    srcInfoStations: store.srcInfoStations || [],
    baselines: store.baselines.map((b) => ({
      ...b,
      /* 显示名优先用 srcid 映射（$SRCINFO 权威建立、稳定不翻）；
       * 映射尚未建立时回退 $BLSOLS 报头名 —— 没有这个回退，
       * 映射缺失的 id 会把报头里明明带的名字显示成 null。 */
      baseName: store.srcIds.resolve(b.baseSrcId) || b.baseName || null,
      roverName: store.srcIds.resolve(b.roverSrcId) || b.roverName || null
    })),
    dtrigs: store.dtrigs,
    // 三角网：前端网图优先用引擎侧权威拓扑；
    // source='engine' 表示来自 showtrinet，前端不得自行重算覆盖
    trinet: store.trinet,
    satellites: store.satellites,
    latest: Object.fromEntries(store.latest),
    history: Object.fromEntries(store.history),
    /* 逐星延迟时序：key 形如 'BL:基线→基线|卫星f频点'。
     * 完整版（本地缓冲 satHistoryLimit 历元）仅 /api/snapshot 提供。 */
    satHistory: Object.fromEntries(store.satHistory)
  };
}

// ---------------------------------------------------------------- 数据接入

/**
 * $NET[cors] → 三角网（基站网图）
 *
 * 内层 CSV（H/V/E/T/END）与控制台 showtrinet **逐字节一致**，
 * 故直接复用 trinet-parser，不需要为 solution 流另写解析器。
 *
 * 与旧 monitor $NET 帧的差别：旧帧只在站网变化时推送（低频事件），
 * 新流**每轮必推**（1.5s 一次），因此这里是幂等覆盖而非增量更新。
 *
 * @param {string} payload 帧载荷（CSV 文本）
 */
function onNetFrame(payload) {
  const nameMap = buildSrcIdMap();
  const graph = trinetParser.toGraph(payload, { nameMap });
  store.trinet = {
    ...graph,
    source: 'solution',
    fetchedAt: Date.now(),
    error: null
  };
  /* 保留原始 CSV：V 行的站名字段实测常为空（引擎侧 sta 尚未关联），
   * 而 $SRCINFO 的映射可能在后一帧才到位。留着 CSV 才能在映射就绪后
   * 用同一份拓扑重建图，否则顶点名会一直停在纯数字。 */
  store.trinetCsv = payload;

  /* 站网变化是低频事件，却要等最多 1 秒的定时广播才发出去，
   * 操作者会明显感到"延迟"。这里立即广播一跳拉到 100ms 级。
   * 带宽无压力：单帧约 1KB，1.5s 一次。 */
  broadcast();
}

/**
 * 站名↔srcId 映射建立后（或补全后）重建站网图。
 *
 * $NET 帧只在站网变化时推送，而 sourceinfo 轮询要等控制台连通，
 * 因此很可能出现「先收到 $NET（此时映射还是空的）→ 之后映射才建好」的顺序。
 * 若不重建，前端会一直看到纯数字的节点名。
 */
function rebuildTrinetIfStale() {
  if (!store.trinetCsv) return;
  const nodes = store.trinet && store.trinet.nodes;
  if (!nodes || !nodes.length) return;

  /* 判据是「**有**数字名」而非「**全部**是数字名」。
   *
   * 原判断 `nodes.some(非数字名) → return` 看似等价，实则相反：
   * 只要有一个顶点拿到了站名就整体放弃重建，于是**部分**顶点缺名
   * （运行期 addsource 新增的源，其映射要等下一轮 sourceinfo）
   * 会一直停在纯 srcId 上，看不到名字。
   *
   * 全部都有站名时才跳过，避免每 3 秒无谓重算。 */
  const hasNumberName = nodes.some((n) => !n.name || /^[0-9]+$/.test(n.name));
  if (!hasNumberName) return;

  const nameMap = buildSrcIdMap();
  if (!nameMap.size) return;
  const graph = trinetParser.toGraph(store.trinetCsv, { nameMap });
  if (!graph.nodes.length) return;
  /* 重建后若仍有数字名，说明映射确实缺这一项，保留原图不覆盖，
   * 等下一轮 sourceinfo 补齐后再试（避免半成品来回抖动）。 */
  const stillBad = graph.nodes.some((n) => !n.name || /^[0-9]+$/.test(n.name));
  if (stillBad && !nodes.some((n) => !n.name || /^[0-9]+$/.test(n.name))) return;

  store.trinet = {
    ...graph,
    source: store.trinet.source || 'monitor',
    fetchedAt: Date.now(),
    error: null
  };
}

/**
 * 清理已不存在的运行期站点。
 *
 * 对称于 upsertRuntime：引擎 delsource 后，sourceinfo 里不再有该源，
 * 但字典里的记录还在 → 站点表会留一个残影，且网图仍尝试画它
 * （byName 命中但引擎已无此顶点）。
 *
 * 只删 runtimeOnly 的记录 —— conf/ntripsources 里的正式台账永不自动删除，
 * 那属于人工维护的配置文件，引擎侧删源不应把它抹掉（重启后仍在）。
 */
/**
 * 清理已不存在的运行期站点。
 *
 * @param {Set<string>} aliveNames 本轮仍存在的运行期源名
 * @param {number} graceMs 保护期，仅 sourceinfo 轮询路径需要（见调用处注释）
 */
function pruneRuntimeStations(aliveNames, graceMs) {
  if (!store.stationDict) return 0;
  const removed = store.stationDict.pruneRuntime?.(aliveNames, graceMs) || 0;
  /* 同步清掉 srcId 映射，否则 buildSrcIdMap 会拿到已删源的 srcId，
   * 而引擎侧该 srcId 已被回收、下一个源可能复用它 → 张冠李戴。 */
  for (const [id, name] of [...(store.srcIds?.map || [])]) {
    if (!store.stationDict.get(name)) store.srcIds.map.delete(id);
  }
  return removed;
}

/** 处理 monitor 推送的一帧 */
/**
 * solution 帧总入口：按帧头分派。
 *
 * solution_write_thread 每 1500ms 推一轮，四类帧顺序固定：
 *   $SRCINFO → $NET[cors] → $BLSOLS → $SRCSOLS
 * 顺序固定这一点被 solution-client 记录在 lastRound 里，便于排障
 * （若某轮少了某类帧，说明引擎侧组帧有问题）。
 *
 * @param {{type:string, base:string|null, rover:string|null, payload:string}} frame
 */
function onSolutionFrame(frame) {
  switch (frame.type) {
    case 'SRCINFO': onSrcInfoFrame(frame.payload); break;
    case 'NET':     onNetFrame(frame.payload);     break;
    case 'BLSOLS':  onBlsolsFrame(frame.payload);  break;
    case 'SRCSOLS': onSrcSolsFrame(frame);         break;
    default:
      log('warn', `未知帧类型: ${frame.type}`);
  }
  store.solution.frames++;
  store.solution.byType[frame.type] = (store.solution.byType[frame.type] || 0) + 1;
  store.solution.lastFrameAt = Date.now();
}

/**
 * $SRCINFO → 基站信息
 *
 * 这一帧**取代**了旧的 sourceinfo all 轮询，同时承担「站名↔srcId 权威映射」
 * 的职责 —— 它每行同时给出站名(第1列)与引擎内部 srcId(第11列)，
 * 是全系统唯一能把数字 ID 翻成站名的来源（$NET 的 V 行第2列也是站名，
 * 但实测常为空字符串，不可依赖）。
 */
function onSrcInfoFrame(payload) {
  const { stations: rows, warnings } = parseSrcInfo(payload);
  warnings.forEach((w) => log('warn', w));

  store.srcInfoStations = rows;

  /* 由 $SRCINFO 的 ECEF 反算经纬度。
   *
   * 为什么必须在这里算：conf 台账（base-stations.info）里可能有经纬度，
   * 但**运行期 addsource 加的源不在台账里**，以及本机这台机器上
   * base-stations.info 解析为空 —— 两种情况都会让「位置」列变成 —
   * 而 $SRCINFO 每行都带权威 ECEF，据此换算才是始终可用的来源。
   * 转换用 trinet-parser 的 Bowring 闭式解（与三角网顶点同一套）。 */
  for (const r of rows) {
    if (!r.pos) { r.lat = null; r.lon = null; r.hgt = null; continue; }
    const geo = trinetParser.ecefToGeodetic(r.pos.x, r.pos.y, r.pos.z);
    r.lat = geo ? geo.lat : null;
    r.lon = geo ? geo.lon : null;
    r.hgt = geo ? geo.hgt : null;
  }

  /* 建立/刷新 站名↔srcId 映射。
   * 这里是最可靠的一处：$SRCINFO 的站名与 srcId 同行出现，
   * 不需要任何跨帧关联或时序假设。 */
  for (const r of rows) {
    if (!r.name) continue;
    store.srcIds.set(r.staSrcId || r.id, r.name);
    if (store.srcIdIndex) store.srcIdIndex.set(r.staSrcId || r.id, r.name);
  }

  /* 把引擎侧信息回填到站点字典（台账）。
   *
   * 台账提供的是「人工维护的事实」：中文名、行政区、天线高、站点类型。
   * 引擎提供的是「运行期事实」：连接状态、ECEF、上游地址、设备型号。
   * 两者按站名对齐后，面板才能同时显示两类信息。 */
  if (store.stationDict) {
    for (const r of rows) {
      const s = store.stationDict.get(r.name);
      if (!s) continue;
      s.online = r.connected;
      if (r.connected) s.lastSeen = Date.now();
      s.ecef = r.pos;
      s.addr = r.addr;
      s.port = r.port;
      s.mountpoint = r.mountpoint;
      s.connState = r.state;
      s.antDes = r.antDes;
      s.antSno = r.antSno;
      s.marker = r.marker;
      s.recType = r.recType;
      s.recSno = r.recSno;
      s.recVer = r.recVer;
      /* 台账缺经纬度时用引擎算出的补上（不覆盖台账已有值，
       * 台账是人工测绘的成果，精度应优于运行时换算）。 */
      if (!Number.isFinite(s.lat) && Number.isFinite(r.lat)) s.lat = r.lat;
      if (!Number.isFinite(s.lon) && Number.isFinite(r.lon)) s.lon = r.lon;
    }
  }

  // 映射可能刚就绪，补建一次站网图（$NET 只每轮推，但顶点名可能此前为空）
  rebuildTrinetIfStale();
}

/**
 * $BLSOLS → 基线解算信息
 *
 * 与旧 showbls 的差异（前端字段因此变化）：
 *   旧：dd/eq 行，报**双差模糊度**（逐方程）
 *   新：逐星行，报**双差大气/对流层/电离层延迟 + 高度角**，另有 poserr 三分量
 * 站名直接内嵌在头行（`3[KURJ00AUS0]->6[MGRV00AUS0]`），
 * 因此基线表**不再需要**靠 SrcIdMap 反查站名。
 */
function onBlsolsFrame(payload) {
  const { baselines, warnings } = parseBlsols(payload, {
    // 解状态 → 语义分类，复用 bls-parser 的表，避免两处维护
    solLevel: (stat) => require('./bls-parser').solLevel(stat)
  });
  warnings.forEach((w) => log('warn', w));

  /* 名称回填到 srcIds：$BLSOLS 头行给了 id[name] 配对。
   *
   * ⚠ 只补缺（setIfAbsent），不能无条件覆盖 —— $SRCINFO 才是权威映射
   * 来源（站名与 srcId 同行出现，且行序固定）。实测该站存在**双挂载**
   * （MGRV00AUS0 id=4 与 BCEP00BKG0 id=5 的 staSrcId 都是 4），引擎
   * 每历元用哪个挂载点的流、报头就写哪个名字；若在此覆盖，同一 srcid
   * 的名字会随帧翻动（BCEP↔MGRV），前端基线表跟着变。 */
  for (const b of baselines) {
    if (b.baseName) store.srcIds.setIfAbsent(b.baseSrcId, b.baseName);
    if (b.roverName) store.srcIds.setIfAbsent(b.roverSrcId, b.roverName);
  }

  store.baselines = baselines;
  learnSrcIds();
  recordSatHistory(baselines);

  // 基线解算状态回填到站点字典（在线/解状态）
  if (store.stationDict) {
    for (const b of baselines) {
      const nm = b.roverName || store.srcIds.resolve(b.roverSrcId);
      if (!nm) continue;
      const s = store.stationDict.get(nm);
      if (s) { s.online = true; s.lastSeen = Date.now(); }
    }
  }
}

/**
 * 逐星双差延迟时序累积。
 *
 * 背景：$BLSOLS 每帧只给**当前历元**的逐星延迟，而排障要看的是
 * 「这颗星的电离层是不是在跳变」——那需要历史曲线。
 * store.baselines 是全量替换的，不留历史，故这里另开一份缓冲。
 *
 * key 设计为 `BL:基线名|卫星名`（如 `BL:KURJ00AUS0→MGRV00AUS0|G05`）：
 *   - 必须含**基线名**：同一颗星在 10 条基线上是 10 个独立的双差量，
 *     混在一条曲线里毫无意义；
 *   - 用 `|` 分隔符而不是 `:`，因为站名本身可能含 `:`。
 *
 * **刻意不带频点后缀**（曾写成 `PRNf<freq>`，已改）：引擎 solution.c:236-238
 * 输出逐星行时，三路延迟一律取 `atmc[0] / trpc[0] / ionc[0]` —— 硬编码索引 0，
 * 与循环变量 `f` 无关。实测（live 模式）确认同一颗星的 f0/f1/f2 三个频点
 * 数值**完全相同**。故按频点分组只会把一条曲线复制成 2~3 条一模一样的，
 * 用户点哪条都一样，纯浪费缓冲与广播体积。
 * 等引擎侧改成 `atmc[f]` 后再加回频点后缀；前端 seriesForSat 同理只按星名匹配。
 *
 * 只保留最近的 satHistoryMaxBaselines 条基线：曲线组数 = 基线数 × 卫星数，
 * 不设上限会随站点规模线性膨胀，而排障关心的总是当前在解的那几条。
 */
function recordSatHistory(baselines) {
  const now = Date.now();

  /* 按「最后一次出现」排序取前 N 条基线，保证被记录的是正在解算的那些。
   * 稳定排序：同 ts 时保持原序，避免每帧顺序抖动导致曲线随机丢一条。 */
  const ranked = baselines
    .map((b, i) => ({ b, i, ts: lastSeenOf(b) }))
    .sort((x, y) => (y.ts - x.ts) || (x.i - y.i))
    .slice(0, cfg.satHistoryMaxBaselines);

  for (const { b } of ranked) {
    const base = b.baseName || store.srcIds.resolve(b.baseSrcId);
    const rover = b.roverName || store.srcIds.resolve(b.roverSrcId);
    if (!base || !rover) continue;   // 站名未回填时 key 不稳定，跳过
    const bl = `${base}→${rover}`;

    for (const s of b.sats || []) {
      if (!s || !s.prn) continue;
      /* 三路延迟全缺的历元不记：记了会在曲线上插出一个 0，
       * 看成「延迟突然归零」，比没有这条点更具误导性。 */
      const hasAny =
        Number.isFinite(s.atmo) || Number.isFinite(s.trpd) || Number.isFinite(s.iond);
      if (!hasAny) continue;

      const key = `BL:${bl}|${s.prn}`;
      if (!store.satHistory.has(key)) store.satHistory.set(key, []);
      const arr = store.satHistory.get(key);
      arr.push({
        ts: now,
        atmo: numOrNull(s.atmo),
        trpd: numOrNull(s.trpd),
        iond: numOrNull(s.iond),
        el: numOrNull(s.el)
      });
      const limit = cfg.satHistoryLimit;
      while (arr.length > limit) arr.shift();
      store.satHistorySeen.set(key, now);
    }
  }

  /* 老化清理：超过 3 个窗口没更新的曲线直接丢掉。
   * 不清理的话，删掉的基线 / 换掉的卫星会永久占着内存与广播体积。 */
  const stale = cfg.satHistoryLimit * 3 * 1000;
  for (const [key, ts] of store.satHistorySeen) {
    if (now - ts > stale) {
      store.satHistorySeen.delete(key);
      store.satHistory.delete(key);
    }
  }
}

/** 该基线最近一次出现的时刻；没有就退回 0，保证排序稳定。 */
function lastSeenOf(b) {
  const t = Date.parse(b.time || '');
  return Number.isFinite(t) ? t : 0;
}

function numOrNull(v) {
  return Number.isFinite(v) ? v : null;
}

/**
 * $SRCSOLS → NMEA（定位质量趋势 + 卫星天空图）
 *
 * 取代旧的 monitor $RTK 帧。帧头形如 $SRCSOLS[base->rover]，
 * 内容是 outsols()/outsolexs() 出的一组标准 NMEA。
 *
 * 注意与旧 monitor 帧的差别：旧帧的 base/rover 是站名且**逐个挂载点**推送；
 * 新帧每条基线推一组，站名同样内嵌在帧头，故 key 用 base->rover 更精确。
 */
function onSrcSolsFrame(frame) {
  applySrcSols(frame, parseSrcSols(frame.payload));
}

/**
 * 把「已解析的 NMEA 组」写入 store。
 *
 * 抽成独立函数是为了让 live（solution 流，文本载荷）与 mock（monitor 式对象帧）
 * 共用同一套 store 更新逻辑 —— 否则两条路径的字段会各写一遍、早晚不一致。
 *
 * @param {{base:string|null, rover:string|null}} frame 帧头信息
 * @param {{gga:object|null, rmc:object|null, gsa:object[], gsv:object[]}} p
 */
function applySrcSols(frame, p) {
  const { gga, rmc, gsa, gsv } = p;
  if (!gga && !rmc && !gsa.length && !gsv.length) return;   // 空帧（该基线尚无解）

  const sats = [];
  for (const g of gsv) sats.push(...g.sats);

  const usedSet = new Set();
  for (const g of gsa) for (const x of g.used) usedSet.add(x);

  /* key 用 RTK: 前缀 —— snapshotForPush 靠这个前缀挑「前端真正会读的那一组」
   * 来裁剪 history，改动前缀会让裁剪逻辑失效、快照体积暴涨。 */
  const key = `RTK:${frame.rover || frame.base || 'unknown'}`;

  const payload = {
    ts: Date.now(),
    type: 'RTK',
    base: frame.base,
    rover: frame.rover,
    quality: gga ? gga.quality : null,
    qualityLabel: gga ? gga.qualityLabel : null,
    level: gga ? gga.level : null,
    lat: gga ? gga.lat : null,
    lon: gga ? gga.lon : null,
    alt: gga ? gga.alt : null,
    hdop: gga ? gga.hdop : null,
    numSat: gga ? gga.numSat : null,
    age: gga ? gga.age : null,
    speedKn: rmc ? rmc.speedKn : null,
    courseDeg: rmc ? rmc.courseDeg : null,
    utc: gga ? gga.utc : rmc ? rmc.utc : null,
    pdop: gsa.length ? gsa[0].pdop : null,
    hdopGsa: gsa.length ? gsa[0].hdop : null,
    vdop: gsa.length ? gsa[0].vdop : null,
    sats,
    usedSats: [...usedSet]
  };

  store.latest.set(key, payload);
  store.satellites = sats;
  if (store.latest.size > 200) {
    const firstKey = store.latest.keys().next().value;
    store.latest.delete(firstKey);
  }

  pushHistory(key, {
    ts: payload.ts,
    hdop: payload.hdop,
    pdop: payload.pdop,
    hdopGsa: payload.hdopGsa,
    vdop: payload.vdop,
    numSat: payload.numSat,
    level: payload.level
  });
}

/**
 * 从基线解算结果学习 srcid -> 站点名。
 *
 * $BLSOLS 的头行已内嵌站名，故这里基本只是把结果搬进索引；
 * 保留此函数是因为站网图的顶点名可能仍是空字符串，
 * 需要靠这份索引把 srcId 翻成可读站名。
 *
 * ⚠ 只补缺（与 onBlsolsFrame 的回填同理）：双挂载下报头名字随历元翻，
 * 无条件覆盖会让映射在两个挂载点名之间抖动。权威来源始终是 $SRCINFO。
 */
function learnSrcIds() {
  if (!store.srcIdIndex) return;
  for (const b of store.baselines) {
    if (b.baseName && !store.srcIdIndex.has(b.baseSrcId))
      store.srcIdIndex.set(b.baseSrcId, b.baseName);
    if (b.roverName && !store.srcIdIndex.has(b.roverSrcId))
      store.srcIdIndex.set(b.roverSrcId, b.roverName);
    if (b.baseName) store.srcIds.setIfAbsent(b.baseSrcId, b.baseName);
    if (b.roverName) store.srcIds.setIfAbsent(b.roverSrcId, b.roverName);
  }
}

function buildSrcIdMap() {
  const map = new Map();
  if (!store.stationDict) return map;

  // 1) 权威来源：store.srcIds（由 $SRCINFO 的「站名+srcId 同行」建立，
  //    并由 $BLSOLS 头行的 id[name] 持续补充）。它不依赖任何跨帧时序假设，
  //    因此 $NET 帧无论何时到达都能翻出站名。
  if (store.srcIds && store.srcIds.map) {
    for (const [id, name] of store.srcIds.map) {
      const s = store.stationDict.get(name) || store.stationDict.getByMountPoint?.(name);
      if (s) map.set(Number(id), s);
    }
  }
  // 2) 兜底：从基线解算结果积累的关联
  if (store.srcIdIndex) {
    for (const [id, name] of store.srcIdIndex) {
      if (map.has(id)) continue;
      const s = store.stationDict.get(name) || store.stationDict.getByMountPoint?.(name);
      if (s) map.set(id, s);
    }
  }
  // 3) 兜底：若字典里的站点已带 srcId 字段，直接收进来
  for (const s of store.stationDict.list()) {
    if (Number.isFinite(s.srcId) && !map.has(s.srcId)) map.set(s.srcId, s);
  }
  return map;
}

// ---------------------------------------------------------------- HTTP/WS

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.png': 'image/png',
  '.svg': 'image/svg+xml',
  '.ico': 'image/x-icon'
};

const WEB_ROOT = path.join(__dirname, '..', 'web');

function jsonRes(res, code, obj) {
  const body = JSON.stringify(obj);
  res.writeHead(code, {
    'Content-Type': 'application/json; charset=utf-8',
    'Content-Length': Buffer.byteLength(body),
    'Access-Control-Allow-Origin': '*'
  });
  res.end(body);
}

/** 读取请求体并解析 JSON */
function readBody(req) {
  return new Promise((resolve, reject) => {
    let d = '';
    req.on('data', (c) => {
      d += c;
      if (d.length > 1e6) reject(new Error('请求体过大'));
    });
    req.on('end', () => {
      if (!d) return resolve({});
      try {
        resolve(JSON.parse(d));
      } catch (e) {
        reject(new Error('JSON 解析失败'));
      }
    });
    req.on('error', reject);
  });
}

/** 执行控制台命令（供 REST 调用） */
async function execConsoleCommand(cmd) {
  if (!store.consoleClient || !store.consoleClient.connected) {
    return { ok: false, error: '引擎控制台未连接，无法下发命令' };
  }
  try {
    const out = await store.consoleClient.send(cmd);
    return { ok: true, cmd, output: out };
  } catch (e) {
    return { ok: false, cmd, error: e.message };
  }
}

const server = http.createServer(async (req, res) => {
  const url = new URL(req.url, `http://${req.headers.host || 'localhost'}`);
  const p = url.pathname;

  // CORS 预检
  if (req.method === 'OPTIONS') {
    res.writeHead(204, {
      'Access-Control-Allow-Origin': '*',
      'Access-Control-Allow-Methods': 'GET,POST,OPTIONS',
      'Access-Control-Allow-Headers': 'Content-Type'
    });
    return res.end();
  }

  try {
    // ---- API ----
    if (p === '/api/snapshot') {
      return jsonRes(res, 200, snapshot());
    }

    if (p === '/api/stations') {
      const dict = store.stationDict;
      return jsonRes(res, 200, {
        type: 'FeatureCollection',
        features: dict ? dict.toGeoJSON().features : []
      });
    }

    if (p === '/api/dict/reload' && req.method === 'POST') {
      if (store.stationDict) {
        store.stationDict.reload();
        store.stationDict.enrichNames();
      }
      return jsonRes(res, 200, { ok: true, stats: store.stationDict ? store.stationDict.stats() : null });
    }

    // 通用控制台命令
    if (p === '/api/console' && req.method === 'POST') {
      const body = await readBody(req);
      if (!body.cmd || typeof body.cmd !== 'string') {
        return jsonRes(res, 400, { ok: false, error: '缺少 cmd 字段' });
      }
      // 白名单校验：只允许已知的引擎命令
      const ALLOWED = [
        'start', 'stop', 'sourceinfo', 'showbls', 'showblsols', 'showdtrigs',
        'showsubnet', 'showvstas', 'showusers', 'showambdds', 'showblsif',
        'showblswl', 'showatmodddelay', 'satellite', 'navidata', 'observ',
        'monisrcinfo', 'agentinfo'
      ];
      const verb = body.cmd.trim().split(/\s+/)[0];
      if (ALLOWED.indexOf(verb) < 0) {
        return jsonRes(res, 403, {
          ok: false,
          error: `命令 "${verb}" 不在只读白名单内`,
          allowed: ALLOWED
        });
      }
      const r = await execConsoleCommand(body.cmd);
      return jsonRes(res, 200, r);
    }

    // ---- 基站增删（写操作） ----
    if (p === '/api/station/add' && req.method === 'POST') {
      const body = await readBody(req);
      const err = validateStation(body);
      if (err) return jsonRes(res, 400, { ok: false, error: err });

      // engine.c cmd_add_source 参数：
      //   args[1]=name args[2]=addr args[3]=port args[4]=user args[5]=passwd
      //   args[6]=mntpnt args[7]=lat(度) args[8]=lon(度) args[9]=h
      //   args[10]=type(M/E/O/V)
      const cmd =
        `addsource ${body.name} ${body.addr} ${body.port} ` +
        `${body.user} ${body.passwd} ${body.mntpnt} ` +
        `${body.lat} ${body.lon} ${body.height} ${body.type}`;

      /* 先登记进字典，**再**下发命令。
       *
       * 顺序很关键：sourceinfo 轮询每 3s 一次，命令下发到引擎接受
       * 再到下一次轮询看到新源，中间有几百毫秒到数秒的窗口。若先下发
       * 命令再登记，轮询可能先跑到 —— 那时 pollSourceInfo 里的
       * upsertRuntime 会用 sourceinfo 的数据建记录：它给的是
       * 连接类型（"NTRIP"，不是 M/E/O/V）且**没有高程**。由于
       * upsertRuntime 承诺「不覆盖已有记录」，之后 add 接口再登记
       * 就会被跳过 → 用户填的类型被悄悄改成 M、高程变 null
       * （实测表格显示「— — 混合」）。
       * 先登记则用户的输入是权威首值，轮询只回填 srcId/ecef/状态。
       *
       * 登记失败（引擎拒绝）时 pruneRuntimeStations 会在下一轮
       * sourceinfo 发现该源不存在并清掉，不会留下残影。 */
      store.stationDict?.upsertRuntime?.({
        name: body.name,
        addr: body.addr,
        port: Number(body.port),
        user: body.user,
        mntpnt: body.mntpnt,
        lat: Number(body.lat),
        lon: Number(body.lon),
        height: Number(body.height),
        type: body.type
      });

      const r = await execConsoleCommand(cmd);
      if (r.ok) {
        log('info', `已下发基站新增命令: ${body.name}`);
        /* 走 upsertRuntime 而不是直接 sources.set()：
         *   1) 直接 set 漏了 runtimeOnly=true，而 pruneRuntimeStations 只
         *      清理 runtimeOnly 的记录 → 该源会变成永久残影，delsource
         *      后站点表仍留着这一行（实测「运行期」标记消失）；
         *   2) 漏了 typeDesc，详情抽屉的类型说明为空；
         *   3) 漏了 ECEF 反解逻辑的入口。 */
      }
      return jsonRes(res, 200, r);
    }

    if (p === '/api/station/del' && req.method === 'POST') {
      const body = await readBody(req);
      if (!body.name) return jsonRes(res, 400, { ok: false, error: '缺少 name 字段' });
      const r = await execConsoleCommand(`delsource ${body.name}`);
      if (r.ok && store.stationDict) {
        store.stationDict.sources.delete(body.name);
        log('info', `已下发基站删除命令: ${body.name}`);
      }
      return jsonRes(res, 200, r);
    }

    // ---- 基线管理 ----
    if (p === '/api/baseline/add' && req.method === 'POST') {
      const body = await readBody(req);
      if (!body.base || !body.rover) {
        return jsonRes(res, 400, { ok: false, error: '缺少 base / rover 字段（基站名）' });
      }
      // engine.c cmd_rtkpos_addbl: rtkpos -b <base> -r <rover>
      const r = await execConsoleCommand(`rtkpos -b ${body.base} -r ${body.rover}`);
      return jsonRes(res, 200, r);
    }

    if (p === '/api/baseline/del' && req.method === 'POST') {
      const body = await readBody(req);
      if (!body.base || !body.rover) {
        return jsonRes(res, 400, { ok: false, error: '缺少 base / rover 字段（基站名）' });
      }
      const r = await execConsoleCommand(`rtkpos -d -b ${body.base} -r ${body.rover}`);
      return jsonRes(res, 200, r);
    }

    // ---- 静态资源 ----
    let rel = p === '/' ? '/index.html' : p;
    rel = rel.replace(/\.\./g, '');
    const file = path.join(WEB_ROOT, rel);
    if (fs.existsSync(file) && fs.statSync(file).isFile()) {
      const ext = path.extname(file).toLowerCase();
      const data = fs.readFileSync(file);
      res.writeHead(200, {
        'Content-Type': MIME[ext] || 'application/octet-stream',
        'Content-Length': data.length,
        // 大屏是排障工具，JS/CSS 改动后必须立刻生效。
        // 不设任何缓存头时浏览器会启发式缓存，改了采集层逻辑却看到旧行为，
        // 很容易误判成"修复没起作用"。
        'Cache-Control': 'no-cache, no-store, must-revalidate',
        'Pragma': 'no-cache'
      });
      return res.end(data);
    }

    res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
    return res.end('404 Not Found');
  } catch (e) {
    return jsonRes(res, 500, { ok: false, error: e.message });
  }
});

/** 基站参数校验 */
function validateStation(s) {
  if (!s.name || !/^[A-Za-z0-9_]{1,32}$/.test(s.name)) {
    return '基站名非法：仅允许字母数字下划线，长度 1-32';
  }
  /* ⚠ 原来的 /^[0-9.]+$/ 只校验「字符集」，999.999.1.1 这种明显非法的
   * 地址会一路通过、被下发到引擎。引擎拿它去解析/连接，失败路径会
   * 把引擎带进异常状态。
   *
   * 判定顺序很关键：**纯数字点分串只允许按 IPv4 规则判定**，不能落到
   * 主机名兜底上 —— 否则 "999.999.1.1" 会被当成合法主机名放行
   * （这正是第一版修复没拦住的原因）。
   * 只有含字母/连字符的才走主机名规则。 */
  if (!s.addr) return 'NTRIP 地址非法（不能为空）';
  const addr = String(s.addr).trim();
  const isIPv4 = (a) => {
    const parts = a.split('.');
    if (parts.length !== 4) return false;
    return parts.every((p) => /^\d{1,3}$/.test(p) && Number(p) <= 255);
  };
  const isHostname = (a) =>
    /^[A-Za-z0-9]([A-Za-z0-9-]{0,61}[A-Za-z0-9])?(\.[A-Za-z0-9]([A-Za-z0-9-]{0,61}[A-Za-z0-9])?)*$/.test(a);
  if (/^[0-9.]+$/.test(addr)) {
    /* 只用数字和点：按 IPv4 严格校验，不合法即拒。 */
    if (!isIPv4(addr)) {
      return 'NTRIP 地址非法：应为合法 IPv4（如 192.168.1.10）';
    }
  } else if (!isHostname(addr)) {
    return 'NTRIP 地址非法：应为合法 IPv4 或主机名';
  }
  const port = Number(s.port);
  if (!Number.isInteger(port) || port < 1 || port > 65535) return '端口非法';
  const lat = Number(s.lat);
  const lon = Number(s.lon);
  if (!Number.isFinite(lat) || lat < -90 || lat > 90) return '纬度非法';
  if (!Number.isFinite(lon) || lon < -180 || lon > 180) return '经度非法';
  const h = Number(s.height);
  if (!Number.isFinite(h)) return '高程非法';
  if (!['M', 'E', 'O', 'V'].includes(String(s.type || '').toUpperCase())) {
    return '类型非法：应为 M(混合)/E(仅星历)/O(仅观测)/V(VRS)';
  }
  if (!s.mntpnt) return '挂载点不能为空';
  return null;
}

// ---------------------------------------------------------------- 启动

const wsl = new WSLite(server);
wsl.on('connection', (client) => {
  client.send(snapshot());
  log('info', `前端已连接 (${wsl.clients.size} 个客户端)`);
});

function broadcast() {
  let snap;
  // 构造快照本身若抛错，不应影响定时器与主进程
  try {
    /* 用裁剪版：实测 325KB → 约 12KB，前端只消费它真正用到的部分。
     * 完整数据仍可通过 /api/snapshot 获取。 */
    snap = snapshotForPush();
  } catch (e) {
    log('error', `快照生成失败: ${e.message}`);
    return;
  }
  try {
    wsl.broadcast(snap);
  } catch (e) {
    log('error', `广播失败: ${e.message}`);
  }
}

let pollTimer = null;
let broadcastTimer = null;

/**
 * 兼容层：把 mock 的 monitor 式帧转成 solution 式的 store 更新。
 *
 * ⚠ 为什么还需要它：mock-engine.js 复刻的是**旧的 monitor 协议**
 *   （`$RTK[base->rover]` + NMEA，以及 `$NET`），而 live 路径已改走
 *   solution 流。两者最终写入的 store 字段完全一致，故这里只做一次
 *   形状适配，避免让 mock 也重写一遍组帧逻辑。
 *
 * live 路径**不走**这里 —— 它由 onSolutionFrame 直接分派。
 * 若后续把 mock-engine 也改成产出 solution 四类帧，本函数即可删除。
 */
function onMockFrame(frame) {
  if (!frame) return;
  if (frame.type === 'NET') {
    if (frame.csv) onNetFrame(frame.csv);
    return;
  }
  // RTK / PNT：NMEA 帧。mock 给的是已解析对象，直接喂共用的 store 更新逻辑
  if (Array.isArray(frame.nmea)) {
    const p = { gga: null, rmc: null, gsa: [], gsv: [] };
    for (const n of frame.nmea) {
      if (n.type === 'GGA') p.gga = n;
      else if (n.type === 'RMC') p.rmc = n;
      else if (n.type === 'GSA') p.gsa.push(n);
      else if (n.type === 'GSV') p.gsv.push(n);
    }
    applySrcSols(frame, p);
  }
}

function startMock() {
  // 模拟源产生的帧走与 live 一致的处理入口，保证两条路径行为一致
  store.onMockFrame = onMockFrame;
  store.mock = createMockEngine(store);
  log('info', '已启用模拟数据源（MCORS_SOURCE=mock）');
}

function start() {
  // 站点字典
  store.stationDict = new StationDict(cfg.conf.dir);
  if (store.stationDict.load()) {
    store.stationDict.enrichNames();
    const st = store.stationDict.stats();
    log('info', `站点字典已加载: ${st.total} 站, 基站信息 ${st.baseInfoCount} 条, sourcetable ${st.sourcetableCount} 条`);
    if (st.errors.length) st.errors.forEach((e) => log('warn', `字典: ${e}`));
  } else {
    log('error', '站点字典加载失败，站网图将为空');
  }

  if (store.source === 'mock') {
    startMock();
  } else {
    /* ---- 唯一数据源：solution 推送服务 ---- */
    store.solutionClient = new SolutionClient(cfg.solution, onSolutionFrame,
      (err) => {
        store.solution.errors++;
        store.console.lastError = err.message;
        log('warn', `[solution] ${err.message}`);
      });
    store.solutionClient.connect();

    // 把连接状态同步进快照（前端横幅据此判断链路是否通）
    const iv = setInterval(() => {
      const c = store.solutionClient;
      store.solution.connected = !!(c && c.connected);
      store.solution.host = c ? c.host : null;
      if (c && c.status) {
        const st = c.status();
        store.solution.lastRound = st.lastRound || '';
      }
    }, 1000);
    iv.unref && iv.unref();

    /* ---- 控制台通道：仅用于下发控制命令 ---- *
     * 增删站走 addsource/delsource、增删基线走 rtkpos -addbl/-delbl，
     * 这些是「输出到引擎」而非「从引擎读数据」，因此保留。
     * 不再有任何状态轮询 —— 那部分已由 solution 流接管。 */
    store.consoleClient = new ConsoleClient({
      ...cfg.console,
      reconnectDelay: cfg.solution.reconnectDelay
    });
    store.consoleClient.connect();

    const iv2 = setInterval(() => {
      store.console.connected = !!(store.consoleClient && store.consoleClient.connected);
    }, 1000);
    iv2.unref && iv2.unref();
  }

  // 广播
  broadcastTimer = setInterval(broadcast, 1000);

  server.listen(cfg.web.port, cfg.web.host, () => {
    log('info', `大屏服务已启动: http://localhost:${cfg.web.port}`);
    log('info', `数据来源: ${store.source}`);
    if (store.source === 'live') {
      log('info', `数据源 solution ${cfg.solution.hostCandidates.join('/')}:${cfg.solution.port}`);
      log('info', `控制台（仅控制命令）${cfg.console.hostCandidates.join('/')}:${cfg.console.port}`);
    }
  });
}

function shutdown() {
  log('info', '正在关闭...');
  if (pollTimer) clearInterval(pollTimer);
  if (broadcastTimer) clearInterval(broadcastTimer);
  if (store.consoleClient) store.consoleClient.close();
  if (store.solutionClient) store.solutionClient.close();
  if (store.mock && store.mock.stop) store.mock.stop();
  wsl.close();
  server.close(() => process.exit(0));
  setTimeout(() => process.exit(0), 2000);
}

process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);

start();