/**
 * solution.c 推送流解析器
 *
 * ══════════════════════════════════════════════════════════════════
 * 数据源：src/cors/solution.c 的 solution_write_thread()
 *
 * 该线程每 **1500ms** 推一轮，4 类帧**顺序固定**，全部用 <<...>> 包裹：
 *
 *   <<$SRCINFO                     基站信息（17 列定宽表格）
 *   <<$NET[cors]                   三角网 H/V/E/T/END CSV
 *   <<$BLSOLS                      基线解算信息（逐双差方程）
 *   <<$SRCSOLS[base->rover]        标准 NMEA（GGA/RMC/GSA/GSV）
 *
 * 与旧通道的关系：本文件**取代**了原先「控制台轮询 showbls/sourceinfo/
 * showtrinet（TCP 9000）+ monitor 订阅 NMEA（TCP 7999）」两条通道。
 * 可视化现在只连 solution.c 的 8080。
 *
 * 复用关系（刻意不重复造轮子）：
 *   $NET[cors] 的内层 CSV 与控制台 showtrinet **逐字节一致**
 *       → 直接交给 trinet-parser.js 的 parseTrinet/toGraph
 *   $SRCSOLS 的内层是标准 NMEA
 *       → 直接交给 nmea-parser.js 的 parseLine
 *   本文件只实现真正新增的两种：$SRCINFO 与 $BLSOLS。
 * ══════════════════════════════════════════════════════════════════
 */
const { parseLine } = require('./nmea-parser');
const { parseShowBls } = require('./bls-parser');

/** 帧头：$SRCINFO | $NET[cors] | $BLSOLS | $SRCSOLS[A->B] */
const HEADER_RE = /^\$([A-Z]+)(?:\[(.*)\])?$/;

/**
 * 把 TCP 字节流切成帧。
 *
 * 分帧规则与 monitor 协议一致（都是 <<...>> 包裹），但**不 trim 行**：
 * $SRCINFO 是定宽表格，尾部的 `%20s` 空字段会留一串空格，
 * trim 掉之后「从末尾取 125 字符」的定位就会错位。
 * 因此这里保留原始行，由各内容解析器自行决定要不要 trim。
 */
class SolutionFrameParser {
  constructor() {
    this.buf = '';
  }

  /**
   * 喂入任意字节，返回本轮解析完成的帧数组。
   * @param {string} chunk
   * @returns {Array<{type:string, base:string|null, rover:string|null, payload:string}>}
   */
  push(chunk) {
    this.buf += chunk;
    const frames = [];

    let guard = 0;
    while (guard++ < 1000) {
      const s = this.buf.indexOf('<<');
      if (s < 0) {
        // 没有帧头，丢弃可能的垃圾（但保留尾部，可能只是半截 '<<'）
        if (this.buf.length > 65536) this.buf = '';
        break;
      }
      const e = this.buf.indexOf('>>', s + 2);
      if (e < 0) {
        this.buf = this.buf.slice(s);   // 保留未完成帧
        break;
      }
      const body = this.buf.slice(s + 2, e);
      this.buf = this.buf.slice(e + 2);

      const f = this._split(body);
      if (f) frames.push(f);
    }
    return frames;
  }

  /** 拆出帧头行与载荷 */
  _split(body) {
    const nl = body.indexOf('\n');
    const headLine = (nl < 0 ? body : body.slice(0, nl)).trim();
    const payload = nl < 0 ? '' : body.slice(nl + 1);

    const m = headLine.match(HEADER_RE);
    if (!m) return null;

    const type = m[1];
    const arg = m[2];   // 'cors' 或 'base->rover' 或 undefined

    if (type === 'NET') {
      return { type: 'NET', base: null, rover: null, payload };
    }
    if (type === 'SRCSOLS') {
      const [b, r] = String(arg || '').split('->');
      return {
        type: 'SRCSOLS',
        base: (b || '').trim() || null,
        rover: (r || '').trim() || null,
        payload
      };
    }
    // SRCINFO / BLSOLS：无括号参数
    return { type, base: null, rover: null, payload };
  }
}

/* ==================================================================
 * $SRCINFO —— 基站信息
 *
 * 输出格式（solution.c solution_srcinfos）：
 *
 *   "%8s %4d %8s %16s %6d %8s %13s %12s %12s %40s %3d %20s %20s %20s %20s %20s %20s\n"
 *    站名 ID NTRIP 地址 端口 用户 密码 挂载点 连接状态 位置ECEF 站ID 天线类型 天线序号
 *    Mark名 接收机类型 接收机序号 接收机版本
 *
 * ── 为什么不能简单 split(/\s+/) ────────────────────────────────
 * 两个字段会破坏「按空白切分」：
 *   ① `%8s` 的站名**经常溢出** —— 实测站名 10 字符（KURJ00AUS0），
 *      printf 会原样打出 10 字符，导致其后所有列右移，按位置取会全错；
 *   ② 尾部 6 个 `%20s`（天线型号/Mark名/接收机型号等）**本身含空格**，
 *      如 "TRM59800.00 SCIS"、"TRIMBLE NETR9"，按空白切会把一个字段拆成两半。
 *
 * ── 采用的策略：头按空白切、尾按定宽切 ────────────────────────
 *   头（前 11 个字段）：站名/ID/.../位置/站ID —— 除「位置」外都是无空格
 *      的单 token，且站名溢出只影响宽度不影响 token 数，故按空白切分最稳。
 *      「位置」是 `%8.3lf,%8.3lf,%8.3lf`，ECEF 量级 1e5~1e7 必然超出 8 字符
 *      宽度，因此实际输出不含填充空格，可当单 token。
 *   尾（后 6 个字段）：每个 `%20s` 定宽，合计 6*20 + 5 个分隔空格 = **125 字符**，
 *      且锚定在**行尾**，与头部是否溢出无关 → 取末 125 字符按 21 步长切。
 *
 * 行内若带前导空格（`%8s` 右对齐），先 trimStart 再切头；尾部空格必须保留。
 * ================================================================== */

/** 尾部 6 个定宽字段的总宽：6*20 + 5 个分隔符 */
const TAIL_WIDTH = 125;
const TAIL_CELL = 20;
const TAIL_STEP = 21;   // 20 + 1 分隔

/**
 * 解析 $SRCINFO 载荷。
 *
 * @param {string} payload 帧载荷（不含帧头行）
 * @returns {{stations:Array, warnings:string[]}}
 */
function parseSrcInfo(payload) {
  const stations = [];
  const warnings = [];

  for (const raw of String(payload || '').split(/\r?\n/)) {
    if (!raw.trim()) continue;

    /* 尾部锚定切片。必须先取尾部再 trimStart 头部，
     * 否则 trim 会把尾部空字段的空格吃掉、125 宽度不成立。 */
    const tail = raw.length >= TAIL_WIDTH
      ? raw.slice(raw.length - TAIL_WIDTH)
      : raw.padStart(TAIL_WIDTH, ' ');
    const head = raw.length >= TAIL_WIDTH ? raw.slice(0, raw.length - TAIL_WIDTH) : '';

    /* 尾部 6 个设备字段。
     *
     * 实测（2026-10-04，真实引擎输出）标定出的布局：
     *   'LEIAR10         NONE             19353032   ...   Trimble Alloy   6443R40004   6.50,20/MAR/2026'
     *   [  0,  7) 'LEIAR10'      [ 16, 20) 'NONE'     [ 33, 41) '19353032'
     *   [ 70, 83) 'Trimble Alloy'  [ 94,104) '6443R40004'  [109,125) '6.50,20/MAR/2026'
     *
     * 即：**后 3 格（接收机型号/序号/版本）严格落在 [63,83) [84,104) [105,125)**，
     * 而定宽切片对**前 3 格**不可靠 —— 引擎侧这几个字符串本身长短不一，
     * 导致「天线型号 / 天线序号 / Mark名」三个值被挤进同一个 20 字符格里
     * （上例中 cell0 = 'LEIAR10         NONE'）。
     *
     * 因此采取混合策略：
     *   前 3 格 → 把 0~63 区间的内容按空白摊平，顺序填充 天线型号/天线序号/Mark名
     *             （这三者都是不含空格的单 token，摊平后不会串位）
     *   后 3 格 → 按定宽切，能正确处理 'Trimble Alloy' 这类**含空格**的值
     * 空字段（未定位/无设备的源）两种策略都自然产出空串。 */
    const headCells = tail.slice(0, 63).split(/\s+/).filter(Boolean);
    const cellOf = (k) => tail.slice(k * TAIL_STEP, k * TAIL_STEP + TAIL_CELL).trim();

    /* 头部解析：**两端锚定**，不用固定 token 数。
     *
     * 为什么不能用「固定 11 个 token」：头部中间有 4 个可选字段
     * （addr / user / passwd / mntpnt），为空时 `%Ns` 只输出空格，
     * 按空白分词就**消失了**。实测 VRS 虚拟站的行是：
     *   `  VRS001   -1    NTRIP                       0  ...  connected  <ECEF>  -1`
     * 只有 7 个 token（name/id/type/port/state/pos/staSrcId），
     * 若要求 11 个就会把合法的 VRS 记录整条丢掉。
     * 同理也不能纯按列位置切：`%8s` 的站名（实测 10 字符）与
     * `%16s` 的域名地址（实测 25 字符）都会溢出，把后续列整体右移。
     *
     * 两端锚定则对上述两种情况都免疫：
     *   前 3 个 token 恒为 name / id / type（三者都不为空）
     *   后 3 个 token 恒为 state / pos / staSrcId（三者都不为空）
     *   中间剩下的是 addr/port/user/passwd/mntpnt 的子集，顺序固定
     * 站名与地址是否溢出、可选字段缺几个，都不影响这三对锚点。 */
    const t = head.trim().split(/\s+/).filter(Boolean);
    if (t.length < 6) {
      warnings.push(`$SRCINFO 行字段过少(${t.length})，已跳过: ${raw.trim().slice(0, 80)}`);
      continue;
    }
    if (t[2] !== 'NTRIP') {
      warnings.push(`$SRCINFO 列错位(第3列应为 NTRIP，实为 "${t[2]}")，已跳过: ${raw.trim().slice(0, 80)}`);
      continue;
    }

    const name = t[0];
    const id = Number(t[1]);
    const stateStr = t[t.length - 3];
    const posStr = t[t.length - 2];
    const staSrcId = Number(t[t.length - 1]);

    /* 中间可选段：addr/port/user/passwd/mntpnt 保序子集。
     * 只做「够用即可」的识别，不做严格列映射 ——
     * 可视化只需 addr 与 mountpoint 展示，port 辅助，
     * user/passwd 本就不展示（passwd 更是刻意丢弃）。
     * 判据：纯数字 = port；首个非数字 = addr；末个非数字 = mountpoint。 */
    const mid = t.slice(3, t.length - 3);
    let addr = '', port = 0, mountpoint = '';
    const nonNum = mid.filter((x) => !/^-?\d+$/.test(x));
    const numTok = mid.find((x) => /^-?\d+$/.test(x));
    if (nonNum.length) addr = nonNum[0];
    if (numTok !== undefined) port = Number(numTok);
    if (nonNum.length > 1) mountpoint = nonNum[nonNum.length - 1];

    let pos = null;
    if (posStr && posStr !== '---') {
      const p = posStr.split(',').map(Number);
      if (p.length === 3 && p.every(Number.isFinite)) pos = { x: p[0], y: p[1], z: p[2] };
    }

    stations.push({
      name,
      id,
      type: t[2],                       // 恒为 "NTRIP"
      addr,
      port,
      /* NTRIP 明文密码**刻意丢弃**：没有展示价值，却会随 WebSocket 快照
       * 进入浏览器内存、开发者工具与截图。需要排障时看引擎输出或 conf。 */
      mountpoint,
      state: stateStr,
      connected: stateStr === 'connected',
      pos,                              // ECEF，可能为 null（未定位）
      posText: posStr === '---' ? null : posStr,
      staSrcId,
      /* 是否是虚拟站：引擎的 VRS 虚拟点 srcId 为 -1、无地址与挂载点。
       * 前端据此区分「真实基站」与「引擎生成的虚拟参考点」。 */
      isVirtual: id < 0 || staSrcId < 0,
      /* 尾部 6 格：设备信息（前 3 格按空白摊平、后 3 格按定宽，见上方说明） */
      antDes: headCells[0] || '',       // 天线型号
      antSno: headCells[1] || '',       // 天线序号
      marker: headCells[2] || '',       // Mark 名
      recType: cellOf(3),               // 接收机类型（可能含空格）
      recSno: cellOf(4),                // 接收机序号
      recVer: cellOf(5)                 // 接收机版本
    });
  }

  return { stations, warnings };
}

/* ==================================================================
 * $BLSOLS —— 基线解算信息
 *
 * 一条基线固定打「2 + N」行：
 *   A) 头行   "2026/10/04 12:34:56.000:   3[KURJ00AUS0]->  6[MGRV00AUS0]: "
 *   B) 汇总行 " stat=   FIX bl=    21KM age= 0s nb= 18 ns= 12 poserr= 0.123 -0.456  0.789"
 *   C) 逐星×N " G05-> G12: freq=0 stat=     3 age= 0s atmo= 0.123 trpd= 0.456 iond= 0.789 el= 45.678"
 *
 * 注意与旧控制台 showbls 的**本质差异**（解析器不能复用）：
 *   - 旧格式是 dd/eq 两行，报的是**双差模糊度**；
 *   - 新格式把双差量换成了**逐星大气/对流层/电离层延迟 + 高度角**，
 *     模糊度不再输出。故 bls-parser.js 的 parseShowBls 不适用。
 *
 * 新增 poserr（三分量定位误差）与 ns（参与解算卫星数）。
 * ================================================================== */

/** 头行：`<时间>: <基站ID>[<基站名>]-><流动站ID>[<流动站名>]: ` */
const BLS_HEAD_RE = /^(.*?):\s*(-?\d+)\s*\[([^\]]*)\]\s*->\s*(-?\d+)\s*\[([^\]]*)\]\s*:\s*$/;

/** 汇总行：stat / bl / age / nb / ns / poserr(x y z) */
const BLS_SUM_RE =
  /stat=\s*(\S+)\s+bl=\s*(-?[\d.]+)\s*KM\s+age=\s*(-?[\d.]+)\s*s\s+nb=\s*(-?\d+)\s+ns=\s*(-?\d+)\s+poserr=\s*(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)/;

/** 逐星行：<prn>-> <refprn>: freq= stat= age= atmo= trpd= iond= el= */
const BLS_SAT_RE =
  /^\s*(\S+)\s*->\s*(\S+)\s*:\s*freq=\s*(\d+)\s+stat=\s*(-?\d+)\s+age=\s*(-?[\d.]+)\s*s\s+atmo=\s*(-?[\d.]+)\s+trpd=\s*(-?[\d.]+)\s+iond=\s*(-?[\d.]+)\s+el=\s*(-?[\d.]+)/;

/**
 * 解析 $BLSOLS 载荷。
 *
 * @param {string} payload
 * @param {object} [opts]
 * @param {(stat:string)=>{level:string,label:string}} [opts.solLevel]
 *        解状态 → 语义分类的函数（由调用方注入，避免与 bls-parser 重复）
 * @returns {{baselines:Array, warnings:string[]}}
 */
function parseBlsols(payload, opts = {}) {
  const baselines = [];
  const warnings = [];
  const solLevel = opts.solLevel || ((stat) => ({ stat, level: 'none', label: stat || '未知' }));

  let cur = null;

  for (const raw of String(payload || '').split(/\r?\n/)) {
    if (!raw.trim()) continue;

    // --- A) 头行：开启一条新基线 ---
    const h = raw.match(BLS_HEAD_RE);
    if (h) {
      cur = {
        time: h[1].trim(),
        baseSrcId: Number(h[2]),
        baseName: h[3].trim() || null,
        roverSrcId: Number(h[4]),
        roverName: h[5].trim() || null,
        /* 以下由汇总行填充，先占位以便前端字段稳定 */
        stat: null, level: 'none', label: null,
        baselineKm: null, ageS: null,
        nb: null, ns: null, poserr: null,
        sats: []
      };
      baselines.push(cur);
      continue;
    }

    if (!cur) continue;   // 汇总行/星行出现在头行之前 → 丢弃

    // --- C) 逐星行（必须先于汇总行判断：它不含 bl=/KM，不会误判，但顺序清晰）---
    const s = raw.match(BLS_SAT_RE);
    if (s) {
      cur.sats.push({
        prn: s[1].trim(),             // 如 "G05"
        refPrn: s[2].trim(),          // 如 "G12"
        freq: Number(s[3]),           // 0=L1 1=L2 2=L5
        stat: Number(s[4]),           // ssat.vsat 有效性标志（0 已在引擎侧剔除）
        ageS: Number(s[5]),
        atmo: Number(s[6]),           // 双差大气延迟合计 (m)
        trpd: Number(s[7]),           // 双差对流层延迟 (m)
        iond: Number(s[8]),           // 双差电离层延迟 (m)
        el: Number(s[9])              // 高度角 (°)
      });
      continue;
    }

    // --- B) 汇总行 ---
    const m = raw.match(BLS_SUM_RE);
    if (m) {
      const stat = m[1];
      const lv = solLevel(stat);
      cur.stat = stat;
      cur.level = lv.level;
      cur.label = lv.label;
      cur.baselineKm = Number(m[2]);
      cur.ageS = Number(m[3]);
      cur.nb = Number(m[4]);
      cur.ns = Number(m[5]);
      cur.poserr = { x: Number(m[6]), y: Number(m[7]), z: Number(m[8]) };
      /* poserr 三分量的模，前端排序/告警用（省得每处都手算） */
      cur.poserrNorm = Math.hypot(cur.poserr.x, cur.poserr.y, cur.poserr.z);
      continue;
    }

    warnings.push(`$BLSOLS 无法识别的行: ${raw.trim().slice(0, 90)}`);
  }

  /* 只有头行、没等到汇总行的残缺记录：丢弃并告警，
   * 避免前端拿到一堆 null 字段误判为「有解算但数值为空」。 */
  const complete = baselines.filter((b) => b.stat !== null);
  if (complete.length !== baselines.length) {
    warnings.push(`${baselines.length - complete.length} 条基线缺少汇总行，已丢弃`);
  }

  return { baselines: complete, warnings };
}

/* ==================================================================
 * $NET[cors] —— 三角网
 *
 * 内层 CSV 与控制台 showtrinet 逐字节一致，故直接复用 bls/trinet 侧的
 * parseTrinet。这里只做一层薄封装，保持「一个入口消化四类帧」的一致性。
 * ================================================================== */
function parseNet(payload) {
  // 延迟 require 以免与 trinet-parser 形成循环依赖
  const { parseTrinet } = require('./trinet-parser');
  return parseTrinet(payload);
}

/* ==================================================================
 * $SRCSOLS —— NMEA
 *
 * outsols() 出 GGA/RMC，outsolexs() 出 GSA/GSV，都是标准语句，
 * 直接复用 nmea-parser 的单句解析器（它已处理 GLO/SBS/QZS 的 PRN 偏移）。
 * ================================================================== */
function parseSrcSols(payload) {
  const out = { gga: null, rmc: null, gsa: [], gsv: [] };
  for (const raw of String(payload || '').split(/\r?\n/)) {
    const line = raw.trim();
    if (!line || line[0] !== '$') continue;
    const p = parseLine(line);
    if (!p || p.error) continue;
    if (p.type === 'GGA') out.gga = p;
    else if (p.type === 'RMC') out.rmc = p;
    else if (p.type === 'GSA') out.gsa.push(p);
    else if (p.type === 'GSV') out.gsv.push(p);
  }
  return out;
}

module.exports = {
  SolutionFrameParser,
  parseSrcInfo,
  parseBlsols,
  parseNet,
  parseSrcSols,
  parseShowBls   // 兼容导出：mock 与旧测试仍在用
};
