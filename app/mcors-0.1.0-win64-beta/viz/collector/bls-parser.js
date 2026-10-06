/**
 * 基线解算 / 控制台命令输出解析
 *
 * 控制台命令的输出都是文本，格式由 engine.c 各 cmd_* 函数用 printf 拼装。
 * 这里按源码中确认的格式做解析，格式变化时只需改对应函数。
 */

/** 解状态字符串表（engine.c cmd_showbls / cmd_showblsols） */
const SOL_STR = ['NONE', 'FIX', 'FLOAT', 'SBAS', 'DGPS', 'SINGLE', 'PPP', 'DR', 'FIXDEG'];

const SOL_LEVEL = {
  FIX: { level: 'fix', label: '固定解' },
  FIXDEG: { level: 'fixdeg', label: '固定解(降级)' },
  FLOAT: { level: 'float', label: '浮点解' },
  PPP: { level: 'ppp', label: 'PPP' },
  DGPS: { level: 'dgps', label: '差分' },
  SBAS: { level: 'sbas', label: 'SBAS' },
  SINGLE: { level: 'single', label: '单点' },
  DR: { level: 'dr', label: '航位推算' },
  NONE: { level: 'none', label: '无解' }
};

/** 基线解状态 → 语义分类 */
function solLevel(stat) {
  const m = SOL_LEVEL[stat] || { level: 'none', label: stat || '未知' };
  return { stat, level: m.level, label: m.label };
}

/**
 * 解析 showbls 输出（cmd_showbls）
 *
 * 格式（engine.c）：
 *   2026/10/03 08:30:00:   1->  2 stat=   FIX bl=  123 KM age=  1s  nb= 28
 *
 * 注意 stat 字段用 %6s 右对齐填充，解析时需 trim。
 */
function parseShowBls(text) {
  const out = [];
  if (!text) return out;

  /* 双差量行的解析。
   *
   * 引擎（engine.c cmd_showbls）对每条基线打：
   *   1) 主行    "…: base->rover stat=… bl=… KM age=… nb=…"   ← 无条件输出
   *   2) 汇总行  "      dd ref=N n=M med=… spread=… resrms=…"   ← 仅当 M>0
   *   3) 方程行×M "        eq K sat=SS f=F amb=… ambi=… ambt=… res=…"
   *
   * 注意 2/3 只在**有有效双差方程时**才输出：引擎遇到 nf==0 时
   * 什么 dd 行都不打（旧版会补一行 n=0 全零行，现已省略）。
   * 因此解析器**不能假设主行后面必然跟一行汇总行** ——
   * 缺汇总行时，本函数只产出主行字段，ddValid 保持假值，
   * 前端据此显示「—」。这是正常路径，不是异常。
   *
   * 第 3 类是**逐双差方程**的（M 个方程，M 取决于参与解算的卫星数）。
   * 模糊度是逐方程的量，不同方程相差整周 —— 这正是模糊度定义，
   * 所以既不能只报一个值（看不出周跳是否被剔除），也不该折叠成中位数
   * （丢了物理意义）。故完整保留 M 个方程的数组。
   *
   * 这些行缩进且无时间戳，靠**行序**与主行配对（引擎按同序紧跟输出），
   * 用 last（最近一条主行）作为挂载目标。
   * 单测时用「连续两条基线各自带自己的方程」验证不串。 */
  const last = { entry: null };

  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line) continue;

    // --- 逐双差方程行（必须先于 dd 行判断，它也含 "dd" 无关但行首是 eq）---
    if (/^eq\s+\d+\s+sat=/.test(line)) {
      const e = parseEqLine(line);
      /* 挂到当前基线的 ddEqs 上（parseDdLine 已把它初始化为 []） */
      if (e && last.entry && last.entry.ddEqs) last.entry.ddEqs.push(e);
      continue;
    }
    // --- 双差汇总行 ---
    if (line.indexOf('dd ref=') >= 0) {
      const d = parseDdLine(line);
      if (d && last.entry) {
        Object.assign(last.entry, d);
        /* parseDdLine 返回的 ddEqs 已是新数组，Object.assign 已替换，
         * 后续 eq 行直接 push 进去即可 */
      }
      continue;
    }

    // 时间: base->rover stat=XXX bl=N KM age=N s[ nb=N]
    const m = line.match(
      /^(.+?):\s*(-?\d+)\s*->\s*(-?\d+)\s+stat=\s*(\S+)\s+bl=\s*(-?[\d.]+)\s*KM\s+age=\s*(-?[\d.]+)s(?:\s+nb=\s*(\d+))?/
    );
    if (!m) continue;

    const stat = m[4];
    const e = {
      time: m[1].trim(),
      baseSrcId: Number(m[2]),
      roverSrcId: Number(m[3]),
      stat,
      ...solLevel(stat),
      baselineKm: Number(m[5]),
      ageS: Number(m[6]),
      nb: m[7] !== undefined ? Number(m[7]) : null
    };
    out.push(e);
    last.entry = e;
  }
  return out;
}

/* 把 "k=v k=v" 形式的键值对串拆成对象（值为数字）。
 * 引擎所有 dd/atm 输出都是这个形式，且字段顺序固定，
 * 用键名取值比按位置更耐后续增删字段。 */
function kvNum(line) {
  const o = {};
  const re = /(\w+)=\s*(-?[\d.]+)/g;
  let m;
  while ((m = re.exec(line)) !== null) {
    const v = Number(m[2]);
    o[m[1]] = Number.isFinite(v) ? v : null;
  }
  return o;
}

/**
 * 解析双差模糊度行。
 * 格式: "dd ref= 2 n= 8 NL=  12.345 dd=   123.456 spread=  0.021 bias= …"
 *
 * 语义（见 engine.c / cors.h 的说明）：
 *   ref   参考星号（1 起，0 = 未定 → 整组视为无效）
 *   n     参与双差统计的卫星数（双差量是否可信的依据）
 *   NL    双差 NL 模糊度 (cycle)
 *   dd    双差模糊度中位数 (m)
 *   spread 双差模糊度离散度 max-min (m)，反映各星解的一致性
 */
function parseDdLine(line) {
  const k = kvNum(line);
  /* n 现在是**双差方程数**（此前是「参与卫星数」）。
   * 一条基线上的双差方程有 N 个：rtkpos.c 逐系统挑一颗仰角最高的
   * 作参考星，其余同系统星各与它构成一个方程。 */
  const valid = k.ref > 0 && k.n > 0;
  return {
    ddRef: k.ref || 0,
    ddNsat: k.n || 0,        /* 实为方程数，字段名沿用以兼容旧调用方 */
    ddMed: valid ? k.med : null,
    ddSpread: valid ? k.spread : null,
    ddResRms: valid ? k.resrms : null,
    ddEqs: [],               /* 逐方程明细，由 parseEqLine 填充 */
    ddValid: valid
  };
}

/**
 * 解析逐双差方程行。
 * 格式: "eq  1 sat=  5 f=0 amb=  245.3341 ambi= 0.0115 ambt= 0.0008 res= 0.0021"
 *
 * 逐方程输出而非只报一个值的原因：模糊度是**逐方程**的量，
 * 不同方程相差整周（这正是模糊度的定义）。正常情况下各方程的模糊度
 * 会聚成少数几个簇（每簇差整周），簇数与散布是判断周跳是否被
 * 剔除的直接依据 —— 折叠成中位数就看不出这些。
 */
function parseEqLine(line) {
  const k = kvNum(line);
  const m = line.match(/^eq\s+(\d+)/);
  return {
    no: m ? Number(m[1]) : null,
    sat: k.sat !== undefined ? k.sat : null,
    f: k.f !== undefined ? k.f : null,
    amb: k.amb !== undefined ? k.amb : null,
    ambi: k.ambi !== undefined ? k.ambi : null,
    ambt: k.ambt !== undefined ? k.ambt : null,
    res: k.res !== undefined ? k.res : null
  };
}

/**
 * 解析大气延迟行。
 * 格式: "atm ref= 2 n= 8 atmc= … ionc= … trpc= … ionm= … trpm= …"
 * 单位 m。atmc 是电离层+对流层合计，ionc/trpc 是分量。
 */
function parseAtmLine(line) {
  const k = kvNum(line);
  const valid = k.ref > 0 && k.n > 0;
  return {
    atmRef: k.ref || 0,
    atmc: valid ? k.atmc : null,
    ionCarrier: valid ? k.ionc : null,
    tropCarrier: valid ? k.trpc : null,
    ionModel: valid ? k.ionm : null,
    tropModel: valid ? k.trpm : null,
    atmValid: valid
  };
}

/**
 * 解析 showblsols 输出（cmd_showblsols）
 * 格式同 showbls 但无 nb 字段
 */
function parseShowBlsols(text) {
  return parseShowBls(text);
}

/**
 * 解析 showdtrigs 输出（cmd_showdtrigs）
 *   "  1: dtrig00000012"
 */
function parseShowDtrigs(text) {
  const out = [];
  if (!text) return out;
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line) continue;
    const m = line.match(/^(\d+):\s*(\S+)$/);
    if (!m) continue;
    out.push({ index: Number(m[1]), id: m[2] });
  }
  return out;
}

/**
 * 解析 sourceinfo / prsourceinfo 输出（cmd_sourceinfo）
 *
 * 实测（引擎 2026-10，sourceinfo all）的输出是**定宽表格**，不是 key=value：
 *
 *   Name       ID  Type  IP   Port  User  Password  Mountpoint  Status  ECEF-XYZ
 *   KURJ00AUS0   1  NTRIP ...  2101 SuJingLan ...      KURJ00AUS0 connected -4638...,2606...,-3505...
 *
 * 其中第 2 列就是引擎内部的 srcId —— 这是把「站名」关联到「srcId」的唯一
 * 权威来源（monitor 帧头只有站名，showbls/showtrinet 只有数字 ID）。
 * 旧实现只认 key=value 形态，实测输出里一个 key=value 都没有，
 * 于是 entries 恒为空，站名↔srcId 映射永远建立不起来，
 * 站网图节点名只能退化成纯数字。
 *
 * 因此这里两种形态都支持：先按 key=value 解析，失败再按表格列位解析。
 */
function parseSourceInfo(text) {
  if (!text) return { raw: [], entries: [] };
  const raw = text.split(/\r?\n/).map((l) => l.trim()).filter(Boolean);

  const entries = [];
  for (const line of raw) {
    // 形态 1：key=value
    const kv = {};
    const re = /(\w+)=([^\s,]+)/g;
    let m;
    while ((m = re.exec(line)) !== null) kv[m[1]] = m[2];
    if (Object.keys(kv).length) { entries.push({ line, kv, ...tableOrKV(kv) }); continue; }

    // 形态 2：定宽表格。表头给出列序，数据行按列位取值。
    const c = line.split(/\s+/);
    if (c.length < 2) continue;
    const name = c[0];
    const id = Number(c[1]);
    if (!name || !Number.isFinite(id)) continue;
    entries.push({
      line,
      kv: {},
      name,
      srcId: id,
      type: c[2] || null,
      addr: c[3] || null,
      port: Number(c[4]) || null,
      mntpnt: c[7] || null,
      status: c[8] || null,
      ecef: c[9] || null
    });
  }
  return { raw, entries };
}

/** 从 key=value 字典里归一化出 name/srcId（表格形态走另一条路） */
function tableOrKV(kv) {
  return {
    name: kv.name || kv.src || null,
    srcId: kv.id !== undefined ? Number(kv.id) : null,
    type: kv.type || null,
    addr: kv.addr || null,
    port: kv.port !== undefined ? Number(kv.port) : null,
    mntpnt: kv.mntpnt || null,
    status: kv.status || null,
    ecef: kv.pos || null
  };
}

/**
 * 解析卫星数据（cmd_satellite）
 * 格式不固定，返回原文行供前端展示
 */
function parseSatellite(text) {
  if (!text) return [];
  return text.split(/\r?\n/).map((l) => l.trim()).filter(Boolean);
}

/**
 * 解析 sourceid 映射：从 showbls 的 srcid 反查站点名
 * 引擎内 srcid 由 cors_generate_source_id() 生成，与 conf 里的名字不是同一套，
 * 因此建立 srcid->name 的运行时映射表，靠 monitor 帧里的 $RTK[A->B] 头补全。
 */
class SrcIdMap {
  constructor() {
    this.map = new Map(); // srcid -> name
    this.nameToId = new Map(); // name -> srcid
  }

  /**
   * 从 monitor 帧头学习映射：$RTK[A001->A002]。
   *
   * 注意：monitor 帧对象只带站名（frame.base / frame.rover），
   * **不带 srcId** —— 引擎的帧头就是纯文本站名，srcId 只出现在控制台
   * showbls/showtrinet 的数字里。所以这里改为按站名反查 srcId：
   * 先用已有的 nameToId 查，查不到再交给调用方提供的 nameToIdFn
   * （通常来自 sourceinfo 之类的「站名 -> srcid」权威表）。
   *
   * 旧实现要求 frame.baseSrcId 是 number，而该字段根本不存在，
   * 于是 learn() 永远不执行，srcid↔站名映射建立不起来，
   * 站网图节点名退化成纯数字。
   */
  learn(frame, nameToIdFn) {
    if (!frame) return;
    const pick = (name, id) => {
      if (!name || name === 'NONE') return;   // NONE 是引擎的"无基站"占位，不是站名
      if (typeof id === 'number' && Number.isFinite(id)) { this.set(id, name); return; }
      if (typeof nameToIdFn === 'function') {
        const n = nameToIdFn(name);
        if (n !== null && n !== undefined && Number.isFinite(Number(n))) {
          this.set(Number(n), name);
        }
      }
    };
    pick(frame.base, frame.baseSrcId);
    pick(frame.rover, frame.roverSrcId);
  }

  set(id, name) {
    if (id === null || id === undefined || !name) return;
    this.map.set(Number(id), name);
    this.nameToId.set(name, Number(id));
  }

  /**
   * 只补缺：该 srcid 已有名字时不覆盖。
   *
   * 为什么必须存在：同一物理站可能双挂载（实测 MGRV00AUS0 id=4 与
   * BCEP00BKG0 id=5 的 staSrcId 都是 4），引擎 $BLSOLS 报头每历元按
   * 「实际用的流」写名字 —— 同一 srcid 的名字在两个挂载点名之间翻。
   * 若这里无条件覆盖，$SRCINFO（权威、行序固定）建立的名字会被报头
   * 每帧冲掉，前端显示名随之翻动。$SRCINFO 用 set()（权威可覆盖），
   * $BLSOLS 用本方法（只补缺），映射即稳定。
   */
  setIfAbsent(id, name) {
    if (id === null || id === undefined || !name) return;
    const k = Number(id);
    const cur = this.map.get(k);
    if (cur) return;
    this.set(k, name);
  }

  /** 支持按名字或数字 srcid 查询 */
  resolve(key) {
    if (key === null || key === undefined) return null;
    const s = String(key).trim();
    if (this.map.has(Number(s))) return this.map.get(Number(s));
    if (this.nameToId.has(s)) return s;
    return null;
  }

  toJSON() {
    return Object.fromEntries(this.map);
  }
}

module.exports = {
  SOL_STR,
  SOL_LEVEL,
  solLevel,
  parseShowBls,
  parseShowBlsols,
  parseDdLine,
  parseEqLine,
  parseAtmLine,
  parseShowDtrigs,
  parseSourceInfo,
  parseSatellite,
  SrcIdMap
};