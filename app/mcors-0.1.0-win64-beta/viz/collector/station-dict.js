/**
 * 站点字典
 *
 * monitor 协议本身只推源表里的 name（如 $RTK[A001->A002]），
 * 不携带站点元数据。经纬度/名称/行政区划必须从 conf 侧加载。
 *
 * 数据来源（已核实）：
 *  - conf/ntripsources
 *      格式: 名称,IP,端口,用户,密码,挂载点,纬度,经度,高程,M#|E#|O#
 *      末位 M=混合(有观测值) E=仅星历 O=仅观测值
 *  - conf/base-stations.info
 *      格式: 编号,中文名,英文名,行政区,纬度,经度,高程,年份,状态
 *  - conf/sourcetable
 *      STR;RTCM32;RTCM32_GG;RTCM3X;...;CHN;...
 */
const fs = require('fs');
const path = require('path');

/** 基站类型 → 展示名 */
const SRC_TYPE = {
  M: { label: '混合', desc: 'RTCM 含观测值' },
  E: { label: '仅星历', desc: 'RTCM 仅星历' },
  O: { label: '仅观测', desc: 'RTCM 仅观测值' },
  V: { label: 'VRS', desc: '虚拟参考站' }
};

/**
 * 归一化源类型，只认 SRC_TYPE 里的四个合法值。
 *
 * 为什么需要它：引擎 `sourceinfo` 表格的第 3 列是**连接类型**，
 * 实际输出 "NTRIP"（实测所有源都是这个词），而不是 addsource 命令
 * 里的 M/E/O/V。原实现只做 `replace(/[^A-Z]/g,'')` 过滤，
 * "NTRIP" 会被清成 "NTRIP"（全是大写字母，一个都留），
 * 查不到 SRC_TYPE 就静默回落到 M —— 界面上所有运行期新增的源
 * 都被错标成"混合"，而用户填的 type='O'/'E' 被完全忽略。
 *
 * 归一化后：非合法值一律回落到 M（默认混合），不伪造具体类型。
 */
function normalizeSrcType(raw) {
  const t = String(raw || '').trim().toUpperCase();
  return Object.prototype.hasOwnProperty.call(SRC_TYPE, t) ? t : 'M';
}

/**
 * 按行读取文本文件，自动处理编码。
 *
 * conf/base-stations.info 与 conf/ntripsources 的中文是 GBK 编码
 * （已核实：D0C2BD = "新"），直接按 utf8 读会乱码。
 * 这里优先按 GBK 解码；若结果含替换字符则回退 latin1，保证不崩溃。
 */
function readLinesSafe(file) {
  let buf;
  try {
    buf = fs.readFileSync(file);
  } catch (e) {
    return null;
  }

  let text;
  try {
    text = new TextDecoder('gbk').decode(buf);
  } catch (e) {
    text = buf.toString('utf8');
  }
  // 解码失败会产生 U+FFFD，出现则回退
  if (text.indexOf('�') >= 0) {
    text = buf.toString('utf8');
  }
  return text.split(/\r?\n/);
}

/**
 * 解析 conf/ntripsources
 * @returns {Map<string, object>} key = 站点名
 */
/**
 * 解析 conf/ntripsources
 *
 * 现有列（10 列，引擎侧见 ntrip.c 的 cors_read_ntrip_sources）：
 *   0:基站名 1:NTRIP地址 2:端口 3:用户名 4:密码 5:基站标识(挂载点)
 *   6:纬度 7:经度 8:高程 9:类型(M混合/E仅星历/O仅观测/V虚拟站)
 *
 * **可选扩展列**（第 10 列起，缺省不影响解析，兼容既有配置文件）：
 *   10:天线高(m)     —— 仪器高，RTCM 常规解算的必要参数
 *   11:天线型号       —— 如 "CHC i80" / "Trimble R8"
 *   12:天线序列号
 *   13:台站中文名     —— 用于大屏显示中文名
 *   14:建站年份
 *
 * 注意：引擎目前只读前 10 列（val[9] 之后未解析），扩展列仅采集层使用，
 * 因此在引擎侧消费这些字段之前，它们只影响大屏展示、不影响解算。
 */
function parseNtripSources(file) {
  const lines = readLinesSafe(file);
  const map = new Map();
  if (!lines) return map;

  for (const raw of lines) {
    const line = raw.trim();
    if (!line || line.startsWith('#')) continue;
    // 去掉行尾注释 (# 或 //)
    const body = line.split('#')[0].split('//')[0].trim();
    if (!body) continue;

    const f = body.split(',');
    if (f.length < 9) continue;

    const name = f[0].trim();
    if (!name) continue;

    const at = (i) => (f[i] !== undefined ? f[i].trim() : '');
    const num = (i) => {
      if (f[i] === undefined) return null;
      const v = Number(f[i]);
      return Number.isFinite(v) ? v : null;
    };

    const typeRaw = normalizeSrcType(f[9]);
    map.set(name, {
      name,
      addr: at(1),
      port: num(2),
      user: at(3),
      mntpnt: at(5),
      lat: num(6),
      lon: num(7),
      height: num(8),
      type: typeRaw,
      typeLabel: (SRC_TYPE[typeRaw] || SRC_TYPE.M).label,
      typeDesc: (SRC_TYPE[typeRaw] || SRC_TYPE.M).desc,
      /* ---- 可选扩展列（配置文件未提供时为 null）---- */
      antHgt: num(10),          // 天线高 (m)
      antType: at(11) || null,  // 天线型号
      antSerial: at(12) || null, // 天线序列号
      cnName: at(13) || null,   // 台站中文名
      builtYear: num(14),       // 建站年份
      // 运行期由引擎数据回填
      srcId: null,
      online: null,
      lastSeen: null,
      // 运行期由引擎 sourceinfo 回填（conf 里没有的精确信息）
      ecef: null,
      addrResolved: null,
      connState: null
    });
  }
  return map;
}

/**
 * 解析 conf/base-stations.info
 *
 * 已核实为 9 字段（无独立 city 列，行政区只在第 3 位）：
 *   0 编号, 1 中文名, 2 英文名, 3 行政区, 4 纬度, 5 经度, 6 高程, 7 年份, 8 状态
 * 例: 1001,新疆和田地区于田县于田公园,XinJiang,和田地区,36.85918698,81.67053647,1422.4313,2014,0
 *
 * 可选扩展列（第 9 位起，缺省为 null，兼容既有文件）：
 *   9 天线高(m), 10 天线型号, 11 天线序列号, 12 天线方位角(deg), 13 厂商, 14 接收机型号
 *
 * @returns {Map<string, object>} key = 编号
 */
function parseBaseStationsInfo(file) {
  const lines = readLinesSafe(file);
  const map = new Map();
  if (!lines) return map;

  for (const raw of lines) {
    const line = raw.trim();
    if (!line || line.startsWith('#')) continue;
    const f = line.split(',');
    if (f.length < 7) continue;

    const id = f[0].trim();
    if (!id) continue;

    // 行政区形如 "和田地区" / "新疆"，按需拆分省/市
    const region = f[3] ? f[3].trim() : '';
    let province = region;
    let city = '';
    const m = region.match(/^(.+?)(地区|市|州|盟|县)/);
    if (m) {
      province = m[1];
      city = m[2];
    }

    const at = (i) => (f[i] !== undefined ? f[i].trim() : '');
    const num = (i) => {
      if (f[i] === undefined) return null;
      const v = Number(f[i]);
      return Number.isFinite(v) ? v : null;
    };
    map.set(id, {
      id,
      cnName: at(1),
      enName: at(2),
      region,
      province,
      city,
      lat: num(4),
      lon: num(5),
      height: num(6),
      year: num(7),
      status: num(8) || 0,
      /* 可选扩展列：天线与设备信息 */
      antHgt: num(9),          // 天线高 (m)
      antType: at(10) || null, // 天线型号
      antSerial: at(11) || null,
      antAzimuth: num(12),     // 天线方位角 (deg)
      vendor: at(13) || null,  // 厂商
      recvModel: at(14) || null // 接收机型号
    });
  }
  return map;
}

/**
 * 解析 conf/sourcetable
 * @returns {Array<object>}
 */
function parseSourcetable(file) {
  const lines = readLinesSafe(file);
  const out = [];
  if (!lines) return out;

  for (const raw of lines) {
    const line = raw.trim();
    if (!line || line.startsWith('#')) continue;
    const f = line.split(';');
    if (f[0] !== 'STR' || f.length < 12) continue;
    out.push({
      name: f[1],
      alias: f[2],
      format: f[3],
      messages: f[4],
      freq: Number(f[5]) || null,
      system: f[6],
      network: f[7],
      country: f[8],
      lat: Number(f[9]),
      lon: Number(f[10]),
      height: Number(f[11])
    });
  }
  return out;
}

class StationDict {
  constructor(confDir) {
    this.confDir = confDir;
    this.sources = new Map(); // name -> 站点（来自 ntripsources，运行期真值）
    this.baseInfo = new Map(); // id -> 站点元信息
    this.sourcetable = [];
    this.loaded = false;
    this.errors = [];
  }

  load() {
    this.errors = [];
    this.sources = parseNtripSources(path.join(this.confDir, 'ntripsources'));
    this.baseInfo = parseBaseStationsInfo(path.join(this.confDir, 'base-stations.info'));
    this.sourcetable = parseSourcetable(path.join(this.confDir, 'sourcetable'));

    if (this.sources.size === 0) this.errors.push('ntripsources 解析为空或文件不存在');
    if (this.baseInfo.size === 0) this.errors.push('base-stations.info 解析为空或文件不存在');

    this.loaded = this.sources.size > 0 || this.baseInfo.size > 0;
    return this.loaded;
  }

  /**
   * 重新加载 conf 配置文件。
   *
   * 必须保留运行期记录（runtimeOnly）。
   *
   * 背景：前端在新增/删除基站后会 POST /api/dict/reload（见 main.js
   * 的 stPanel.onChanged）。而 load() 会把 this.sources 整个换成从
   * conf/ntripsources 解析出的新 Map，所有运行期 addsource 进来的源
   * （含用户在表单里填的 height / type / 经纬度）**全部丢失**。
   * 紧接着的 sourceinfo 轮询会用残缺数据重建它们：引擎只给 ECEF
   * 和连接类型 "NTRIP"，于是 height 变 null、type 归一为 M ——
   * 实测界面表格显示「— — 混合」，而用户明明填了「仅观测 95m」。
   *
   * 做法：先取出 runtimeOnly 记录，load() 之后按名字并回去。
   * conf 里的同名条目优先（人工台账是权威），运行期记录只补空缺。
   */
  reload() {
    const runtime = [];
    for (const s of this.sources.values()) {
      if (s.runtimeOnly) runtime.push(s);
    }
    const ok = this.load();
    for (const r of runtime) {
      if (this.sources.has(r.name)) continue;   // conf 里有同名，以 conf 为准
      this.sources.set(r.name, r);
    }
    return ok;
  }

  /** 全部站点列表（以 ntripsources 为主表） */
  list() {
    const out = [];
    for (const s of this.sources.values()) out.push(s);
    return out;
  }

  get(name) {
    return this.sources.get(name) || null;
  }

  /**
   * 按 NTRIP 挂载点名查站点。
   *
   * 引擎 sourceinfo 同时给出 Name 与 Mountpoint，多数站点两者相同，
   * 但也存在挂载点与站名不一致的部署；回填 srcId 时两条路都要试。
   */
  getByMountPoint(mntpnt) {
    if (!mntpnt) return null;
    for (const s of this.sources.values()) {
      if (s.mntpnt === mntpnt) return s;
    }
    return null;
  }

  /**
   * 登记一个**运行期**站点（引擎 addsource 命令新增、conf 文件里没有的）。
   *
   * 为什么需要它：站网图把三角网顶点映射到站点元信息时，依赖
   * `stationDict.get(name)` 命中。而字典只从 conf/ntripsources 读，
   * 运行期 addsource 加进来的源不在文件里 → 查不到 →
   *   1) 顶点名退化成纯 srcId（实测显示 "10" 而不是 NEWTEST01）
   *   2) 顶点不画出来（网图按名字找坐标，byName.get 返回 null 就跳过）
   *
   * sourceinfo 轮询已经能拿到这些源的完整信息（名字/地址/ECEF/连接状态），
   * 这里把它落到字典里，让两条链路一致。
   *
   * 只在源不存在时插入，**绝不覆盖** conf 里已有的记录 ——
   * conf 是人工维护的权威台账，运行期数据只是补充。
   */
  upsertRuntime(info) {
    if (!info || !info.name) return null;
    const exist = this.sources.get(info.name);
    if (exist) {
      /* 已存在时的补齐规则。
       *
       * 两条路径都会调 upsertRuntime，谁先到取决于时序：
       *   A) pollSourceInfo —— 每 3s 一次，用 sourceinfo 的数据。
       *      它给的是连接类型（"NTRIP"）和 ECEF，**没有高程**。
       *   B) /api/station/add —— 用户在表单里填的权威值
       *      （type 必为 M/E/O/V，height/lat/lon 必填）。
       *
       * 若 A 先建记录，B 后到，就必须用 B 补齐 A 缺的那些字段；
       * 否则用户在界面上填的类型/高程会被静默丢掉
       * （实测表格显示「— — 混合」，而用户明明选了「仅观测 95m」）。
       *
       * 补齐规则：
       *   - height/lat/lon：仅在现有值为 null 时补（引擎侧没有的才补）
       *   - type：A 存的是 normalizeSrcType("NTRIP") = 'M'，这是**回落值**
       *     而非用户选择。B 带来合法类型时必须覆盖。
       *     判断依据是 incoming 本身合法（M/E/O/V）——这已由
       *     /api/station/add 的 validateStation 保证。
       *     源信息侧再次调用时 incoming 是 "NTRIP"，归一后为 'M'，
       *     与现有值相同，覆盖也无副作用。
       */
      if ((exist.height === null || exist.height === undefined) &&
          Number.isFinite(info.height)) {
        exist.height = info.height;
      }
      if (exist.lat === null && Number.isFinite(info.lat)) exist.lat = info.lat;
      if (exist.lon === null && Number.isFinite(info.lon)) exist.lon = info.lon;

      const inType = String(info.type || '').trim().toUpperCase();
      if (Object.prototype.hasOwnProperty.call(SRC_TYPE, inType) && exist.type !== inType) {
        exist.type = inType;
        exist.typeLabel = SRC_TYPE[inType].label;
        exist.typeDesc = SRC_TYPE[inType].desc;
      }
      return exist;
    }

    const typeRaw = normalizeSrcType(info.type);
    const s = {
      name: info.name,
      addr: info.addr || '',
      port: Number(info.port) || null,
      user: info.user || '',
      mntpnt: info.mntpnt || info.name,
      // 引擎的 ECEF 反解出经纬度，供网图定位；conf 源用经纬度直读
      lat: Number.isFinite(info.lat) ? info.lat : null,
      lon: Number.isFinite(info.lon) ? info.lon : null,
      height: Number.isFinite(info.height) ? info.height : null,
      type: typeRaw,
      typeLabel: (SRC_TYPE[typeRaw] || SRC_TYPE.M).label,
      typeDesc: (SRC_TYPE[typeRaw] || SRC_TYPE.M).desc,
      ecef: info.ecef || null,
      srcId: Number.isFinite(info.srcId) ? info.srcId : null,
      connState: info.connState || null,
      online: true,
      lastSeen: Date.now(),
      /* 运行期新增：conf 文件里没有这条记录。UI 据此加标记区分，
         避免把临时加的源误当作正式台账条目。 */
      runtimeOnly: true
    };

    /* 引擎只给 ECEF 时反解经纬度，否则网图画不出这个点。
     * 用与 pntpos 相同的 WGS84 参数做简化换算：
     * lon = atan2(y,x)，lat 由 pz 与水平半径求地心角。 */
    if ((s.lat === null || s.lon === null) && info.ecef) {
      const p = String(info.ecef).split(',').map(Number);
      if (p.length >= 3 && p.every(Number.isFinite)) {
        const [x, y, z] = p;
        const a = 6378137.0, f = 1 / 298.257223563, b = a * (1 - f);
        const e2 = 1 - (b * b) / (a * a);
        const lon = Math.atan2(y, x);
        const pH = Math.hypot(x, y);
        let lat = Math.atan2(z, pH * (1 - e2));
        for (let i = 0; i < 5; i++) {   // 迭代收敛（Bowring 近似足够）
          const sin = Math.sin(lat);
          const N = a / Math.sqrt(1 - e2 * sin * sin);
          const h = pH / Math.cos(lat) - N;
          lat = Math.atan2(z, pH * (1 - e2 * (N / (N + h))));
        }
        s.lat = (lat * 180) / Math.PI;
        s.lon = (lon * 180) / Math.PI;
      }
    }

    this.sources.set(s.name, s);
    /* 登记时刻。用于 pruneRuntimeStations 的保护期：
     * /api/station/add 是「先登记字典、再下发 addsource 命令」，
     * 命令走控制台 TCP 再到引擎接受需要几百毫秒到数秒。这段时间里
     * 一次 sourceinfo 轮询就会跑完，而新源此时还不在引擎里，
     * 若无保护会被当成「已删除的残影」清掉，下一轮又用
     * sourceinfo 的残缺数据（无高程、类型回落 M）重建，
     * 用户填的值就丢了。 */
    s.registeredAt = Date.now();
    return s;
  }

  /**
   * 移除已不存在的运行期站点。
   *
   * 对称于 upsertRuntime：引擎 delsource 后该源已不存在，
   * 若不清理，站点表会留一个残影（且网图仍尝试去画它）。
   *
   * @param {Set<string>} aliveNames 本轮仍存在的运行期源名
   * @param {number} graceMs 保护期（毫秒）。处于保护期内的登记记录不删。
   * @returns {number} 移除条数
   *
   * 只删 runtimeOnly 的记录 —— conf/ntripsources 里的正式台账
   * 属人工维护的配置文件，引擎侧 delsource 不应把它抹掉。
   *
   * graceMs 默认为 0，即「不在存活集合里就删」，这是本方法的固有语义，
   * 单元测试与「用户明确删除后立即清干净」的场景都依赖它。
   *
   * 只有 sourceinfo 轮询那一个调用点需要传非 0 值：/api/station/add 是
   * 「先登记字典、再下发 addsource 命令」，命令走控制台 TCP 到引擎接受
   * 需要几百毫秒到数秒。这段时间里新源还不在引擎的 sourceinfo 中，
   * 一次轮询就会把它当成「已删除的残影」清掉，随后又被 sourceinfo 的
   * 残缺数据重建（引擎只给 ECEF 和连接类型 NTRIP）→ 用户在表单里填的
   * 高程变 null、类型回落成 M。实测表格显示「— — 混合」。
   */
  pruneRuntime(aliveNames, graceMs = 0) {
    const alive = aliveNames instanceof Set ? aliveNames : new Set(aliveNames || []);
    const now = Date.now();
    let removed = 0;
    for (const s of [...this.sources.values()]) {
      if (!s.runtimeOnly || alive.has(s.name)) continue;
      if (Number.isFinite(s.registeredAt) && now - s.registeredAt < graceMs) continue;
      this.sources.delete(s.name);
      removed++;
    }
    return removed;
  }

  /**
   * 补充展示名与行政区划。
   *
   * 注意：conf/ntripsources 与 conf/base-stations.info 是两套独立编号体系
   * （前者 A001 之类，后者 1001 之类），且实际收录的区域不同
   * （ntripsources 为北京周边，base-stations.info 多为新疆/西藏）。
   * 因此这里按地理距离全局择优匹配，并记录距离供前端判断可信度；
   * 超出容差时不强行匹配，避免张冠李戴。
   */
  enrichNames(toleranceDeg = 0.05) {
    const infos = [...this.baseInfo.values()].filter(
      (i) => Number.isFinite(i.lat) && Number.isFinite(i.lon)
    );
    let matched = 0;
    const unmatched = [];

    for (const s of this.sources.values()) {
      if (!Number.isFinite(s.lat) || !Number.isFinite(s.lon)) continue;

      let best = null;
      let bestD = Infinity;
      for (const info of infos) {
        // 经度差需按地球尺度归一，避免高纬度处横向距离被压缩
        const dLat = info.lat - s.lat;
        const dLon = (info.lon - s.lon) * Math.cos(((s.lat + info.lat) / 2) * Math.PI / 180);
        const d = Math.hypot(dLat, dLon);
        if (d < bestD) {
          bestD = d;
          best = info;
        }
      }

      // 距离换算为公里，仅用于判断可信度
      const km = bestD * 111;
      if (best && bestD < toleranceDeg) {
        s.cnName = best.cnName;
        s.enName = best.enName;
        s.region = best.region;
        s.province = best.province;
        s.city = best.city;
        s.year = best.year;
        s.baseId = best.id;
        s.nameMatchKm = Number(km.toFixed(2));
        matched++;
      } else if (best) {
        // 记录最近的候选，但不当作可信匹配
        s.nearestBaseId = best.id;
        s.nearestCnName = best.cnName;
        s.nameMatchKm = Number(km.toFixed(2));
        s.nameMatched = false;
        unmatched.push(s.name);
      }
    }
    this.nameMatchStats = { matched, unmatched: unmatched.length };
    return matched;
  }

  /** 基站网图需要的精简结构 */
  toGeoJSON() {
    return {
      type: 'FeatureCollection',
      features: this.list()
        .filter((s) => Number.isFinite(s.lat) && Number.isFinite(s.lon))
        .map((s) => ({
          type: 'Feature',
          geometry: { type: 'Point', coordinates: [s.lon, s.lat] },
          properties: {
            /* --- 标识 --- */
            name: s.name,                       // 基站名（conf 里的唯一键）
            srcId: s.srcId,                     // 引擎内部 ID（运行期回填）
            /* 运行期 addsource 新增、不在 conf/ntripsources 里的源。
             * 前端据此加标记，避免把临时加的源误当正式台账。 */
            runtimeOnly: !!s.runtimeOnly,
            cnName: s.cnName || s.nearestCnName || s.name,
            enName: s.enName || '',             // 英文名/台站 ID
            nameMatched: s.nameMatched !== false && !!s.cnName,
            nameMatchKm: s.nameMatchKm ?? null,
            /* --- 行政区划 --- */
            region: s.region || '',
            province: s.province || '',
            city: s.city || '',
            /* --- 坐标 --- */
            height: s.height,                   // 高程/大地水准高 (m)
            ecef: s.ecef || null,               // 引擎换算的精确 ECEF（mm 级）
            builtYear: s.builtYear ?? s.year ?? null,
            /* --- 天线（用户重点要求）---
             * 数据来自 conf 的可选扩展列；配置文件未提供时为 null。
             * 引擎目前不读这些列（ntrip.c 只解析到 val[9]），
             * 因此它们只影响展示、不影响解算。 */
            antHgt: s.antHgt ?? null,           // 天线高/仪器高 (m)
            antType: s.antType || null,         // 天线型号
            antSerial: s.antSerial || null,     // 天线序列号
            antAzimuth: s.antAzimuth ?? null,   // 天线方位角 (deg)
            /* --- 设备 --- */
            vendor: s.vendor || null,
            recvModel: s.recvModel || null,
            /* --- 类型与连接 --- */
            type: s.type,
            typeLabel: s.typeLabel,
            typeDesc: s.typeDesc || '',
            mntpnt: s.mntpnt,                   // NTRIP 挂载点
            addr: s.addrResolved || s.addr,    // 上游地址（优先引擎侧实际值）
            port: s.port,
            connState: s.connState || null,     // connected / disconnected
            online: s.online,
            lastSeen: s.lastSeen
          }
        }))
    };
  }

  stats() {
    const all = this.list();
    return {
      total: all.length,
      withGeo: all.filter((s) => Number.isFinite(s.lat) && Number.isFinite(s.lon)).length,
      mix: all.filter((s) => s.type === 'M').length,
      eph: all.filter((s) => s.type === 'E').length,
      baseInfoCount: this.baseInfo.size,
      sourcetableCount: this.sourcetable.length,
      nameMatch: this.nameMatchStats || null,
      loaded: this.loaded,
      errors: this.errors.slice()
    };
  }
}

module.exports = { StationDict, SRC_TYPE, parseNtripSources, parseBaseStationsInfo, parseSourcetable };