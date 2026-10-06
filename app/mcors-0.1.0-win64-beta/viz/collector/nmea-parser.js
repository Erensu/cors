/**
 * NMEA-0183 解析器（适配 mcors / RTKLIB 输出）
 *
 * mcors 的 monitor 协议推的是标准 NMEA，但有两个必须处理的差异：
 *
 * 1. PRN 编码偏移（solution.c outnmea_gsa/outnmea_gsv）
 *      GLO (GL): PRN + 64   → 还原为 1..32
 *      SBS (SB): PRN - 87   → 还原为 33..64
 *      QZS (QZ): PRN - 192  → 还原为 1..10
 *    不还原会导致卫星天空图的星位和星座归属全错。
 *
 * 2. GSA 的 systemId 与 talker 组合使用
 *      多系统时 talker 固定为 GN，此时必须靠 systemId 区分星座。
 *
 * quality 字段按 mcors 实际映射处理（solution.c nmea_solq[]）：
 *      0=NONE 1=SINGLE 2=DGPS 3=PPP 4=FIX 5=FLOAT 6=DR
 *    注意与 NMEA 标准不同：mcors 把 SOLQ_PPP 映射为 3、SOLQ_FIX 映射为 4。
 */

/** talker → 星座标识 */
const TALKER_SYS = {
  GP: 'GPS',
  GL: 'GLO',
  GA: 'GAL',
  GB: 'BDS',
  GQ: 'QZS',
  GI: 'IRN',
  GN: 'MIX', // 多系统混合，需靠 systemId 细分
  SB: 'SBS'
};

/** GSA/GSV 的 systemId → 星座（solution.c nmea_sid[] 顺序） */
const SID_SYS = { 1: 'GPS', 2: 'GLO', 3: 'GAL', 4: 'BDS', 5: 'QZS', 6: 'IRN' };

/** 解状态 → 中文描述 + 语义分类（用于配色） */
const QUALITY_MAP = {
  0: { label: '无解', level: 'none' },
  1: { label: '单点', level: 'single' },
  2: { label: '差分', level: 'dgps' },
  3: { label: 'PPP', level: 'ppp' },
  4: { label: '固定解', level: 'fix' },
  5: { label: '浮点解', level: 'float' },
  6: { label: '航位推算', level: 'dr' }
};

/** 校验和：$ 与 * 之间所有字符异或 */
function checksum(body) {
  let sum = 0;
  for (let i = 0; i < body.length; i++) sum ^= body.charCodeAt(i);
  return sum & 0xff;
}

/**
 * 按 talker + systemId 还原星座与真实 PRN
 * @param {string} talker  两字母 talker，如 GP/GL/GN
 * @param {number} prn     报文中出现的 PRN
 * @param {number} sid     GSA 的 systemId（GSV 无此字段，传 0）
 */
function resolveSat(talker, prn, sid) {
  let sys = TALKER_SYS[talker] || null;

  // GN 是混合标识，优先用 systemId 细分
  if (sys === 'MIX') sys = SID_SYS[sid] || 'GPS';

  // 反向还原 PRN 偏移。
  // 编码规则见 solution.c outnmea_gsa/outnmea_gsv：
  //   GLO: prn += 64   → 还原 prn -= 64
  //   SBS: prn -= 87   → 还原 prn += 87
  //   QZS: prn -= 192  → 还原 prn += 192
  // 注意 QZS 编码后可能为负数（真实 PRN 1~10 → 报文 -191~-182），
  // NMEA 用无符号两位格式书写时会表现为 93~118 或 193~202 等区间，
  // 因此这里按模 200 归一化后再还原。
  let realPrn = prn;
  if (sys === 'GLO') {
    realPrn = prn - 64;
  } else if (sys === 'SBS') {
    realPrn = prn + 87;
  } else if (sys === 'QZS') {
    // 报文值可能是 0-99（真实1-10 编码后取模）或 193-202（等价表示）
    realPrn = prn < 100 ? prn + 192 : prn - 192;
    if (realPrn < 1 || realPrn > 10) realPrn = prn < 100 ? prn + 92 : prn - 192;
  }

  if (realPrn < 1) realPrn = prn; // 兜底：偏移不适用时保持原值
  return { sys, prn: realPrn, label: sys + String(realPrn).padStart(2, '0') };
}

/** "123.456" → 123.456；空串 → null */
function num(v) {
  if (v === undefined || v === null || v === '') return null;
  const n = Number(v);
  return Number.isFinite(n) ? n : null;
}

/** ddmm.mmmm + 'N' → 十进制度（值与半球合并在同一字段时用，如 RMC） */
function lat(v) {
  if (!v) return null;
  const hemi = v.slice(-1);
  const val = Number(v.slice(0, -1));
  if (!Number.isFinite(val)) return null;
  const deg = Math.floor(val / 100);
  const min = val - deg * 100;
  const d = deg + min / 60;
  return hemi === 'S' ? -d : d;
}

/** dddmm.mmmm + 'E' → 十进制度（值与半球合并在同一字段时用，如 RMC） */
function lon(v) {
  if (!v) return null;
  const hemi = v.slice(-1);
  const val = Number(v.slice(0, -1));
  if (!Number.isFinite(val)) return null;
  const deg = Math.floor(val / 100);
  const min = val - deg * 100;
  const d = deg + min / 60;
  return hemi === 'W' ? -d : d;
}

/**
 * ddmm.mmmm + 独立半球字段 → 十进制度（GGA 用）。
 *
 * GGA 的纬度/经度是**两个独立字段**：
 *   $GNGGA,102145.00,3353.1698257,S,15042.1464778,E,...
 *                    f[2]        f[3]  f[4]          f[5]
 * 而 RMC 是合并在一个字段里（f[3]=ddmm.mmmmS）。
 *
 * 此前 parseGGA 直接调 lat(f[2])，把「半球」当成 lat 字段的最后一个字符，
 * 于是 slice(-1) 取到的是小数末位（例如 "3353.1698257" 的 "7"），
 * 永远判不成 'S' → **南纬被当成北纬**。实测悉尼（南纬 33.88）
 * 被解析成 +33.886，流动站位置画到地球另一侧。
 */
function latHemi(val, hemi) {
  if (!val) return null;
  const v = Number(val);
  if (!Number.isFinite(v)) return null;
  const deg = Math.floor(v / 100);
  const min = v - deg * 100;
  const d = deg + min / 60;
  return String(hemi || '').toUpperCase() === 'S' ? -d : d;
}

/** dddmm.mmmm + 独立半球字段 → 十进制度（GGA 用） */
function lonHemi(val, hemi) {
  if (!val) return null;
  const v = Number(val);
  if (!Number.isFinite(v)) return null;
  const deg = Math.floor(v / 100);
  const min = v - deg * 100;
  const d = deg + min / 60;
  return String(hemi || '').toUpperCase() === 'W' ? -d : d;
}

/**
 * 解析单条 NMEA 语句
 * @returns {object|null} 解析结果；校验和不符时返回 { error: 'checksum' }
 */
function parseLine(line) {
  const s = line.trim();
  if (s.length < 8 || s[0] !== '$') return null;
  if (!s.endsWith('\r') && !s.endsWith('\n') && s[s.length - 1] !== '*') {
    // 允许缺尾，但必须有校验位
  }
  const star = s.lastIndexOf('*');
  if (star < 0) return null;
  const body = s.slice(1, star);
  const csField = s.slice(star + 1, s.length).trim();
  const cs = parseInt(csField.slice(0, 2), 16);
  if (Number.isFinite(cs) && checksum(body) !== cs) return { error: 'checksum', body };

  const fields = body.split(',');
  const talker = fields[0].slice(0, 2);
  const type = fields[0].slice(2);

  switch (type) {
    case 'RMC':
      return parseRMC(talker, fields);
    case 'GGA':
      return parseGGA(talker, fields);
    case 'GSA':
      return parseGSA(talker, fields);
    case 'GSV':
      return parseGSV(talker, fields);
    default:
      return null;
  }
}

function parseRMC(talker, f) {
  const quality = f[2] === 'A' ? 'ok' : 'void';
  return {
    type: 'RMC',
    talker,
    utc: f[1] || null,
    status: quality,
    lat: lat(f[3]),
    lon: lon(f[5]),
    speedKn: num(f[7]),
    courseDeg: num(f[8]),
    date: f[9] || null,
    mode: f[13] || null // A=自主 D=差分 R=RTK P=PPP
  };
}

function parseGGA(talker, f) {
  const q = num(f[6]);
  const meta = QUALITY_MAP[q] || { label: '未知', level: 'none' };
  return {
    type: 'GGA',
    talker,
    utc: f[1] || null,
    // GGA 的 lat/hemi 是两个独立字段（f[2]/f[3]、f[4]/f[5]），
    // 不能用 lat(f[2]) —— 那会把半球当成数值末位，导致南北纬符号丢失
    lat: latHemi(f[2], f[3]),
    lon: lonHemi(f[4], f[5]),
    quality: q,
    qualityLabel: meta.label,
    level: meta.level,
    numSat: num(f[7]),
    hdop: num(f[8]),
    alt: num(f[9]),
    geoidSep: num(f[11]),
    age: num(f[13]), // 差分龄期
    stationId: f[14] || null
  };
}

function parseGSA(talker, f) {
  const sid = num(f[18]);
  const prns = [];
  // GGA/GSA 字段布局（solution.c outnmea_gsa 的 sprintf 顺序）：
  //   $GN GSA,A,<mode>,<prn1>..<prn12>,<pdop>,<hdop>,<vdop>,<systemId>
  // PRN 槽位固定为 12 个（f[3]..f[14]），f[15] 起为 DOP。
  for (let i = 3; i <= 14; i++) {
    const p = num(f[i]);
    if (p === null) continue;
    // NMEA PRN 范围 1~999：QZSS 为 193~202（还原前需 +192 偏移），
    // 因此上限不能按 99 收，否则会误丢 QZSS 卫星。
    if (!Number.isInteger(p) || p < 1 || p > 999) continue;
    prns.push(resolveSat(talker, p, sid).label);
  }
  return {
    type: 'GSA',
    talker,
    sid,
    sys: SID_SYS[sid] || TALKER_SYS[talker] || 'GPS',
    used: prns,
    pdop: num(f[15]),
    hdop: num(f[16]),
    vdop: num(f[17])
  };
}

function parseGSV(talker, f) {
  const total = num(f[1]);
  const msgNo = num(f[2]);
  const visible = num(f[3]);
  const sats = [];
  // 每颗星 4 个字段：PRN, 仰角, 方位角, SNR
  // f[3] 之后每 4 个一组，不足 4 个的是填充位（,,,），需跳过。
  // 填充位常写作 99,00,000,00 这种占位值，仰角为 0，不应计入可见星。
  for (let i = 4; i + 3 < f.length; i += 4) {
    const p = num(f[i]);
    if (p === null || p <= 0) continue;
    const el = num(f[i + 1]);
    const az = num(f[i + 2]);
    const snr = num(f[i + 3]);
    // 仰角为 0 或无值说明是填充位（真实可见星仰角必 > 0）
    if (el === null || el <= 0) continue;
    sats.push({
      ...resolveSat(talker, p, 0),
      el,
      az: az === null ? null : az,
      snr: snr === null ? null : snr
    });
  }
  return { type: 'GSV', talker, total, msgNo, visible, sats, sys: TALKER_SYS[talker] };
}

/**
 * 解析 mcors monitor 协议的数据帧
 *
 * 帧结构（monitorsrc.c monitor_src_str）：
 *     \r\n<<\r\n
 *     $RTK[base->rover]\r\n     或 $PNT[...]
 *     <NMEA 报文...>
 *     >>\r\n
 *
 * TCP 是字节流，帧边界需自行处理：按 << 分帧、>> 收帧。
 */
class FrameParser {
  constructor() {
    this.buf = '';
  }

  /** 喂入任意字节，返回解析完成的帧数组 */
  push(chunk) {
    this.buf += chunk;
    const frames = [];

    let guard = 0;
    while (guard++ < 1000) {
      const s = this.buf.indexOf('<<');
      if (s < 0) {
        if (this.buf.length > 65536) this.buf = ''; // 异常保护
        break;
      }
      const e = this.buf.indexOf('>>', s + 2);
      if (e < 0) {
        this.buf = this.buf.slice(s); // 保留未完成帧
        break;
      }
      const body = this.buf.slice(s + 2, e);
      this.buf = this.buf.slice(e + 2);
      const f = this.parseFrame(body);
      if (f) frames.push(f);
    }
    return frames;
  }

  parseFrame(body) {
    const lines = body.split(/\r?\n/).map((x) => x.trim()).filter(Boolean);
    if (!lines.length) return null;

    let type = null;
    let base = null;
    let rover = null;

    const nmea = [];
    // $NET 帧承载的是 CSS 文本（三角站网快照），不是 NMEA，单独收集
    const csvLines = [];

    for (const line of lines) {
      // 首行形如 $RTK[A001->A002] / $PNT[...] / $NET[cors]
      const m = line.match(/^\$([A-Z]{3})\[(.*?)\]$/);
      if (m) {
        if (m[1] === 'NET') {
          type = 'NET';
          continue;
        }
        type = m[1] === 'RTK' ? 'RTK' : 'PNT';
        const [b, r] = m[2].split('->');
        base = (b || '').trim() || null;
        rover = (r || '').trim() || null;
        continue;
      }
      // 站网 CSV 行原样保留，交给 trinet-parser 解析
      if (type === 'NET') {
        csvLines.push(line);
        continue;
      }
      const p = parseLine(line);
      if (p && !p.error) nmea.push(p);
    }
    if (!type) return null;
    if (type === 'NET') return { type: 'NET', csv: csvLines.join('\n') };
    return { type, base, rover, nmea };
  }
}

module.exports = { parseLine, FrameParser, resolveSat, QUALITY_MAP, TALKER_SYS, SID_SYS };