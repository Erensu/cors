/**
 * 模拟数据源（MCORS_SOURCE=mock 时启用）
 *
 * 目的：引擎未运行、未编译成功、或 conf 不足以联调时，仍能把大屏界面
 * 和交互流程完整跑通。数据形态严格对齐真实协议：
 *   - 基线解算：复刻 engine.c cmd_showbls 的文本格式，再走同一套解析器
 *   - 卫星/定位：复刻 solution.c 的 NMEA 报文，再走同一套解析器
 * 这样一旦切到 live，只需改配置，不动前端与解析逻辑。
 */

/** 生成一条 NMEA 校验和 */
function nmea(body) {
  let sum = 0;
  for (let i = 0; i < body.length; i++) sum ^= body.charCodeAt(i);
  return `$${body}*${sum.toString(16).toUpperCase().padStart(2, '0')}\r\n`;
}

const pad = (n, w = 2) => String(n).padStart(w, '0');
const deg2dms = (deg) => {
  const d = Math.floor(deg);
  const m = (deg - d) * 60;
  return `${pad(d, 3)}${m.toFixed(4).padStart(7, '0')}`;
};

function mockGga(t, pos, quality, nsat, hdop) {
  const utc = `${pad(t.getUTCHours())}${pad(t.getUTCMinutes())}${pad(t.getUTCSeconds())}.${pad(t.getUTCMilliseconds(), 3)}`;
  const [lat, lon, alt] = pos;
  return nmea(
    `GN GGA`.replace(' ', '') +
      `,${utc},${deg2dms(lat)},N,${deg2dms(lon)},E,${quality},${pad(nsat, 2)},${hdop.toFixed(1)},${alt.toFixed(3)},M,${(alt - 30).toFixed(3)},M,1.0,0000`
  );
}

function mockRmc(t, pos, speed, course) {
  const utc = `${pad(t.getUTCHours())}${pad(t.getUTCMinutes())}${pad(t.getUTCSeconds())}.${pad(t.getUTCMilliseconds(), 3)}`;
  const [lat, lon] = pos;
  const y = t.getUTCFullYear();
  const mo = pad(t.getUTCMonth() + 1);
  const dd = pad(t.getUTCDate());
  return nmea(
    `GNRMC,${utc},A,${deg2dms(lat)},N,${deg2dms(lon)},E,${(speed / 1.94384).toFixed(2)},${course.toFixed(2)},${dd}${mo}${y},,${speed > 0.1 ? 'R' : 'A'},`
  );
}

/**
 * 生成 GSA 报文
 *
 * mcors 的 outnmea_gsa 会对每个星座各输出一条 GSA，
 * 多星座时 talker 用 GN 并携带对应 systemId。
 * 这里保持同样行为，保证解析器按 systemId 还原 PRN 偏移时能拿到正确星座。
 */
function mockGsaFor(sats, dop, sid, tid) {
  const list = [];
  for (let i = 0; i < 12; i++) list.push(sats[i] !== undefined ? pad(sats[i]) : '');
  return nmea(`${tid}GSA,A,3,${list.join(',')},${dop[0].toFixed(1)},${dop[1].toFixed(1)},${dop[2].toFixed(1)},${sid}`);
}

function mockGsv(sats, tid) {
  if (!sats.length) return '';
  const nmsg = Math.ceil(sats.length / 4);
  let out = '';
  for (let j = 0; j < nmsg; j++) {
    const chunk = sats.slice(j * 4, j * 4 + 4);
    let body = `${tid}GSV,${nmsg},${j + 1},${pad(sats.length)}`;
    for (let k = 0; k < 4; k++) {
      const s = chunk[k];
      body += s ? `,${pad(s.prn)},${pad(s.el)},${pad(Math.round(s.az))},${pad(s.snr)}` : ',,,,';
    }
    body += ',0';
    out += nmea(body);
  }
  return out;
}

/**
 * 生成一颗可见星（仰角/方位角随机但稳定）
 * PRN 分配贴近真实：GPS 1-32，GLONASS 1-32（偏移由调用方按 NMEA 规则处理）
 */
function genSat(prn, seed) {
  const r = (n) => {
    const x = Math.sin(seed * 9301 + n * 49297) * 43758.5453;
    return x - Math.floor(x);
  };
  const el = 5 + r(1) * 80;
  const az = r(2) * 360;
  const snr = 20 + r(3) * 25;
  // GPS 占 1-32，GLO 占 1-24（真实 GLONASS 为 1-24）
  const sys = prn <= 32 ? 'GPS' : 'GLO';
  const realPrn = sys === 'GPS' ? prn : prn - 32;
  return { prn: realPrn, sys, el, az, snr };
}

/**
 * 启动模拟：周期性地生成基线解算与卫星报文，喂给与 live 相同的入口
 */
function createMockEngine(store) {
  const stations = store.stationDict ? store.stationDict.list() : [];
  const names = stations.map((s) => s.name);
  const { FrameParser } = require('./nmea-parser');

  /**
   * 仅星历模式。
   *
   * 实测结论（2026-10-03）：上游挂点 RTCM32eph 只推星历报文
   * （1019/1020/1042/1045/1046），零 MSM 观测值。因此引擎拿不到
   * 伪距/载波相位，基线不会收敛，天空图也没有可见星。
   *
   * 打开此开关可复现该真实条件，用于验证界面的"无数据"提示是否
   * 准确（而不是误显示为引擎故障）：
   *
   *   MCORS_MOCK_EPH_ONLY=1 node index.js
   */
  const ephOnly = process.env.MCORS_MOCK_EPH_ONLY === '1';
  if (ephOnly) {
    console.log('[mock] 仅星历模式：模拟上游只推星历、无观测值的情形');
  }

  if (names.length < 2) {
    console.warn('[mock] conf 中站点不足 2 个，模拟数据有限');
  }

  // 为每个站点分配 srcid（与 live 下一致，前端无需区分）
  names.forEach((n, i) => {
    const st = store.stationDict.get(n);
    if (st) st.srcId = i + 1;
    store.srcIds.set(i + 1, n);
  });

  /* 注入一个 VRS 虚拟参考点（srcId 为负、无地址与挂载点）。
   *
   * 为什么 mock 需要它：虚拟站「着重显示」是站网图的一项实质功能，
   * 但 conf/ntripsources 里全是 M/E 类型的真实基站（实测 7 个源无一为 V），
   * 引擎也没配VRS 生成器 —— 于是开发/演示环境**永远看不到**虚拟站长什么样，
   * 这项功能只能靠代码审阅保证，改坏了也没人发现。
   *
   * 模拟的是「引擎 VRS 生成的虚拟参考点」这条路径：直接写进
   * store.srcInfoStations，字段与 applySrcInfo 的产物一致
   * （isVirtual=true、id/staSrcId 为负、经纬度已换算好）。
   * 位置取台账站群的中心附近，保证落在网图视野内。 */
  if (stations.length >= 2 && process.env.MCORS_MOCK_NO_VRS !== '1') {
    const cLat = stations.reduce((a, s) => a + (s.lat || 0), 0) / stations.length;
    const cLon = stations.reduce((a, s) => a + (s.lon || 0), 0) / stations.length;
    store.srcInfoStations = (store.srcInfoStations || []).concat([{
      name: 'VRS001',
      id: -1,
      staSrcId: -1,
      type: 'NTRIP',
      addr: '',
      port: null,
      mountpoint: '',
      state: 'virtual',
      connected: true,
      pos: null,              // 引擎侧 VRS 点无 ECEF 输出，坐标直接给经纬度
      posText: null,
      lat: cLat + 0.02,
      lon: cLon + 0.02,
      hgt: 0,
      isVirtual: true,
      antDes: null, antSno: null, marker: null,
      recType: null, recSno: null, recVer: null
    }]);
    console.log('[mock] 已注入虚拟参考点 VRS001（虚拟站着重显示功能的验证需要）');
  }

  const parser = new FrameParser();
  let tick = 0;
  let t = 0;

  const timer = setInterval(() => {
    t += 1;
    tick += 1;

    // ---- 实时帧：轮流为每个站点生成 RTK/PNT 帧 ----
    const idx = t % Math.max(1, names.length);
    const st = stations[idx];
    if (!st) return;

    const base = st;
    const rover = stations[(idx + 1) % stations.length];
    const now = new Date();

    // 解状态按周期变化，让界面能看到不同配色
    const phase = Math.floor(tick / 6) % 4;
    const quality = [4, 5, 3, 4][phase]; // FIX/FLOAT/PPP/FIX
    const statNames = { 4: 'FIX', 5: 'FLOAT', 3: 'PPP' };

    // 基线长度：由两点经纬距估算
    let blKm = 0;
    if (rover) {
      const dLat = (rover.lat - base.lat) * 111;
      const dLon = (rover.lon - base.lon) * 111 * Math.cos((base.lat * Math.PI) / 180);
      blKm = Math.hypot(dLat, dLon);
    }

    // ---- 基线解算行：复刻 cmd_showbls 格式 ----
    // 仅星历模式下没有观测值，引擎不会产生任何基线解算结果
    if (ephOnly) {
      store.baselines = [];
    } else {
      const blLine =
        `${now.toISOString().replace('T', ' ').slice(0, 19)}: ` +
        `${pad(base.srcId, 3)}->${pad(rover ? rover.srcId : base.srcId, 3)} ` +
        `stat=${statNames[quality].padStart(6)} bl=${blKm.toFixed(0).padStart(6)} KM ` +
        `age=${(1 + (tick % 3)).toFixed(0).padStart(2)}s  nb=${pad(20 + (tick % 9))}`;

      store.baselines = require('./bls-parser').parseShowBls(blLine);
    }

    // ---- 卫星与定位帧 ----
    // PRN 编排：
    //   1-32   GPS
    //   33-56  GLONASS（真实 PRN 1-24，NMEA 报文里加64 偏移后为 65-88）
    // GSA 中 GLONASS 的槽位值需为 prn+64（solution.c outnmea_gsa），
    // 因此参与解算列表里直接存编码后的值。
    const usedCodes = [];
    const allSats = [];

    for (let p = 1; p <= 56; p++) {
      const s = genSat(p, tick + p * 7);
      // GSA 里的编码值：GPS 原值，GLONASS 加 64
      const code = s.sys === 'GLO' ? s.prn + 64 : s.prn;

      // 前若干颗且仰角够高的参与解算
      if (usedCodes.length < 14 && s.el >= 15) {
        s.el = Math.max(15, s.el);
        s.snr = 32 + (s.snr % 12);
        usedCodes.push(code);
      }
      if (s.el > 8) allSats.push(s);
    }
    if (!usedCodes.length) usedCodes.push(1);

    // 仅星历模式：没有观测值，GSV/GSA 都不会产生，天空图为空
    const sats = ephOnly ? [] : allSats;

    const dop = [
      0.8 + (tick % 5) * 0.1,
      0.6 + (tick % 4) * 0.1,
      1.1 + (tick % 3) * 0.2
    ];

    let body = '\r\n<<\r\n';
    body += `$RTK[${base.name}->${rover ? rover.name : base.name}]\r\n`;
    // 仅星历模式下无观测值，GGA 应报"无定位"（quality=0）而非编造坐标
    const ggaQuality = ephOnly ? 0 : quality;
    body += mockRmc(now, [base.lat, base.lon, base.height], 5 + (tick % 12), (tick * 7) % 360);
    body += mockGga(now, [base.lat, base.lon, base.height], ggaQuality, Math.min(12, sats.length), dop[1]);

    // GSA / GSV 按星座分别输出，与 mcors outnmea_gsa/outnmea_gsv 的行为一致。
    // GN talker 下靠 systemId 区分星座（1=GPS, 2=GLONASS）。
    // 仅星历模式下上游无观测值，引擎不会输出这两类报文。
    const gpsSats = sats.filter((s) => s.sys === 'GPS');
    const gloSats = sats.filter((s) => s.sys === 'GLO');

    // GSA 槽位值：GPS 原值；GLONASS 需 +64 偏移
    const gpsUsed = usedCodes.filter((c) => c < 64);
    const gloUsed = usedCodes.filter((c) => c >= 64);

    if (!ephOnly) {
      if (gpsUsed.length) body += mockGsaFor(gpsUsed, dop, 1, 'GN');
      if (gloUsed.length) body += mockGsaFor(gloUsed, dop, 2, 'GN');

      // GSV 中 GLONASS 的 PRN 同样需 +64 偏移
      const gloGsv = gloSats.map((s) => ({ ...s, prn: s.prn + 64 }));
      body += mockGsv(gpsSats, 'GP');
      body += mockGsv(gloGsv, 'GL');
    }
    body += '>>\r\n';

    const frames = parser.push(body);
    for (const f of frames) {
      f.baseSrcId = base.srcId;
      f.roverSrcId = rover ? rover.srcId : base.srcId;
      store.srcIds.learn(f);
      if (store.onMockFrame) store.onMockFrame(f);
    }
  }, 1000);

  return {
    stop() {
      clearInterval(timer);
    }
  };
}

module.exports = { createMockEngine };