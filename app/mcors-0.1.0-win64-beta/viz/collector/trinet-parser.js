/**
 * 三角网快照解析器
 *
 * 对应引擎命令 `showtrinet`（见 src/engine/engine.c:cmd_showtrinet）。
 *
 * 为什么不让前端自己算 Delaunay？
 *   - 引擎侧用 Shewchuk triangle.c 生成的三角网是**权威拓扑**，
 *     它考虑了观测质量、边长阈值、可用性判定等业务约束；
 *   - 前端若自行按经纬度重算，得到的三角形与引擎实际用于
 *     基线解算/内插的三角形可能不一致，会误导运维判断。
 *   - viz/web/js/delaunay.js 仍保留，作为引擎未启动时的降级渲染。
 *
 * 输出格式（CSV，行首单字符区分记录类型）：
 *   H,showtrinet,<顶点数>
 *   V,<srcid>,<name>,<x>,<y>,<z>        ECEF 坐标，未定位时为 0
 *   E,<srcid1>,<srcid2>                 无向边，已按 srcid1 < srcid2 去重
 *   T,<id>,<a>,<b>,<c>                  三角面
 *   END,<nv>,<ne>,<nt>
 *
 * 注意：站名可能含逗号（如 "A001,备用"），因此 V 行按「前 2 个字段定长、
 * 后 3 个字段从尾部取」的方式切分，避免简单 split(',') 错位。
 */

/**
 * 解析 showtrinet 的原始文本输出。
 *
 * @param {string} text 引擎返回的原始文本
 * @returns {{vertices: Array, edges: Array, triangles: Array, header: object|null, tail: object|null, warnings: string[]}}
 */
function parseTrinet(text) {
  const vertices = [];
  const edges = [];
  const triangles = [];
  const warnings = [];
  let header = null;
  let tail = null;

  const vMap = new Map(); // srcid -> vertex

  if (typeof text !== 'string' || !text) {
    return { vertices, edges, triangles, header, tail, warnings: ['输入为空'] };
  }

  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line) continue;

    // END 必须先判：它的首字符是 'E'，会与边记录 'E,' 撞车。
    // 判据用「END 后紧跟逗号或行尾」，避免误吞站名以 END 开头的行。
    if (/^END(,|$)/.test(line)) {
      const f = line.split(',');
      tail = { nv: Number(f[1]) || 0, ne: Number(f[2]) || 0, nt: Number(f[3]) || 0 };
      continue;
    }

    const tag = line[0];
    // 只处理已知记录类型，其余（如引擎的提示文本）忽略并记录
    if (tag === 'H') {
      const f = line.split(',');
      header = { cmd: f[1] || '', count: Number(f[2]) || 0 };
      continue;
    }
    if (tag === 'V') {
      const v = parseVertex(line);
      if (!v) {
        warnings.push('无法解析顶点行: ' + line);
        continue;
      }
      vertices.push(v);
      vMap.set(v.srcId, v);
      continue;
    }
    if (tag === 'E') {
      const f = line.split(',');
      const a = Number(f[1]);
      const b = Number(f[2]);
      if (!Number.isFinite(a) || !Number.isFinite(b)) {
        warnings.push('无法解析边行: ' + line);
        continue;
      }
      edges.push({ a, b });
      continue;
    }
    if (tag === 'T') {
      const f = line.split(',');
      const a = Number(f[2]);
      const b = Number(f[3]);
      const c = Number(f[4]);
      if (!Number.isFinite(a) || !Number.isFinite(b) || !Number.isFinite(c)) {
        warnings.push('无法解析三角面行: ' + line);
        continue;
      }
      triangles.push({ id: f[1] || '', v: [a, b, c] });
      continue;
    }
    // 引擎可能返回 "unknown command" / 状态提示等
    if (!/^\s*$/.test(line)) warnings.push('忽略未知行: ' + line);
  }

  // 完整性校验：引擎自报的计数与实收条数是否一致
  if (tail) {
    if (tail.nv !== vertices.length) {
      warnings.push(`顶点数不符: 引擎报 ${tail.nv}，实收 ${vertices.length}`);
    }
    if (tail.nt !== triangles.length) {
      warnings.push(`三角面数不符: 引擎报 ${tail.nt}，实收 ${triangles.length}`);
    }
    // 边数不做强校验：引擎统计的是哈希表内边总数，
    // 其中可能包含参数不完整而被跳过的项（见 cmd_showtrinet 的 continue）
    if (tail.ne !== edges.length) {
      warnings.push(`边数不一致: 引擎报 ${tail.ne}，实收 ${edges.length}（可能含参数不全被跳过的边）`);
    }
  } else {
    warnings.push('缺少 END 尾行，输出可能被截断');
  }

  // 悬挂边检查：边的端点必须都在顶点集中
  const dangling = edges.filter((e) => !vMap.has(e.a) || !vMap.has(e.b));
  if (dangling.length) {
    warnings.push(`有 ${dangling.length} 条边的端点不在顶点集中`);
  }

  return { vertices, edges, triangles, header, tail, warnings };
}

/**
 * 解析 V 行。站名可能含逗号，故从两端定长切。
 * V,<srcid>,<name>,<x>,<y>,<z>
 */
function parseVertex(line) {
  const first = line.indexOf(',');
  if (first < 0) return null;
  const second = line.indexOf(',', first + 1);
  if (second < 0) return null;

  const srcId = Number(line.slice(first + 1, second));
  if (!Number.isFinite(srcId)) return null;

  // 尾部固定三个数值字段，从末尾往前找出最后 3 个逗号
  const rest = line.slice(second + 1);
  const parts = rest.split(',');
  if (parts.length < 4) return null;

  const z = Number(parts[parts.length - 1]);
  const y = Number(parts[parts.length - 2]);
  const x = Number(parts[parts.length - 3]);
  const name = parts.slice(0, parts.length - 3).join(',');

  return {
    srcId,
    name,
    x: Number.isFinite(x) ? x : 0,
    y: Number.isFinite(y) ? y : 0,
    z: Number.isFinite(z) ? z : 0,
    // 坐标为全 0 视为引擎尚未解算出该站位置
    hasPos: !(x === 0 && y === 0 && z === 0)
  };
}

/**
 * ECEF → 大地坐标（WGS-84）。
 *
 * 引擎给的是 ECEF 米制坐标，而前端画布用经纬度。转换用 Bowring 闭式解，
 * 对地固站量级的点位精度达毫米级，足够绘图。
 *
 * @returns {{lat:number, lon:number, hgt:number}|null}
 */
function ecefToGeodetic(x, y, z) {
  if (![x, y, z].every(Number.isFinite)) return null;
  const a = 6378137.0;
  const f = 1 / 298.257223563;
  const e2 = f * (2 - f);
  const b = a * (1 - f);
  const ep2 = (a * a - b * b) / (b * b);

  const p = Math.hypot(x, y);
  if (p < 1e-9) {
    // 极点附近
    const sign = z >= 0 ? 1 : -1;
    return { lat: sign * 90, lon: 0, hgt: Math.abs(z) - b };
  }

  const theta = Math.atan2(z * a, p * b);
  const st = Math.sin(theta);
  const ct = Math.cos(theta);
  const lat = Math.atan2(z + ep2 * b * st * st * st, p - e2 * a * ct * ct * ct);
  const lon = Math.atan2(y, x);

  const sinLat = Math.sin(lat);
  const N = a / Math.sqrt(1 - e2 * sinLat * sinLat);
  const hgt = p / Math.cos(lat) - N;

  return { lat: (lat * 180) / Math.PI, lon: (lon * 180) / Math.PI, hgt };
}

/**
 * 把三角网快照转换为前端画布可直接消费的结构。
 *
 * @param {string} text 引擎原始输出
 * @param {object} [opts]
 * @param {Map<number,object>} [opts.nameMap] srcid -> 站点元信息（来自 station-dict）
 * @returns {{nodes:Array, edges:Array, triangles:Array, warnings:string[], source:string}}
 */
function toGraph(text, opts = {}) {
  const snap = parseTrinet(text);
  const nameMap = opts.nameMap || null;
  const warnings = snap.warnings.slice();

  if (!snap.vertices.length) {
    return { nodes: [], edges: [], triangles: [], warnings, source: 'engine' };
  }

  const nodes = snap.vertices.map((v) => {
    const geo = v.hasPos ? ecefToGeodetic(v.x, v.y, v.z) : null;
    const meta = nameMap ? nameMap.get(v.srcId) : null;
    return {
      srcId: v.srcId,
      name: v.name || (meta && meta.name) || String(v.srcId),
      lat: geo ? geo.lat : null,
      lon: geo ? geo.lon : null,
      hgt: geo ? geo.hgt : null,
      hasPos: v.hasPos && !!geo,
      // 站点元信息优先取 conf 侧的（含中文名/行政区划）
      cnName: (meta && meta.cnName) || null,
      type: (meta && meta.type) || null,
      typeLabel: (meta && meta.typeLabel) || null
    };
  });

  const byId = new Map(nodes.map((n) => [n.srcId, n]));
  const usable = (id) => {
    const n = byId.get(id);
    return n && n.hasPos ? n : null;
  };

  // 只输出两端都有坐标的边，否则前端无法落笔
  const edges = [];
  let droppedEdges = 0;
  for (const e of snap.edges) {
    const A = usable(e.a);
    const B = usable(e.b);
    if (!A || !B) {
      droppedEdges++;
      continue;
    }
    edges.push({ a: e.a, b: e.b, aName: A.name, bName: B.name });
  }
  if (droppedEdges) {
    warnings.push(`${droppedEdges} 条边因端点缺坐标被跳过`);
  }

  // 三角面同理，并且只保留顶点可查的
  const triangles = [];
  let droppedTris = 0;
  for (const t of snap.triangles) {
    if (!t.v.every((id) => usable(id))) {
      droppedTris++;
      continue;
    }
    triangles.push({
      id: t.id,
      v: t.v,
      nodes: t.v.map((id) => byId.get(id))
    });
  }
  if (droppedTris) {
    warnings.push(`${droppedTris} 个三角面因顶点缺坐标被跳过`);
  }

  return {
    nodes,
    edges,
    triangles,
    warnings,
    source: 'engine',
    counts: {
      vertices: snap.vertices.length,
      edges: edges.length,
      triangles: triangles.length,
      positioned: nodes.filter((n) => n.hasPos).length
    }
  };
}

module.exports = { parseTrinet, parseVertex, ecefToGeodetic, toGraph };
