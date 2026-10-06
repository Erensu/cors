/**
 * Delaunay 三角剖分（纯 JS，零依赖）
 *
 * 为什么自己实现：
 *   引擎侧用的是 Shewchuk 的 triangle.c（src/dtrignet/dtrignet.c 调 triangulate()），
 *   前端要画出同构的站网三角网，但不宜为此引入 JS 版三角库（体积、许可、
 *   且 CORS 站网规模只有几十个点，增量法完全够用）。
 *
 * 算法：Bowyer-Watson 增量插入
 *   1. 构造一个足够大的「超级三角形」把全部点包住，作为初始剖分
 *   2. 逐点插入：找出所有外接圆包含该点的三角形（"坏三角形"）
 *   3. 删除坏三角形，形成空腔（星形多边形），把空腔边界与新点连成新三角形
 *   4. 全部点插入完成后，删掉所有用到超级三角形顶点的三角形
 *
 * 复杂度 O(n²)，n=几十时毫秒级。
 *
 * 退化处理（实测 CORS 站网常见）：
 *   - 完全共线的点集 → 无有效三角形，返回空（不抛异常）
 *   - 重复点（同一坐标）→ 去重后再剖分
 *   - 点数 < 3 → 返回空
 *
 * 注意：本模块用「平面坐标」而非经纬度，因为 Delaunay 的数学基础是欧氏平面。
 *   绘制时先把经纬度做等距圆柱投影（经度乘 cos(lat) 修正），再送进来。
 */
(function (global) {
  'use strict';

  /** 三点构成的有向面积的两倍（>0 逆时针，<0 顺时针，=0 共线） */
  function orient2d(ax, ay, bx, by, cx, cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  }

  /**
   * 三点外接圆判定：点 d 是否落在 (a,b,c) 的外接圆内
   *
   * 用行列式判据（Guibas-Stolfi 形式），相比「算圆心 + 比半径」少两次开方，
   * 且不引入除零风险。要求 (a,b,c) 逆时针。
   *
   *   | ax-dx  ay-dy  (ax-dx)²+(ay-dy)² |
   *   | bx-dx  by-dy  (bx-dx)²+(by-dy)² |  > 0  则 d 在圆内
   *   | cx-dx  cy-dy  (cx-dx)²+(cy-dy)² |
   *
   * ⚠ 容差带（本项目踩过的坑）：
   *   行列式的量纲是「长度⁴」，四点共圆时理论值为 0，但浮点运算会给出
   *   ±ε 级的噪声。CORS 站网里四点共圆并不罕见（规则布网、对称展点），
   *   此时符号完全由舍入误差决定，同一组点可能被判定得互相矛盾，
   *   破坏 Bowyer-Watson 的空腔星形性，产出**重叠三角形**。
   *   （实测：正五边形站网面积和是凸包的 1.89 倍。）
   *
   *   因此这里设一个相对容差带：|det| 小于 eps 时一律判为「圆外」。
   *   这等价于把共圆的四点当作「严格外侧」，从而保证判定在全网内一致
   *   —— 退化情形下得到的是**合法的**三角剖分（可能非唯一，但不会重叠）。
   */
  const EPS_REL = 1e-10;

  function inCircle(ax, ay, bx, by, cx, cy, dx, dy) {
    const adx = ax - dx, ady = ay - dy;
    const bdx = bx - dx, bdy = by - dy;
    const cdx = cx - dx, cdy = cy - dy;

    const ad = adx * adx + ady * ady;
    const bd = bdx * bdx + bdy * bdy;
    const cd = cdx * cdx + cdy * cdy;

    const det =
      adx * (bdy * cd - bd * cdy) -
      ady * (bdx * cd - bd * cdx) +
      ad * (bdx * cdy - bdy * cdx);

    // 相对容差：det 的量纲约等于 (长度²)²，用 ad/bd/cd 的尺度做归一
    const scale = ad + bd + cd;
    const tol = EPS_REL * scale * scale;

    return det > tol;
  }

  /**
   * 对点集做 Delaunay 三角剖分
   *
   * @param {Array<{x:number,y:number,i:number}>} pts 平面坐标点（i 为原始索引）
   * @returns {{triangles: Array<[number,number,number]>, edges: Array<[number,number]>}}
   *          三角形与边均为原始索引
   */
  function triangulate(pts) {
    const empty = { triangles: [], edges: [] };
    if (!pts || pts.length < 3) return empty;

    // ── 去重：同一坐标只保留一个 ──────────────────────────────
    const seen = new Map();
    const ptsU = [];
    for (const p of pts) {
      const k = p.x.toFixed(9) + ',' + p.y.toFixed(9);
      if (seen.has(k)) continue;
      seen.set(k, 1);
      ptsU.push(p);
    }
    if (ptsU.length < 3) return empty;

    // ── 度量范围 ────────────────────────────────────────────
    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    for (const p of ptsU) {
      if (p.x < minX) minX = p.x;
      if (p.x > maxX) maxX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.y > maxY) maxY = p.y;
    }
    const dx = maxX - minX;
    const dy = maxY - minY;
    const dmax = Math.max(dx, dy);

    // 全部共点（已在去重后排除）或 dmax 为 0
    if (!(dmax > 0)) return empty;

    // ── 超级三角形：把点集包在内部，留足余量 ──────────────────
    // 用 20 倍跨度，保证超级三角形的三个顶点不会出现在最终结果里，
    // 也避免因余量不足导致外接圆判定溢出（浮点相对误差 ~1e-16）。
    const midX = (minX + maxX) / 2;
    const midY = (minY + maxY) / 2;
    const R = dmax * 20;

    const sp = [
      { x: midX - R * 2, y: midY - R, i: -1 },
      { x: midX + R * 2, y: midY - R, i: -2 },
      { x: midX, y: midY + R * 2, i: -3 }
    ];

    // ── 共线预检：若全部点共线，直接返回空 ────────────────────
    // 从点集中找两个距离最远的点作基准线，检查其余点是否都在线上。
    {
      let p0 = ptsU[0], p1 = ptsU[1];
      let bd = -1;
      for (let a = 0; a < ptsU.length; a++) {
        for (let b = a + 1; b < ptsU.length; b++) {
          const d2 = (ptsU[a].x - ptsU[b].x) ** 2 + (ptsU[a].y - ptsU[b].y) ** 2;
          if (d2 > bd) { bd = d2; p0 = ptsU[a]; p1 = ptsU[b]; }
        }
      }
      const scale = Math.sqrt(bd) || 1;
      let maxOff = 0;
      for (const p of ptsU) {
        const off = Math.abs(orient2d(p0.x, p0.y, p1.x, p1.y, p.x, p.y)) / scale;
        if (off > maxOff) maxOff = off;
      }
      // 相对偏差小于 1e-9 视为共线（纯共线站网无三角形可画）
      if (maxOff < 1e-9 * Math.max(1, scale)) return empty;
    }

    // ── 增量插入 ────────────────────────────────────────────
    // 三角形用顶点对象数组表示；超级三角形的三个顶点也视为普通顶点，
    // 最后统一剔除即可，无需特殊标记每个三角形。
    let tris = [{
      a: sp[0], b: sp[1], c: sp[2],
      // 保证逆时针（外接圆判定的前提）
      ok: true
    }];
    normalizeCCW(tris[0]);

    for (const p of ptsU) {
      const bad = [];
      for (const t of tris) {
        if (inCircle(t.a.x, t.a.y, t.b.x, t.b.y, t.c.x, t.c.y, p.x, p.y)) {
          bad.push(t);
        }
      }
      if (!bad.length) continue;

      // 空腔边界：只出现过一次的边（内部边成对出现）
      const edgeMap = new Map();
      for (const t of bad) {
        addEdge(edgeMap, t.a, t.b);
        addEdge(edgeMap, t.b, t.c);
        addEdge(edgeMap, t.c, t.a);
      }

      // 移除坏三角形，保留边界边
      const badSet = new Set(bad);
      tris = tris.filter((t) => !badSet.has(t));

      for (const e of edgeMap.values()) {
        if (e.count !== 1) continue;
        const nt = { a: e.p, b: e.q, c: p };
        // 跳过退化三角形：新点恰好落在边界边上（共线）时面积为 0，
        // 这种三角形不贡献面积，混入结果会造成边重复与后续判定异常。
        if (Math.abs(orient2d(e.p.x, e.p.y, e.q.x, e.q.y, p.x, p.y)) < 1e-12) {
          continue;
        }
        normalizeCCW(nt);
        tris.push(nt);
      }
    }

    // ── 剔除含超级三角形顶点的三角形 ──────────────────────────
    const final = tris.filter(
      (t) => t.a.i >= 0 && t.b.i >= 0 && t.c.i >= 0
    );

    const triangles = final.map((t) => [t.a.i, t.b.i, t.c.i]);

    // ── 收集边（去重，无向） ──────────────────────────────────
    const es = new Map();
    const key = (u, v) => (u < v ? u + '_' + v : v + '_' + u);
    for (const [a, b, c] of triangles) {
      for (const [u, v] of [[a, b], [b, c], [c, a]]) {
        es.set(key(u, v), [Math.min(u, v), Math.max(u, v)]);
      }
    }

    return { triangles, edges: [...es.values()] };
  }

  /** 保证三角形顶点逆时针排列（原地修改） */
  function normalizeCCW(t) {
    if (orient2d(t.a.x, t.a.y, t.b.x, t.b.y, t.c.x, t.c.y) < 0) {
      const tmp = t.b;
      t.b = t.c;
      t.c = tmp;
    }
    return t;
  }

  function addEdge(map, p, q) {
    const k = p.i < q.i ? p.i + '_' + q.i : q.i + '_' + p.i;
    const e = map.get(k);
    if (e) e.count++;
    else map.set(k, { p, q, count: 1 });
  }

  global.MCORS = global.MCORS || {};
  global.MCORS.delaunay = { triangulate, orient2d, inCircle };
})(typeof window !== 'undefined' ? window : globalThis);
