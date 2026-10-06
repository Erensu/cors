/**
 * 基线解算信息面板
 * 数据来自采集层对 showbls 命令输出的解析（engine.c cmd_showbls 格式）
 */
(function (global) {
  'use strict';

  const U = global.MCORS.util;

  /* 数值 → 显示串。null/undefined/NaN 统一显示为破折号。
   * $BLSOLS 的数值字段在「该历元无解」时可能缺失，直接 U.num 会显示
   * "NaN"，故统一走这里，保证界面上不会出现 NaN。 */
  function numOrDash(v, digits) {
    const n = Number(v);
    if (v === null || v === undefined || v === '' || !Number.isFinite(n)) return '—';
    return digits === undefined ? String(n) : n.toFixed(digits);
  }

  /**
   * 逐星双差大气延迟的算术平均 (m)。无有效逐星行时返回 null。
   *
   * 引擎 $BLSOLS 逐星行给三路延迟：
   *   atmo = 大气延迟合计、trpd = 对流层、iond = 电离层
   * atmo 已含 trpd+iond，是「总的大气影响」，故用它代表整列。
   *
   * 取**带符号**的算术平均（不是 |atmo| 均值）：双差后的延迟有正有负，
   * 那是真实的系统性偏差方向；先取绝对值会把符号信息抹掉，
   * 也让「平均」这个量失去意义。表头与单元格均标注「平均」。
   *
   * 非有限值（该历元无解时 atmo 可能缺失）在求和前剔除，
   * 分母只算有效样本数，避免用缺失值稀释平均值。
   */
  function meanAtmo(b) {
    const arr = (b && b.sats) || [];
    const vals = arr.map((s) => s.atmo).filter((v) => Number.isFinite(v));
    if (!vals.length) return null;
    return vals.reduce((a, v) => a + v, 0) / vals.length;
  }

  /** 参与平均的有效逐星数。表头/详情里标注「平均 N=」。 */
  function atmoCount(b) {
    const arr = (b && b.sats) || [];
    return arr.map((s) => s.atmo).filter((v) => Number.isFinite(v)).length;
  }

  /** 频点下标 → 名称。索引与引擎 NFREQ 顺序一致。 */
  function freqName(f) {
    return f === 0 ? 'L1' : f === 1 ? 'L2' : f === 2 ? 'L5' : 'F' + f;
  }

  /* 基线对的唯一标识：`基线名→流动站名`。
   *
   * 它有两个用途，都必须**跨帧稳定**：
   *   ① 排序的最终决胜键（见 render() 注释）；
   *   ② _blKey / 行级 diff 的匹配键（决定哪一行是"同一行"）。
   * 故写成模块级纯函数，不缓存在数据对象上 —— 数据对象每帧都是新的
   * （采集层 `store.baselines = baselines` 全量替换），缓存不会命中。 */
  function blKeyOf(b) {
    const bn = b.baseName || String(b.baseSrcId ?? '');
    const rn = b.roverName || String(b.roverSrcId ?? '');
    return bn + '\u2192' + rn;
  }

  /* 行匹配键：优先用名册分配的稳定 rowKey，直塞的裸数据对象退回 blKeyOf。
   *
   * 为什么需要两套键：blKeyOf 是「名字」键 —— 而 srcid→站名映射是
   * **渐进学习**的（$SRCINFO / $BLSOLS 头行陆续补全），同一行某帧叫
   * `ID3→ID6`、下一帧就叫 `SPWD00AUS0→VRS001`。若直接拿它当 DOM 行的
   * 匹配键，名字一学全，所有行都会被当成「新行」重建一遍 —— 又是一次全表闪。
   * 名册键（rowKey）用 **srcid 无向对**，跨帧、跨命名学习都稳定。
   */
  function rowKeyOf(b) {
    return b.rowKey || blKeyOf(b);
  }

  /* ================================================================
   * 「名册 + 填入」模型（2026-10-05 第五次报障后重构）
   *
   * 用户原话：「把基线列出来，然后从 cors-engine 接收到的数据
   *           对应填入不就可以了吗？这样表格显示就不会跳来跳去了」
   *
   * 之前的模型是「显示 = 最后一帧」：表格每一帧整体替换成引擎本帧
   * 给出的基线列表。这个模型把上游的每一次抖动都原样放大到界面上：
   *   · 某帧 $BLSOLS 缺某条基线（该历元没解出来/解析丢弃）→ 行消失；
   *   · 某帧整包为空 → 全表清空 + 行序记忆重置，下一帧恢复时
   *     按**当时的输入顺序**重新编号 —— 若引擎两帧的行序不同，
   *     整张表就洗一次牌。用户截图实证：14 行数据下面沉着一行
   *     「暂无基线解算数据」—— 正是「空帧 → 数据帧」切换留下的残骸。
   *
   * 新模型：表格行 = **名册**（roster），数据帧只负责「往对应行里填数」：
   *   · 名册的种子来自 trinet（引擎三角网 = 权威基线拓扑，与网图一一
   *     对应、**无向等价**：1→2 与 2→1 是同一条基线）——网图里有的
   *     基线先显示「待解算」占位行，数据没到也先把基线列出来；
   *   · 播种**只收两端都有真实站名的边**：toGraph 对无名顶点把 name
   *     回退成 String(srcId)，实测引擎三角网里有无解算输出的幽灵顶点
   *     （顶点 1：无名、不在 $SRCINFO、$BLSOLS 从不输出它），不过滤
   *     会产出「1→X」永不填入的占位行（用户截图实证）；
   *   · $BLSOLS 每帧按 srcid 无向对匹配到名册槽位，**原地更新**数值
   *     （数据帧方向与播种方向相反也能对上号 —— 键无向）；
   *   · 数据里的新基线（如 VRS001 虚拟站，网图没有的）首次出现 →
   *     追加末尾；
   *   · 某帧缺某条基线 → 该行**保留**，超过 STALE_MS 没更新才标灰
   *     （陈旧），行数与顺序都不动；
   *   · 只有引擎真断链（caps.level === 'error'）才清空名册 ——
   *     那是「重新开始」，与「一轮空帧」是两回事。
   *
   * 行数跳变、顺序洗牌这两类抖动由此**从结构上**消失：
   * 名册只增（新基线）或整体清空（断链），不存在中间态。
   * ================================================================ */

  /* 陈旧阈值：solution 流每 1.5s 一轮（$BLSOLS 每轮都推），
   * 连续约 2.5 轮没等到某条基线的新解算 → 判陈旧。取 4s 整避开边界抖动。 */
  const STALE_MS = 4000;

  /* 已删除基线的移除缓冲（2026-10-06 用户需求）：
   * 「删除掉的基线，经过 30 秒后从表格中删除」。
   * 判定「已删除」= 最近一次播种的网图边集里没有它 **且** 无数据或数据已
   * 陈旧 —— 网图拓扑确认这条边没了（删基站 / Delaunay 重剖淘汰），同时
   * 解算侧也停了。满足后开始 30s 倒计时，到期从名册删除，行走既有
   * .bl-leaving 淡出路径消失（无断崖感）。
   * 为什么必须两个条件同时成立：
   *   · 只看「不在网图」会误杀 VRS001 虚拟站基线 —— 它不在网图拓扑里，
   *     但数据每帧都来（lastSeen 一直刷新），必须保护；
   *   · 只看「数据陈旧」会在上游断流时把全表 30s 内清空 —— 违背名册
   *     「行不因数据抖动而消失」的设计初衷；网图拓扑才是删除的权威来源。
   * 倒计时期间行保持原样（灰显 / 待解算占位），复活则取消倒计时。 */
  const REMOVE_DELAY_MS = 30000;

  /* 名册槽位键：**无向**的基线对标识，srcid 优先。
   * 无向：trinet 的边是 `E,<id1>,<id2>`（已按 id1<id2 去重），$BLSOLS
   * 的行是有向的 `base→rover`；同一条物理基线两处给的方向一致，
   * 但键若带方向，两处就对不上号。srcid：站名是渐进学习的，id 才稳定。 */
  function slotKeyOf(aId, bId) {
    const x = Number(aId), y = Number(bId);
    if (Number.isFinite(x) && Number.isFinite(y)) {
      return x < y ? 'id:' + x + '|' + y : 'id:' + y + '|' + x;
    }
    return null;
  }

  /* srcid 缺失时的兜底键（按名字无向对）。正常链路两处都有 srcid，
   * 这条只为测试桩/异常数据兜底。 */
  function nameSlotKey(b) {
    const a = String(b.baseName ?? b.baseSrcId ?? '?');
    const c = String(b.roverName ?? b.roverSrcId ?? '??');
    return 'nm:' + (a < c ? a + '|' + c : c + '|' + a);
  }

  /* ================================================================
   * 排序稳定性：「沉降式排序」
   *
   * 问题（实测 2026-10-05，live 模式 10 条基线）：
   *   `level` 分支的第一关键字是解状态序号，而 10 条基线**全是 FIX**，
   *   第一关键字恒为 0；原第二关键字 `a.time.localeCompare(b.time)`
   *   也不起作用 —— `time` 是本帧历元时间，同一帧里 10 条**完全相同**
   *   （实测 10 条 time 全为 `2026/10/05 11:11:37.000`）。
   *   于是比较函数对任意两项都返回 0，顺序退化为输入数组顺序，
   *   输入又每帧全量替换 → 表格每帧重排。
   *
   * 只加「唯一决胜键」还不够：
   *   `poserrNorm` 这类**排序键本身就在抖**。实测同一时刻
   *   SPWD→WGBA 的 poserrNorm = 0.02387，下一帧就变了；
   *   按它降序排，两条基线会随数值大小反复对调。
   *   单纯用 key 决胜只能保证「相等时」不乱，挡不住「不相等时正常换位」。
   *
   * 解法 = 主键分档 + 档位粘滞（hysteresis）：
   *   ① 主键**量化到档位**（解状态序号本身就是档；poserr 取到 0.1m、
   *      长度取到 1km、星数取到 1颗）。同一档内不再比较主键，
   *      彻底消除"数值微抖 → 位置对调"。
   *   ② 记录每条基线**上一帧落在哪个档位**；本帧算出的档位与上次不同
   *      且差值未超滞回阈值时，**沿用上次档位**。
   *      这样只有真实的状态跃变（FLOAT→FIX）才会跨档移动，
   *      在档位边界附近抖动的项会被"吸住"不动。
   *   ③ 档内按唯一键排序 —— 绝对确定，与输入顺序无关。
   *   ④ 与上一帧的显示顺序做**最长公共子序列（LCS）保持**：
   *      任何一项都不因为无关项增删而被挤走（见 settleOrder）。
   *
   * 结果：连续几帧里，只要解状态与档位没真变，表格逐像素稳定。 */

  /** 把 order 序号数组转成 0/1/… 的密集秩，避免档位值跨度太大不好设阈值 */
  function denseRank(vals) {
    const uniq = Array.from(new Set(vals)).sort((a, b) => a - b);
    const idx = new Map(uniq.map((v, i) => [v, i]));
    return vals.map((v) => idx.get(v));
  }

  /* 预设排序模式 → 主键提取器。
   *
   * 每个提取器返回**已量化的档位**（不是原始浮点）—— 这是档位粘滞的前提。
   * 排序统一走「档位升序」，所以**需要降序展示的模式必须在 band 里取负号**，
   * 而不是在比较函数里翻。让所有模式共用同一条 `bands[x] - bands[y]` 路径，
   * 决胜逻辑才一致。
   * 踩过的坑：poserr 忘了取负，误差最大的基线反而排到了最后 ——
   * 表头写着「按定位误差」而实际是升序，最该先看的那几条沉在底下。 */
  const MODES = {
    level: {
      label: '解状态',
      /* 解状态天生离散，序号即档位。FIXDEG 视作 FIX 同档（都是固定解，
       * 差别只在是否用了 GGA 辅助，不值得让行跳位置）。 */
      band: (b) => {
        const t = { fix: 0, fixdeg: 0, float: 1, ppp: 2, dgps: 3, sbas: 4, single: 5, dr: 6, none: 7 };
        return t[b.level] ?? 7;
      },
      /* 滞回阈值：0 表示状态序号差值 ≥1 才换档 —— 状态变化本就该立刻反映 */
      hysteresis: 0
    },
    length: {
      label: '基线长度',
      /* 长度是**静态几何量**，本身不抖；量化到 1km 只为让等长项聚在一起。
       * 取负 → 降序（长的在前）。
       *
       * 无值的哨兵同样是**正** 9999（见 poserr 的说明）：
       * 有效值取负后最大为 -0，故 +9999 必沉底。
       * 注意 `VRS001` 那条长度为 0（虚拟参考站基线的占位值）——
       * 它是**合法值**，取负后为 0，会排在所有真实基线之后、
       * 无值项之前，位置合理。 */
      band: (b) => (Number.isFinite(b.baselineKm) ? -Math.round(b.baselineKm) : 9999),
      hysteresis: 1
    },
    ns: {
      label: '参与星数',
      /* 升序展示（星少的在前）：星少说明可用观测少、解更脆弱。 */
      band: (b) => (Number.isFinite(b.ns) ? b.ns : 99),
      hysteresis: 1
    },
    poserr: {
      label: '定位误差',
      /* 取负 → 降序（误差最大的在前），这是运维最该先看的几条。
       *
       * ⚠ 取负只作用于**有效值**；无值必须给**正**的哨兵值。
       * 排序恒为「档位升序」，若要无值沉底，它的档位就得比任何有效值都大。
       * 有效值取负后最大也就是 -0（poserr=0 时），故 +9999 必然沉底。
       * 踩过的坑：哨兵写成 -9999（跟着取负一起翻），结果无 poserr 的
       * 基线冒到了表头第一位 —— C 位给了"没有数据"的行，
       * 真正误差大的那几条反而看不见。
       *
       * 不给 0 是因为 0 是「误差完美」的合法值，两者必须区分。
       *
       * 这是抖得最厉害的键：实测同一条基线相邻帧 poserrNorm 就从
       * 0.0239 变成别的值。量化到 0.1m 一档，再加 1 档滞回 ——
       * 0.1m 级的位置对调在 3 位小数显示下肉眼几乎看不出，
       * 却会导致整行横跳。 */
      band: (b) => (Number.isFinite(b.poserrNorm) ? -Math.round(b.poserrNorm * 10) : 9999),
      hysteresis: 1
    }
  };

  /* 逐星延迟表也做稳定排序：同星多频点在同一行内固定按 freq 升序，
   * 引擎输出顺序虽已稳定，但不依赖上游行为更安全。 */
  function sortSats(sats) {
    return (sats || []).slice().sort((a, b) => {
      const p = String(a.prn || '').localeCompare(String(b.prn || ''));
      if (p !== 0) return p;
      const fa = Number(a.freq) || 0;
      const fb = Number(b.freq) || 0;
      return fa - fb;
    });
  }
  void sortSats;

  /* ================================================================
   * 用「最长公共子序列」把新顺序对齐到旧顺序
   *
   * 场景：表格按某种规则排序，但规则无法保证跨帧稳定 ——
   *   ① 新基线随时出现、旧基线随时消失（基站上下线）；
   *   ② 排序键本身在抖（poserrNorm 之类）。
   *
   * 直接按规则重排的后果是「蝴蝶效应」：一条新基线排到第 2 位，
   * 原来第 2~10 位全部顺势下移一格。用户看到整张表在滑动，
   * 哪怕这些基线自身数据**一点没变**。
   *
   * 做法：只把「相对次序真的变了」的那部分挪动，其余保持原位。
   * 具体是求 `新顺序 ∩ 旧顺序` 的最长公共子序列（LCS）——
   * 子序列里的项，相对顺序在两帧中一致，理应原地不动；
   * 不在子序列里的项（新增 + 跨档移动的）才重新插位。
   *
   * 复杂度 O(n·m)，n/m 是基线数（实测 10 条），可忽略。
   *
   * @param {Array} next   按规则算出的新顺序
   * @param {Array} prevKeys 上一帧显示顺序的 key 数组
   * @param {Function} keyOf 元素 → key
   * @returns {Array} 对齐后的新顺序
   * ================================================================ */
  function settleOrder(next, prevKeys, keyOf) {
    if (!next.length) return next;
    if (!prevKeys || !prevKeys.length) return next;

    const nk = next.map(keyOf);
    const pos = new Map(prevKeys.map((k, i) => [k, i]));

    /* dp[i][j] = nk[i..] 与 prevKeys[j..] 的 LCS 长度。
     * 只保留一个滚动行即可，但这里项数极小，写满矩阵更好读。 */
    const n = nk.length;
    const m = prevKeys.length;
    const dp = Array.from({ length: n + 1 }, () => new Uint16Array(m + 1));
    for (let i = n - 1; i >= 0; i--) {
      for (let j = m - 1; j >= 0; j--) {
        dp[i][j] = (nk[i] === prevKeys[j])
          ? dp[i + 1][j + 1] + 1
          : Math.max(dp[i + 1][j], dp[i][j + 1]);
      }
    }

    /* 回溯出「稳定项」（在 LCS 中）的下标集合，其余为「浮动项」 */
    const stable = new Set();
    for (let i = 0, j = 0; i < n && j < m; ) {
      if (nk[i] === prevKeys[j]) { stable.add(i); i++; j++; }
      else if (dp[i + 1][j] >= dp[i][j + 1]) i++;
      else j++;
    }

    /* 稳定项按它们在**新顺序**里的位置占坑；
     * 浮动项按自身顺序填入剩下的空位。
     *
     * 为什么浮动项不插回「旧位置」：它之所以浮动，正是因为它的
     * 相对次序真的变了（新增，或跨越了档位）—— 该动就动，
     * 但要保证它周围的稳定项不跟着动。 */
    const out = new Array(n);
    const slots = [];
    for (let i = 0; i < n; i++) {
      if (stable.has(i)) out[i] = next[i];
      else slots.push(i);
    }
    let s = 0;
    for (let i = 0; i < n; i++) {
      if (stable.has(i)) continue;
      out[slots[s++]] = next[i];
    }
    void pos;
    return out;
  }

  class BaselinePanel {
    constructor() {
      this.tbody = document.getElementById('blBody');
      this.sortSel = document.getElementById('blSort');
      this.data = [];
      this.caps = null;

      /* 双差详情抽屉：逐星大气/对流层/电离层延迟 + 高度角有 12+ 个量，
       * 全塞进表格会把行高撑爆、破坏一屏布局。故表格只放几个高频量，
       * 点击行看全部。 */
      this.drawer = document.getElementById('blDrawer');
      this.drawerTitle = document.getElementById('blDrawerTitle');
      this.drawerSub = document.getElementById('blDrawerSub');
      this.drawerBody = document.getElementById('blDrawerBody');
      this._detailKey = null;
      this._bindDrawer();

      /* ============================================================
       * 行序记忆与状态提示色所需的实例状态
       *
       * 全部显式初始化（不再依赖「用到时再 if (!this._x) 兜一下」）：
       *   · _rowSeq      fixed 档的「首次出现序号」表
       *   · _prevLevel   上一帧每条基线的解状态（stat），用于状态跃变提示
       *   · _rowTop      上一帧真正渲染出来的行位置（key → 下标）
       *   · _bands/_bandsMode  沉降式排序的档位粘滞记忆
       *
       * 为什么必须在这里建：这些字段在 _settleOrder / _paint 里被读写，
       * 而那两个方法在测试里常被挂在 Object.create(prototype) 的裸对象上
       * 调用。若字段只靠「首次使用时创建」，测试就得手工复刻每个字段名，
       * 少写一个就静默退化成另一条代码路径 —— 那种「测试以为在测 A、
       * 实际走的是 B」的偏差，比直接抛错难查得多。
       * ============================================================ */
      this._rowSeq = new Map();
      this._rowSeqNext = 0;
      this._seqSeenAt = new Map();
      this._prevLevel = new Map();
      this._rowTop = null;
      this._bands = null;
      this._bandsMode = null;
      this._prevOrder = [];

      /* 名册：slotKey → slot。见文件顶部「名册 + 填入」模型的说明。
       * slot 形如 { key, baseName, roverName, baseSrcId, roverSrcId,
       *             base(最近一帧完整数据|null), lastSeen, stale }。
       * Map 的插入序 = 名册序 = fixed 档的显示序（新槽只追加在尾部）。 */
      this._roster = new Map();
      /* 最近一次播种的网图边集（slotKey 集合）。null = 还没见过 trinet
       * （旧测试桩直塞路径）→ 不做「已删除」判定，行为与历史版本一致。 */
      this._seedKeys = null;
      /* 移除缓冲时长。实例属性便于测试用假时钟缩短（生产恒为 30s）。 */
      this.removeDelayMs = REMOVE_DELAY_MS;
      /* 上次播种用的 trinet 对象引用：同一份 trinet 不重复播种；
       * 采集层 rebuildTrinetIfStale 每次重建对象 → 引用变了才重播
       * （播种本身幂等，重播只为刷新待解算行的显示名）。 */
      this._trinetSeedSrc = null;

      if (this.sortSel) {
        this.sortSel.onchange = () => this.render();
      }

      const btn = document.getElementById('btnRefreshBl');
      if (btn) {
        btn.onclick = () => {
          this.onRefresh && this.onRefresh();
          U.toast('已请求刷新', '正在向引擎查询基线解算');
        };
      }
    }

    /**
     * @param {Array} baselines        基线解算列表（本帧；可为空 = 本轮没解出）
     * @param {object} [caps]          引擎能力诊断
     * @param {object} [satHistory]    逐星延迟时序（key: BL:基线→基线|卫星，**无频点后缀**）
     * @param {object} [trinet]        引擎三角网（名册的播种来源；与网图同源同数据）
     */
    setData(baselines, caps, satHistory, trinet) {
      if (caps) this.caps = caps;
      if (satHistory) this.satHistory = satHistory;
      this._mergeFrame(baselines || [], caps, trinet);
      /* this.data 从「最后一帧的原始列表」变为「名册快照」——
       * render()/openDetail() 等既有消费方无需改动。 */
      this.data = this._rosterList();
      this.render();
    }

    /* ------------------------------------------------------------
     * 把一帧数据合入名册（只增改、不删 —— 删除只有「断链清空」一种）
     * ------------------------------------------------------------ */
    _mergeFrame(incoming, caps, trinet) {
      if (!this._roster) this._roster = new Map();

      /* 引擎断链（NO_SOLUTION 等 error 级诊断）→ 名册整体作废。
       * warn 级（NO_BASELINE / EPH_ONLY / NO_FRAMES）**不清** ——
       * 那只是「这一轮没解出基线」，名册里的行转陈旧即可；
       * 因为一轮空帧就把表清空再重编号，正是之前「变来变去」的根源。 */
      if (caps && caps.level === 'error' && this._roster.size) {
        this._roster.clear();
        this._trinetSeedSrc = null;
        this._seedKeys = null;
      }

      if (trinet && trinet !== this._trinetSeedSrc) this._seedFromTrinet(trinet);

      const now = Date.now();
      for (const b of incoming) {
        const k = slotKeyOf(b.baseSrcId, b.roverSrcId) || nameSlotKey(b);
        let slot = this._roster.get(k);
        if (!slot) {
          /* 新基线首次出现（如 VRS001 虚拟站）→ 追加到名册末尾。
           * 「先来后到」由追加保证 —— 这正是行序稳定的结构性来源。 */
          slot = {
            key: k,
            baseName: b.baseName || String(b.baseSrcId ?? ''),
            roverName: b.roverName || String(b.roverSrcId ?? ''),
            baseSrcId: b.baseSrcId, roverSrcId: b.roverSrcId,
            base: null, lastSeen: 0, stale: false
          };
          this._roster.set(k, slot);
        }
        /* 显示名以后到的为准（srcid→站名映射是渐进学习的）；
         * 行的 DOM 匹配键是 slotKey（id 对），改名只重写单元格、不重建行。 */
        slot.baseName = b.baseName || slot.baseName;
        slot.roverName = b.roverName || slot.roverName;
        slot.baseSrcId = b.baseSrcId;
        slot.roverSrcId = b.roverSrcId;
        slot.base = b;
        slot.lastSeen = now;
        slot.stale = false;
      }

      /* 陈旧判定：有数据历史、但超过 STALE_MS 没更新的行。
       * 只有最后一帧没提到它才可能超时 —— 名册模型下行**不消失**，
       * 只是用降透明度告诉运维「这个数是老的」。 */
      for (const slot of this._roster.values()) {
        if (slot.base && !slot.stale && now - slot.lastSeen > STALE_MS) {
          slot.stale = true;
        }
      }

      /* 已删除基线的 30s 缓冲移除（见顶部 REMOVE_DELAY_MS 注释）：
       * 「最近一次网图播种的边集里没有它」且「无数据或数据已陈旧」
       * → 开始倒计时；到期从名册删除（行走既有淡出路径）。
       * 数据/网图任一恢复 → 取消倒计时。 */
      if (this._seedKeys) {
        const delay = this.removeDelayMs == null ? REMOVE_DELAY_MS : this.removeDelayMs;
        for (const slot of this._roster.values()) {
          const fresh = !!(slot.base && now - slot.lastSeen <= STALE_MS);
          if (!this._seedKeys.has(slot.key) && !fresh) {
            if (!slot.removeAt) slot.removeAt = now + delay;
          } else if (slot.removeAt) {
            slot.removeAt = 0;
          }
        }
        for (const [k, slot] of Array.from(this._roster)) {
          if (slot.removeAt && now >= slot.removeAt) this._roster.delete(k);
        }
      }
    }

    /* ------------------------------------------------------------
     * 用三角网播种名册：网图里的每条边 = 一行「待解算」——
     * 基线表与网图一一对应（用户要求：「基线正常是要和网图对应上的，
     * 可以不考虑方向，1→2 和 2→1 等价」；无向等价由 slotKeyOf 保证）。
     *
     * ⚠ 收紧：只播种**两端都有真实站名**的边。toGraph 对无名顶点把
     * name 回退成 String(srcId)，引擎三角网实测存在无解算输出的幽灵
     * 顶点（顶点 1：无名、不在 $SRCINFO、$BLSOLS 从不输出）—— 不过滤
     * 会产出「1→X」永不填入的占位行（用户截图实证后加的过滤）。
     *
     * 播种幂等：已存在的槽位不重建设置（保序），只在尚无数据时刷新显示名。
     * ------------------------------------------------------------ */
    _seedFromTrinet(trinet) {
      this._trinetSeedSrc = trinet;
      /* 重建边集快照：本次播种实际收下的边（与播种同一套幽灵过滤），
       * 供「已删除基线」判定使用 —— 判定必须与播种对称，否则幽灵边
       * 会让两边对不上号。 */
      this._seedKeys = new Set();
      for (const e of (trinet.edges || [])) {
        const k = slotKeyOf(e.a, e.b);
        if (!k) continue;
        /* 「回退名」= 名字恰好是自己的 srcid 字符串 → 该端不是真实站 */
        const aOk = e.aName && e.aName !== String(e.a);
        const bOk = e.bName && e.bName !== String(e.b);
        if (!aOk || !bOk) continue;
        this._seedKeys.add(k);
        const slot = this._roster.get(k);
        if (!slot) {
          this._roster.set(k, {
            key: k,
            baseName: e.aName,
            roverName: e.bName,
            baseSrcId: e.a, roverSrcId: e.b,
            base: null, lastSeen: 0, stale: false
          });
        } else if (!slot.base) {
          /* 播种过但还没等到数据：站名映射学习是渐进的，刷新一遍显示名 */
          slot.baseName = e.aName || slot.baseName;
          slot.roverName = e.bName || slot.roverName;
        }
      }
    }

    /* ------------------------------------------------------------
     * 名册 → 显示列表（插入序 = fixed 档的显示序）。
     * 有数据的槽：携带最近一帧的完整字段 + stale 标记 + 稳定 rowKey；
     * 没数据的槽（种子行）：占位对象，解状态显示「待解算」。
     * ------------------------------------------------------------ */
    _rosterList() {
      const out = [];
      for (const slot of this._roster.values()) {
        if (slot.base) {
          out.push(Object.assign({}, slot.base, {
            rowKey: slot.key,
            baseName: slot.baseName,
            roverName: slot.roverName,
            stale: slot.stale
          }));
        } else {
          out.push({
            rowKey: slot.key,
            baseName: slot.baseName, roverName: slot.roverName,
            baseSrcId: slot.baseSrcId, roverSrcId: slot.roverSrcId,
            stat: null, level: 'pending', label: '待解算',
            baselineKm: null, ageS: null, ns: null,
            poserr: null, poserrNorm: null,
            time: '', sats: [],
            stale: false, pending: true
          });
        }
      }
      return out;
    }

    /**
     * 空状态文案。
     *
     * 不再统一显示"暂无基线解算数据"，而是按采集层给出的能力诊断
     * 区分四种成因，避免把"上游没推观测值"误读成"引擎没工作"。
     */
    _emptyReason() {
      const c = this.caps;
      if (!c) return ['暂无基线解算数据', '引擎未运行或尚未建立基线'];
      if (c.code === 'NO_SOLUTION')
        return ['未连接 solution 数据源', c.hint || '请确认 cors-engine 已启动'];
      if (c.code === 'NO_FRAMES')
        return ['已连接但无数据帧', c.hint || '上游未推流'];
      if (c.code === 'EPH_ONLY')
        return ['上游仅提供星历，无法解算基线', c.hint || '缺少 MSM 观测值'];
      if (c.code === 'NO_BASELINE')
        return ['尚未建立基线', c.hint || '可在基站管理中配置基线关系'];
      if (c.code === 'MOCK')
        return ['暂无基线解算数据', '当前为 mock 数据模式'];
      return ['暂无基线解算数据', '引擎未运行或尚未建立基线'];
    }

    render() {
      if (!this.tbody) return;

      if (!this.data.length) {
        const [title, detail] = this._emptyReason();
        /* colspan 必须等于**实际列数**：本表 7 列（基线/解状态/长度/龄期/
         * 卫星数/定位误差/双差大气延迟）。原写 6 是个陈旧值 ——
         * 表格加过列但这里没跟，会让居中文字偏左约半列。 */
        this._clearRows();
        this.tbody.innerHTML =
          `<tr class="empty"><td colspan="7">${U.esc(title)}` +
          `<br><small style="color:var(--text-3)">${U.esc(detail)}</small></td></tr>`;
        /* 空状态意味着「基线集合清空」—— 上一帧的行位置记忆必须一并清掉，
         * 否则基线恢复后 _paint 会拿几十秒前的旧顺序做 LCS 对齐，
         * 试图把已经不存在的中间状态"还原"出来。
         * 同理 fixed 档的首次出现序号也要一并释放（见 _resetRowMemory）。 */
        this._resetRowMemory();
        return;
      }

      const mode = this.sortSel ? this.sortSel.value : 'fixed';
      /* 切了排序档位 → 旧档位/旧序号/旧位置全部作废。
       * 不重置的话，切档后第一帧会拿「上一档的目标顺序」当基准去算
       * 哪一行移动了，于是整表刷一层假的移动提示色。
       *
       * ⚠ 赋值必须在 if **外面**。
       *
       * 踩过的坑（2026-10-05，靠插桩才抓到）：原写法把
       * `this._lastSortMode = mode` 写在 if 体内，于是
       *   · 首帧：_lastSortMode 为 undefined → lastMode 取 mode → 相等 →
       *     不进 if → **没赋值** → _lastSortMode 仍是 undefined；
       *   · 第二帧：又是 undefined → lastMode 又取当前 mode → 又相等 →
       *     又没赋值 …
       * 结果 `mode !== lastMode` **永远为假**，切档检测形同虚设，
       * 而且每一帧都在做一次无意义的比较 —— 是一个典型的
       * 「只在变化时记录状态」写法的自锁 bug：
       * 状态还没建立起来时它恰好不记录，于是状态永远建立不起来。
       *
       * 这类 bug 不会报错、不会让任何"顺序正确性"断言变红，
       * 只表现为「提示色偶尔多刷一层」。是插桩打日志才发现的
       * （三次 render 分别是 fixed/level/fixed，last 却恒为 undefined）。 */
      const lastMode = this._lastSortMode;
      this._lastSortMode = mode;
      if (mode !== lastMode) {
        this._resetRowMemory();
      }
      const arr = this._settleOrder(this.data, mode);

      this._paint(arr);

      this._bind();
      /* 采集层每 3s 推一次快照 → 表格重渲染。若抽屉正开着，需用新数据
       * 刷新内容，否则会停留在双差量尚未成形的旧值。
       *
       * 重开会重写 drawerBody.innerHTML，逐星表连同曲线容器一起被换掉。
       * 故先把曲线的「看的是哪颗星哪个指标」记下来，重开后再恢复 ——
       * 否则用户正看着曲线，它会每 3s 闪一下消失。 */
      if (this.drawer && !this.drawer.hidden && this._detailKey) {
        const i = this._detailKey.indexOf('|');
        /* 连 cellFreq 一起带走：曲线本身按星唯一，但**高亮的是表格里
         * 某一具体格**（同一颗星有 f0/f1/f2 三行）。只带 sat+metric 的话，
         * 重开时会退回 f0 行高亮 —— 用户在 f1 行点开的曲线，3s 后高亮
         * 悄悄跳到 f0 行，且 toggle 判定（比对 cellFreq）也随之错乱：
         * 再点 f1 行会被判成「换到另一行」而不是收起。 */
        const keep = this._satChartCtx
          ? { sat: this._satChartCtx.sat, metric: this._satChartCtx.metric,
              cellFreq: this._satChartCtx.cellFreq }
          : null;
        this.openDetail(this._detailKey.slice(0, i), this._detailKey.slice(i + 1));
        if (keep) {
          /* 该卫星若已退出解算（集合变了），_openSatChart 找不到曲线，
           * 容器会显示「暂无历史数据」而不是崩掉 —— 这是可接受的降级。
           * 同理 cellFreq 对应的行可能已不存在，此时不高亮任何格。 */
          this._openSatChartByKey(keep.sat, keep.metric, keep.cellFreq);
        }
      }
    }

    /* 撤销某一行「待移除」的淡出定时器。
     *
     * 为什么需要按行撤销（而不是只能全清）：一行可以在 450ms 淡出期内
     * 「复活」（数据又出现了）。此时必须精确撤掉它自己那一个定时器，
     * 否则 450ms 后它会在屏幕上被生生删掉 —— 这就是「行数跳变」的成因。
     * 全清会把其它**确实该删**的行的定时器也一起撤销，那些行会永远
     * 留在 DOM 里（隐形但占位），同样不对。 */
    _cancelFade(tr) {
      if (!this._fadeTimerOf) return;
      const k = tr.dataset.key;
      const t = this._fadeTimerOf.get(k);
      if (t === undefined) return;
      clearTimeout(t);
      this._fadeTimers.delete(t);
      this._fadeTimerOf.delete(k);
    }

    /* 清空表格并撤销所有未完成的淡出定时器。
     *
     * 为什么必须清定时器：_paint 里消失的行挂的是
     * `setTimeout(() => tr.remove(), 450)`。若这些行还没被移除时
     * 表格就整体切到空状态（innerHTML 重写），那些 tr 已脱离文档，
     * 450ms 后仍会调用 remove() —— 无害但纯属浪费；
     * 更要紧的是 `_leaving` 集合会一直留着已死节点的引用（内存泄漏）。
     * 删行操作本身是低频的，这里老老实实全部撤销。 */
    _clearRows() {
      if (this._fadeTimers) {
        this._fadeTimers.forEach((t) => clearTimeout(t));
        this._fadeTimers.clear();
      }
      /* key → 定时器 的反查表必须同步清空，否则会留着已死 key 的映射，
       * 且后续 _cancelFade 可能去 clear 一个早已触发的定时器 id。 */
      if (this._fadeTimerOf) this._fadeTimerOf.clear();
      if (this._worseTimers) {
        this._worseTimers.forEach((t) => clearTimeout(t));
        this._worseTimers.clear();
      }
      if (this._moveTimers) {
        this._moveTimers.forEach((t) => clearTimeout(t));
        this._moveTimers.clear();
      }
    }

    /* 重置全部「跨帧记忆」。
     *
     * 只在两类时刻调用：
     *   ① render() 遇到空数据 —— 基线集合被整体清空，任何跨帧记忆都失去意义；
     *   ② sortSel 切换排序档位 —— 换了排序维度，旧档位/旧序号/旧的
     *      「上一帧位置」全都不再是有效基准。
     *
     * 为什么必须一起清（而不是各清各的）：
     * 这几张表是**互相咬合**的 —— _rowSeq 决定目标顺序，_rowTop 决定
     * 「哪一行算移动了」，_bands 决定档位粘滞。只清其中一张，另外两张
     * 就会拿一个其它维度早已推翻的旧基准去比对，于是切档后第一帧
     * 会看到一堆假的「行移动了」提示色。
     *
     * ⚠ _rowSeq 的清理有一个用户可见的后果，是有意为之：
     *   表格切到空状态再恢复时，行序会**从头重新编号**（按恢复后的
     *   首次出现顺序）。这与用户要求的「先来后到」一致 ——
     *   集合已经清空过，就不存在"原来的先后"了。
     *   若不在这里清，序号会无限增长（引擎反复解/停同一条基线时），
     *   且恢复后新行会被插到旧行后面而不是按新的到达顺序排列。 */
    _resetRowMemory() {
      if (this._rowSeq) this._rowSeq.clear();
      this._rowSeqNext = 0;
      /* _seqSeenAt 是「key 最后出现在数据里的时刻」，与 _rowSeq 一一对应 ——
       * 只清一个会留下孤儿时刻，下一帧会拿几十秒前的旧时刻去算宽限，
       * 判定「早就该释放了」而把刚恢复的行的序号立刻删掉。 */
      if (this._seqSeenAt) this._seqSeenAt.clear();
      if (this._prevLevel) this._prevLevel.clear();
      this._rowTop = null;
      this._bands = null;
      this._bandsMode = null;
      this._prevOrder = [];
    }

    /* ==============================================================
     * 计算本帧的显示顺序。
     *
     * 「fixed」档（默认）—— 行序由**名册插入序**钉死：
     *
     *   this.data 由 _rosterList() 按名册插入序生成；名册只「追加新槽」
     *   或「断链整体清空」，已有槽位的相对次序**永不改变**。
     *   下面仍维护 this._rowSeq（首次出现序号表）：它是对名册序的
     *   逐 key 复核 —— 直塞裸数据（绕过名册的调用路径，如旧测试桩）
     *   时它独立提供同样的「首次出现序」保证；走名册时两者天然一致。
     *   于是：
     *     · 已显示的行永远不动（哪怕它的解状态、误差、长度全变了）
     *     · 新基线追加到**末尾**，不插队、不把别人挤下去
     *     · 直塞路径下行消失 → 淡出移除并**释放**序号；再出现视作新行
     *       （名册路径下行不消失，只转「陈旧」灰显 —— 见 _mergeFrame）
     *
     * 其余档（level / length / ns / poserr / name）—— 沿用沉降式排序：
     *   ① 每项算档位（带粘滞）→ ② 按「档位 + 唯一键」排序
     *   → ③ 用 LCS 与上一帧顺序对齐，保住无关项的原有位置
     *
     * @param {Array} data 本帧基线列表
     * @param {string} mode 排序模式（sortSel 的 value）
     * @returns {Array} 排好序的新数组（不改动入参）
     * ============================================================== */
    _settleOrder(data, mode) {
      const keys = data.map(rowKeyOf);

      /* ---------- 固定顺序档：只按首次出现序号排 ---------- */
      if (mode === 'fixed') {
        if (!this._rowSeq) this._rowSeq = new Map();
        if (this._rowSeqNext === undefined) this._rowSeqNext = 0;

        /* 分配序号：只给**没见过**的 key 分配，已见过的一律保留原号。
         * 这一步就是「行序不变」的全部保证 —— 序号一旦给了就不动。 */
        for (const k of keys) {
          if (!this._rowSeq.has(k)) this._rowSeq.set(k, this._rowSeqNext++);
        }

        /* 释放本帧不存在的 key 的序号。
         *
         * 为什么必须释放：否则引擎反复解/停同一条基线时，序号会无限增长，
         * 且该行「回来」时不会追加到末尾，而是凭空插回原来的高位 ——
         * 表现为「消失的行突然插到中间」，与「新行追加到末尾」的约定相悖。
         *
         * ⚠ 但**不能立刻释放** —— 要宽限 450ms（= 淡出动画时长）。
         *
         * 这是本文件里最容易搞错的一处，务必看清：
         *
         *   「行还在淡出中」与「行已经真的走了」是两回事。
         *
         *   · 本帧数据里没有它，但 DOM 里那个 <tr> 正带着 .bl-leaving
         *     播 450ms 的淡出动画 —— 屏幕上**它还看得见**（在渐隐）。
         *     此时它若回来，用户眼里就是「同一行闪了一下又恢复」，
         *     它**必须留在原位**。若此刻释放序号，它会被当成新行拿走
         *     一个更大的号 → 跳到表尾：用户看到的就是「闪了一下，
         *     然后那行突然跑到最后面去了」，比不管还糟。
         *
         *   · 真的走了（过了淡出时长）之后才该释放。
         *     这时用户眼里它已经消失，回来就是「新来的」，追加末尾正确。
         *
         * ⚠ 判定**不能**读 _fadeTimerOf。踩过的坑（变异 C2 暴露）：
         *   _settleOrder 跑在 _paint **之前**，而行刚消失的那一帧，
         *   那个 450ms 定时器是在 _paint **里面**才挂上的 ——
         *   于是「消失当帧」_fadeTimerOf 里还没有这个 key，
         *   判定成「已走干净」→ 立刻释放 → 淡出期内复活时跳表尾。
         *   实测：原序号 2 被释放，复活后拿到 5（表尾）。
         *
         * 正确做法：在本方法内自己维护「该 key 上次出现在数据里的时刻」，
         * 用**时间**而不是某张由别人维护的表来算宽限。
         * 这样它与 _paint 的执行顺序完全解耦 —— 谁先谁后都不影响判定，
         * 也不会重蹈「依赖别人的清理时机」这种隐式耦合。
         *
         * 用 Date.now() 而非帧计数：真实数据是每 3s 一帧，
         * 但用户手动切排序档会触发额外 render()，帧间隔不均匀；
         * 淡出时长本身就是**时间**概念（450ms），用时间对齐最直白。 */
        const now = Date.now();
        if (!this._seqSeenAt) this._seqSeenAt = new Map();

        /* 本帧出现的 → 刷新它的「最后出现时刻」 */
        for (const k of keys) this._seqSeenAt.set(k, now);

        const alive = new Set(keys);
        const GRACE = 450;              /* = CSS bl-leave 动画时长 */
        for (const k of Array.from(this._rowSeq.keys())) {
          if (alive.has(k)) continue;
          const seen = this._seqSeenAt.get(k);
          /* 仍在宽限期内 → 留住序号，等它可能复活 */
          if (seen !== undefined && (now - seen) < GRACE) continue;
          /* 超出宽限期 → 真走了，释放 */
          this._rowSeq.delete(k);
          this._seqSeenAt.delete(k);
        }

        /* 按序号升序。序号互不相同 → 结果是**全序**，与输入顺序无关，
         * 不需要任何决胜键，也不需要稳定性保证。
         * 用 map+sort 而不是原地排，避免改动调用方传入的数组。 */
        const ordered = data
          .map((b, i) => ({ b, i, s: this._rowSeq.get(keys[i]) }))
          .sort((x, y) => (x.s - y.s) || (x.i - y.i))
          .map((o) => o.b);

        /* 固定序下 LCS 对齐无意义（顺序本就是唯一确定的），
         * 但仍要维护 _prevOrder，供切回其它档位时作为对齐基准。 */
        this._prevOrder = ordered;
        return ordered;
      }

      /* ---------- 以下是原有沉降式排序（可选档位） ---------- */
      if (mode === 'name') return this._orderByName(data, keys);

      const m = MODES[mode] || MODES.level;
      const raw = data.map((b) => m.band(b));

      /* 档位粘滞：与上一帧比较，只有跨过 hysteresis 阈值才认这次变化。
       *
       * 为什么必须有：poserrNorm 这类排序键**本身每帧都在抖**，
       * 实测同一条基线相邻帧数值就不同。若严格按当前值排序，
       * 两条基线会在 3 位小数的第 4 位上来回对调 —— 用户看到的就是「行在跳」。
       * 粘滞后，0.1m 以内的变化不再触发换位，只有真实跃变才会动。
       *
       * mode 变了则丢弃历史档位（换了排序维度，旧档位无意义）。 */
      const prev = (this._bands && this._bandsMode === mode) ? this._bands : null;
      const bands = raw.map((v, i) => {
        const p = prev ? prev.get(keys[i]) : undefined;
        if (p === undefined) return v;
        /* hysteresis=0（解状态）时严格跟随；>0 时容差内的抖动不动 */
        if (m.hysteresis > 0 && Math.abs(v - p) <= m.hysteresis) return p;
        return v;
      });

      /* 档位 + 唯一键，两级都确定 → 排序结果与输入顺序无关 */
      const idx = data.map((_, i) => i);
      idx.sort((x, y) => (bands[x] - bands[y]) || keys[x].localeCompare(keys[y]));

      const ordered = idx.map((i) => data[i]);

      /* 与上一帧顺序对齐。缺了这步会出现「蝴蝶效应」：
       * 新来一条基线（或某条状态变了）排到最前，其余全部顺势下移一格 ——
       * 即使它们自身数据毫无变化。视觉上就是整张表在滑动。 */
      const settled = settleOrder(
        ordered,
        (this._prevOrder || []).map((b) => rowKeyOf(b)),
        rowKeyOf
      );

      /* 记下本帧档位与顺序，供下一帧做粘滞与对齐。
       *
       * 关于 `_prevOrder` 存 ordered 还是 settled：这里存 settled，
       * 因为 LCS 的语义是「保住用户**上一眼看到的**顺序」，理应用
       * 上一帧真正渲染出来的序列去比。
       *
       * 但实测结论是**两者恒等**，存哪个都一样 —— 值得记下来避免日后
       * 有人重新纠结：
       *   ① 首帧 `_prevOrder` 为空 → settleOrder 直接返回 ordered，
       *      即首帧 settled === ordered；
       *   ② 此后每帧 ordered 由「档位 + 唯一键」唯一确定，与输入顺序无关。
       *      若无增删，本帧 ordered 与上帧逐项相同 → LCS 全部命中
       *      → settled === ordered，归纳成立；
       *   ③ 有增删时，LCS 会把共有项保持在上帧相对次序上，
       *      而共有项在 ordered 里本就按 key 排好、相对次序与上帧一致，
       *      故对齐不改变任何位置 → 仍是 settled === ordered。
       * 换言之 LCS 真正起作用的是**保住了「输入顺序被打乱」这件事的影响**
       * （第 ① 步的输入顺序），不是修正后续帧。
       *
       * 已用 30 帧连跑 + 增删后 10 帧收敛的断言锁住这个不变量
       * （见 viz/test/bl-sort-stability-test.js 第 6b 节）。 */
      this._bands = new Map(keys.map((k, i) => [k, bands[i]]));
      this._bandsMode = mode;
      this._prevOrder = settled;

      void denseRank;   /* 保留：量化档位若需转密集秩时用得上 */
      return settled;
    }

    /* ==============================================================
     * 按「站点名」排序。
     *
     * 为什么不放进上面的 MODES 表：
     * MODES 的每个 band() 返回的是**数值档位**，比较走 `bands[x] - bands[y]`
     * 一条路径。而名字比较是字符串序（localeCompare），混进去要么得把
     * 名字映射成假数字（既丑又易错），要么得给比较函数加分支。名字排序
     * 还要求「按档位分组」的语义 —— 浮点解的基线不该因为名字靠前就
     * 挤到固定解上面。故单列一个分支：先按解状态档位分组，组内按名字。
     *
     * 注意决胜键必须**同时**用基线名与流动站名。只用基线名的话，
     * 同一基准站挂多个流动站（真实数据里 SPWD00AUS0 就挂了 3 个）
     * 会退化成一个「名字全相等」的组，组内顺序又回到输入顺序 → 每帧抖。
     * ============================================================== */
    _orderByName(data, keys) {
      const lvl = MODES.level.band;
      const idx = data.map((_, i) => i);
      idx.sort((x, y) => {
        const d = lvl(data[x]) - lvl(data[y]);
        if (d !== 0) return d;
        /* baseName/roverName 缺失时退回 key（key 本身已含 ID 兜底），
         * 保证比较键永不 undefined。 */
        const bx = data[x].baseName || keys[x];
        const by = data[y].baseName || keys[y];
        const p = String(bx).localeCompare(String(by));
        if (p !== 0) return p;
        const rx = data[x].roverName || keys[x];
        const ry = data[y].roverName || keys[y];
        const q = String(rx).localeCompare(String(ry));
        if (q !== 0) return q;
        /* 三级决胜到 key：key 是唯一键（blKeyOf 保证不重复），
         * 于是排序是**全序**，与输入顺序无关，永不抖动。 */
        return keys[x] < keys[y] ? -1 : (keys[x] > keys[y] ? 1 : 0);
      });

      const ordered = idx.map((i) => data[i]);
      /* 与上一帧顺序对齐，理由与 MODES 分支完全相同（见那里的长注释）：
       * 新来一条基线不该把其余行整体挤下一格。 */
      const settled = settleOrder(ordered, (this._prevOrder || []).map(rowKeyOf), rowKeyOf);
      this._bands = null;
      this._bandsMode = null;
      this._prevOrder = settled;
      return settled;
    }

    /* ==============================================================
     * 绘制表格：按 key 复用 <tr> DOM 节点，只更新变化的内容
     *
     * 为什么不用 innerHTML 全量重写（原实现）：
     *   ① 每帧重建全部 <tr>，任何 CSS 过渡都无法生效 —— 新节点
     *      不会从「上一帧的状态」过渡过来，只会直接出现；
     *   ② 用户正在悬停的行、正在文本选择的内容每 3s 被打断；
     *   ③ 节点全部丢弃重建，GC 压力与重排成本都高。
     *
     * 复用节点后：
     *   · 新出现的基线 → 加 .bl-enter，CSS 淡入 + 淡绿底一闪；
     *   · 消失的基线 → 行保留 450ms 淡出，避免「瞬间少一行」的断崖感；
     *   · 位置变化的行 → 加 .bl-move，短暂高亮，让用户看得出「它挪到这儿了」。
     *
     * --------------------------------------------------------------
     * ⚠ 关键教训（2026-10-05「还是不行，闪来闪去」）：
     *
     *   前一个版本是把**所有**行 appendChild 进 DocumentFragment，
     *   再整体 append 回 tbody —— 注释里当时还写着「appendChild 会自动
     *   把节点从旧父节点摘下来，于是按 arr 顺序 append 就等于完成重排」。
     *   这句话本身没错，但它漏掉了代价：
     *
     *     从文档移除再插入 → 该元素上**所有正在播放的 animation 被重启**。
     *
     *   于是每一行**每一帧**都经历一次「移除→插入」：
     *     · .bl-move 一旦被加上就每帧重启，600ms 的动画永远播不完
     *       → 该行持续闪黄；
     *     · .bl-enter 的 animationend 迟迟不触发 → 一次性类摘不掉；
     *     · 行序**完全没变**，画面照样在闪。
     *
     *   实测（viz/test/bl-flicker-repro-test.js 第 1 节）：10 行连推 5 帧，
     *   竟然发生 **100 次**节点移动 —— 即每行每帧都动一次。
     *   这就是用户看到「闪来闪去」的直接原因，与排序无关。
     *
     *   修法：**只移动真正需要换位的行**。做法是先把「当前 DOM 顺序」
     *   与「目标顺序」比一遍，从前往后逐位对齐：
     *     · 该位已经是正确的节点 → 原地不动（不 append，动画不重启）；
     *     · 不是 → 把正确的节点 insertBefore 到该位（只动这一行）。
     *
     *   insertBefore 同样会触发一次移动（动画重启），但**只对真正
     *   换了位的行**发生 —— 稳态下（行序不变）移动次数为 0。
     * ============================================================== */
    _paint(arr) {
      if (!this._fadeTimers) this._fadeTimers = new Set();
      /* key → 该行「待移除」定时器 id。必须能按 key 反查，
       * 才能在行回来时精确撤销它那一个定时器（见下方「复活」分支）。*/
      if (!this._fadeTimerOf) this._fadeTimerOf = new Map();
      /* 状态提示色定时器。与 _moveTimers 分开管理：两者时长不同
       * （移动 600ms / 状态 2400ms），混在一个集合里就无法按语义清理。 */
      if (!this._worseTimers) this._worseTimers = new Set();

      const rows = new Map();
      this.tbody.querySelectorAll('tr[data-key]').forEach((tr) => {
        rows.set(tr.dataset.key, tr);
      });
      /* 摘掉可能残留的**空状态行**（.empty 行没有 data-key，上面的
       * rows 表看不到它，逐位对齐也不会动它）。
       *
       * 不摘的后果（用户截图实证，2026-10-05 22:12）：空帧把 tbody 换成
       * 一行「暂无基线解算数据」，下一帧数据恢复后 14 行数据重新建出来，
       * 那行空状态文案**永远沉在表尾** —— 一张表同时显示「有数据」和
       * 「无数据」，自相矛盾。
       * children 做了存在性防御：旧测试桩的 tbody 可能没有该属性。 */
      const tbKids = this.tbody.children;
      if (tbKids && tbKids.length) {
        for (const c of Array.from(tbKids)) {
          if (c.classList && c.classList.contains('empty') && c.remove) c.remove();
        }
      }
      /* key → 本帧的 <tr>。后面「状态跃变提示」要用它按 key 反查节点。
       * 单独建一张首帧后不再变的表，避免在提示逻辑里再扫一遍 DOM。 */
      const nodeByKey = new Map();

      const alive = new Set();
      const nodes = arr.map((b) => {
        const k = rowKeyOf(b);
        alive.add(k);
        let tr = rows.get(k);
        if (!tr) {
          tr = document.createElement('tr');
          tr.dataset.key = k;
          /* 只用 classList.add，不要 className = 'bl-enter'。
           * 整串赋值会**清掉行上其它所有类**（如残留的 .bl-move），
           * 属于隐性副作用。 */
          tr.classList.add('bl-enter');
          /* 进场动效只播一次：动画结束后移除类，避免用户在「同一行」
           * 的后续帧里看到反复闪烁。 */
          tr.addEventListener('animationend', () => tr.classList.remove('bl-enter'), { once: true });
        } else if (tr.classList.contains('bl-leaving')) {
          /* ==========================================================
           * 「复活」分支 —— 本函数最要紧的一段（2026-10-05 行数跳变修复）
           *
           * 场景：某基线第 N 帧缺数据（被判"消失"、标 .bl-leaving、挂
           * 450ms 后 remove 的定时器），第 N+1 帧在 450ms 之内**又回来了**。
           *
           * 因为上面建 rows 用的 querySelectorAll 包含淡出中的行，
           * `rows.get(k)` 能取到那个待删的 tr，于是 `if (!tr)` 为假、
           * 复用旧节点 —— 这个复用本身是对的（保住节点、不重启动画），
           * 但**必须把"待删"状态一并撤销**，否则：
           *
           *   · .bl-leaving 的 CSS 是 `animation: bl-leave .45s forwards`，
           *     `forwards` 让终态 opacity:0 **永久保持** → 该行在屏幕上
           *     彻底隐形（数据里有、DOM 里有、就是看不见）；
           *   · 且它从此再也走不到下面"重新标 .bl-leaving"的分支
           *     （那里有 `if (contains('bl-leaving')) return;`），
           *     所以连定时器都不会重挂 —— 状态永久卡死。
           *
           * 表现就是用户看到的**行数忽多忽少、帧率不稳**：一会儿 6 行
           * 一会儿 8 行，而数据源其实是稳定的。
           *
           * 修法：撤销定时器 + 摘掉 .bl-leaving + 重播进场动效。 */
          this._cancelFade(tr);
          tr.classList.remove('bl-leaving');
          /* 重新给一次进场提示：对用户而言它确实是"又出现了"。
           * 先摘再加强制重排，否则同一帧内加的类不会重启动画。 */
          tr.classList.remove('bl-enter');
          void tr.offsetWidth;
          tr.classList.add('bl-enter');
        }
        this._fillRow(tr, b);
        /* 陈旧行：整行降透明度（数值停留在最后一次收到的结果上）。
         * 只在类变化时写 —— title 属性赋值会进 Style 阶段，没必要每帧重复。
         * 用 add/remove 而不是 toggle(force)：部分测试桩的 classList
         * 只实现了 add/remove/contains 三个方法。
         * pending（名册种子行，从未收到过数据）不算陈旧 —— 它有自己
         * 的「待解算」标识，含义是「还没等到第一次解算」，不是「断了」。 */
        const wantStale = !!(b.stale && !b.pending);
        if (tr.classList.contains('bl-stale') !== wantStale) {
          if (wantStale) tr.classList.add('bl-stale');
          else tr.classList.remove('bl-stale');
          tr.title = wantStale
            ? '该基线已超过 ' + Math.round(STALE_MS / 1000) + 's 未收到新解算，显示的是最后一次结果'
            : '';
        }
        nodeByKey.set(k, tr);
        return tr;
      });

      /* 消失的行：先标 .bl-leaving 让它淡出，450ms 后再真正移除。
       * 不能立刻摘掉 —— 那会「瞬间少一行」形成断崖感。 */
      rows.forEach((tr, k) => {
        if (alive.has(k)) return;
        if (tr.classList.contains('bl-leaving')) return;
        tr.classList.add('bl-leaving');
        const t = setTimeout(() => {
          this._fadeTimers.delete(t);
          this._fadeTimerOf.delete(k);
          /* 只有仍处于"待删"状态时才真删。
           * 若期间它复活过（_cancelFade 已撤掉这个定时器），本回调不会跑；
           * 这里是双保险，防"定时器已被撤销但回调仍排到"的边界情况。 */
          if (tr.classList.contains('bl-leaving') && tr.parentNode) tr.remove();
        }, 450);
        this._fadeTimers.add(t);
        this._fadeTimerOf.set(k, t);
      });

      if (!this._moveTimers) this._moveTimers = new Set();

      /* ---- 逐位对齐：只搬动位置不对的行 ----
       *
       * 遍历目标顺序，用一个「DOM 游标」跟着走：
       *   cursor = 当前这一位上应该出现的节点。
       * 若该位已经是它 → 只需把 cursor 推进到它的下一个兄弟；
       * 否则 → insertBefore 把它插到 cursor 前面（只动这一个节点）。
       *
       * 用「当前 DOM 里存活的行」为基准，而不是直接用 tbody.children：
       * 淡出中的 .bl-leaving 行还留在 DOM 里，它们不参与顺序，
       * 但要作为 insertBefore 的参照锚点，故不排除、只是不作为目标。 */
      let cursor = this.tbody.firstElementChild || null;
      /* 跳过开头可能存在的淡出行：它们不属于本帧顺序 */
      while (cursor && cursor.classList.contains('bl-leaving')) {
        cursor = cursor.nextElementSibling || null;
      }

      for (const tr of nodes) {
        if (tr === cursor) {
          /* 已在该位 —— 什么都不做（不 append → 动画不重启）*/
          cursor = this._nextAliveSibling(cursor);
          continue;
        }
        /* 需要就位。若该节点是新建的（还没进过 DOM），同样走这条路径。 */
        if (cursor) this.tbody.insertBefore(tr, cursor);
        else this.tbody.appendChild(tr);
        /* 插入后 cursor 不变：tr 现在正占着 cursor 的位置，
         * 下一步要处理的仍是原来的 cursor。 */
      }

      /* 位置变化的行：与上一帧的行序比较，位置不同的给个短暂底色。
       *
       * ⚠ 基准必须取「上一帧**真正渲染出来**的顺序」，即上一次 _rowTop。
       * 同时比较必须在**移动之前**完成 —— 否则本帧的 DOM 已经重排过，
       * 拿重排后的位置去比就恒等于自己。故这里用 nodes 的下标（目标位置）
       * 与 after 比较，二者同源，语义清晰。
       *
       * 只有真的换了位的行才加 .bl-move；稳态下加不上，
       * 配合上面的「只搬动需要换位的行」，动画就不会被反复重启。 */
      /* ================================================================
       * 解状态跃变 → 整行加提示色（用户选择：位置「原地不动」，但要有提示）
       *
       * 用户在「固定首次出现序」下明确要求：状态从 FIX 掉到 FLOAT 时，
       * 行**不要挪位置**（挪了就等于行序又在变），但必须能一眼看出来。
       * 故这里把「位置提示」（.bl-move）与「状态提示」分成两个独立的
       * 通道：前者由 _rowTop 比对驱动，后者由 _prevLevel 比对驱动。
       *
       * 三条设计约束：
       *   ① 只在**向差**方向变时才提示（FIX→FLOAT / FLOAT→NONE 等）。
       *      变好（FLOAT→FIX）不加色 —— 恢复是常态，每次都闪一下会把
       *      提示色变成噪音，"一直有行是黄的"就失去指示意义。
       *   ② 同一个 base 的 rover 是固定的（key 就是 基线→流动站），
       *      故 key 直接拿来做状态映射的键，不需要额外配对逻辑。
       *   ③ 首次见到的行（_prevLevel 里没有）不提示 —— 新行有 .bl-enter
       *      负责提示，再来一个提示色只会让首帧整表都染上颜色。
       *
       * 阈值用解状态序号（MODES.level.band）而不是字符串比较：
       * FIXDEG 与 FIX 同档（都是固定解），二者互相切换不该报警。
       * ================================================================ */
      const lvlBand = MODES.level.band;
      const curLevel = this._prevLevel || (this._prevLevel = new Map());
      arr.forEach((b) => {
        const k = rowKeyOf(b);
        const now = lvlBand(b);
        const was = curLevel.get(k);
        curLevel.set(k, now);
        if (was === undefined || now <= was) return;
        const tr = nodeByKey.get(k);
        if (!tr) return;                       /* 防御：理论上一定存在 */
        if (tr.classList.contains('bl-worse')) return;   /* 已在提示，别重置计时 */
        tr.classList.add('bl-worse');
        const t = setTimeout(() => {
          this._worseTimers.delete(t);
          tr.classList.remove('bl-worse');
        }, 2400);
        this._worseTimers.add(t);
      });
      /* 清掉本帧已不存在 key 的状态记忆 —— 与 _rowSeq 的释放同理：
       * 不释放的话，某条基线消失很久后回来，会被判成"刚刚变差"而闪黄。 */
      if (curLevel.size > arr.length) {
        const aliveK = new Set(arr.map(rowKeyOf));
        for (const k of Array.from(curLevel.keys())) {
          if (!aliveK.has(k)) curLevel.delete(k);
        }
      }

      const after = new Map();
      arr.forEach((b, i) => after.set(rowKeyOf(b), i));
      const before = this._rowTop || null;
      this._rowTop = after;

      if (before) {
        for (const tr of nodes) {
          const k = tr.dataset.key;
          const b0 = before.get(k);
          if (b0 === undefined) continue;        // 本帧新来的行，有 .bl-enter 负责提示
          if (b0 === after.get(k)) continue;     // 位置没变
          /* 已经在闪的话不重复加，避免不断重置动画计时 */
          if (tr.classList.contains('bl-move')) continue;
          tr.classList.add('bl-move');
          const t = setTimeout(() => {
            this._moveTimers.delete(t);
            tr.classList.remove('bl-move');
          }, 600);
          this._moveTimers.add(t);
        }
      }
    }

    /* 找 cursor 之后的下一个「非淡出中」的兄弟节点。
     * 淡出中的行仍占着 DOM 位置，但不该成为下一次 insertBefore 的锚点，
     * 否则新行会被插到一个正在消失的行前面，视觉上跳错地方。 */
    _nextAliveSibling(from) {
      let n = from && from.nextElementSibling;
      while (n && n.classList.contains('bl-leaving')) n = n.nextElementSibling;
      return n || null;
    }

    /* 用一条基线数据填充（或更新）一个 <tr>。
     *
     * ==============================================================
     * 为什么不能每帧重写 tr.innerHTML（bug#3，2026-10-05 修复）
     *
     * 原实现每帧无条件 `tr.innerHTML = '<td>…</td>…'`。浏览器对
     * innerHTML 赋值的语义是「**先清空全部子节点，再按字符串重建**」：
     *   · 7 个 <td> 全部销毁重建 → 全是新节点
     *   · 新节点上不存在任何正在进行的 CSS transition / animation
     *     → 数值变化只能「跳变」，永远做不出渐变
     *   · 每帧重建 7 个单元格 × N 行（12~20 条基线 ≈ 84~140 个节点）
     *     → 强制 Style + Layout + Paint，重绘面积覆盖整张表
     * 表现就是「数值每帧在跳 + 整张表持续重绘」= 用户说的**闪来闪去**。
     *
     * 这与 bug#1（行被搬运导致 animation 重启）是**两个独立成因**：
     * bug#1 在行序维度，bug#3 在单元格维度。只修 bug#1 时行序已完全
     * 稳定（移动次数 0），但只要数值区在更新，画面仍在闪 ——
     * 这就是「一行换一行的闪」的直接来源。
     *
     * 修法：把行拆成「骨架」与「内容」两段生命周期 ——
     *
     *   1. 首次见到该 tr：一次性建好 7 个 <td>，**缓存引用到 tr._cells**，
     *      此后该 tr 的子节点集合不再变动（节点存活 → transition 可延续）。
     *   2. 后续每帧：逐单元格比对指纹（tr._sig），**只写真正变了的那个
     *      textContent**。稳态下（数值不变）写入次数为 0。
     *
     * 为什么不只加「整行指纹相同就 return」的粗守卫：
     *   基线表每帧 ageS / poserrNorm / atmo 几乎总在变，整行指纹几乎
     *   不会相同 → 粗守卫等于不生效。必须细到「单元格」，才能把重绘
     *   面积从「整行 7 格」压到「真正跳动的那 1~3 格」。
     * ============================================================== */
    _fillRow(tr, b) {
      const base = b.baseName || `ID${b.baseSrcId}`;
      const rover = b.roverName || `ID${b.roverSrcId}`;
      const label = b.label || U.labelOf(b.stat);
      /* 时间戳作为整行 title 悬停提示：它原本是独立一整行（<div> 块级），
       * 把行高从 ~29px 撑到 ~65px，12 条基线一屏放不下；
       * 同行显示也会挤占首列宽度导致站名被截断。 */
      const t = b.time || '';
      /* 双差大气延迟：所有参与星 atmo 的算术平均（atmo 已含对流层+电离层）。 */
      const atmo = meanAtmo(b);
      const atmoN = atmoCount(b);

      /* 行级属性：dataset 赋值很轻（不触发重排），但也没必要每帧重复写 */
      if (tr.dataset.base !== base) tr.dataset.base = base;
      if (tr.dataset.rover !== rover) tr.dataset.rover = rover;

      /* ---------- 首帧：建骨架，缓存 7 个 <td> ---------- */
      if (!tr._cells) {
        tr.innerHTML =
          `<td>` +
            `<b></b>` +
            `<span class="bl-arrow">→</span>` +
            `<span class="bl-rover"></span>` +
          `</td>` +
          `<td><span class="badge"></span></td>` +
          `<td class="num"></td>` +
          `<td class="num"></td>` +
          `<td class="num" title="引擎 $BLSOLS 的 ns：参与解算卫星数"></td>` +
          `<td class="num" title="poserr 合成模（X/Y/Z）"></td>` +
          `<td class="num" title="所有参与解算卫星的双差大气延迟算术平均 (m)，含对流层与电离层；本列取平均，非单星值"><span class="avg-tag"></span></td>`;

        const td = tr.children;
        /* 骨架必须齐 7 格才能按下标取引用。理论上 innerHTML 刚写完
         * 必然成立，但若某环境（或测试桩）没能同步建出子节点，宁可
         * 退回「整行渲染」也不要抛异常把整张表打挂 —— 表格是主视图，
         * 崩了比闪一下严重得多。 */
        if (!td || td.length < 7) {
          tr._cells = null;
          this._fallbackHTML(tr, b, base, rover, label, t, atmo, atmoN);
          return;
        }
        tr._cells = {
          nameTd:     td[0],
          nameB:      td[0].children[0],   /* <b> 基准站名 */
          nameRover:  td[0].children[2],   /* 流动站名（跳过中间的 .bl-arrow） */
          badge:      td[1].children[0],
          km:         td[2],
          age:        td[3],
          ns:         td[4],
          poserr:     td[5],
          atmoTd:     td[6],
          avgTag:     td[6].children[0],
          _title: null, _peTitle: null, _tagTitle: null
        };
        /* 骨架里 <td>[0] 的 <b>/流动站名、badge 等节点若也缺失，
         * 同样视为不可用 → 走降级路径。 */
        if (!tr._cells.nameB || !tr._cells.nameRover || !tr._cells.badge ||
            !tr._cells.avgTag) {
          tr._cells = null;
          this._fallbackHTML(tr, b, base, rover, label, t, atmo, atmoN);
          return;
        }
      }

      const C = tr._cells;
      const sig = tr._sig || (tr._sig = {});

      /* ---------- 单元格精修：只写变化项 ----------
       * 每个字段先与上一帧指纹比，相同则整个跳过 —— 连 textContent
       * 赋值都不做。稳态下本函数「零写入、零重排」。 */

      /* 第 1 列：基准站名 / 流动站名 / 悬停时间戳 */
      if (sig.b !== base) { C.nameB.textContent = base; sig.b = base; }
      if (sig.r !== rover) { C.nameRover.textContent = rover; sig.r = rover; }
      /* title 属性写入会进 Style 阶段，同样加指纹守卫 */
      const wantTitle = `${base} → ${rover}　${t}`;
      if (C._title !== wantTitle) { C.nameTd.title = wantTitle; C._title = wantTitle; }

      /* 第 2 列：解状态徽标（class 变化才动 class，避免无谓的样式重算） */
      const stat = b.stat || 'NONE';
      if (sig.stat !== stat) { C.badge.className = 'badge ' + stat; sig.stat = stat; }
      if (sig.lbl !== label) { C.badge.textContent = label; sig.lbl = label; }

      /* 第 3~7 列：数值。这些是每帧真正会跳的字段，逐个守卫。 */
      const vKm = U.num(b.baselineKm, 1);
      if (sig.km !== vKm) { C.km.textContent = vKm; sig.km = vKm; }

      const vAge = U.num(b.ageS, 0);
      if (sig.age !== vAge) { C.age.textContent = vAge; sig.age = vAge; }

      const vNs = numOrDash(b.ns);
      if (sig.ns !== vNs) { C.ns.textContent = vNs; sig.ns = vNs; }

      const vPe = numOrDash(b.poserrNorm, 3);
      if (sig.pe !== vPe) { C.poserr.textContent = vPe; sig.pe = vPe; }

      const vAt = numOrDash(atmo, 3);
      if (sig.at !== vAt) { C.atmoTd.textContent = vAt; sig.at = vAt; }
      /* ⚠ 第 7 列有两个子节点：数值文本 + .avg-tag。
       * 上面的 textContent 赋值会把 .avg-tag 一并抹掉，故必须在写
       * 数值之后**重新挂回** avgTag（它被缓存了引用，节点不会重建）。 */
      if (C.atmoTd.lastChild !== C.avgTag) C.atmoTd.appendChild(C.avgTag);

      const vTag = '平均' + (atmoN ? String(atmoN) : '');
      if (sig.tag !== vTag) { C.avgTag.textContent = vTag; sig.tag = vTag; }

      const wantTagTitle = `该值为 ${atmoN} 颗参与星双差大气延迟的算术平均`;
      if (C._tagTitle !== wantTagTitle) { C.avgTag.title = wantTagTitle; C._tagTitle = wantTagTitle; }
    }

    /* 降级渲染路径：整行重写 innerHTML。
     *
     * 仅在「骨架建不出来」时走（见 _fillRow 里的 td.length < 7 判断）。
     * 这条路径**会**导致每帧重建 <td>、从而闪 —— 但它是保底，不是常态：
     * 正常浏览器里 innerHTML 一定会同步建出子节点，永远走精修路径。
     * 留着它是为了「宁可闪，不可白」：万一某环境建树行为异常，
     * 表格仍能显示正确数据，而不是整张表抛异常变成空白。 */
    _fallbackHTML(tr, b, base, rover, label, t, atmo, atmoN) {
      tr.innerHTML =
        `<td title="${U.esc(base)} → ${U.esc(rover)}　${U.esc(t)}">` +
          `<b>${U.esc(base)}</b>` +
          `<span class="bl-arrow">→</span>` +
          `${U.esc(rover)}` +
        `</td>` +
        `<td><span class="badge ${U.esc(b.stat || 'NONE')}">${U.esc(label)}</span></td>` +
        `<td class="num">${U.num(b.baselineKm, 1)}</td>` +
        `<td class="num">${U.num(b.ageS, 0)}</td>` +
        `<td class="num" title="引擎 $BLSOLS 的 ns：参与解算卫星数">${numOrDash(b.ns)}</td>` +
        `<td class="num" title="poserr 合成模（X/Y/Z）">${numOrDash(b.poserrNorm, 3)}</td>` +
        `<td class="num" title="所有参与解算卫星的双差大气延迟算术平均 (m)，含对流层与电离层；本列取平均，非单星值">` +
          `${numOrDash(atmo, 3)}` +
          `<span class="avg-tag" title="该值为 ${U.esc(String(atmoN))} 颗参与星双差大气延迟的算术平均">` +
            `平均${atmoN ? U.esc(String(atmoN)) : ''}` +
          `</span>` +
        `</td>`;
    }

    _bind() {
      /* 行点击联动网图 + 打开逐星详情抽屉。
       *
       * 整行都可点（没有独立「操作」列的按钮）：解绑基线属于**低频、
       * 需谨慎**的操作，放进每行的常驻按钮太容易误点。
       * 需要解绑时在详情抽屉里操作，或走引擎控制台的 delvsta。
       *
       * 选择器用 tr[data-key] 而不是 tr[data-base]：data-key 是每行
       * 必有的稳定标识（_paint 给所有行都设了），data-base 在
       * 站名缺失时是 `ID<n>` 形式，理论上仍会有值，但没有 data-key 可靠。
       * 同时跳过 .bl-leaving 的行 —— 它们在淡出，不该再响应点击
       * （否则用户点到一个正在消失的行，抽屉会打开一条已下线的基线）。 */
      this.tbody.querySelectorAll('tr[data-key]:not(.bl-leaving)').forEach((tr) => {
        tr.style.cursor = 'pointer';
        tr.onclick = () => {
          this.openDetail(tr.dataset.base, tr.dataset.rover);
          this.onSelect && this.onSelect(tr.dataset.rover || tr.dataset.base);
        };
      });
    }
  }

  /* ================================================================
   * 双差详情抽屉
   * ================================================================ */
  BaselinePanel.prototype._bindDrawer = function () {
    const btn = document.getElementById('blDrawerClose');
    if (btn) btn.onclick = () => this.closeDrawer();
    if (this.drawer) {
      this.drawer.onclick = (e) => { if (e.target === this.drawer) this.closeDrawer(); };
    }
    if (!this._escBound) {
      this._escBound = true;
      document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && this.drawer && !this.drawer.hidden) this.closeDrawer();
      });
    }
  };

  BaselinePanel.prototype.closeDrawer = function () {
    this._detailKey = null;
    /* 曲线实例的 window resize 监听必须摘掉：它的 canvas 已被 innerHTML
     * 移出文档，但监听还挂着的话，每次窗口变化都会执行一次对已销毁
     * canvas 的 render()，久了必然抛错。 */
    if (this._satChart) { this._satChart.destroy(); this._satChart = null; }
    this._satChartCtx = null;
    if (this.drawer) this.drawer.hidden = true;
  };

  /* 键值行。na=true 时灰字显示「无数据」，与「值为 0」区分开。 */
  function kv(label, value, mono) {
    const na = value === null || value === undefined || value === '';
    return '<div class="st-kv"><dt>' + U.esc(label) + '</dt>' +
      '<dd class="' + (na ? 'na' : (mono ? 'mono' : '')) + '">' +
      (na ? '无数据' : U.esc(value)) + '</dd></div>';
  }

  function grp(title) {
    return '<div class="st-grp">' + U.esc(title) + '</div>';
  }

  /** 打开某条基线的双差详情。 */
  BaselinePanel.prototype.openDetail = function (base, rover) {
    if (!this.drawer) return;
    const b = this.data.find((x) =>
      (x.baseName || 'ID' + x.baseSrcId) === base &&
      (x.roverName || 'ID' + x.roverSrcId) === rover);
    if (!b) return;

    this._detailKey = base + '|' + rover;
    this.drawerTitle.textContent = base + ' \u2192 ' + rover;

    const sub = [];
    sub.push(b.label || U.labelOf(b.stat));
    if (Number.isFinite(b.baselineKm)) sub.push(U.num(b.baselineKm, 1) + ' km');
    if (Number.isFinite(b.ns)) sub.push(b.ns + ' 颗参与星');
    this.drawerSub.textContent = sub.join(' \u00b7 ');

    let h = '';

    /* ---- 解状态（$BLSOLS 汇总行的各字段一一对应）---- */
    h += grp('解状态');
    h += kv('解状态', (b.label || U.labelOf(b.stat)) + '（' + (b.stat || '-') + '）');
    h += kv('观测时刻', b.time);
    h += kv('基线长度', Number.isFinite(b.baselineKm) ? U.num(b.baselineKm, 3) + ' km' : null);
    h += kv('差分龄期', Number.isFinite(b.ageS) ? U.num(b.ageS, 0) + ' s' : null);
    h += kv('模糊度个数 nb', numOrDash(b.nb));
    h += kv('参与卫星数 ns', numOrDash(b.ns));

    /* ---- 定位误差：$BLSOLS 独有字段，逐分量列出 ----
     * 合成模单独给出，因为「哪个方向偏」与「整体偏多少」是两个不同问题：
     * 前者用于判断是否有系统性形变，后者用于快速判好坏。 */
    h += grp('定位误差（poserr）');
    if (b.poserr) {
      h += kv('ΔX', numOrDash(b.poserr.x, 4) + ' m', true);
      h += kv('ΔY', numOrDash(b.poserr.y, 4) + ' m', true);
      h += kv('ΔZ', numOrDash(b.poserr.z, 4) + ' m', true);
      h += kv('合成模', numOrDash(b.poserrNorm, 4) + ' m', true);
    } else {
      h += kv('定位误差', null);
    }

    /* ---- 逐星明细 ----
     *
     * 每颗参与星与它的参考星构成一个双差观测，各有一组大气延迟。
     * 逐条列出而非只给均值，是因为**问题星可由此定位**：
     * 某颗星的电离层/对流层明显偏离同批其他星时，它就是可疑的粗差源。
     * 引擎侧逐星输出（solution.c）正是为此。 */
    const sats = b.sats || [];
    h += grp('逐星双差延迟（N=' + sats.length + '）');
    if (sats.length) {
      /* 表格列与抽屉列同源，故这里也给出「平均」汇总行，
       * 避免用户在抽屉里逐星看完后还要回表格对一次。 */
      const mA = meanAtmo(b);
      const mT = (b.sats || []).map((s) => s.trpd).filter((v) => Number.isFinite(v));
      const mI = (b.sats || []).map((s) => s.iond).filter((v) => Number.isFinite(v));
      const avg = (a) => (a.length ? a.reduce((x, v) => x + v, 0) / a.length : null);
      h += '<table class="dd-eqs">'
        + '<thead><tr><th>卫星</th><th>参考星</th><th>频</th>'
        + '<th>有效</th><th>龄期(s)</th>'
        + '<th title="点击查看该星双差大气延迟的历史曲线">大气(m)</th>'
        + '<th title="点击查看该星双差对流层延迟的历史曲线">对流层(m)</th>'
        + '<th title="点击查看该星双差电离层延迟的历史曲线">电离层(m)</th>'
        + '<th>高度角(°)</th></tr></thead><tbody>'
        + sats.map((e) => {
            /* 电离层绝对值超阈值的标红：它是双差模糊度的主要残差源，
             * 明显偏大通常意味着该星观测质量差或电离层活动剧烈。 */
            const bad = Number.isFinite(e.iond) && Math.abs(e.iond) > 0.5;
            return '<tr' + (bad ? ' class="bad"' : '') + '>'
              + '<td class="sat">' + U.esc(e.prn) + '</td>'
              + '<td class="sat">' + U.esc(e.refPrn) + '</td>'
              + '<td>' + freqName(e.freq) + '</td>'
              + '<td>' + (e.stat ? '是' : '否') + '</td>'
              + '<td class="num">' + numOrDash(e.ageS, 0) + '</td>'
              /* 三路延迟单元格可点：点谁看谁的曲线。
               * data-sat 与 collector 的 key（BL:基线→基线|卫星）对应；
               * data-freq 仅用于高亮/toggle 定位（曲线本身按星唯一）。 */
              + '<td class="num sat-plot" data-metric="atmo" data-sat="' + U.esc(e.prn)
              + '" data-freq="' + e.freq + '" title="点击查看该星双差大气延迟历史曲线">'
              + numOrDash(e.atmo, 3) + '</td>'
              + '<td class="num sat-plot" data-metric="trpd" data-sat="' + U.esc(e.prn)
              + '" data-freq="' + e.freq + '" title="点击查看该星双差对流层延迟历史曲线">'
              + numOrDash(e.trpd, 3) + '</td>'
              + '<td class="num sat-plot" data-metric="iond" data-sat="' + U.esc(e.prn)
              + '" data-freq="' + e.freq + '" title="点击查看该星双差电离层延迟历史曲线">'
              + numOrDash(e.iond, 3) + '</td>'
              + '<td class="num">' + numOrDash(e.el, 1) + '</td>'
              + '</tr>';
          }).join('')
        + '<tr class="dd-avg"><td colspan="5">平均（' + atmoCount(b) + ' 星）</td>'
        + '<td class="num">' + numOrDash(mA, 3) + '</td>'
        + '<td class="num">' + numOrDash(avg(mT), 3) + '</td>'
        + '<td class="num">' + numOrDash(avg(mI), 3) + '</td>'
        + '<td class="num">' + numOrDash(avg(sats.map((s) => s.el).filter((v) => Number.isFinite(v))), 1) + '</td>'
        + '</tr>'
        + '</tbody></table>'
        + '<div class="dd-note">大气 = 双差大气延迟合计（≈ 对流层 + 电离层），'
        + '表中为<b>所有参与星的算术平均</b>；'
        + '|电离层| &gt;0.5m 的星以红色标出，通常是可疑的粗差源。'
        + '<br><b>点击任一卫星的延迟数值</b>可查看它的历史曲线。</div>'
        /* 曲线图容器：初始隐藏，点单元格才展开。
         * 放在表格下方而非单元格内，是因为 canvas 有最小可用宽度，
         * 塞进表格单元会被列宽（15%）压成一条竖线。 */
        + '<div class="sat-chart-box" id="satChartBox" hidden>'
        + '<div class="sat-chart-hd">'
        + '<span id="satChartTitle">—</span>'
        + '<span class="sat-chart-tabs" id="satChartTabs"></span>'
        + '<button type="button" class="sat-chart-close" id="satChartClose"'
        + ' title="关闭曲线">✕</button>'
        + '</div>'
        + '<div class="sat-chart-canvas" id="satChartCanvas"></div>'
        + '<div class="dd-note" id="satChartNote"></div>'
        + '</div>';
    } else {
      h += kv('逐星明细', null);
      h += kv('说明', '该历元引擎未输出逐星行（nf=0，无有效双差方程）');
    }

    this.drawerBody.innerHTML = h;
    this.drawer.hidden = false;
    this._bindSatChart(b);
  };

  /* ================================================================
   * 逐星延迟历史曲线
   *
   * 点逐星表里的三路延迟数值 → 展开该星该指标的历史曲线。
   * 曲线数据来自采集层的 satHistory（key: BL:基线→基线|卫星，无频点后缀），
   * 与抽屉逐星表同源，因此「表里这一行的数」与「曲线当前值」必然一致。
   * ================================================================ */

  /**
   * 当前展开的曲线上下文：{ base, rover, sat, cellFreq, metric }
   *
   * sat 决定**取哪条曲线**（键里无频点，见 sat-delay-trend.js 注释）；
   * cellFreq 只用于**高亮与 toggle 定位** —— 抽屉里同一颗星有 f0/f1/f2 三行，
   * 点哪一行就该亮哪一行，即便它们指向同一条曲线。
   */
  BaselinePanel.prototype._satChartCtx = null;

  BaselinePanel.prototype._bindSatChart = function (b) {
    if (!this.drawerBody) return;
    const self = this;

    /* 旧的 chart 实例随 innerHTML 重写而失效，必须先销毁，
     * 否则它的 resize 监听会一直挂在 window 上（泄漏）。 */
    if (this._satChart) {
      this._satChart.destroy();
      this._satChart = null;
    }
    this._satChartCtx = null;

    this.drawerBody.querySelectorAll('td.sat-plot').forEach((td) => {
      td.onclick = () => {
        const sat = td.dataset.sat;
        const cellFreq = Number(td.dataset.freq);
        const metric = td.dataset.metric;
        if (!sat || !metric) return;
        /* 重复点同一格 → 收起。用户预期是 toggle，不需要额外关闭按钮。
         * 判定要连 cellFreq 一起比：点 G05/f0 再点 G05/f1 应当是「切到另一行」
         * 而非「收起」，因为用户看到的是表格里两个不同的格子。 */
        const c = self._satChartCtx;
        if (c && c.sat === sat && c.cellFreq === cellFreq && c.metric === metric) {
          self._closeSatChart();
          return;
        }
        self._openSatChart(b, sat, cellFreq, metric);
      };
    });

    const close = this.drawerBody.querySelector('#satChartClose');
    if (close) close.onclick = () => self._closeSatChart();
  };

  BaselinePanel.prototype._closeSatChart = function () {
    const box = this.drawerBody && this.drawerBody.querySelector('#satChartBox');
    if (box) box.hidden = true;
    if (this._satChart) { this._satChart.destroy(); this._satChart = null; }
    this._satChartCtx = null;
    this.drawerBody.querySelectorAll('td.sat-plot.on').forEach((td) => td.classList.remove('on'));
  };

  /** 供 render() 恢复曲线用：按 _detailKey 对应的基线重开曲线 */
  BaselinePanel.prototype._openSatChartByKey = function (sat, metric, cellFreq) {
    if (!this._detailKey || !this.data) return;
    const i = this._detailKey.indexOf('|');
    const base = this._detailKey.slice(0, i);
    const rover = this._detailKey.slice(i + 1);
    const b = this.data.find((x) =>
      (x.baseName || 'ID' + x.baseSrcId) === base &&
      (x.roverName || 'ID' + x.roverSrcId) === rover);
    if (!b) return;
    /* cellFreq 缺省为 0：卫星只有单频点时它本就只能是 0。 */
    this._openSatChart(b, sat, Number(cellFreq) || 0, metric);
  };

  BaselinePanel.prototype._openSatChart = function (b, sat, freq, metric) {
    const box = this.drawerBody && this.drawerBody.querySelector('#satChartBox');
    const cvBox = this.drawerBody.querySelector('#satChartCanvas');
    if (!box || !cvBox) return;

    const base = b.baseName || ('ID' + b.baseSrcId);
    const rover = b.roverName || ('ID' + b.roverSrcId);
    const hist = this.satHistory || {};
    const list = global.MCORS.satDelay.seriesForSat(hist, base, rover, sat);
    /* 键里没有频点（引擎对各频点输出同一组延迟，见 sat-delay-trend.js 注释），
     * 故一条星只对应一条曲线，取第一条即可。 */
    const hit = list[0] || null;

    box.hidden = false;
    this.drawerBody.querySelectorAll('td.sat-plot.on').forEach((td) => td.classList.remove('on'));
    const cur = this.drawerBody.querySelector(
      'td.sat-plot[data-sat="' + sat + '"][data-freq="' + freq + '"][data-metric="' + metric + '"]');
    if (cur) cur.classList.add('on');

    const titleEl = this.drawerBody.querySelector('#satChartTitle');
    const noteEl = this.drawerBody.querySelector('#satChartNote');
    const M = global.MCORS.satDelay.METRICS[metric] || global.MCORS.satDelay.METRICS.iond;
    if (titleEl) titleEl.textContent = `${base} → ${rover} · ${sat}`;

    /* 指标切换按钮：三路延迟都要能看，不必回表格再点一次 */
    const tabs = this.drawerBody.querySelector('#satChartTabs');
    if (tabs) {
      tabs.innerHTML = Object.keys(global.MCORS.satDelay.METRICS)
        .map((k) => {
          const m = global.MCORS.satDelay.METRICS[k];
          return '<button type="button" class="sat-tab' + (k === metric ? ' on' : '') +
            '" data-metric="' + k + '">' + U.esc(m.short) + '</button>';
        })
        .join('');
      tabs.querySelectorAll('.sat-tab').forEach((btn) => {
        btn.onclick = () => this._openSatChart(b, sat, freq, btn.dataset.metric);
      });
    }

    /* container 刚 unhide，宽度要等布局完成才准 —— 用 rAF 推一帧再量。 */
    const draw = () => {
      if (this._satChart) { this._satChart.destroy(); this._satChart = null; }
      const chart = new global.MCORS.SatDelayChart(cvBox);
      if (!chart.canvas) return;
      chart.setSat(sat, `${base} → ${rover}`);
      chart.setMetric(metric);
      chart.setData(hit ? hit.rows : []);
      this._satChart = chart;
      this._satChartCtx = { base, rover, sat, cellFreq: freq, metric };

      if (noteEl) {
        if (!hit) {
          noteEl.innerHTML = '该星暂无历史数据（引擎可能刚接入，或此卫星刚进入解算）。';
        } else if (hit.rows.length < 2) {
          noteEl.innerHTML = '正在累积历史（当前 ' + hit.rows.length +
            ' 个历元，至少需要 2 个才能画出曲线）。';
        } else {
          noteEl.innerHTML = '数据来源：采集层 satHistory，' + hit.rows.length + ' 个历元。';
        }
      }
    };
    if (typeof requestAnimationFrame === 'function') requestAnimationFrame(draw);
    else draw();
  };

  global.MCORS.BaselinePanel = BaselinePanel;
})(window);