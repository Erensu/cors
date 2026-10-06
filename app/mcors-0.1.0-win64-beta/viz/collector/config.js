/**
 * 配置：mcors 引擎连接参数
 *
 * 通道划分（2026-10-04 起）：
 *  - solution: **唯一的数据输入源**。连 src/cors/solution.c 的推送服务
 *              （TCP 8080），每 1500ms 收一轮四类帧：$SRCINFO / $NET[cors]
 *              / $BLSOLS / $SRCSOLS。大屏所有面板的数据都从这里来。
 *  - console : **仅用于下发控制命令**（增删站 / 增删基线 / 重载配置）。
 *              它不再承担任何数据读取职责 —— 原先的 sourceinfo /
 *              showbls / showtrinet 轮询已全部移除。
 *  - monitor : 已废弃。原先订阅 NMEA 实时流（TCP 7999），现由 solution 流
 *              的 $SRCSOLS 帧取代，故整段配置删除。
 */
const path = require('path');
const os = require('os');

/**
 * 解析 conf 目录为绝对路径
 *
 * 基准是 viz/collector（本文件所在目录），保证无论从哪个 cwd 启动
 * 都能找到配置。这是踩过的坑：原先直接用 '../../conf'，从项目根启动
 * 会拼成 D:/work/src/conf，站点字典静默加载失败。
 */
function resolveConfDir(envVal) {
  if (envVal) {
    return path.isAbsolute(envVal) ? envVal : path.resolve(__dirname, envVal);
  }
  return path.resolve(__dirname, '..', '..', 'conf');
}

/**
 * 计算引擎连接的主机候选列表。
 *
 * ⚠ 为什么不能默认 127.0.0.1（实测踩到的坑）：
 *   引擎在 src/engine/engine.c:1231 用 get_local_ipaddress() 取本机 IP
 *   再 uv_tcp_bind()，**只绑定那一个地址**，不绑 127.0.0.1 也不绑 0.0.0.0。
 *   本机有多个 IPv4 时它可能选中任意一块网卡——实测选中了
 *   VMware VMnet8 的 192.168.83.1，于是连 127.0.0.1:9000 直接
 *   ECONNREFUSED。
 *
 *   因此这里返回候选列表，由客户端依次尝试，第一个连上的即采用。
 *   可用 MCORS_HOST 显式指定（最高优先级）。
 *
 * 顺序：显式指定 > 127.0.0.1 > 各非回环 IPv4
 */
function hostCandidates(envVal) {
  const out = [];
  const push = (h) => {
    if (h && !out.includes(h)) out.push(h);
  };

  push(envVal);
  push('127.0.0.1');

  try {
    const ni = os.networkInterfaces();
    for (const name of Object.keys(ni)) {
      for (const a of ni[name] || []) {
        if (a.family === 'IPv4' && !a.internal) push(a.address);
      }
    }
  } catch (e) {
    /* 取不到网卡就只用前两项 */
  }
  return out;
}

const HOSTS = hostCandidates(process.env.MCORS_HOST);

module.exports = {
  solution: {
    /* 数据推送服务（src/cors/solution.c 的 solution_svr_thread）。
     *
     * ⚠ 端口必须与 solution.c 里 `int port=8080;` 一致。
     *   8080 原本也是大屏 HTTP 的端口，两者会 EADDRINUSE ——
     *   现已把大屏挪到 8181（见下方 web.port），8080 让给 solution。
     *
     * ⚠ 引擎侧只用 get_local_ipaddress() 取的那一块网卡绑定，
     *   不绑 127.0.0.1 也不绑 0.0.0.0，故仍需多候选地址依次尝试
     *   （原因详见下方 hostCandidates 的说明）。 */
    host: HOSTS[0],
    hostCandidates: HOSTS,
    port: Number(process.env.MCORS_SOLUTION_PORT) || 8080,
    connectTimeout: 5000,
    reconnectDelay: 3000
  },

  console: {
    // 主用地址（日志展示用），实际连接走 hostCandidates
    host: HOSTS[0],
    hostCandidates: HOSTS,
    port: Number(process.env.MCORS_CONSOLE_PORT) || 9000,
    // 连接超时(ms)
    connectTimeout: 5000,
    // 命令应答等待超时(ms)
    replyTimeout: 4000,
    /* 轮询间隔：已不再用于数据读取（数据走 solution 流）。
     * 保留字段是因为「站名↔srcId」映射仍需它定期校准 —— 见 index.js。 */
    pollInterval: 3000,
    // 应答切分：静默间隔(ms)
    //
    // 引擎的提示符 "cors-engine> " 只发给本地控制台，TCP 客户端走
    // printout() 广播，只收到命令结果、收不到提示符。因此应答边界不能用
    // 提示符判定，只能按"连续 quietMs 无新数据"来切分。
    quietMs: 350
  },

  web: {
    /* 大屏 HTTP/WS 的**监听**地址。
     *
     * 默认 0.0.0.0 = 对外 + 本机都能访问（内核同时接受所有网卡含回环），
     * 这是最不容易出错的选择：本机用 127.0.0.1 进、别人用网卡 IP 进，
     * 都通。引擎的 start-gui 会在本机模式下传 127.0.0.1（只听回环，
     * 不把内部监控界面暴露到局域网）、lan 模式下传 0.0.0.0。
     *
     * ⚠ 别把监听地址和「访问地址」搞混：监听 0.0.0.0 后，本机应该用
     *   127.0.0.1 去访问（走回环最稳），**不要**用网卡 IP ——
     *   换 WiFi/断网/改 IP 都会让那个地址失效。 */
    host: process.env.MCORS_WEB_HOST || '0.0.0.0',
    /* 大屏 HTTP/WS 端口。
     *
     * 从 8080 改到 8181 的原因：solution.c 的推送服务占用 8080，
     * 两者都要 bind 8080 会冲突（Windows 下 0.0.0.0:8080 与
     * <网卡IP>:8080 也互斥）。让 solution 保持 8080 不动，
     * 大屏让到 8181，是改动面最小的方案。
     *
     * 改动同步点：无。engine.c 重写后已移除 start-gui 命令与 GUI_PORT，
     * 采集层不再由引擎拉起，改为手动/脚本启动，因此本端口只影响
     * 你打开大屏的地址：http://127.0.0.1:8181/ */
    port: Number(process.env.MCORS_WEB_PORT) || 8181,
    // 数据来源：live=连引擎, mock=内置模拟数据(引擎未启动时用)
    source: process.env.MCORS_SOURCE || 'mock'
  },

  // conf 目录：站点字典来源
  //
  // ⚠ 相对路径以**本文件所在目录**（viz/collector）为基准解析，而不是
  //   进程 cwd。否则从项目根、viz 或任意其他目录启动，'../../conf' 会
  //   落到不同位置，导致「站点字典加载失败、站网图将为空」。
  //   MCORS_CONF 环境变量可覆盖（绝对路径或相对本文件的路径均可）。
  conf: {
    dir: resolveConfDir(process.env.MCORS_CONF),
    ntripSources: 'ntripsources',
    baseStationsInfo: 'base-stations.info',
    baselines: 'beselines',
    sourcetable: 'sourcetable'
  },

  // 时序缓冲保留的历元数
  historyLimit: 720,

  // 逐星双差延迟的时序缓冲（点击逐星表格里的延迟值时画曲线用）
  //
  // 单独一份配置而不复用 historyLimit，理由是两者生命周期不同：
  //   history（定位质量）每条基线一组，条数少、每点字段多，窗口可长；
  //   satHistory 是**每颗卫星**一组，条数是前者的十几倍（10 基线 × 9 星 = 90 组），
  //   且每点只有 4 个字段。若沿用 720 历元，广播体积会失控。
  //
  // 90 组 × 120 历元 × 约 58 字节 ≈ 626 KB —— 仍然偏大，故：
  //   1) 窗口压到 120（约 2 分钟 @1Hz，足够看出趋势与跳变）；
  //   2) 广播时逐组截断到 satHistoryWindow 并只留 4 个字段（见 snapshotForPush）。
  satHistoryLimit: 120,
  // 广播给前端时每条曲线保留的点数（比本地缓冲短，抗网络抖动）
  satHistoryWindow: 120,
  // 参与逐星历史的基线数上限。
  //
  // 为什么需要上限：站点数可配置，基线随之增长，而曲线组数 = 基线数 × 卫星数。
  // 40 条基线 × 12 星 = 480 组，即使每组只 30 点也有 800 KB。
  // 取最近的 N 条基线，超出的不记录 —— 排障时用户关心的总是当前在解的那些。
  satHistoryMaxBaselines: 12
};