/**
 * cors-monitor 订阅客户端（实时数据流）
 *
 * 协议（monitor.c / monitorsrc.c）：
 *   1. TCP 连到 monitor-port
 *   2. 主动发送订阅命令：  MONITOR-SOURCE <password> <mntpnt>
 *      - 命令关键字固定 "MONITOR-SOURCE"
 *      - argv[1] 是硬编码口令（cors.h CORS_MONITOT_CMD_PASSWORD）
 *      - argv[2] 为挂载点，需存在于引擎源表，否则服务端静默丢弃
 *   3. 服务端持续推送帧： \r\n<<\r\n $RTK[base->rover] ... >>\r\n
 *
 * 该通道是"推模式"：不先发订阅命令就收不到任何数据。
 */
const net = require('net');
const { FrameParser } = require('./nmea-parser');

class MonitorClient {
  constructor(cfg, onFrame) {
    this.cfg = cfg;
    this.onFrame = onFrame;
    this.parser = new FrameParser();
    this.socket = null;
    this.connected = false;
    this.subscribed = [];
    this.reconnectTimer = null;
    this.stopped = false;
    this.lastFrameAt = 0;
    this.frameCount = 0;
    this.checksumErrors = 0;
  }

  connect() {
    if (this.stopped) return;
    const { port } = this.cfg;
    const hosts = this.cfg.hostCandidates && this.cfg.hostCandidates.length
      ? this.cfg.hostCandidates
      : [this.cfg.host];
    this._tryHosts(hosts, 0, port);
  }

  /**
   * 依次尝试候选地址，第一个连上的即采用。
   * 与 console-client 同理：引擎只绑定本机某一个网卡地址，
   * 默认的 127.0.0.1 不一定可用。详见 config.js 的说明。
   */
  _tryHosts(hosts, idx, port) {
    if (this.stopped) return;
    if (idx >= hosts.length) {
      console.error(`[monitor] 全部候选地址均不可用: ${hosts.join(', ')}:${port}`);
      this.connected = false;
      this.scheduleReconnect();
      return;
    }

    const host = hosts[idx];
    const sock = net.createConnection({ host, port });
    sock.setNoDelay(true);
    this.socket = sock;

    let settled = false;
    // 同 console-client：候选失败后的 close 不能当作掉线
    let connectedOnce = false;

    const timer = setTimeout(() => {
      if (settled) return;
      settled = true;
      sock.destroy();
      this._tryHosts(hosts, idx + 1, port);
    }, this.cfg.connectTimeout || 5000);

    sock.on('connect', () => {
      if (settled) return;
      settled = true;
      connectedOnce = true;
      clearTimeout(timer);
      this.connected = true;
      this.host = host;
      console.log(`[monitor] 已连接 ${host}:${port}`);
      this.subscribeAll();
    });

    sock.on('data', (d) => {
      const frames = this.parser.push(d.toString('utf8'));
      if (frames.length) {
        this.lastFrameAt = Date.now();
        this.frameCount += frames.length;
        frames.forEach((f) => {
          try {
            this.onFrame(f);
          } catch (e) {
            console.error('[monitor] 帧处理异常:', e.message);
          }
        });
      }
    });

    sock.on('error', (e) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      this.connected = false;
      console.error(`[monitor] ${host}:${port} 连接失败: ${e.message}`);
      this._tryHosts(hosts, idx + 1, port);
    });

    sock.on('close', () => {
      if (!connectedOnce) return; // 候选切换/连接失败，不算掉线
      this.connected = false;
      this.subscribed = [];
      this.scheduleReconnect();
    });
  }

  subscribeAll() {
    for (const mnt of this.cfg.mountpoints || []) {
      this.subscribe(mnt);
    }
  }

  /** 订阅一个挂载点 */
  subscribe(mntpnt) {
    if (!this.connected || !this.socket) return false;
    const cmd = `MONITOR-SOURCE ${this.cfg.password} ${mntpnt}`;
    this.socket.write(cmd + '\r\n');
    if (!this.subscribed.includes(mntpnt)) this.subscribed.push(mntpnt);
    console.log(`[monitor] 已订阅挂载点: ${mntpnt}`);
    return true;
  }

  unsubscribeAll() {
    // 协议未定义退订命令，重新连接即可清空
    if (this.socket) this.socket.destroy();
  }

  scheduleReconnect() {
    if (this.stopped || this.reconnectTimer) return;
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, this.cfg.reconnectDelay || 5000);
  }

  status() {
    return {
      connected: this.connected,
      subscribed: this.subscribed.slice(),
      frameCount: this.frameCount,
      lastFrameAt: this.lastFrameAt,
      staleMs: this.lastFrameAt ? Date.now() - this.lastFrameAt : null
    };
  }

  close() {
    this.stopped = true;
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    if (this.socket) this.socket.destroy();
  }
}

module.exports = MonitorClient;