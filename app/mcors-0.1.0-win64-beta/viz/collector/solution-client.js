/**
 * solution.c 推送流客户端
 *
 * 连 src/cors/solution.c 的 solution_svr_thread()（默认 TCP 8080），
 * 被动接收 4 类帧，不做任何命令下发。
 *
 * ── 与 console-client 的关键差异 ──────────────────────────────
 *   console-client: 主动发命令 + 按「静默间隔」切应答（因为引擎不发提示符）
 *   solution-client: 纯被动流。分帧信号是明确的 <<...>>，不需要靠超时猜测，
 *                    因此可以做成「收到多少解析多少」，延迟更低也更可靠。
 *
 * ── 为什么必须多候选地址 ─────────────────────────────────────
 * 与引擎控制台同样的坑：solution.c 用 get_local_ipaddress() 取本机 IP 再
 * uv_tcp_bind()，**只绑那一块网卡**，不绑 127.0.0.1 也不绑 0.0.0.0。
 * 本机多网卡时可能选中任意一块（实测 192.168.83.1），故需依次尝试候选。
 */
const net = require('net');
const { SolutionFrameParser } = require('./solution-parser');

class SolutionClient {
  /**
   * @param {{host?:string, hostCandidates?:string[], port:number,
   *          connectTimeout?:number, reconnectDelay?:number}} cfg
   * @param {(frame:object)=>void} onFrame 每解析出一帧回调
   * @param {(err:Error)=>void} [onError]
   */
  constructor(cfg, onFrame, onError) {
    this.cfg = cfg;
    this.onFrame = onFrame;
    this.onError = onError || (() => {});

    this.socket = null;
    this.connected = false;
    this.host = null;
    this.stopped = false;
    this.reconnectTimer = null;
    this.parser = new SolutionFrameParser();

    this.stats = {
      frames: 0,
      bytes: 0,
      byType: { SRCINFO: 0, NET: 0, BLSOLS: 0, SRCSOLS: 0 },
      badLines: 0,
      lastFrameAt: null
    };
    /** 最近一次收到的帧类型序列，用于验证「4 类帧顺序固定」这一约定 */
    this.lastRound = [];
  }

  connect() {
    if (this.stopped) return;
    const hosts = (this.cfg.hostCandidates && this.cfg.hostCandidates.length)
      ? this.cfg.hostCandidates
      : [this.cfg.host];
    this._tryHosts(hosts, 0, this.cfg.port);
  }

  _tryHosts(hosts, idx, port) {
    if (this.stopped) return;
    if (idx >= hosts.length) {
      this.connected = false;
      this.onError(new Error(
        `solution 全部候选地址均不可用: ${hosts.join(', ')}:${port}`));
      this._scheduleReconnect();
      return;
    }

    const host = hosts[idx];
    const sock = net.createConnection({ host, port });
    sock.setNoDelay(true);
    this.socket = sock;

    let settled = false;
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
      /* 换连接必须清空半截缓冲：否则上一个连接的残帧会与新流拼接，
       * 产生一帧「头在旧连接、尾在新连接」的脏数据。 */
      this.parser.buf = '';
    });

    sock.on('data', (d) => {
      this.stats.bytes += d.length;
      const frames = this.parser.push(d.toString('latin1'));
      for (const f of frames) {
        this.stats.frames++;
        this.stats.byType[f.type] = (this.stats.byType[f.type] || 0) + 1;
        this.stats.lastFrameAt = Date.now();
        /* 记录帧序列：solution_write_thread 每轮固定推 4 帧且顺序不变，
         * 据此可判断「是否收全一轮」，排障时非常有用。 */
        this.lastRound.push(f.type);
        if (this.lastRound.length > 16) this.lastRound.shift();
        try {
          this.onFrame(f);
        } catch (err) {
          this.onError(err);
        }
      }
    });

    sock.on('error', (e) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      this.connected = false;
      this.onError(new Error(`${host}:${port} 连接失败: ${e.message}`));
      this._tryHosts(hosts, idx + 1, port);
    });

    sock.on('close', () => {
      if (!connectedOnce) return;   // 候选切换/失败，不算掉线
      this.connected = false;
      this._scheduleReconnect();
    });
  }

  _scheduleReconnect() {
    if (this.stopped || this.reconnectTimer) return;
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, this.cfg.reconnectDelay || 5000);
  }

  status() {
    return {
      connected: this.connected,
      host: this.host,
      port: this.cfg.port,
      ...this.stats,
      lastRound: this.lastRound.join('>')
    };
  }

  close() {
    this.stopped = true;
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    if (this.socket) this.socket.destroy();
  }
}

module.exports = SolutionClient;
