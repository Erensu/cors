/**
 * cors-engine 控制台客户端
 *
 * engine.c 的 svr_open(con, port) 监听 TCP（默认 9000），on_conn 接受连接后
 * 由 on_read_cb → cmdprc() 处理命令文本。
 *
 * ── 应答边界：为什么不能用提示符 ──────────────────────────────
 * 引擎的提示符 CMDPROMPT 只在 con_thread() 里发给**本地控制台**
 * （engine.c:1068 的 vt_puts(con->vt, CMDPROMPT)），走的是 vt/终端通道。
 * 而 TCP 客户端收到的是 printout() 的广播（engine.c:83-89），它只推命令
 * 执行结果，**不下发提示符**；on_conn 里也没有任何初始 greet。
 *
 * 所以按 "cors-engine> " 切分应答，在 mock 下能跑、在真引擎上会 100% 超时。
 * 正确做法是按**静默间隔**判定：引擎处理完一条命令后不再有输出，
 * 因此「连续 quietMs 毫秒没有新字节」即应答结束。
 *
 * 另注：命令匹配用 strstr(cmds[i], args[0])，是"命令表项包含输入串"即命中，
 * 输入前缀有误命中风险，因此这里统一发完整命令名。
 */
const net = require('net');

class ConsoleClient {
  constructor(cfg) {
    this.cfg = cfg;
    this.socket = null;
    this.connected = false;
    this.buffer = '';
    this.waiters = [];
    this.reconnectTimer = null;
    this.stopped = false;
    this.quietTimer = null;
    // send() 串行链的尾节点，见 send() 注释
    this.tail = Promise.resolve();
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
   *
   * ⚠ 为什么需要多候选：引擎只绑定 get_local_ipaddress() 选出的那一个
   *   地址（engine.c:1231），本机有多块网卡时它可能选中任意一块，
   *   默认的 127.0.0.1 会直接 ECONNREFUSED。详见 config.js 的说明。
   */
  _tryHosts(hosts, idx, port) {
    if (this.stopped) return;
    if (idx >= hosts.length) {
      console.error(`[console] 全部候选地址均不可用: ${hosts.join(', ')}:${port}`);
      this.connected = false;
      this.scheduleReconnect();
      return;
    }

    const host = hosts[idx];
    const sock = net.createConnection({ host, port });
    sock.setNoDelay(true);
    this.socket = sock;

    let settled = false;
    // 只有真正建立过连接，之后的 close 才算"掉线"。
    // 不能用 settled 判断：候选失败时 error 会把 settled 置真，
    // 紧接着 close 也会触发，若据此调 scheduleReconnect()，
    // 就会与 _tryHosts 的下一个候选并发，造成候选扫掠反复空转。
    let connectedOnce = false;

    const timer = setTimeout(() => {
      if (settled) return;
      settled = true;
      sock.destroy();
      this._tryHosts(hosts, idx + 1, port);
    }, this.cfg.connectTimeout);

    sock.on('connect', () => {
      if (settled) return;
      settled = true;
      connectedOnce = true;
      clearTimeout(timer);
      this.connected = true;
      this.host = host;
      console.log(`[console] 已连接 ${host}:${port}`);
    });

    sock.on('data', (d) => this.onData(d));

    sock.on('error', (e) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      this.connected = false;
      console.error(`[console] ${host}:${port} 连接失败: ${e.message}`);
      this._tryHosts(hosts, idx + 1, port);
    });

    sock.on('close', () => {
      if (!connectedOnce) return; // 候选切换/连接失败，不算掉线
      this.connected = false;
      this.failAllWaiters(new Error('console 连接已断开'));
      this.scheduleReconnect();
    });
  }

  scheduleReconnect() {
    if (this.stopped || this.reconnectTimer) return;
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, this.cfg.reconnectDelay || 5000);
  }

  onData(chunk) {
    this.buffer += chunk.toString('latin1');
    // 异常保护：单次应答过大则丢弃
    if (this.buffer.length > 1024 * 512) {
      this.buffer = '';
      return;
    }
    this.armQuietTimer();
  }

  /**
   * 静默定时器：连续 quietMs 无新字节 → 当前命令应答结束
   *
   * 这是 TCP 通道唯一可靠的结束信号（引擎不发提示符，见文件头说明）。
   * 由 send() 在**写命令时**启动，而非等收到数据才启动 —— 因为引擎对
   * 某些命令（如 addsource）正常返回时就是零输出，此时若不启动定时器，
   * waiter 会一直挂到 replyTimeout 才返回，轮询会被拖垮。
   */
  armQuietTimer() {
    if (this.quietTimer) clearTimeout(this.quietTimer);
    this.quietTimer = setTimeout(() => this.flush(), this.cfg.quietMs || 350);
  }

  /**
   * 结束当前命令的应答。
   *
   * 注意：即使 buffer 为空也要 resolve 队首 waiter —— 零输出是合法应答
   * （引擎的 addsource / rtkpos -add 等命令成功时不打印任何内容）。
   */
  flush() {
    if (this.quietTimer) {
      clearTimeout(this.quietTimer);
      this.quietTimer = null;
    }
    const out = this.buffer;
    this.buffer = '';
    const waiter = this.waiters.shift();
    if (waiter) waiter.resolve(this.clean(out));
  }

  clean(text) {
    // 去掉回车与可能的 ANSI 控制序列
    return text.replace(/\r/g, '').replace(/\x1b\[[0-9;]*[A-Za-z]/g, '');
  }

  failAllWaiters(err) {
    if (this.quietTimer) {
      clearTimeout(this.quietTimer);
      this.quietTimer = null;
    }
    this.buffer = '';
    const ws = this.waiters.splice(0);
    ws.forEach((w) => w.reject(err));
  }

  /**
   * 下发一条命令并等待应答
   *
   * 串行化：引擎的 printout() 把结果**广播给所有已连客户端**
   * （engine.c:83-89 的 for 循环遍历 cli[]），所以若本进程并发发两条命令，
   * 两条的输出会串在一起、无法切分。这里用 tail 链强制排队。
   *
   * @returns {Promise<string>} 命令输出文本
   */
  send(command, timeout) {
    const run = () => this._sendNow(command, timeout);
    // 挂在上一条之后，保证同一时刻只有一条命令在途
    const next = this.tail.then(run, run);
    this.tail = next.catch(() => {});
    return next;
  }

  _sendNow(command, timeout) {
    const tmo = timeout || this.cfg.replyTimeout;
    if (!this.connected || !this.socket) {
      return Promise.reject(new Error('console 未连接'));
    }
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        const i = this.waiters.findIndex((w) => w.timer === timer);
        if (i >= 0) this.waiters.splice(i, 1);
        // 超时也要清空缓冲，否则残留会串到下一条命令
        this.buffer = '';
        resolve('(超时未收到应答)');
      }, tmo);

      this.waiters.push({
        timer,
        resolve: (v) => {
          clearTimeout(timer);
          resolve(v);
        },
        reject: (e) => {
          clearTimeout(timer);
          reject(e);
        }
      });
      this.socket.write(command + '\r\n');
      // 写命令即启动静默定时器：引擎零输出时也要能正常结束本次应答
      this.armQuietTimer();
    });
  }

  /** 不关心应答，仅确保命令送达 */
  fire(command) {
    if (!this.connected || !this.socket) return false;
    this.socket.write(command + '\r\n');
    return true;
  }

  close() {
    this.stopped = true;
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    if (this.socket) this.socket.destroy();
  }
}

module.exports = ConsoleClient;