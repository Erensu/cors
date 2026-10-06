/**
 * 极简 WebSocket 服务端（RFC 6455），零依赖。
 *
 * 只实现大屏需要的能力：握手、文本帧发送、ping/pong、关闭。
 * 不支持分片消息（客户端→服务端方向），前端只用于接收推送。
 */
const crypto = require('crypto');
const { EventEmitter } = require('events');

const GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11';

class WSClient {
  constructor(socket) {
    this.socket = socket;
    this.alive = true;
  }

  /**
   * 发送文本帧；传入对象时自动 JSON 序列化
   */
  send(data) {
    const text = typeof data === 'string' ? data : JSON.stringify(data);
    return this.sendFrame(0x1, Buffer.from(text, 'utf8'));
  }

  sendFrame(opcode, payload) {
    const s = this.socket;
    if (!s || s.destroyed) return false;

    const len = payload.length;
    let header;
    if (len < 126) {
      header = Buffer.alloc(2);
      header[1] = len;
    } else if (len < 65536) {
      header = Buffer.alloc(4);
      header[1] = 126;
      header.writeUInt16BE(len, 2);
    } else {
      header = Buffer.alloc(10);
      header[1] = 127;
      header.writeUInt32BE(0, 2);
      header.writeUInt32BE(len, 6);
    }
    header[0] = 0x80 | opcode; // FIN + opcode

    try {
      s.write(Buffer.concat([header, payload]));
      return true;
    } catch (e) {
      return false;
    }
  }

  ping() {
    this.sendFrame(0x9, Buffer.alloc(0));
  }

  close() {
    try {
      this.sendFrame(0x8, Buffer.alloc(0));
      this.socket.end();
    } catch (e) {
      /* ignore */
    }
  }
}

class WSLite extends EventEmitter {
  constructor(httpServer) {
    super();
    this.clients = new Set();
    httpServer.on('upgrade', (req, socket, head) => this.handleUpgrade(req, socket, head));
  }

  handleUpgrade(req, socket, head) {
    const key = req.headers['sec-websocket-key'];
    if (!key || (req.headers.upgrade || '').toLowerCase() !== 'websocket') {
      socket.destroy();
      return;
    }

    const accept = crypto
      .createHash('sha1')
      .update(key + GUID)
      .digest('base64');

    socket.write(
      'HTTP/1.1 101 Switching Protocols\r\n' +
        'Upgrade: websocket\r\n' +
        'Connection: Upgrade\r\n' +
        `Sec-WebSocket-Accept: ${accept}\r\n\r\n`
    );
    socket.setNoDelay(true);

    const client = new WSClient(socket);
    this.clients.add(client);

    socket.on('data', (buf) => {
      try {
        this.onData(client, buf);
      } catch (e) {
        /* 单个客户端的异常不影响服务 */
      }
    });
    socket.on('close', () => this.clients.delete(client));
    socket.on('error', () => {
      this.clients.delete(client);
      try {
        socket.destroy();
      } catch (e) {
        /* ignore */
      }
    });

    if (head && head.length) {
      try {
        this.onData(client, head);
      } catch (e) {
        /* ignore */
      }
    }

    try {
      this.emit('connection', client);
    } catch (e) {
      /* 回调异常不应影响连接建立 */
    }
  }

  onData(client, buf) {
    // 只处理控制帧与关闭帧；客户端不需要给我们发业务数据
    if (buf.length < 2) return;
    const opcode = buf[0] & 0x0f;

    if (opcode === 0x8) {
      client.close();
      this.clients.delete(client);
    } else if (opcode === 0x9) {
      client.sendFrame(0xa, buf.slice(2)); // pong
    } else if (opcode === 0xa) {
      client.alive = true;
    }
    // 0x1/0x2 文本/二进制：忽略
  }

  broadcast(obj) {
    if (this.clients.size === 0) return;
    const text = typeof obj === 'string' ? obj : JSON.stringify(obj);
    for (const c of this.clients) {
      if (!c.send(text)) this.clients.delete(c);
    }
  }

  close() {
    for (const c of this.clients) c.close();
    this.clients.clear();
  }
}

module.exports = WSLite;