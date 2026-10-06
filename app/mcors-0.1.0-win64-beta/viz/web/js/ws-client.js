/**
 * 前端 WebSocket 客户端
 * 负责接收采集层推送的快照，并提供 REST 调用封装。
 */
(function (global) {
  'use strict';

  class Api {
    constructor() {
      this.ws = null;
      this.listeners = [];
      this.retry = 0;
      this.connected = false;
      this.onStatusChange = null;
    }

    connect() {
      const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
      const url = `${proto}//${location.host}/ws`;

      try {
        this.ws = new WebSocket(url);
      } catch (e) {
        this.scheduleRetry();
        return;
      }

      this.ws.onopen = () => {
        this.retry = 0;
        this.setConnected(true);
      };

      this.ws.onmessage = (ev) => {
        let data;
        try {
          data = JSON.parse(ev.data);
        } catch (e) {
          return;
        }
        this.listeners.forEach((fn) => {
          try {
            fn(data);
          } catch (e) {
            console.error('[ws] 监听器异常', e);
          }
        });
      };

      this.ws.onclose = () => {
        this.setConnected(false);
        this.scheduleRetry();
      };

      this.ws.onerror = () => {
        // onclose 会紧随，这里不重复处理
      };
    }

    setConnected(v) {
      if (this.connected === v) return;
      this.connected = v;
      if (this.onStatusChange) this.onStatusChange(v);
    }

    scheduleRetry() {
      this.retry++;
      const delay = Math.min(1000 * this.retry, 8000);
      setTimeout(() => this.connect(), delay);
    }

    onSnapshot(fn) {
      this.listeners.push(fn);
    }
  }

  /** REST 调用封装 */
  const rest = {
    async get(url) {
      const r = await fetch(url);
      return r.json();
    },
    async post(url, body) {
      const r = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body || {})
      });
      return r.json();
    }
  };

  global.MCORS = global.MCORS || {};
  global.MCORS.Api = Api;
  global.MCORS.rest = rest;
})(window);