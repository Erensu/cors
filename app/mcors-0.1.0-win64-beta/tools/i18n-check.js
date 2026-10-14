/**
 * 中英文切换 · 一体化回归检查
 *
 * 为什么做成单个 node 脚本而不是 shell 编排：
 *   受限 shell 里后台进程会随命令结束被回收，导致「起了服务、
 *   还没访问就被杀掉」。把服务、浏览器、检查全放进同一个进程，
 *   由它负责拉起与回收，就没这个问题了。
 *
 * 用法：node tools/i18n-check.js
 * 退出码仅作参考，以stdout 的 [OK]/[FAIL] 标记为准
 *（本沙箱里子进程退出码不可靠，见 MEMORY.md）。
 */
const http = require('http');
const fs = require('fs');
const os = require('os');
const path = require('path');
const net = require('net');
const crypto = require('crypto');
const { spawn, spawnSync } = require('child_process');

const ROOT = path.resolve(__dirname, '..');
const SITE_PORT = 8391;
const CDP_PORT = 8399;

const CHROME_CANDIDATES = [
  'C:/Program Files/Google/Chrome/Application/chrome.exe',
  'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe'
];

let failed = 0;
const ok = (m) => console.log('[OK]   ' + m);
const bad = (m) => { console.error('[FAIL] ' + m); failed++; };

/* ─────────────────静态服务 ───────────────── */
const ROOTS = {
  '/web': path.join(ROOT, 'viz', 'web'),
  '/doc': path.join(ROOT, 'doc')
};
const INDEX = { '/web': 'index.html', '/doc': 'manual.html' };
const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.png': 'image/png',
  '.json': 'application/json; charset=utf-8'
};

function startServer() {
  return new Promise((resolve, reject) => {
    const srv = http.createServer((req, res) => {
      let url;
      try { url = decodeURIComponent(req.url.split('?')[0]); }
      catch (e) { res.writeHead(400); return res.end('bad'); }
      const seg = url.split('/').filter(Boolean);
      const base = ROOTS['/' + seg[0]];
      if (!base) { res.writeHead(404); return res.end('no root'); }
      let f = path.join(base, seg.slice(1).join('/'));
      if (!f.startsWith(base)) { res.writeHead(403); return res.end('forbidden'); }
      if (!fs.existsSync(f) || fs.statSync(f).isDirectory()) {
        f = path.join(base, INDEX['/' + seg[0]]);
      }
      if (!fs.existsSync(f)) { res.writeHead(404); return res.end('nf'); }
      res.writeHead(200, {
        'Content-Type': MIME[path.extname(f)] || 'application/octet-stream',
        'Cache-Control': 'no-store'
      });
      fs.createReadStream(f).pipe(res);
    });
    srv.on('error', reject);
    srv.listen(SITE_PORT, '127.0.0.1', () => resolve(srv));
  });
}

/* ───────────────── 最小 CDP 客户端 ───────────────── */
class WS {
  constructor(url) {
    const u = new URL(url);
    this.id = 0;
    this.pending = new Map();
    this.buf = Buffer.alloc(0);
    this.hs = false;
    this.sock = net.connect(u.port, u.hostname, () => {
      const key = crypto.randomBytes(16).toString('base64');
      this.sock.write(
        `GET ${u.pathname}${u.search} HTTP/1.1\r\nHost: ${u.host}\r\n` +
        `Upgrade: websocket\r\nConnection: Upgrade\r\n` +
        `Sec-WebSocket-Key: ${key}\r\nSec-WebSocket-Version: 13\r\n\r\n`);
    });
    this.sock.on('error', () => { /* 浏览器被回收时 socket 会断，忽略 */ });
    this.sock.on('data', (d) => {
      if (!this.hs) {
        if (!d.toString('latin1').includes('101')) throw new Error('CDP 握手失败');
        this.hs = true;
        const i = d.indexOf('\r\n\r\n');
        if (i >= 0 && i + 4 < d.length) this._parse(d.slice(i + 4));
        return;
      }
      this._parse(d);
    });
  }
  _parse(buf) {
    this.buf = Buffer.concat([this.buf, buf]);
    for (;;) {
      if (this.buf.length < 2) return;
      const op = this.buf[0] & 0x0f;
      let len = this.buf[1] & 0x7f, p = 2;
      if (len === 126) { if (this.buf.length < 4) return; len = this.buf.readUInt16BE(2); p = 4; }
      else if (len === 127) { if (this.buf.length < 10) return; len = Number(this.buf.readBigUInt64BE(2)); p = 10; }
      if (this.buf.length < p + len) return;
      const pl = this.buf.slice(p, p + len);
      this.buf = this.buf.slice(p + len);
      if (op !== 1) continue;
      let m; try { m = JSON.parse(pl.toString('utf8')); } catch (e) { continue; }
      if (m.id && this.pending.has(m.id)) {
        this.pending.get(m.id)(m);
        this.pending.delete(m.id);
      }
    }
  }
  send(method, params) {
    const id = ++this.id;
    const body = Buffer.from(JSON.stringify({ id, method, params: params || {} }), 'utf8');
    const mask = crypto.randomBytes(4);
    let hdr;
    if (body.length < 126) { hdr = Buffer.alloc(2); hdr[1] = 0x80 | body.length; }
    else if (body.length < 65536) { hdr = Buffer.alloc(4); hdr[1] = 0x80 | 126; hdr.writeUInt16BE(body.length, 2); }
    else { hdr = Buffer.alloc(10); hdr[1] = 0x80 | 127; hdr.writeBigUInt64BE(BigInt(body.length), 2); }
    hdr[0] = 0x81;
    const masked = Buffer.from(body);
    for (let i = 0; i < masked.length; i++) masked[i] ^= mask[i % 4];
    this.sock.write(Buffer.concat([hdr, mask, masked]));
    return new Promise((r) => this.pending.set(id, r));
  }
}

const getJSON = (port, p) => new Promise((res, rej) => {
  http.get({ host: '127.0.0.1', port, path: p }, (r) => {
    let d = '';
    r.on('data', (c) => (d += c));
    r.on('end', () => { try { res(JSON.parse(d)); } catch (e) { rej(e); } });
  }).on('error', rej);
});

function waitPort(port, timeoutMs) {
  const deadline = Date.now() + timeoutMs;
  return new Promise((resolve) => {
    (function attempt() {
      const s = net.connect(port, '127.0.0.1');
      s.on('connect', () => { s.destroy(); resolve(true); });
      s.on('error', () => {
        s.destroy();
        if (Date.now() > deadline) return resolve(false);
        setTimeout(attempt, 400);
      });
    })();
  });
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/* 统计页面上残留的中文（注入页面执行，故需双反斜杠） */
const SCAN_ZH = `(() => {
  const out = [];
  document.querySelectorAll('body *').forEach((el) => {
    if (el.children.length) return;
    const t = (el.textContent || '').trim();
    if (/[\\u4e00-\\u9fff]/.test(t)) out.push(t.slice(0, 40));
  });
  return [...new Set(out)];
})()`;
const HAS_ZH = /[一-鿿]/;

/* ───────────────── 主流程 ───────────────── */
(async () => {
  console.log('=== 静态检查 ===');
  const manualEn = path.join(ROOT, 'doc', 'manual.html');
  const manualZh = path.join(ROOT, 'doc', 'manual.zh.html');
  const dashHtml = path.join(ROOT, 'viz', 'web', 'index.html');
  const i18nJs = path.join(ROOT, 'viz', 'web', 'js', 'i18n.js');

  /* 说明书现在是两份独立文档：主文件英文、.zh.html 中文，都不带切换。
     切换只保留在大屏（那是应用，需要就地切语言）。 */
  if (!fs.existsSync(manualEn)) bad('缺少英文主说明书 doc/manual.html');
  else {
    const h = fs.readFileSync(manualEn, 'utf8');
    h.includes('<html lang="en">') ? ok('英文主说明书 lang=en')
      : bad('英文主说明书 lang 不是 en');
    h.includes('User Manual') ? ok('英文主说明书标题已英文化')
      : bad('英文主说明书标题异常');
  }
  if (!fs.existsSync(manualZh)) bad('缺少中文对照说明书 doc/manual.zh.html');
  else {
    const h = fs.readFileSync(manualZh, 'utf8');
    h.includes('lang="zh-CN"') ? ok('中文说明书 lang=zh-CN')
      : bad('中文说明书 lang 不是 zh-CN');
  }

  const dhtml = fs.readFileSync(dashHtml, 'utf8');
  dhtml.includes('id="lang-toggle"') ? ok('大屏含切换按钮')
    : bad('大屏缺切换按钮');
  dhtml.includes('js/i18n.js') ? ok('大屏已引用 i18n.js')
    : bad('大屏未引用 i18n.js');

  const jsFiles = ['i18n', 'theme', 'station-panel', 'baseline-panel',
                   'skyplot', 'network-map', 'sat-delay-trend', 'trend',
                   'main', 'ws-client'];
  let syntaxBad = 0;
  for (const f of jsFiles) {
    const r = spawnSync(process.execPath,
      ['--check', path.join(ROOT, 'viz', 'web', 'js', f + '.js')],
      { encoding: 'utf8' });
    const out = ((r.stdout || '') + (r.stderr || '')).trim();
    if (out) { bad(f + '.js 语法错误：' + out.slice(0, 120)); syntaxBad++; }
  }
  if (!syntaxBad) ok('大屏 ' + jsFiles.length + ' 个 JS 文件语法正常');

  /* 起服务 */
  console.log('=== 启动测试环境 ===');
  let srv;
  try {
    srv = await startServer();
    ok('静态服务已就绪（端口 ' + SITE_PORT + '）');
  } catch (e) {
    bad('静态服务启动失败：' + e.message);
    console.log('失败 ' + failed + ' 项');
    return;
  }

  const chrome = CHROME_CANDIDATES.find((p) => fs.existsSync(p));
  if (!chrome) {
    bad('未找到 Chrome/Edge，无法实测');
    srv.close();
    return;
  }
  /* profile 放系统临时目录，不要写进发布包（每个 profile 约 20MB，
     残留几轮就会把包撑大到几十 MB） */
  const profile = path.join(os.tmpdir(),
    'mcors-i18n-check-' + process.pid);
  const browser = spawn(chrome, [
    '--headless=new', '--disable-gpu',
    '--remote-debugging-port=' + CDP_PORT,
    '--user-data-dir=' + profile, 'about:blank'
  ], { stdio: 'ignore' });

  const cleanup = () => {
    try { browser.kill(); } catch (e) {}
    try { srv.close(); } catch (e) {}
    try { fs.rmSync(profile, { recursive: true, force: true }); } catch (e) {}
  };
  process.on('exit', cleanup);

  if (!await waitPort(CDP_PORT, 15000)) {
    bad('Chrome 调试端口未就绪');
    cleanup();
    return;
  }
  ok('浏览器已就绪');

  const list = await getJSON(CDP_PORT, '/json/list');
  const target = list.find((t) => t.type === 'page' && t.webSocketDebuggerUrl);
  if (!target) { bad('没有可用 CDP target'); cleanup(); return; }

  const ws = new WS(target.webSocketDebuggerUrl);
  await sleep(600);
  await ws.send('Runtime.enable');
  await ws.send('Page.enable');
  const ev = async (expr) => {
    const r = await ws.send('Runtime.evaluate',
      { expression: expr, returnByValue: true });
    return r.result && r.result.result ? r.result.result.value : undefined;
  };

  /* ── 英文版说明书（主文件）── */
  console.log('=== doc/manual.html（英文 · 主文件）===');
  await ws.send('Page.navigate', { url: `http://127.0.0.1:${SITE_PORT}/doc/` });
  await sleep(3500);

  (await ev('document.documentElement.lang')) === 'en'
    ? ok('lang 属性为 en') : bad('lang 属性应为 en');

  const enTitle = await ev('document.title');
  HAS_ZH.test(enTitle)
    ? bad('标题仍含中文：' + enTitle) : ok('标题：' + enTitle);

  const enToc = await ev('document.querySelector("nav.toc li a").textContent.trim()');
  HAS_ZH.test(enToc)
    ? bad('目录仍含中文：' + enToc) : ok('目录已英文化（首项：' + enToc + '）');

  /* 整份英文文档必须零中文 —— 代码块注释、CSS 注释、alt/title 属性
     也都要求翻译，故这里不再排除 pre/code/style。 */
  const enLeft = await ev(`(() => {
    const out = [];
    document.querySelectorAll('body *').forEach((el) => {
      if (el.children.length) return;
      const t = (el.textContent || '').trim();
      if (/[\\u4e00-\\u9fff]/.test(t)) out.push(t.slice(0, 40));
    });
    return [...new Set(out)];
  })()`);
  (enLeft || []).length
    ? bad('英文文档仍有中文 ' + enLeft.length + ' 处：' + enLeft.slice(0, 6).join(' | '))
    : ok('全文档零中文（含代码块与注释）');

  /* 静态复查：整份 HTML 源码（含 <style> 与属性）都不该有中文 */
  const raw = fs.readFileSync(manualEn, 'utf8');
  const rawZh = (raw.match(/[\u4e00-\u9fff]/g) || []).length;
  rawZh === 0 ? ok('源码零中文（含 style 与属性）')
              : bad('源码仍有 ' + rawZh + ' 个中文字符');

  /* ── 中文版说明书（对照）── */
  console.log('=== doc/manual.zh.html（中文 · 对照）===');
  await ws.send('Page.navigate',
    { url: `http://127.0.0.1:${SITE_PORT}/doc/manual.zh.html` });
  await sleep(3000);

  (await ev('document.documentElement.lang')) === 'zh-CN'
    ? ok('lang 属性为 zh-CN') : bad('中文版 lang 属性应为 zh-CN');

  const zhToc = await ev('document.querySelector("nav.toc li a").textContent.trim()');
  HAS_ZH.test(zhToc)
    ? ok('目录为中文（首项：' + zhToc + '）') : bad('中文版目录异常：' + zhToc);


  console.log('=== 大屏 viz/web/ ===');
  await ws.send('Page.navigate', { url: `http://127.0.0.1:${SITE_PORT}/web/` });
  await sleep(6000);

  (await ev('typeof MCORS !== "undefined" && typeof MCORS.i18n === "object"'))
    ? ok('MCORS.i18n 已初始化') : bad('MCORS.i18n 未初始化');

  /* 默认语言必须是英文：清掉 localStorage 后重载，模拟新用户首次访问 */
  await ev('try{localStorage.removeItem("mcors.lang")}catch(e){}');
  await ws.send('Page.reload', { ignoreCache: true });
  await sleep(6500);
  const defLang = await ev('window.MCORS && MCORS.i18n ? MCORS.i18n.getLang() : null');
  defLang === 'en'
    ? ok('默认语言为英文（无历史选择时）')
    : bad('默认语言应为英文，实际 ' + defLang);
  const defH1 = await ev(
    '(document.querySelector("[data-i18n]")||{}).textContent || ""');
  HAS_ZH.test(defH1)
    ? bad('英文默认态下仍见中文文案：' + defH1)
    : ok('默认态界面文案已英文化');

  await ev('MCORS.i18n.setLang("zh")');
  await sleep(700);
  await ev('document.getElementById("lang-toggle").click()');
  await sleep(1500);

  (await ev('MCORS.i18n.getLang()')) === 'en'
    ? ok('点击按钮切换生效') : bad('点击按钮未切到英文');

  const label = await ev('document.getElementById("lang-toggle").textContent');
  label === '中文' ? ok('按钮文案更新为「中文」')
    : bad('英文态按钮文案应为「中文」，实际「' + label + '」');

  const dZh = await ev(SCAN_ZH) || [];
  /* 大屏文案已全覆盖，仅允许切换按钮自身的「中文」二字 */
  dZh.length > 3
    ? bad('大屏英文态残留中文 ' + dZh.length + ' 处：' + dZh.slice(0, 6).join(' | '))
    : ok('英文态残留 ' + dZh.length + ' 处（切换按钮自身）');

  /* 属性类文案单独扫一遍：title / placeholder / aria-label 不参与
     textContent 遍历，只有悬停才看得到，曾整批漏掉 12 条
     （index.html 的 data-i18n-title）。 */
  const attrZh = await ev(`(() => {
    const out = [];
    document.querySelectorAll('*').forEach((el) => {
      ['title', 'placeholder', 'aria-label'].forEach((a) => {
        let v = null;
        try { v = el.getAttribute(a); } catch (e) {}
        if (v && /[\\u4e00-\\u9fff]/.test(v)) out.push(a + ': ' + v.slice(0, 45));
      });
    });
    return [...new Set(out)];
  })()`);
  (attrZh || []).length
    ? bad('英文态属性残留中文 ' + attrZh.length + ' 处：' + attrZh.slice(0, 5).join(' | '))
    : ok('属性类文案零中文（title / placeholder / aria-label）');

  /* 基线详情抽屉是**隐藏区域**（点行才展开），常规扫描看不到，
     曾整块漏翻（STATUS / PER-SATELLITE DD DELAYS / Note 等十余条）。
     这里主动点开首行再扫一遍。 */
  const drawerOpened = await ev(`(() => {
    const tr = document.querySelector('#blTable tbody tr');
    /* 空态占位行是 <tr class="empty">，点它不会展开抽屉。
       静态服务下没有 WebSocket 数据，走的正是这条分支。 */
    if (!tr || tr.classList.contains('empty')) return false;
    tr.click();
    return true;
  })()`);
  if (!drawerOpened) {
    console.log('[SKIP] 无基线行，跳过抽屉检查');
  } else {
    await sleep(1200);
    const dHtml = await ev(`(() => {
      const d = document.getElementById('blDrawer');
      return d && !d.hidden ? d.innerHTML : null;
    })()`);
    if (!dHtml) {
      bad('抽屉未能展开');
    } else {
      const dz = await ev(`(() => {
        const d = document.getElementById('blDrawer');
        const out = [];
        d.querySelectorAll('*').forEach((el) => {
          if (el.children.length) return;
          const t = (el.textContent || '').trim();
          if (/[\\u4e00-\\u9fff]/.test(t)) out.push(t.slice(0, 40));
          ['title'].forEach((a) => {
            const v = el.getAttribute && el.getAttribute(a);
            if (v && /[\\u4e00-\\u9fff]/.test(v)) out.push('title: ' + v.slice(0, 36));
          });
        });
        return [...new Set(out)];
      })()`);
      (dz || []).length
        ? bad('抽屉内残留中文 ' + dz.length + ' 处：' + dz.slice(0, 5).join(' | '))
        : ok('基线详情抽屉零中文（含逐星表与说明）');
      /* 收起抽屉，避免影响后续检查 */
      await ev('document.getElementById("blDrawerClose") && document.getElementById("blDrawerClose").click()');
      await sleep(400);
    }
  }

  /* 静态复查：index.html 里所有 data-i18n* 的值都要在语言包里有译文 */
  const htmlRaw = fs.readFileSync(dashHtml, 'utf8');
  const i18nJsRaw = fs.readFileSync(i18nJs, 'utf8');
  const di = i18nJsRaw.indexOf('const DICT = {');
  const dj = i18nJsRaw.indexOf('\n  };', di);
  const dictKeys = new Set(
    (i18nJsRaw.slice(di, dj).match(/^\s*'((?:[^'\\]|\\.)*)'\s*:/gm) || [])
      .map((x) => x.trim().replace(/^'|':$/g, '')));
  const missingAttr = [];
  ['data-i18n', 'data-i18n-title', 'data-i18n-ph', 'data-i18n-aria'].forEach((attr) => {
    const re = new RegExp(attr + '="([^"]+)"', 'g');
    let m;
    while ((m = re.exec(htmlRaw))) {
      const v = m[1];
      if (/[\u4e00-\u9fff]/.test(v) && !dictKeys.has(v)) missingAttr.push(attr + '=' + v.slice(0, 35));
    }
  });
  missingAttr.length
    ? bad('index.html 有 ' + missingAttr.length + ' 条标注缺译文：' + missingAttr.slice(0, 4).join(' | '))
    : ok('index.html 全部标注均有译文');

  cleanup();
  console.log('');
  console.log(failed
    ? `检查未通过：${failed} 项失败`
    : '全部通过：中英文切换功能正常。');
})().catch((e) => {
  console.error('[FAIL] 执行异常', e);
  process.exit(1);
});
