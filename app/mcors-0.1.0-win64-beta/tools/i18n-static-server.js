/**
 * 双语回归用的临时静态服务（check-i18n.sh 调用）
 *
 * 只为让浏览器能以 http:// 打开说明书与大屏。生产环境不需要它——
 * 大屏由采集层 collector/index.js 提供 HTTP 服务（端口 8181），
 * 说明书直接双击用 file:// 打开即可。
 *
 * 用法：node i18n-static-server.js <端口>
 *   /web/*→ viz/web      /web/        → index.html
 *   /doc/*→ doc/         /doc/        → manual.html
 */
const http = require('http');
const fs = require('fs');
const path = require('path');

const ROOT = path.resolve(__dirname, '..');
const PORT = Number(process.argv[2]) || 8299;

const ROOTS = {
  '/web': path.join(ROOT, 'viz', 'web'),
  '/doc': path.join(ROOT, 'doc')
};
const INDEX = {
  '/web': 'index.html',
  '/doc': 'manual.html'
};
const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.svg': 'image/svg+xml',
  '.json': 'application/json; charset=utf-8'
};

http.createServer((req, res) => {
  let url;
  try {
    url = decodeURIComponent(req.url.split('?')[0]);
  } catch (e) {
    res.writeHead(400);
    return res.end('bad url');
  }
  const seg = url.split('/').filter(Boolean);
  const base = ROOTS['/' + seg[0]];
  if (!base) {
    res.writeHead(404);
    return res.end('no such root');
  }
  let file = path.join(base, seg.slice(1).join('/'));
  // 防目录穿越
  if (!file.startsWith(base)) {
    res.writeHead(403);
    return res.end('forbidden');
  }
  if (!fs.existsSync(file) || fs.statSync(file).isDirectory()) {
    file = path.join(base, INDEX['/' + seg[0]]);
  }
  if (!fs.existsSync(file)) {
    res.writeHead(404);
    return res.end('not found');
  }
  res.writeHead(200, {
    'Content-Type': MIME[path.extname(file)] || 'application/octet-stream',
    'Cache-Control': 'no-store'
  });
  fs.createReadStream(file).pipe(res);
}).listen(PORT, '127.0.0.1', () => {
  console.log('i18n static server listening on http://127.0.0.1:' + PORT);
});
