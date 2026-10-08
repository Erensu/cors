#!/bin/sh
# 离线可用性回归：验证大屏不依赖系统 Node.js 也能起来。
#
# 背景：早期版本要求目标机器先装 Node.js 才能开大屏。现在包内自带
# runtime\node.exe，这个脚本就是防止将来打包时把它漏掉。
#
# 用法（在包根目录执行）：
#   sh tools/check-offline-runtime.sh
#
# 通过标准：包内 node 可执行、能拉起采集层、8181 返回 200、WS 能收到推送。

# 不开 set -e：本脚本所有判定都基于输出内容而非退出码（受限环境下
# 子进程退出码不可靠），失败一律走 fail() 显式退出。

cd "$(dirname "$0")/.."
ROOT=$(pwd)

fail() { echo "[FAIL] $1"; exit 1; }
ok()   { echo "[OK]   $1"; }

# 1) 包内运行时存在且可执行
[ -f "$ROOT/runtime/node.exe" ] || fail "runtime/node.exe 不存在，打包时漏了"
chmod +x "$ROOT/runtime/node.exe" 2>/dev/null || true
NODE="$ROOT/runtime/node.exe"
# 用输出内容判断而非退出码：部分受限/沙箱环境里进程退出码不可靠
# （实测 node.exe 正常打印版本但返回 127）。
VER=$("$NODE" -v 2>/dev/null | tr -d '\r')
case "$VER" in
  v[0-9]*) ok "包内运行时可用：$VER" ;;
  *) fail "包内 node.exe 无法执行（输出='$VER'）" ;;
esac

# 2) 采集层不依赖任何第三方模块（否则内置运行时也跑不起来）
if [ -d "$ROOT/viz/collector/node_modules" ]; then
  fail "采集层出现 node_modules，零依赖前提被破坏"
fi
ok "采集层零第三方依赖"

# 3) 故意清空 PATH，确认启动不依赖系统 Node
PORT=8199
LOG=$(mktemp)
cd "$ROOT/viz"
env -i PATH="" SYSTEMROOT="C:/Windows" windir="C:/Windows" \
    MCORS_SOURCE=mock MCORS_WEB_PORT=$PORT \
    "$NODE" collector/index.js >"$LOG" 2>&1 &
DASH_PID=$!
trap 'kill $DASH_PID 2>/dev/null || true; rm -f "$LOG"' EXIT

# 等端口就绪（最多 20s）
i=0
while [ $i -lt 40 ]; do
  # 同样只看输出：端口通则打印 open
  P=$("$NODE" -e "
    const net=require('net');const s=net.connect($PORT,'127.0.0.1');
    s.on('connect',()=>{s.destroy();console.log('open');process.exit(0)});
    s.on('error',()=>{console.log('closed');process.exit(0)});
    setTimeout(()=>{console.log('closed');process.exit(0)},1000);
  " 2>/dev/null | tr -d '\r')
  [ "$P" = "open" ] && break
  i=$((i+1))
  sleep 0.5
done
[ $i -lt 40 ] || { cat "$LOG"; fail "采集层在无 PATH 环境下未能监听 $PORT"; }
ok "无系统 Node 环境下采集层已启动（端口 $PORT）"

# 4) HTTP 首页可达
CODE=$("$NODE" -e "
  const http=require('http');
  http.get({host:'127.0.0.1',port:$PORT,path:'/'},r=>{
    let n=0;r.on('data',c=>n+=c.length);
    r.on('end',()=>{console.log(r.statusCode+' '+n);process.exit(0)});
  }).on('error',()=>{console.log('0000');process.exit(0)})
    .setTimeout(5000,function(){this.destroy();console.log('0000');process.exit(0)});
" 2>/dev/null | tr -d '\r')
case "$CODE" in
  200*) ok "HTTP 首页正常（$CODE）" ;;
  *)    fail "HTTP 首页返回异常：$CODE" ;;
esac

# 5) WebSocket 握手 + 至少收到一帧推送
WS=$("$NODE" -e "
  const net=require('net'),crypto=require('crypto');
  const key=crypto.randomBytes(16).toString('base64');
  const s=net.connect($PORT,'127.0.0.1',()=>{
    s.write('GET /ws HTTP/1.1\r\nHost: 127.0.0.1:$PORT\r\nUpgrade: websocket\r\n'+
            'Connection: Upgrade\r\nSec-WebSocket-Key: '+key+'\r\nSec-WebSocket-Version: 13\r\n\r\n');
  });
  let hs=false,frames=0,buf=Buffer.alloc(0);
  s.on('data',d=>{
    if(!hs){ if(d.toString('latin1').includes('101')) hs=true; else return; }
    buf=Buffer.concat([buf,d]);
    for(;;){
      if(buf.length<2)return;
      const op=buf[0]&0x0f;let len=buf[1]&0x7f,p=2;
      if(len===126){if(buf.length<4)return;len=buf.readUInt16BE(2);p=4;}
      else if(len===127){if(buf.length<10)return;len=Number(buf.readBigUInt64BE(2));p=10;}
      if(buf.length<p+len)return;
      buf=buf.slice(p+len);
      if(op===1)frames++;
    }
  });
  setTimeout(()=>{console.log((hs?'101':'000')+' '+frames);s.destroy();process.exit(0)},6000);
" 2>/dev/null)
HS=$(echo "$WS" | cut -d' ' -f1)
FRAMES=$(echo "$WS" | cut -d' ' -f2)
[ "$HS" = "101" ] || fail "WebSocket 握手失败（$WS）"
[ "${FRAMES:-0}" -gt 0 ] || fail "WebSocket 未收到推送帧"
ok "WebSocket 握手 101，收到 $FRAMES 帧推送"

# 6) 说明书引用的本地资源必须都在包里
#    踩过的坑：doc\images\ 整个目录没进发布包，说明书 6.1 节的配图
#    变成裂图，但打包流程当时没有任何一环会报错。
MANUAL="$ROOT/doc/使用说明书.html"
if [ -f "$MANUAL" ]; then
  MISSING=$("$NODE" -e "
    const fs=require('fs'),path=require('path');
    const dir=path.dirname(process.argv[1]);
    const h=fs.readFileSync(process.argv[1],'utf8');
    const refs=[...h.matchAll(/(?:src|href)=\"([^\"]+)\"/g)].map(m=>m[1]);
    const local=[...new Set(refs)].filter(r=>!/^(https?:|#|data:|mailto:)/.test(r));
    const bad=local.filter(r=>!fs.existsSync(path.join(dir,r)));
    console.log(bad.join(','));
  " "$MANUAL" 2>/dev/null | tr -d '\r')
  [ -z "$MISSING" ] && ok "说明书本地资源引用完整" \
                  || fail "说明书引用了包内不存在的资源：$MISSING"
else
  echo "[SKIP] 未找到说明书，跳过资源检查"
fi

echo
echo "全部通过：发布包可脱离系统 Node.js 独立运行。"