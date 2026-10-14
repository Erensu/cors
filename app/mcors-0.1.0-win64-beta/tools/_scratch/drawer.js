const net=require('net'),http=require('http'),crypto=require('crypto'),fs=require('fs');
const getJSON=p=>new Promise((res,rej)=>{http.get({host:'127.0.0.1',port:8430,path:p},r=>{let d='';r.on('data',c=>d+=c);r.on('end',()=>{try{res(JSON.parse(d))}catch(e){rej(e)}})}).on('error',rej)});
class WS{constructor(url){const u=new URL(url);this.id=0;this.pending=new Map();this.buf=Buffer.alloc(0);this.hs=false;
 this.sock=net.connect(u.port,u.hostname,()=>{const k=crypto.randomBytes(16).toString('base64');
  this.sock.write('GET '+u.pathname+u.search+' HTTP/1.1\r\nHost: '+u.host+'\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: '+k+'\r\nSec-WebSocket-Version: 13\r\n\r\n')});
 this.sock.on('error',()=>{});
 this.sock.on('data',d=>{if(!this.hs){this.hs=true;const i=d.indexOf('\r\n\r\n');if(i>=0&&i+4<d.length)this._p(d.slice(i+4));return}this._p(d)})}
 _p(buf){this.buf=Buffer.concat([this.buf,buf]);for(;;){if(this.buf.length<2)return;const op=this.buf[0]&0x0f;let len=this.buf[1]&0x7f,p=2;
  if(len===126){if(this.buf.length<4)return;len=this.buf.readUInt16BE(2);p=4}else if(len===127){if(this.buf.length<10)return;len=Number(this.buf.readBigUInt64BE(2));p=10}
  if(this.buf.length<p+len)return;const pl=this.buf.slice(p,p+len);this.buf=this.buf.slice(p+len);
  if(op===1){let m;try{m=JSON.parse(pl.toString('utf8'))}catch(e){continue}
   if(m.id&&this.pending.has(m.id)){this.pending.get(m.id)(m);this.pending.delete(m.id)}}}}
 send(method,params){const id=++this.id;const body=Buffer.from(JSON.stringify({id,method,params:params||{}}));
  const mask=crypto.randomBytes(4);let hdr;
  if(body.length<126){hdr=Buffer.alloc(2);hdr[1]=0x80|body.length}
  else if(body.length<65536){hdr=Buffer.alloc(4);hdr[1]=0x80|126;hdr.writeUInt16BE(body.length,2)}
  else{hdr=Buffer.alloc(10);hdr[1]=0x80|127;hdr.writeBigUInt64BE(BigInt(body.length),2)}
  hdr[0]=0x81;const m2=Buffer.from(body);for(let i=0;i<m2.length;i++)m2[i]^=mask[i%4];
  this.sock.write(Buffer.concat([hdr,mask,m2]));return new Promise(r=>this.pending.set(id,r))}}
(async()=>{
 const list=await getJSON('/json/list');
 const tg=list.find(x=>x.type==='page'&&x.webSocketDebuggerUrl);
 const ws=new WS(tg.webSocketDebuggerUrl);
 await new Promise(r=>setTimeout(r,700));
 await ws.send('Runtime.enable');await ws.send('Page.enable');
 await ws.send('Emulation.setDeviceMetricsOverride',{width:1400,height:1300,deviceScaleFactor:1,mobile:false});
 await ws.send('Page.navigate',{url:'http://127.0.0.1:8190/'});
 await new Promise(r=>setTimeout(r,13000));
 const ev=async e=>{const r=await ws.send('Runtime.evaluate',{expression:e,returnByValue:true});
  return r.result&&r.result.result?r.result.result.value:undefined};
 // 点开基线行，展开抽屉
 const opened = await ev(`(()=>{const tr=document.querySelector('#blTable tbody tr');if(!tr)return 'no row';tr.click();return 'clicked'})()`);
 console.log('抽屉打开:', opened);
 await new Promise(r=>setTimeout(r,1500));
 const info = await ev(`(()=>{const d=document.getElementById('blDrawer');
   if(!d)return {err:'no drawer'};
   return {hidden: d.hidden, sub: (document.getElementById('blDrawerSub')||{}).textContent,
           bodyLen: (document.getElementById('blDrawerBody')||{}).innerHTML?.length||0}})()`);
 console.log('抽屉状态:', JSON.stringify(info));
 // 抽屉内中文残留
 const zh = await ev(`(()=>{const d=document.getElementById('blDrawer');if(!d)return [];
   const o=[];d.querySelectorAll('*').forEach(el=>{
     if(el.children.length)return; const t=(el.textContent||'').trim();
     if(/[\\u4e00-\\u9fff]/.test(t)) o.push(t.slice(0,44));
     ['title'].forEach(a=>{const v=el.getAttribute&&el.getAttribute(a);
       if(v&&/[\\u4e00-\\u9fff]/.test(v)) o.push('title: '+v.slice(0,40))})});
   return [...new Set(o)]})()`);
 console.log('抽屉内中文残留:', (zh||[]).length, '处');
 (zh||[]).forEach(x=>console.log('   ', x));
 // 截图
 const sh=await ws.send('Page.captureScreenshot',{format:'png'});
 fs.writeFileSync(process.argv[2]+'/drawer.png',Buffer.from(sh.result.data,'base64'));
 console.log('截图已保存');
 process.exit(0);
})().catch(e=>{console.error('ERR',e);process.exit(1)});
