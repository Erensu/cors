// 中英双语验证：分别在 zh / en 态截图，并统计残留中文
const net=require('net'),http=require('http'),crypto=require('crypto'),fs=require('fs');
const PORT=9226;
const getJSON=p=>new Promise((res,rej)=>{http.get({host:'127.0.0.1',port:PORT,path:p},r=>{let d='';r.on('data',c=>d+=c);r.on('end',()=>{try{res(JSON.parse(d))}catch(e){rej(e)}})}).on('error',rej)});
class WS{constructor(url){const u=new URL(url);this.id=0;this.pending=new Map();this.buf=Buffer.alloc(0);this.hs=false;
 this.sock=net.connect(u.port,u.hostname,()=>{const k=crypto.randomBytes(16).toString('base64');
  this.sock.write(`GET ${u.pathname}${u.search} HTTP/1.1\r\nHost: ${u.host}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: ${k}\r\nSec-WebSocket-Version: 13\r\n\r\n`)});
 this.sock.on('data',d=>{if(!this.hs){if(!d.toString('latin1').includes('101')){console.error('hs fail');process.exit(1)}this.hs=true;const i=d.indexOf('\r\n\r\n');if(i>=0&&i+4<d.length)this._p(d.slice(i+4));return}this._p(d)})}
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
 const dir=process.argv[2];
 const list=await getJSON('/json/list');
 const t=list.find(x=>x.type==='page'&&x.webSocketDebuggerUrl);
 const ws=new WS(t.webSocketDebuggerUrl);
 await new Promise(r=>setTimeout(r,700));
 await ws.send('Page.enable');await ws.send('Runtime.enable');
 await ws.send('Emulation.setDeviceMetricsOverride',{width:1400,height:1600,deviceScaleFactor:1,mobile:false});
 await ws.send('Page.navigate',{url:'file:///D:/RjDir/UserData/Downloads/cors-main/app/mcors-0.1.0-win64-beta/doc/%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6.html'});
 await new Promise(r=>setTimeout(r,13000));
 const ev=async e=>{const r=await ws.send('Runtime.evaluate',{expression:e,returnByValue:true});
  return r.result&&r.result.result?r.result.result.value:undefined};
 const scan=async()=>await ev(`(()=>{
   const bad=[];
   document.querySelectorAll('body *').forEach(el=>{
     if(el.children.length===0){
       const t=(el.textContent||'').trim();
       if(/[\u4e00-\u9fff]/.test(t)) bad.push(t.slice(0,40));
     }
   });
   const btn=document.getElementById('lang-toggle');
   return {lang:MCORS_DOC.getLang(), btn:btn?btn.textContent:null, left:bad.length, sample:bad.slice(0,12)};
 })()`);
 let s=await scan();
 console.log('ZH state:',JSON.stringify(s,null,0));
 let sh=await ws.send('Page.captureScreenshot',{format:'png'});
 fs.writeFileSync(dir+'/doc_zh.png',Buffer.from(sh.result.data,'base64'));
 // 切英文
 await ev(`document.getElementById('lang-toggle').click()`);
 await new Promise(r=>setTimeout(r,2000));
 s=await scan();
 console.log('EN state:',JSON.stringify(s,null,0));
 sh=await ws.send('Page.captureScreenshot',{format:'png'});
 fs.writeFileSync(dir+'/doc_en.png',Buffer.from(sh.result.data,'base64'));
 console.log('saved both');
 process.exit(0);
})().catch(e=>{console.error('ERR',e);process.exit(1)});
