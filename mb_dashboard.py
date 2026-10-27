#!/usr/bin/env python3
"""Live web dashboard for the Modbus master demo (standard library only, works without internet).

Used by qemu_serial_bridge.py (--dashboard PORT). The guest sends one "DATA {json}" line per poll; the
bridge hands each one to Dashboard.push(). Open http://localhost:PORT/ in a browser.
Can also be run alone with --demo to see the page with fake data.
"""
import json, threading, time, os, sys
try:
    from mb_fonts import FONTS
except ImportError:
    FONTS = {}
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Modbus gateway</title>
<style>
@font-face{font-family:Inter;font-weight:400;src:url(f/inter-400.woff) format("woff")}
@font-face{font-family:Inter;font-weight:500;src:url(f/inter-500.woff) format("woff")}
@font-face{font-family:Inter;font-weight:600;src:url(f/inter-600.woff) format("woff")}
:root{--bg:#0f1115;--panel:#16191f;--line:#262b33;--fg:#d8dce3;--mut:#8b93a1;--dim:#5d6572;
--temp:#f0883e;--hum:#58a6ff;--ok:#56d364;--warn:#e3b341;--bad:#f85149}
*{box-sizing:border-box;margin:0}
body{background:var(--bg);color:var(--fg);font:400 13px/1.45 Inter,system-ui,sans-serif;font-feature-settings:"tnum","cv11"}
header{display:flex;align-items:center;gap:16px;height:48px;padding:0 20px;border-bottom:1px solid var(--line);background:var(--panel)}
header h1{font-size:14px;font-weight:600;letter-spacing:.01em;white-space:nowrap}
@media(max-width:700px){#tag,#age{display:none}}
header .tag{color:var(--mut)}header .sp{flex:1}
.pill{display:inline-flex;align-items:center;gap:7px;padding:3px 10px;border:1px solid var(--line);border-radius:999px;font-weight:500;font-size:12px}
.pill i{width:7px;height:7px;border-radius:50%;background:var(--bad)}
.pill.live i{background:var(--ok)}.pill.stale i{background:var(--warn)}
main{padding:16px 20px 28px;display:grid;gap:12px;grid-template-columns:repeat(12,1fr);max-width:1280px;margin:0 auto}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:4px;min-width:0;position:relative}
.panel>h2{font-size:12px;font-weight:500;color:var(--mut);padding:10px 14px 0}
.s3{grid-column:span 3}.s6{grid-column:span 6}.s12{grid-column:span 12}
@media(max-width:900px){.s3{grid-column:span 6}.s6{grid-column:span 12}}
@media(max-width:520px){.s3{grid-column:span 12}}
.stat{padding:2px 14px 0;height:62px;display:flex;align-items:baseline;gap:5px}
.stat b{font-size:34px;font-weight:600;letter-spacing:-.02em;line-height:62px}
.stat span{color:var(--mut);font-size:14px}
canvas{display:block;width:100%}
.spark{height:44px}.chart{height:220px;cursor:crosshair}
.legend{display:flex;gap:22px;padding:0 14px 10px;color:var(--mut);font-size:12px;flex-wrap:wrap}
.legend em{font-style:normal;color:var(--fg);font-weight:500}.legend .name{color:var(--fg);display:flex;align-items:center;gap:7px}
.legend .name i{width:12px;height:3px;border-radius:2px}
table{width:100%;border-collapse:collapse;margin-top:6px}
td{padding:7px 14px;border-top:1px solid var(--line)}td:first-child{color:var(--mut)}td:last-child{text-align:right;font-weight:500}
.bar{height:6px;margin:12px 14px 10px;background:var(--line);border-radius:3px;overflow:hidden;display:flex}
.bar i{display:block;height:100%}
.tip{position:fixed;pointer-events:none;background:#0b0d10;border:1px solid var(--line);border-radius:4px;padding:6px 9px;font-size:12px;display:none;z-index:5;white-space:nowrap}
.tip div{color:var(--mut);margin-bottom:2px}
</style></head><body>
<header><h1>Modbus gateway</h1><span class="tag" id="tag">bare-metal RV64 master</span><span class="sp"></span>
<span class="tag" id="age"></span><span class="pill" id="pill"><i></i><span id="state">Connecting</span></span></header>
<main>
 <section class="panel s3"><h2>Temperature</h2><div class="stat"><b id="t" style="color:var(--temp)">--</b><span>°C</span></div><canvas class="spark" id="st"></canvas></section>
 <section class="panel s3"><h2>Humidity</h2><div class="stat"><b id="h" style="color:var(--hum)">--</b><span>%</span></div><canvas class="spark" id="sh"></canvas></section>
 <section class="panel s3"><h2>Modbus round trip</h2><div class="stat"><b id="rtt">--</b><span>ms</span></div><canvas class="spark" id="sr"></canvas></section>
 <section class="panel s3"><h2>Firmware uptime</h2><div class="stat"><b id="up">--</b></div><canvas class="spark" id="su"></canvas></section>
 <section class="panel s12"><h2>Temperature</h2><canvas class="chart" id="ct"></canvas><div class="legend" id="lt"></div></section>
 <section class="panel s12"><h2>Relative humidity</h2><canvas class="chart" id="ch"></canvas><div class="legend" id="lh"></div></section>
 <section class="panel s6"><h2>Link health</h2><div class="bar" id="bar"></div>
  <table><tr><td>Polls</td><td id="n">0</td></tr><tr><td>Good replies</td><td id="ok">0</td></tr><tr><td>Retries</td><td id="retry">0</td></tr>
  <tr><td>No answer</td><td id="tmo">0</td></tr><tr><td>Damaged replies</td><td id="bad">0</td></tr><tr><td>Exception replies</td><td id="exc">0</td></tr></table></section>
 <section class="panel s6"><h2>Firmware</h2>
  <table><tr><td>Boot to first reading</td><td id="first">--</td></tr><tr><td>Poll start jitter, max</td><td id="jit">--</td></tr>
  <tr><td>Poll start jitter, last</td><td id="jlast">--</td></tr><tr><td>Image size</td><td id="img">--</td></tr><tr><td>Last result</td><td id="rc">--</td></tr></table></section>
</main>
<div class="tip" id="tip"></div>
<script>
const $=id=>document.getElementById(id),css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
let rows=[],jmax=0,last=0,imgBytes=0;const WIN=300;
const dec=d=>Math.max(0,Math.round(Math.log10(d||1)));
const fmt=(v,d)=>(v/d).toFixed(dec(d));
const clock=ms=>{const s=Math.floor(ms/1000),m=Math.floor(s/60);return m+":"+String(s%60).padStart(2,"0")};
const dur=ms=>ms<1000?ms+" ms":ms<60000?(ms/1000).toFixed(2)+" s":Math.floor(ms/60000)+" min "+Math.floor(ms/1000%60)+" s";
function nice(lo,hi,n){let span=(hi-lo)||1,raw=span/n,p=Math.pow(10,Math.floor(Math.log10(raw))),f=raw/p,s=(f<1.5?1:f<3?2:f<7?5:10)*p;
 const a=Math.floor(lo/s)*s,b=Math.ceil(hi/s)*s,t=[];for(let v=a;v<=b+s/2;v+=s)t.push(+v.toFixed(10));return t}
function setup(cv){const r=cv.getBoundingClientRect(),d=devicePixelRatio||1;cv.width=Math.round(r.width*d);cv.height=Math.round(r.height*d);
 const g=cv.getContext("2d");g.setTransform(d,0,0,d,0,0);g.clearRect(0,0,r.width,r.height);return{g,W:r.width,H:r.height}}
function series(f){return rows.slice(-WIN).map(r=>({x:r.up,y:r.rc===0?f(r):null,bad:r.rc!==0}))}
function spark(cv,pts,col){const{g,W,H}=setup(cv);const v=pts.filter(p=>p.y!==null).map(p=>p.y);if(v.length<2)return;
 let lo=Math.min(...v),hi=Math.max(...v);if(hi-lo<1e-9){lo-=1;hi+=1}const x0=pts[0].x,x1=pts[pts.length-1].x||1;
 const X=x=>(x-x0)/((x1-x0)||1)*W,Y=y=>H-4-(H-8)*(y-lo)/(hi-lo);
 g.beginPath();let first=true,lx=0;pts.forEach(p=>{if(p.y===null)return;first?g.moveTo(X(p.x),Y(p.y)):g.lineTo(X(p.x),Y(p.y));first=false;lx=X(p.x)});
 g.strokeStyle=col;g.lineWidth=1.5;g.stroke();g.lineTo(lx,H);g.lineTo(X(pts.find(p=>p.y!==null).x),H);g.closePath();
 const gr=g.createLinearGradient(0,0,0,H);gr.addColorStop(0,col+"44");gr.addColorStop(1,col+"00");g.fillStyle=gr;g.fill()}
const charts={};
function chart(id,pts,col,unit,dp){const cv=$(id),{g,W,H}=setup(cv),L=46,R=14,T=12,B=24;
 const v=pts.filter(p=>p.y!==null).map(p=>p.y);if(!v.length){charts[id]=null;return}
 let lo=Math.min(...v),hi=Math.max(...v);if(hi-lo<1){const m=(hi+lo)/2;lo=m-.5;hi=m+.5}const pad=(hi-lo)*.15;lo-=pad;hi+=pad;
 const yt=nice(lo,hi,4);lo=Math.min(lo,yt[0]);hi=Math.max(hi,yt[yt.length-1]);
 const x0=pts[0].x,x1=pts[pts.length-1].x,X=x=>L+(W-L-R)*((x-x0)/((x1-x0)||1)),Y=y=>T+(H-T-B)*(1-(y-lo)/(hi-lo));
 g.font="11px Inter,system-ui,sans-serif";g.fillStyle=css("--dim");g.strokeStyle=css("--line");g.lineWidth=1;g.textBaseline="middle";g.textAlign="right";
 yt.forEach(t=>{const y=Math.round(Y(t))+.5;g.beginPath();g.moveTo(L,y);g.lineTo(W-R,y);g.stroke();g.fillText(t.toFixed(dp),L-8,y)});
 const xt=nice(x0/1000,x1/1000,Math.max(2,Math.floor((W-L-R)/110)));g.textAlign="center";g.textBaseline="top";
 xt.forEach(t=>{const x=X(t*1000);if(x<L-1||x>W-R+1)return;g.fillText(clock(t*1000),Math.min(Math.max(x,L+14),W-R-14),H-B+8)});
 g.fillStyle=css("--bad")+"22";pts.forEach(p=>{if(p.bad){const x=X(p.x);g.fillRect(x-3,T,6,H-T-B)}});
 g.beginPath();let pen=false;pts.forEach(p=>{if(p.y===null){pen=false;return}pen?g.lineTo(X(p.x),Y(p.y)):g.moveTo(X(p.x),Y(p.y));pen=true});
 g.strokeStyle=col;g.lineWidth=1.5;g.lineJoin="round";g.stroke();
 const gr=g.createLinearGradient(0,T,0,H-B);gr.addColorStop(0,col+"30");gr.addColorStop(1,col+"00");
 g.beginPath();pen=false;let sx=0,ex=0;pts.forEach(p=>{if(p.y===null){if(pen){g.lineTo(ex,H-B);g.lineTo(sx,H-B);g.closePath();g.fillStyle=gr;g.fill()}pen=false;return}
  const x=X(p.x);if(!pen){g.beginPath();g.moveTo(x,H-B);g.lineTo(x,Y(p.y));sx=x}else g.lineTo(x,Y(p.y));pen=true;ex=x});
 if(pen){g.lineTo(ex,H-B);g.lineTo(sx,H-B);g.closePath();g.fillStyle=gr;g.fill()}
 charts[id]={pts,X,Y,L,R,T,B,W,H,col,unit,dp,cv};
 const m=v.reduce((a,b)=>a+b,0)/v.length;
 $("l"+id.slice(1)).innerHTML='<span class="name"><i style="background:'+col+'"></i>'+(id==="ct"?"Temperature":"Humidity")+'</span>'+
  [["Last",v[v.length-1]],["Min",Math.min(...v)],["Max",Math.max(...v)],["Mean",m]].map(a=>"<span>"+a[0]+" <em>"+a[1].toFixed(dp)+" "+unit+"</em></span>").join("")}
function hover(id){const cv=$(id);cv.addEventListener("mousemove",e=>{const c=charts[id];if(!c)return;const r=cv.getBoundingClientRect(),mx=e.clientX-r.left;
  let best=null,bd=1e9;c.pts.forEach(p=>{const d=Math.abs(c.X(p.x)-mx);if(d<bd){bd=d;best=p}});if(!best)return;
  render();const k=charts[id],g=cv.getContext("2d"),x=k.X(best.x);g.save();g.strokeStyle=css("--mut");g.lineWidth=1;g.setLineDash([3,3]);g.beginPath();g.moveTo(x+.5,k.T);g.lineTo(x+.5,k.H-k.B);g.stroke();g.restore();
  if(best.y!==null){g.fillStyle=k.col;g.beginPath();g.arc(x,k.Y(best.y),3.5,0,7);g.fill()}
  const t=$("tip");t.style.display="block";t.innerHTML="<div>uptime "+clock(best.x)+"</div>"+(best.y===null?'<span style="color:var(--bad)">no reading</span>':'<b style="color:'+k.col+'">'+best.y.toFixed(k.dp)+" "+k.unit+"</b>");
  t.style.left=Math.min(e.clientX+14,innerWidth-t.offsetWidth-8)+"px";t.style.top=(e.clientY-t.offsetHeight-12)+"px"});
 cv.addEventListener("mouseleave",()=>{$("tip").style.display="none";render()})}
function render(){const L=rows[rows.length-1];if(!L)return;const ok=rows.filter(r=>r.rc===0),G=ok[ok.length-1];
 if(G){$("t").textContent=fmt(G.t,G.td);$("h").textContent=fmt(G.h,G.hd)}
 $("rtt").textContent=L.rtt;$("up").textContent=dur(L.up);$("first").textContent=L.first?dur(L.first):"--";
 jmax=Math.max(jmax,Math.abs(L.jit));$("jit").textContent=(jmax/1000).toFixed(3)+" ms";$("jlast").textContent=(L.jit/1000).toFixed(3)+" ms";
 $("n").textContent=L.seq;["ok","retry","tmo","bad","exc"].forEach(k=>$(k).textContent=L[k]);
 $("rc").textContent=L.rc===0?"ok":L.rc===-62?"no answer":L.rc===-71?"exception reply":"error "+L.rc;$("rc").style.color=L.rc===0?"var(--ok)":"var(--bad)";
 const tot=L.seq||1,g=L.ok/tot*100,b=(L.seq-L.ok)/tot*100;$("bar").innerHTML='<i style="width:'+g+'%;background:var(--ok)"></i><i style="width:'+b+'%;background:var(--bad)"></i>';
 const T=series(r=>r.t/r.td),Hh=series(r=>r.h/r.hd);
 spark($("st"),T,css("--temp"));spark($("sh"),Hh,css("--hum"));spark($("sr"),series(r=>r.rtt),css("--mut"));spark($("su"),rows.slice(-WIN).map(r=>({x:r.up,y:r.up})),css("--mut"));
 chart("ct",T,css("--temp"),"°C",1);chart("ch",Hh,css("--hum"),"%",1)}
function add(r){rows.push(r);if(rows.length>600)rows.shift();last=Date.now();render()}
function status(){const p=$("pill");if(!last){return}const s=(Date.now()-last)/1000;$("age").textContent="Updated "+Math.round(s)+" s ago";
 const st=s>15?"stale":"live";p.className="pill "+st;$("state").textContent=st==="live"?"Live":"No data"}
const es=new EventSource("events");es.onerror=()=>{$("pill").className="pill";$("state").textContent="Disconnected"};
es.onmessage=e=>{add(JSON.parse(e.data));status()};setInterval(status,1000);
fetch("info").then(r=>r.json()).then(i=>{imgBytes=i.image_bytes;$("img").textContent=imgBytes?(imgBytes/1024).toFixed(1)+" KiB":"not given"}).catch(()=>{});
hover("ct");hover("ch");addEventListener("resize",render);
</script></body></html>
"""

class Dashboard:
    def __init__(self, port, image=None, history=600):
        self.rows, self.cap, self.cond = [], history, threading.Condition()
        self.seq = 0
        self.image_bytes = os.path.getsize(image) if image and os.path.exists(image) else 0
        dash = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a): pass
            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    b = PAGE.encode(); self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8'); self.send_header('Content-Length', str(len(b)))
                    self.end_headers(); self.wfile.write(b)
                elif self.path.startswith('/f/') and self.path[3:].rsplit('.', 1)[0] in FONTS:
                    b = FONTS[self.path[3:].rsplit('.', 1)[0]]; self.send_response(200)
                    self.send_header('Content-Type', 'font/woff'); self.send_header('Cache-Control', 'max-age=86400')
                    self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
                elif self.path == '/info':
                    b = json.dumps({'image_bytes': dash.image_bytes}).encode(); self.send_response(200)
                    self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(b)))
                    self.end_headers(); self.wfile.write(b)
                elif self.path == '/history.json':
                    with dash.cond: b = json.dumps(dash.rows).encode()
                    self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(b)))
                    self.end_headers(); self.wfile.write(b)
                elif self.path == '/events':
                    self.send_response(200); self.send_header('Content-Type', 'text/event-stream')
                    self.send_header('Cache-Control', 'no-cache'); self.end_headers()
                    try:
                        with dash.cond: sent = len(dash.rows); first = list(dash.rows)
                        base = dash.seq - sent
                        for r in first: self.wfile.write(('data: %s\n\n' % json.dumps(r)).encode())
                        self.wfile.flush(); pos = dash.seq
                        while True:
                            with dash.cond:
                                dash.cond.wait(15)
                                new = dash.seq - pos
                                rows = dash.rows[-new:] if new > 0 else []
                                pos = dash.seq
                            if not rows: self.wfile.write(b': keepalive\n\n')
                            for r in rows: self.wfile.write(('data: %s\n\n' % json.dumps(r)).encode())
                            self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        pass
                else:
                    self.send_error(404)

        self.httpd = ThreadingHTTPServer(('0.0.0.0', port), H)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def push(self, text):
        """text is the JSON after "DATA ". Returns the parsed dict, or None if it was not valid."""
        try: r = json.loads(text)
        except ValueError: return None
        with self.cond:
            self.rows.append(r); self.seq += 1
            if len(self.rows) > self.cap: del self.rows[0]
            self.cond.notify_all()
        return r

if __name__ == '__main__':
    import random
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    d = Dashboard(port); print('demo data on http://localhost:%d/' % port)
    n = 0; t0 = time.time()
    while True:
        n += 1; fail = n % 17 == 0
        d.push(json.dumps({'seq': n, 'up': int((time.time() - t0) * 1000) + 800, 'rc': -62 if fail else 0,
                           't': 0 if fail else 2900 + random.randint(-30, 30), 'h': 0 if fail else 4800 + random.randint(-80, 80), 'td': 100, 'hd': 100,
                           'rtt': 2000 if fail else 583, 'jit': random.randint(0, 40), 'ok': n - n // 17, 'retry': 2 * (n // 17), 'tmo': n // 17,
                           'bad': 0, 'exc': 0, 'first': 1400}))
        time.sleep(2)