#include "interface_web.h"

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

#include "ball_position.h"
#include "controller.h"
#include "servo_cmd.h"
#include "tof.h"

static const char *AP_SSID = "Kit_Robovaly";
static const char *AP_PASS = "robovaly123";

static const BaseType_t WEB_TASK_CORE = 0;
static const uint32_t WEB_TASK_DELAY_MS = 1;

static WebServer server(80);
static TaskHandle_t web_task_handle = nullptr;

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PID Table Debug</title>
<style>
:root{--bg:#f4f1e8;--ink:#171717;--muted:#666;--line:#202020;--panel:#fffdf6;--red:#c43131;--blue:#2457b8;--green:#208444}
*{box-sizing:border-box} body{margin:0;background:var(--bg);color:var(--ink);font-family:Arial,Helvetica,sans-serif}
.app{width:min(1180px,100vw);margin:0 auto;padding:10px;display:grid;gap:10px}
.panel{background:var(--panel);border:3px solid var(--line);border-radius:4px;padding:10px}
.sceneWrap{height:42vh;min-height:280px;position:relative}.sceneWrap canvas{width:100%;height:100%;display:block}
.topBtn{position:absolute;top:10px;left:10px;width:52px;height:44px;border:3px solid var(--line);background:#f8f5ea;font-weight:900;font-size:24px}
.sectionTitle{text-align:center;font-size:25px;font-weight:800;margin:0 0 8px}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}.plotBox{height:300px;position:relative}.plotBox canvas{width:100%;height:100%}
.plotLabel{position:absolute;top:8px;right:50px;font-weight:800;text-decoration:underline}.plotBtn{position:absolute;top:8px;right:8px;width:34px;height:30px;border:3px solid var(--line);background:#f8f5ea;font-weight:900}
.controls{display:grid;grid-template-columns:1fr 1fr;gap:10px}.formGrid{display:grid;grid-template-columns:auto 1fr;gap:10px;align-items:center}
label{font-weight:800}.field{height:38px;border:3px solid var(--line);background:white;font-size:18px;padding:3px 8px;width:100%}
.stabHeader{display:flex;justify-content:space-between;align-items:center;gap:12px;font-size:24px;font-weight:800}.toggle{width:46px;height:38px;border:3px solid var(--line);background:#f8f5ea;font-size:24px;font-weight:900}
.manualTitle{text-align:center;font-size:22px;font-weight:800;text-decoration:underline;margin:8px 0}.sliderRow{display:grid;grid-template-columns:34px 1fr 46px;gap:10px;align-items:center}
input[type=range]{width:100%;accent-color:#111}.small{font-size:13px;color:var(--muted);margin-top:8px}.status{display:flex;gap:16px;flex-wrap:wrap;font-size:14px;color:var(--muted)}
button{cursor:pointer}button:disabled,input:disabled{opacity:.5;cursor:not-allowed}
@media(max-width:800px){.grid,.controls{grid-template-columns:1fr}.sceneWrap{height:34vh}.plotBox{height:240px}}
</style>
</head>
<body>
<div class="app">
  <div class="panel sceneWrap">
    <button class="topBtn" id="scenePause">||</button>
    <canvas id="scene"></canvas>
  </div>

  <div class="panel">
    <div class="sectionTitle">- Data Viz - <button class="toggle" id="plotToggle">Go</button></div>
    <div class="grid">
      <div class="panel plotBox"><canvas id="anglePlot"></canvas><div class="plotLabel">Angle</div><button class="plotBtn" id="anglePause">||</button></div>
      <div class="panel plotBox"><canvas id="posPlot"></canvas><div class="plotLabel">Pos</div><button class="plotBtn" id="posPause">||</button></div>
      <div class="panel plotBox"><canvas id="speedPlot"></canvas><div class="plotLabel">Speed</div><button class="plotBtn" id="speedPause">||</button></div>
    </div>
    <div class="small" id="plotInfo">Plot stopped. Press Go to start a 30 s capture.</div>
  </div>

  <div class="controls">
    <div class="panel">
      <div class="formGrid">
        <label for="refInput">Ref X0</label><input class="field" id="refInput" type="number" step="1">
        <label for="kpInput">Kp</label><input class="field" id="kpInput" type="number" step="0.001">
        <label for="kiInput">Ki</label><input class="field" id="kiInput" type="number" step="0.001">
        <label for="kdInput">Kd</label><input class="field" id="kdInput" type="number" step="0.001">
      </div>
    </div>
    <div class="panel">
      <div class="stabHeader"><span>Stabilization</span><button class="toggle" id="stabToggle">▶</button></div>
      <div class="manualTitle">Manual Ctrl</div>
      <div class="sliderRow"><span>0</span><input id="manualSlider" type="range" min="0" max="180" step="1"><span>180</span></div>
      <div class="small">Servo: <b id="servoTxt">--</b> deg | table display angle: <b id="tableTxt">--</b> deg</div>
      <div class="status">
        <span>x=<b id="xTxt">--</b> mm</span>
        <span>v=<b id="vTxt">--</b> mm/s</span>
        <span>D1=<b id="d1Txt">--</b></span>
        <span>D2=<b id="d2Txt">--</b></span>
      </div>
    </div>
  </div>
</div>
<script>
const TABLE_LEN_MM=290, REFRESH_MS=55, MAX_PLOT_S=30;
const scene=document.getElementById('scene'), anglePlot=document.getElementById('anglePlot'), posPlot=document.getElementById('posPlot'), speedPlot=document.getElementById('speedPlot');
const scenePause=document.getElementById('scenePause'), plotToggle=document.getElementById('plotToggle'), plotInfo=document.getElementById('plotInfo');
const anglePause=document.getElementById('anglePause'), posPause=document.getElementById('posPause'), speedPause=document.getElementById('speedPause');
const stabToggle=document.getElementById('stabToggle'), manualSlider=document.getElementById('manualSlider');
const refInput=document.getElementById('refInput'), kpInput=document.getElementById('kpInput'), kiInput=document.getElementById('kiInput'), kdInput=document.getElementById('kdInput');
const servoTxt=document.getElementById('servoTxt'), tableTxt=document.getElementById('tableTxt'), xTxt=document.getElementById('xTxt'), vTxt=document.getElementById('vTxt'), d1Txt=document.getElementById('d1Txt'), d2Txt=document.getElementById('d2Txt');
let state={x:-1,v:0,speed_valid:false,servo_angle:90,stabilization:true,kp:0,ki:0,kd:0,ref:150,d1:-1,d2:-1};
let sceneFrozen=false, plotRunning=false, angleFrozen=false, posFrozen=false, speedFrozen=false, plotStart=0, angleData=[], posData=[], speedData=[], lastStab=true, editing=false;
function fit(c){const r=c.getBoundingClientRect(),d=window.devicePixelRatio||1;const w=Math.max(1,Math.floor(r.width*d)),h=Math.max(1,Math.floor(r.height*d));if(c.width!==w||c.height!==h){c.width=w;c.height=h}}
function drawScene(){
fit(scene);
const c=scene.getContext('2d'),w=scene.width,h=scene.height;
c.clearRect(0,0,w,h);
c.lineWidth=4;c.strokeStyle='#171717';c.fillStyle='#171717';
const cx=w*.5,cy=h*.58,len=w*.68;
const tableDeg=state.servo_angle/2;
const slopeDeg=45-tableDeg;
const a=slopeDeg*Math.PI/180;
const ux=Math.cos(a),uy=Math.sin(a);
const nx=uy,ny=-ux;
const x1=cx-ux*len/2,y1=cy-uy*len/2,x2=cx+ux*len/2,y2=cy+uy*len/2;
c.beginPath();c.moveTo(x1,y1);c.lineTo(x2,y2);c.stroke();
c.beginPath();c.moveTo(cx,cy+8);c.lineTo(cx-w*.035,cy+h*.19);c.lineTo(cx+w*.035,cy+h*.19);c.closePath();c.stroke();
let p=state.x>=0?Math.max(0,Math.min(TABLE_LEN_MM,state.x))/TABLE_LEN_MM:.5;
const r=Math.max(16,Math.min(w,h)*.04);
let contactX=x1+(x2-x1)*p,contactY=y1+(y2-y1)*p;
let bx=contactX+nx*r,by=contactY+ny*r;
c.beginPath();c.arc(bx,by,r,0,Math.PI*2);c.stroke();
c.beginPath();c.moveTo(bx-r*.65,by-r*.65);c.lineTo(bx+r*.65,by+r*.65);c.moveTo(bx+r*.65,by-r*.65);c.lineTo(bx-r*.65,by+r*.65);c.stroke();
c.font=`${Math.max(14,w*.018)}px Arial`;
c.fillText(`x=${state.x} mm`,18,h-44);
c.fillText(`servo=${state.servo_angle} deg / table=${tableDeg.toFixed(1)} deg`,18,h-20);
}
function scaleMax(t){return t<10?10:t<20?20:30}
function drawPlot(canvas,data,color,label,freeze){
fit(canvas);
const c=canvas.getContext('2d'),w=canvas.width,h=canvas.height,p=44;
c.clearRect(0,0,w,h);
c.lineWidth=3;c.strokeStyle='#171717';
c.beginPath();c.moveTo(p,12);c.lineTo(p,h-p);c.lineTo(w-12,h-p);c.stroke();
const tNow=plotRunning?(performance.now()-plotStart)/1000:(data.length?data[data.length-1].t:0);
const xmax=scaleMax(tNow);
let vals=data.map(d=>d.y).filter(Number.isFinite);
let ymin=label==='pos'?0:0,ymax=label==='pos'?TABLE_LEN_MM:90;
if(label==='speed'){
  const maxAbs=Math.max(100,...vals.map(v=>Math.abs(v)));
  ymax=Math.ceil(maxAbs/50)*50;
  ymin=-ymax;
}
else if(vals.length&&label!=='pos'){
  ymin=Math.min(0,...vals)-5;
  ymax=Math.max(90,...vals)+5;
  if(ymax-ymin<20){ymin-=10;ymax+=10}
}
const yOf=v=>h-p-(v-ymin)/(ymax-ymin)*(h-p-16);
function dashedRef(value,text){
  if(value<ymin||value>ymax)return;
  const y=yOf(value);
  c.save();
  c.setLineDash([10,8]);
  c.strokeStyle='#555';
  c.lineWidth=2;
  c.beginPath();c.moveTo(p,y);c.lineTo(w-12,y);c.stroke();
  c.restore();
  c.fillStyle='#555';
  c.font=`${Math.max(11,w*.021)}px Arial`;
  c.fillText(text,p+8,y-6);
}
if(label==='angle')dashedRef(45,'neutral 45 deg');
if(label==='pos')dashedRef(state.ref,`x0 ${state.ref} mm`);
if(label==='speed')dashedRef(0,'0 mm/s');
c.strokeStyle=color;c.lineWidth=4;c.beginPath();
data.forEach((d,i)=>{
  const x=p+(d.t/xmax)*(w-p-18);
  const y=yOf(d.y);
  if(i===0)c.moveTo(x,y);else c.lineTo(x,y);
});
c.stroke();
c.fillStyle='#171717';
c.font=`${Math.max(12,w*.025)}px Arial`;
c.fillText(`${label} | 0-${xmax}s`,p+8,24);
if(vals.length)c.fillText(`${vals[vals.length-1].toFixed(1)}`,w-90,24);
}
function drawAll(){if(!sceneFrozen)drawScene();if(!angleFrozen)drawPlot(anglePlot,angleData,'#c43131','angle',false);if(!posFrozen)drawPlot(posPlot,posData,'#2457b8','pos',false);if(!speedFrozen)drawPlot(speedPlot,speedData,'#208444','speed',false)}
function updateTexts(){servoTxt.textContent=state.servo_angle;tableTxt.textContent=(state.servo_angle/2).toFixed(1);xTxt.textContent=state.x>=0?state.x:'--';vTxt.textContent=state.speed_valid?state.v:'--';d1Txt.textContent=state.d1>=0?state.d1:'--';d2Txt.textContent=state.d2>=0?state.d2:'--';stabToggle.textContent=state.stabilization?'||':'▶';manualSlider.disabled=state.stabilization;if(!editing){refInput.value=state.ref;kpInput.value=Number(state.kp).toFixed(3);kiInput.value=Number(state.ki).toFixed(3);kdInput.value=Number(state.kd).toFixed(3)}if(lastStab&&!state.stabilization)manualSlider.value=state.servo_angle;lastStab=state.stabilization}
async function fetchState(){try{const r=await fetch('/api/state',{cache:'no-store'});state=await r.json();updateTexts();if(plotRunning){const t=(performance.now()-plotStart)/1000;if(t<=MAX_PLOT_S){angleData.push({t,y:state.servo_angle/2});posData.push({t,y:state.x>=0?state.x:NaN});speedData.push({t,y:state.speed_valid?state.v:NaN});plotInfo.textContent=`Plot running: ${t.toFixed(1)} s / 30 s`}else{plotRunning=false;plotToggle.textContent='Go';plotInfo.textContent='30 s reached. Plot stopped.'}}drawAll()}catch(e){}}
function startPlot(){angleData=[];posData=[];speedData=[];plotStart=performance.now();plotRunning=true;angleFrozen=false;posFrozen=false;speedFrozen=false;plotToggle.textContent='Stop';plotInfo.textContent='Plot running'}
function stopPlot(){plotRunning=false;plotToggle.textContent='Go';plotInfo.textContent='Plot frozen. Press Go to restart from 0.'}
plotToggle.onclick=()=>plotRunning?stopPlot():startPlot();scenePause.onclick=()=>{sceneFrozen=!sceneFrozen;scenePause.textContent=sceneFrozen?'▶':'||'};anglePause.onclick=()=>{angleFrozen=!angleFrozen;anglePause.textContent=angleFrozen?'▶':'||'};posPause.onclick=()=>{posFrozen=!posFrozen;posPause.textContent=posFrozen?'▶':'||'};speedPause.onclick=()=>{speedFrozen=!speedFrozen;speedPause.textContent=speedFrozen?'▶':'||'};
stabToggle.onclick=async()=>{const en=state.stabilization?0:1;await fetch(`/api/control?stabilization=${en}`,{cache:'no-store'});if(!en)manualSlider.value=state.servo_angle;fetchState()};
manualSlider.oninput=()=>{servoTxt.textContent=manualSlider.value;tableTxt.textContent=(Number(manualSlider.value)/2).toFixed(1)};
manualSlider.onchange=()=>{fetch(`/api/control?angle=${manualSlider.value}`,{cache:'no-store'}).then(fetchState)};
[refInput,kpInput,kiInput,kdInput].forEach(el=>{el.onfocus=()=>editing=true;el.onblur=()=>editing=false;el.onchange=()=>{const q=`ref=${refInput.value}&kp=${kpInput.value}&ki=${kiInput.value}&kd=${kdInput.value}`;fetch(`/api/params?${q}`,{cache:'no-store'}).then(fetchState)}});
window.onresize=drawAll;setInterval(fetchState,REFRESH_MS);fetchState();
</script>
</body>
</html>
)rawliteral";

static void send_state(void) {
  float kp;
  float ki;
  float kd;
  get_controller_gains(&kp, &ki, &kd);

  String json = "{";
  json += "\"d1\":" + String(get_d1()) + ",";
  json += "\"d2\":" + String(get_d2()) + ",";
  json += "\"x\":" + String(get_ball_position()) + ",";
  json += "\"v\":" + String(get_ball_speed()) + ",";
  json += "\"speed_valid\":" + String(is_ball_speed_valid() ? "true" : "false") + ",";
  json += "\"servo_angle\":" + String(get_controller_last_angle_deg()) + ",";
  json += "\"stabilization\":" + String(controller_is_enabled() ? "true" : "false") + ",";
  json += "\"controller_valid\":" + String(controller_last_update_was_valid() ? "true" : "false") + ",";
  json += "\"ref\":" + String(get_controller_reference_mm()) + ",";
  json += "\"kp\":" + String(kp, 6) + ",";
  json += "\"ki\":" + String(ki, 6) + ",";
  json += "\"kd\":" + String(kd, 6);
  json += "}";

  server.send(200, "application/json; charset=utf-8", json);
}

static void setup_routes(void) {
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
  });

  server.on("/api/state", HTTP_GET, []() {
    send_state();
  });

  server.on("/api/control", HTTP_GET, []() {
    if (server.hasArg("stabilization")) {
      set_controller_enabled(server.arg("stabilization").toInt() != 0);
    }

    if (server.hasArg("angle")) {
      set_controller_manual_angle(server.arg("angle").toInt());
    }

    send_state();
  });

  server.on("/api/params", HTTP_GET, []() {
    float kp;
    float ki;
    float kd;
    get_controller_gains(&kp, &ki, &kd);

    if (server.hasArg("ref")) {
      set_controller_reference_mm(server.arg("ref").toInt());
    }

    if (server.hasArg("kp")) kp = server.arg("kp").toFloat();
    if (server.hasArg("ki")) ki = server.arg("ki").toFloat();
    if (server.hasArg("kd")) kd = server.arg("kd").toFloat();
    set_controller_gains(kp, ki, kd);

    send_state();
  });

  server.onNotFound([]() {
    server.send(404, "text/plain; charset=utf-8", "404");
  });
}

static void web_task(void *pv) {
  (void)pv;

  WiFi.mode(WIFI_AP);
  bool ap_ok = WiFi.softAP(AP_SSID, AP_PASS);
  delay(200);

  Serial.printf("\nWeb AP %s: %s\n", AP_SSID, ap_ok ? "OK" : "FAIL");
  Serial.print("Web IP: ");
  Serial.println(WiFi.softAPIP());

  setup_routes();
  server.begin();

  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(WEB_TASK_DELAY_MS));
  }
}

bool launch_interface_web(void) {
  if (web_task_handle != nullptr) {
    return true;
  }

  BaseType_t created = xTaskCreatePinnedToCore(
      web_task, "TaskWeb", 10000, nullptr, 1, &web_task_handle, WEB_TASK_CORE);

  if (created != pdPASS) {
    web_task_handle = nullptr;
    return false;
  }

  return true;
}
