#include "interface_web.h"

#include <Arduino.h>
#include <Preferences.h>
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
static const char *CALIBRATION_NAMESPACE = "tof_cal";
static const char *CALIBRATION_DONE_KEY = "done";
static const char *CALIBRATION_VERSION_KEY = "version";
static const uint32_t CALIBRATION_SCHEMA_VERSION = 2;

static WebServer server(80);
static TaskHandle_t web_task_handle = nullptr;
static bool distance_sensors_calibrated = false;

enum CalibrationStep {
  CAL_TOF1_FIND_FOV,
  CAL_TOF1_ENTER_FOV_DISTANCE,
  CAL_TOF1_PLACE_145,
  CAL_TOF1_PLACE_72,
  CAL_TOF1_PLACE_0,
  CAL_TOF2_FIND_FOV,
  CAL_TOF2_ENTER_FOV_DISTANCE,
  CAL_TOF2_PLACE_145,
  CAL_TOF2_PLACE_72,
  CAL_TOF2_PLACE_0,
  CAL_VERIFY,
  CAL_ERROR,
};

struct TofCalibrationDraft {
  int meas_fov = INFINITE_TOF_VALUE;
  int real_fov = 145;
  int meas_0 = 0;
  int meas_72 = INFINITE_TOF_VALUE;
  int meas_145 = INFINITE_TOF_VALUE;
};

static TofCalibrationDraft calibration_tof1;
static TofCalibrationDraft calibration_tof2;
static CalibrationStep calibration_step = CAL_TOF1_FIND_FOV;
static String calibration_error_msg = "";

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

static const char CALIBRATION_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Calibration PID Table</title>
<style>
body{margin:0;min-height:100vh;background:#f4f1e8;color:#171717;font-family:Arial,Helvetica,sans-serif;padding:14px}
.app{width:min(1000px,100%);margin:0 auto;display:grid;gap:12px}.panel{background:#fffdf6;border:3px solid #202020;border-radius:4px;padding:16px}
h1{font-size:25px;line-height:1.2;margin:0 0 10px}.msg{font-size:18px;line-height:1.45;margin:0 0 12px}.scene{height:330px;position:relative}.scene canvas{width:100%;height:100%;display:block}
.row{display:flex;gap:12px;align-items:center;flex-wrap:wrap}.dot{width:18px;height:18px;border-radius:50%;background:#b91c1c;border:2px solid #202020}.ok{background:#1f8f45}.bad{background:#b91c1c}
button{border:3px solid #202020;background:#f8f5ea;color:#171717;font-size:20px;font-weight:900;padding:10px 18px;cursor:pointer}button:disabled{opacity:.55;cursor:wait}
input{height:42px;border:3px solid #202020;background:white;font-size:20px;padding:4px 8px;width:160px}.hint{font-size:14px;color:#666}.hidden{display:none}.err{color:#b91c1c;font-weight:800}
</style>
</head>
<body>
<div class="app">
<section class="panel">
<h1>Merci d'avoir participe au Workshop PID-TABLE de ROBOVALY.</h1>
<p class="msg">Pour continuer, veuillez callibrer les capteurs de distance du systeme.</p>
<div class="row"><span class="dot" id="dot"></span><b id="rawTxt">--</b><span id="tofTxt">--</span></div>
</section>
<section class="panel scene"><canvas id="calScene"></canvas></section>
<section class="panel">
<h1 id="title">Calibration</h1>
<p class="msg" id="instruction">Chargement...</p>
<div class="row" id="realRow"><label for="realInput"><b>Distance reelle au FOV max</b></label><input id="realInput" type="number" min="1" max="400" step="1"><span>mm</span></div>
<div class="row">
<button id="doneBtn">Done</button>
<button id="submitBtn">Valider distance</button>
<button id="acceptBtn">Valider calibration</button>
<button id="restartBtn">Restart</button>
</div>
<div class="hint" id="status">Les valeurs sont sauvegardees apres validation finale.</div>
</section>
</div>
<script>
const TABLE_LEN_MM=290, REFRESH_MS=80;
const scene=document.getElementById('calScene'),dot=document.getElementById('dot'),rawTxt=document.getElementById('rawTxt'),tofTxt=document.getElementById('tofTxt');
const title=document.getElementById('title'),instruction=document.getElementById('instruction'),statusEl=document.getElementById('status');
const realRow=document.getElementById('realRow'),realInput=document.getElementById('realInput');
const doneBtn=document.getElementById('doneBtn'),submitBtn=document.getElementById('submitBtn'),acceptBtn=document.getElementById('acceptBtn'),restartBtn=document.getElementById('restartBtn');
let s={};
function fit(c){const r=c.getBoundingClientRect(),d=window.devicePixelRatio||1;const w=Math.max(1,Math.floor(r.width*d)),h=Math.max(1,Math.floor(r.height*d));if(c.width!==w||c.height!==h){c.width=w;c.height=h}}
function drawScene(){
fit(scene);const c=scene.getContext('2d'),w=scene.width,h=scene.height;c.clearRect(0,0,w,h);c.lineWidth=4;c.strokeStyle='#171717';c.fillStyle='#171717';
const cx=w*.5,cy=h*.56,len=w*.72,a=0,ux=Math.cos(a),uy=Math.sin(a),nx=0,ny=-1;const x1=cx-len/2,y1=cy,x2=cx+len/2,y2=cy;
c.beginPath();c.moveTo(x1,y1);c.lineTo(x2,y2);c.stroke();c.beginPath();c.moveTo(cx,cy+8);c.lineTo(cx-w*.035,cy+h*.22);c.lineTo(cx+w*.035,cy+h*.22);c.closePath();c.stroke();
let pos=s.visual_pos_mm; if(!Number.isFinite(pos)||pos<0)pos=TABLE_LEN_MM/2; pos=Math.max(0,Math.min(TABLE_LEN_MM,pos)); const p=pos/TABLE_LEN_MM;
const r=Math.max(15,Math.min(w,h)*.045),contactX=x1+(x2-x1)*p,contactY=y1+(y2-y1)*p,bx=contactX+nx*r,by=contactY+ny*r;
c.beginPath();c.arc(bx,by,r,0,Math.PI*2);c.stroke();c.beginPath();c.moveTo(bx-r*.65,by-r*.65);c.lineTo(bx+r*.65,by+r*.65);c.moveTo(bx+r*.65,by-r*.65);c.lineTo(bx-r*.65,by+r*.65);c.stroke();
if(s.step==='verify'){[0,72,145].forEach(mm=>{const x=x1+(x2-x1)*(mm/TABLE_LEN_MM);c.strokeStyle='#208444';c.fillStyle='#208444';c.lineWidth=3;c.beginPath();c.moveTo(x,cy+30);c.lineTo(x,cy+70);c.stroke();c.beginPath();c.moveTo(x,cy+24);c.lineTo(x-8,cy+42);c.lineTo(x+8,cy+42);c.closePath();c.fill();c.fillText(`${mm}`,x-10,cy+90)});c.strokeStyle='#171717';c.fillStyle='#171717'}
c.font=`${Math.max(14,w*.018)}px Arial`;c.fillText(`visual pos=${Math.round(pos)} mm`,18,h-22);
}
function setButtons(){
doneBtn.classList.toggle('hidden',!s.needs_done);
submitBtn.classList.toggle('hidden',!s.needs_real_input);
realRow.classList.toggle('hidden',!s.needs_real_input);
acceptBtn.classList.toggle('hidden',s.step!=='verify');
restartBtn.classList.toggle('hidden',false);
if(s.needs_real_input && !realInput.value)realInput.value=s.real_fov||145;
}
function update(){
title.textContent=s.title||'Calibration';
instruction.textContent=s.instruction||'';
rawTxt.textContent=s.raw_valid?`raw=${s.raw_mm} mm`:'raw invalid';
tofTxt.textContent=s.tof?`TOF ${s.tof}`:'';
dot.className='dot '+(s.raw_valid?'ok':'bad');
if(s.error)statusEl.innerHTML=`<span class="err">${s.error}</span>`;else statusEl.textContent=s.status||'';
setButtons();drawScene();
}
async function getState(){try{const r=await fetch('/api/calibration/state',{cache:'no-store'});s=await r.json();update()}catch(e){statusEl.textContent='Erreur reseau'}}
async function action(q){doneBtn.disabled=submitBtn.disabled=acceptBtn.disabled=restartBtn.disabled=true;try{const r=await fetch('/api/calibration/action?'+q,{cache:'no-store'});s=await r.json();update();if(s.done)setTimeout(()=>location.reload(),600)}catch(e){statusEl.textContent='Erreur action'}doneBtn.disabled=submitBtn.disabled=acceptBtn.disabled=restartBtn.disabled=false}
doneBtn.onclick=()=>action('cmd=done');
submitBtn.onclick=()=>action('cmd=real_fov&value='+encodeURIComponent(realInput.value||'145'));
acceptBtn.onclick=()=>action('cmd=accept');
restartBtn.onclick=()=>action('cmd=restart');
window.onresize=drawScene;setInterval(getState,REFRESH_MS);getState();
</script>
</body>
</html>
)rawliteral";

static TofCalibrationDraft &draft_for_tof(int tof_number) {
  return (tof_number == TOF1) ? calibration_tof1 : calibration_tof2;
}

static int current_calibration_tof(void) {
  switch (calibration_step) {
    case CAL_TOF1_FIND_FOV:
    case CAL_TOF1_ENTER_FOV_DISTANCE:
    case CAL_TOF1_PLACE_145:
    case CAL_TOF1_PLACE_72:
    case CAL_TOF1_PLACE_0:
      return TOF1;

    case CAL_TOF2_FIND_FOV:
    case CAL_TOF2_ENTER_FOV_DISTANCE:
    case CAL_TOF2_PLACE_145:
    case CAL_TOF2_PLACE_72:
    case CAL_TOF2_PLACE_0:
      return TOF2;

    default:
      return 0;
  }
}

static int raw_tof_value(int tof_number) {
  return (tof_number == TOF1) ? get_mes_tof_1() : get_mes_tof_2();
}

static uint32_t raw_tof_timestamp(int tof_number) {
  return (tof_number == TOF1) ? get_tof_1_last_update_ms() : get_tof_2_last_update_ms();
}

static bool raw_tof_is_valid(int value) {
  return value >= 0 && value <= 290;
}

static int visual_position_from_raw(int tof_number, int raw_mm) {
  if (!raw_tof_is_valid(raw_mm)) {
    return -1;
  }

  return (tof_number == TOF1) ? 290 - raw_mm : raw_mm;
}

static bool capture_raw_average(int tof_number, int *average_mm) {
  static const int SAMPLE_COUNT = 8;
  static const uint32_t CAPTURE_TIMEOUT_MS = 1600;

  int sum = 0;
  int count = 0;
  uint32_t last_seen_ms = 0;
  uint32_t start_ms = millis();

  while (millis() - start_ms < CAPTURE_TIMEOUT_MS && count < SAMPLE_COUNT) {
    int value = raw_tof_value(tof_number);
    uint32_t timestamp = raw_tof_timestamp(tof_number);

    if (timestamp != 0 && timestamp != last_seen_ms && raw_tof_is_valid(value)) {
      sum += value;
      count++;
      last_seen_ms = timestamp;
    }

    vTaskDelay(pdMS_TO_TICKS(15));
  }

  if (count == 0) {
    return false;
  }

  *average_mm = (int)lroundf((float)sum / (float)count);
  return true;
}

static bool apply_draft_calibration(int tof_number) {
  TofCalibrationDraft &draft = draft_for_tof(tof_number);
  return set_tof_calibration(tof_number,
                             draft.meas_fov,
                             draft.real_fov,
                             draft.meas_0,
                             draft.meas_72,
                             draft.meas_145);
}

static void reset_calibration_drafts(void) {
  calibration_tof1 = TofCalibrationDraft();
  calibration_tof2 = TofCalibrationDraft();
  calibration_step = CAL_TOF1_FIND_FOV;
  calibration_error_msg = "";
  distance_sensors_calibrated = false;
}

static void save_draft_to_preferences(void) {
  Preferences prefs;
  prefs.begin(CALIBRATION_NAMESPACE, false);
  prefs.putBool(CALIBRATION_DONE_KEY, true);
  prefs.putUInt(CALIBRATION_VERSION_KEY, CALIBRATION_SCHEMA_VERSION);
  prefs.putInt("t1_mf", calibration_tof1.meas_fov);
  prefs.putInt("t1_rf", calibration_tof1.real_fov);
  prefs.putInt("t1_m0", calibration_tof1.meas_0);
  prefs.putInt("t1_m72", calibration_tof1.meas_72);
  prefs.putInt("t1_m145", calibration_tof1.meas_145);
  prefs.putInt("t2_mf", calibration_tof2.meas_fov);
  prefs.putInt("t2_rf", calibration_tof2.real_fov);
  prefs.putInt("t2_m0", calibration_tof2.meas_0);
  prefs.putInt("t2_m72", calibration_tof2.meas_72);
  prefs.putInt("t2_m145", calibration_tof2.meas_145);
  prefs.end();
}

static bool load_draft_from_preferences(void) {
  Preferences prefs;
  prefs.begin(CALIBRATION_NAMESPACE, true);
  bool done = prefs.getBool(CALIBRATION_DONE_KEY, false);
  uint32_t version = prefs.getUInt(CALIBRATION_VERSION_KEY, 0);
  bool compatible_calibration = done && (version == CALIBRATION_SCHEMA_VERSION);

  if (compatible_calibration) {
    calibration_tof1.meas_fov = prefs.getInt("t1_mf", 145);
    calibration_tof1.real_fov = prefs.getInt("t1_rf", 145);
    calibration_tof1.meas_0 = prefs.getInt("t1_m0", 0);
    calibration_tof1.meas_72 = prefs.getInt("t1_m72", INFINITE_TOF_VALUE);
    calibration_tof1.meas_145 = prefs.getInt("t1_m145", INFINITE_TOF_VALUE);
    calibration_tof2.meas_fov = prefs.getInt("t2_mf", 145);
    calibration_tof2.real_fov = prefs.getInt("t2_rf", 145);
    calibration_tof2.meas_0 = prefs.getInt("t2_m0", 0);
    calibration_tof2.meas_72 = prefs.getInt("t2_m72", INFINITE_TOF_VALUE);
    calibration_tof2.meas_145 = prefs.getInt("t2_m145", INFINITE_TOF_VALUE);
  }

  prefs.end();
  return compatible_calibration;
}

static bool load_calibration_done(void) {
  return load_draft_from_preferences();
}

static void save_calibration_done(bool done) {
  Preferences prefs;
  prefs.begin(CALIBRATION_NAMESPACE, false);
  prefs.putBool(CALIBRATION_DONE_KEY, done);
  prefs.putUInt(CALIBRATION_VERSION_KEY, done ? CALIBRATION_SCHEMA_VERSION : 0);
  prefs.end();
}

static bool should_calibrate_target(const TofCalibrationDraft &draft, int target_mm) {
  return draft.real_fov > target_mm;
}

static CalibrationStep next_step_after_fov_distance(int tof_number) {
  TofCalibrationDraft &draft = draft_for_tof(tof_number);

  if (should_calibrate_target(draft, 145)) {
    return (tof_number == TOF1) ? CAL_TOF1_PLACE_145 : CAL_TOF2_PLACE_145;
  }

  if (should_calibrate_target(draft, 72)) {
    return (tof_number == TOF1) ? CAL_TOF1_PLACE_72 : CAL_TOF2_PLACE_72;
  }

  return (tof_number == TOF1) ? CAL_TOF1_PLACE_0 : CAL_TOF2_PLACE_0;
}

static CalibrationStep next_step_after_target(int tof_number, int completed_target_mm) {
  TofCalibrationDraft &draft = draft_for_tof(tof_number);

  if (completed_target_mm == 145 && should_calibrate_target(draft, 72)) {
    return (tof_number == TOF1) ? CAL_TOF1_PLACE_72 : CAL_TOF2_PLACE_72;
  }

  if (completed_target_mm == 145 || completed_target_mm == 72) {
    return (tof_number == TOF1) ? CAL_TOF1_PLACE_0 : CAL_TOF2_PLACE_0;
  }

  if (apply_draft_calibration(tof_number)) {
    return (tof_number == TOF1) ? CAL_TOF2_FIND_FOV : CAL_VERIFY;
  }

  calibration_error_msg = (tof_number == TOF1) ? "Calibration TOF 1 invalide." : "Calibration TOF 2 invalide.";
  return CAL_ERROR;
}

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

static void send_calibration_state(void) {
  int tof_number = current_calibration_tof();
  int raw_mm = (tof_number == 0) ? -1 : raw_tof_value(tof_number);
  bool raw_valid = raw_tof_is_valid(raw_mm);
  int visual_pos = (calibration_step == CAL_VERIFY) ? get_ball_position()
                                                   : visual_position_from_raw(tof_number, raw_mm);

  const char *step_name = "unknown";
  const char *title = "Calibration";
  const char *instruction = "";
  bool needs_done = false;
  bool needs_real_input = false;
  int real_fov = 145;

  switch (calibration_step) {
    case CAL_TOF1_FIND_FOV:
      step_name = "tof1_fov";
      title = "TOF 1 - Find TOF's FOV";
      instruction = "Eloignez progressivement la balle du TOF 1. Quand la linearite disparait ou que la mesure devient limite, cliquez sur Done.";
      needs_done = true;
      break;
    case CAL_TOF1_ENTER_FOV_DISTANCE:
      step_name = "tof1_real_fov";
      title = "TOF 1 - Distance reelle au FOV";
      instruction = "Mesurez avec une regle la vraie distance de la balle au TOF 1, puis entrez la valeur en mm.";
      needs_real_input = true;
      real_fov = calibration_tof1.real_fov;
      break;
    case CAL_TOF1_PLACE_145:
      step_name = "tof1_145";
      title = "TOF 1 - Point 145 mm";
      instruction = "Placez la balle a 145 mm du TOF 1. Quand elle est stable, cliquez sur Done.";
      needs_done = true;
      break;
    case CAL_TOF1_PLACE_72:
      step_name = "tof1_72";
      title = "TOF 1 - Point 72 mm";
      instruction = "Placez la balle a 72 mm du TOF 1. Quand elle est stable, cliquez sur Done.";
      needs_done = true;
      break;
    case CAL_TOF1_PLACE_0:
      step_name = "tof1_0";
      title = "TOF 1 - Point 0 mm";
      instruction = "Placez la balle a 0 mm du TOF 1. Quand elle est stable, cliquez sur Done.";
      needs_done = true;
      break;
    case CAL_TOF2_FIND_FOV:
      step_name = "tof2_fov";
      title = "TOF 2 - Find TOF's FOV";
      instruction = "Eloignez progressivement la balle du TOF 2. Quand la linearite disparait ou que la mesure devient limite, cliquez sur Done.";
      needs_done = true;
      break;
    case CAL_TOF2_ENTER_FOV_DISTANCE:
      step_name = "tof2_real_fov";
      title = "TOF 2 - Distance reelle au FOV";
      instruction = "Mesurez avec une regle la vraie distance de la balle au TOF 2, puis entrez la valeur en mm.";
      needs_real_input = true;
      real_fov = calibration_tof2.real_fov;
      break;
    case CAL_TOF2_PLACE_145:
      step_name = "tof2_145";
      title = "TOF 2 - Point 145 mm";
      instruction = "Placez la balle a 145 mm du TOF 2. Quand elle est stable, cliquez sur Done.";
      needs_done = true;
      break;
    case CAL_TOF2_PLACE_72:
      step_name = "tof2_72";
      title = "TOF 2 - Point 72 mm";
      instruction = "Placez la balle a 72 mm du TOF 2. Quand elle est stable, cliquez sur Done.";
      needs_done = true;
      break;
    case CAL_TOF2_PLACE_0:
      step_name = "tof2_0";
      title = "TOF 2 - Point 0 mm";
      instruction = "Placez la balle a 0 mm du TOF 2. Quand elle est stable, cliquez sur Done.";
      needs_done = true;
      break;
    case CAL_VERIFY:
      step_name = "verify";
      title = "Verification de la calibration";
      instruction = "Verifiez que la position calibree est coherente. Les fleches vertes indiquent 0, 72 et 145 mm. Validez si tout est correct.";
      tof_number = 0;
      break;
    case CAL_ERROR:
      step_name = "error";
      title = "Erreur de calibration";
      instruction = "La calibration a echoue. Cliquez sur Restart pour recommencer.";
      tof_number = 0;
      break;
  }

  String json = "{";
  json += "\"step\":\"" + String(step_name) + "\",";
  json += "\"title\":\"" + String(title) + "\",";
  json += "\"instruction\":\"" + String(instruction) + "\",";
  json += "\"tof\":" + String(tof_number) + ",";
  json += "\"raw_mm\":" + String(raw_mm) + ",";
  json += "\"raw_valid\":" + String(raw_valid ? "true" : "false") + ",";
  json += "\"visual_pos_mm\":" + String(visual_pos) + ",";
  json += "\"needs_done\":" + String(needs_done ? "true" : "false") + ",";
  json += "\"needs_real_input\":" + String(needs_real_input ? "true" : "false") + ",";
  json += "\"real_fov\":" + String(real_fov) + ",";
  json += "\"status\":\"Calibration non finalisee\",";
  json += "\"error\":\"" + calibration_error_msg + "\",";
  json += "\"done\":" + String(distance_sensors_calibrated ? "true" : "false");
  json += "}";

  server.send(200, "application/json; charset=utf-8", json);
}

static void handle_calibration_action(void) {
  String cmd = server.arg("cmd");
  calibration_error_msg = "";

  if (cmd == "restart") {
    reset_calibration_drafts();
    save_calibration_done(false);
    send_calibration_state();
    return;
  }

  if (cmd == "accept" && calibration_step == CAL_VERIFY) {
    save_draft_to_preferences();
    distance_sensors_calibrated = true;
    send_calibration_state();
    return;
  }

  if (cmd == "real_fov") {
    int tof_number = current_calibration_tof();
    int real_fov = server.arg("value").toInt();

    if (tof_number == 0 || real_fov <= 0) {
      calibration_error_msg = "Distance FOV invalide.";
      send_calibration_state();
      return;
    }

    draft_for_tof(tof_number).real_fov = real_fov;
    calibration_step = next_step_after_fov_distance(tof_number);
    send_calibration_state();
    return;
  }

  if (cmd == "done") {
    int tof_number = current_calibration_tof();
    int average_mm = -1;

    if (tof_number == 0 || !capture_raw_average(tof_number, &average_mm)) {
      calibration_error_msg = "Impossible de moyenner une mesure brute valide.";
      send_calibration_state();
      return;
    }

    TofCalibrationDraft &draft = draft_for_tof(tof_number);

    switch (calibration_step) {
      case CAL_TOF1_FIND_FOV:
      case CAL_TOF2_FIND_FOV:
        draft.meas_fov = average_mm;
        calibration_step = (tof_number == TOF1) ? CAL_TOF1_ENTER_FOV_DISTANCE : CAL_TOF2_ENTER_FOV_DISTANCE;
        break;

      case CAL_TOF1_PLACE_145:
      case CAL_TOF2_PLACE_145:
        draft.meas_145 = average_mm;
        calibration_step = next_step_after_target(tof_number, 145);
        break;

      case CAL_TOF1_PLACE_72:
      case CAL_TOF2_PLACE_72:
        draft.meas_72 = average_mm;
        calibration_step = next_step_after_target(tof_number, 72);
        break;

      case CAL_TOF1_PLACE_0:
      case CAL_TOF2_PLACE_0:
        draft.meas_0 = average_mm;
        calibration_step = next_step_after_target(tof_number, 0);
        break;

      default:
        calibration_error_msg = "Action Done impossible a cette etape.";
        break;
    }

    send_calibration_state();
    return;
  }

  calibration_error_msg = "Commande de calibration inconnue.";
  send_calibration_state();
}

static void setup_routes(void) {
  server.on("/", HTTP_GET, []() {
    if (distance_sensors_calibrated) {
      server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
    } else {
      server.send_P(200, "text/html; charset=utf-8", CALIBRATION_HTML);
    }
  });

  server.on("/calibration", HTTP_GET, []() {
    server.send_P(200, "text/html; charset=utf-8", CALIBRATION_HTML);
  });

  server.on("/api/calibrate", HTTP_GET, []() {
    handle_calibration_action();
  });

  server.on("/api/calibration/state", HTTP_GET, []() {
    send_calibration_state();
  });

  server.on("/api/calibration/action", HTTP_GET, []() {
    handle_calibration_action();
  });

  server.on("/api/state", HTTP_GET, []() {
    if (!distance_sensors_calibrated) {
      server.send(423, "application/json; charset=utf-8", "{\"calibrated\":false}");
      return;
    }
    send_state();
  });

  server.on("/api/control", HTTP_GET, []() {
    if (!distance_sensors_calibrated) {
      server.send(423, "application/json; charset=utf-8", "{\"calibrated\":false}");
      return;
    }

    if (server.hasArg("stabilization")) {
      set_controller_enabled(server.arg("stabilization").toInt() != 0);
    }

    if (server.hasArg("angle")) {
      set_controller_manual_angle(server.arg("angle").toInt());
    }

    send_state();
  });

  server.on("/api/params", HTTP_GET, []() {
    if (!distance_sensors_calibrated) {
      server.send(423, "application/json; charset=utf-8", "{\"calibrated\":false}");
      return;
    }

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

  distance_sensors_calibrated = load_calibration_done();
  if (distance_sensors_calibrated) {
    apply_draft_calibration(TOF1);
    apply_draft_calibration(TOF2);
  }

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
