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
static const uint32_t CALIBRATION_SCHEMA_VERSION = 6;
static const char *CONTROLLER_NAMESPACE = "ctrl";
static const char *CONTROLLER_VERSION_KEY = "version";
static const uint32_t CONTROLLER_SCHEMA_VERSION = 1;
static const uint32_t CONTROLLER_SAVE_COOLDOWN_MS = 2000;
static const float CONTROLLER_SAVE_FLOAT_EPSILON = 0.000001f;
static const char *ADVANCED_NAMESPACE = "advanced";
static const char *ADVANCED_VERSION_KEY = "version";
static const uint32_t ADVANCED_SCHEMA_VERSION = 1;
static const uint32_t ADVANCED_SAVE_COOLDOWN_MS = 2000;

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

enum CalibrationMode {
  CAL_MODE_INITIAL_BOTH,
  CAL_MODE_MANUAL_TOF1,
  CAL_MODE_MANUAL_TOF2,
  CAL_MODE_VERIFY_ONLY,
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
static CalibrationMode calibration_mode = CAL_MODE_INITIAL_BOTH;
static bool calibration_flow_done = false;
static String calibration_error_msg = "";
static uint32_t last_controller_save_request_ms = 0;
static uint32_t last_advanced_save_request_ms = 0;

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
.topBtn,.gearBtn{position:absolute;top:10px;width:52px;height:44px;border:3px solid var(--line);background:#f8f5ea;font-weight:900;font-size:24px}
.topBtn{left:10px}.gearBtn{right:10px}
.settingsMenu{position:absolute;top:62px;right:10px;display:none;min-width:190px;background:#fffdf6;border:3px solid var(--line);padding:8px;z-index:5}
.settingsMenu.open{display:grid;gap:8px}.menuBtn{border:3px solid var(--line);background:#f8f5ea;font-weight:800;font-size:16px;padding:9px 10px;text-align:left}
.sectionTitle{text-align:center;font-size:25px;font-weight:800;margin:0 0 8px}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}.plotBox{height:300px;position:relative}.plotBox canvas{width:100%;height:100%}
.plotLabel{position:absolute;top:8px;right:50px;font-weight:800;text-decoration:underline}.plotBtn{position:absolute;top:8px;right:8px;width:34px;height:30px;border:3px solid var(--line);background:#f8f5ea;font-weight:900}
.controls{display:grid;grid-template-columns:1fr 1fr;gap:10px}.formGrid{display:grid;grid-template-columns:auto 1fr;gap:10px;align-items:center}
.saveRow{display:flex;justify-content:flex-end;gap:8px;margin-top:10px}.saveBtn{border:3px solid var(--line);background:#f8f5ea;font-weight:900;font-size:16px;padding:8px 12px}
label{font-weight:800}.field{height:38px;border:3px solid var(--line);background:white;font-size:18px;padding:3px 8px;width:100%}
.stabHeader{display:flex;justify-content:space-between;align-items:center;gap:12px;font-size:24px;font-weight:800}.toggle{min-width:46px;height:38px;border:3px solid var(--line);background:#f8f5ea;font-size:24px;font-weight:900;padding:0 10px}
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
    <button class="gearBtn" id="settingsBtn">&#9881;</button>
    <div class="settingsMenu" id="settingsMenu">
      <button class="menuBtn" id="calibrateBtn">Calibrate TOFs</button>
      <button class="menuBtn" id="advancedBtn">Advanced parameters</button>
    </div>
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
      <div class="saveRow"><button class="saveBtn" id="resetValuesBtn">Reset</button><button class="saveBtn" id="saveValuesBtn">Save values</button></div>
      <div class="small" id="saveStatus">Values are saved only when requested.</div>
    </div>
    <div class="panel">
      <div class="stabHeader"><span>Stabilization</span><button class="toggle" id="stabToggle">▶</button></div>
      <div class="manualTitle">Manual Ctrl</div>
      <div class="sliderRow"><span>0</span><input id="manualSlider" type="range" min="0" max="180" step="1"><span>180</span></div>
      <div class="saveRow"><button class="saveBtn" id="neutralBtn">Neutral pos</button></div>
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
let TABLE_LEN_MM=290; const REFRESH_MS=55, MAX_PLOT_S=30;
const scene=document.getElementById('scene'), anglePlot=document.getElementById('anglePlot'), posPlot=document.getElementById('posPlot'), speedPlot=document.getElementById('speedPlot');
const scenePause=document.getElementById('scenePause'), plotToggle=document.getElementById('plotToggle'), plotInfo=document.getElementById('plotInfo');
const settingsBtn=document.getElementById('settingsBtn'), settingsMenu=document.getElementById('settingsMenu'), calibrateBtn=document.getElementById('calibrateBtn'), advancedBtn=document.getElementById('advancedBtn');
const anglePause=document.getElementById('anglePause'), posPause=document.getElementById('posPause'), speedPause=document.getElementById('speedPause');
const stabToggle=document.getElementById('stabToggle'), manualSlider=document.getElementById('manualSlider');
const refInput=document.getElementById('refInput'), kpInput=document.getElementById('kpInput'), kiInput=document.getElementById('kiInput'), kdInput=document.getElementById('kdInput');
const saveValuesBtn=document.getElementById('saveValuesBtn'), resetValuesBtn=document.getElementById('resetValuesBtn'), neutralBtn=document.getElementById('neutralBtn'), saveStatus=document.getElementById('saveStatus');
const servoTxt=document.getElementById('servoTxt'), tableTxt=document.getElementById('tableTxt'), xTxt=document.getElementById('xTxt'), vTxt=document.getElementById('vTxt'), d1Txt=document.getElementById('d1Txt'), d2Txt=document.getElementById('d2Txt');
let state={x:-1,v:0,speed_valid:false,servo_angle:90,stabilization:true,kp:0,ki:0,kd:0,ref:150,d1:-1,d2:-1,servo_min:0,servo_max:180,servo_neutral:90};
let sceneFrozen=false, plotRunning=false, angleFrozen=false, posFrozen=false, speedFrozen=false, plotStart=0, angleData=[], posData=[], speedData=[], lastStab=true, editing=false, neutralTimer=null;
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
if(label==='speed'&&vals.length){
  const mn=Math.min(...vals),mx=Math.max(...vals);
  c.font=`${Math.max(11,w*.021)}px Arial`;
  c.fillText(`min ${mn.toFixed(0)} / max ${mx.toFixed(0)} mm/s`,p+8,h-12);
}
}
function drawAll(){if(!sceneFrozen)drawScene();if(!angleFrozen)drawPlot(anglePlot,angleData,'#c43131','angle',false);if(!posFrozen)drawPlot(posPlot,posData,'#2457b8','pos',false);if(!speedFrozen)drawPlot(speedPlot,speedData,'#208444','speed',false)}
function updateTexts(){if(state.stabilization&&neutralTimer){clearInterval(neutralTimer);neutralTimer=null}servoTxt.textContent=state.servo_angle;tableTxt.textContent=(state.servo_angle/2).toFixed(1);xTxt.textContent=state.x>=0?state.x:'--';vTxt.textContent=state.speed_valid?state.v:'--';d1Txt.textContent=state.d1>=0?state.d1:'--';d2Txt.textContent=state.d2>=0?state.d2:'--';stabToggle.textContent=state.stabilization?'||':'▶';manualSlider.min=state.servo_min||0;manualSlider.max=state.servo_max||180;manualSlider.disabled=state.stabilization;neutralBtn.disabled=state.stabilization;if(!editing){refInput.value=state.ref;kpInput.value=Number(state.kp).toFixed(3);kiInput.value=Number(state.ki).toFixed(3);kdInput.value=Number(state.kd).toFixed(3)}if(lastStab&&!state.stabilization)manualSlider.value=state.servo_angle;lastStab=state.stabilization}
async function fetchState(){try{const r=await fetch('/api/state',{cache:'no-store'});state=await r.json();TABLE_LEN_MM=state.table_length||290;updateTexts();if(plotRunning){const t=(performance.now()-plotStart)/1000;if(t<=MAX_PLOT_S){angleData.push({t,y:state.servo_angle/2});posData.push({t,y:state.x>=0?state.x:NaN});speedData.push({t,y:state.speed_valid?state.v:NaN});plotInfo.textContent=`Plot running: ${t.toFixed(1)} s / 30 s`}else{plotRunning=false;plotToggle.textContent='Go';plotInfo.textContent='30 s reached. Plot stopped.'}}drawAll()}catch(e){}}
function startPlot(){angleData=[];posData=[];speedData=[];plotStart=performance.now();plotRunning=true;angleFrozen=false;posFrozen=false;speedFrozen=false;plotToggle.textContent='Stop';plotInfo.textContent='Plot running'}
function stopPlot(){plotRunning=false;plotToggle.textContent='Go';plotInfo.textContent='Plot frozen. Press Go to restart from 0.'}
plotToggle.onclick=()=>plotRunning?stopPlot():startPlot();scenePause.onclick=()=>{sceneFrozen=!sceneFrozen;scenePause.textContent=sceneFrozen?'▶':'||'};anglePause.onclick=()=>{angleFrozen=!angleFrozen;anglePause.textContent=angleFrozen?'▶':'||'};posPause.onclick=()=>{posFrozen=!posFrozen;posPause.textContent=posFrozen?'▶':'||'};speedPause.onclick=()=>{speedFrozen=!speedFrozen;speedPause.textContent=speedFrozen?'▶':'||'};
stabToggle.onclick=async()=>{const en=state.stabilization?0:1;await fetch(`/api/control?stabilization=${en}`,{cache:'no-store'});if(!en)manualSlider.value=state.servo_angle;fetchState()};
manualSlider.oninput=()=>{servoTxt.textContent=manualSlider.value;tableTxt.textContent=(Number(manualSlider.value)/2).toFixed(1)};
manualSlider.onchange=()=>{fetch(`/api/control?angle=${manualSlider.value}`,{cache:'no-store'}).then(fetchState)};
[refInput,kpInput,kiInput,kdInput].forEach(el=>{el.onfocus=()=>editing=true;el.onblur=()=>editing=false;el.onchange=()=>{const q=`ref=${refInput.value}&kp=${kpInput.value}&ki=${kiInput.value}&kd=${kdInput.value}`;fetch(`/api/params?${q}`,{cache:'no-store'}).then(fetchState)}});
settingsBtn.onclick=()=>settingsMenu.classList.toggle('open');
calibrateBtn.onclick=()=>{location.href='/calibration_select'};
advancedBtn.onclick=()=>{location.href='/advanced'};
saveValuesBtn.onclick=async()=>{
  const q=`ref=${refInput.value}&kp=${kpInput.value}&ki=${kiInput.value}&kd=${kdInput.value}`;
  saveValuesBtn.disabled=true;saveStatus.textContent='Saving...';
  try{await fetch(`/api/params/save?${q}`,{cache:'no-store'});saveStatus.textContent='Saved.';fetchState()}catch(e){saveStatus.textContent='Save failed.'}
  setTimeout(()=>{saveValuesBtn.disabled=false},2000);
};
resetValuesBtn.onclick=async()=>{saveStatus.textContent='Resetting...';try{await fetch('/api/params/reload',{cache:'no-store'});saveStatus.textContent='Restored saved values.';fetchState()}catch(e){saveStatus.textContent='Reset failed.'}};
neutralBtn.onclick=()=>{
  if(state.stabilization)return;
  if(neutralTimer)clearInterval(neutralTimer);
  const start=Number(manualSlider.value||state.servo_angle),target=Number(state.servo_neutral||90),duration=900,period=45,t0=performance.now();
  neutralBtn.disabled=true;
  neutralTimer=setInterval(()=>{
    const u=Math.min(1,(performance.now()-t0)/duration);
    const angle=Math.round(start+(target-start)*u);
    manualSlider.value=angle;servoTxt.textContent=angle;tableTxt.textContent=(angle/2).toFixed(1);
    fetch(`/api/control?angle=${angle}`,{cache:'no-store'});
    if(u>=1){clearInterval(neutralTimer);neutralTimer=null;fetchState()}
  },period);
};
window.onresize=drawAll;setInterval(fetchState,REFRESH_MS);fetchState();
</script>
</body>
</html>
)rawliteral";

static const char WELCOME_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PID Table Workshop</title>
<style>
body{margin:0;min-height:100vh;background:#f4f1e8;color:#171717;font-family:Arial,Helvetica,sans-serif;display:grid;place-items:center;padding:18px}
.panel{width:min(760px,100%);background:#fffdf6;border:3px solid #202020;border-radius:4px;padding:28px;text-align:center}
h1{font-size:30px;line-height:1.2;margin:0 0 18px}.msg{font-size:20px;line-height:1.45;margin:0 0 24px}
button{border:3px solid #202020;background:#f8f5ea;color:#171717;font-size:26px;font-weight:900;padding:12px 34px;cursor:pointer}
</style>
</head>
<body>
<section class="panel">
<h1>Merci d'avoir participe au Workshop PID-TABLE de ROBOVALY.</h1>
<p class="msg">Pour continuer, veuillez callibrer les capteurs de distance du systeme.</p>
<button id="startBtn">START</button>
</section>
<script>
document.getElementById('startBtn').onclick=async()=>{
  await fetch('/api/calibration/action?cmd=start&mode=initial',{cache:'no-store'});
  location.href='/calibration?initial=1';
};
</script>
</body>
</html>
)rawliteral";

static const char CALIBRATION_SELECT_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Calibration TOF</title>
<style>
body{margin:0;min-height:100vh;background:#f4f1e8;color:#171717;font-family:Arial,Helvetica,sans-serif;display:grid;place-items:center;padding:18px}
.panel{width:min(680px,100%);background:#fffdf6;border:3px solid #202020;border-radius:4px;padding:24px}
h1{font-size:28px;margin:0 0 12px}.msg{font-size:18px;line-height:1.4;color:#555;margin:0 0 22px}.row{display:grid;grid-template-columns:1fr 1fr;gap:12px}
button,a{border:3px solid #202020;background:#f8f5ea;color:#171717;font-size:22px;font-weight:900;padding:14px 16px;cursor:pointer;text-align:center;text-decoration:none}
.back{display:block;margin-top:14px;font-size:18px}
@media(max-width:560px){.row{grid-template-columns:1fr}}
</style>
</head>
<body>
<section class="panel">
<h1>Calibration des TOFs</h1>
<p class="msg">Choisissez le capteur a recalibrer. Les anciens parametres restent utilises tant que la nouvelle calibration n'est pas validee.</p>
<div class="row">
<button onclick="location.href='/calibration?manual=1&target=1'">Calibrate TOF 1</button>
<button onclick="location.href='/calibration?manual=1&target=2'">Calibrate TOF 2</button>
</div>
<button class="back" onclick="location.href='/calibration?verify=1'">Verify calibration</button>
<a class="back" href="/">Retour</a>
</section>
</body>
</html>
)rawliteral";

static const char ADVANCED_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Advanced parameters</title>
<style>
body{margin:0;min-height:100vh;background:#f4f1e8;color:#171717;font-family:Arial,Helvetica,sans-serif;padding:14px}
.app{width:min(980px,100%);margin:0 auto;display:grid;gap:12px}.panel{background:#fffdf6;border:3px solid #202020;border-radius:4px;padding:16px}
h1{font-size:28px;margin:0 0 10px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}.form{display:grid;grid-template-columns:1fr 150px;gap:10px;align-items:center}
label{font-weight:800}input{height:38px;border:3px solid #202020;background:white;font-size:18px;padding:4px 8px;width:100%}
button,a{border:3px solid #202020;background:#f8f5ea;color:#171717;font-size:18px;font-weight:900;padding:10px 14px;cursor:pointer;text-decoration:none;text-align:center}.actions{display:flex;gap:10px;justify-content:flex-end;flex-wrap:wrap}.small{font-size:13px;color:#666}
@media(max-width:760px){.grid{grid-template-columns:1fr}.form{grid-template-columns:1fr}}
</style>
</head>
<body>
<div class="app">
<section class="panel"><h1>Advanced parameters</h1><div class="small" id="status">Runtime parameters. Use reset to restore development defaults.</div></section>
<div class="grid">
<section class="panel"><h1>Position and speed filtering</h1><div class="form">
<label>Position moving average window</label><input id="posWin" type="number" min="1" max="20" step="1">
<label>Speed moving average window</label><input id="speedWin" type="number" min="1" max="20" step="1">
</div></section>
<section class="panel"><h1>PID controller</h1><div class="form">
<label>Max angle step / cycle</label><input id="maxStep" type="number" min="0" max="180" step="1">
<label>Position precision</label><input id="posDb" type="number" min="0" max="50" step="1">
<label>Speed precision</label><input id="speedDb" type="number" min="0" max="300" step="1">
<label>Lost ball delay</label><input id="lostDelay" type="number" min="0" max="10000" step="100">
<label>Lost ball iter</label><input id="lostIter" type="number" min="1" max="20" step="1">
</div></section>
<section class="panel"><h1>Servo</h1><div class="form">
<label>Min angle</label><input id="servoMin" type="number" min="0" max="180" step="1">
<label>Max angle</label><input id="servoMax" type="number" min="0" max="180" step="1">
<label>Neutral angle</label><input id="servoNeutral" type="number" min="0" max="180" step="1">
</div></section>
<section class="panel"><h1>Geometry</h1><div class="form">
<label>Table length</label><input id="tableLen" type="number" min="1" max="300" step="1">
</div></section>
</div>
<section class="panel actions">
<button id="resetBtn">RESET TO DEFAULT VALUES</button>
<button id="saveBtn">Save parameters</button>
<button id="applyBtn">Apply</button>
<a href="/">Back</a>
</section>
</div>
<script>
const ids=['posWin','speedWin','maxStep','posDb','speedDb','lostDelay','lostIter','servoMin','servoMax','servoNeutral','tableLen'];
const el=Object.fromEntries(ids.map(id=>[id,document.getElementById(id)]));
const statusEl=document.getElementById('status');
const saveBtn=document.getElementById('saveBtn');
function fill(s){el.posWin.value=s.position_window;el.speedWin.value=s.speed_window;el.maxStep.value=s.max_step;el.posDb.value=s.position_deadband;el.speedDb.value=s.speed_deadband;el.lostDelay.value=s.lost_delay;el.lostIter.value=s.lost_iter;el.servoMin.value=s.servo_min;el.servoMax.value=s.servo_max;el.servoNeutral.value=s.servo_neutral;el.tableLen.value=s.table_length}
async function load(){try{const r=await fetch('/api/advanced',{cache:'no-store'});fill(await r.json())}catch(e){statusEl.textContent='Load failed.'}}
function query(){return `pos_win=${el.posWin.value}&speed_win=${el.speedWin.value}&max_step=${el.maxStep.value}&pos_db=${el.posDb.value}&speed_db=${el.speedDb.value}&lost_delay=${el.lostDelay.value}&lost_iter=${el.lostIter.value}&servo_min=${el.servoMin.value}&servo_max=${el.servoMax.value}&servo_neutral=${el.servoNeutral.value}&table_len=${el.tableLen.value}`}
document.getElementById('applyBtn').onclick=async()=>{statusEl.textContent='Applying...';try{const r=await fetch('/api/advanced/set?'+query(),{cache:'no-store'});const s=await r.json();fill(s);statusEl.textContent=s.ok?'Applied.':'Some values were rejected.'}catch(e){statusEl.textContent='Apply failed.'}};
document.getElementById('resetBtn').onclick=async()=>{statusEl.textContent='Resetting...';try{const r=await fetch('/api/advanced/reset',{cache:'no-store'});fill(await r.json());statusEl.textContent='Defaults restored.'}catch(e){statusEl.textContent='Reset failed.'}};
saveBtn.onclick=async()=>{statusEl.textContent='Saving...';saveBtn.disabled=true;try{const r=await fetch('/api/advanced/save?'+query(),{cache:'no-store'});const s=await r.json();fill(s);statusEl.textContent=s.ok?'Saved.':'Some values were rejected.'}catch(e){statusEl.textContent='Save failed.'}setTimeout(()=>{saveBtn.disabled=false},2000)};
load();
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
<section class="panel scene"><canvas id="calScene"></canvas></section>
<section class="panel">
<h1 id="title">Calibration</h1>
<p class="msg" id="instruction">Chargement...</p>
<div class="row" id="rawRow"><span class="dot" id="dot"></span><b id="rawTxt">--</b><span id="tofTxt">--</span></div>
<div class="row" id="realRow"><label for="realInput"><b>Distance reelle au FOV max</b></label><input id="realInput" type="number" min="1" max="400" step="1"><span>mm</span></div>
<div class="row">
<button id="doneBtn">Done</button>
<button id="submitBtn">Valider distance</button>
<button id="acceptBtn">Valider calibration</button>
<button id="restartBtn">Restart</button>
<button id="cancelBtn">Cancel</button>
</div>
<div class="hint" id="status">Les valeurs sont sauvegardees apres validation finale.</div>
</section>
</div>
<script>
const TABLE_LEN_MM=290, REFRESH_MS=80;
const scene=document.getElementById('calScene'),dot=document.getElementById('dot'),rawTxt=document.getElementById('rawTxt'),tofTxt=document.getElementById('tofTxt'),rawRow=document.getElementById('rawRow');
const title=document.getElementById('title'),instruction=document.getElementById('instruction'),statusEl=document.getElementById('status');
const realRow=document.getElementById('realRow'),realInput=document.getElementById('realInput');
const doneBtn=document.getElementById('doneBtn'),submitBtn=document.getElementById('submitBtn'),acceptBtn=document.getElementById('acceptBtn'),restartBtn=document.getElementById('restartBtn'),cancelBtn=document.getElementById('cancelBtn');
let s={};
const params=new URLSearchParams(location.search);
let startupDone=false;
let realInputEditing=false;
let lastInputStep='';
function fit(c){const r=c.getBoundingClientRect(),d=window.devicePixelRatio||1;const w=Math.max(1,Math.floor(r.width*d)),h=Math.max(1,Math.floor(r.height*d));if(c.width!==w||c.height!==h){c.width=w;c.height=h}}
function drawScene(){
fit(scene);const c=scene.getContext('2d'),w=scene.width,h=scene.height;c.clearRect(0,0,w,h);c.lineWidth=4;c.strokeStyle='#171717';c.fillStyle='#171717';
const cx=w*.5,cy=h*.56,len=w*.72,a=0,ux=Math.cos(a),uy=Math.sin(a),nx=0,ny=-1;const x1=cx-len/2,y1=cy,x2=cx+len/2,y2=cy;
c.beginPath();c.moveTo(x1,y1);c.lineTo(x2,y2);c.stroke();c.beginPath();c.moveTo(cx,cy+8);c.lineTo(cx-w*.035,cy+h*.22);c.lineTo(cx+w*.035,cy+h*.22);c.closePath();c.stroke();
let pos=s.visual_pos_mm; if(!Number.isFinite(pos)||pos<0)pos=TABLE_LEN_MM/2; pos=Math.max(0,Math.min(TABLE_LEN_MM,pos)); const p=pos/TABLE_LEN_MM;
const r=Math.max(15,Math.min(w,h)*.045),contactX=x1+(x2-x1)*p,contactY=y1+(y2-y1)*p,bx=contactX+nx*r,by=contactY+ny*r;
c.beginPath();c.arc(bx,by,r,0,Math.PI*2);c.stroke();c.beginPath();c.moveTo(bx-r*.65,by-r*.65);c.lineTo(bx+r*.65,by+r*.65);c.moveTo(bx+r*.65,by-r*.65);c.lineTo(bx-r*.65,by+r*.65);c.stroke();
if(s.step==='verify'){[0,72,145,218,290].forEach(mm=>{const x=x1+(x2-x1)*(mm/TABLE_LEN_MM);c.strokeStyle='#208444';c.fillStyle='#208444';c.lineWidth=3;c.beginPath();c.moveTo(x,cy+30);c.lineTo(x,cy+70);c.stroke();c.beginPath();c.moveTo(x,cy+24);c.lineTo(x-8,cy+42);c.lineTo(x+8,cy+42);c.closePath();c.fill();c.fillText(`${mm}`,x-10,cy+90)});c.strokeStyle='#171717';c.fillStyle='#171717'}
c.font=`${Math.max(14,w*.018)}px Arial`;c.fillText(`visual pos=${Math.round(pos)} mm`,18,h-22);
}
function setButtons(){
const verifyOnly=params.get('verify')==='1';
const verifyStep=s.step==='verify';
doneBtn.classList.toggle('hidden',!s.needs_done);
submitBtn.classList.toggle('hidden',!s.needs_real_input);
realRow.classList.toggle('hidden',!s.needs_real_input);
rawRow.classList.toggle('hidden',verifyStep);
statusEl.classList.toggle('hidden',verifyStep);
acceptBtn.classList.toggle('hidden',s.step!=='verify');
restartBtn.classList.toggle('hidden',false);
cancelBtn.classList.toggle('hidden',params.get('initial')==='1');
if(verifyOnly){
  doneBtn.classList.remove('hidden');
  submitBtn.classList.add('hidden');
  acceptBtn.classList.add('hidden');
  restartBtn.classList.add('hidden');
  cancelBtn.classList.add('hidden');
  realRow.classList.add('hidden');
  doneBtn.textContent='Done';
}else{
  doneBtn.textContent='Done';
}
if(s.needs_real_input && !realInputEditing && (s.step!==lastInputStep || !realInput.value)){
  realInput.value=s.real_fov||145;
}
lastInputStep=s.step||'';
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
async function ensureStarted(){
if(startupDone)return;
startupDone=true;
if(params.get('initial')==='1')await fetch('/api/calibration/action?cmd=start&mode=initial',{cache:'no-store'});
else if(params.get('verify')==='1')await fetch('/api/calibration/action?cmd=start&mode=verify',{cache:'no-store'});
else if(params.get('target')==='1')await fetch('/api/calibration/action?cmd=start&target=1',{cache:'no-store'});
else if(params.get('target')==='2')await fetch('/api/calibration/action?cmd=start&target=2',{cache:'no-store'});
}
async function getState(){try{await ensureStarted();const r=await fetch('/api/calibration/state',{cache:'no-store'});s=await r.json();update()}catch(e){statusEl.textContent='Erreur reseau'}}
async function action(q){doneBtn.disabled=submitBtn.disabled=acceptBtn.disabled=restartBtn.disabled=cancelBtn.disabled=true;try{const r=await fetch('/api/calibration/action?'+q,{cache:'no-store'});s=await r.json();update();if(s.done)setTimeout(()=>{location.href='/'},600)}catch(e){statusEl.textContent='Erreur action'}doneBtn.disabled=submitBtn.disabled=acceptBtn.disabled=restartBtn.disabled=cancelBtn.disabled=false}
realInput.onfocus=()=>{realInputEditing=true};
realInput.onblur=()=>{realInputEditing=false};
doneBtn.onclick=()=>{if(params.get('verify')==='1')location.href='/calibration_select';else action('cmd=done')};
submitBtn.onclick=()=>action('cmd=real_fov&value='+encodeURIComponent(realInput.value||'145'));
acceptBtn.onclick=()=>action('cmd=accept');
restartBtn.onclick=()=>action('cmd=restart');
cancelBtn.onclick=()=>action('cmd=cancel');
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
  calibration_mode = CAL_MODE_INITIAL_BOTH;
  calibration_flow_done = false;
  calibration_error_msg = "";
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

static void start_initial_calibration(void) {
  reset_calibration_drafts();
  calibration_mode = CAL_MODE_INITIAL_BOTH;
  calibration_step = CAL_TOF1_FIND_FOV;
  calibration_flow_done = false;
  distance_sensors_calibrated = false;
  save_calibration_done(false);
}

static void start_manual_calibration(int tof_number) {
  calibration_error_msg = "";
  calibration_flow_done = false;

  if (!load_draft_from_preferences()) {
    reset_calibration_drafts();
    calibration_error_msg = "Aucune calibration sauvegardee. Faites la calibration initiale complete.";
    distance_sensors_calibrated = false;
    return;
  }

  apply_draft_calibration(TOF1);
  apply_draft_calibration(TOF2);
  distance_sensors_calibrated = true;

  if (tof_number == TOF1) {
    calibration_tof1 = TofCalibrationDraft();
    calibration_mode = CAL_MODE_MANUAL_TOF1;
    calibration_step = CAL_TOF1_FIND_FOV;
  } else {
    calibration_tof2 = TofCalibrationDraft();
    calibration_mode = CAL_MODE_MANUAL_TOF2;
    calibration_step = CAL_TOF2_FIND_FOV;
  }
}

static void restore_saved_calibration(void) {
  calibration_error_msg = "";
  calibration_flow_done = true;

  if (load_draft_from_preferences()) {
    apply_draft_calibration(TOF1);
    apply_draft_calibration(TOF2);
    distance_sensors_calibrated = true;
  } else {
    reset_calibration_drafts();
    distance_sensors_calibrated = false;
  }
}

static int manual_calibration_target(void) {
  if (calibration_mode == CAL_MODE_MANUAL_TOF1) {
    return TOF1;
  }

  if (calibration_mode == CAL_MODE_MANUAL_TOF2) {
    return TOF2;
  }

  return 0;
}

static void start_verify_calibration(void) {
  calibration_error_msg = "";
  calibration_flow_done = false;
  calibration_mode = CAL_MODE_VERIFY_ONLY;
  calibration_step = CAL_VERIFY;

  if (load_draft_from_preferences()) {
    apply_draft_calibration(TOF1);
    apply_draft_calibration(TOF2);
    distance_sensors_calibrated = true;
  } else {
    reset_calibration_drafts();
    calibration_mode = CAL_MODE_VERIFY_ONLY;
    calibration_step = CAL_ERROR;
    distance_sensors_calibrated = false;
    calibration_error_msg = "Aucune calibration sauvegardee a verifier.";
  }
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
    if (calibration_mode == CAL_MODE_INITIAL_BOTH && tof_number == TOF1) {
      return CAL_TOF2_FIND_FOV;
    }

    return CAL_VERIFY;
  }

  calibration_error_msg = (tof_number == TOF1) ? "Calibration TOF 1 invalide." : "Calibration TOF 2 invalide.";
  return CAL_ERROR;
}

static void load_controller_settings(void) {
  Preferences prefs;
  prefs.begin(CONTROLLER_NAMESPACE, true);
  uint32_t version = prefs.getUInt(CONTROLLER_VERSION_KEY, 0);

  if (version == CONTROLLER_SCHEMA_VERSION) {
    float current_kp;
    float current_ki;
    float current_kd;
    get_controller_gains(&current_kp, &current_ki, &current_kd);

    int reference_mm = prefs.getInt("ref", get_controller_reference_mm());
    float kp = prefs.getFloat("kp", current_kp);
    float ki = prefs.getFloat("ki", current_ki);
    float kd = prefs.getFloat("kd", current_kd);

    set_controller_reference_mm(reference_mm);
    set_controller_gains(kp, ki, kd);
  }

  prefs.end();
}

static bool controller_settings_match_saved(void) {
  float kp;
  float ki;
  float kd;
  get_controller_gains(&kp, &ki, &kd);

  Preferences prefs;
  prefs.begin(CONTROLLER_NAMESPACE, true);
  uint32_t version = prefs.getUInt(CONTROLLER_VERSION_KEY, 0);

  if (version != CONTROLLER_SCHEMA_VERSION) {
    prefs.end();
    return false;
  }

  int saved_reference_mm = prefs.getInt("ref", get_controller_reference_mm() + 1);
  float saved_kp = prefs.getFloat("kp", kp + 1.0f);
  float saved_ki = prefs.getFloat("ki", ki + 1.0f);
  float saved_kd = prefs.getFloat("kd", kd + 1.0f);
  prefs.end();

  return saved_reference_mm == get_controller_reference_mm() &&
         fabsf(saved_kp - kp) <= CONTROLLER_SAVE_FLOAT_EPSILON &&
         fabsf(saved_ki - ki) <= CONTROLLER_SAVE_FLOAT_EPSILON &&
         fabsf(saved_kd - kd) <= CONTROLLER_SAVE_FLOAT_EPSILON;
}

static bool save_controller_settings(void) {
  uint32_t now = millis();
  if (last_controller_save_request_ms != 0 &&
      now - last_controller_save_request_ms < CONTROLLER_SAVE_COOLDOWN_MS) {
    return false;
  }
  last_controller_save_request_ms = now;

  if (controller_settings_match_saved()) {
    return false;
  }

  float kp;
  float ki;
  float kd;
  get_controller_gains(&kp, &ki, &kd);

  Preferences prefs;
  prefs.begin(CONTROLLER_NAMESPACE, false);
  prefs.putUInt(CONTROLLER_VERSION_KEY, CONTROLLER_SCHEMA_VERSION);
  prefs.putInt("ref", get_controller_reference_mm());
  prefs.putFloat("kp", kp);
  prefs.putFloat("ki", ki);
  prefs.putFloat("kd", kd);
  prefs.end();
  return true;
}

static void update_controller_params_from_request(void) {
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
  json += "\"kd\":" + String(kd, 6) + ",";
  json += "\"table_length\":" + String(get_table_length_mm()) + ",";
  json += "\"servo_min\":" + String(get_servo_min_angle_deg()) + ",";
  json += "\"servo_max\":" + String(get_servo_max_angle_deg()) + ",";
  json += "\"servo_neutral\":" + String(get_servo_neutral_angle_deg());
  json += "}";

  server.send(200, "application/json; charset=utf-8", json);
}

static void send_advanced_state(bool ok = true) {
  String json = "{";
  json += "\"ok\":" + String(ok ? "true" : "false") + ",";
  json += "\"position_window\":" + String(get_position_filter_window()) + ",";
  json += "\"speed_window\":" + String(get_speed_filter_window()) + ",";
  json += "\"max_step\":" + String(get_controller_max_step_deg()) + ",";
  json += "\"position_deadband\":" + String(get_controller_stabilization_position_deadband_mm()) + ",";
  json += "\"speed_deadband\":" + String(get_controller_stabilization_speed_deadband_mm_s()) + ",";
  json += "\"lost_delay\":" + String(get_controller_lost_ball_delay_ms()) + ",";
  json += "\"lost_iter\":" + String(get_controller_lost_ball_iter()) + ",";
  json += "\"servo_min\":" + String(get_servo_min_angle_deg()) + ",";
  json += "\"servo_max\":" + String(get_servo_max_angle_deg()) + ",";
  json += "\"servo_neutral\":" + String(get_servo_neutral_angle_deg()) + ",";
  json += "\"table_length\":" + String(get_table_length_mm());
  json += "}";
  server.send(200, "application/json; charset=utf-8", json);
}

static bool update_advanced_params_from_request(void) {
  bool ok = true;

  if (server.hasArg("pos_win")) {
    ok = set_position_filter_window(server.arg("pos_win").toInt()) && ok;
  }

  if (server.hasArg("speed_win")) {
    ok = set_speed_filter_window(server.arg("speed_win").toInt()) && ok;
  }

  if (server.hasArg("max_step")) {
    ok = set_controller_max_step_deg(server.arg("max_step").toInt()) && ok;
  }

  if (server.hasArg("pos_db")) {
    ok = set_controller_stabilization_position_deadband_mm(server.arg("pos_db").toInt()) && ok;
  }

  if (server.hasArg("speed_db")) {
    ok = set_controller_stabilization_speed_deadband_mm_s(server.arg("speed_db").toInt()) && ok;
  }

  if (server.hasArg("lost_delay")) {
    ok = set_controller_lost_ball_delay_ms((uint32_t)server.arg("lost_delay").toInt()) && ok;
  }

  if (server.hasArg("lost_iter")) {
    ok = set_controller_lost_ball_iter(server.arg("lost_iter").toInt()) && ok;
  }

  int servo_min = get_servo_min_angle_deg();
  int servo_max = get_servo_max_angle_deg();
  int servo_neutral = get_servo_neutral_angle_deg();

  if (server.hasArg("servo_min")) servo_min = server.arg("servo_min").toInt();
  if (server.hasArg("servo_max")) servo_max = server.arg("servo_max").toInt();
  if (server.hasArg("servo_neutral")) servo_neutral = server.arg("servo_neutral").toInt();
  if (server.hasArg("servo_min") || server.hasArg("servo_max") || server.hasArg("servo_neutral")) {
    ok = set_servo_angle_range(servo_min, servo_max, servo_neutral) && ok;
    reset_controller();
  }

  if (server.hasArg("table_len")) {
    ok = set_table_length_mm(server.arg("table_len").toInt()) && ok;
  }

  return ok;
}

static void reset_all_advanced_parameters(void) {
  reset_ball_position_advanced_parameters();
  reset_servo_advanced_parameters();
  reset_controller_advanced_parameters();
}

static bool advanced_settings_match_saved(void) {
  Preferences prefs;
  prefs.begin(ADVANCED_NAMESPACE, true);
  uint32_t version = prefs.getUInt(ADVANCED_VERSION_KEY, 0);

  if (version != ADVANCED_SCHEMA_VERSION) {
    prefs.end();
    return false;
  }

  bool same = prefs.getInt("pos_win", -1) == get_position_filter_window() &&
              prefs.getInt("speed_win", -1) == get_speed_filter_window() &&
              prefs.getInt("max_step", -1) == get_controller_max_step_deg() &&
              prefs.getInt("pos_db", -1) == get_controller_stabilization_position_deadband_mm() &&
              prefs.getInt("speed_db", -1) == get_controller_stabilization_speed_deadband_mm_s() &&
              prefs.getUInt("lost_delay", UINT32_MAX) == get_controller_lost_ball_delay_ms() &&
              prefs.getInt("lost_iter", -1) == get_controller_lost_ball_iter() &&
              prefs.getInt("servo_min", -1) == get_servo_min_angle_deg() &&
              prefs.getInt("servo_max", -1) == get_servo_max_angle_deg() &&
              prefs.getInt("servo_neutral", -1) == get_servo_neutral_angle_deg() &&
              prefs.getInt("table_len", -1) == get_table_length_mm();

  prefs.end();
  return same;
}

static bool save_advanced_settings(void) {
  uint32_t now = millis();
  if (last_advanced_save_request_ms != 0 &&
      now - last_advanced_save_request_ms < ADVANCED_SAVE_COOLDOWN_MS) {
    return false;
  }
  last_advanced_save_request_ms = now;

  if (advanced_settings_match_saved()) {
    return false;
  }

  Preferences prefs;
  prefs.begin(ADVANCED_NAMESPACE, false);
  prefs.putUInt(ADVANCED_VERSION_KEY, ADVANCED_SCHEMA_VERSION);
  prefs.putInt("pos_win", get_position_filter_window());
  prefs.putInt("speed_win", get_speed_filter_window());
  prefs.putInt("max_step", get_controller_max_step_deg());
  prefs.putInt("pos_db", get_controller_stabilization_position_deadband_mm());
  prefs.putInt("speed_db", get_controller_stabilization_speed_deadband_mm_s());
  prefs.putUInt("lost_delay", get_controller_lost_ball_delay_ms());
  prefs.putInt("lost_iter", get_controller_lost_ball_iter());
  prefs.putInt("servo_min", get_servo_min_angle_deg());
  prefs.putInt("servo_max", get_servo_max_angle_deg());
  prefs.putInt("servo_neutral", get_servo_neutral_angle_deg());
  prefs.putInt("table_len", get_table_length_mm());
  prefs.end();
  return true;
}

static bool load_advanced_settings(void) {
  Preferences prefs;
  prefs.begin(ADVANCED_NAMESPACE, true);
  uint32_t version = prefs.getUInt(ADVANCED_VERSION_KEY, 0);

  if (version != ADVANCED_SCHEMA_VERSION) {
    prefs.end();
    return false;
  }

  int pos_win = prefs.getInt("pos_win", POSITION_FILTER_DEFAULT_WINDOW);
  int speed_win = prefs.getInt("speed_win", SPEED_FILTER_DEFAULT_WINDOW);
  int max_step = prefs.getInt("max_step", CONTROLLER_DEFAULT_MAX_STEP_DEG);
  int pos_db = prefs.getInt("pos_db", CONTROLLER_DEFAULT_POSITION_DEADBAND_MM);
  int speed_db = prefs.getInt("speed_db", CONTROLLER_DEFAULT_SPEED_DEADBAND_MM_S);
  uint32_t lost_delay = prefs.getUInt("lost_delay", CONTROLLER_DEFAULT_LOST_BALL_DELAY_MS);
  int lost_iter = prefs.getInt("lost_iter", CONTROLLER_DEFAULT_LOST_BALL_ITER);
  int servo_min = prefs.getInt("servo_min", SERVO_CMD_MIN_DEG);
  int servo_max = prefs.getInt("servo_max", SERVO_CMD_MAX_DEG);
  int servo_neutral = prefs.getInt("servo_neutral", SERVO_CMD_NEUTRAL_DEG);
  int table_len = prefs.getInt("table_len", TABLE_LENGTH_DEFAULT_MM);
  prefs.end();

  bool ok = true;
  ok = set_position_filter_window(pos_win) && ok;
  ok = set_speed_filter_window(speed_win) && ok;
  ok = set_controller_max_step_deg(max_step) && ok;
  ok = set_controller_stabilization_position_deadband_mm(pos_db) && ok;
  ok = set_controller_stabilization_speed_deadband_mm_s(speed_db) && ok;
  ok = set_controller_lost_ball_delay_ms(lost_delay) && ok;
  ok = set_controller_lost_ball_iter(lost_iter) && ok;
  ok = set_servo_angle_range(servo_min, servo_max, servo_neutral) && ok;
  ok = set_table_length_mm(table_len) && ok;
  reset_controller();
  return ok;
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
      instruction = "Verifiez que la position calibree est coherente. Les fleches vertes indiquent 0, 72, 145, 218 et 290 mm. Validez si tout est correct.";
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
  json += "\"done\":" + String(calibration_flow_done ? "true" : "false");
  json += "}";

  server.send(200, "application/json; charset=utf-8", json);
}

static void handle_calibration_action(void) {
  String cmd = server.arg("cmd");
  calibration_error_msg = "";

  if (cmd == "start") {
    if (server.arg("mode") == "initial") {
      start_initial_calibration();
    } else if (server.arg("mode") == "verify") {
      start_verify_calibration();
    } else if (server.arg("target").toInt() == TOF1) {
      start_manual_calibration(TOF1);
    } else if (server.arg("target").toInt() == TOF2) {
      start_manual_calibration(TOF2);
    } else {
      calibration_error_msg = "Demarrage de calibration invalide.";
    }

    send_calibration_state();
    return;
  }

  if (cmd == "cancel") {
    restore_saved_calibration();
    send_calibration_state();
    return;
  }

  if (cmd == "restart") {
    int manual_target = manual_calibration_target();

    if (calibration_mode == CAL_MODE_INITIAL_BOTH || manual_target == 0) {
      start_initial_calibration();
    } else {
      start_manual_calibration(manual_target);
    }

    send_calibration_state();
    return;
  }

  if (cmd == "accept" && calibration_step == CAL_VERIFY) {
    save_draft_to_preferences();
    distance_sensors_calibrated = true;
    calibration_flow_done = true;
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
      server.send_P(200, "text/html; charset=utf-8", WELCOME_HTML);
    }
  });

  server.on("/calibration", HTTP_GET, []() {
    server.send_P(200, "text/html; charset=utf-8", CALIBRATION_HTML);
  });

  server.on("/calibration_select", HTTP_GET, []() {
    if (distance_sensors_calibrated) {
      server.send_P(200, "text/html; charset=utf-8", CALIBRATION_SELECT_HTML);
    } else {
      server.send_P(200, "text/html; charset=utf-8", WELCOME_HTML);
    }
  });

  server.on("/advanced", HTTP_GET, []() {
    if (distance_sensors_calibrated) {
      server.send_P(200, "text/html; charset=utf-8", ADVANCED_HTML);
    } else {
      server.send_P(200, "text/html; charset=utf-8", WELCOME_HTML);
    }
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

  server.on("/api/advanced", HTTP_GET, []() {
    if (!distance_sensors_calibrated) {
      server.send(423, "application/json; charset=utf-8", "{\"calibrated\":false}");
      return;
    }
    send_advanced_state();
  });

  server.on("/api/advanced/set", HTTP_GET, []() {
    if (!distance_sensors_calibrated) {
      server.send(423, "application/json; charset=utf-8", "{\"calibrated\":false}");
      return;
    }

    bool ok = update_advanced_params_from_request();
    send_advanced_state(ok);
  });

  server.on("/api/advanced/reset", HTTP_GET, []() {
    if (!distance_sensors_calibrated) {
      server.send(423, "application/json; charset=utf-8", "{\"calibrated\":false}");
      return;
    }

    reset_all_advanced_parameters();
    send_advanced_state();
  });

  server.on("/api/advanced/save", HTTP_GET, []() {
    if (!distance_sensors_calibrated) {
      server.send(423, "application/json; charset=utf-8", "{\"calibrated\":false}");
      return;
    }

    bool ok = update_advanced_params_from_request();
    if (ok) {
      save_advanced_settings();
    }
    send_advanced_state(ok);
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

    update_controller_params_from_request();

    send_state();
  });

  server.on("/api/params/save", HTTP_GET, []() {
    if (!distance_sensors_calibrated) {
      server.send(423, "application/json; charset=utf-8", "{\"calibrated\":false}");
      return;
    }

    update_controller_params_from_request();
    save_controller_settings();
    send_state();
  });

  server.on("/api/params/reload", HTTP_GET, []() {
    if (!distance_sensors_calibrated) {
      server.send(423, "application/json; charset=utf-8", "{\"calibrated\":false}");
      return;
    }

    load_controller_settings();
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
  load_advanced_settings();
  load_controller_settings();

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
