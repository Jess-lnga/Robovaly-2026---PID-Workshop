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
static const uint32_t CALIBRATION_SCHEMA_VERSION = 10;
static const char *CONTROLLER_NAMESPACE = "ctrl";
static const char *CONTROLLER_VERSION_KEY = "version";
static const uint32_t CONTROLLER_SCHEMA_VERSION = 1;
static const uint32_t CONTROLLER_SAVE_COOLDOWN_MS = 2000;
static const float CONTROLLER_SAVE_FLOAT_EPSILON = 0.000001f;
static const char *ADVANCED_NAMESPACE = "advanced";
static const char *ADVANCED_VERSION_KEY = "version";
static const uint32_t ADVANCED_SCHEMA_VERSION = 8;
static const uint32_t ADVANCED_SAVE_COOLDOWN_MS = 2000;
static const int PLOT_DEFAULT_MAX_SECONDS = 10;
static const int PLOT_MIN_MAX_SECONDS = 10;
static const int PLOT_MAX_MAX_SECONDS = 50;
static const int MANUAL_ANGLE_DEFAULT_STEP_DEG = 5;
static const int MANUAL_ANGLE_MIN_STEP_DEG = 1;
static const int MANUAL_ANGLE_MAX_STEP_DEG = 30;
static const int NOISE_CAPTURE_TARGET_TOLERANCE_MM = 15;

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
  CAL_NOISE_0,
  CAL_NOISE_72,
  CAL_NOISE_145,
  CAL_NOISE_218,
  CAL_NOISE_290,
  CAL_NOISE_DONE,
  CAL_ERROR,
};

enum CalibrationMode {
  CAL_MODE_INITIAL_BOTH,
  CAL_MODE_MANUAL_TOF1,
  CAL_MODE_MANUAL_TOF2,
  CAL_MODE_VERIFY_ONLY,
  CAL_MODE_NOISE_ONLY,
};

struct TofCalibrationDraft {
  int meas_fov = INFINITE_TOF_VALUE;
  int real_fov = 145;
  int meas_0 = 0;
  int meas_72 = INFINITE_TOF_VALUE;
  int meas_145 = INFINITE_TOF_VALUE;
};

struct ServoCalibrationDraft {
  int theoretical_min_angle = SERVO_CMD_DEFAULT_THEORETICAL_MIN_DEG;
  int theoretical_max_angle = SERVO_CMD_DEFAULT_THEORETICAL_MAX_DEG;
  int limit_min_angle = SERVO_CMD_DEFAULT_LIMIT_MIN_DEG;
  int limit_max_angle = SERVO_CMD_DEFAULT_LIMIT_MAX_DEG;
  int neutral_offset_us = 0;
  int pwm_step_us = SERVO_CMD_DEFAULT_PWM_STEP_US;
};

struct NoiseCalibrationDraft {
  int position_mm[NOISE_PROFILE_POINT_COUNT] = {0, 72, 145, 218, TABLE_LENGTH_DEFAULT_MM};
  int position_noise_mm[NOISE_PROFILE_POINT_COUNT] = {
      DEFAULT_POSITION_NOISE_DEADBAND_MM,
      DEFAULT_POSITION_NOISE_DEADBAND_MM,
      DEFAULT_POSITION_NOISE_DEADBAND_MM,
      DEFAULT_POSITION_NOISE_DEADBAND_MM,
      DEFAULT_POSITION_NOISE_DEADBAND_MM
  };
  int tof1_position_noise_mm[NOISE_PROFILE_POINT_COUNT] = {
      DEFAULT_POSITION_NOISE_DEADBAND_MM,
      DEFAULT_POSITION_NOISE_DEADBAND_MM,
      DEFAULT_POSITION_NOISE_DEADBAND_MM,
      DEFAULT_POSITION_NOISE_DEADBAND_MM,
      DEFAULT_POSITION_NOISE_DEADBAND_MM
  };
  int tof2_position_noise_mm[NOISE_PROFILE_POINT_COUNT] = {
      DEFAULT_POSITION_NOISE_DEADBAND_MM,
      DEFAULT_POSITION_NOISE_DEADBAND_MM,
      DEFAULT_POSITION_NOISE_DEADBAND_MM,
      DEFAULT_POSITION_NOISE_DEADBAND_MM,
      DEFAULT_POSITION_NOISE_DEADBAND_MM
  };
  int speed_noise_mm_s[NOISE_PROFILE_POINT_COUNT] = {
      DEFAULT_SPEED_NOISE_DEADBAND_MM_S,
      DEFAULT_SPEED_NOISE_DEADBAND_MM_S,
      DEFAULT_SPEED_NOISE_DEADBAND_MM_S,
      DEFAULT_SPEED_NOISE_DEADBAND_MM_S,
      DEFAULT_SPEED_NOISE_DEADBAND_MM_S
  };
};

static ServoCalibrationDraft calibration_servo;
static TofCalibrationDraft calibration_tof1;
static TofCalibrationDraft calibration_tof2;
static NoiseCalibrationDraft calibration_noise;
static CalibrationStep calibration_step = CAL_TOF1_FIND_FOV;
static CalibrationMode calibration_mode = CAL_MODE_INITIAL_BOTH;
static bool calibration_flow_done = false;
static bool servo_calibration_initial_in_progress = false;
static String calibration_error_msg = "";
static uint32_t last_controller_save_request_ms = 0;
static uint32_t last_advanced_save_request_ms = 0;
static uint32_t last_client_mode_update_ms = 0;
static int plot_max_seconds = PLOT_DEFAULT_MAX_SECONDS;
static int manual_angle_step_deg = MANUAL_ANGLE_DEFAULT_STEP_DEG;
static bool web_client_present = false;
static bool auto_stabilization_without_client = false;
static bool client_mode_initialized = false;
static bool neutral_return_pending = false;

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
.dataVizHeader{display:grid;gap:8px;justify-items:center;margin:0 0 8px}.plotActions{display:flex;gap:8px;align-items:center;justify-content:center;flex-wrap:wrap}
.plotModeBtn{border:3px solid var(--line);background:#f8f5ea;font-weight:900;font-size:16px;padding:8px 12px;vertical-align:middle}
.plotModeBtn.active{background:#171717;color:#fffdf6}
.toast{position:fixed;right:14px;bottom:14px;max-width:min(420px,calc(100vw - 28px));background:#171717;color:#fffdf6;border:3px solid var(--line);padding:10px 14px;font-weight:800;z-index:20;display:none}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}.plotBox{height:320px;position:relative}.plotBox canvas{width:100%;height:100%}
.plotLabel{position:absolute;top:8px;right:12px;font-weight:800;text-decoration:underline}
.plotTune{position:absolute;left:8px;right:8px;top:38px;z-index:2;display:flex;gap:5px;align-items:center;justify-content:center;flex-wrap:nowrap}
.tuneGroup{display:flex;gap:3px;align-items:center;white-space:nowrap}.tuneGroup span{font-size:10px;font-weight:900}.tuneValue{border:2px solid var(--line);background:white;width:38px;height:22px;display:grid;place-items:center;font-size:12px;font-weight:900}
.tuneBtn,.tuneSave{height:22px;border:2px solid var(--line);background:#f8f5ea;font-weight:900;padding:0}.tuneBtn{width:22px;font-size:15px;line-height:1}.tuneSave{font-size:10px;padding:0 5px}
.controls{display:grid;grid-template-columns:1fr 1fr;gap:10px}.formGrid{display:grid;grid-template-columns:auto 1fr;gap:10px;align-items:center}
.saveRow{display:flex;justify-content:flex-end;gap:8px;margin-top:10px}.saveBtn{border:3px solid var(--line);background:#f8f5ea;font-weight:900;font-size:16px;padding:8px 12px}
label{font-weight:800}.field{height:38px;border:3px solid var(--line);background:white;font-size:18px;padding:3px 8px;width:100%}
.stabHeader{display:flex;justify-content:space-between;align-items:center;gap:12px;font-size:24px;font-weight:800}.toggle{min-width:46px;height:38px;border:3px solid var(--line);background:#f8f5ea;font-size:24px;font-weight:900;padding:0 10px}
.manualTitle{text-align:center;font-size:22px;font-weight:800;text-decoration:underline;margin:8px 0}.manualRow{display:grid;grid-template-columns:52px 7rem 52px;gap:10px;justify-content:center;align-items:center}
.manualBtn{width:52px;height:52px;border:3px solid var(--line);background:#f8f5ea;font-size:30px;font-weight:900;padding:0}.angleBox{height:52px;border:3px solid var(--line);background:white;display:grid;place-items:center;font-size:28px;font-weight:900}
.small{font-size:13px;color:var(--muted);margin-top:8px}.status{display:flex;gap:16px;flex-wrap:wrap;font-size:14px;color:var(--muted)}
button{cursor:pointer}button:disabled,input:disabled{opacity:.5;cursor:not-allowed}
@media(max-width:800px){.grid,.controls{grid-template-columns:1fr}.sceneWrap{height:34vh}.plotBox{height:260px}.plotTune{top:34px}.tuneGroup span{font-size:10px}}
</style>
</head>
<body>
<div class="app">
  <div class="panel sceneWrap">
    <button class="gearBtn" id="settingsBtn">&#9881;</button>
    <div class="settingsMenu" id="settingsMenu">
      <button class="menuBtn" id="calibrateBtn">Calibrate TOFs</button>
      <button class="menuBtn" id="calibrateServoBtn">Calibrate servo</button>
      <button class="menuBtn" id="advancedBtn">Advanced parameters</button>
    </div>
    <canvas id="scene"></canvas>
  </div>

  <div class="panel">
    <div class="dataVizHeader">
      <div class="sectionTitle">- Data Viz -</div>
      <div class="plotActions"><button class="toggle" id="plotToggle">Go</button><button class="plotModeBtn" id="continuousPlotBtn">Continuous plot</button></div>
    </div>
    <div class="grid">
      <div class="panel plotBox"><canvas id="anglePlot"></canvas><div class="plotLabel">Angle</div></div>
      <div class="panel plotBox"><canvas id="posPlot"></canvas><div class="plotLabel">Pos</div><div class="plotTune">
        <div class="tuneGroup"><span>min alpha</span><button class="tuneBtn" id="alphaMinMinus">-</button><div class="tuneValue" id="alphaMinTxt">--</div><button class="tuneBtn" id="alphaMinPlus">+</button></div>
        <div class="tuneGroup"><span>max alpha</span><button class="tuneBtn" id="alphaMaxMinus">-</button><div class="tuneValue" id="alphaMaxTxt">--</div><button class="tuneBtn" id="alphaMaxPlus">+</button></div>
        <button class="tuneSave" id="alphaSaveBtn">Save</button>
      </div></div>
      <div class="panel plotBox"><canvas id="speedPlot"></canvas><div class="plotLabel">Speed</div><div class="plotTune">
        <div class="tuneGroup"><span>min beta</span><button class="tuneBtn" id="betaMinMinus">-</button><div class="tuneValue" id="betaMinTxt">--</div><button class="tuneBtn" id="betaMinPlus">+</button></div>
        <div class="tuneGroup"><span>max beta</span><button class="tuneBtn" id="betaMaxMinus">-</button><div class="tuneValue" id="betaMaxTxt">--</div><button class="tuneBtn" id="betaMaxPlus">+</button></div>
        <button class="tuneSave" id="betaSaveBtn">Save</button>
      </div></div>
    </div>
    <div class="small" id="plotInfo">Plot stopped. Press Go to start a capture.</div>
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
      <div class="manualRow"><button class="manualBtn" id="manualMinusBtn">-</button><div class="angleBox" id="manualAngleTxt">--</div><button class="manualBtn" id="manualPlusBtn">+</button></div>
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
<div class="toast" id="toast"></div>
<script>
let TABLE_LEN_MM=290, PLOT_MAX_S=30; const REFRESH_MS=55;
const scene=document.getElementById('scene'), anglePlot=document.getElementById('anglePlot'), posPlot=document.getElementById('posPlot'), speedPlot=document.getElementById('speedPlot');
const plotToggle=document.getElementById('plotToggle'), continuousPlotBtn=document.getElementById('continuousPlotBtn'), plotInfo=document.getElementById('plotInfo');
const toast=document.getElementById('toast');
const settingsBtn=document.getElementById('settingsBtn'), settingsMenu=document.getElementById('settingsMenu'), calibrateBtn=document.getElementById('calibrateBtn'), calibrateServoBtn=document.getElementById('calibrateServoBtn'), advancedBtn=document.getElementById('advancedBtn');
const stabToggle=document.getElementById('stabToggle'), manualMinusBtn=document.getElementById('manualMinusBtn'), manualPlusBtn=document.getElementById('manualPlusBtn'), manualAngleTxt=document.getElementById('manualAngleTxt');
const refInput=document.getElementById('refInput'), kpInput=document.getElementById('kpInput'), kiInput=document.getElementById('kiInput'), kdInput=document.getElementById('kdInput');
const saveValuesBtn=document.getElementById('saveValuesBtn'), resetValuesBtn=document.getElementById('resetValuesBtn'), neutralBtn=document.getElementById('neutralBtn'), saveStatus=document.getElementById('saveStatus');
const servoTxt=document.getElementById('servoTxt'), tableTxt=document.getElementById('tableTxt'), xTxt=document.getElementById('xTxt'), vTxt=document.getElementById('vTxt'), d1Txt=document.getElementById('d1Txt'), d2Txt=document.getElementById('d2Txt');
const alphaMinTxt=document.getElementById('alphaMinTxt'),alphaMaxTxt=document.getElementById('alphaMaxTxt'),betaMinTxt=document.getElementById('betaMinTxt'),betaMaxTxt=document.getElementById('betaMaxTxt');
const alphaMinMinus=document.getElementById('alphaMinMinus'),alphaMinPlus=document.getElementById('alphaMinPlus'),alphaMaxMinus=document.getElementById('alphaMaxMinus'),alphaMaxPlus=document.getElementById('alphaMaxPlus'),alphaSaveBtn=document.getElementById('alphaSaveBtn');
const betaMinMinus=document.getElementById('betaMinMinus'),betaMinPlus=document.getElementById('betaMinPlus'),betaMaxMinus=document.getElementById('betaMaxMinus'),betaMaxPlus=document.getElementById('betaMaxPlus'),betaSaveBtn=document.getElementById('betaSaveBtn');
let state={x:-1,v:0,speed_valid:false,servo_angle:90,stabilization:true,controller_valid:false,ball_stable:false,kp:0,ki:0,kd:0,ref:150,d1:-1,d2:-1,servo_min:0,servo_max:180,servo_neutral:90,servo_theoretical_min:0,servo_theoretical_max:180,manual_angle_step:5,alpha_beta_min_alpha:.3,alpha_beta_max_alpha:.85,alpha_beta_min_beta:.08,alpha_beta_max_beta:.6};
let plotRunning=false, continuousPlot=false, plotStart=0, plotLastT=0, angleData=[], posData=[], speedData=[], lostIntervals=[], lastBallFoundPlotT=0, plotWasLost=false, lastStab=true, editing=false, neutralTimer=null, tuneSaveTimer=null, tuneLocalUntil=0;
let tuneDraft=null,tuneApplying=false;
let toastTimer=null;
function notify(msg){if(!toast)return;toast.textContent=msg;toast.style.display='block';if(toastTimer)clearTimeout(toastTimer);toastTimer=setTimeout(()=>toast.style.display='none',2600)}
function fit(c){const r=c.getBoundingClientRect(),d=window.devicePixelRatio||1;const w=Math.max(1,Math.floor(r.width*d)),h=Math.max(1,Math.floor(r.height*d));if(c.width!==w||c.height!==h){c.width=w;c.height=h}}
function systemStatus(){
  if(isBallLost())return {text:'Ball lost',color:'#c43131'};
  if(!state.stabilization)return {text:'Manual',color:'#2457b8'};
  if(state.ball_stable)return {text:'Stable',color:'#208444'};
  return {text:'Balancing...',color:'#666'};
}
function isBallLost(){
  return state.x<0||state.controller_valid===false;
}
function roundedRect(c,x,y,w,h,r){
  r=Math.min(r,w/2,h/2);
  c.beginPath();
  c.moveTo(x+r,y);c.lineTo(x+w-r,y);c.quadraticCurveTo(x+w,y,x+w,y+r);
  c.lineTo(x+w,y+h-r);c.quadraticCurveTo(x+w,y+h,x+w-r,y+h);
  c.lineTo(x+r,y+h);c.quadraticCurveTo(x,y+h,x,y+h-r);
  c.lineTo(x,y+r);c.quadraticCurveTo(x,y,x+r,y);
  c.closePath();
}
function drawStatusBadge(c,w,h){
  const st=systemStatus();
  const fs=Math.max(15,w*.022),padX=Math.max(12,w*.014),bh=fs+18;
  c.save();
  c.font=`900 ${fs}px Arial`;
  const bw=c.measureText(st.text).width+padX*2;
  const x=w-bw-18,y=h-bh-18;
  roundedRect(c,x,y,bw,bh,bh/2);
  c.fillStyle='#fffdf6';c.fill();
  c.lineWidth=3;c.strokeStyle=st.color;c.stroke();
  c.fillStyle=st.color;c.textBaseline='middle';
  c.fillText(st.text,x+padX,y+bh/2+1);
  c.restore();
}
function drawScene(){
fit(scene);
const c=scene.getContext('2d'),w=scene.width,h=scene.height;
c.clearRect(0,0,w,h);
c.lineWidth=4;c.strokeStyle='#171717';c.fillStyle='#171717';
const cx=w*.5,cy=h*.58,len=w*.68;
const tableDeg=state.servo_angle-state.servo_neutral;
const a=(-tableDeg)*Math.PI/180;
const ux=Math.cos(a),uy=Math.sin(a);
const nx=uy,ny=-ux;
const x1=cx-ux*len/2,y1=cy-uy*len/2,x2=cx+ux*len/2,y2=cy+uy*len/2;
c.beginPath();c.moveTo(x1,y1);c.lineTo(x2,y2);c.stroke();
c.beginPath();c.moveTo(cx,cy+8);c.lineTo(cx-w*.035,cy+h*.19);c.lineTo(cx+w*.035,cy+h*.19);c.closePath();c.stroke();
let p=state.x>=0?Math.max(0,Math.min(TABLE_LEN_MM,state.x))/TABLE_LEN_MM:.5;
const r=Math.max(16,Math.min(w,h)*.04);
let contactX=x1+(x2-x1)*p,contactY=y1+(y2-y1)*p;
let bx=contactX+nx*r,by=contactY+ny*r;
if(state.x>=0){
  c.beginPath();c.arc(bx,by,r,0,Math.PI*2);c.stroke();
  c.beginPath();c.moveTo(bx-r*.65,by-r*.65);c.lineTo(bx+r*.65,by+r*.65);c.moveTo(bx+r*.65,by-r*.65);c.lineTo(bx-r*.65,by+r*.65);c.stroke();
}else{
  c.save();
  c.setLineDash([10,7]);
  c.strokeStyle='#c43131';
  c.lineWidth=4;
  c.beginPath();c.arc(bx,by,r,0,Math.PI*2);c.stroke();
  c.restore();
}
c.font=`${Math.max(14,w*.018)}px Arial`;
c.fillText(`x=${state.x} mm`,18,h-76);
c.fillText(`servo=${state.servo_angle} deg / table=${tableDeg.toFixed(1)} deg`,18,h-24);
drawStatusBadge(c,w,h);
}
function scaleMax(t){return Math.min(PLOT_MAX_S,Math.max(10,Math.ceil(Math.max(0.001,t)/10)*10))}
function windowStats(data,label,xStart,xEnd){
  const vals=data.filter(d=>d.t>=xStart&&d.t<=xEnd&&!d.lost&&Number.isFinite(d.y)).map(d=>Number(d.y));
  if(!vals.length)return null;
  const min=Math.min(...vals),max=Math.max(...vals),mean=vals.reduce((a,b)=>a+b,0)/vals.length;
  const variance=vals.reduce((a,b)=>a+(b-mean)*(b-mean),0)/vals.length;
  const std=Math.sqrt(variance);
  return {min,max,mean,std,unit:label==='angle'?'deg':label==='pos'?'mm':'mm/s'};
}
function drawPlot(canvas,data,color,label,freeze){
fit(canvas);
const c=canvas.getContext('2d'),w=canvas.width,h=canvas.height,p=44;
c.clearRect(0,0,w,h);
const topPad=138;
const tNow=plotRunning?(performance.now()-plotStart)/1000:plotLastT;
const xStart=continuousPlot&&tNow>PLOT_MAX_S?tNow-PLOT_MAX_S:0;
const xEnd=continuousPlot&&tNow>PLOT_MAX_S?tNow:scaleMax(tNow);
let vals=data.map(d=>d.y).filter(Number.isFinite);
let ymin=label==='pos'?0:0,ymax=label==='pos'?TABLE_LEN_MM:90;
if(label==='speed'){
  const maxAbs=Math.max(100,...vals.map(v=>Math.abs(v)));
  ymax=Math.ceil(maxAbs/50)*50;
  ymin=-ymax;
}
else if(label==='angle'){
  ymin=Number.isFinite(Number(state.servo_theoretical_min))?Number(state.servo_theoretical_min):0;
  ymax=Number.isFinite(Number(state.servo_theoretical_max))?Number(state.servo_theoretical_max):180;
  if(ymax<=ymin){ymin=0;ymax=180}
}
else if(vals.length&&label!=='pos'){
  ymin=Math.min(0,...vals)-5;
  ymax=Math.max(90,...vals)+5;
  if(ymax-ymin<20){ymin-=10;ymax+=10}
}
const yOf=v=>h-p-(v-ymin)/(ymax-ymin)*(h-p-topPad);
const xOf=t=>p+((t-xStart)/(xEnd-xStart))*(w-p-18);
if(label==='pos'||label==='speed'){
  lostIntervals.forEach(iv=>{
    const end=iv.end==null?tNow:iv.end;
    const a=Math.max(iv.start,xStart),b=Math.min(end,xEnd);
    if(b>a){
      c.save();
      c.fillStyle='rgba(196,49,49,.24)';
      c.fillRect(xOf(a),topPad,xOf(b)-xOf(a),h-p-topPad);
      c.restore();
    }
  });
}
c.lineWidth=3;c.strokeStyle='#171717';
c.beginPath();c.moveTo(p,topPad);c.lineTo(p,h-p);c.lineTo(w-12,h-p);c.stroke();
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
if(label==='angle')dashedRef(state.servo_neutral||90,`neutral ${state.servo_neutral||90} deg`);
if(label==='pos')dashedRef(state.ref,`x0 ${state.ref} mm`);
if(label==='speed')dashedRef(0,'0 mm/s');
c.strokeStyle=color;c.lineWidth=4;c.beginPath();
let drawing=false;
data.forEach(d=>{
  if(((label==='pos'||label==='speed')&&d.lost)||!Number.isFinite(d.y)||d.t<xStart||d.t>xEnd){
    drawing=false;
    return;
  }
  const x=xOf(d.t);
  const y=yOf(d.y);
  if(!drawing){c.moveTo(x,y);drawing=true}else c.lineTo(x,y);
});
c.stroke();
c.fillStyle='#171717';
c.font=`${Math.max(12,w*.025)}px Arial`;
c.fillText(`${label} | ${Math.round(xStart)}-${Math.round(xEnd)}s`,p+8,topPad+12);
if(vals.length)c.fillText(`${vals[vals.length-1].toFixed(1)}`,w-90,topPad+12);
const stats=windowStats(data,label,xStart,xEnd);
if(stats){
  c.font=`${Math.max(11,w*.017)}px Arial`;
  c.fillStyle=color;
  const y=h-11,span=(w-p-22)/4;
  c.fillText(`min ${stats.min.toFixed(1)} ${stats.unit}`,p+4,y);
  c.fillText(`max ${stats.max.toFixed(1)} ${stats.unit}`,p+4+span,y);
  c.fillText(`mean ${stats.mean.toFixed(1)} ${stats.unit}`,p+4+span*2,y);
  c.fillText(`std ${stats.std.toFixed(1)} ${stats.unit}`,p+4+span*3,y);
}
}
function drawAll(){drawScene();drawPlot(anglePlot,angleData,'#c43131','angle',false);drawPlot(posPlot,posData,'#2457b8','pos',false);drawPlot(speedPlot,speedData,'#208444','speed',false)}
function updateTuneTexts(){alphaMinTxt.textContent=Number(state.alpha_beta_min_alpha).toFixed(2);alphaMaxTxt.textContent=Number(state.alpha_beta_max_alpha).toFixed(2);betaMinTxt.textContent=Number(state.alpha_beta_min_beta).toFixed(2);betaMaxTxt.textContent=Number(state.alpha_beta_max_beta).toFixed(2)}
function updateTexts(){if(state.stabilization&&neutralTimer){clearInterval(neutralTimer);neutralTimer=null}servoTxt.textContent=state.servo_angle;manualAngleTxt.textContent=state.servo_angle;tableTxt.textContent=(state.servo_angle-state.servo_neutral).toFixed(1);xTxt.textContent=state.x>=0?state.x:'--';vTxt.textContent=state.speed_valid?state.v:'--';d1Txt.textContent=state.d1>=0?state.d1:'--';d2Txt.textContent=state.d2>=0?state.d2:'--';stabToggle.textContent=state.stabilization?'||':'▶';manualMinusBtn.disabled=state.stabilization;manualPlusBtn.disabled=state.stabilization;neutralBtn.disabled=state.stabilization;if(performance.now()>tuneLocalUntil)updateTuneTexts();if(!editing){refInput.value=state.ref;kpInput.value=Number(state.kp).toFixed(3);kiInput.value=Number(state.ki).toFixed(3);kdInput.value=Number(state.kd).toFixed(3)}lastStab=state.stabilization}
function trimContinuousData(t){const minT=Math.max(0,t-PLOT_MAX_S);angleData=angleData.filter(d=>d.t>=minT);posData=posData.filter(d=>d.t>=minT);speedData=speedData.filter(d=>d.t>=minT);lostIntervals=lostIntervals.filter(iv=>(iv.end==null?t:iv.end)>=minT)}
function updateLostIntervals(t,lost){
  if(!lost)lastBallFoundPlotT=t;
  if(lost&&!plotWasLost){
    lostIntervals.push({start:lastBallFoundPlotT,end:null});
    posData.push({t,y:NaN,lost:true,gap:true});
    speedData.push({t,y:NaN,lost:true,gap:true});
  }else if(!lost&&plotWasLost&&lostIntervals.length){
    lostIntervals[lostIntervals.length-1].end=t;
    posData.push({t,y:NaN,lost:true,gap:true});
    speedData.push({t,y:NaN,lost:true,gap:true});
  }
  plotWasLost=lost;
}
function keepTuneDraftIfNeeded(nextState){
  if(tuneDraft&&(tuneApplying||performance.now()<tuneLocalUntil)){
    nextState.alpha_beta_min_alpha=tuneDraft.alpha_beta_min_alpha;
    nextState.alpha_beta_max_alpha=tuneDraft.alpha_beta_max_alpha;
    nextState.alpha_beta_min_beta=tuneDraft.alpha_beta_min_beta;
    nextState.alpha_beta_max_beta=tuneDraft.alpha_beta_max_beta;
  }
  return nextState;
}
async function fetchState(){try{const r=await fetch('/api/state',{cache:'no-store'});state=keepTuneDraftIfNeeded(await r.json());TABLE_LEN_MM=state.table_length||290;PLOT_MAX_S=state.plot_max_s||30;updateTexts();if(plotRunning){const t=(performance.now()-plotStart)/1000;plotLastT=t;if(t<=PLOT_MAX_S||continuousPlot){const lost=isBallLost();updateLostIntervals(t,lost);angleData.push({t,y:state.servo_angle,lost:false});posData.push({t,y:lost?NaN:state.x,lost});speedData.push({t,y:lost||!state.speed_valid?NaN:state.v,lost});if(continuousPlot)trimContinuousData(t);plotInfo.textContent=`Plot running: ${t.toFixed(1)} s / ${PLOT_MAX_S} s${continuousPlot?' continuous':''}`}else{plotRunning=false;plotToggle.textContent='Go';plotInfo.textContent=`${PLOT_MAX_S} s reached. Plot stopped.`}}drawAll()}catch(e){}}
function startPlot(){angleData=[];posData=[];speedData=[];lostIntervals=[];lastBallFoundPlotT=0;plotWasLost=false;plotLastT=0;plotStart=performance.now();plotRunning=true;plotToggle.textContent='Stop';plotInfo.textContent='Plot running'}
function stopPlot(){plotRunning=false;plotToggle.textContent='Go';plotInfo.textContent='Plot frozen. Press Go to restart from 0.'}
function resetPlotData(reason){
  angleData=[];posData=[];speedData=[];lostIntervals=[];
  lastBallFoundPlotT=0;plotWasLost=false;plotLastT=0;
  if(plotRunning)plotStart=performance.now();
  plotInfo.textContent=reason||'Plot reset.';
}
plotToggle.onclick=()=>plotRunning?stopPlot():startPlot();
continuousPlotBtn.onclick=()=>{continuousPlot=!continuousPlot;continuousPlotBtn.classList.toggle('active',continuousPlot);if(plotRunning){plotInfo.textContent=continuousPlot?'Continuous plot enabled.':'Continuous plot disabled.'}};
function tuneQuery(){return `ab_min_alpha=${Number(state.alpha_beta_min_alpha).toFixed(4)}&ab_max_alpha=${Number(state.alpha_beta_max_alpha).toFixed(4)}&ab_min_beta=${Number(state.alpha_beta_min_beta).toFixed(4)}&ab_max_beta=${Number(state.alpha_beta_max_beta).toFixed(4)}`}
function clampTune(){
  state.alpha_beta_min_alpha=Math.max(0,Math.min(1,Number(state.alpha_beta_min_alpha)));
  state.alpha_beta_max_alpha=Math.max(0,Math.min(1,Number(state.alpha_beta_max_alpha)));
  state.alpha_beta_min_beta=Math.max(0,Math.min(2,Number(state.alpha_beta_min_beta)));
  state.alpha_beta_max_beta=Math.max(0,Math.min(2,Number(state.alpha_beta_max_beta)));
  if(state.alpha_beta_min_alpha>state.alpha_beta_max_alpha)state.alpha_beta_max_alpha=state.alpha_beta_min_alpha;
  if(state.alpha_beta_min_beta>state.alpha_beta_max_beta)state.alpha_beta_max_beta=state.alpha_beta_min_beta;
}
async function applyTune(save){
  clampTune();updateTuneTexts();
  const url=save?'/api/advanced/save?':'/api/advanced/set?';
  tuneApplying=true;
  try{const r=await fetch(url+tuneQuery(),{cache:'no-store'});const js=await r.json();if(!js.ok){notify('Valeur alpha/beta refusee.');return}state.alpha_beta_min_alpha=Number(js.alpha_beta_min_alpha);state.alpha_beta_max_alpha=Number(js.alpha_beta_max_alpha);state.alpha_beta_min_beta=Number(js.alpha_beta_min_beta);state.alpha_beta_max_beta=Number(js.alpha_beta_max_beta);tuneDraft=null;tuneLocalUntil=0;updateTuneTexts();if(save)notify('Alpha/beta saved.')}catch(e){notify('Erreur reseau.')}finally{tuneApplying=false}
}
function scheduleTune(){if(tuneSaveTimer)clearTimeout(tuneSaveTimer);tuneSaveTimer=setTimeout(()=>applyTune(false),120)}
function stepTune(key,delta){resetPlotData('Alpha/beta changed. Plot restarted.');tuneLocalUntil=performance.now()+1200;state[key]=Number((Number(state[key])+delta).toFixed(4));clampTune();tuneDraft={alpha_beta_min_alpha:state.alpha_beta_min_alpha,alpha_beta_max_alpha:state.alpha_beta_max_alpha,alpha_beta_min_beta:state.alpha_beta_min_beta,alpha_beta_max_beta:state.alpha_beta_max_beta};updateTuneTexts();scheduleTune();drawAll()}
function saveTune(){if(tuneSaveTimer)clearTimeout(tuneSaveTimer);applyTune(true)}
alphaMinMinus.onclick=()=>stepTune('alpha_beta_min_alpha',-0.01);alphaMinPlus.onclick=()=>stepTune('alpha_beta_min_alpha',0.01);alphaMaxMinus.onclick=()=>stepTune('alpha_beta_max_alpha',-0.01);alphaMaxPlus.onclick=()=>stepTune('alpha_beta_max_alpha',0.01);alphaSaveBtn.onclick=saveTune;
betaMinMinus.onclick=()=>stepTune('alpha_beta_min_beta',-0.01);betaMinPlus.onclick=()=>stepTune('alpha_beta_min_beta',0.01);betaMaxMinus.onclick=()=>stepTune('alpha_beta_max_beta',-0.01);betaMaxPlus.onclick=()=>stepTune('alpha_beta_max_beta',0.01);betaSaveBtn.onclick=saveTune;
stabToggle.onclick=async()=>{const en=state.stabilization?0:1;await fetch(`/api/control?stabilization=${en}`,{cache:'no-store'});fetchState()};
async function setManualAngle(angle){angle=Math.max(Number(state.servo_min||0),Math.min(Number(state.servo_max||180),Math.round(angle)));manualAngleTxt.textContent=angle;servoTxt.textContent=angle;tableTxt.textContent=(angle-Number(state.servo_neutral||90)).toFixed(1);await fetch(`/api/control?angle=${angle}`,{cache:'no-store'});fetchState()}
manualMinusBtn.onclick=()=>{if(!state.stabilization)setManualAngle(Number(state.servo_angle||state.servo_neutral||90)-Number(state.manual_angle_step||5))};
manualPlusBtn.onclick=()=>{if(!state.stabilization)setManualAngle(Number(state.servo_angle||state.servo_neutral||90)+Number(state.manual_angle_step||5))};
[refInput,kpInput,kiInput,kdInput].forEach(el=>{el.onfocus=()=>editing=true;el.onblur=()=>editing=false;el.onchange=()=>{const q=`ref=${refInput.value}&kp=${kpInput.value}&ki=${kiInput.value}&kd=${kdInput.value}`;fetch(`/api/params?${q}`,{cache:'no-store'}).then(r=>{if(!r.ok)notify('Modification invalide.');fetchState()}).catch(()=>notify('Erreur reseau.'))}});
settingsBtn.onclick=()=>settingsMenu.classList.toggle('open');
calibrateBtn.onclick=()=>{location.href='/calibration_select'};
calibrateServoBtn.onclick=()=>{location.href='/servo_calibration'};
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
  const start=Number(state.servo_angle),target=Number(state.servo_neutral||90),duration=900,period=45,t0=performance.now();
  neutralBtn.disabled=true;
  neutralTimer=setInterval(()=>{
    const u=Math.min(1,(performance.now()-t0)/duration);
    const angle=Math.round(start+(target-start)*u);
    manualAngleTxt.textContent=angle;servoTxt.textContent=angle;tableTxt.textContent=(angle-Number(state.servo_neutral||90)).toFixed(1);
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
<p class="msg">Pour continuer, veuillez callibrer le systeme.</p>
<button id="startBtn">START</button>
</section>
<script>
document.getElementById('startBtn').onclick=async()=>{
  location.href='/servo_calibration?initial=1';
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
<button class="back" onclick="location.href='/calibration?noise=1'">Noise rejection</button>
<button class="back" onclick="location.href='/calibration?noise_result=1'">Verify noise rejection</button>
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
h1{font-size:28px;margin:0 0 10px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}.form{display:grid;grid-template-columns:minmax(0,1fr) 5.5rem;gap:10px;align-items:center}
label{font-weight:800;min-width:0}input{height:38px;border:3px solid #202020;background:white;font-size:18px;padding:4px 8px;width:5.5rem;max-width:100%;justify-self:end}
button,a{border:3px solid #202020;background:#f8f5ea;color:#171717;font-size:18px;font-weight:900;padding:10px 14px;cursor:pointer;text-decoration:none;text-align:center}.actions{display:flex;gap:10px;justify-content:flex-end;flex-wrap:wrap}.small{font-size:13px;color:#666}
.toast{position:fixed;right:14px;bottom:14px;max-width:min(420px,calc(100vw - 28px));background:#171717;color:#fffdf6;border:3px solid #202020;padding:10px 14px;font-weight:800;z-index:20;display:none}
@media(max-width:760px){.grid{grid-template-columns:1fr}.form{grid-template-columns:1fr}}
</style>
</head>
<body>
<div class="app">
<section class="panel"><h1>Advanced parameters</h1><div class="small" id="status">Runtime parameters. Use reset to restore development defaults.</div></section>
<div class="grid">
<section class="panel"><h1>Position and speed filtering</h1><div class="form">
<label>Max speed used by controller [mm/s]</label><input id="maxSpeed" type="number" min="0" max="2000" step="10">
<label>Alpha-beta min alpha</label><input id="abMinAlpha" type="number" min="0" max="1" step="0.01">
<label>Alpha-beta max alpha</label><input id="abMaxAlpha" type="number" min="0" max="1" step="0.01">
<label>Alpha-beta min beta</label><input id="abMinBeta" type="number" min="0" max="2" step="0.01">
<label>Alpha-beta max beta</label><input id="abMaxBeta" type="number" min="0" max="2" step="0.01">
</div></section>
<section class="panel"><h1>PID controller</h1><div class="form">
<label>Controller period [ms]</label><input id="ctrlPeriod" type="number" min="10" max="100" step="1">
<label>Max angle step / cycle [deg]</label><input id="maxStep" type="number" min="0" max="180" step="1">
<label>Position precision [mm]</label><input id="posDb" type="number" min="0" max="50" step="1">
<label>Speed precision [mm/s]</label><input id="speedDb" type="number" min="0" max="300" step="1">
<label>Stable confirm time [ms]</label><input id="stableTime" type="number" min="100" max="5000" step="50">
<label>Idle exit hysteresis [%]</label><input id="idleExit" type="number" min="100" max="500" step="10">
<label>Lost ball delay [s]</label><input id="lostDelay" type="number" min="0" max="10" step="0.1">
<label>Lost ball iter [cycles]</label><input id="lostIter" type="number" min="1" max="20" step="1">
</div></section>
<section class="panel"><h1>Servo</h1><div class="form">
<label>Min allowed angle [deg]</label><input id="servoMin" type="number" min="0" max="180" step="1">
<label>Max allowed angle [deg]</label><input id="servoMax" type="number" min="0" max="180" step="1">
<label>Step increment servomotor [us]</label><input id="servoStep" type="number" min="1" max="100" step="1">
<label>Manual angle step [deg]</label><input id="manualStep" type="number" min="1" max="30" step="1">
</div></section>
<section class="panel"><h1>Geometry</h1><div class="form">
<label>Table length [mm]</label><input id="tableLen" type="number" min="1" max="300" step="1">
</div></section>
<section class="panel"><h1>Data viz</h1><div class="form">
<label>Max plot time [s]</label><input id="plotMax" type="number" min="10" max="50" step="10">
</div></section>
</div>
<section class="panel actions">
<button id="resetBtn">RESET TO DEFAULT VALUES</button>
<button id="saveBtn">Save parameters</button>
<button id="applyBtn">Apply</button>
<a href="/">Back</a>
</section>
</div>
<div class="toast" id="toast"></div>
<script>
const ids=['maxSpeed','abMinAlpha','abMaxAlpha','abMinBeta','abMaxBeta','ctrlPeriod','maxStep','posDb','speedDb','stableTime','idleExit','lostDelay','lostIter','servoMin','servoMax','servoStep','manualStep','tableLen','plotMax'];
const el=Object.fromEntries(ids.map(id=>[id,document.getElementById(id)]));
const statusEl=document.getElementById('status');
const saveBtn=document.getElementById('saveBtn');
const toast=document.getElementById('toast');let toastTimer=null;
function notify(msg){toast.textContent=msg;toast.style.display='block';if(toastTimer)clearTimeout(toastTimer);toastTimer=setTimeout(()=>toast.style.display='none',3000)}
function fill(s){el.maxSpeed.value=s.max_control_speed;el.abMinAlpha.value=Number(s.alpha_beta_min_alpha).toFixed(2);el.abMaxAlpha.value=Number(s.alpha_beta_max_alpha).toFixed(2);el.abMinBeta.value=Number(s.alpha_beta_min_beta).toFixed(2);el.abMaxBeta.value=Number(s.alpha_beta_max_beta).toFixed(2);el.ctrlPeriod.value=s.controller_period;el.maxStep.value=s.max_step;el.posDb.value=s.position_deadband;el.speedDb.value=s.speed_deadband;el.stableTime.value=s.stable_time;el.idleExit.value=s.idle_exit_percent;el.lostDelay.value=(Number(s.lost_delay)/1000).toFixed(1);el.lostIter.value=s.lost_iter;el.servoMin.value=s.servo_min;el.servoMax.value=s.servo_max;el.servoStep.value=s.servo_step_us;el.manualStep.value=s.manual_angle_step;el.tableLen.value=s.table_length;el.plotMax.value=s.plot_max_s}
async function load(){try{const r=await fetch('/api/advanced',{cache:'no-store'});fill(await r.json())}catch(e){statusEl.textContent='Load failed.'}}
function query(){return `max_speed=${el.maxSpeed.value}&ab_min_alpha=${el.abMinAlpha.value}&ab_max_alpha=${el.abMaxAlpha.value}&ab_min_beta=${el.abMinBeta.value}&ab_max_beta=${el.abMaxBeta.value}&ctrl_period=${el.ctrlPeriod.value}&max_step=${el.maxStep.value}&pos_db=${el.posDb.value}&speed_db=${el.speedDb.value}&stable_time=${el.stableTime.value}&idle_exit=${el.idleExit.value}&lost_delay=${Math.round(Number(el.lostDelay.value)*1000)}&lost_iter=${el.lostIter.value}&servo_min=${el.servoMin.value}&servo_max=${el.servoMax.value}&servo_step=${el.servoStep.value}&manual_step=${el.manualStep.value}&table_len=${el.tableLen.value}&plot_max=${el.plotMax.value}`}
function num(id){return Number(el[id].value)}
function inRange(v,min,max){return Number.isFinite(v)&&v>=min&&v<=max}
function validateAdvanced(){
  const sm=num('servoMin'),sx=num('servoMax');
  if(!inRange(num('maxSpeed'),0,2000))return 'La vitesse max utilisee par le controleur doit etre entre 0 et 2000 mm/s.';
  if(!inRange(num('abMinAlpha'),0,1)||!inRange(num('abMaxAlpha'),0,1)||num('abMinAlpha')>num('abMaxAlpha'))return 'Alpha invalide: 0 <= min <= max <= 1.';
  if(!inRange(num('abMinBeta'),0,2)||!inRange(num('abMaxBeta'),0,2)||num('abMinBeta')>num('abMaxBeta'))return 'Beta invalide: 0 <= min <= max <= 2.';
  if(!inRange(num('ctrlPeriod'),10,100))return 'La periode du controleur doit etre entre 10 et 100 ms.';
  if(!inRange(num('maxStep'),0,180)||!inRange(num('posDb'),0,50)||!inRange(num('speedDb'),0,300))return 'Parametre PID hors limites.';
  if(!inRange(num('stableTime'),100,5000)||!inRange(num('idleExit'),100,500))return 'Parametre de detection stable hors limites.';
  if(!inRange(num('lostDelay'),0,10)||!inRange(num('lostIter'),1,20))return 'Parametre de balle perdue hors limites.';
  if(!inRange(sm,0,180)||!inRange(sx,0,180)||sm>=sx)return 'Angles servo invalides: min < max dans la plage 0-180 deg.';
  if(!inRange(num('servoStep'),1,100))return 'Le pas servo doit etre entre 1 et 100 us.';
  if(!inRange(num('manualStep'),1,30))return 'Le pas manuel doit etre entre 1 et 30 deg.';
  if(!inRange(num('tableLen'),1,300))return 'La longueur de table doit etre entre 1 et 300 mm.';
  if(!inRange(num('plotMax'),10,50))return 'Le temps de plot doit etre entre 10 et 50 s.';
  return '';
}
document.getElementById('applyBtn').onclick=async()=>{const err=validateAdvanced();if(err){notify(err);return}statusEl.textContent='Applying...';try{const r=await fetch('/api/advanced/set?'+query(),{cache:'no-store'});const s=await r.json();fill(s);statusEl.textContent=s.ok?'Applied.':'Some values were rejected.';if(!s.ok)notify('Modification invalide: verifiez les bornes des parametres.')}catch(e){statusEl.textContent='Apply failed.';notify('Erreur reseau.')}};
document.getElementById('resetBtn').onclick=async()=>{statusEl.textContent='Resetting...';try{const r=await fetch('/api/advanced/reset',{cache:'no-store'});fill(await r.json());statusEl.textContent='Defaults restored.'}catch(e){statusEl.textContent='Reset failed.'}};
saveBtn.onclick=async()=>{const err=validateAdvanced();if(err){notify(err);return}statusEl.textContent='Saving...';saveBtn.disabled=true;try{const r=await fetch('/api/advanced/save?'+query(),{cache:'no-store'});const s=await r.json();fill(s);statusEl.textContent=s.ok?'Saved.':'Some values were rejected.';if(!s.ok)notify('Sauvegarde refusee: une valeur est invalide.')}catch(e){statusEl.textContent='Save failed.';notify('Erreur reseau.')}setTimeout(()=>{saveBtn.disabled=false},2000)};
load();
</script>
</body>
</html>
)rawliteral";

static const char SERVO_CALIBRATION_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Calibration servo</title>
<style>
body{margin:0;min-height:100vh;background:#f4f1e8;color:#171717;font-family:Arial,Helvetica,sans-serif;padding:14px}
.app{width:min(920px,100%);margin:0 auto;display:grid;gap:12px}.panel{background:#fffdf6;border:3px solid #202020;border-radius:4px;padding:16px}
.scene{height:330px}.scene canvas{width:100%;height:100%;display:block}h1{font-size:27px;margin:0 0 10px}.msg{font-size:18px;line-height:1.4;color:#555}
.form{display:grid;grid-template-columns:1fr 160px;gap:10px;align-items:center}label{font-weight:800}input{height:42px;border:3px solid #202020;background:white;font-size:20px;padding:4px 8px;width:100%}
.pwmRow{display:grid;grid-template-columns:52px 7rem 52px;gap:10px;justify-content:center;align-items:center;margin:14px 0}.pwmBtn{width:52px;height:52px;padding:0;font-size:30px}.pwmBox{height:52px;border:3px solid #202020;background:white;display:grid;place-items:center;font-size:28px;font-weight:900}
.row{display:flex;gap:10px;justify-content:flex-end;flex-wrap:wrap}.small{font-size:14px;color:#666}
button,a{border:3px solid #202020;background:#f8f5ea;color:#171717;font-size:20px;font-weight:900;padding:10px 18px;cursor:pointer;text-decoration:none;text-align:center}button:disabled{opacity:.55;cursor:wait}
.toast{position:fixed;right:14px;bottom:14px;max-width:min(420px,calc(100vw - 28px));background:#171717;color:#fffdf6;border:3px solid #202020;padding:10px 14px;font-weight:800;z-index:20;display:none}
@media(max-width:620px){.form{grid-template-columns:1fr}.scene{height:260px}}
</style>
</head>
<body>
<div class="app">
<section class="panel scene"><canvas id="scene"></canvas></section>
<section class="panel">
<h1>Calibration du servomoteur</h1>
<p class="msg">Placez le servo a 1500 us, vissez la mecanique au plus proche de l'horizontale, puis ajustez le PWM et enregistrez l'offset.</p>
<div class="pwmRow">
<button class="pwmBtn" id="minusBtn">-</button>
<div class="pwmBox" id="pwmTxt">1500</div>
<button class="pwmBtn" id="plusBtn">+</button>
</div>
<div class="form">
<label for="minInput">Angle a 1000 us</label><input id="minInput" type="number" min="0" max="180" step="1">
<label for="maxInput">Angle a 2000 us</label><input id="maxInput" type="number" min="0" max="180" step="1">
<label for="limitMinInput">Min allowed angle</label><input id="limitMinInput" type="number" min="0" max="180" step="1">
<label for="limitMaxInput">Max allowed angle</label><input id="limitMaxInput" type="number" min="0" max="180" step="1">
</div>
<p class="small">Neutre logique: <b id="neutralTxt">--</b> deg | Offset: <b id="offsetTxt">--</b> us | Servo: <b id="servoTxt">--</b> deg</p>
<div class="row">
<button id="neutralBtn">Neutral pos</button>
<button id="offsetBtn">Set pos offset</button>
<button id="animateBtn">Animate</button>
<button id="saveBtn">Valider servo</button>
<a id="cancelBtn" href="/">Cancel</a>
</div>
<p class="small" id="status">Chargement...</p>
</section>
</div>
<div class="toast" id="toast"></div>
<script>
const scene=document.getElementById('scene'),minInput=document.getElementById('minInput'),maxInput=document.getElementById('maxInput'),limitMinInput=document.getElementById('limitMinInput'),limitMaxInput=document.getElementById('limitMaxInput'),neutralTxt=document.getElementById('neutralTxt'),offsetTxt=document.getElementById('offsetTxt'),servoTxt=document.getElementById('servoTxt'),pwmTxt=document.getElementById('pwmTxt'),statusEl=document.getElementById('status');
const minusBtn=document.getElementById('minusBtn'),plusBtn=document.getElementById('plusBtn'),neutralBtn=document.getElementById('neutralBtn'),offsetBtn=document.getElementById('offsetBtn'),animateBtn=document.getElementById('animateBtn'),saveBtn=document.getElementById('saveBtn'),cancelBtn=document.getElementById('cancelBtn');
const toast=document.getElementById('toast');let toastTimer=null;
const params=new URLSearchParams(location.search);let st={theoretical_min_angle:0,theoretical_max_angle:180,limit_min_angle:0,limit_max_angle:180,neutral_angle:90,current_angle:90,current_pwm_us:1500,neutral_offset_us:0,pwm_step_us:10};let started=false,animating=false;
function notify(msg){toast.textContent=msg;toast.style.display='block';if(toastTimer)clearTimeout(toastTimer);toastTimer=setTimeout(()=>toast.style.display='none',3000)}
function fit(c){const r=c.getBoundingClientRect(),d=window.devicePixelRatio||1;const w=Math.max(1,Math.floor(r.width*d)),h=Math.max(1,Math.floor(r.height*d));if(c.width!==w||c.height!==h){c.width=w;c.height=h}}
function neutralFromInputs(){const mn=Number(minInput.value||0),mx=Number(maxInput.value||180);return Math.round((mn+mx)/2)}
function displayAngle(){return Number(st.current_angle||neutralFromInputs())-neutralFromInputs()}
function draw(){
fit(scene);const c=scene.getContext('2d'),w=scene.width,h=scene.height;c.clearRect(0,0,w,h);c.lineWidth=4;c.strokeStyle='#171717';c.fillStyle='#171717';
const cx=w*.5,cy=h*.56,len=w*.72,tableDeg=displayAngle(),a=(-tableDeg)*Math.PI/180,ux=Math.cos(a),uy=Math.sin(a);
const x1=cx-ux*len/2,y1=cy-uy*len/2,x2=cx+ux*len/2,y2=cy+uy*len/2;
c.beginPath();c.moveTo(x1,y1);c.lineTo(x2,y2);c.stroke();
c.beginPath();c.moveTo(cx,cy+8);c.lineTo(cx-w*.035,cy+h*.22);c.lineTo(cx+w*.035,cy+h*.22);c.closePath();c.stroke();
c.font=`${Math.max(14,w*.018)}px Arial`;c.fillText(`servo=${st.current_angle} deg`,18,h-76);c.fillText(`table=${tableDeg.toFixed(1)} deg`,18,h-24);
}
function syncInputs(){minInput.value=st.theoretical_min_angle;maxInput.value=st.theoretical_max_angle;limitMinInput.value=st.limit_min_angle;limitMaxInput.value=st.limit_max_angle;neutralTxt.textContent=neutralFromInputs();offsetTxt.textContent=st.neutral_offset_us;pwmTxt.textContent=st.current_pwm_us;servoTxt.textContent=st.current_angle;draw()}
async function start(){if(started)return;started=true;const mode=params.get('initial')==='1'?'initial':'manual';await fetch('/api/servo_calibration/action?cmd=start&mode='+mode,{cache:'no-store'})}
async function load(){try{await start();const r=await fetch('/api/servo_calibration/state',{cache:'no-store'});st=await r.json();syncInputs();statusEl.textContent=st.error||'Pret.'}catch(e){statusEl.textContent='Erreur reseau'}}
async function setAngle(angle){st.current_angle=angle;servoTxt.textContent=angle;draw();await fetch('/api/servo_calibration/action?cmd=angle&value='+angle,{cache:'no-store'})}
async function setPwm(pwm){pwm=Math.max(1000,Math.min(2000,Math.round(pwm)));st.current_pwm_us=pwm;pwmTxt.textContent=pwm;await fetch('/api/servo_calibration/action?cmd=pwm&value='+pwm,{cache:'no-store'});load()}
async function animateSegment(from,to,duration=900){const t0=performance.now(),period=35;return new Promise(resolve=>{const timer=setInterval(async()=>{const u=Math.min(1,(performance.now()-t0)/duration);const a=Math.round(from+(to-from)*u);setAngle(a);if(u>=1){clearInterval(timer);resolve()}},period)})}
animateBtn.onclick=async()=>{
 if(animating)return;animating=true;animateBtn.disabled=saveBtn.disabled=true;statusEl.textContent='Animation...';
 const mn=Number(minInput.value||0),mx=Number(maxInput.value||180),lmn=Number(limitMinInput.value||mn),lmx=Number(limitMaxInput.value||mx),neu=neutralFromInputs();
 const pr=await fetch(`/api/servo_calibration/action?cmd=preview&min=${mn}&max=${mx}&limit_min=${lmn}&limit_max=${lmx}&offset=${st.neutral_offset_us}`,{cache:'no-store'});
 const ps=await pr.json();
 if(!ps.ok){statusEl.textContent=ps.error||'Valeurs invalides.';notify(ps.error||'Valeurs servo invalides.');animateBtn.disabled=saveBtn.disabled=false;animating=false;return}
 await animateSegment(st.current_angle,neu,600);await animateSegment(neu,mx,900);await animateSegment(mx,mn,1200);await animateSegment(mn,neu,900);
 statusEl.textContent='Animation terminee.';animateBtn.disabled=saveBtn.disabled=false;animating=false;
};
saveBtn.onclick=async()=>{
 saveBtn.disabled=true;statusEl.textContent='Validation...';
 const mn=Number(minInput.value||0),mx=Number(maxInput.value||180),lmn=Number(limitMinInput.value||mn),lmx=Number(limitMaxInput.value||mx);
 try{const r=await fetch(`/api/servo_calibration/action?cmd=save&min=${mn}&max=${mx}&limit_min=${lmn}&limit_max=${lmx}&offset=${st.neutral_offset_us}&step=${st.pwm_step_us}`,{cache:'no-store'});st=await r.json();syncInputs();if(st.ok){statusEl.textContent='Servo valide.';setTimeout(()=>{location.href=params.get('initial')==='1'?'/calibration?after_servo=1':'/'},500)}else{statusEl.textContent=st.error||'Valeurs invalides.';notify(st.error||'Valeurs servo invalides.')}}catch(e){statusEl.textContent='Erreur validation.';notify('Erreur reseau.')}
 saveBtn.disabled=false;
};
minusBtn.onclick=()=>setPwm(Number(st.current_pwm_us||1500)-Number(st.pwm_step_us||10));
plusBtn.onclick=()=>setPwm(Number(st.current_pwm_us||1500)+Number(st.pwm_step_us||10));
neutralBtn.onclick=()=>setPwm(1500);
offsetBtn.onclick=async()=>{try{const r=await fetch('/api/servo_calibration/action?cmd=offset',{cache:'no-store'});st=await r.json();syncInputs();statusEl.textContent=st.ok?'Offset pret a sauvegarder.':(st.error||'Offset invalide.')}catch(e){statusEl.textContent='Erreur reseau'}};
[minInput,maxInput].forEach(i=>i.oninput=()=>{neutralTxt.textContent=neutralFromInputs();draw()});
cancelBtn.onclick=e=>{if(params.get('initial')==='1'){e.preventDefault();location.href='/'}};
window.onresize=draw;load();
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
.verifyNav{position:absolute;top:50%;transform:translateY(-50%);width:44px;height:52px;border:3px solid #202020;background:#f8f5ea;color:#171717;font-size:30px;font-weight:900;padding:0;display:none;z-index:2}.verifyNav.show{display:block}.verifyNav.left{left:10px}.verifyNav.right{right:10px}
.noisePlots{display:grid;grid-template-columns:1fr 1fr;gap:12px}.noisePlot{height:260px;position:relative}.noisePlot canvas{width:100%;height:100%;display:block}.plotLabel{position:absolute;top:8px;right:12px;font-weight:900;text-decoration:underline}.legend{font-size:14px;color:#666;margin-top:8px}.abGrid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px;margin:12px 0}.abGrid label{font-size:13px;font-weight:900;display:grid;gap:4px}.abGrid input{width:100%;height:36px;font-size:16px}.abStatus{font-size:13px;color:#666;font-weight:800}
.row{display:flex;gap:12px;align-items:center;flex-wrap:wrap}.dot{width:18px;height:18px;border-radius:50%;background:#b91c1c;border:2px solid #202020}.ok{background:#1f8f45}.bad{background:#b91c1c}
button{border:3px solid #202020;background:#f8f5ea;color:#171717;font-size:20px;font-weight:900;padding:10px 18px;cursor:pointer}button:disabled{opacity:.55;cursor:wait}
input{height:42px;border:3px solid #202020;background:white;font-size:20px;padding:4px 8px;width:160px}.hint{font-size:14px;color:#666}.hidden{display:none}.err{color:#b91c1c;font-weight:800}
.toast{position:fixed;right:14px;bottom:14px;max-width:min(420px,calc(100vw - 28px));background:#171717;color:#fffdf6;border:3px solid #202020;padding:10px 14px;font-weight:800;z-index:20;display:none}
@media(max-width:800px){.noisePlots,.abGrid{grid-template-columns:1fr 1fr}.noisePlot{height:230px}}
</style>
</head>
<body>
<div class="app">
<section class="panel scene"><button class="verifyNav left" id="verifyPrevBtn">&lt;</button><canvas id="calScene"></canvas><button class="verifyNav right" id="verifyNextBtn">&gt;</button></section>
<section class="panel hidden" id="noisePlotPanel">
<h1>Noise rejection result</h1>
<div class="abGrid">
<label>Min alpha<input id="nrMinAlpha" type="number" min="0" max="1" step="0.01"></label>
<label>Max alpha<input id="nrMaxAlpha" type="number" min="0" max="1" step="0.01"></label>
<label>Min beta<input id="nrMinBeta" type="number" min="0" max="2" step="0.01"></label>
<label>Max beta<input id="nrMaxBeta" type="number" min="0" max="2" step="0.01"></label>
</div>
<div class="abStatus" id="nrAbStatus">Alpha-beta parameters are applied live.</div>
<div class="noisePlots">
<div class="panel noisePlot"><canvas id="noisePosPlot"></canvas><div class="plotLabel">Position</div></div>
<div class="panel noisePlot"><canvas id="noiseSpeedPlot"></canvas><div class="plotLabel">Speed</div></div>
</div>
<div class="legend">Grey: raw fused data | Blue/red: filtered data after noise rejection</div>
</section>
<section class="panel">
<h1 id="title">Calibration</h1>
<p class="msg" id="instruction">Chargement...</p>
<div class="row" id="rawRow"><span class="dot" id="dot"></span><b id="rawTxt">--</b><span id="tofTxt">--</span></div>
<div class="row" id="realRow"><label for="realInput"><b>Distance reelle au FOV max</b></label><input id="realInput" type="number" min="1" max="400" step="1"><span>mm</span></div>
<div class="row">
<button id="doneBtn">Done</button>
<button id="submitBtn">Valider distance</button>
<button id="acceptBtn">Valider calibration</button>
<button id="verifyTof1Btn">Calibrate TOF 1</button>
<button id="verifyTof2Btn">Calibrate TOF 2</button>
<button id="goNoiseBtn">Go to noise rejection</button>
<button id="restartBtn">Restart</button>
<button id="cancelBtn">Cancel</button>
</div>
<div class="hint" id="status">Les valeurs sont sauvegardees apres validation finale.</div>
</section>
</div>
<div class="toast" id="toast"></div>
<script>
let TABLE_LEN_MM=290; const REFRESH_MS=80;
const scene=document.getElementById('calScene'),dot=document.getElementById('dot'),rawTxt=document.getElementById('rawTxt'),tofTxt=document.getElementById('tofTxt'),rawRow=document.getElementById('rawRow');
const noisePlotPanel=document.getElementById('noisePlotPanel'),noisePosPlot=document.getElementById('noisePosPlot'),noiseSpeedPlot=document.getElementById('noiseSpeedPlot');
const nrMinAlpha=document.getElementById('nrMinAlpha'),nrMaxAlpha=document.getElementById('nrMaxAlpha'),nrMinBeta=document.getElementById('nrMinBeta'),nrMaxBeta=document.getElementById('nrMaxBeta'),nrAbStatus=document.getElementById('nrAbStatus');
const verifyPrevBtn=document.getElementById('verifyPrevBtn'),verifyNextBtn=document.getElementById('verifyNextBtn');
const title=document.getElementById('title'),instruction=document.getElementById('instruction'),statusEl=document.getElementById('status');
const realRow=document.getElementById('realRow'),realInput=document.getElementById('realInput');
const doneBtn=document.getElementById('doneBtn'),submitBtn=document.getElementById('submitBtn'),acceptBtn=document.getElementById('acceptBtn'),verifyTof1Btn=document.getElementById('verifyTof1Btn'),verifyTof2Btn=document.getElementById('verifyTof2Btn'),goNoiseBtn=document.getElementById('goNoiseBtn'),restartBtn=document.getElementById('restartBtn'),cancelBtn=document.getElementById('cancelBtn');
const toast=document.getElementById('toast');let toastTimer=null;
let s={};
const params=new URLSearchParams(location.search);
let startupDone=false;
let realInputEditing=false;
let lastInputStep='';
let lastError='';
let lastStepName='';
let verifyView=null;
let nrAlphaBetaLoaded=false,nrApplyTimer=null;
let noisePlotStart=0,noisePlotLastT=0,noisePlotWasLost=false,noiseLastRawPos=null,noiseLastRawT=0,noisePosData=[],noiseSpeedData=[],noiseLostIntervals=[];
const verifyViews=['both','tof1','tof2'];
function notify(msg){toast.textContent=msg;toast.style.display='block';if(toastTimer)clearTimeout(toastTimer);toastTimer=setTimeout(()=>toast.style.display='none',3000)}
function fit(c){const r=c.getBoundingClientRect(),d=window.devicePixelRatio||1;const w=Math.max(1,Math.floor(r.width*d)),h=Math.max(1,Math.floor(r.height*d));if(c.width!==w||c.height!==h){c.width=w;c.height=h}}
function roundedRect(c,x,y,w,h,r){r=Math.min(r,w/2,h/2);c.beginPath();c.moveTo(x+r,y);c.lineTo(x+w-r,y);c.quadraticCurveTo(x+w,y,x+w,y+r);c.lineTo(x+w,y+h-r);c.quadraticCurveTo(x+w,y+h,x+w-r,y+h);c.lineTo(x+r,y+h);c.quadraticCurveTo(x,y+h,x,y+h-r);c.lineTo(x,y+r);c.quadraticCurveTo(x,y,x+r,y);c.closePath()}
function selectedVerifyTof(){return verifyView==='tof1'?1:(verifyView==='tof2'?2:0)}
function cycleVerifyView(delta){const i=verifyViews.indexOf(verifyView||'both');verifyView=verifyViews[(i+delta+verifyViews.length)%verifyViews.length];drawScene()}
function verifyLabel(){return verifyView==='tof1'?'TOF 1 only':verifyView==='tof2'?'TOF 2 only':'TOF 1 + TOF 2'}
function verifyVisualPosition(){
  if(s.step!=='verify')return s.visual_pos_mm;
  if(verifyView==='tof1')return s.tof1_pos_mm;
  if(verifyView==='tof2')return s.tof2_pos_mm;
  return s.visual_pos_mm;
}
function verifyBallLost(){
  if(s.step!=='verify')return false;
  const pos=verifyVisualPosition();
  return !Number.isFinite(pos)||pos<0;
}
function drawCalibrationBadge(c,w,h,text,color){
  const fs=Math.max(14,w*.02),padX=Math.max(12,w*.014),bh=fs+16;
  c.save();
  c.font=`900 ${fs}px Arial`;
  const bw=c.measureText(text).width+padX*2;
  const x=w-bw-16,y=16;
  roundedRect(c,x,y,bw,bh,bh/2);
  c.fillStyle='#fffdf6';c.fill();
  c.lineWidth=3;c.strokeStyle=color;c.stroke();
  c.fillStyle=color;c.textBaseline='middle';
  c.fillText(text,x+padX,y+bh/2+1);
  c.restore();
}
function resetNoisePlots(){
  noisePlotStart=performance.now();noisePlotLastT=0;noisePlotWasLost=false;noiseLastRawPos=null;noiseLastRawT=0;noisePosData=[];noiseSpeedData=[];noiseLostIntervals=[];
}
function fillNoiseAlphaBeta(){
  if(nrAlphaBetaLoaded)return;
  nrMinAlpha.value=Number(s.alpha_beta_min_alpha||0).toFixed(2);
  nrMaxAlpha.value=Number(s.alpha_beta_max_alpha||0).toFixed(2);
  nrMinBeta.value=Number(s.alpha_beta_min_beta||0).toFixed(2);
  nrMaxBeta.value=Number(s.alpha_beta_max_beta||0).toFixed(2);
  nrAlphaBetaLoaded=true;
}
function validNoiseAlphaBeta(){
  const a0=Number(nrMinAlpha.value),a1=Number(nrMaxAlpha.value),b0=Number(nrMinBeta.value),b1=Number(nrMaxBeta.value);
  return Number.isFinite(a0)&&Number.isFinite(a1)&&Number.isFinite(b0)&&Number.isFinite(b1)&&a0>=0&&a1<=1&&a0<=a1&&b0>=0&&b1<=2&&b0<=b1;
}
async function applyNoiseAlphaBeta(){
  if(!validNoiseAlphaBeta()){nrAbStatus.textContent='Invalid alpha-beta bounds.';return}
  nrAbStatus.textContent='Applying...';
  const q=`ab_min_alpha=${nrMinAlpha.value}&ab_max_alpha=${nrMaxAlpha.value}&ab_min_beta=${nrMinBeta.value}&ab_max_beta=${nrMaxBeta.value}`;
  try{const r=await fetch('/api/advanced/set?'+q,{cache:'no-store'});const js=await r.json();nrAbStatus.textContent=js.ok?'Applied live.':'Rejected.'}catch(e){nrAbStatus.textContent='Apply failed.'}
}
function scheduleNoiseAlphaBetaApply(){if(nrApplyTimer)clearTimeout(nrApplyTimer);nrApplyTimer=setTimeout(applyNoiseAlphaBeta,250)}
function trimNoisePlots(t){
  const minT=Math.max(0,t-10);
  noisePosData=noisePosData.filter(d=>d.t>=minT);
  noiseSpeedData=noiseSpeedData.filter(d=>d.t>=minT);
  noiseLostIntervals=noiseLostIntervals.filter(iv=>(iv.end==null?t:iv.end)>=minT);
}
function updateNoiseLostIntervals(t,lost){
  if(lost&&!noisePlotWasLost)noiseLostIntervals.push({start:t,end:null});
  else if(!lost&&noisePlotWasLost&&noiseLostIntervals.length)noiseLostIntervals[noiseLostIntervals.length-1].end=t;
  if(lost!==noisePlotWasLost){
    noisePosData.push({t,raw:NaN,filtered:NaN,lost:true,gap:true});
    noiseSpeedData.push({t,raw:NaN,filtered:NaN,lost:true,gap:true});
  }
  noisePlotWasLost=lost;
}
function addNoisePlotSample(){
  if(s.step!=='noise_done')return;
  const t=(performance.now()-noisePlotStart)/1000;
  noisePlotLastT=t;
  const lost=!Number.isFinite(Number(s.visual_pos_mm))||Number(s.visual_pos_mm)<0||!Number.isFinite(Number(s.raw_pos_mm))||Number(s.raw_pos_mm)<0;
  updateNoiseLostIntervals(t,lost);
  let rawSpeed=NaN;
  const rawPos=Number(s.raw_pos_mm);
  if(!lost&&noiseLastRawPos!==null&&t>noiseLastRawT){
    rawSpeed=(rawPos-noiseLastRawPos)/(t-noiseLastRawT);
  }
  if(!lost){noiseLastRawPos=rawPos;noiseLastRawT=t}else{noiseLastRawPos=null;noiseLastRawT=t}
  noisePosData.push({t,raw:lost?NaN:rawPos,filtered:lost?NaN:Number(s.visual_pos_mm),lost});
  noiseSpeedData.push({t,raw:lost?NaN:rawSpeed,filtered:lost||!s.speed_valid?NaN:Number(s.speed_mm_s),lost});
  trimNoisePlots(t);
}
function drawNoisePlot(canvas,data,label,color){
  fit(canvas);const c=canvas.getContext('2d'),w=canvas.width,h=canvas.height,p=44;c.clearRect(0,0,w,h);
  const tNow=noisePlotLastT,xStart=Math.max(0,tNow-10),xEnd=Math.max(10,tNow);
  let vals=[];data.forEach(d=>{if(Number.isFinite(d.raw))vals.push(d.raw);if(Number.isFinite(d.filtered))vals.push(d.filtered)});
  let ymin=label==='pos'?0:-100,ymax=label==='pos'?TABLE_LEN_MM:100;
  if(label==='speed'&&vals.length){const m=Math.max(100,...vals.map(v=>Math.abs(v)));ymax=Math.ceil(m/50)*50;ymin=-ymax}
  const xOf=t=>p+((t-xStart)/(xEnd-xStart))*(w-p-18),yOf=v=>h-p-(v-ymin)/(ymax-ymin)*(h-p-16);
  noiseLostIntervals.forEach(iv=>{const end=iv.end==null?tNow:iv.end,a=Math.max(iv.start,xStart),b=Math.min(end,xEnd);if(b>a){c.fillStyle='rgba(196,49,49,.24)';c.fillRect(xOf(a),12,xOf(b)-xOf(a),h-p-12)}});
  c.lineWidth=3;c.strokeStyle='#171717';c.beginPath();c.moveTo(p,12);c.lineTo(p,h-p);c.lineTo(w-12,h-p);c.stroke();
  function ref(v,text){if(v<ymin||v>ymax)return;c.save();c.setLineDash([10,8]);c.strokeStyle='#555';c.lineWidth=2;const y=yOf(v);c.beginPath();c.moveTo(p,y);c.lineTo(w-12,y);c.stroke();c.restore();c.fillStyle='#555';c.font=`${Math.max(11,w*.021)}px Arial`;c.fillText(text,p+8,y-6)}
  if(label==='pos')ref(s.visual_target_mm||145,`${s.visual_target_mm||145} mm`);else ref(0,'0 mm/s');
  function line(key,stroke){c.strokeStyle=stroke;c.lineWidth=key==='raw'?3:4;c.beginPath();let drawing=false;data.forEach(d=>{const v=d[key];if(d.lost||!Number.isFinite(v)||d.t<xStart||d.t>xEnd){drawing=false;return}const x=xOf(d.t),y=yOf(v);if(!drawing){c.moveTo(x,y);drawing=true}else c.lineTo(x,y)});c.stroke()}
  line('raw','#777');line('filtered',color);
  c.fillStyle='#171717';c.font=`${Math.max(12,w*.025)}px Arial`;c.fillText(`${label} | ${Math.round(xStart)}-${Math.round(xEnd)}s`,p+8,24);
  if(label==='speed'){
    const rawVals=data.map(d=>d.raw).filter(Number.isFinite);
    const filteredVals=data.map(d=>d.filtered).filter(Number.isFinite);
    c.font=`${Math.max(11,w*.02)}px Arial`;
    c.fillStyle='#777';
    if(rawVals.length)c.fillText(`raw min ${Math.min(...rawVals).toFixed(0)} / max ${Math.max(...rawVals).toFixed(0)} mm/s`,p+8,h-26);
    c.fillStyle=color;
    if(filteredVals.length)c.fillText(`filtered min ${Math.min(...filteredVals).toFixed(0)} / max ${Math.max(...filteredVals).toFixed(0)} mm/s`,p+8,h-10);
  }
}
function drawNoisePlots(){drawNoisePlot(noisePosPlot,noisePosData,'pos','#2457b8');drawNoisePlot(noiseSpeedPlot,noiseSpeedData,'speed','#c43131')}
function drawScene(){
fit(scene);const c=scene.getContext('2d'),w=scene.width,h=scene.height;c.clearRect(0,0,w,h);c.lineWidth=4;c.strokeStyle='#171717';c.fillStyle='#171717';
const cx=w*.5,cy=h*.56,len=w*.72,a=0,ux=Math.cos(a),uy=Math.sin(a),nx=0,ny=-1;const x1=cx-len/2,y1=cy,x2=cx+len/2,y2=cy;
c.beginPath();c.moveTo(x1,y1);c.lineTo(x2,y2);c.stroke();c.beginPath();c.moveTo(cx,cy+8);c.lineTo(cx-w*.035,cy+h*.22);c.lineTo(cx+w*.035,cy+h*.22);c.closePath();c.stroke();
const stepName=String(s.step||''),noiseCaptureStep=stepName.startsWith('noise')&&stepName!=='noise_done';
const targetPos=Number(s.visual_target_mm??s.visual_pos_mm);
const livePos=Number(s.live_pos_mm);
const ballLost=noiseCaptureStep?(!Number.isFinite(livePos)||livePos<0):verifyBallLost();
let pos=noiseCaptureStep?livePos:verifyVisualPosition(); if(ballLost||!Number.isFinite(pos)||pos<0)pos=Number.isFinite(targetPos)?targetPos:TABLE_LEN_MM/2; pos=Math.max(0,Math.min(TABLE_LEN_MM,pos)); const p=pos/TABLE_LEN_MM;
const r=Math.max(15,Math.min(w,h)*.045),contactX=x1+(x2-x1)*p,contactY=y1+(y2-y1)*p,bx=contactX+nx*r,by=contactY+ny*r;
if(noiseCaptureStep&&Number.isFinite(targetPos)&&targetPos>=0){
  const tp=Math.max(0,Math.min(TABLE_LEN_MM,targetPos))/TABLE_LEN_MM;
  const tx=x1+(x2-x1)*tp,ty=cy-r;
  c.save();
  c.setLineDash([8,6]);
  c.strokeStyle='#777';
  c.lineWidth=4;
  c.beginPath();c.arc(tx,ty,r,0,Math.PI*2);c.stroke();
  c.fillStyle='#777';
  c.font=`900 ${Math.max(12,w*.016)}px Arial`;
  c.fillText('target',tx-24,ty-r-10);
  c.restore();
}
if(ballLost){
  c.save();
  c.setLineDash([10,7]);
  c.strokeStyle='#c43131';
  c.lineWidth=4;
  c.beginPath();c.arc(bx,by,r,0,Math.PI*2);c.stroke();
  c.restore();
}else{
  if(noiseCaptureStep&&s.noise_capture_ready===false){
    c.strokeStyle='#c43131';
  }
  c.beginPath();c.arc(bx,by,r,0,Math.PI*2);c.stroke();c.beginPath();c.moveTo(bx-r*.65,by-r*.65);c.lineTo(bx+r*.65,by+r*.65);c.moveTo(bx+r*.65,by-r*.65);c.lineTo(bx-r*.65,by+r*.65);c.stroke();
  c.strokeStyle='#171717';
}
if(s.step==='verify'||String(s.step||'').startsWith('noise')){[0,72,145,218,290].forEach(mm=>{const x=x1+(x2-x1)*(mm/TABLE_LEN_MM);c.strokeStyle='#208444';c.fillStyle='#208444';c.lineWidth=3;c.beginPath();c.moveTo(x,cy+30);c.lineTo(x,cy+70);c.stroke();c.beginPath();c.moveTo(x,cy+24);c.lineTo(x-8,cy+42);c.lineTo(x+8,cy+42);c.closePath();c.fill();c.fillText(`${mm}`,x-10,cy+90)});c.strokeStyle='#171717';c.fillStyle='#171717'}
const focusTof=s.step==='verify'?selectedVerifyTof():s.tof;
if((s.step!=='verify'&&!String(s.step||'').startsWith('noise')&&s.tof)||(s.step==='verify'&&focusTof)){
  const tx=focusTof===1?x2:x1,dir=focusTof===1?-1:1,ay=cy-76;
  c.strokeStyle='#2457b8';c.fillStyle='#2457b8';c.lineWidth=5;
  c.beginPath();c.moveTo(tx+dir*70,ay);c.lineTo(tx+dir*14,ay);c.stroke();
  c.beginPath();c.moveTo(tx+dir*8,ay);c.lineTo(tx+dir*24,ay-10);c.lineTo(tx+dir*24,ay+10);c.closePath();c.fill();
  c.font=`${Math.max(13,w*.017)}px Arial`;c.fillText(`TOF ${focusTof}`,tx+dir*42-22,ay-16);
  if(s.step==='verify'){
    const fovPos=focusTof===1?s.tof1_fov_pos_mm:s.tof2_fov_pos_mm;
    if(Number.isFinite(fovPos)&&fovPos>=0){
      const fx=x1+(x2-x1)*(Math.max(0,Math.min(TABLE_LEN_MM,fovPos))/TABLE_LEN_MM);
      c.strokeStyle='#c43131';c.fillStyle='#c43131';c.lineWidth=4;
      c.beginPath();c.moveTo(fx,cy-70);c.lineTo(fx,cy-20);c.stroke();
      c.beginPath();c.moveTo(fx,cy-12);c.lineTo(fx-10,cy-30);c.lineTo(fx+10,cy-30);c.closePath();c.fill();
      c.font=`${Math.max(12,w*.016)}px Arial`;c.fillText('FOV',fx-16,cy-78);
    }
  }
  c.strokeStyle='#171717';c.fillStyle='#171717';
}
c.font=`${Math.max(14,w*.018)}px Arial`;c.fillText(`visual pos=${Math.round(pos)} mm`,18,h-76);
if(s.step==='verify'){c.fillText(verifyLabel(),18,h-20)}
if(noiseCaptureStep){
  c.fillText(`target=${Math.round(targetPos)} mm`,18,h-24);
  if(ballLost)drawCalibrationBadge(c,w,h,'Ball lost','#c43131');
  else if(s.noise_capture_ready===false)drawCalibrationBadge(c,w,h,'Move to target','#c43131');
  else drawCalibrationBadge(c,w,h,'Ready','#208444');
}else if(ballLost)drawCalibrationBadge(c,w,h,'Ball lost','#c43131');
}
function setButtons(){
const verifyOnly=params.get('verify')==='1';
const verifyStep=s.step==='verify';
const noiseStep=String(s.step||'').startsWith('noise');
const noiseDone=s.step==='noise_done';
doneBtn.classList.toggle('hidden',!s.needs_done);
submitBtn.classList.toggle('hidden',!s.needs_real_input);
realRow.classList.toggle('hidden',!s.needs_real_input);
rawRow.classList.toggle('hidden',verifyStep||noiseStep);
noisePlotPanel.classList.toggle('hidden',!noiseDone);
verifyPrevBtn.classList.toggle('show',verifyStep);
verifyNextBtn.classList.toggle('show',verifyStep);
statusEl.classList.toggle('hidden',verifyStep);
acceptBtn.classList.toggle('hidden',s.step!=='verify'&&s.step!=='noise_done');
verifyTof1Btn.classList.add('hidden');
verifyTof2Btn.classList.add('hidden');
goNoiseBtn.classList.add('hidden');
if(verifyStep){
  const justCalibrated=Number(s.default_verify_tof||0);
  verifyTof1Btn.classList.toggle('hidden',justCalibrated===1);
  verifyTof2Btn.classList.toggle('hidden',justCalibrated===2);
  goNoiseBtn.classList.remove('hidden');
}
restartBtn.classList.toggle('hidden',false);
cancelBtn.classList.toggle('hidden',params.get('initial')==='1');
if((verifyOnly&&verifyStep)||params.get('noise_result')==='1'){
  doneBtn.classList.remove('hidden');
  submitBtn.classList.add('hidden');
  acceptBtn.classList.add('hidden');
  restartBtn.classList.toggle('hidden',!verifyStep);
  cancelBtn.classList.add('hidden');
  realRow.classList.add('hidden');
  doneBtn.textContent='Done';
}else{
  doneBtn.textContent=noiseStep?'Capture bruit':'Done';
}
if(s.needs_real_input && !realInputEditing && (s.step!==lastInputStep || !realInput.value)){
  realInput.value=s.real_fov||145;
}
lastInputStep=s.step||'';
}
function update(){
TABLE_LEN_MM=s.table_length||290;
if(s.step!==lastStepName){
  if(s.step==='noise_done'){resetNoisePlots();nrAlphaBetaLoaded=false}
  lastStepName=s.step||'';
}
if(s.step==='noise_done')fillNoiseAlphaBeta();
if(s.step==='verify'&&!verifyView){
  verifyView=s.default_verify_tof===1?'tof1':s.default_verify_tof===2?'tof2':'both';
}
if(s.step!=='verify'){
  verifyView=null;
}
title.textContent=s.title||'Calibration';
instruction.textContent=s.instruction||'';
rawTxt.textContent=s.raw_valid?`raw=${s.raw_mm} mm`:'raw invalid';
tofTxt.textContent=s.tof?`TOF ${s.tof}`:'';
dot.className='dot '+(s.raw_valid?'ok':'bad');
const noiseCaptureStep=String(s.step||'').startsWith('noise')&&s.step!=='noise_done';
if(s.error)statusEl.innerHTML=`<span class="err">${s.error}</span>`;
else if(noiseCaptureStep&&s.noise_capture_ready===false)statusEl.innerHTML=`<span class="err">Placez la balle a +/- ${s.noise_capture_tolerance_mm||15} mm de la cible avant la capture.</span>`;
else if(noiseCaptureStep)statusEl.innerHTML='<span style="color:#208444">Balle dans la zone de capture.</span>';
else statusEl.textContent=s.status||'';
if(s.error&&s.error!==lastError)notify(s.error);
lastError=s.error||'';
setButtons();drawScene();
if(s.step==='noise_done'){addNoisePlotSample();drawNoisePlots()}
}
async function ensureStarted(){
if(startupDone)return;
startupDone=true;
if(params.get('initial')==='1'||params.get('after_servo')==='1')await fetch('/api/calibration/action?cmd=start&mode=initial_tofs',{cache:'no-store'});
else if(params.get('verify')==='1')await fetch('/api/calibration/action?cmd=start&mode=verify',{cache:'no-store'});
else if(params.get('noise')==='1')await fetch('/api/calibration/action?cmd=start&mode=noise',{cache:'no-store'});
else if(params.get('noise_result')==='1')await fetch('/api/calibration/action?cmd=start&mode=noise_result',{cache:'no-store'});
else if(params.get('target')==='1')await fetch('/api/calibration/action?cmd=start&target=1',{cache:'no-store'});
else if(params.get('target')==='2')await fetch('/api/calibration/action?cmd=start&target=2',{cache:'no-store'});
}
async function getState(){try{await ensureStarted();const r=await fetch('/api/calibration/state',{cache:'no-store'});s=await r.json();update()}catch(e){statusEl.textContent='Erreur reseau'}}
async function action(q){doneBtn.disabled=submitBtn.disabled=acceptBtn.disabled=verifyTof1Btn.disabled=verifyTof2Btn.disabled=goNoiseBtn.disabled=restartBtn.disabled=cancelBtn.disabled=true;try{const r=await fetch('/api/calibration/action?'+q,{cache:'no-store'});s=await r.json();update();if(s.done)setTimeout(()=>{location.href='/'},600)}catch(e){statusEl.textContent='Erreur action';notify('Erreur reseau.')}doneBtn.disabled=submitBtn.disabled=acceptBtn.disabled=verifyTof1Btn.disabled=verifyTof2Btn.disabled=goNoiseBtn.disabled=restartBtn.disabled=cancelBtn.disabled=false}
realInput.onfocus=()=>{realInputEditing=true};
realInput.onblur=()=>{realInputEditing=false};
doneBtn.onclick=()=>{if((params.get('verify')==='1'&&s.step==='verify')||params.get('noise_result')==='1')location.href='/calibration_select';else action('cmd=done')};
submitBtn.onclick=()=>action('cmd=real_fov&value='+encodeURIComponent(realInput.value||'145'));
acceptBtn.onclick=()=>action('cmd=accept');
verifyTof1Btn.onclick=()=>action('cmd=calibrate_tof&target=1');
verifyTof2Btn.onclick=()=>action('cmd=calibrate_tof&target=2');
goNoiseBtn.onclick=()=>action('cmd=go_noise');
restartBtn.onclick=()=>action('cmd=restart');
cancelBtn.onclick=()=>action('cmd=cancel');
verifyPrevBtn.onclick=()=>cycleVerifyView(-1);
verifyNextBtn.onclick=()=>cycleVerifyView(1);
[nrMinAlpha,nrMaxAlpha,nrMinBeta,nrMaxBeta].forEach(el=>{el.oninput=scheduleNoiseAlphaBetaApply});
window.onresize=()=>{drawScene();if(s.step==='noise_done')drawNoisePlots()};setInterval(getState,REFRESH_MS);getState();
</script>
</body>
</html>
)rawliteral";

static TofCalibrationDraft &draft_for_tof(int tof_number) {
  return (tof_number == TOF1) ? calibration_tof1 : calibration_tof2;
}

static int noise_step_index(CalibrationStep step) {
  switch (step) {
    case CAL_NOISE_0: return 0;
    case CAL_NOISE_72: return 1;
    case CAL_NOISE_145: return 2;
    case CAL_NOISE_218: return 3;
    case CAL_NOISE_290: return 4;
    default: return -1;
  }
}

static CalibrationStep noise_step_for_index(int index) {
  switch (index) {
    case 0: return CAL_NOISE_0;
    case 1: return CAL_NOISE_72;
    case 2: return CAL_NOISE_145;
    case 3: return CAL_NOISE_218;
    case 4: return CAL_NOISE_290;
    default: return CAL_NOISE_DONE;
  }
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
  return value >= 0 && value <= get_table_length_mm();
}

static int visual_position_from_raw(int tof_number, int raw_mm) {
  if (!raw_tof_is_valid(raw_mm)) {
    return -1;
  }

  return (tof_number == TOF1) ? get_table_length_mm() - raw_mm : raw_mm;
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

static int estimate_position_noise_from_samples(const int *samples, int count, int fallback_noise_mm) {
  static const int POSITION_NOISE_MARGIN_MM = 2;

  if (count < 4) {
    return fallback_noise_mm;
  }

  int sum_position_mm = 0;
  for (int i = 0; i < count; i++) {
    sum_position_mm += samples[i];
  }

  int mean_position_mm = (int)lroundf((float)sum_position_mm / (float)count);
  int max_position_noise_mm = 0;
  for (int i = 0; i < count; i++) {
    int noise_mm = abs(samples[i] - mean_position_mm);
    if (noise_mm > max_position_noise_mm) {
      max_position_noise_mm = noise_mm;
    }
  }

  return max(DEFAULT_POSITION_NOISE_DEADBAND_MM,
             max_position_noise_mm + POSITION_NOISE_MARGIN_MM);
}

static bool capture_noise_estimate(int target_position_mm,
                                   int *position_noise_mm,
                                   int *speed_noise_mm_s,
                                   int *tof1_position_noise_mm,
                                   int *tof2_position_noise_mm) {
  static const int SAMPLE_COUNT = 28;
  static const uint32_t CAPTURE_TIMEOUT_MS = 2600;
  static const uint32_t SAMPLE_PERIOD_MS = 45;
  static const int SPEED_NOISE_MARGIN_MM_S = 20;

  int count = 0;
  int samples[SAMPLE_COUNT] = {0};
  int tof1_samples[SAMPLE_COUNT] = {0};
  int tof2_samples[SAMPLE_COUNT] = {0};
  int tof1_count = 0;
  int tof2_count = 0;
  int max_static_speed_mm_s = 0;
  int previous_position_mm = 0;
  uint32_t previous_sample_ms = 0;
  uint32_t last_sample_ms = 0;
  uint32_t start_ms = millis();

  reset_controller();

  while (millis() - start_ms < CAPTURE_TIMEOUT_MS && count < SAMPLE_COUNT) {
    update_tof_distances();
    bool valid = compute_ball_position();
    uint32_t now = millis();

    if (valid && now - last_sample_ms >= SAMPLE_PERIOD_MS) {
      int position_mm = get_ball_position();
      samples[count] = position_mm;

      int tof1_position_mm = get_ball_position_from_tof(TOF1);
      int tof2_position_mm = get_ball_position_from_tof(TOF2);
      if (tof1_position_mm >= 0 && tof1_count < SAMPLE_COUNT) {
        tof1_samples[tof1_count++] = tof1_position_mm;
      }
      if (tof2_position_mm >= 0 && tof2_count < SAMPLE_COUNT) {
        tof2_samples[tof2_count++] = tof2_position_mm;
      }

      if (count > 0 && now > previous_sample_ms) {
        int dt_ms = (int)(now - previous_sample_ms);
        int static_speed_mm_s = abs((position_mm - previous_position_mm) * 1000 / dt_ms);
        if (static_speed_mm_s > max_static_speed_mm_s) {
          max_static_speed_mm_s = static_speed_mm_s;
        }
      }

      previous_position_mm = position_mm;
      previous_sample_ms = now;
      last_sample_ms = now;
      count++;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (count < SAMPLE_COUNT / 2) {
    return false;
  }

  int sum_position_mm = 0;
  for (int i = 0; i < count; i++) {
    sum_position_mm += samples[i];
  }

  int mean_position_mm = (int)lroundf((float)sum_position_mm / (float)count);
  int target_bias_mm = abs(mean_position_mm - target_position_mm);
  if (target_bias_mm > 25) {
    return false;
  }

  *position_noise_mm = estimate_position_noise_from_samples(samples,
                                                            count,
                                                            DEFAULT_POSITION_NOISE_DEADBAND_MM);
  *speed_noise_mm_s = max(MIN_SPEED_NOISE_FLOOR_MM_S,
                          max_static_speed_mm_s + SPEED_NOISE_MARGIN_MM_S);
  *tof1_position_noise_mm = estimate_position_noise_from_samples(tof1_samples,
                                                                 tof1_count,
                                                                 DEFAULT_POSITION_NOISE_DEADBAND_MM);
  *tof2_position_noise_mm = estimate_position_noise_from_samples(tof2_samples,
                                                                 tof2_count,
                                                                 DEFAULT_POSITION_NOISE_DEADBAND_MM);
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
  calibration_servo = ServoCalibrationDraft();
  calibration_tof1 = TofCalibrationDraft();
  calibration_tof2 = TofCalibrationDraft();
  calibration_noise = NoiseCalibrationDraft();
  calibration_step = CAL_TOF1_FIND_FOV;
  calibration_mode = CAL_MODE_INITIAL_BOTH;
  calibration_flow_done = false;
  calibration_error_msg = "";
}

static bool apply_servo_calibration_draft(void) {
  bool ok = set_servo_theoretical_angle_range(calibration_servo.theoretical_min_angle,
                                              calibration_servo.theoretical_max_angle);
  ok = set_servo_neutral_offset_us(calibration_servo.neutral_offset_us) && ok;
  ok = set_servo_angle_limits(calibration_servo.limit_min_angle,
                              calibration_servo.limit_max_angle) && ok;
  return ok;
}

static bool set_servo_calibration_draft(int theoretical_min_angle,
                                        int theoretical_max_angle,
                                        int limit_min_angle,
                                        int limit_max_angle,
                                        int neutral_offset_us) {
  int neutral_angle = (theoretical_min_angle + theoretical_max_angle + 1) / 2;

  if (theoretical_min_angle < SERVO_CMD_MIN_DEG ||
      theoretical_max_angle > SERVO_CMD_MAX_DEG ||
      theoretical_min_angle >= theoretical_max_angle ||
      limit_min_angle < theoretical_min_angle ||
      limit_max_angle > theoretical_max_angle ||
      limit_min_angle >= limit_max_angle ||
      neutral_angle < limit_min_angle ||
      neutral_angle > limit_max_angle ||
      neutral_offset_us < -500 ||
      neutral_offset_us > 500) {
    return false;
  }

  calibration_servo.theoretical_min_angle = theoretical_min_angle;
  calibration_servo.theoretical_max_angle = theoretical_max_angle;
  calibration_servo.limit_min_angle = limit_min_angle;
  calibration_servo.limit_max_angle = limit_max_angle;
  calibration_servo.neutral_offset_us = neutral_offset_us;
  return apply_servo_calibration_draft();
}

static void save_draft_to_preferences(void) {
  Preferences prefs;
  prefs.begin(CALIBRATION_NAMESPACE, false);
  prefs.putBool(CALIBRATION_DONE_KEY, true);
  prefs.putUInt(CALIBRATION_VERSION_KEY, CALIBRATION_SCHEMA_VERSION);
  prefs.putInt("sv_tmin", calibration_servo.theoretical_min_angle);
  prefs.putInt("sv_tmax", calibration_servo.theoretical_max_angle);
  prefs.putInt("sv_lmin", calibration_servo.limit_min_angle);
  prefs.putInt("sv_lmax", calibration_servo.limit_max_angle);
  prefs.putInt("sv_off", calibration_servo.neutral_offset_us);
  prefs.putInt("sv_step", calibration_servo.pwm_step_us);
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
  for (int i = 0; i < NOISE_PROFILE_POINT_COUNT; i++) {
    prefs.putInt(("n_p" + String(i)).c_str(), calibration_noise.position_mm[i]);
    prefs.putInt(("n_x" + String(i)).c_str(), calibration_noise.position_noise_mm[i]);
    prefs.putInt(("n_v" + String(i)).c_str(), calibration_noise.speed_noise_mm_s[i]);
    prefs.putInt(("n_t1x" + String(i)).c_str(), calibration_noise.tof1_position_noise_mm[i]);
    prefs.putInt(("n_t2x" + String(i)).c_str(), calibration_noise.tof2_position_noise_mm[i]);
  }
  prefs.end();
}

static bool load_draft_from_preferences(void) {
  Preferences prefs;
  prefs.begin(CALIBRATION_NAMESPACE, true);
  bool done = prefs.getBool(CALIBRATION_DONE_KEY, false);
  uint32_t version = prefs.getUInt(CALIBRATION_VERSION_KEY, 0);
  bool compatible_calibration = done && (version >= 7 && version <= CALIBRATION_SCHEMA_VERSION);

  if (compatible_calibration) {
    int old_min = prefs.getInt("sv_min", SERVO_CMD_MIN_DEG);
    int old_max = prefs.getInt("sv_max", SERVO_CMD_MAX_DEG);
    calibration_servo.theoretical_min_angle = prefs.getInt("sv_tmin", old_min);
    calibration_servo.theoretical_max_angle = prefs.getInt("sv_tmax", old_max);
    calibration_servo.limit_min_angle = prefs.getInt("sv_lmin", calibration_servo.theoretical_min_angle);
    calibration_servo.limit_max_angle = prefs.getInt("sv_lmax", calibration_servo.theoretical_max_angle);
    calibration_servo.neutral_offset_us = prefs.getInt("sv_off", 0);
    calibration_servo.pwm_step_us = prefs.getInt("sv_step", SERVO_CMD_DEFAULT_PWM_STEP_US);
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
    calibration_noise = NoiseCalibrationDraft();
    for (int i = 0; i < NOISE_PROFILE_POINT_COUNT; i++) {
      calibration_noise.position_mm[i] = prefs.getInt(("n_p" + String(i)).c_str(), calibration_noise.position_mm[i]);
      calibration_noise.position_noise_mm[i] = prefs.getInt(("n_x" + String(i)).c_str(), calibration_noise.position_noise_mm[i]);
      calibration_noise.speed_noise_mm_s[i] = prefs.getInt(("n_v" + String(i)).c_str(), calibration_noise.speed_noise_mm_s[i]);
      calibration_noise.tof1_position_noise_mm[i] = prefs.getInt(("n_t1x" + String(i)).c_str(), calibration_noise.tof1_position_noise_mm[i]);
      calibration_noise.tof2_position_noise_mm[i] = prefs.getInt(("n_t2x" + String(i)).c_str(), calibration_noise.tof2_position_noise_mm[i]);
    }
    set_noise_rejection_profile(calibration_noise.position_mm,
                                calibration_noise.position_noise_mm,
                                calibration_noise.speed_noise_mm_s,
                                NOISE_PROFILE_POINT_COUNT);
    set_tof_position_noise_profile(TOF1,
                                   calibration_noise.tof1_position_noise_mm,
                                   NOISE_PROFILE_POINT_COUNT);
    set_tof_position_noise_profile(TOF2,
                                   calibration_noise.tof2_position_noise_mm,
                                   NOISE_PROFILE_POINT_COUNT);
  }

  prefs.end();
  return compatible_calibration;
}

static void save_servo_calibration_to_preferences(void) {
  Preferences prefs;
  prefs.begin(CALIBRATION_NAMESPACE, false);
  prefs.putUInt(CALIBRATION_VERSION_KEY, CALIBRATION_SCHEMA_VERSION);
  prefs.putInt("sv_tmin", calibration_servo.theoretical_min_angle);
  prefs.putInt("sv_tmax", calibration_servo.theoretical_max_angle);
  prefs.putInt("sv_lmin", calibration_servo.limit_min_angle);
  prefs.putInt("sv_lmax", calibration_servo.limit_max_angle);
  prefs.putInt("sv_off", calibration_servo.neutral_offset_us);
  prefs.putInt("sv_step", calibration_servo.pwm_step_us);
  prefs.end();
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

static void start_initial_tof_calibration(void) {
  calibration_tof1 = TofCalibrationDraft();
  calibration_tof2 = TofCalibrationDraft();
  calibration_mode = CAL_MODE_INITIAL_BOTH;
  calibration_step = CAL_TOF1_FIND_FOV;
  calibration_flow_done = false;
  calibration_error_msg = "";
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
  apply_servo_calibration_draft();
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
    apply_servo_calibration_draft();
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
    apply_servo_calibration_draft();
    distance_sensors_calibrated = true;
  } else {
    reset_calibration_drafts();
    calibration_mode = CAL_MODE_VERIFY_ONLY;
    calibration_step = CAL_ERROR;
    distance_sensors_calibrated = false;
    calibration_error_msg = "Aucune calibration sauvegardee a verifier.";
  }
}

static void start_noise_calibration(bool manual_mode) {
  calibration_error_msg = "";
  calibration_flow_done = false;
  calibration_mode = manual_mode ? CAL_MODE_NOISE_ONLY : CAL_MODE_INITIAL_BOTH;
  calibration_step = CAL_NOISE_0;
  set_controller_enabled(false);

  if (load_draft_from_preferences()) {
    apply_draft_calibration(TOF1);
    apply_draft_calibration(TOF2);
    apply_servo_calibration_draft();
    distance_sensors_calibrated = true;
  } else {
    calibration_step = CAL_ERROR;
    distance_sensors_calibrated = false;
    calibration_error_msg = "Aucune calibration TOF sauvegardee. Faites la calibration initiale avant le bruit.";
  }
}

static void start_noise_result_view(void) {
  calibration_error_msg = "";
  calibration_flow_done = false;
  calibration_mode = CAL_MODE_NOISE_ONLY;
  calibration_step = CAL_NOISE_DONE;
  set_controller_enabled(false);

  if (load_draft_from_preferences()) {
    apply_draft_calibration(TOF1);
    apply_draft_calibration(TOF2);
    apply_servo_calibration_draft();
    distance_sensors_calibrated = true;
  } else {
    calibration_step = CAL_ERROR;
    distance_sensors_calibrated = false;
    calibration_error_msg = "Aucun resultat de rejection du bruit sauvegarde.";
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

static bool set_plot_max_seconds(int seconds) {
  if (seconds < PLOT_MIN_MAX_SECONDS || seconds > PLOT_MAX_MAX_SECONDS) {
    return false;
  }

  plot_max_seconds = seconds;
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
  float ab_min_alpha = 0.0f;
  float ab_max_alpha = 0.0f;
  float ab_min_beta = 0.0f;
  float ab_max_beta = 0.0f;
  get_alpha_beta_parameters(&ab_min_alpha, &ab_max_alpha, &ab_min_beta, &ab_max_beta);

  String json = "{";
  json += "\"d1\":" + String(get_d1()) + ",";
  json += "\"d2\":" + String(get_d2()) + ",";
  json += "\"x\":" + String(get_ball_position()) + ",";
  json += "\"v\":" + String(get_ball_speed()) + ",";
  json += "\"speed_valid\":" + String(is_ball_speed_valid() ? "true" : "false") + ",";
  json += "\"servo_angle\":" + String(get_controller_last_angle_deg()) + ",";
  json += "\"stabilization\":" + String(controller_is_enabled() ? "true" : "false") + ",";
  json += "\"controller_valid\":" + String(controller_last_update_was_valid() ? "true" : "false") + ",";
  json += "\"ball_stable\":" + String(controller_ball_is_stable() ? "true" : "false") + ",";
  json += "\"controller_idle\":" + String(controller_is_idle() ? "true" : "false") + ",";
  json += "\"ref\":" + String(get_controller_reference_mm()) + ",";
  json += "\"kp\":" + String(kp, 6) + ",";
  json += "\"ki\":" + String(ki, 6) + ",";
  json += "\"kd\":" + String(kd, 6) + ",";
  json += "\"table_length\":" + String(get_table_length_mm()) + ",";
  json += "\"plot_max_s\":" + String(plot_max_seconds) + ",";
  json += "\"servo_min\":" + String(get_servo_min_angle_deg()) + ",";
  json += "\"servo_max\":" + String(get_servo_max_angle_deg()) + ",";
  json += "\"servo_neutral\":" + String(get_servo_neutral_angle_deg()) + ",";
  json += "\"servo_theoretical_min\":" + String(get_servo_theoretical_min_angle_deg()) + ",";
  json += "\"servo_theoretical_max\":" + String(get_servo_theoretical_max_angle_deg()) + ",";
  json += "\"alpha_beta_min_alpha\":" + String(ab_min_alpha, 4) + ",";
  json += "\"alpha_beta_max_alpha\":" + String(ab_max_alpha, 4) + ",";
  json += "\"alpha_beta_min_beta\":" + String(ab_min_beta, 4) + ",";
  json += "\"alpha_beta_max_beta\":" + String(ab_max_beta, 4) + ",";
  json += "\"manual_angle_step\":" + String(manual_angle_step_deg);
  json += "}";

  server.send(200, "application/json; charset=utf-8", json);
}

static void send_advanced_state(bool ok = true) {
  float ab_min_alpha = 0.0f;
  float ab_max_alpha = 0.0f;
  float ab_min_beta = 0.0f;
  float ab_max_beta = 0.0f;
  get_alpha_beta_parameters(&ab_min_alpha, &ab_max_alpha, &ab_min_beta, &ab_max_beta);

  String json = "{";
  json += "\"ok\":" + String(ok ? "true" : "false") + ",";
  json += "\"max_control_speed\":" + String(get_controller_max_control_speed_mm_s()) + ",";
  json += "\"alpha_beta_min_alpha\":" + String(ab_min_alpha, 4) + ",";
  json += "\"alpha_beta_max_alpha\":" + String(ab_max_alpha, 4) + ",";
  json += "\"alpha_beta_min_beta\":" + String(ab_min_beta, 4) + ",";
  json += "\"alpha_beta_max_beta\":" + String(ab_max_beta, 4) + ",";
  json += "\"controller_period\":" + String(get_controller_period_ms()) + ",";
  json += "\"max_step\":" + String(get_controller_max_step_deg()) + ",";
  json += "\"position_deadband\":" + String(get_controller_stabilization_position_deadband_mm()) + ",";
  json += "\"speed_deadband\":" + String(get_controller_stabilization_speed_deadband_mm_s()) + ",";
  json += "\"stable_time\":" + String(get_controller_stable_time_ms()) + ",";
  json += "\"idle_exit_percent\":" + String(get_controller_idle_exit_percent()) + ",";
  json += "\"lost_delay\":" + String(get_controller_lost_ball_delay_ms()) + ",";
  json += "\"lost_iter\":" + String(get_controller_lost_ball_iter()) + ",";
  json += "\"servo_min\":" + String(get_servo_min_angle_deg()) + ",";
  json += "\"servo_max\":" + String(get_servo_max_angle_deg()) + ",";
  json += "\"servo_step_us\":" + String(calibration_servo.pwm_step_us) + ",";
  json += "\"manual_angle_step\":" + String(manual_angle_step_deg) + ",";
  json += "\"table_length\":" + String(get_table_length_mm()) + ",";
  json += "\"plot_max_s\":" + String(plot_max_seconds);
  json += "}";
  server.send(200, "application/json; charset=utf-8", json);
}

static bool update_advanced_params_from_request(void) {
  bool ok = true;

  if (server.hasArg("max_speed")) {
    ok = set_controller_max_control_speed_mm_s(server.arg("max_speed").toInt()) && ok;
  }

  float ab_min_alpha = 0.0f;
  float ab_max_alpha = 0.0f;
  float ab_min_beta = 0.0f;
  float ab_max_beta = 0.0f;
  get_alpha_beta_parameters(&ab_min_alpha, &ab_max_alpha, &ab_min_beta, &ab_max_beta);

  if (server.hasArg("ab_min_alpha")) ab_min_alpha = server.arg("ab_min_alpha").toFloat();
  if (server.hasArg("ab_max_alpha")) ab_max_alpha = server.arg("ab_max_alpha").toFloat();
  if (server.hasArg("ab_min_beta")) ab_min_beta = server.arg("ab_min_beta").toFloat();
  if (server.hasArg("ab_max_beta")) ab_max_beta = server.arg("ab_max_beta").toFloat();
  if (server.hasArg("ab_min_alpha") || server.hasArg("ab_max_alpha") ||
      server.hasArg("ab_min_beta") || server.hasArg("ab_max_beta")) {
    ok = set_alpha_beta_parameters(ab_min_alpha, ab_max_alpha,
                                   ab_min_beta, ab_max_beta) && ok;
  }

  if (server.hasArg("ctrl_period")) {
    ok = set_controller_period_ms((uint32_t)server.arg("ctrl_period").toInt()) && ok;
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

  if (server.hasArg("stable_time")) {
    ok = set_controller_stable_time_ms((uint32_t)server.arg("stable_time").toInt()) && ok;
  }

  if (server.hasArg("idle_exit")) {
    ok = set_controller_idle_exit_percent(server.arg("idle_exit").toInt()) && ok;
  }

  if (server.hasArg("lost_delay")) {
    ok = set_controller_lost_ball_delay_ms((uint32_t)server.arg("lost_delay").toInt()) && ok;
  }

  if (server.hasArg("lost_iter")) {
    ok = set_controller_lost_ball_iter(server.arg("lost_iter").toInt()) && ok;
  }

  int servo_min = get_servo_min_angle_deg();
  int servo_max = get_servo_max_angle_deg();

  if (server.hasArg("servo_min")) servo_min = server.arg("servo_min").toInt();
  if (server.hasArg("servo_max")) servo_max = server.arg("servo_max").toInt();
  if (server.hasArg("servo_min") || server.hasArg("servo_max")) {
    ok = set_servo_angle_limits(servo_min, servo_max) && ok;
    if(ok) {
      calibration_servo.limit_min_angle = servo_min;
      calibration_servo.limit_max_angle = servo_max;
    }
    reset_controller();
  }

  if (server.hasArg("servo_step")) {
    int servo_step = server.arg("servo_step").toInt();
    if(servo_step < 1 || servo_step > 100) {
      ok = false;
    } else {
      calibration_servo.pwm_step_us = servo_step;
    }
  }

  if (server.hasArg("manual_step")) {
    int manual_step = server.arg("manual_step").toInt();
    if(manual_step < MANUAL_ANGLE_MIN_STEP_DEG || manual_step > MANUAL_ANGLE_MAX_STEP_DEG) {
      ok = false;
    } else {
      manual_angle_step_deg = manual_step;
    }
  }

  if (server.hasArg("table_len")) {
    ok = set_table_length_mm(server.arg("table_len").toInt()) && ok;
  }

  if (server.hasArg("plot_max")) {
    ok = set_plot_max_seconds(server.arg("plot_max").toInt()) && ok;
  }

  return ok;
}

static void reset_all_advanced_parameters(void) {
  reset_ball_position_advanced_parameters();
  reset_servo_advanced_parameters();
  reset_controller_advanced_parameters();
  calibration_servo.limit_min_angle = get_servo_min_angle_deg();
  calibration_servo.limit_max_angle = get_servo_max_angle_deg();
  calibration_servo.pwm_step_us = SERVO_CMD_DEFAULT_PWM_STEP_US;
  manual_angle_step_deg = MANUAL_ANGLE_DEFAULT_STEP_DEG;
  plot_max_seconds = PLOT_DEFAULT_MAX_SECONDS;
}

static bool advanced_settings_match_saved(void) {
  float ab_min_alpha = 0.0f;
  float ab_max_alpha = 0.0f;
  float ab_min_beta = 0.0f;
  float ab_max_beta = 0.0f;
  get_alpha_beta_parameters(&ab_min_alpha, &ab_max_alpha, &ab_min_beta, &ab_max_beta);

  Preferences prefs;
  prefs.begin(ADVANCED_NAMESPACE, true);
  uint32_t version = prefs.getUInt(ADVANCED_VERSION_KEY, 0);

  if (version != ADVANCED_SCHEMA_VERSION) {
    prefs.end();
    return false;
  }

  bool same = prefs.getInt("max_speed", -1) == get_controller_max_control_speed_mm_s() &&
              fabsf(prefs.getFloat("ab_min_a", -1.0f) - ab_min_alpha) <= CONTROLLER_SAVE_FLOAT_EPSILON &&
              fabsf(prefs.getFloat("ab_max_a", -1.0f) - ab_max_alpha) <= CONTROLLER_SAVE_FLOAT_EPSILON &&
              fabsf(prefs.getFloat("ab_min_b", -1.0f) - ab_min_beta) <= CONTROLLER_SAVE_FLOAT_EPSILON &&
              fabsf(prefs.getFloat("ab_max_b", -1.0f) - ab_max_beta) <= CONTROLLER_SAVE_FLOAT_EPSILON &&
              prefs.getUInt("ctrl_period", UINT32_MAX) == get_controller_period_ms() &&
              prefs.getInt("max_step", -1) == get_controller_max_step_deg() &&
              prefs.getInt("pos_db", -1) == get_controller_stabilization_position_deadband_mm() &&
              prefs.getInt("speed_db", -1) == get_controller_stabilization_speed_deadband_mm_s() &&
              prefs.getUInt("stable_time", UINT32_MAX) == get_controller_stable_time_ms() &&
              prefs.getInt("idle_exit", -1) == get_controller_idle_exit_percent() &&
              prefs.getUInt("lost_delay", UINT32_MAX) == get_controller_lost_ball_delay_ms() &&
              prefs.getInt("lost_iter", -1) == get_controller_lost_ball_iter() &&
              prefs.getInt("servo_min", -1) == get_servo_min_angle_deg() &&
              prefs.getInt("servo_max", -1) == get_servo_max_angle_deg() &&
              prefs.getInt("servo_step", -1) == calibration_servo.pwm_step_us &&
              prefs.getInt("manual_step", -1) == manual_angle_step_deg &&
              prefs.getInt("table_len", -1) == get_table_length_mm() &&
              prefs.getInt("plot_max", -1) == plot_max_seconds;

  prefs.end();
  return same;
}

static bool save_advanced_settings(void) {
  float ab_min_alpha = 0.0f;
  float ab_max_alpha = 0.0f;
  float ab_min_beta = 0.0f;
  float ab_max_beta = 0.0f;
  get_alpha_beta_parameters(&ab_min_alpha, &ab_max_alpha, &ab_min_beta, &ab_max_beta);

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
  prefs.putInt("max_speed", get_controller_max_control_speed_mm_s());
  prefs.putFloat("ab_min_a", ab_min_alpha);
  prefs.putFloat("ab_max_a", ab_max_alpha);
  prefs.putFloat("ab_min_b", ab_min_beta);
  prefs.putFloat("ab_max_b", ab_max_beta);
  prefs.putUInt("ctrl_period", get_controller_period_ms());
  prefs.putInt("max_step", get_controller_max_step_deg());
  prefs.putInt("pos_db", get_controller_stabilization_position_deadband_mm());
  prefs.putInt("speed_db", get_controller_stabilization_speed_deadband_mm_s());
  prefs.putUInt("stable_time", get_controller_stable_time_ms());
  prefs.putInt("idle_exit", get_controller_idle_exit_percent());
  prefs.putUInt("lost_delay", get_controller_lost_ball_delay_ms());
  prefs.putInt("lost_iter", get_controller_lost_ball_iter());
  prefs.putInt("servo_min", get_servo_min_angle_deg());
  prefs.putInt("servo_max", get_servo_max_angle_deg());
  prefs.putInt("servo_step", calibration_servo.pwm_step_us);
  prefs.putInt("manual_step", manual_angle_step_deg);
  prefs.putInt("table_len", get_table_length_mm());
  prefs.putInt("plot_max", plot_max_seconds);
  prefs.end();

  if (distance_sensors_calibrated) {
    calibration_servo.limit_min_angle = get_servo_min_angle_deg();
    calibration_servo.limit_max_angle = get_servo_max_angle_deg();
    save_servo_calibration_to_preferences();
  }

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

  int max_speed = prefs.getInt("max_speed", CONTROLLER_DEFAULT_MAX_CONTROL_SPEED_MM_S);
  float ab_min_alpha = prefs.getFloat("ab_min_a", ALPHA_BETA_DEFAULT_MIN_ALPHA);
  float ab_max_alpha = prefs.getFloat("ab_max_a", ALPHA_BETA_DEFAULT_MAX_ALPHA);
  float ab_min_beta = prefs.getFloat("ab_min_b", ALPHA_BETA_DEFAULT_MIN_BETA);
  float ab_max_beta = prefs.getFloat("ab_max_b", ALPHA_BETA_DEFAULT_MAX_BETA);
  uint32_t controller_period = prefs.getUInt("ctrl_period", CONTROLLER_DEFAULT_PERIOD_MS);
  int max_step = prefs.getInt("max_step", CONTROLLER_DEFAULT_MAX_STEP_DEG);
  int pos_db = prefs.getInt("pos_db", CONTROLLER_DEFAULT_POSITION_DEADBAND_MM);
  int speed_db = prefs.getInt("speed_db", CONTROLLER_DEFAULT_SPEED_DEADBAND_MM_S);
  uint32_t stable_time = prefs.getUInt("stable_time", CONTROLLER_DEFAULT_STABLE_TIME_MS);
  int idle_exit = prefs.getInt("idle_exit", CONTROLLER_DEFAULT_IDLE_EXIT_PERCENT);
  uint32_t lost_delay = prefs.getUInt("lost_delay", CONTROLLER_DEFAULT_LOST_BALL_DELAY_MS);
  int lost_iter = prefs.getInt("lost_iter", CONTROLLER_DEFAULT_LOST_BALL_ITER);
  int servo_min = prefs.getInt("servo_min", SERVO_CMD_DEFAULT_LIMIT_MIN_DEG);
  int servo_max = prefs.getInt("servo_max", SERVO_CMD_DEFAULT_LIMIT_MAX_DEG);
  int servo_step = prefs.getInt("servo_step", SERVO_CMD_DEFAULT_PWM_STEP_US);
  int manual_step = prefs.getInt("manual_step", MANUAL_ANGLE_DEFAULT_STEP_DEG);
  int table_len = prefs.getInt("table_len", TABLE_LENGTH_DEFAULT_MM);
  int saved_plot_max = prefs.getInt("plot_max", PLOT_DEFAULT_MAX_SECONDS);
  prefs.end();

  bool ok = true;
  ok = set_controller_max_control_speed_mm_s(max_speed) && ok;
  ok = set_alpha_beta_parameters(ab_min_alpha, ab_max_alpha, ab_min_beta, ab_max_beta) && ok;
  ok = set_controller_period_ms(controller_period) && ok;
  ok = set_controller_max_step_deg(max_step) && ok;
  ok = set_controller_stabilization_position_deadband_mm(pos_db) && ok;
  ok = set_controller_stabilization_speed_deadband_mm_s(speed_db) && ok;
  ok = set_controller_stable_time_ms(stable_time) && ok;
  ok = set_controller_idle_exit_percent(idle_exit) && ok;
  ok = set_controller_lost_ball_delay_ms(lost_delay) && ok;
  ok = set_controller_lost_ball_iter(lost_iter) && ok;
  ok = set_servo_angle_limits(servo_min, servo_max) && ok;
  if(servo_step >= 1 && servo_step <= 100) {
    calibration_servo.pwm_step_us = servo_step;
  } else {
    ok = false;
  }
  if(manual_step >= MANUAL_ANGLE_MIN_STEP_DEG && manual_step <= MANUAL_ANGLE_MAX_STEP_DEG) {
    manual_angle_step_deg = manual_step;
  } else {
    ok = false;
  }
  ok = set_table_length_mm(table_len) && ok;
  ok = set_plot_max_seconds(saved_plot_max) && ok;
  reset_controller();
  return ok;
}

static void step_servo_to_neutral(void) {
  static const int RETURN_STEP_DEG = 2;

  int current = get_controller_last_angle_deg();
  int target = get_servo_neutral_angle_deg();

  if (abs(current - target) <= RETURN_STEP_DEG) {
    set_controller_manual_angle(target);
    neutral_return_pending = false;
    return;
  }

  int next_angle = current + ((target > current) ? RETURN_STEP_DEG : -RETURN_STEP_DEG);
  set_controller_manual_angle(next_angle);
}

static void update_web_client_control_mode(void) {
  if (!distance_sensors_calibrated) {
    return;
  }

  uint32_t now = millis();
  bool has_client = WiFi.softAPgetStationNum() > 0;

  if (!client_mode_initialized || has_client != web_client_present) {
    client_mode_initialized = true;
    web_client_present = has_client;

    if (has_client) {
      auto_stabilization_without_client = false;
      neutral_return_pending = true;
      set_controller_enabled(false);
    } else {
      neutral_return_pending = false;
      load_controller_settings();
      set_controller_enabled(true);
      auto_stabilization_without_client = true;
    }
  }

  if (!has_client && !controller_is_enabled()) {
    load_controller_settings();
    set_controller_enabled(true);
    auto_stabilization_without_client = true;
  }

  if (has_client && neutral_return_pending && !controller_is_enabled() &&
      now - last_client_mode_update_ms >= 50) {
    last_client_mode_update_ms = now;
    step_servo_to_neutral();
  }
}

static void send_servo_calibration_state(bool ok = true) {
  String json = "{";
  json += "\"ok\":" + String(ok ? "true" : "false") + ",";
  json += "\"theoretical_min_angle\":" + String(calibration_servo.theoretical_min_angle) + ",";
  json += "\"theoretical_max_angle\":" + String(calibration_servo.theoretical_max_angle) + ",";
  json += "\"limit_min_angle\":" + String(calibration_servo.limit_min_angle) + ",";
  json += "\"limit_max_angle\":" + String(calibration_servo.limit_max_angle) + ",";
  json += "\"neutral_angle\":" + String(get_servo_neutral_angle_deg()) + ",";
  json += "\"current_angle\":" + String(get_servo_angle()) + ",";
  json += "\"current_pwm_us\":" + String(get_servo_current_pulse_us()) + ",";
  json += "\"neutral_offset_us\":" + String(calibration_servo.neutral_offset_us) + ",";
  json += "\"pwm_step_us\":" + String(calibration_servo.pwm_step_us) + ",";
  json += "\"error\":\"" + calibration_error_msg + "\"";
  json += "}";

  server.send(200, "application/json; charset=utf-8", json);
}

static void start_servo_calibration(bool initial_mode) {
  calibration_error_msg = "";
  servo_calibration_initial_in_progress = initial_mode;
  set_controller_enabled(false);

  if (!initial_mode && load_draft_from_preferences()) {
    apply_servo_calibration_draft();
  } else {
    calibration_servo.theoretical_min_angle = get_servo_theoretical_min_angle_deg();
    calibration_servo.theoretical_max_angle = get_servo_theoretical_max_angle_deg();
    calibration_servo.limit_min_angle = get_servo_min_angle_deg();
    calibration_servo.limit_max_angle = get_servo_max_angle_deg();
    calibration_servo.neutral_offset_us = get_servo_neutral_offset_us();
  }

  set_servo_calibration_pulse_us(SERVO_CMD_NEUTRAL_PULSE_US);
  reset_controller();
}

static void handle_servo_calibration_action(void) {
  String cmd = server.arg("cmd");
  calibration_error_msg = "";

  if (cmd == "start") {
    start_servo_calibration(server.arg("mode") == "initial");
    send_servo_calibration_state();
    return;
  }

  if (cmd == "preview") {
    int min_angle = server.arg("min").toInt();
    int max_angle = server.arg("max").toInt();
    int limit_min = server.hasArg("limit_min") ? server.arg("limit_min").toInt() : min_angle;
    int limit_max = server.hasArg("limit_max") ? server.arg("limit_max").toInt() : max_angle;
    int offset_us = server.hasArg("offset") ? server.arg("offset").toInt() : calibration_servo.neutral_offset_us;

    if (!set_servo_calibration_draft(min_angle, max_angle, limit_min, limit_max, offset_us)) {
      calibration_error_msg = "Angles servo invalides.";
      send_servo_calibration_state(false);
      return;
    }

    send_servo_calibration_state();
    return;
  }

  if (cmd == "pwm") {
    int pulse_us = server.arg("value").toInt();
    if (!set_servo_calibration_pulse_us((uint16_t)pulse_us)) {
      calibration_error_msg = "PWM servo invalide.";
      send_servo_calibration_state(false);
      return;
    }
    send_servo_calibration_state();
    return;
  }

  if (cmd == "offset") {
    int offset_us = (int)get_servo_current_pulse_us() - SERVO_CMD_NEUTRAL_PULSE_US;
    calibration_servo.neutral_offset_us = offset_us;
    if (!set_servo_neutral_offset_us(offset_us)) {
      calibration_error_msg = "Offset servo invalide.";
      send_servo_calibration_state(false);
      return;
    }
    set_servo_angle(get_servo_neutral_angle_deg());
    reset_controller();
    send_servo_calibration_state();
    return;
  }

  if (cmd == "angle") {
    int angle = server.arg("value").toInt();
    set_servo_angle(angle);
    send_servo_calibration_state();
    return;
  }

  if (cmd == "save") {
    int min_angle = server.arg("min").toInt();
    int max_angle = server.arg("max").toInt();
    int limit_min = server.hasArg("limit_min") ? server.arg("limit_min").toInt() : min_angle;
    int limit_max = server.hasArg("limit_max") ? server.arg("limit_max").toInt() : max_angle;
    int offset_us = server.hasArg("offset") ? server.arg("offset").toInt() : calibration_servo.neutral_offset_us;
    int step_us = server.hasArg("step") ? server.arg("step").toInt() : calibration_servo.pwm_step_us;

    if(step_us < 1 || step_us > 100) {
      calibration_error_msg = "Pas PWM invalide.";
      send_servo_calibration_state(false);
      return;
    }
    calibration_servo.pwm_step_us = step_us;

    if (!set_servo_calibration_draft(min_angle, max_angle, limit_min, limit_max, offset_us)) {
      calibration_error_msg = "Angles servo invalides. Verifiez que min < max et que les valeurs restent entre 0 et 180 deg.";
      send_servo_calibration_state(false);
      return;
    }

    set_servo_angle(get_servo_neutral_angle_deg());
    reset_controller();

    if (distance_sensors_calibrated) {
      save_servo_calibration_to_preferences();
      save_advanced_settings();
    }

    send_servo_calibration_state();
    return;
  }

  calibration_error_msg = "Commande servo inconnue.";
  send_servo_calibration_state(false);
}

static void send_calibration_state(void) {
  update_tof_distances();
  bool position_valid = compute_ball_position();
  float ab_min_alpha = 0.0f;
  float ab_max_alpha = 0.0f;
  float ab_min_beta = 0.0f;
  float ab_max_beta = 0.0f;
  get_alpha_beta_parameters(&ab_min_alpha, &ab_max_alpha, &ab_min_beta, &ab_max_beta);

  int tof_number = current_calibration_tof();
  int raw_mm = (tof_number == 0) ? -1 : raw_tof_value(tof_number);
  bool raw_valid = raw_tof_is_valid(raw_mm);
  int noise_index = noise_step_index(calibration_step);
  int visual_pos = (calibration_step == CAL_VERIFY) ? get_ball_position()
                   : (noise_index >= 0) ? calibration_noise.position_mm[noise_index]
                   : (calibration_step == CAL_NOISE_DONE) ? get_ball_position()
                   : visual_position_from_raw(tof_number, raw_mm);
  int visual_target = (noise_index >= 0) ? calibration_noise.position_mm[noise_index]
                      : (calibration_step == CAL_NOISE_DONE) ? get_controller_reference_mm()
                      : 145;
  int tof1_visual_pos = get_ball_position_from_tof(TOF1);
  int tof2_visual_pos = get_ball_position_from_tof(TOF2);
  int default_verify_tof = (calibration_step == CAL_VERIFY) ? manual_calibration_target() : 0;
  int live_position_mm = position_valid ? get_ball_position() : -1;
  bool noise_capture_ready = (noise_index >= 0) &&
                             live_position_mm >= 0 &&
                             abs(live_position_mm - calibration_noise.position_mm[noise_index]) <=
                                 NOISE_CAPTURE_TARGET_TOLERANCE_MM;

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
    case CAL_NOISE_0:
    case CAL_NOISE_72:
    case CAL_NOISE_145:
    case CAL_NOISE_218:
    case CAL_NOISE_290:
      step_name = (noise_index == 0) ? "noise_0" :
                  (noise_index == 1) ? "noise_72" :
                  (noise_index == 2) ? "noise_145" :
                  (noise_index == 3) ? "noise_218" : "noise_290";
      title = "Noise rejection";
      instruction = "Placez la balle immobile sur la fleche indiquee, puis cliquez sur Capture bruit.";
      needs_done = true;
      tof_number = 0;
      break;
    case CAL_NOISE_DONE:
      step_name = "noise_done";
      title = "Noise rejection terminee";
      instruction = "Les seuils de bruit ont ete mesures. Validez pour sauvegarder ce profil de rejection.";
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
  json += "\"raw_pos_mm\":" + String(get_ball_position_raw()) + ",";
  json += "\"speed_mm_s\":" + String(get_ball_speed()) + ",";
  json += "\"speed_valid\":" + String(is_ball_speed_valid() ? "true" : "false") + ",";
  json += "\"visual_target_mm\":" + String(visual_target) + ",";
  json += "\"live_pos_mm\":" + String(live_position_mm) + ",";
  json += "\"noise_capture_tolerance_mm\":" + String(NOISE_CAPTURE_TARGET_TOLERANCE_MM) + ",";
  json += "\"noise_capture_ready\":" + String(noise_capture_ready ? "true" : "false") + ",";
  json += "\"alpha_beta_min_alpha\":" + String(ab_min_alpha, 4) + ",";
  json += "\"alpha_beta_max_alpha\":" + String(ab_max_alpha, 4) + ",";
  json += "\"alpha_beta_min_beta\":" + String(ab_min_beta, 4) + ",";
  json += "\"alpha_beta_max_beta\":" + String(ab_max_beta, 4) + ",";
  json += "\"tof1_pos_mm\":" + String(tof1_visual_pos) + ",";
  json += "\"tof2_pos_mm\":" + String(tof2_visual_pos) + ",";
  json += "\"tof1_fov_pos_mm\":" + String(get_tof_fov_position_mm(TOF1)) + ",";
  json += "\"tof2_fov_pos_mm\":" + String(get_tof_fov_position_mm(TOF2)) + ",";
  json += "\"default_verify_tof\":" + String(default_verify_tof) + ",";
  json += "\"needs_done\":" + String(needs_done ? "true" : "false") + ",";
  json += "\"needs_real_input\":" + String(needs_real_input ? "true" : "false") + ",";
  json += "\"real_fov\":" + String(real_fov) + ",";
  json += "\"table_length\":" + String(get_table_length_mm()) + ",";
  json += "\"status\":\"\",";
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
    } else if (server.arg("mode") == "initial_tofs") {
      start_initial_tof_calibration();
    } else if (server.arg("mode") == "verify") {
      start_verify_calibration();
    } else if (server.arg("mode") == "noise") {
      start_noise_calibration(true);
    } else if (server.arg("mode") == "noise_result") {
      start_noise_result_view();
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

  if (cmd == "calibrate_tof" && calibration_step == CAL_VERIFY) {
    int target = server.arg("target").toInt();

    if (target != TOF1 && target != TOF2) {
      calibration_error_msg = "TOF a calibrer invalide.";
      send_calibration_state();
      return;
    }

    save_draft_to_preferences();
    distance_sensors_calibrated = true;
    start_manual_calibration(target);
    send_calibration_state();
    return;
  }

  if (cmd == "go_noise" && calibration_step == CAL_VERIFY) {
    save_draft_to_preferences();
    distance_sensors_calibrated = true;
    start_noise_calibration(false);
    send_calibration_state();
    return;
  }

  if (cmd == "restart") {
    int manual_target = manual_calibration_target();

    if (calibration_mode == CAL_MODE_VERIFY_ONLY) {
      start_verify_calibration();
    } else if (calibration_mode == CAL_MODE_NOISE_ONLY) {
      start_noise_calibration(true);
    } else if (noise_step_index(calibration_step) >= 0 || calibration_step == CAL_NOISE_DONE) {
      start_noise_calibration(false);
    } else if (calibration_mode == CAL_MODE_INITIAL_BOTH || manual_target == 0) {
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
    servo_calibration_initial_in_progress = false;
    if (calibration_mode == CAL_MODE_INITIAL_BOTH) {
      start_noise_calibration(false);
    } else {
      calibration_flow_done = true;
    }
    send_calibration_state();
    return;
  }

  if (cmd == "accept" && calibration_step == CAL_NOISE_DONE) {
    set_noise_rejection_profile(calibration_noise.position_mm,
                                calibration_noise.position_noise_mm,
                                calibration_noise.speed_noise_mm_s,
                                NOISE_PROFILE_POINT_COUNT);
    set_tof_position_noise_profile(TOF1,
                                   calibration_noise.tof1_position_noise_mm,
                                   NOISE_PROFILE_POINT_COUNT);
    set_tof_position_noise_profile(TOF2,
                                   calibration_noise.tof2_position_noise_mm,
                                   NOISE_PROFILE_POINT_COUNT);
    save_draft_to_preferences();
    distance_sensors_calibrated = true;
    servo_calibration_initial_in_progress = false;
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
    int noise_index = noise_step_index(calibration_step);
    if (noise_index >= 0) {
      int target_position_mm = calibration_noise.position_mm[noise_index];
      int position_noise_mm = DEFAULT_POSITION_NOISE_DEADBAND_MM;
      int speed_noise_mm_s = DEFAULT_SPEED_NOISE_DEADBAND_MM_S;
      int tof1_position_noise_mm = DEFAULT_POSITION_NOISE_DEADBAND_MM;
      int tof2_position_noise_mm = DEFAULT_POSITION_NOISE_DEADBAND_MM;

      update_tof_distances();
      bool position_valid = compute_ball_position();
      int current_position_mm = position_valid ? get_ball_position() : -1;
      if (current_position_mm < 0 ||
          abs(current_position_mm - target_position_mm) > NOISE_CAPTURE_TARGET_TOLERANCE_MM) {
        calibration_error_msg = "Balle hors de la zone de capture. Placez-la pres de la cible avant de mesurer le bruit.";
        send_calibration_state();
        return;
      }

      if (!capture_noise_estimate(target_position_mm,
                                  &position_noise_mm,
                                  &speed_noise_mm_s,
                                  &tof1_position_noise_mm,
                                  &tof2_position_noise_mm)) {
        calibration_error_msg = "Impossible de mesurer le bruit. Verifiez que la balle est immobile et visible.";
        send_calibration_state();
        return;
      }

      calibration_noise.position_noise_mm[noise_index] = position_noise_mm;
      calibration_noise.speed_noise_mm_s[noise_index] = speed_noise_mm_s;
      calibration_noise.tof1_position_noise_mm[noise_index] = tof1_position_noise_mm;
      calibration_noise.tof2_position_noise_mm[noise_index] = tof2_position_noise_mm;
      calibration_step = noise_step_for_index(noise_index + 1);
      send_calibration_state();
      return;
    }

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

  server.on("/servo_calibration", HTTP_GET, []() {
    if (distance_sensors_calibrated || server.hasArg("initial")) {
      server.send_P(200, "text/html; charset=utf-8", SERVO_CALIBRATION_HTML);
    } else {
      server.send_P(200, "text/html; charset=utf-8", WELCOME_HTML);
    }
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

  server.on("/api/servo_calibration/state", HTTP_GET, []() {
    send_servo_calibration_state();
  });

  server.on("/api/servo_calibration/action", HTTP_GET, []() {
    if (!distance_sensors_calibrated &&
        !servo_calibration_initial_in_progress &&
        !(server.arg("mode") == "initial" && server.arg("cmd") == "start")) {
      server.send(423, "application/json; charset=utf-8", "{\"calibrated\":false}");
      return;
    }

    handle_servo_calibration_action();
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
      neutral_return_pending = false;
      set_controller_enabled(server.arg("stabilization").toInt() != 0);
    }

    if (server.hasArg("angle")) {
      neutral_return_pending = false;
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

  load_advanced_settings();
  load_controller_settings();
  distance_sensors_calibrated = load_calibration_done();
  if (distance_sensors_calibrated) {
    apply_draft_calibration(TOF1);
    apply_draft_calibration(TOF2);
    apply_servo_calibration_draft();
    reset_controller();
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
    update_web_client_control_mode();
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

bool load_startup_persistent_settings(void) {
  bool calibration_loaded = load_draft_from_preferences();

  if (calibration_loaded) {
    apply_servo_calibration_draft();
  } else {
    calibration_servo = ServoCalibrationDraft();
    apply_servo_calibration_draft();
  }

  load_advanced_settings();
  load_controller_settings();
  return calibration_loaded;
}
