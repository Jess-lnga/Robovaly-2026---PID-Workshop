#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ===================== WiFi SoftAP =====================
static const char* AP_SSID = "Kit_Robovaly";
static const char* AP_PASS = "robovaly123";

// ===================== I2C pins =====================
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;

// ===================== XSHUT pins =====================
static const int XSHUT_RIGHT = 16;
static const int XSHUT_LEFT  = 17;

// ===================== I2C addresses =====================
static const uint8_t ADDR_RIGHT = 0x30;
static const uint8_t ADDR_LEFT  = 0x31;

// ===================== Servo (MG90S) =====================
static const int SERVO_PIN = 18;
static const int SERVO_CH  = 0;
static const int SERVO_FREQ = 50;
static const int SERVO_RES_BITS = 16;

static const uint16_t SERVO_MIN_US = 1000;
static const uint16_t SERVO_MAX_US = 2000;

// ===================== PID constants =====================
static const int16_t REF_MM = 140;
static const int     SERVO_NEUTRAL_DEG = 90;
static const int16_t TABLE_LEN_MM = 280;

static const int     SERVO_MIN_DEG = 0;
static const int     SERVO_MAX_DEG = 180;

// PID timing (fixe)
static const uint32_t PID_PERIOD_MS = 10;     // 100 Hz
static const float    PID_DT_S = PID_PERIOD_MS * 0.001f;

// Anti-windup clamp sur I (unité: mm*s)
static const float    ITERM_CLAMP = 5000.0f;

// Filtre vitesse (pour D)
static const float    VEL_LP_ALPHA = 0.25f;   // 0..1 (plus petit = plus filtré)

// ===================== Shared variables =====================
static portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

static volatile int16_t distRight = -1;
static volatile int16_t distLeft  = -1;

static volatile bool manualMode = true; // défaut: manuel ON

static volatile int  servoAngleCmd     = SERVO_NEUTRAL_DEG;
static volatile int  servoAngleApplied = SERVO_NEUTRAL_DEG;

// Gains PID
static volatile float Kp = 1.0f;
static volatile float Ki = 0.0f;   // deg / (mm*s)
static volatile float Kd = 0.0f;   // deg / (mm/s)

// Reset demandé (gains changés)
static volatile bool pidResetRequest = false;

// Télémétrie position/vitesse
static volatile float ballPosMm   = NAN;
static volatile float ballVelMmS  = NAN;

// ===================== Sensors =====================
Adafruit_VL53L0X tofRight;
Adafruit_VL53L0X tofLeft;
static bool rightOk = false;
static bool leftOk  = false;

// ===================== Web server =====================
WebServer server(80);

// ===================== HTML UI =====================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="fr">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>Prototype Kit PID</title>
  <style>
    :root { --bg:#0b1220; --card:#121b2f; --text:#e9eefc; --muted:#9fb0d0; }
    body{
      margin:0; font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial;
      background: radial-gradient(1200px 600px at 20% 0%, #172447 0%, var(--bg) 55%);
      color: var(--text); min-height:100vh; display:flex; align-items:center; justify-content:center;
      padding:24px;
    }
    .wrap{ width:min(980px, 100%); }
    .title{ font-size: 28px; font-weight: 800; margin: 0 0 14px 0; }
    .subtitle{ color: var(--muted); margin:0 0 18px 0; line-height:1.4; }
    .stack{ display:grid; gap:16px; }
    .card{
      background: color-mix(in oklab, var(--card) 92%, black 8%);
      border: 1px solid rgba(255,255,255,.08);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 10px 30px rgba(0,0,0,.35);
    }
    .grid{ display:grid; grid-template-columns: repeat(2, minmax(0,1fr)); gap:16px; }
    .grid3{ display:grid; grid-template-columns: repeat(3, minmax(0,1fr)); gap:16px; }
    .label{ color: var(--muted); font-weight: 800; }
    .value{ font-size: 44px; font-weight: 900; margin-top:10px; }
    .valueSm{ font-size: 34px; font-weight: 900; margin-top:10px; }
    .unit{ font-size: 14px; color: var(--muted); font-weight: 700; margin-left:6px; }
    .row{ display:flex; align-items:baseline; gap:8px; }
    .pill{
      display:inline-flex; align-items:center; gap:8px;
      margin-top: 12px; padding: 8px 10px;
      border-radius: 999px; border:1px solid rgba(255,255,255,.10);
      color: var(--muted); font-weight:700; font-size:12px;
    }
    .dot{ width:10px; height:10px; border-radius:999px; background:#2dd4bf; box-shadow:0 0 18px rgba(45,212,191,.5); }

    .servoHead{ display:flex; justify-content:space-between; gap:12px; align-items:baseline; }
    .servoVal{ font-size: 38px; font-weight: 900; }
    .servoUnit{ font-size: 14px; color: var(--muted); font-weight:700; margin-left:6px; }
    input[type="range"]{ width:100%; margin-top: 14px; accent-color: #2dd4bf; }
    .hint{ margin-top:10px; color: var(--muted); font-size: 12px; line-height:1.4; }

    .switchRow{ display:flex; align-items:center; justify-content:space-between; gap:12px; margin-top:8px; }
    .switch{
      position: relative; width: 52px; height: 30px; flex: 0 0 auto;
    }
    .switch input{ display:none; }
    .track{
      position:absolute; inset:0; border-radius:999px;
      background: rgba(255,255,255,.12);
      border: 1px solid rgba(255,255,255,.14);
      transition: .15s ease;
    }
    .thumb{
      position:absolute; top:4px; left:4px;
      width:22px; height:22px; border-radius:999px;
      background:#fb7185;
      box-shadow:0 8px 20px rgba(0,0,0,.35);
      transition: .15s ease;
    }
    .switch input:checked + .track{ background: rgba(45,212,191,.18); border-color: rgba(45,212,191,.35); }
    .switch input:checked + .track .thumb{ transform: translateX(22px); background:#2dd4bf; }

    .disabled { opacity:.55; filter:saturate(.8); }

    .pidGrid{ display:grid; grid-template-columns: repeat(3, minmax(0,1fr)); gap:12px; margin-top: 12px; }
    .field{
      border:1px solid rgba(255,255,255,.10);
      border-radius: 14px;
      padding: 10px;
      background: rgba(255,255,255,.03);
    }
    .field label{ display:block; font-size:12px; color: var(--muted); font-weight:800; margin-bottom:6px; }
    .field input{
      width:100%;
      padding:10px 10px;
      border-radius: 12px;
      border:1px solid rgba(255,255,255,.10);
      background: rgba(0,0,0,.18);
      color: var(--text);
      font-size: 14px;
      outline: none;
    }
    .field input:disabled{ opacity:.6; }

    .foot{ margin-top: 16px; color: var(--muted); font-size: 12px; }
    @media (max-width: 820px){
      .grid{ grid-template-columns: 1fr; }
      .grid3{ grid-template-columns: 1fr; }
      .pidGrid{ grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <h1 class="title">Prototype Kit PID</h1>
    <p class="subtitle">Mesures TOF + position/vitesse + mode manuel / PID auto + tuning.</p>

    <div class="stack">
      <!-- TOF + Position/Vitesse -->
      <div class="card">
        <div class="label" style="margin-bottom:12px;">Mesures</div>

        <div class="grid3">
          <div>
            <div class="label">TOF Droite</div>
            <div class="row"><div class="value" id="rightVal">--</div><div class="unit">mm</div></div>
            <div class="pill"><span class="dot" id="rightDot"></span><span id="rightStatus">connexion...</span></div>
          </div>

          <div>
            <div class="label">TOF Gauche</div>
            <div class="row"><div class="value" id="leftVal">--</div><div class="unit">mm</div></div>
            <div class="pill"><span class="dot" id="leftDot"></span><span id="leftStatus">connexion...</span></div>
          </div>

          <div>
            <div class="label">Vitesse balle</div>
            <div class="row"><div class="valueSm" id="velVal">--</div><div class="unit">mm/s</div></div>
            <div class="pill"><span class="dot" id="velDot"></span><span id="velStatus">--</span></div>
          </div>
        </div>

        <div class="hint" style="margin-top:12px;">
          La vitesse est calculée à partir de la position unifiée (mm/s), dérivée filtrée.
        </div>
      </div>

      <!-- Servo -->
      <div class="card" id="servoCard">
        <div class="label" style="margin-bottom:12px;">Servomoteur MG90S</div>

        <div class="switchRow">
          <div class="label">Contrôle manuel (web)</div>
          <label class="switch" title="ON = manuel via slider / OFF = PID automatique">
            <input id="manualEnable" type="checkbox" checked />
            <div class="track"><div class="thumb"></div></div>
          </label>
        </div>

        <div class="servoHead" style="margin-top:10px;">
          <div class="label">Angle</div>
          <div class="row"><div class="servoVal" id="servoVal">90</div><div class="servoUnit">°</div></div>
        </div>

        <input id="servoSlider" type="range" min="0" max="180" value="90" />

        <div class="pill" style="margin-top:14px;">
          <span class="dot" id="servoDot"></span><span id="servoStatus">prêt</span>
        </div>
      </div>

      <!-- PID -->
      <div class="card" id="pidCard">
        <div class="label" style="margin-bottom:8px;">Tuning PID (modifiable uniquement en mode manuel)</div>
        <div class="pidGrid">
          <div class="field">
            <label for="kp">Kp</label>
            <input id="kp" type="number" step="0.01" value="1.00" />
          </div>
          <div class="field">
            <label for="ki">Ki</label>
            <input id="ki" type="number" step="0.0001" value="0.0000" />
          </div>
          <div class="field">
            <label for="kd">Kd</label>
            <input id="kd" type="number" step="0.01" value="0.00" />
          </div>
        </div>
        <div class="hint" style="margin-top:10px;">
          Ici Ki utilise une intégrale en mm*s (I += erreur * dt) et Kd agit sur la vitesse (mm/s).
        </div>
      </div>
    </div>

    <div class="foot">Wi-Fi <b>Kit_Robovaly</b> — ouvre <b>http://192.168.4.1</b>.</div>
  </div>

<script>
  const rightVal = document.getElementById('rightVal');
  const leftVal  = document.getElementById('leftVal');
  const rightStatus = document.getElementById('rightStatus');
  const leftStatus  = document.getElementById('leftStatus');
  const rightDot = document.getElementById('rightDot');
  const leftDot  = document.getElementById('leftDot');

  const velVal = document.getElementById('velVal');
  const velStatus = document.getElementById('velStatus');
  const velDot = document.getElementById('velDot');

  const servoCard   = document.getElementById('servoCard');
  const manualEnable = document.getElementById('manualEnable');
  const servoSlider = document.getElementById('servoSlider');
  const servoVal    = document.getElementById('servoVal');
  const servoStatus = document.getElementById('servoStatus');
  const servoDot    = document.getElementById('servoDot');

  const kpEl = document.getElementById('kp');
  const kiEl = document.getElementById('ki');
  const kdEl = document.getElementById('kd');

  let manualMode = true;

  function setOk(dot, statusEl, ok, msg){
    dot.style.background = ok ? '#2dd4bf' : '#fb7185';
    dot.style.boxShadow = ok ? '0 0 18px rgba(45,212,191,.5)' : '0 0 18px rgba(251,113,133,.45)';
    statusEl.textContent = msg;
  }

  function applyUiState(){
    manualEnable.checked = !!manualMode;

    servoSlider.disabled = !manualMode;
    servoCard.classList.toggle('disabled', !manualMode);
    setOk(servoDot, servoStatus, true, manualMode ? 'manuel' : 'PID auto');

    kpEl.disabled = !manualMode;
    kiEl.disabled = !manualMode;
    kdEl.disabled = !manualMode;
  }

  // Ne pas écraser pendant édition PID
  let pidEditing = false;
  function isPidFieldFocused(){
    const ae = document.activeElement;
    return ae === kpEl || ae === kiEl || ae === kdEl;
  }
  function setPidInputsFromStatus(j){
    const kp = Number(j.kp);
    const ki = Number(j.ki);
    const kd = Number(j.kd);
    if (Number(kpEl.value) !== kp) kpEl.value = kp.toFixed(2);
    if (Number(kiEl.value) !== ki) kiEl.value = ki.toFixed(4);
    if (Number(kdEl.value) !== kd) kdEl.value = kd.toFixed(2);
  }
  [kpEl, kiEl, kdEl].forEach(el => {
    el.addEventListener('focus', () => { pidEditing = true; });
    el.addEventListener('blur',  () => { pidEditing = false; });
  });

  async function tickTOF(){
    try{
      const r = await fetch('/data', { cache: 'no-store' });
      const j = await r.json();

      rightVal.textContent = (j.right_mm >= 0) ? j.right_mm : '--';
      leftVal.textContent  = (j.left_mm  >= 0) ? j.left_mm  : '--';

      setOk(rightDot, rightStatus, j.right_mm >= 0, (j.right_mm >= 0) ? 'OK' : 'Erreur mesure');
      setOk(leftDot,  leftStatus,  j.left_mm  >= 0, (j.left_mm  >= 0) ? 'OK' : 'Erreur mesure');
    }catch(e){
      setOk(rightDot, rightStatus, false, 'Erreur réseau');
      setOk(leftDot,  leftStatus,  false, 'Erreur réseau');
    }
  }
  setInterval(tickTOF, 200);
  tickTOF();

  async function tickStatus(){
    try{
      const r = await fetch('/status', { cache:'no-store' });
      const j = await r.json();

      manualMode = !!j.manual_mode;
      applyUiState();

      const a = parseInt(j.servo_angle, 10);
      servoVal.textContent = a;
      servoSlider.value = a;

      // vitesse
      if (isFinite(j.vel_mms)) {
        velVal.textContent = Number(j.vel_mms).toFixed(1);
        setOk(velDot, velStatus, true, 'OK');
      } else {
        velVal.textContent = '--';
        setOk(velDot, velStatus, false, 'N/A');
      }

      // gains
      if (!manualMode) {
        setPidInputsFromStatus(j);
      } else {
        if (!pidEditing && !isPidFieldFocused()) setPidInputsFromStatus(j);
      }
    }catch(e){}
  }
  setInterval(tickStatus, 200);
  tickStatus();

  // Toggle mode
  let modeTimer = null;
  function sendMode(manual){
    fetch(`/mode?manual=${manual ? 1 : 0}`, { cache:'no-store', keepalive:true }).catch(()=>{});
  }
  manualEnable.addEventListener('change', () => {
    manualMode = manualEnable.checked;
    applyUiState();
    if (modeTimer) clearTimeout(modeTimer);
    modeTimer = setTimeout(() => sendMode(manualMode), 20);
  });

  // Servo manuel
  let lastSentAngle = -1;
  let angleTimer = null;
  function scheduleSendAngle(){
    if (!manualMode) return;
    if (angleTimer) return;
    angleTimer = setTimeout(() => {
      angleTimer = null;
      if (!manualMode) return;
      const a = parseInt(servoSlider.value, 10);
      if (a === lastSentAngle) return;
      lastSentAngle = a;
      fetch(`/servo?angle=${a}`, { cache:'no-store', keepalive:true }).catch(()=>{});
    }, 10);
  }
  servoSlider.addEventListener('input', () => {
    const a = parseInt(servoSlider.value, 10);
    servoVal.textContent = a;
    scheduleSendAngle();
  });

  // PID gains
  let pidTimer = null;
  function sendPid(){
    if (!manualMode) return;
    const kp = parseFloat(kpEl.value || '0');
    const ki = parseFloat(kiEl.value || '0');
    const kd = parseFloat(kdEl.value || '0');
    fetch(`/pid?kp=${encodeURIComponent(kp)}&ki=${encodeURIComponent(ki)}&kd=${encodeURIComponent(kd)}`, {
      cache:'no-store', keepalive:true
    }).catch(()=>{});
  }
  function scheduleSendPid(){
    if (!manualMode) return;
    if (pidTimer) clearTimeout(pidTimer);
    pidTimer = setTimeout(sendPid, 200);
  }
  kpEl.addEventListener('input', scheduleSendPid);
  kiEl.addEventListener('input', scheduleSendPid);
  kdEl.addEventListener('input', scheduleSendPid);

  applyUiState();
</script>
</body>
</html>
)rawliteral";

// ===================== Helpers =====================
static int16_t readMm(Adafruit_VL53L0X &sensor, bool ok) {
  if (!ok) return -1;
  VL53L0X_RangingMeasurementData_t m;
  sensor.rangingTest(&m, false);
  if (m.RangeStatus != 0) return -1;
  if (m.RangeMilliMeter == 0) return -1;
  return (int16_t)m.RangeMilliMeter;
}

// ===================== Servo helpers (LEDC) =====================
static void servoInit() {
  ledcSetup(SERVO_CH, SERVO_FREQ, SERVO_RES_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_CH);
}

static void servoWriteMicroseconds(uint16_t us) {
  const uint32_t maxDuty = (1UL << SERVO_RES_BITS) - 1;
  const uint32_t periodUs = 1000000UL / SERVO_FREQ;
  us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  uint32_t duty = (uint32_t)((uint64_t)us * maxDuty / periodUs);
  ledcWrite(SERVO_CH, duty);
}

static void servoWriteAngle(int deg) {
  deg = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  uint16_t us = map(deg, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  servoWriteMicroseconds(us);
}

// ===================== XSHUT sequencing =====================
static void sensorsAllOff() {
  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  delay(10);
}

// ===================== Measurement selection + unification =====================
enum UsedSide : int8_t { SIDE_NONE = 0, SIDE_LEFT = -1, SIDE_RIGHT = +1 };

static bool pickDistanceAndSide(int16_t left_mm, int16_t right_mm, int16_t &used_raw_mm, UsedSide &side) {
  const bool lValid = (left_mm  > 0);
  const bool rValid = (right_mm > 0);

  if (!lValid && !rValid) {
    side = SIDE_NONE;
    used_raw_mm = -1;
    return false;
  }
  if (lValid && !rValid) {
    side = SIDE_LEFT;
    used_raw_mm = left_mm;
    return true;
  }
  if (!lValid && rValid) {
    side = SIDE_RIGHT;
    used_raw_mm = right_mm;
    return true;
  }

  // 2 valides -> plus petite
  if (left_mm <= right_mm) {
    side = SIDE_LEFT;
    used_raw_mm = left_mm;
  } else {
    side = SIDE_RIGHT;
    used_raw_mm = right_mm;
  }
  return true;
}

static bool computeUnifiedPositionMm(int16_t left_mm, int16_t right_mm, float &pos_mm) {
  int16_t used_raw;
  UsedSide side;
  if (!pickDistanceAndSide(left_mm, right_mm, used_raw, side) || side == SIDE_NONE) return false;

  if (side == SIDE_LEFT) {
    pos_mm = (float)TABLE_LEN_MM - (float)used_raw;
  } else {
    pos_mm = (float)used_raw;
  }
  return true;
}

// ===================== Web routes =====================
static void setupWebRoutes() {
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
  });

  server.on("/data", HTTP_GET, []() {
    int16_t r, l;
    portENTER_CRITICAL(&gMux);
    r = distRight;
    l = distLeft;
    portEXIT_CRITICAL(&gMux);

    String json = "{";
    json += "\"right_mm\":" + String(r) + ",";
    json += "\"left_mm\":"  + String(l);
    json += "}";
    server.send(200, "application/json; charset=utf-8", json);
  });

  server.on("/status", HTTP_GET, []() {
    bool man;
    int a;
    float kp, ki, kd;
    float v;

    portENTER_CRITICAL(&gMux);
    man = manualMode;
    a = servoAngleApplied;
    kp = Kp; ki = Ki; kd = Kd;
    v = ballVelMmS;
    portEXIT_CRITICAL(&gMux);

    String json = "{";
    json += "\"manual_mode\":" + String(man ? "true" : "false") + ",";
    json += "\"servo_angle\":" + String(a) + ",";
    json += "\"kp\":" + String(kp, 6) + ",";
    json += "\"ki\":" + String(ki, 6) + ",";
    json += "\"kd\":" + String(kd, 6) + ",";
    if (isnan(v)) json += "\"vel_mms\":null";
    else          json += "\"vel_mms\":" + String(v, 3);
    json += "}";
    server.send(200, "application/json; charset=utf-8", json);
  });

  server.on("/mode", HTTP_GET, []() {
    if (!server.hasArg("manual")) {
      server.send(400, "text/plain; charset=utf-8", "missing manual");
      return;
    }
    bool man = (server.arg("manual").toInt() != 0);

    portENTER_CRITICAL(&gMux);
    manualMode = man;
    // reset PID quand on change de mode
    pidResetRequest = true;
    portEXIT_CRITICAL(&gMux);

    server.send(204, "text/plain", "");
  });

  server.on("/servo", HTTP_GET, []() {
    if (!server.hasArg("angle")) {
      server.send(400, "text/plain; charset=utf-8", "missing angle");
      return;
    }
    int a = constrain(server.arg("angle").toInt(), SERVO_MIN_DEG, SERVO_MAX_DEG);

    bool man;
    portENTER_CRITICAL(&gMux);
    man = manualMode;
    if (man) servoAngleCmd = a;
    portEXIT_CRITICAL(&gMux);

    server.send(204, "text/plain", "");
  });

  // Reset I/D si gains changent
  server.on("/pid", HTTP_GET, []() {
    if (!server.hasArg("kp") || !server.hasArg("ki") || !server.hasArg("kd")) {
      server.send(400, "text/plain; charset=utf-8", "missing kp/ki/kd");
      return;
    }

    float kp = server.arg("kp").toFloat();
    float ki = server.arg("ki").toFloat();
    float kd = server.arg("kd").toFloat();

    bool man;
    bool changed = false;

    portENTER_CRITICAL(&gMux);
    man = manualMode;
    if (man) {
      if (fabsf(Kp - kp) > 1e-9f) changed = true;
      if (fabsf(Ki - ki) > 1e-9f) changed = true;
      if (fabsf(Kd - kd) > 1e-9f) changed = true;

      Kp = kp; Ki = ki; Kd = kd;
      if (changed) pidResetRequest = true;
    }
    portEXIT_CRITICAL(&gMux);

    server.send(204, "text/plain", "");
  });

  server.onNotFound([]() {
    server.send(404, "text/plain; charset=utf-8", "404 - Not found");
  });

  server.begin();
}

// ===================== FreeRTOS Tasks =====================
static TaskHandle_t taskWebHandle   = nullptr;
static TaskHandle_t taskTofHandle   = nullptr;
static TaskHandle_t taskServoHandle = nullptr;

static const BaseType_t CORE_WEB   = 0;
static const BaseType_t CORE_SENSE = 1;

void TaskWeb(void *pv) {
  WiFi.mode(WIFI_AP);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASS);
  delay(200);

  Serial.printf("SoftAP %s : %s\n", AP_SSID, apOk ? "OK" : "FAIL");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  setupWebRoutes();

  for (;;) {
    server.handleClient();
    vTaskDelay(1);
  }
}

void TaskTOF(void *pv) {
  for (;;) {
    int16_t r = readMm(tofRight, rightOk);
    int16_t l = readMm(tofLeft,  leftOk);

    portENTER_CRITICAL(&gMux);
    distRight = r;
    distLeft  = l;
    portEXIT_CRITICAL(&gMux);

    vTaskDelay(1);
  }
}

void TaskServo(void *pv) {
  int lastApplied = -1;

  // PID state
  float iTerm = 0.0f;        // mm*s
  float velFilt = 0.0f;      // mm/s
  float prevPos = NAN;

  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PID_PERIOD_MS));

    bool man;
    int cmdManual;
    int16_t l, r;
    float kp, ki, kd;
    bool doReset;

    portENTER_CRITICAL(&gMux);
    man = manualMode;
    cmdManual = servoAngleCmd;
    l = distLeft;
    r = distRight;
    kp = Kp; ki = Ki; kd = Kd;
    doReset = pidResetRequest;
    pidResetRequest = false;
    portEXIT_CRITICAL(&gMux);

    if (doReset) {
      iTerm = 0.0f;
      velFilt = 0.0f;
      prevPos = NAN;
    }

    // Estimation position/vitesse (même en manuel pour affichage)
    float pos;
    bool posOk = computeUnifiedPositionMm(l, r, pos);

    float vel = NAN;
    if (posOk) {
      if (!isnan(prevPos)) {
        vel = (pos - prevPos) / PID_DT_S; // mm/s
        velFilt = velFilt + VEL_LP_ALPHA * (vel - velFilt);
      } else {
        velFilt = 0.0f;
      }
      prevPos = pos;

      portENTER_CRITICAL(&gMux);
      ballPosMm  = pos;
      ballVelMmS = velFilt;
      portEXIT_CRITICAL(&gMux);
    } else {
      portENTER_CRITICAL(&gMux);
      ballPosMm  = NAN;
      ballVelMmS = NAN;
      portEXIT_CRITICAL(&gMux);
    }

    int cmd = cmdManual;

    if (!man && posOk) {
      // erreur unifiée
      float error = (float)REF_MM - pos;

      // Derivée d'erreur = -v (puisque error = ref - pos)
      float dError = -velFilt;

      // Calcul commande non saturée
      float u_unsat = (float)SERVO_NEUTRAL_DEG + kp * error + kd * dError + ki * iTerm;

      // Saturation servo
      float u_sat = u_unsat;
      if (u_sat < SERVO_MIN_DEG) u_sat = SERVO_MIN_DEG;
      if (u_sat > SERVO_MAX_DEG) u_sat = SERVO_MAX_DEG;

      // Anti-windup conditionnel:
      // si saturé ET que l'erreur pousse encore dans le même sens, on n'intègre pas
      bool saturatingLow  = (u_unsat < SERVO_MIN_DEG);
      bool saturatingHigh = (u_unsat > SERVO_MAX_DEG);

      bool allowIntegrate = true;
      if (saturatingLow  && error < 0) allowIntegrate = false;
      if (saturatingHigh && error > 0) allowIntegrate = false;

      if (allowIntegrate) {
        iTerm += error * PID_DT_S;              // mm*s
        if (iTerm > ITERM_CLAMP) iTerm = ITERM_CLAMP;
        if (iTerm < -ITERM_CLAMP) iTerm = -ITERM_CLAMP;
      }

      // Recalcul optionnel après intégration (pour cohérence)
      float u = (float)SERVO_NEUTRAL_DEG + kp * error + kd * dError + ki * iTerm;
      int out = (int)lroundf(u);
      out = constrain(out, SERVO_MIN_DEG, SERVO_MAX_DEG);

      cmd = out;

      portENTER_CRITICAL(&gMux);
      servoAngleCmd = cmd; // pour UI
      portEXIT_CRITICAL(&gMux);
    }

    if (cmd != lastApplied) {
      servoWriteAngle(cmd);
      lastApplied = cmd;

      portENTER_CRITICAL(&gMux);
      servoAngleApplied = cmd;
      portEXIT_CRITICAL(&gMux);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);

  Wire.begin(SDA_PIN, SCL_PIN, 100000);

  servoInit();
  servoWriteAngle(SERVO_NEUTRAL_DEG);

  portENTER_CRITICAL(&gMux);
  servoAngleCmd = SERVO_NEUTRAL_DEG;
  servoAngleApplied = SERVO_NEUTRAL_DEG;
  ballPosMm = NAN;
  ballVelMmS = NAN;
  portEXIT_CRITICAL(&gMux);

  sensorsAllOff();

  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  Serial.println("Init TOF Droite...");
  rightOk = tofRight.begin(0x29, false, &Wire);
  if (rightOk) {
    tofRight.setAddress(ADDR_RIGHT);
    Serial.println("OK Droite -> addr 0x30");
  } else {
    Serial.println("ERREUR Droite");
  }

  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  Serial.println("Init TOF Gauche...");
  leftOk = tofLeft.begin(0x29, false, &Wire);
  if (leftOk) {
    tofLeft.setAddress(ADDR_LEFT);
    Serial.println("OK Gauche -> addr 0x31");
  } else {
    Serial.println("ERREUR Gauche");
  }

  xTaskCreatePinnedToCore(TaskWeb,   "TaskWeb",   8192, nullptr, 2, &taskWebHandle,   CORE_WEB);
  xTaskCreatePinnedToCore(TaskTOF,   "TaskTOF",   8192, nullptr, 2, &taskTofHandle,   CORE_SENSE);
  xTaskCreatePinnedToCore(TaskServo, "TaskServo", 4096, nullptr, 3, &taskServoHandle, CORE_SENSE);
}

void loop() {
  vTaskDelay(1000);
}
