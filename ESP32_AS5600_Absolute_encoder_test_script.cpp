// This is a simple ESP32 sketch to read AS5600 encoder data and serve it via a web interface.
// It uses the Wire library for I2C communication and the WebServer library for HTTP serving.

// This code will be expanded on when the Userside Tele OP arm and encoders are verified and working well(Currently going through multiple iterations of CAD to get the bearing position correct)
// Exciting 


#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

// ===================== USER SETTINGS =====================
// If you leave SSID empty, it will start an AP automatically.
static const char* WIFI_SSID     = "YOUR_WIFI_SSID";
static const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Fallback AP (or primary if WIFI_SSID is empty)
static const char* AP_SSID = "AS5600-Test";
static const char* AP_PASS = "as5600test";

// Correct default I2C pins for ESP32 DevKit (WROOM-32)
static const int I2C_SDA = 21;   // D21
static const int I2C_SCL = 22;   // D22

static const uint8_t AS5600_ADDR = 0x36;
// =========================================================

WebServer server(80);

// Shared state
static volatile uint16_t g_raw = 0;     // 0..4095
static volatile uint8_t  g_status = 0;  // STATUS reg
static volatile uint8_t  g_agc = 0;     // AGC reg
static volatile uint16_t g_mag = 0;     // MAGNITUDE 0..4095
static volatile int32_t  g_turns = 0;   // multi-turn via wrap detect

static volatile bool     g_zeroSet = false;
static volatile int32_t  g_zeroTurns = 0;
static volatile uint16_t g_zeroRaw = 0;

static inline float countsToDeg(uint16_t c) {
  return (float)c * 360.0f / 4096.0f;
}

static bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t* out, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start
  size_t got = Wire.requestFrom((int)addr, (int)len);
  if (got != len) return false;
  for (size_t i = 0; i < len; i++) out[i] = Wire.read();
  return true;
}

static bool i2cReadU8(uint8_t addr, uint8_t reg, uint8_t &val) {
  uint8_t b = 0;
  if (!i2cReadBytes(addr, reg, &b, 1)) return false;
  val = b;
  return true;
}

static bool i2cReadU16(uint8_t addr, uint8_t regMSB, uint16_t &val) {
  uint8_t b[2] = {0,0};
  if (!i2cReadBytes(addr, regMSB, b, 2)) return false;
  val = ((uint16_t)b[0] << 8) | b[1];
  return true;
}

static bool as5600ReadAll(uint16_t &raw, uint8_t &status, uint8_t &agc, uint16_t &mag) {
  uint16_t raw16 = 0, mag16 = 0;
  uint8_t st = 0, a = 0;

  // RAW_ANGLE: 0x0C/0x0D
  if (!i2cReadU16(AS5600_ADDR, 0x0C, raw16)) return false;
  raw16 &= 0x0FFF;

  // STATUS: 0x0B
  if (!i2cReadU8(AS5600_ADDR, 0x0B, st)) return false;

  // AGC: 0x1A
  if (!i2cReadU8(AS5600_ADDR, 0x1A, a)) return false;

  // MAGNITUDE: 0x1B/0x1C
  if (!i2cReadU16(AS5600_ADDR, 0x1B, mag16)) return false;
  mag16 &= 0x0FFF;

  raw = raw16; status = st; agc = a; mag = mag16;
  return true;
}

// Wrap detect for multi-turn
static void updateTurns(uint16_t prevRaw, uint16_t newRaw) {
  const uint16_t LOW  = 1024;
  const uint16_t HIGH = 3072;
  if (prevRaw > HIGH && newRaw < LOW) g_turns++;
  else if (prevRaw < LOW && newRaw > HIGH) g_turns--;
}

// --------------- Minimal Web UI ---------------
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>AS5600 Tester</title>
<style>
body{font-family:Arial;margin:16px}
.card{border:1px solid #ddd;border-radius:10px;padding:14px;display:inline-block}
.kv{display:grid;grid-template-columns:160px 1fr;gap:6px 10px;min-width:320px}
button{padding:10px 12px;border-radius:8px;border:1px solid #ccc;background:#fff;cursor:pointer}
button:hover{background:#f7f7f7}
.muted{color:#666;font-size:.92em}
</style></head><body>
<h2>ESP32 (WROOM-32) + AS5600 Tester</h2>
<p class="muted">Open <code>/api</code> for JSON. Use buttons to set/clear software zero.</p>
<div class="card">
<canvas id="dial" width="320" height="320"></canvas>
<div style="margin-top:10px;display:flex;gap:10px">
<button onclick="fetch('/zero')">Set Zero</button>
<button onclick="fetch('/clearzero')">Clear Zero</button>
</div>
</div>
<div class="card" style="vertical-align:top;margin-left:12px">
<div class="kv">
<div>RAW (0..4095)</div><div id="raw">-</div>
<div>Angle (deg)</div><div id="deg">-</div>
<div>Turns</div><div id="turns">-</div>
<div>Total deg</div><div id="tdeg">-</div>
<div>Rel deg</div><div id="rdeg">-</div>
<div>Status</div><div id="st">-</div>
<div>Magnet flags</div><div id="flags">-</div>
<div>AGC</div><div id="agc">-</div>
<div>Magnitude</div><div id="mag">-</div>
<div>RSSI</div><div id="rssi">-</div>
</div>
<p class="muted" style="margin-top:10px">
Flags: MD=magnet detected, ML=too weak, MH=too strong.
</p>
</div>
<script>
const dial=document.getElementById('dial'),ctx=dial.getContext('2d');
function draw(deg){
  const w=dial.width,h=dial.height,cx=w/2,cy=h/2,r=Math.min(w,h)*0.42;
  ctx.clearRect(0,0,w,h);
  ctx.beginPath();ctx.arc(cx,cy,r,0,Math.PI*2);ctx.strokeStyle='#333';ctx.lineWidth=2;ctx.stroke();
  for(let a=0;a<360;a+=30){
    const rad=(a-90)*Math.PI/180;
    const x1=cx+Math.cos(rad)*r*0.88,y1=cy+Math.sin(rad)*r*0.88;
    const x2=cx+Math.cos(rad)*r*0.98,y2=cy+Math.sin(rad)*r*0.98;
    ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x2,y2);ctx.strokeStyle='#555';ctx.lineWidth=2;ctx.stroke();
  }
  const nrad=(deg-90)*Math.PI/180;
  const nx=cx+Math.cos(nrad)*r*0.85,ny=cy+Math.sin(nrad)*r*0.85;
  ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(nx,ny);ctx.strokeStyle='#d22';ctx.lineWidth=4;ctx.stroke();
  ctx.beginPath();ctx.arc(cx,cy,6,0,Math.PI*2);ctx.fillStyle='#111';ctx.fill();
  ctx.fillStyle='#111';ctx.font='16px Arial';ctx.textAlign='center';
  ctx.fillText(`Angle: ${deg.toFixed(1)}°`,cx,cy+r+28);
}
function set(id,v){document.getElementById(id).textContent=v;}
async function poll(){
  try{
    const j=await (await fetch('/api',{cache:'no-store'})).json();
    set('raw',j.raw);
    set('deg',j.deg.toFixed(2));
    set('turns',j.turns);
    set('tdeg',j.total_deg.toFixed(2));
    set('rdeg',j.rel_deg.toFixed(2));
    set('st','0x'+j.status.toString(16).padStart(2,'0'));
    set('flags',`MD=${j.md} ML=${j.ml} MH=${j.mh}`);
    set('agc',j.agc);
    set('mag',j.mag);
    set('rssi',j.rssi+' dBm');
    draw(j.deg);
  }catch(e){ draw(0); }
}
setInterval(poll,100);
poll();
</script></body></html>
)HTML";
// --------------------------------------------

static void handleIndex() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

static void handleZero() {
  g_zeroSet = true;
  g_zeroTurns = g_turns;
  g_zeroRaw = g_raw;
  server.send(200, "text/plain", "OK");
}

static void handleClearZero() {
  g_zeroSet = false;
  server.send(200, "text/plain", "OK");
}

static void handleApi() {
  uint8_t st = g_status;
  int mh = (st >> 3) & 0x01;
  int ml = (st >> 4) & 0x01;
  int md = (st >> 5) & 0x01;

  int32_t totalCounts = (int32_t)g_turns * 4096 + (int32_t)g_raw;
  float totalDeg = (float)totalCounts * 360.0f / 4096.0f;

  int32_t relCounts = totalCounts;
  if (g_zeroSet) {
    int32_t zeroCounts = (int32_t)g_zeroTurns * 4096 + (int32_t)g_zeroRaw;
    relCounts = totalCounts - zeroCounts;
  }
  float relDeg = (float)relCounts * 360.0f / 4096.0f;

  float deg = countsToDeg(g_raw);
  int rssi = WiFi.isConnected() ? WiFi.RSSI() : 0;

  String json = "{";
  json += "\"raw\":" + String(g_raw) + ",";
  json += "\"deg\":" + String(deg, 4) + ",";
  json += "\"turns\":" + String(g_turns) + ",";
  json += "\"total_deg\":" + String(totalDeg, 4) + ",";
  json += "\"rel_deg\":" + String(relDeg, 4) + ",";
  json += "\"status\":" + String((int)st) + ",";
  json += "\"mh\":" + String(mh) + ",";
  json += "\"ml\":" + String(ml) + ",";
  json += "\"md\":" + String(md) + ",";
  json += "\"agc\":" + String((int)g_agc) + ",";
  json += "\"mag\":" + String(g_mag) + ",";
  json += "\"rssi\":" + String(rssi);
  json += "}";

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

static void startWiFi() {
  if (WIFI_SSID && WIFI_SSID[0] != '\0') {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("WiFi connecting");
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < 12000) {
      delay(250);     // yields to avoid WDT
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("WiFi connected. IP: ");
      Serial.println(WiFi.localIP());
      return;
    }
    Serial.println("WiFi failed; starting AP...");
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP SSID: "); Serial.println(AP_SSID);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
}

// Encoder sampling task (prevents WDT by yielding)
void encoderTask(void* pv) {
  (void)pv;
  uint16_t prevRaw = 0;
  bool havePrev = false;

  for (;;) {
    uint16_t raw = 0, mag = 0;
    uint8_t st = 0, agc = 0;

    if (as5600ReadAll(raw, st, agc, mag)) {
      if (havePrev) updateTurns(prevRaw, raw);
      prevRaw = raw;
      havePrev = true;

      g_raw = raw;
      g_status = st;
      g_agc = agc;
      g_mag = mag;
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz, always yields
  }
}

static void i2cScan() {
  Serial.println("I2C scan...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("  Found 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
    delay(2);
  }
  if (!found) Serial.println("  No I2C devices found.");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("Booting AS5600 tester on ESP32 (WROOM-32).");
  Serial.println("Note: 'PSRAM: NOT FOUND' is normal on this board.");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // Basic presence check for AS5600 at 0x36
  Wire.beginTransmission(AS5600_ADDR);
  uint8_t err = Wire.endTransmission();
  if (err == 0) Serial.println("AS5600 detected on I2C address 0x36.");
  else {
    Serial.println("AS5600 not detected at 0x36. Check SDA=GPIO21, SCL=GPIO22, VCC=3V3, GND.");
    i2cScan();
  }

  startWiFi();

  server.on("/", handleIndex);
  server.on("/api", handleApi);
  server.on("/zero", handleZero);
  server.on("/clearzero", handleClearZero);
  server.begin();
  Serial.println("HTTP server started.");

  // Start sampling task on core 1
  xTaskCreatePinnedToCore(encoderTask, "enc", 4096, nullptr, 1, nullptr, 1);

  Serial.println("Open the printed IP in your browser (or AP IP if using fallback).");
}

void loop() {
  server.handleClient();
  delay(2); // yields; prevents WDT in main loop
}
