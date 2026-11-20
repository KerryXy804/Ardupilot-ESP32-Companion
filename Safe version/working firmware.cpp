#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>
#include <time.h>
#include <Preferences.h>
#include <common/mavlink.h> 
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// --- Pin Mapping (Matek F405-Wing V1 to ESP32) ---
#define RXD2 16      // GPIO 16 (RX2) -> Connect to Matek TX1
#define TXD2 17      // GPIO 17 (TX2) -> Connect to Matek RX1
#define SDA_PIN 21   // I2C Bus 0 SDA (Port Vane)
#define SCL_PIN 22   // I2C Bus 0 SCL
#define SDA1_PIN 25  // I2C Bus 1 SDA (Starboard Vane)
#define SCL1_PIN 26  // I2C Bus 1 SCL
const int PWM_PIN = 34; // Direct Throttle Tap
const int RPM_PIN = 35; // Hobbywing RPM Signal
const int chipSelect = 5; 

// --- Aircraft Constants ---
const int MOTOR_POLE_PAIRS = 21; // Update for your specific motor (14 poles = 7 pairs)
const unsigned long logInterval = 100; // 10 Hz Logging rate

// --- Global Objects ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences prefs;
File aoaFile, throttleFile;

// --- Instrumentation State ---
char aoaFileName[64], throttleFileName[64];
bool sdAvailable = false, filesCreated = false, calibrationDone = false;
float zeroOffset1 = 0, zeroOffset2 = 0;
volatile unsigned long pwmDuration = 0, pwmStartTime = 0;
volatile unsigned long rpmPulseCount = 0;
unsigned long lastLogTime = 0, lastFlushTime = 0, lastRpmCalcMillis = 0;
float currentRpm = 0;

// --- MAVLink & Sync State ---
uint64_t fcBootUs = 0; 
bool gpsTimeValid = false;
uint32_t lastMavMsgMillis = 0;
uint32_t mavPacketCount = 0;
uint8_t lastMsgId = 0;

// --- Dashboard HTML ---
const char *HTML_PAGE = R"RAW(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RPI DBF Flight Hub</title>
    <style>
        body { font-family: -apple-system, sans-serif; background: #f0f2f5; padding: 20px; display: flex; flex-direction: column; align-items: center; }
        .card { background: white; border-radius: 12px; padding: 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.08); text-align: center; width: 100%; max-width: 450px; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }
        .stat-label { font-size: 11px; color: #666; text-transform: uppercase; font-weight: bold; }
        .stat-value { font-size: 18px; font-weight: bold; margin-top: 5px; }
        .data-value { font-size: 32px; color: #ad2713; font-weight: 300; }
        .status-pill { padding: 4px 12px; border-radius: 20px; font-size: 11px; font-weight: bold; color: white; display: inline-block; margin-top: 5px; }
        .bg-green { background: #28a745; } .bg-red { background: #dc3545; } .bg-orange { background: #fd7e14; }
        button { background: #ad2713; color: white; border: none; padding: 15px; border-radius: 8px; cursor: pointer; font-weight: bold; width: 100%; margin-top: 10px; }
        button:disabled { background: #ccc; }
        .debug-box { font-family: monospace; font-size: 10px; color: #666; margin-top: 10px; background: #f8f9fa; padding: 8px; border-radius: 4px; text-align: left; }
        .override { font-size: 11px; color: #ad2713; text-decoration: underline; cursor: pointer; margin-top: 12px; display: block; }
    </style>
</head>
<body>
    <div class="card">
        <h2>RPI DBF Instrumentation</h2>
        <div class="grid">
            <div><div class="stat-label">MAVLink (FC)</div><div id="mav-pill" class="status-pill bg-red">OFFLINE</div></div>
            <div><div class="stat-label">Throttle (RX)</div><div id="thr-pill" class="status-pill bg-red">NO SIGNAL</div></div>
        </div>
        <div class="grid" style="margin-top:15px; border-top: 1px solid #eee; padding-top:10px;">
            <div><div class="stat-label">Motor RPM</div><div class="stat-value" id="rpm">0</div></div>
            <div><div class="stat-label">Throttle %</div><div class="stat-value" id="thr-pct">0%</div></div>
        </div>
    </div>
    <div class="card">
        <div class="grid">
            <div><div class="stat-label">Port AoA</div><div class="data-value" id="a1">--.-°</div></div>
            <div><div class="stat-label">Stbd AoA</div><div class="data-value" id="a2">--.-°</div></div>
        </div>
        <button id="start-btn" disabled onclick="ws.send('zero')">WAITING FOR GPS SYNC...</button>
        <div class="debug-box">
            Packets: <span id="pkts">0</span> | Last ID: <span id="last-id">--</span><br>
            TimeUS: <span id="time-us">0</span><br>
            File: <span id="log-status">Ready.</span>
        </div>
        <a id="force" class="override" onclick="ws.send('force_start')" style="display:none;">No GPS? Force Start Logging</a>
    </div>
    <script>
        let ws = new WebSocket(`ws://${window.location.host}/ws`);
        ws.onmessage = (e) => {
            const d = JSON.parse(e.data);
            document.getElementById('rpm').textContent = Math.round(d.rpm);
            document.getElementById('thr-pct').textContent = d.thr + "%";
            document.getElementById('a1').textContent = d.a1.toFixed(1) + "°";
            document.getElementById('a2').textContent = d.a2.toFixed(1) + "°";
            document.getElementById('pkts').textContent = d.pkt_count;
            document.getElementById('last-id').textContent = d.last_id;
            document.getElementById('time-us').textContent = d.fc_boot_us;
            document.getElementById('log-status').textContent = d.logging ? d.log_file : "Ready.";
            
            const mavPill = document.getElementById('mav-pill');
            const thrPill = document.getElementById('thr-pill');
            const btn = document.getElementById('start-btn');
            const force = document.getElementById('force');

            if (!d.mav_active) { 
                mavPill.className="status-pill bg-red"; mavPill.textContent="OFFLINE"; 
                btn.disabled=true; force.style.display="none";
            } else if (!d.gps_synced) { 
                mavPill.className="status-pill bg-orange"; mavPill.textContent="NO GPS"; 
                btn.disabled=true; force.style.display=d.logging?"none":"block";
            } else { 
                mavPill.className="status-pill bg-green"; mavPill.textContent="SYNCED"; 
                btn.disabled=d.logging; btn.textContent=d.logging?"LOGGING LIVE":"ZERO & START";
                force.style.display="none";
            }

            if (d.pwm < 800) { thrPill.className="status-pill bg-red"; thrPill.textContent="NO SIGNAL"; }
            else { thrPill.className="status-pill bg-green"; thrPill.textContent="PULSE OK"; }
        };
    </script>
</body>
</html>
)RAW";

// --- ISRs ---
void IRAM_ATTR handlePWM() {
    if (digitalRead(PWM_PIN) == HIGH) pwmStartTime = micros();
    else {
        unsigned long dur = micros() - pwmStartTime;
        if (dur > 800 && dur < 2200) pwmDuration = dur;
    }
}
void IRAM_ATTR handleRpmPulse() { rpmPulseCount++; }

// --- Sensor Logic ---
float getRawAngle(TwoWire &bus) {
    bus.beginTransmission(0x36); bus.write(0x0E);
    if(bus.endTransmission() != 0) return NAN;
    bus.requestFrom(0x36, 2);
    if (bus.available() == 2) return (((bus.read() & 0x0F) << 8) | bus.read()) * 360.0 / 4096.0;
    return NAN;
}

void createFiles(bool forced) {
    char datePrefix[32];
    if (forced) sprintf(datePrefix, "FORCED_START");
    else {
        struct tm ti; getLocalTime(&ti);
        strftime(datePrefix, sizeof(datePrefix), "%Y%m%d_%H%M", &ti);
    }
    for (int i = 1; i < 1000; i++) {
        sprintf(aoaFileName, "/%s_F%03d_AOA.CSV", datePrefix, i);
        sprintf(throttleFileName, "/%s_F%03d_THR.CSV", datePrefix, i);
        if (!SD.exists(aoaFileName)) {
            aoaFile = SD.open(aoaFileName, FILE_WRITE);
            throttleFile = SD.open(throttleFileName, FILE_WRITE);
            if(aoaFile && throttleFile) {
                aoaFile.println("utc_time,fc_boot_us,esp_millis,angle1_deg,angle2_deg,motor_rpm");
                throttleFile.println("utc_time,fc_boot_us,esp_millis,pwm_us,throttle_pct");
                filesCreated = true; return;
            }
        }
    }
}

void processMavlink() {
    mavlink_message_t msg; mavlink_status_t status;
    while (Serial2.available() > 0) {
        uint8_t c = Serial2.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            mavPacketCount++; lastMsgId = msg.msgid; lastMavMsgMillis = millis();
            if (msg.msgid == MAVLINK_MSG_ID_SYSTEM_TIME) {
                mavlink_system_time_t st; mavlink_msg_system_time_decode(&msg, &st);
                fcBootUs = (uint64_t)st.time_boot_ms * 1000ULL; 
                if (st.time_unix_usec > 0) {
                    struct timeval tv = { .tv_sec = (time_t)(st.time_unix_usec / 1000000ULL) };
                    settimeofday(&tv, NULL);
                    gpsTimeValid = true;
                }
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2); // Connect to Matek UART1
    Wire.begin(SDA_PIN, SCL_PIN); Wire1.begin(SDA1_PIN, SCL1_PIN);
    
    pinMode(PWM_PIN, INPUT); attachInterrupt(digitalPinToInterrupt(PWM_PIN), handlePWM, CHANGE);
    pinMode(RPM_PIN, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(RPM_PIN), handleRpmPulse, RISING);

    SPI.begin(18, 19, 23, 5);
    if (SD.begin(chipSelect)) sdAvailable = true;

    prefs.begin("offsets", false);
    zeroOffset1 = prefs.getFloat("v1", 0); zeroOffset2 = prefs.getFloat("v2", 0);
    if (zeroOffset1 != 0) calibrationDone = true;
    prefs.end();

    WiFi.softAP("RPI_DBF_HUB", "CHANGE_ME"); // set your own password (8+ chars) before flashing
    ws.onEvent([](AsyncWebSocket *s, AsyncWebSocketClient *c, AwsEventType t, void *arg, uint8_t *data, size_t len) {
        if (t == WS_EVT_DATA) {
            if (strncmp((char*)data, "zero", len) == 0) {
                zeroOffset1 = getRawAngle(Wire); zeroOffset2 = getRawAngle(Wire1);
                prefs.begin("offsets", false); prefs.putFloat("v1", zeroOffset1); prefs.putFloat("v2", zeroOffset2); prefs.end();
                calibrationDone = true; if (!filesCreated) createFiles(false);
            } else if (strncmp((char*)data, "force_start", len) == 0) {
                zeroOffset1 = getRawAngle(Wire); zeroOffset2 = getRawAngle(Wire1);
                calibrationDone = true; if (!filesCreated) createFiles(true);
            }
        }
    });
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send(200, "text/html", HTML_PAGE); });
    server.begin();
}

void loop() {
    processMavlink();
    unsigned long now = millis();
    ws.cleanupClients();
    
    if (now - lastLogTime >= logInterval) {
        unsigned long dur = now - lastRpmCalcMillis;
        noInterrupts(); unsigned long pulses = rpmPulseCount; rpmPulseCount = 0; interrupts();
        currentRpm = ((float)pulses / (dur / 1000.0)) * 60.0 / MOTOR_POLE_PAIRS;
        lastRpmCalcMillis = now; lastLogTime = now;

        struct tm ti; char ts[25]; getLocalTime(&ti); strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &ti);
        noInterrupts(); unsigned long pwm = pwmDuration; interrupts();
        long thrPct = (pwm > 800) ? map(constrain(pwm, 1000, 2000), 1000, 2000, 0, 100) : 0;
        
        float a1 = (getRawAngle(Wire) - zeroOffset1);
        float a2 = (zeroOffset2 - getRawAngle(Wire1));

        if (filesCreated) {
            aoaFile.printf("%s,%llu,%lu,%.2f,%.2f,%.0f\n", ts, fcBootUs, now, a1, a2, currentRpm);
            throttleFile.printf("%s,%llu,%lu,%lu,%ld\n", ts, fcBootUs, now, pwm, thrPct);
        }

        String json = "{\"rpm\":"+String(currentRpm)+",\"thr\":"+String(thrPct)+",\"pwm\":"+String(pwm)+",\"a1\":"+String(a1)+",\"a2\":"+String(a2)+",\"pkt_count\":"+String(mavPacketCount)+",\"last_id\":"+String(lastMsgId)+",\"fc_boot_us\":"+String(fcBootUs)+",\"logging\":"+String(filesCreated?"true":"false")+",\"log_file\":\""+String(aoaFileName)+"\",\"mav_active\":"+String(now - lastMavMsgMillis < 3000?"true":"false")+",\"gps_synced\":"+String(gpsTimeValid?"true":"false")+"}";
        ws.textAll(json);
    }
    
    if (filesCreated && (now - lastFlushTime >= 1000)) {
        aoaFile.flush(); throttleFile.flush(); lastFlushTime = now;
    }
}