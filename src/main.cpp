#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <common/mavlink.h>

// Target: ESP32-WROOM-32 (PlatformIO board "esp32dev")

// =============================================================================
#define USE_AOA_SENSORS 1      // 1 = Active, 0 = Bypassed
#define LOG_INTERVAL_MS 20     // 50Hz (20ms window)
#define I2C_SPEED 100000       // 100kHz Standard Mode for Signal Integrity
#define FC_BAUD 57600
#define FC_RX_BUFFER 1024      // Absorbs MAVLink bytes while an SD flush blocks loop()
#define PWM_TIMEOUT_US 100000  // No valid throttle pulse for 100ms -> report 0
#define RPM_TIMEOUT_US 250000  // No RPM pulse for 250ms -> report 0
#define RPM_MIN_GAP_US 50      // Reject glitches closer together than this
// =============================================================================

// --- Pins ---
#define RXD2 16
#define TXD2 17
#define SDA_PIN 21
#define SCL_PIN 22
#define SDA1_PIN 25
#define SCL1_PIN 26
const int LED_PIN = 2;
const int PWM_PIN = 34;   // Input-only pin: throttle tap is actively driven by the FC
const int RPM_PIN = 35;   // Input-only pin, NO internal pull-up: fit an external 10k pull-up to 3.3V
const int chipSelect = 5;
const int MOTOR_POLE_PAIRS = 21; // TODO: confirm against motor spec / tachometer (14 poles = 7 pairs)

// --- AS5600 ---
#define AS5600_ADDR       0x36
#define AS5600_REG_STATUS 0x0B
#define AS5600_REG_ANGLE  0x0E

// --- Global Objects ---
File aoaFile, thrFile;

// --- State ---
float currentA1 = -999, currentA2 = -999, currentRpm = 0;
long currentThrPct = 0;
bool filesCreated = false;
bool fcTimeSeen = false;
int blinkCounter = 0;
uint64_t fcBootUs = 0;

char aoaName[32], thrName[32];
unsigned long lastLogTime = 0, lastFlushTime = 0;
unsigned long rpmRefUs = 0;      // timestamp of the last pulse used in the previous RPM calc
bool rpmRefValid = false;

volatile unsigned long pwmDuration = 0, pwmStartTime = 0, lastPwmValidUs = 0;
volatile unsigned long rpmPulseCount = 0, lastRpmPulseUs = 0;

// --- ISRs (Interrupts) ---
void IRAM_ATTR handlePWM() {
    unsigned long t = micros();
    if (digitalRead(PWM_PIN) == HIGH) pwmStartTime = t;
    else {
        unsigned long dur = t - pwmStartTime;
        if (dur > 800 && dur < 2200) {
            pwmDuration = dur;
            lastPwmValidUs = t;
        }
    }
}

void IRAM_ATTR handleRpmPulse() {
    unsigned long t = micros();
    if (t - lastRpmPulseUs < RPM_MIN_GAP_US) return;
    lastRpmPulseUs = t;
    rpmPulseCount++;
}

// --- I2C Read (Standard 100kHz) ---
// Sets the AS5600 register pointer, then reads n bytes with a repeated start.
bool readAS5600(TwoWire &bus, uint8_t reg, uint8_t *buf, uint8_t n) {
    bus.beginTransmission(AS5600_ADDR);
    bus.write(reg);
    if (bus.endTransmission(false) != 0) return false;
    if (bus.requestFrom((uint16_t)AS5600_ADDR, n) != n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = bus.read();
    return true;
}

float getRawAngle(TwoWire &bus) {
    uint8_t b[2];
    if (!readAS5600(bus, AS5600_REG_ANGLE, b, 2)) return NAN;
    return (((b[0] & 0x0F) << 8) | b[1]) * 360.0f / 4096.0f;
}

void reportAS5600(TwoWire &bus, const char *label) {
    uint8_t st;
    if (!readAS5600(bus, AS5600_REG_STATUS, &st, 1)) {
        Serial.printf("[AS5600 %s] NOT RESPONDING\n", label);
        return;
    }
    const char *mag = (st & 0x20) ? ((st & 0x10) ? "too weak" : (st & 0x08) ? "too strong" : "OK")
                                  : "NOT DETECTED";
    Serial.printf("[AS5600 %s] magnet %s, angle %.2f deg\n", label, mag, getRawAngle(bus));
}

void createFiles() {
    for (int i = 1; i < 1000; i++) {
        sprintf(thrName, "/T%03d.CSV", i);
        sprintf(aoaName, "/A%03d.CSV", i);
        if (SD.exists(thrName)) continue;
        #if USE_AOA_SENSORS
            if (SD.exists(aoaName)) continue;
        #endif
        thrFile = SD.open(thrName, FILE_WRITE);
        if (!thrFile) {
            Serial.printf("[SD] Could not create %s\n", thrName);
            return;
        }
        thrFile.println("esp_ms,fc_boot_us,pwm_us,thr_pct,rpm");
        Serial.printf("[SD] Logging throttle/RPM to %s\n", thrName);
        #if USE_AOA_SENSORS
            aoaFile = SD.open(aoaName, FILE_WRITE);
            if (aoaFile) {
                aoaFile.println("esp_ms,fc_boot_us,a1,a2");
                Serial.printf("[SD] Logging angles to %s\n", aoaName);
            } else {
                Serial.printf("[SD] WARNING: could not create %s\n", aoaName);
            }
        #endif
        filesCreated = true;
        return;
    }
    Serial.println("[SD] No free file name (T001-T999 all used)");
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    delay(200);
    Serial.println("\n=== AoA LOGGER BOOT ===");

    pinMode(RXD2, INPUT_PULLUP);
    Serial2.setRxBufferSize(FC_RX_BUFFER);   // must precede begin()
    Serial2.begin(FC_BAUD, SERIAL_8N1, RXD2, TXD2);

    pinMode(PWM_PIN, INPUT); attachInterrupt(digitalPinToInterrupt(PWM_PIN), handlePWM, CHANGE);
    pinMode(RPM_PIN, INPUT); attachInterrupt(digitalPinToInterrupt(RPM_PIN), handleRpmPulse, RISING);

    #if USE_AOA_SENSORS
    // 100kHz is robust against motor noise and longer wire runs
    Wire.begin(SDA_PIN, SCL_PIN, I2C_SPEED); Wire.setTimeOut(10);
    Wire1.begin(SDA1_PIN, SCL1_PIN, I2C_SPEED); Wire1.setTimeOut(10);
    reportAS5600(Wire,  "port / bus 0");
    reportAS5600(Wire1, "stbd / bus 1");
    #endif

    SPI.begin(18, 19, 23, 5);
    if (SD.begin(chipSelect)) {
        createFiles();
    } else {
        Serial.println("[SD] Card mount FAILED");
    }

    if (filesCreated) {
        Serial.println("=== LOGGING (LED blinks 5 Hz) ===");
    } else {
        digitalWrite(LED_PIN, HIGH);   // solid LED = not logging
        Serial.println("=== NOT LOGGING (LED solid) ===");
    }
}

void loop() {
    // 1. MAVLink (Frequent small reads to clear buffer)
    mavlink_message_t msg; mavlink_status_t status;
    int bRead = 0;
    while (Serial2.available() > 0 && bRead < 128) {
        bRead++;
        uint8_t c = Serial2.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            if (msg.msgid == MAVLINK_MSG_ID_SYSTEM_TIME) {
                mavlink_system_time_t st; mavlink_msg_system_time_decode(&msg, &st);
                fcBootUs = (uint64_t)st.time_boot_ms * 1000ULL;
                if (!fcTimeSeen) {
                    fcTimeSeen = true;
                    Serial.printf("[MAVLink] First SYSTEM_TIME: FC boot %lu ms\n", (unsigned long)st.time_boot_ms);
                }
            }
        }
    }

    unsigned long now = millis();

    // 2. 50Hz LOGGING TASK (20ms)
    if (now - lastLogTime >= LOG_INTERVAL_MS) {
        lastLogTime = now;

        // RPM: pulses between the last edge of the previous frame and the last edge of
        // this frame, divided by the exact time between those edges (no window quantization).
        noInterrupts();
        unsigned long pulses = rpmPulseCount; rpmPulseCount = 0;
        unsigned long lastEdge = lastRpmPulseUs;
        unsigned long pwm = pwmDuration;
        unsigned long lastPwm = lastPwmValidUs;
        interrupts();
        unsigned long nowUs = micros();

        if (pulses > 0) {
            if (rpmRefValid) {
                unsigned long span = lastEdge - rpmRefUs;
                if (span > 0) currentRpm = (pulses * 60000000.0f / span) / MOTOR_POLE_PAIRS;
            }
            rpmRefUs = lastEdge;
            rpmRefValid = true;
        } else if (nowUs - lastEdge > RPM_TIMEOUT_US) {
            currentRpm = 0;
            rpmRefValid = false;
        }

        // Throttle: drop to 0 if the PWM signal has gone away
        if (nowUs - lastPwm > PWM_TIMEOUT_US) pwm = 0;
        currentThrPct = (pwm > 800) ? map(constrain(pwm, 1000, 2000), 1000, 2000, 0, 100) : 0;

        #if USE_AOA_SENSORS
            float r1 = getRawAngle(Wire);
            float r2 = getRawAngle(Wire1);
            currentA1 = isnan(r1) ? -999 : r1;
            currentA2 = isnan(r2) ? -999 : r2;
        #endif

        if (filesCreated) {
            thrFile.printf("%lu,%llu,%lu,%ld,%.0f\n", now, fcBootUs, pwm, currentThrPct, currentRpm);
            #if USE_AOA_SENSORS
                if (aoaFile) aoaFile.printf("%lu,%llu,%.2f,%.2f\n", now, fcBootUs, currentA1, currentA2);
            #endif

            // LED Activity (Blink at 5Hz)
            blinkCounter++;
            if (blinkCounter >= 5) {
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                blinkCounter = 0;
            }
        }
    }

    // Flush every 1s (2s is risky at 50Hz due to buffer size)
    if (filesCreated && (now - lastFlushTime >= 1000)) {
        thrFile.flush();
        #if USE_AOA_SENSORS
            if (aoaFile) aoaFile.flush();
        #endif
        lastFlushTime = now;
    }
}
