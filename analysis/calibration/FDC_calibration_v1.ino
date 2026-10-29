// KYNETIX - FDC1004 capacitance readout / weight CALIBRATION (one sensor, no GCT)
// Based on the team's GCT_v1.6: same wiring, same FDC1004 setup / CAPDAC tuning / baseline code.
// Removed: step detection and GCT. Added: one-sensor streaming in the exact format
// callibration_code.ipynb expects, so record(100), record(200)... work unchanged.
//
// WIRING (unchanged from GCT_v1.6):
//   ESP32 3V3 -> breakout 3.3V (breakout VCC disconnected), ESP32 GND -> breakout GND
//   GPIO21 -> SDA, GPIO22 -> SCL
//   CIN1 -> sensor 1 upper copper plate, CIN2 -> sensor 2 upper copper plate
//   Both lower copper plates -> GND.
//   Only the sensor chosen in SENSOR is measured; the other can stay plugged in.
//
// THREE OUTPUT MODES, same as our old RC sketch (pick one in SETTINGS), 115200 baud:
//   labeled (both false) -> Tools > Serial Monitor, 4 lines per second
//   PLOT_MODE = true     -> Tools > Serial Plotter, live graph, 20 points per second
//   CSV_MODE  = true     -> 100 rows per second, 6 columns, for callibration_code.ipynb:
//                           t_s, raw, delta_raw, cap_pF, delta_pF, timeouts
//   (the notebook calls columns 2-3 "ticks" / "delta_ticks"; here they are FDC1004
//    counts: 524288 counts = 1 pF. delta_pF is the column that matters.)
// Commands: b + Enter = new baseline (sensor UNLOADED). The notebook's rezero() sends this.

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "esp_timer.h"

// =========================== SETTINGS ===========================
const uint8_t SENSOR    = 1;      // which pad to calibrate: 1 = CIN1, 2 = CIN2
const bool    PLOT_MODE  = false; // true = live graph in Tools > Serial Plotter
const float   PLOT_RANGE = 0.2;   // graph shows -0.2 to +0.2 pF (raise for big presses)
const bool    CSV_MODE   = true;  // true = CSV rows for the notebook (only used when PLOT_MODE = false)
// Chip runs at 400 readings/s; each printed line averages AVG_N of them:
const int     AVG_N = PLOT_MODE ? 20      // graph:   20 points per second
                    : CSV_MODE  ? 4       // CSV:     100 rows per second
                    :             100;    // labeled: 4 lines per second
// ================================================================

const int SDA_PIN = 21;
const int SCL_PIN = 22;
const uint8_t FDC_ADDR = 0x50;
const uint8_t RATE_CODE = 3;            // 400 conversions/s
const int BASELINE_SAMPLES = 100;
const uint32_t TIMEOUT_MS = 50;
const float COUNTS_PER_PF = 524288.0f;
const float CAPDAC_STEP_PF = 3.125f;
const float VALID_RESIDUAL_LIMIT_PF = 14.9f;
const uint8_t REG_FDC_CONFIG = 0x0C;

const uint8_t S = SENSOR - 1;           // 0 or 1 inside the code
uint8_t capdac = 0;
float baselinePF = 0;
int32_t baselineRaw = 0;
bool ready = false;

// ======================= LOW-LEVEL I2C (from GCT_v1.6) ==========================
bool writeReg(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(FDC_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)value);
  return Wire.endTransmission() == 0;
}

bool readReg(uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(FDC_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(FDC_ADDR, (uint8_t)2) != 2) return false;
  uint16_t hi = (uint8_t)Wire.read();
  uint16_t lo = (uint8_t)Wire.read();
  value = (hi << 8) | lo;
  return true;
}

int32_t decodeResult(uint16_t msb, uint16_t lsb) {
  uint32_t bits = ((uint32_t)msb << 8) | (lsb >> 8);
  return (bits & 0x800000U) ? (int32_t)bits - 16777216L : (int32_t)bits;
}

bool configureSensor() {
  // CINx single-ended against the CAPDAC (CHB = 4)
  uint16_t conf = ((uint16_t)S << 13) | (4U << 10) | ((uint16_t)capdac << 5);
  return writeReg(0x08 + S, conf);
}

// Read the latest result for our sensor (raw 24-bit counts)
bool readResult(int32_t &raw) {
  uint16_t msb, lsb;
  if (!readReg(S * 2, msb) || !readReg(S * 2 + 1, lsb)) return false;
  raw = decodeResult(msb, lsb);
  return true;
}

// Single triggered measurement (used while tuning / baselining)
bool readOne(int32_t &raw) {
  uint16_t trigger = ((uint16_t)RATE_CODE << 10) | (0x0080U >> S);
  uint16_t doneMask = 0x0008U >> S;
  if (!writeReg(REG_FDC_CONFIG, trigger)) return false;
  uint32_t startMS = millis();
  while (true) {
    uint16_t status;
    if (!readReg(REG_FDC_CONFIG, status)) return false;
    if (status & doneMask) break;
    if ((uint32_t)(millis() - startMS) >= TIMEOUT_MS) return false;
    delay(1);
  }
  return readResult(raw);
}

bool tuneOffset() {
  capdac = 0;
  for (int tries = 0; tries < 64; tries++) {
    if (!configureSensor()) return false;
    int32_t raw;
    if (!readOne(raw)) return false;
    float residual = raw / COUNTS_PER_PF;
    if (residual > 4.0f && capdac < 31) { capdac++; continue; }
    if (residual < -4.0f && capdac > 0) { capdac--; continue; }
    return fabsf(residual) < VALID_RESIDUAL_LIMIT_PF;
  }
  return false;
}

bool initializeFDC() {
  uint16_t manufacturer, device;
  if (!readReg(0xFE, manufacturer) || !readReg(0xFF, device)) return false;
  if (manufacturer != 0x5449 || device != 0x1004) return false;
  if (!writeReg(REG_FDC_CONFIG, 0x8000)) return false;   // software reset
  delay(10);
  uint16_t status;
  if (!readReg(REG_FDC_CONFIG, status) || (status & 0x8000)) return false;
  return configureSensor();
}

// Tune CAPDAC, average 100 unloaded readings, then start continuous measuring
void takeBaseline() {
  ready = false;
  writeReg(REG_FDC_CONFIG, 0x0000);                         // stop continuous mode
  if (!tuneOffset()) {
    Serial.printf("ERROR: CIN%d reading out of range - check that pad's wiring.\n", SENSOR);
    return;
  }
  double sum = 0;
  int good = 0;
  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    int32_t raw;
    if (!readOne(raw) || fabsf(raw / COUNTS_PER_PF) >= VALID_RESIDUAL_LIMIT_PF) continue;
    sum += raw;
    good++;
  }
  if (good < BASELINE_SAMPLES * 8 / 10) {
    Serial.println("ERROR: too many failed baseline readings. Unload the sensor and send b.");
    return;
  }
  baselineRaw = (int32_t)(sum / good);
  baselinePF = baselineRaw / COUNTS_PER_PF + capdac * CAPDAC_STEP_PF;
  // Repeat measurement of our sensor only, 400/s
  writeReg(REG_FDC_CONFIG, ((uint16_t)RATE_CODE << 10) | (1U << 8) | (0x0080U >> S));
  ready = true;
  if (!PLOT_MODE) {
    Serial.printf("Baseline: %.3f pF on CIN%d (CAPDAC %d)\n", baselinePF, SENSOR, capdac);
    if (CSV_MODE) Serial.println("t_s,raw,delta_raw,cap_pF,delta_pF,timeouts");
  }
}

// Wait for the next new result from the chip (continuous mode)
bool readNext(int32_t &raw) {
  uint16_t doneMask = 0x0008U >> S;
  uint32_t startMS = millis();
  while (true) {
    uint16_t status;
    if (!readReg(REG_FDC_CONFIG, status)) return false;
    if (status & doneMask) return readResult(raw);       // reading clears DONE
    if ((uint32_t)(millis() - startMS) >= TIMEOUT_MS) return false;
    delayMicroseconds(200);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(20);
  if (!initializeFDC()) {
    Serial.println("ERROR: FDC1004 not detected. Check 3V3, GND, SDA 21, SCL 22.");
    return;
  }
  takeBaseline();   // sensor must be UNLOADED at power-up
}

void loop() {
  bool rebase = false;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'b' || c == 'B') rebase = true;
  }
  if (rebase) {
    if (!initializeFDC()) { Serial.println("ERROR: FDC1004 not detected."); return; }
    takeBaseline();
    return;
  }
  if (!ready) { delay(10); return; }

  int64_t sumRaw = 0;
  int good = 0, timeouts = 0;
  for (int i = 0; i < AVG_N; i++) {
    int32_t raw;
    if (readNext(raw) && fabsf(raw / COUNTS_PER_PF) < VALID_RESIDUAL_LIMIT_PF) { sumRaw += raw; good++; }
    else timeouts++;
  }
  double t_s = esp_timer_get_time() / 1e6;

  if (!good) {
    if (PLOT_MODE) return;
    if (CSV_MODE) Serial.printf("%.3f,TIMEOUT,,,,%d\n", t_s, timeouts);
    else Serial.printf("t=%8.3f s | TIMEOUT - no reading from the FDC1004\n", t_s);
    return;
  }
  int32_t raw = (int32_t)(sumRaw / good);
  float capPF = raw / COUNTS_PER_PF + capdac * CAPDAC_STEP_PF;
  float deltaPF = capPF - baselinePF;

  if (PLOT_MODE) {
    Serial.printf("change_pF:%.4f,top:%.2f,bottom:%.2f\n", deltaPF, PLOT_RANGE, -PLOT_RANGE);
  } else if (CSV_MODE) {
    Serial.printf("%.3f,%ld,%ld,%.4f,%.4f,%d\n", t_s, (long)raw, (long)(raw - baselineRaw),
                  capPF, deltaPF, timeouts);
  } else {
    Serial.printf("t=%8.3f s | C=%8.4f pF | change=%+8.4f pF | CIN%d | timeouts=%d/%d\n",
                  t_s, capPF, deltaPF, SENSOR, timeouts, AVG_N);
  }
}
