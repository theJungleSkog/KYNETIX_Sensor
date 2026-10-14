// KYNETIX Ground Contact Timer - GCT_v1.4
// Original two-sensor ProtoCentral FDC1004 measurement and wiring.
// ESP32 3V3 -> breakout 3.3V; breakout VCC unconnected.
// ESP32 GND -> breakout GND; GPIO21 -> SDA; GPIO22 -> SCL.
// Sensor 1: one copper plate -> CIN1, other plate -> GND.
// Sensor 2: one copper plate -> CIN2, other plate -> GND.
// CIN3/4 and SLD1/2 unconnected. No RC resistor.
//
// Ground contact begins when EITHER delta capacitance > 0.5 pF.
// Ground contact ends when BOTH are <= 0.5 pF.
// Only completed steps are printed: Step 1 time: 0.412 s
// Enter b in Serial Monitor to recalibrate with both sensors unloaded.
// Serial baud: 115200. No external libraries needed.

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

const int SDA_PIN = 21;
const int SCL_PIN = 22;
const uint8_t FDC_ADDR = 0x50;
const uint8_t RATE_CODE = 3; // 400 S/s conversion setting, not sampling rate.
const uint32_t PERIOD_US = 25000; // Target 20 paired measurements/second.
const uint32_t TIMEOUT_MS = 50;
const int BASELINE_SAMPLES = 100;
const float COUNTS_PER_PF = 524288.0f;
const float CAPDAC_STEP_PF = 3.125f;
const float CONTACT_THRESHOLD_PF = 0.5f;

uint8_t capdac[2] = {0, 0};
float baselinePF[2] = {0, 0};
bool baselineValid[2] = {false, false};
bool deviceReady = false;
uint32_t lastOutput = 0;

bool inGroundContact = false;
uint32_t contactStartUS = 0;
uint32_t stepNumber = 0;

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

bool configureSensor(uint8_t sensor) {
  uint16_t conf = ((uint16_t)sensor << 13) |
                  (4U << 10) | ((uint16_t)capdac[sensor] << 5);
  return writeReg(0x08 + sensor, conf);
}

bool readOne(uint8_t sensor, float &residualPF) {
  uint16_t trigger = ((uint16_t)RATE_CODE << 10) | (0x0080U >> sensor);
  uint16_t doneMask = 0x0008U >> sensor;
  if (!writeReg(0x0C, trigger)) return false;

  uint32_t start = millis();
  while (true) {
    uint16_t status;
    if (!readReg(0x0C, status)) return false;
    if (status & doneMask) break;
    if ((uint32_t)(millis() - start) >= TIMEOUT_MS) return false;
    delay(1);
  }

  uint16_t msb, lsb;
  uint8_t dataReg = sensor * 2;
  if (!readReg(dataReg, msb) || !readReg(dataReg + 1, lsb)) return false;
  residualPF = decodeResult(msb, lsb) / COUNTS_PER_PF;
  return true;
}

bool tuneOffset(uint8_t sensor) {
  capdac[sensor] = 0;
  for (int tries = 0; tries < 64; tries++) {
    if (!configureSensor(sensor)) return false;
    float residual;
    if (!readOne(sensor, residual)) return false;
    if (residual > 4.0f && capdac[sensor] < 31) {
      capdac[sensor]++;
      continue;
    }
    if (residual < -4.0f && capdac[sensor] > 0) {
      capdac[sensor]--;
      continue;
    }
    return fabsf(residual) < 14.9f;
  }
  return false;
}

bool initializeFDC() {
  uint16_t manufacturer, device;
  if (!readReg(0xFE, manufacturer) || !readReg(0xFF, device)) return false;
  if (manufacturer != 0x5449 || device != 0x1004) return false;
  if (!writeReg(0x0C, 0x8000)) return false;
  delay(10);
  uint16_t status;
  if (!readReg(0x0C, status) || (status & 0x8000)) return false;
  return configureSensor(0) && configureSensor(1);
}

void takeBaselines() {
  baselineValid[0] = baselineValid[1] = false;
  for (uint8_t sensor = 0; sensor < 2; sensor++) {
    if (!tuneOffset(sensor)) continue;
    double sum = 0;
    int good = 0;
    for (int i = 0; i < BASELINE_SAMPLES; i++) {
      float residual;
      if (!readOne(sensor, residual) || fabsf(residual) >= 14.9f) continue;
      sum += residual + capdac[sensor] * CAPDAC_STEP_PF;
      good++;
    }
    if (good >= BASELINE_SAMPLES * 8 / 10) {
      baselinePF[sensor] = sum / good;
      baselineValid[sensor] = true;
    }
  }
}

float readDelta(uint8_t sensor) {
  if (!deviceReady || !baselineValid[sensor]) return NAN;
  float residual;
  if (!readOne(sensor, residual) || fabsf(residual) >= 14.9f) return NAN;
  return residual + capdac[sensor] * CAPDAC_STEP_PF - baselinePF[sensor];
}

bool baselineRequested() {
  bool requested = false;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'b' || c == 'B') requested = true;
  }
  return requested;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  // Keep both sensors unloaded during startup baseline collection.
  deviceReady = initializeFDC();
  if (deviceReady) takeBaselines();
  lastOutput = micros();
}

void loop() {
  if (baselineRequested()) {
    // Do not count a partially measured step after recalibration.
    inGroundContact = false;
    contactStartUS = 0;
    if (!deviceReady) deviceReady = initializeFDC();
    if (deviceReady) takeBaselines();
    lastOutput = micros();
    return;
  }

  if ((uint32_t)(micros() - lastOutput) < PERIOD_US) {
    delay(1);
    return;
  }
  lastOutput = micros();

  // Same sequential FDC1004 measurements and deltas as the working sketch.
  float delta1 = readDelta(0);
  float delta2 = readDelta(1);
  uint32_t readingTimeUS = micros();

  // Invalid readings cannot safely establish contact or release.
  // Ignore the pair rather than treating NAN as zero or ending a step.
  if (!isfinite(delta1) || !isfinite(delta2)) return;

  bool contact = (delta1 > CONTACT_THRESHOLD_PF) ||
                 (delta2 > CONTACT_THRESHOLD_PF);

  if (contact && !inGroundContact) {
    inGroundContact = true;
    contactStartUS = readingTimeUS;
    ++stepNumber;
  } else if (!contact && inGroundContact) {
    uint32_t elapsedUS = readingTimeUS - contactStartUS;
    Serial.printf("Step %lu time: %.3f s\n",
                  (unsigned long)stepNumber, elapsedUS / 1000000.0f);
    inGroundContact = false;
  }
}
