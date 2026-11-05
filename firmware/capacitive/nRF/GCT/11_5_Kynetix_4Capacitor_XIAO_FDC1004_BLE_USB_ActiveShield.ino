/* KYNETIX | XIAO nRF52840 Sense | FDC1004 + 4 capacitive sensors + BLE + USB
   ACTIVE-SHIELD + NOISE-DEADBAND VERSION
   Works with the Kynetix Capacitive Dashboard: same Bluetooth name, same 16-byte packet,
   same USB CSV (t_ms,S1,S2,S3,S4,S5,contact).

   Board package: Seeed nRF52 mbed-enabled Boards ("XIAO nRF52840 Sense (No Updates)")
   Library: ArduinoBLE (Library Manager). Wire is built in.

   WIRING
     XIAO 3V3 -> FDC1004 3.3V        XIAO GND -> FDC1004 GND
     XIAO D4 (pin labelled 4) -> FDC1004 SDA
     XIAO D5 (pin labelled 5) -> FDC1004 SCL
     Sensor n: sensing plate -> CINn, other plate -> GND.
     ACTIVE SHIELD (copper shield layers):
       Shields for the CIN1 and CIN2 sensors -> FDC1004 SHLD1
       Shields for the CIN3 and CIN4 sensors -> FDC1004 SHLD2
       The shields must NOT also be connected to GND, and must not touch either plate.
       The FDC1004 drives SHLD1/SHLD2 automatically during every measurement;
       no code is needed to turn the shield on.
     Nothing else may be connected to D4/D5 (no FSRs on those pins: they are the I2C bus).

   UNITS: each number is the CHANGE IN CAPACITANCE in femtofarads (fF): 1000 fF = 1 pF.

   CHANNEL -> DASHBOARD SPOT (edit SLOT_OF_CIN to match where each sensor sits)
     Slots: 0 = big toe, 1 = lateral midfoot (unused), 2 = 5th met, 3 = 1st met, 4 = heel.
     Default: CIN1 -> heel, CIN2 -> 1st met, CIN3 -> 5th met, CIN4 -> big toe.

   START-UP
     Keep all four sensors UNLOADED for ~2 s after power-on/reset (offset tuning + zero).
     Serial Monitor (115200): type  b  + Enter to re-zero, lines starting with # are status.
     Type  d  + Enter to toggle a diagnostic line 10x per second with each pad's SIGNED
     change in pF (the normal output can't show decreases; this can). The diagnostic line
     shows the raw change, before the deadband, so use it to pick DEADBAND_FF.

   TUNING DEADBAND_FF
     1. Turn on diagnostics (d). Hold your hand near a sensor without touching: note the
        largest change (e.g. +0.15 pF = 150 fF).
     2. Step on the sensor: note the change (e.g. +1.2 pF = 1200 fF).
     3. Set DEADBAND_FF somewhere between the two, closer to the hand value.
*/
#include <ArduinoBLE.h>
#include <Wire.h>
#include <math.h>

// ---------------- user settings ----------------
const int8_t SLOT_OF_CIN[4] = {4, 3, 2, 0};   // CIN1..CIN4 -> dashboard slot index
const uint32_t I2C_HZ = 400000;              // use 100000 if wires are long or readings drop out
const uint16_t DEADBAND_FF = 300;            // changes below this are sent as 0 (raise if a nearby hand still shows contact)
const float DRIFT_RATE = 0.001f;             // slow automatic re-zero while unloaded (~10 s time constant); 0 = off
// ------------------------------------------------

const int NCH = 4, NSLOT = 5;
const uint8_t FDC_ADDR = 0x50;
const uint8_t RATE_CODE = 3;                 // 400 S/s per measurement; 4 in turn -> ~100 Hz each
const float COUNTS_PER_PF = 524288.0f;       // 2^19 counts per pF
const float CAPDAC_STEP_PF = 3.125f;
const float VALID_PF = 14.9f, RERANGE_PF = 10.0f;
const uint32_t PERIOD_US = 10000;            // 100 Hz output
const int BASELINE_SAMPLES = 50;             // per channel (~0.5 s)
const uint16_t ON_FF = 400, OFF_FF = 300;    // firmware contact flag (dashboard decides contact itself)
const float END_FRAC = 0.10f;

const char* SERVICE_UUID = "713d0001-5b8c-4e35-ae3f-5296c7420010";
const char* DATA_UUID    = "713d0002-5b8c-4e35-ae3f-5296c7420010";
BLEService kynetixService(SERVICE_UUID);
BLECharacteristic dataCharacteristic(DATA_UUID, BLERead | BLENotify, 16, true);

uint8_t capdac[NCH] = {0};
float baselinePF[NCH] = {0};
float latestPF[NCH] = {0};
float hist[NCH][3] = {{0}};                  // last 3 readings per channel (median filter)
uint8_t histN[NCH] = {0}, histI[NCH] = {0};
float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}
bool fdcOK = false;
uint32_t nextSample = 0, lastGoodMs = 0, lastDoneMs = 0;
uint16_t lastStatus = 0;
bool warnedNoDone = false;
bool diag = false;
uint32_t lastDiagMs = 0;
uint8_t sequenceNumber = 0;
bool inContact = false;
uint32_t stepPeak = 0, airFloor = 0;

// ---------------- FDC1004 low level ----------------
bool writeReg(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(FDC_ADDR);
  Wire.write(reg); Wire.write((uint8_t)(value >> 8)); Wire.write((uint8_t)value);
  return Wire.endTransmission() == 0;
}
bool readReg(uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(FDC_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(true) != 0) return false;
  if (Wire.requestFrom(FDC_ADDR, (uint8_t)2) != 2) return false;
  uint16_t hi = (uint8_t)Wire.read(), lo = (uint8_t)Wire.read();
  value = (hi << 8) | lo;
  return true;
}
int32_t decodeResult(uint16_t msb, uint16_t lsb) {
  uint32_t bits = ((uint32_t)msb << 8) | (lsb >> 8);
  return (bits & 0x800000U) ? (int32_t)bits - 16777216L : (int32_t)bits;
}
// Measurement slot ch (0..3) measures CIN(ch+1) against the CAPDAC offset.
// (Single-ended mode: the FDC1004 drives the shield pins automatically.)
bool configureChannel(int ch) {
  return writeReg(0x08 + ch, ((uint16_t)ch << 13) | (4U << 10) | ((uint16_t)capdac[ch] << 5));
}
bool readResultPF(int ch, float &residualPF) {
  uint16_t msb, lsb;
  if (!readReg(0x00 + 2 * ch, msb) || !readReg(0x01 + 2 * ch, lsb)) return false;
  residualPF = decodeResult(msb, lsb) / COUNTS_PER_PF;
  return isfinite(residualPF);
}
bool readOneShot(int ch, float &residualPF) {      // single measurement (used during setup)
  if (!writeReg(0x0C, ((uint16_t)RATE_CODE << 10) | (0x0080U >> ch))) return false;
  uint32_t t0 = millis();
  while (true) {
    uint16_t st;
    if (!readReg(0x0C, st)) return false;
    if (st & (0x0008U >> ch)) break;
    if (millis() - t0 > 50) return false;
    delay(1);
  }
  return readResultPF(ch, residualPF);
}
bool tuneOffset(int ch) {                          // pick the CAPDAC that centres the reading
  capdac[ch] = 0;
  for (int i = 0; i < 64; i++) {
    if (!configureChannel(ch)) return false;
    float r;
    if (!readOneShot(ch, r)) return false;
    if (r > 4.0f && capdac[ch] < 31) { capdac[ch]++; continue; }
    if (r < -4.0f && capdac[ch] > 0) { capdac[ch]--; continue; }
    return fabsf(r) < VALID_PF;
  }
  return false;
}
void rerange(int ch, float residualPF) {           // follow large changes (hand on a wire, etc.)
  int steps = (int)lroundf(residualPF / CAPDAC_STEP_PF);
  int c = constrain((int)capdac[ch] + steps, 0, 31);
  if (c == capdac[ch]) return;
  capdac[ch] = (uint8_t)c;
  configureChannel(ch);
}

// Full start-up: detect chip, tune each channel, take the zero, start continuous mode.
bool setupFDC() {
  uint16_t man, dev;
  if (!readReg(0xFE, man) || !readReg(0xFF, dev) || man != 0x5449 || dev != 0x1004) {
    if (Serial) Serial.println("# FDC1004 not found: check 3V3, GND, SDA = D4, SCL = D5");
    return false;
  }
  writeReg(0x0C, 0x8000); delay(10);                // reset the chip
  for (int ch = 0; ch < NCH; ch++) {
    if (!tuneOffset(ch)) {
      if (Serial) { Serial.print("# CIN"); Serial.print(ch + 1); Serial.println(": out of range (short to GND or shield? over ~110 pF?)"); }
      capdac[ch] = 0; configureChannel(ch);
    }
  }
  for (int ch = 0; ch < NCH; ch++) {                // zero = average with sensors unloaded
    double sum = 0; int good = 0;
    for (int k = 0; k < BASELINE_SAMPLES; k++) {
      float r;
      if (readOneShot(ch, r) && fabsf(r) < VALID_PF) { sum += r + capdac[ch] * CAPDAC_STEP_PF; good++; }
    }
    baselinePF[ch] = good ? (float)(sum / good) : 0;
    latestPF[ch] = baselinePF[ch];
    histN[ch] = 0; histI[ch] = 0;
  }
  // continuous mode: all four measurements, repeating
  if (!writeReg(0x0C, ((uint16_t)RATE_CODE << 10) | 0x0100 | 0x00F0)) return false;
  if (Serial) {
    Serial.print("# FDC1004 ready. Zero pF:");
    for (int ch = 0; ch < NCH; ch++) { Serial.print(" CIN"); Serial.print(ch + 1); Serial.print('='); Serial.print(baselinePF[ch], 3); }
    Serial.println();
  }
  lastGoodMs = millis(); lastDoneMs = millis();
  return true;
}

// Read whichever channels have a new result.
void pollFDC() {
  uint16_t st;
  if (!readReg(0x0C, st)) return;
  lastStatus = st;
  if (!(st & 0x0100)) {                               // chip lost its settings (power dip)
    if (Serial) { Serial.print("# FDC1004 lost its settings (status 0x"); Serial.print(st, HEX); Serial.println("): restarting"); }
    fdcOK = setupFDC(); return;
  }
  bool anyDone = (st & 0x000F) != 0;
  if (anyDone) lastDoneMs = millis();
  // Fallback: if no "done" flags for 100 ms, read the results anyway.
  bool readAll = !anyDone && millis() - lastDoneMs > 100;
  if (readAll && !warnedNoDone && Serial) { warnedNoDone = true; Serial.println("# note: no DONE flags seen, reading results directly"); }
  for (int ch = 0; ch < NCH; ch++) {
    if (!readAll && !(st & (0x0008U >> ch))) continue;
    float r;
    if (!readResultPF(ch, r)) continue;
    if (fabsf(r) < VALID_PF) {
      float pf = r + capdac[ch] * CAPDAC_STEP_PF;
      hist[ch][histI[ch]] = pf; histI[ch] = (histI[ch] + 1) % 3; if (histN[ch] < 3) histN[ch]++;
      latestPF[ch] = histN[ch] < 3 ? pf : median3(hist[ch][0], hist[ch][1], hist[ch][2]);   // drops 1-sample spikes
      lastGoodMs = millis();
      if (fabsf(r) > RERANGE_PF) rerange(ch, r);
    } else {
      rerange(ch, r > 0 ? 4 * CAPDAC_STEP_PF : -4 * CAPDAC_STEP_PF);
    }
  }
}

// Slowly pull the zero toward the current reading while a pad is unloaded,
// so temperature / humidity drift doesn't creep above the deadband.
void trackDrift() {
  if (DRIFT_RATE <= 0 || inContact) return;
  for (int ch = 0; ch < NCH; ch++) {
    float dfF = (latestPF[ch] - baselinePF[ch]) * 1000.0f;
    if (fabsf(dfF) < DEADBAND_FF) baselinePF[ch] += DRIFT_RATE * (latestPF[ch] - baselinePF[ch]);
  }
}

void putU16(uint8_t* p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
void putU32(uint8_t* p, uint32_t v) { p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; p[2] = (v >> 16) & 0xff; p[3] = (v >> 24) & 0xff; }

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) delay(10);   // give the Serial Monitor a moment (not required)
  Wire.begin();                                        // XIAO: SDA = D4, SCL = D5
  Wire.setClock(I2C_HZ);
  fdcOK = setupFDC();
  if (!BLE.begin()) {
    if (Serial) Serial.println("# BLE initialization failed");
    while (true) delay(1000);
  }
  BLE.setLocalName("Kynetix Cap Insole");
  BLE.setDeviceName("Kynetix Cap Insole");
  BLE.setAdvertisedService(kynetixService);
  kynetixService.addCharacteristic(dataCharacteristic);
  BLE.addService(kynetixService);
  uint8_t initial[16] = {0};
  dataCharacteristic.writeValue(initial, sizeof(initial));
  BLE.advertise();
  if (Serial) Serial.println("t_ms,S1,S2,S3,S4,S5,contact");
  nextSample = micros();
  lastGoodMs = millis(); lastDoneMs = millis();       // Bluetooth start-up takes time: don't count it as a dropout
}

void loop() {
  BLE.poll();
  while (Serial && Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'b' || c == 'B') fdcOK = setupFDC();      // re-zero (sensors unloaded!)
    if (c == 'd' || c == 'D') { diag = !diag; Serial.println(diag ? "# diagnostics ON" : "# diagnostics OFF"); }
  }
  const uint32_t nowUS = micros();
  if ((int32_t)(nowUS - nextSample) < 0) return;
  if ((uint32_t)(nowUS - nextSample) > 50000UL) nextSample = nowUS;
  nextSample += PERIOD_US;
  const uint32_t timestamp = millis();

  if (!fdcOK) {
    static uint32_t lastTry = 0;
    if (timestamp - lastTry > 2000) { lastTry = timestamp; fdcOK = setupFDC(); }
  } else {
    pollFDC();
    trackDrift();
    if ((int32_t)(millis() - lastGoodMs) > 1000) {      // no valid data for 1 s: restart the chip
      if (Serial) { Serial.print("# FDC1004 gave no valid readings for 1 s (status 0x"); Serial.print(lastStatus, HEX); Serial.println("): restarting"); }
      fdcOK = setupFDC();
    }
  }

  // change in capacitance per dashboard slot, in femtofarads (fF), with noise deadband
  uint16_t values[NSLOT] = {0};
  uint32_t total = 0;
  for (int ch = 0; ch < NCH; ch++) {
    int8_t slot = SLOT_OF_CIN[ch];
    if (slot < 0 || slot >= NSLOT) continue;
    float dfF = (latestPF[ch] - baselinePF[ch]) * 1000.0f;
    if (dfF < DEADBAND_FF) dfF = 0;                    // hand nearby / noise -> reported as 0
    values[slot] = (uint16_t)constrain((long)lroundf(dfF), 0L, 65535L);
    total += values[slot];
  }

  // simple contact flag. The dashboard decides contact itself from the values.
  if (!inContact) {
    if (total < airFloor) airFloor = total;
    if (total > ON_FF && total - airFloor > ON_FF) { inContact = true; stepPeak = total; }
  } else {
    if (total > stepPeak) stepPeak = total;
    uint32_t off = (uint32_t)(END_FRAC * stepPeak);
    if (off < OFF_FF) off = OFF_FF;
    if (total < off) { inContact = false; airFloor = total; }
  }

  uint8_t packet[16];
  putU32(packet, timestamp);
  for (int i = 0; i < NSLOT; i++) putU16(packet + 4 + 2 * i, values[i]);
  packet[14] = inContact ? 1 : 0;
  packet[15] = sequenceNumber++;
  if (dataCharacteristic.subscribed()) dataCharacteristic.writeValue(packet, sizeof(packet));

  if (diag && Serial && timestamp - lastDiagMs >= 100) {   // signed change per pad, in pF (raw, before deadband)
    lastDiagMs = timestamp;
    Serial.print("# dpF");
    for (int ch = 0; ch < NCH; ch++) {
      Serial.print(" CIN"); Serial.print(ch + 1); Serial.print('=');
      float d = latestPF[ch] - baselinePF[ch];
      if (d >= 0) Serial.print('+');
      Serial.print(d, 3);
      Serial.print(" (CAPDAC "); Serial.print(capdac[ch]); Serial.print(')');
    }
    Serial.println();
  }
  if (Serial && !diag) {
    Serial.print(timestamp);
    for (int i = 0; i < NSLOT; i++) { Serial.print(','); Serial.print(values[i]); }
    Serial.print(','); Serial.println(inContact ? 1 : 0);
  }
}
