/* KYNETIX | XIAO nRF52840 Sense | V6 flex insole, 4 pads on ONE FDC1004 | BLE + USB  -- v8 (plotter labels)
   Same Bluetooth name, same 16-byte packet and same USB CSV (t_ms,S1,S2,S3,S4,S5,contact),
   so the Kynetix dashboards keep working. Lateral midfoot is skipped (always sends 0).

   Board package: Seeed nRF52 mbed-enabled Boards ("XIAO nRF52840 Sense (No Updates)")
   Library: ArduinoBLE (Library Manager). Wire is built in.

   WIRING (one FDC1004, hardware I2C, no second chip)
     XIAO 3V3 -> FDC 3V3 (red)      XIAO GND -> FDC GND (black)
     XIAO D4  -> FDC SDA (orange)   XIAO D5  -> FDC SCL (brown)
     Tail pin 13 (S1 heel)    -> CIN1 (yellow)
     Tail pin 3  (S4 1st met) -> CIN2 (green)
     Tail pin 8  (S3 5th met) -> CIN3 (blue)
     Tail pin 6  (S5 big toe) -> CIN4 (purple)
     Tail pins 14, 5, 2 (SH_A, SH_B, SH_C) -> SHLD1 (white, shared breadboard row)
     Tail pins 1 and 15 (GND) -> FDC GND (black)
     Tail pin 11 (lateral midfoot) and pins 4, 7, 9, 10, 12 -> nothing
     NEVER connect an SH pin to GND, SHLD2 or a CIN. The lid is grounded through its ear jumper.

   TWO OUTPUT STYLES (type p + Enter to switch):
     PLOTTER (default): 4 labelled lines, 10 times per second (about 5 s on the Serial Plotter):
         Heel:12,1stMet:-3,5thMet:40,BigToe:7      (change since zero, in fF, can go negative)
     DASHBOARD: the usual CSV at 100 Hz (t_ms,S1,S2,S3,S4,S5,contact) + "# health"/"# noise"
         lines every 2 s. Use this for the website over USB.  (Bluetooth works in both.)

   SERIAL MONITOR / PLOTTER COMMANDS (115200, "New Line"):
     z = quick re-zero (insole untouched)     b = full restart + re-zero (insole untouched)
     p = switch plotter <-> dashboard output   d = diagnostic text line 10x/s (in pF)

   UNITS: change in capacitance in femtofarads (fF): 1000 fF = 1 pF.
   DASHBOARD CSV slots: S1 big toe, S2 lateral midfoot (0), S3 5th met, S4 1st met, S5 heel.
   START-UP: keep the insole UNLOADED for ~2 s after power-on/reset.
*/
#include <ArduinoBLE.h>
#include <Wire.h>
#include <math.h>

// ---------------- user settings ----------------
const int NUM_FDC = 1;                       // 1 = one FDC1004 (4 pads). 2 = two FDC1004s (5 pads)
uint8_t runMode = 1;                         // one chip: always 1 (FDC A only)
const uint32_t I2C_HZ = 400000;              // FDC A bus. If "# health" shows I2C errors, change to 100000
const int SOFT_SDA = D2, SOFT_SCL = D3;      // FDC B bus (software I2C, only used with 2 chips)
const int SOFT_DELAY_US = 2;                 // FDC B bit timing
// slot of each CIN (CIN1..CIN4) for each FDC1004, -1 = nothing connected
// slot numbers: 0 big toe, 1 lateral midfoot, 2 5th met, 3 1st met, 4 heel
// (SLOTS_2FDC is only used if NUM_FDC = 2)
const int8_t SLOTS_1FDC[1][4] = { {0, 4, 3, 2} };                     // CIN1 heel, CIN2 1st met, CIN3 5th met, CIN4 big toe
const int8_t SLOTS_2FDC[2][4] = { {-1, 3, 2, -1},                     // A: -, 1st met, 5th met, (CIN4 off)
                                  { 4, 1, -1, -1} };                  // B: heel, lateral midfoot
// ------------------------------------------------

const int NCH = 4, NSLOT = 5, MAXFDC = 2;
const char* SLOT_NAME[NSLOT] = {"bigtoe(flexS5)", "latmid(flexS2)", "5thmet(flexS3)", "1stmet(flexS4)", "heel(flexS1)"};
const char* SLOT_SHORT[NSLOT] = {"bigtoe", "latmid", "5thmet", "1stmet", "heel"};
const char* MODE_NAME[5] = {"", "1 = FDC A only", "2 = FDC B only", "3 = both at once", "4 = take turns"};
const uint8_t FDC_ADDR = 0x50;
const uint8_t RATE_CODE = 3;                 // 400 S/s per measurement
const uint32_t CONV_US = 2500;               // one measurement at 400 S/s
const float COUNTS_PER_PF = 524288.0f;       // 2^19 counts per pF
const float CAPDAC_STEP_PF = 3.125f;
const float VALID_PF = 14.9f, RERANGE_PF = 10.0f;
const uint32_t PERIOD_US = 10000;            // 100 Hz output
const uint32_t POLL_US = 1000;               // check the chip every 1 ms
const int BASELINE_SAMPLES = 50;             // per channel (~0.5 s)
const uint16_t ON_FF = 400, OFF_FF = 300;    // firmware contact flag (dashboard decides contact itself)
const float END_FRAC = 0.10f;
const float ZERO_FOLLOW_DOWN = 0.05f;        // zero follows drift DOWN (~0.2 s) so presses are never hidden below 0
const uint32_t HEALTH_MS = 2000;             // "# health" + "# noise" lines every 2 s (dashboard style only)
bool plotMode = true;                        // true = labelled plotter lines, false = dashboard CSV (type p to switch)
const uint32_t PLOT_DIV = 10;                // plotter style: print every 10th sample (10 lines/s)
const char* PLOT_LABEL[5] = {"BigToe", "LatMid", "5thMet", "1stMet", "Heel"};

const char* SERVICE_UUID = "713d0001-5b8c-4e35-ae3f-5296c7420010";
const char* DATA_UUID    = "713d0002-5b8c-4e35-ae3f-5296c7420010";
BLEService kynetixService(SERVICE_UUID);
BLECharacteristic dataCharacteristic(DATA_UUID, BLERead | BLENotify, 16, true);

struct Fdc {
  bool ok;
  int8_t slot[NCH];
  uint8_t capdac[NCH];
  uint16_t enable;                           // MEAS enable bits of the used channels
  uint8_t usedMask, nUsed;
  float baselinePF[NCH], latestPF[NCH];
  float hist[NCH][3];
  uint8_t histN[NCH], histI[NCH];
  uint32_t lastGoodMs, lastDoneMs, lastTry;
  uint16_t lastStatus;
  bool warnedNoDone;
};
Fdc fdc[MAXFDC];
const char FDC_NAME[MAXFDC] = {'A', 'B'};

// take-turns state (2 chips only)
int curChip = 0;
uint32_t curStartUs = 0, lastCheckUs = 0;
uint8_t pending[MAXFDC];

// quick re-zero state
bool zeroing = false;
uint32_t zeroFromMs = 0, zeroToMs = 0;
double zSum[MAXFDC][NCH]; uint32_t zN[MAXFDC][NCH];

// health counters (reset every HEALTH_MS)
uint32_t hErr[MAXFDC], hRes[MAXFDC], hOor[MAXFDC], hRestart[MAXFDC], hTurns = 0, hLate = 0;
uint32_t hTicks = 0, lastHealthMs = 0;
double nSum[NSLOT], nSq[NSLOT]; uint32_t nCnt[NSLOT];

bool diag = false;
uint32_t nextSample = 0, nextPoll = 0, lastDiagMs = 0;
uint8_t sequenceNumber = 0;
bool inContact = false;
uint32_t stepPeak = 0, airFloor = 0;

float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}
bool used(int c, int ch) { return fdc[c].slot[ch] >= 0 && fdc[c].slot[ch] < NSLOT; }
bool chipActive(int c) {
  if (c >= NUM_FDC || !fdc[c].ok) return false;
  if (runMode == 1) return c == 0;
  if (runMode == 2) return c == 1;
  return true;
}
bool takingTurns() { return runMode == 4 && NUM_FDC == 2 && fdc[0].ok && fdc[1].ok; }

// ---------------- software I2C pins (FDC B, only used with 2 chips) ----------------
#if defined(NRF_P0)
const bool FAST_PINS = true;
const uint32_t SDA_BIT = 1UL << 28, SCL_BIT = 1UL << 29;
inline void ioInit() {
  NRF_P0->OUTSET = SDA_BIT | SCL_BIT;
  NRF_P0->PIN_CNF[28] = 0x60D;
  NRF_P0->PIN_CNF[29] = 0x60D;
}
inline void sdaRelease() { NRF_P0->OUTSET = SDA_BIT; }
inline void sdaLow()     { NRF_P0->OUTCLR = SDA_BIT; }
inline void sclLow()     { NRF_P0->OUTCLR = SCL_BIT; }
inline bool sdaRead()    { return (NRF_P0->IN & SDA_BIT) != 0; }
inline bool sclRead()    { return (NRF_P0->IN & SCL_BIT) != 0; }
inline void sclUp()      { NRF_P0->OUTSET = SCL_BIT; }
#else
const bool FAST_PINS = false;
inline void ioInit() {}
inline void sdaRelease() { pinMode(SOFT_SDA, INPUT_PULLUP); }
inline void sdaLow()     { pinMode(SOFT_SDA, OUTPUT); digitalWrite(SOFT_SDA, LOW); }
inline void sclLow()     { pinMode(SOFT_SCL, OUTPUT); digitalWrite(SOFT_SCL, LOW); }
inline bool sdaRead()    { return digitalRead(SOFT_SDA); }
inline bool sclRead()    { return digitalRead(SOFT_SCL); }
inline void sclUp()      { pinMode(SOFT_SCL, INPUT_PULLUP); }
#endif
inline void bitDelay() { delayMicroseconds(SOFT_DELAY_US); }
inline void sclRelease() {
  sclUp();
  uint32_t t0 = micros();
  while (!sclRead() && micros() - t0 < 1000) {}
}
void softStart() { sdaRelease(); bitDelay(); sclRelease(); bitDelay(); sdaLow(); bitDelay(); sclLow(); bitDelay(); }
void softStop() { sdaLow(); bitDelay(); sclRelease(); bitDelay(); sdaRelease(); bitDelay(); }
bool softWriteByte(uint8_t b) {
  for (int i = 0; i < 8; i++) {
    if (b & 0x80) sdaRelease(); else sdaLow();
    bitDelay(); sclRelease(); bitDelay(); sclLow();
    b <<= 1;
  }
  sdaRelease(); bitDelay(); sclRelease(); bitDelay();
  bool ack = !sdaRead();
  sclLow(); bitDelay();
  return ack;
}
uint8_t softReadByte(bool ack) {
  uint8_t b = 0;
  sdaRelease();
  for (int i = 0; i < 8; i++) {
    bitDelay(); sclRelease(); bitDelay();
    b = (b << 1) | (sdaRead() ? 1 : 0);
    sclLow();
  }
  if (ack) sdaLow(); else sdaRelease();
  bitDelay(); sclRelease(); bitDelay(); sclLow(); bitDelay();
  sdaRelease();
  return b;
}
void softBusInit() {
  ioInit();
  sdaRelease(); sclRelease();
  for (int i = 0; i < 9; i++) { sclLow(); bitDelay(); sclRelease(); bitDelay(); }
  softStop();
}
bool softWriteReg(uint8_t reg, uint16_t value) {
  softStart();
  bool ok = softWriteByte(FDC_ADDR << 1) && softWriteByte(reg) &&
            softWriteByte((uint8_t)(value >> 8)) && softWriteByte((uint8_t)value);
  softStop();
  return ok;
}
bool softReadReg(uint8_t reg, uint16_t &value) {
  softStart();
  if (!softWriteByte(FDC_ADDR << 1) || !softWriteByte(reg)) { softStop(); return false; }
  softStart();
  if (!softWriteByte((FDC_ADDR << 1) | 1)) { softStop(); return false; }
  uint16_t hi = softReadByte(true), lo = softReadByte(false);
  softStop();
  value = (hi << 8) | lo;
  return true;
}

// ---------------- FDC1004 register access (c = 0 -> FDC A on Wire, 1 -> FDC B on soft I2C) ----------------
bool writeReg(int c, uint8_t reg, uint16_t value) {
  bool ok;
  if (c == 1) ok = softWriteReg(reg, value);
  else {
    Wire.beginTransmission(FDC_ADDR);
    Wire.write(reg); Wire.write((uint8_t)(value >> 8)); Wire.write((uint8_t)value);
    ok = Wire.endTransmission() == 0;
  }
  if (!ok) hErr[c]++;
  return ok;
}
bool readReg(int c, uint8_t reg, uint16_t &value) {
  bool ok = false;
  if (c == 1) ok = softReadReg(reg, value);
  else {
    Wire.beginTransmission(FDC_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(true) == 0 && Wire.requestFrom(FDC_ADDR, (uint8_t)2) == 2) {
      uint16_t hi = (uint8_t)Wire.read(), lo = (uint8_t)Wire.read();
      value = (hi << 8) | lo;
      ok = true;
    }
  }
  if (!ok) hErr[c]++;
  return ok;
}
int32_t decodeResult(uint16_t msb, uint16_t lsb) {
  uint32_t bits = ((uint32_t)msb << 8) | (lsb >> 8);
  return (bits & 0x800000U) ? (int32_t)bits - 16777216L : (int32_t)bits;
}
bool configureChannel(int c, int ch) {
  return writeReg(c, 0x08 + ch, ((uint16_t)ch << 13) | (4U << 10) | ((uint16_t)fdc[c].capdac[ch] << 5));
}
bool readResultPF(int c, int ch, float &residualPF) {
  uint16_t msb, lsb;
  if (!readReg(c, 0x00 + 2 * ch, msb) || !readReg(c, 0x01 + 2 * ch, lsb)) return false;
  residualPF = decodeResult(msb, lsb) / COUNTS_PER_PF;
  return isfinite(residualPF);
}
bool readOneShot(int c, int ch, float &residualPF) {
  if (!writeReg(c, 0x0C, ((uint16_t)RATE_CODE << 10) | (0x0080U >> ch))) return false;
  uint32_t t0 = millis();
  while (true) {
    uint16_t st;
    if (!readReg(c, 0x0C, st)) return false;
    if (st & (0x0008U >> ch)) break;
    if (millis() - t0 > 50) return false;
    delay(1);
  }
  return readResultPF(c, ch, residualPF);
}
bool tuneOffset(int c, int ch) {
  fdc[c].capdac[ch] = 0;
  for (int i = 0; i < 64; i++) {
    if (!configureChannel(c, ch)) return false;
    float r;
    if (!readOneShot(c, ch, r)) return false;
    if (r > 4.0f && fdc[c].capdac[ch] < 31) { fdc[c].capdac[ch]++; continue; }
    if (r < -4.0f && fdc[c].capdac[ch] > 0) { fdc[c].capdac[ch]--; continue; }
    return fabsf(r) < VALID_PF;
  }
  return false;
}
void rerange(int c, int ch, float residualPF) {
  int steps = (int)lroundf(residualPF / CAPDAC_STEP_PF);
  int v = constrain((int)fdc[c].capdac[ch] + steps, 0, 31);
  if (v == fdc[c].capdac[ch]) return;
  fdc[c].capdac[ch] = (uint8_t)v;
  configureChannel(c, ch);
}

void printPad(int c, int ch) {
  Serial.print(" FDC_"); Serial.print(FDC_NAME[c]); Serial.print("-CIN"); Serial.print(ch + 1);
  Serial.print('['); Serial.print(SLOT_NAME[fdc[c].slot[ch]]); Serial.print(']');
}

void processResult(int c, int ch, float r) {
  Fdc &f = fdc[c];
  if (fabsf(r) < VALID_PF) {
    hRes[c]++;
    float pf = r + f.capdac[ch] * CAPDAC_STEP_PF;
    if (zeroing && millis() >= zeroFromMs) { zSum[c][ch] += pf; zN[c][ch]++; }
    f.hist[ch][f.histI[ch]] = pf; f.histI[ch] = (f.histI[ch] + 1) % 3; if (f.histN[ch] < 3) f.histN[ch]++;
    f.latestPF[ch] = f.histN[ch] < 3 ? pf : median3(f.hist[ch][0], f.hist[ch][1], f.hist[ch][2]);
    f.lastGoodMs = millis();
    if (fabsf(r) > RERANGE_PF) rerange(c, ch, r);
  } else {
    hOor[c]++;
    rerange(c, ch, r > 0 ? 4 * CAPDAC_STEP_PF : -4 * CAPDAC_STEP_PF);
  }
}

void startChip(int c) { fdc[c].lastDoneMs = millis(); writeReg(c, 0x0C, ((uint16_t)RATE_CODE << 10) | 0x0100 | fdc[c].enable); }
void stopChip(int c)  { writeReg(c, 0x0C, ((uint16_t)RATE_CODE << 10)); }

void applyMode() {
  for (int c = 0; c < NUM_FDC; c++) if (fdc[c].ok) stopChip(c);
  for (int c = 0; c < MAXFDC; c++) { fdc[c].lastGoodMs = millis(); fdc[c].lastDoneMs = millis(); pending[c] = 0; }
  if (takingTurns()) {
    curChip = 0; startChip(0); curStartUs = micros(); lastCheckUs = curStartUs;
  } else {
    for (int c = 0; c < NUM_FDC; c++) if (chipActive(c)) startChip(c);
  }
}

void startQuickZero() {
  zeroing = true; zeroFromMs = millis() + 300; zeroToMs = zeroFromMs + 600;
  for (int c = 0; c < MAXFDC; c++) for (int ch = 0; ch < NCH; ch++) { zSum[c][ch] = 0; zN[c][ch] = 0; }
}
void finishQuickZero() {
  zeroing = false;
  for (int c = 0; c < NUM_FDC; c++) for (int ch = 0; ch < NCH; ch++)
    if (used(c, ch) && zN[c][ch] > 0) fdc[c].baselinePF[ch] = (float)(zSum[c][ch] / zN[c][ch]);
  if (Serial) Serial.println("# zero updated (insole should have been untouched)");
}

bool setupFDC(int c) {
  Fdc &f = fdc[c];
  uint16_t man, dev;
  hRestart[c]++;
  if (c == 1) softBusInit();
  if (!readReg(c, 0xFE, man) || !readReg(c, 0xFF, dev) || man != 0x5449 || dev != 0x1004) {
    if (Serial) {
      Serial.print("# FDC_"); Serial.print(FDC_NAME[c]);
      Serial.println(c == 0 ? " not found: check 3V3, GND, SDA = D4, SCL = D5"
                            : " not found: check 3V3, GND, SDA = D2, SCL = D3");
    }
    return false;
  }
  writeReg(c, 0x0C, 0x8000); delay(10);
  f.enable = 0; f.usedMask = 0; f.nUsed = 0;
  for (int ch = 0; ch < NCH; ch++) {
    if (!used(c, ch)) continue;
    f.enable |= (0x0080U >> ch);
    f.usedMask |= (1 << ch); f.nUsed++;
    if (!tuneOffset(c, ch)) {
      if (Serial) { Serial.print("#"); printPad(c, ch); Serial.println(": out of range (short to GND? SH pin on GND?)"); }
      f.capdac[ch] = 0; configureChannel(c, ch);
    }
  }
  for (int ch = 0; ch < NCH; ch++) {
    if (!used(c, ch)) continue;
    double sum = 0; int good = 0;
    for (int k = 0; k < BASELINE_SAMPLES; k++) {
      float r;
      if (readOneShot(c, ch, r) && fabsf(r) < VALID_PF) { sum += r + f.capdac[ch] * CAPDAC_STEP_PF; good++; }
    }
    f.baselinePF[ch] = good ? (float)(sum / good) : 0;
    f.latestPF[ch] = f.baselinePF[ch];
    f.histN[ch] = 0; f.histI[ch] = 0;
  }
  if (Serial) {
    Serial.print("# FDC_"); Serial.print(FDC_NAME[c]); Serial.print(" ready. Zero pF:");
    for (int ch = 0; ch < NCH; ch++) {
      if (!used(c, ch)) continue;
      printPad(c, ch); Serial.print('='); Serial.print(f.baselinePF[ch], 3);
      Serial.print("(dac "); Serial.print(f.capdac[ch]); Serial.print(')');
    }
    Serial.println();
  }
  f.lastGoodMs = millis(); f.lastDoneMs = millis();
  return true;
}
void stopAllChips() {
  for (int c = 0; c < NUM_FDC; c++) { uint16_t v; if (c == 1) softBusInit(); if (readReg(c, 0xFE, v)) writeReg(c, 0x0C, 0x8000); }
  delay(10);
}
void setupAll() { stopAllChips(); for (int c = 0; c < NUM_FDC; c++) fdc[c].ok = setupFDC(c); applyMode(); }
void restartChip(int c) {
  stopAllChips();
  fdc[c].ok = setupFDC(c);
  for (int k = 0; k < NUM_FDC; k++) {
    if (k == c || !fdc[k].ok) continue;
    for (int ch = 0; ch < NCH; ch++) if (used(k, ch)) configureChannel(k, ch);
  }
  applyMode();
}

void pollFDC(int c) {
  Fdc &f = fdc[c];
  uint16_t st;
  if (!readReg(c, 0x0C, st)) return;
  f.lastStatus = st;
  if (!(st & 0x0100)) {
    if (Serial) { Serial.print("# FDC_"); Serial.print(FDC_NAME[c]); Serial.print(" lost its settings (status 0x"); Serial.print(st, HEX); Serial.println("): restarting"); }
    restartChip(c); return;
  }
  bool anyDone = (st & 0x000F) != 0;
  if (anyDone) f.lastDoneMs = millis();
  bool readAll = !anyDone && millis() - f.lastDoneMs > 100;
  if (readAll && !f.warnedNoDone && Serial) { f.warnedNoDone = true; Serial.print("# note: FDC_"); Serial.print(FDC_NAME[c]); Serial.println(" shows no DONE flags, reading results directly"); }
  for (int ch = 0; ch < NCH; ch++) {
    if (!used(c, ch)) continue;
    if (!readAll && !(st & (0x0008U >> ch))) continue;
    float r;
    if (readResultPF(c, ch, r)) processResult(c, ch, r);
  }
}

void turnStep() {
  int cur = curChip, other = 1 - curChip;
  if (pending[other]) {
    int ch = 0; while (!(pending[other] & (1 << ch))) ch++;
    pending[other] &= ~(1 << ch);
    float r;
    if (readResultPF(other, ch, r)) processResult(other, ch, r);
    return;
  }
  uint32_t now = micros();
  if ((uint32_t)(now - curStartUs) < fdc[cur].nUsed * CONV_US + 150) return;
  if ((uint32_t)(now - lastCheckUs) < 150) return;
  lastCheckUs = now;
  uint16_t st;
  if (!readReg(cur, 0x0C, st)) return;
  fdc[cur].lastStatus = st;
  uint8_t done = 0;
  for (int ch = 0; ch < NCH; ch++) if (st & (0x0008U >> ch)) done |= (1 << ch);
  bool allDone = (done & fdc[cur].usedMask) == fdc[cur].usedMask;
  bool late = (uint32_t)(now - curStartUs) > 50000UL;
  if (!allDone && !late) return;
  if (late) hLate++;
  stopChip(cur);
  pending[cur] = allDone ? fdc[cur].usedMask : (done & fdc[cur].usedMask);
  if (allDone) fdc[cur].lastDoneMs = millis();
  curChip = other;
  startChip(other);
  curStartUs = micros(); lastCheckUs = curStartUs;
  hTurns++;
}

void maintain() {
  for (int c = 0; c < NUM_FDC; c++) {
    Fdc &f = fdc[c];
    if (!f.ok) {
      if (millis() - f.lastTry > 2000) { f.lastTry = millis(); restartChip(c); }
      continue;
    }
    if (!chipActive(c)) continue;
    if ((int32_t)(millis() - f.lastGoodMs) > 1000) {
      if (Serial) { Serial.print("# FDC_"); Serial.print(FDC_NAME[c]); Serial.print(" gave no valid readings for 1 s (status 0x"); Serial.print(f.lastStatus, HEX); Serial.println("): restarting"); }
      restartChip(c);
    }
  }
}

void resetHealth(uint32_t now) {
  for (int c = 0; c < MAXFDC; c++) { hErr[c] = 0; hRes[c] = 0; hOor[c] = 0; hRestart[c] = 0; }
  for (int s = 0; s < NSLOT; s++) { nSum[s] = 0; nSq[s] = 0; nCnt[s] = 0; }
  hTicks = 0; hTurns = 0; hLate = 0; lastHealthMs = now;
}
void printHealth(uint32_t now) {
  float secs = (now - lastHealthMs) / 1000.0f;
  if (secs <= 0) secs = 1;
  Serial.print("# health: output "); Serial.print(hTicks / secs, 0); Serial.print(" Hz (should be ~100), mode ");
  Serial.print(MODE_NAME[runMode]);
  if (takingTurns()) { Serial.print(", "); Serial.print(hTurns / secs / 2, 0); Serial.print(" full cycles/s"); if (hLate) { Serial.print(", "); Serial.print(hLate); Serial.print(" late"); } }
  if (!FAST_PINS && NUM_FDC == 2) Serial.print(", SLOW PINS for FDC B");
  for (int c = 0; c < NUM_FDC; c++) {
    Serial.print(" | FDC_"); Serial.print(FDC_NAME[c]);
    if (!fdc[c].ok) Serial.print(" OFFLINE");
    else if (!chipActive(c)) Serial.print(" stopped");
    Serial.print(": "); Serial.print(fdc[c].nUsed ? hRes[c] / secs / fdc[c].nUsed : 0, 0); Serial.print(" readings/s per pad, ");
    Serial.print(hErr[c]); Serial.print(" I2C errors, ");
    Serial.print(hOor[c]); Serial.print(" out-of-range, ");
    Serial.print(hRestart[c]); Serial.print(" restarts");
  }
  Serial.println();
  Serial.print("# noise (wobble in fF while untouched, good < ~20):");
  for (int c = 0; c < NUM_FDC; c++) for (int ch = 0; ch < NCH; ch++) {
    if (!used(c, ch)) continue;
    int s = fdc[c].slot[ch];
    Serial.print(' '); Serial.print(SLOT_SHORT[s]); Serial.print('=');
    if (nCnt[s] < 2 || !chipActive(c)) { Serial.print('-'); continue; }
    double m = nSum[s] / nCnt[s];
    double var = nSq[s] / nCnt[s] - m * m;
    Serial.print(var > 0 ? sqrt(var) * 1000.0 : 0.0, 0);
  }
  Serial.println();
  resetHealth(now);
}

void putU16(uint8_t* p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
void putU32(uint8_t* p, uint32_t v) { p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; p[2] = (v >> 16) & 0xff; p[3] = (v >> 24) & 0xff; }

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) delay(10);
  for (int c = 0; c < MAXFDC; c++) {
    memset(&fdc[c], 0, sizeof(Fdc));
    for (int ch = 0; ch < NCH; ch++) fdc[c].slot[ch] = (NUM_FDC == 2) ? SLOTS_2FDC[c][ch] : (c == 0 ? SLOTS_1FDC[0][ch] : -1);
  }
  Wire.begin();                                          // XIAO: SDA = D4, SCL = D5
  Wire.setClock(I2C_HZ);
  if (Serial) { Serial.print("# Kynetix v8 (one FDC, 4 pads, plotter labels), NUM_FDC = "); Serial.print(NUM_FDC); Serial.print(", mode "); Serial.println(MODE_NAME[runMode]); }
  setupAll();
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
  applyMode();
  nextSample = micros(); nextPoll = micros();
  resetHealth(millis());
}

void loop() {
  BLE.poll();
  while (Serial && Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == 'b' || ch == 'B') { setupAll(); resetHealth(millis()); }   // full restart + re-zero (insole unloaded!)
    if (ch == 'z' || ch == 'Z') { startQuickZero(); Serial.println("# quick re-zero: keep the insole untouched for 1 s"); }
    if (ch == 'd' || ch == 'D') { diag = !diag; Serial.println(diag ? "# diagnostics ON" : "# diagnostics OFF"); }
    if (ch == 'p' || ch == 'P') { plotMode = !plotMode; Serial.println(plotMode ? "# plotter output" : "# dashboard output (CSV)"); }
  }

  // read the chip(s)
  if (takingTurns()) turnStep();
  else if ((int32_t)(micros() - nextPoll) >= 0) {
    nextPoll = micros() + POLL_US;
    for (int c = 0; c < NUM_FDC; c++) if (chipActive(c)) pollFDC(c);
  }
  maintain();
  if (zeroing && millis() >= zeroToMs) finishQuickZero();

  // output at 100 Hz
  const uint32_t nowUS = micros();
  if ((int32_t)(nowUS - nextSample) < 0) return;
  if ((uint32_t)(nowUS - nextSample) > 50000UL) nextSample = nowUS;
  nextSample += PERIOD_US;
  const uint32_t timestamp = millis();
  hTicks++;

  // change in capacitance per dashboard slot, in femtofarads (fF)
  uint16_t values[NSLOT] = {0};
  uint32_t total = 0;
  for (int c = 0; c < NUM_FDC; c++) {
    if (!chipActive(c)) continue;
    for (int ch = 0; ch < NCH; ch++) {
      if (!used(c, ch)) continue;
      int s = fdc[c].slot[ch];
      nSum[s] += fdc[c].latestPF[ch]; nSq[s] += (double)fdc[c].latestPF[ch] * fdc[c].latestPF[ch]; nCnt[s]++;
      float dPF = fdc[c].latestPF[ch] - fdc[c].baselinePF[ch];
      if (dPF < 0 && !zeroing) fdc[c].baselinePF[ch] += ZERO_FOLLOW_DOWN * dPF;   // zero follows drift DOWN
      float dfF = dPF * 1000.0f;
      uint16_t v = (uint16_t)constrain((long)lroundf(dfF), 0L, 65535L);
      values[s] = v;
      total += v;
    }
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

  if (diag && Serial && timestamp - lastDiagMs >= 100) {  // signed change per pad, in pF
    lastDiagMs = timestamp;
    Serial.print("# dpF");
    for (int c = 0; c < NUM_FDC; c++) {
      for (int ch = 0; ch < NCH; ch++) {
        if (!used(c, ch)) continue;
        printPad(c, ch); Serial.print('=');
        if (!chipActive(c)) { Serial.print("off"); continue; }
        float d = fdc[c].latestPF[ch] - fdc[c].baselinePF[ch];
        if (d >= 0) Serial.print('+');
        Serial.print(d, 3);
      }
    }
    Serial.println();
  }
  if (Serial && !diag && plotMode && (hTicks % PLOT_DIV == 0)) {   // labelled lines for the Serial Plotter
    bool first = true;
    for (int c = 0; c < NUM_FDC; c++) for (int ch = 0; ch < NCH; ch++) {
      if (!used(c, ch) || !chipActive(c)) continue;
      if (!first) Serial.print(',');
      first = false;
      Serial.print(PLOT_LABEL[fdc[c].slot[ch]]); Serial.print(':');
      Serial.print((long)lroundf((fdc[c].latestPF[ch] - fdc[c].baselinePF[ch]) * 1000.0f));
    }
    Serial.println();
  }
  if (Serial && !diag && !plotMode) {                           // dashboard CSV
    Serial.print(timestamp);
    for (int i = 0; i < NSLOT; i++) { Serial.print(','); Serial.print(values[i]); }
    Serial.print(','); Serial.println(inContact ? 1 : 0);
  }
  if (timestamp - lastHealthMs >= HEALTH_MS) {
    if (Serial && !plotMode) printHealth(timestamp);
    else resetHealth(timestamp);
  }
}