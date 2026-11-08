/* KYNETIX | XIAO nRF52840 Sense | V6 flex insole (5 pads) | FDC1004 + BLE + USB
   Same Bluetooth name, same 16-byte packet and same USB CSV (t_ms,S1,S2,S3,S4,S5,contact)
   as the 4-capacitor sketch, so the Kynetix dashboards keep working.

   Board package: Seeed nRF52 mbed-enabled Boards ("XIAO nRF52840 Sense (No Updates)")
   Library: ArduinoBLE (Library Manager). Wire is built in.

   ---------------------------------------------------------------------------------
   TWO WAYS TO RUN IT (set NUM_FDC below)
   ---------------------------------------------------------------------------------
   One FDC1004 has only 4 inputs (CIN1..CIN4), so it can read 4 of the 5 flex pads.

   NUM_FDC = 1  (one FDC1004, 4 pads; lateral midfoot is not read and is sent as 0)
     FDC "A" on the normal I2C pins:  SDA -> XIAO D4,  SCL -> XIAO D5
       CIN1 <- tail pin 13  (flex S1, heel)
       CIN2 <- tail pin 3   (flex S4, 1st met head)
       CIN3 <- tail pin 8   (flex S3, 5th met head)
       CIN4 <- tail pin 6   (flex S5, big toe)
       SHLD1 <- tail pins 2, 5 AND 10 (all three shield nets: SH_C, SH_B, SH_A)

   NUM_FDC = 2  (add a second FDC1004 "B", all 5 pads)
     FDC "A" (SDA D4, SCL D5):
       CIN1  (nothing)
       CIN2 <- tail pin 3   (flex S4, 1st met)
       CIN3 <- tail pin 8   (flex S3, 5th met)
       CIN4 <- tail pin 6   (flex S5, big toe)
       SHLD1 <- tail pins 2 and 5 (SH_C, SH_B)
     FDC "B" on its own 2 wires (both FDC1004s have the same fixed address 0x50, so they
     can't share D4/D5):  SDA -> XIAO D2,  SCL -> XIAO D3
       CIN1 <- tail pin 13  (flex S1, heel)          <- moved here from FDC A
       CIN2 <- tail pin 11  (flex S2, lateral midfoot)
       SHLD1 <- tail pin 10 (SH_A, the shield that runs beside S1 and S2)

   Both modes: tail pin 1 and pin 15 -> GND.  3V3/GND of every FDC1004 -> XIAO 3V3/GND.
   NEVER connect an SH pin to GND: the shields on the flex are DRIVEN by the FDC1004.
   The ground lid must be tied to GND through its ear (jumper hole next to the tail).

   UNITS: each value is the CHANGE IN CAPACITANCE in femtofarads (fF): 1000 fF = 1 pF.

   PACKET / CSV SLOTS (dashboard numbering, NOT the flex numbering)
     slot 0 (CSV "S1") = big toe        = flex S5
     slot 1 (CSV "S2") = lateral midfoot = flex S2
     slot 2 (CSV "S3") = 5th met        = flex S3
     slot 3 (CSV "S4") = 1st met        = flex S4
     slot 4 (CSV "S5") = heel           = flex S1

   START-UP
     Keep the insole UNLOADED for ~2 s after power-on/reset (offset tuning + zero).
     Serial Monitor (115200): b + Enter = re-zero.  d + Enter = diagnostic line 10x per
     second with each pad's SIGNED change in pF.  Lines starting with # are status.
*/
#include <ArduinoBLE.h>
#include <Wire.h>
#include <math.h>

// ---------------- user settings ----------------
const int NUM_FDC = 2;                       // 1 = one FDC1004 (4 pads). 2 = two FDC1004s (5 pads)
const uint32_t I2C_HZ = 400000;              // FDC A bus. Use 100000 if wires are long or readings drop out
const int SOFT_SDA = D2, SOFT_SCL = D3;      // FDC B bus (software I2C), only used when NUM_FDC = 2
// slot of each CIN (CIN1..CIN4) for each FDC1004, -1 = nothing connected
const int8_t SLOTS_1FDC[1][4] = { {4, 3, 2, 0} };                     // A: heel, 1st met, 5th met, big toe
const int8_t SLOTS_2FDC[2][4] = { {-1, 3, 2, 0},                      // A: -, 1st met, 5th met, big toe
                                  { 4, 1, -1, -1} };                  // B: heel, lateral midfoot
// ------------------------------------------------

const int NCH = 4, NSLOT = 5, MAXFDC = 2;
const char* SLOT_NAME[NSLOT] = {"bigtoe(flexS5)", "latmid(flexS2)", "5thmet(flexS3)", "1stmet(flexS4)", "heel(flexS1)"};
const uint8_t FDC_ADDR = 0x50;
const uint8_t RATE_CODE = 3;                 // 400 S/s per measurement
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

struct Fdc {
  bool ok;
  int8_t slot[NCH];
  uint8_t capdac[NCH];
  float baselinePF[NCH], latestPF[NCH];
  float hist[NCH][3];
  uint8_t histN[NCH], histI[NCH];
  uint32_t lastGoodMs, lastDoneMs, lastTry;
  uint16_t lastStatus;
  bool warnedNoDone;
};
Fdc fdc[MAXFDC];
const char FDC_NAME[MAXFDC] = {'A', 'B'};

bool diag = false;
uint32_t nextSample = 0, lastDiagMs = 0;
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

// ---------------- software I2C (FDC B) ----------------
// Open-drain: a pin is "released" (input with pull-up) for 1 and driven LOW for 0.
inline void bitDelay() { delayMicroseconds(2); }   // pin switching adds a few us, ~50-100 kHz overall
inline void sdaRelease() { pinMode(SOFT_SDA, INPUT_PULLUP); }
inline void sdaLow() { pinMode(SOFT_SDA, OUTPUT); digitalWrite(SOFT_SDA, LOW); }
inline void sclLow() { pinMode(SOFT_SCL, OUTPUT); digitalWrite(SOFT_SCL, LOW); }
inline void sclRelease() {
  pinMode(SOFT_SCL, INPUT_PULLUP);
  uint32_t t0 = micros();
  while (!digitalRead(SOFT_SCL) && micros() - t0 < 1000) {}   // allow clock stretching
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
  bool ack = digitalRead(SOFT_SDA) == LOW;
  sclLow(); bitDelay();
  return ack;
}
uint8_t softReadByte(bool ack) {
  uint8_t b = 0;
  sdaRelease();
  for (int i = 0; i < 8; i++) {
    bitDelay(); sclRelease(); bitDelay();
    b = (b << 1) | (digitalRead(SOFT_SDA) ? 1 : 0);
    sclLow();
  }
  if (ack) sdaLow(); else sdaRelease();
  bitDelay(); sclRelease(); bitDelay(); sclLow(); bitDelay();
  sdaRelease();
  return b;
}
void softBusInit() {                       // free a stuck bus: 9 clocks with SDA released, then STOP
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
  softStart();                                         // repeated start
  if (!softWriteByte((FDC_ADDR << 1) | 1)) { softStop(); return false; }
  uint16_t hi = softReadByte(true), lo = softReadByte(false);
  softStop();
  value = (hi << 8) | lo;
  return true;
}

// ---------------- FDC1004 low level (c = 0 -> FDC A on Wire, 1 -> FDC B on soft I2C) ----------------
bool writeReg(int c, uint8_t reg, uint16_t value) {
  if (c == 1) return softWriteReg(reg, value);
  Wire.beginTransmission(FDC_ADDR);
  Wire.write(reg); Wire.write((uint8_t)(value >> 8)); Wire.write((uint8_t)value);
  return Wire.endTransmission() == 0;
}
bool readReg(int c, uint8_t reg, uint16_t &value) {
  if (c == 1) return softReadReg(reg, value);
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
bool configureChannel(int c, int ch) {
  return writeReg(c, 0x08 + ch, ((uint16_t)ch << 13) | (4U << 10) | ((uint16_t)fdc[c].capdac[ch] << 5));
}
bool readResultPF(int c, int ch, float &residualPF) {
  uint16_t msb, lsb;
  if (!readReg(c, 0x00 + 2 * ch, msb) || !readReg(c, 0x01 + 2 * ch, lsb)) return false;
  residualPF = decodeResult(msb, lsb) / COUNTS_PER_PF;
  return isfinite(residualPF);
}
bool readOneShot(int c, int ch, float &residualPF) {     // single measurement (used during setup)
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
bool tuneOffset(int c, int ch) {                         // pick the CAPDAC that centres the reading
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
void rerange(int c, int ch, float residualPF) {          // follow large changes (hand on a wire, etc.)
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

// Full start-up of one FDC1004: detect, tune each used channel, take the zero, start continuous mode.
bool setupFDC(int c) {
  Fdc &f = fdc[c];
  uint16_t man, dev;
  if (c == 1) softBusInit();
  if (!readReg(c, 0xFE, man) || !readReg(c, 0xFF, dev) || man != 0x5449 || dev != 0x1004) {
    if (Serial) {
      Serial.print("# FDC_"); Serial.print(FDC_NAME[c]);
      Serial.println(c == 0 ? " not found: check 3V3, GND, SDA = D4, SCL = D5"
                            : " not found: check 3V3, GND, SDA = D2, SCL = D3");
    }
    return false;
  }
  writeReg(c, 0x0C, 0x8000); delay(10);                   // reset the chip
  uint16_t enable = 0;
  for (int ch = 0; ch < NCH; ch++) {
    if (!used(c, ch)) continue;
    enable |= (0x0080U >> ch);
    if (!tuneOffset(c, ch)) {
      if (Serial) { Serial.print("#"); printPad(c, ch); Serial.println(": out of range (short to GND? SH pin on GND?)"); }
      f.capdac[ch] = 0; configureChannel(c, ch);
    }
  }
  for (int ch = 0; ch < NCH; ch++) {                      // zero = average with sensors unloaded
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
  // continuous mode: the used measurements, repeating
  if (!writeReg(c, 0x0C, ((uint16_t)RATE_CODE << 10) | 0x0100 | enable)) return false;
  if (Serial) {
    Serial.print("# FDC_"); Serial.print(FDC_NAME[c]); Serial.print(" ready. Zero pF:");
    for (int ch = 0; ch < NCH; ch++) {
      if (!used(c, ch)) continue;
      printPad(c, ch); Serial.print('='); Serial.print(f.baselinePF[ch], 3);
    }
    Serial.println();
  }
  f.lastGoodMs = millis(); f.lastDoneMs = millis();
  return true;
}
void setupAll() { for (int c = 0; c < NUM_FDC; c++) fdc[c].ok = setupFDC(c); }

// Read whichever channels of FDC c have a new result.
void pollFDC(int c) {
  Fdc &f = fdc[c];
  uint16_t st;
  if (!readReg(c, 0x0C, st)) return;
  f.lastStatus = st;
  if (!(st & 0x0100)) {                                  // chip lost its settings (power dip)
    if (Serial) { Serial.print("# FDC_"); Serial.print(FDC_NAME[c]); Serial.print(" lost its settings (status 0x"); Serial.print(st, HEX); Serial.println("): restarting"); }
    f.ok = setupFDC(c); return;
  }
  bool anyDone = (st & 0x000F) != 0;
  if (anyDone) f.lastDoneMs = millis();
  bool readAll = !anyDone && millis() - f.lastDoneMs > 100;   // fallback if DONE flags never show
  if (readAll && !f.warnedNoDone && Serial) { f.warnedNoDone = true; Serial.print("# note: FDC_"); Serial.print(FDC_NAME[c]); Serial.println(" shows no DONE flags, reading results directly"); }
  for (int ch = 0; ch < NCH; ch++) {
    if (!used(c, ch)) continue;
    if (!readAll && !(st & (0x0008U >> ch))) continue;
    float r;
    if (!readResultPF(c, ch, r)) continue;
    if (fabsf(r) < VALID_PF) {
      float pf = r + f.capdac[ch] * CAPDAC_STEP_PF;
      f.hist[ch][f.histI[ch]] = pf; f.histI[ch] = (f.histI[ch] + 1) % 3; if (f.histN[ch] < 3) f.histN[ch]++;
      f.latestPF[ch] = f.histN[ch] < 3 ? pf : median3(f.hist[ch][0], f.hist[ch][1], f.hist[ch][2]);  // drops 1-sample spikes
      f.lastGoodMs = millis();
      if (fabsf(r) > RERANGE_PF) rerange(c, ch, r);
    } else {
      rerange(c, ch, r > 0 ? 4 * CAPDAC_STEP_PF : -4 * CAPDAC_STEP_PF);
    }
  }
}

void putU16(uint8_t* p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
void putU32(uint8_t* p, uint32_t v) { p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; p[2] = (v >> 16) & 0xff; p[3] = (v >> 24) & 0xff; }

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) delay(10);     // give the Serial Monitor a moment (not required)
  for (int c = 0; c < MAXFDC; c++) {
    memset(&fdc[c], 0, sizeof(Fdc));
    for (int ch = 0; ch < NCH; ch++) fdc[c].slot[ch] = (NUM_FDC == 2) ? SLOTS_2FDC[c][ch] : (c == 0 ? SLOTS_1FDC[0][ch] : -1);
  }
  Wire.begin();                                          // XIAO: SDA = D4, SCL = D5
  Wire.setClock(I2C_HZ);
  if (Serial) { Serial.print("# Kynetix flex, NUM_FDC = "); Serial.println(NUM_FDC); }
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
  nextSample = micros();
  for (int c = 0; c < NUM_FDC; c++) { fdc[c].lastGoodMs = millis(); fdc[c].lastDoneMs = millis(); }  // BLE start-up isn't a dropout
}

void loop() {
  BLE.poll();
  while (Serial && Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == 'b' || ch == 'B') setupAll();              // re-zero (insole unloaded!)
    if (ch == 'd' || ch == 'D') { diag = !diag; Serial.println(diag ? "# diagnostics ON" : "# diagnostics OFF"); }
  }
  const uint32_t nowUS = micros();
  if ((int32_t)(nowUS - nextSample) < 0) return;
  if ((uint32_t)(nowUS - nextSample) > 50000UL) nextSample = nowUS;
  nextSample += PERIOD_US;
  const uint32_t timestamp = millis();

  for (int c = 0; c < NUM_FDC; c++) {
    Fdc &f = fdc[c];
    if (!f.ok) {
      if (timestamp - f.lastTry > 2000) { f.lastTry = timestamp; f.ok = setupFDC(c); }
    } else {
      pollFDC(c);
      if (f.ok && (int32_t)(millis() - f.lastGoodMs) > 1000) {   // no valid data for 1 s: restart that chip
        if (Serial) { Serial.print("# FDC_"); Serial.print(FDC_NAME[c]); Serial.print(" gave no valid readings for 1 s (status 0x"); Serial.print(f.lastStatus, HEX); Serial.println("): restarting"); }
        f.ok = setupFDC(c);
      }
    }
  }

  // change in capacitance per dashboard slot, in femtofarads (fF)
  uint16_t values[NSLOT] = {0};
  uint32_t total = 0;
  for (int c = 0; c < NUM_FDC; c++) {
    if (!fdc[c].ok) continue;
    for (int ch = 0; ch < NCH; ch++) {
      if (!used(c, ch)) continue;
      float dfF = (fdc[c].latestPF[ch] - fdc[c].baselinePF[ch]) * 1000.0f;
      uint16_t v = (uint16_t)constrain((long)lroundf(dfF), 0L, 65535L);
      values[fdc[c].slot[ch]] = v;
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
        if (!fdc[c].ok) { Serial.print("offline"); continue; }
        float d = fdc[c].latestPF[ch] - fdc[c].baselinePF[ch];
        if (d >= 0) Serial.print('+');
        Serial.print(d, 3);
        Serial.print("(dac "); Serial.print(fdc[c].capdac[ch]); Serial.print(')');
      }
    }
    Serial.println();
  }
  if (Serial && !diag) {
    Serial.print(timestamp);
    for (int i = 0; i < NSLOT; i++) { Serial.print(','); Serial.print(values[i]); }
    Serial.print(','); Serial.println(inContact ? 1 : 0);
  }
}
