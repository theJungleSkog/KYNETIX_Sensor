/* KYNETIX | FLEX PCB 5-PAD TEST | XIAO nRF52840 Sense | Seeed mbed 2.9.3
   Based on the original 11_16_26_Flex_PCB.ino with fixed CAPDAC and no filtering.

   Existing FDC #1: D4->SDA, D5->SCL, 3V3->3V3, GND->GND;
                    electrode pads 1..4 -> CIN1..CIN4.
   New FDC #2:      D2->SDA, D3->SCL, 3V3->3V3, GND->GND;
                    fifth electrode -> CIN1.
   Ground lid -> GND; do not connect a shield output directly to ground.
   Each independent SDA/SCL pair needs pull-ups to 3.3V (often on FDC breakout).
   Do not power the FDC boards from 5V.

   Serial Plotter at 115200: 5 sensor changes (fF) and OVERLOAD.
   Serial Monitor command 'z' re-zeros pads unloaded.
   Sensor 5's position is unknown: rename 'CIN1_Pad5' after tracing the tail.
   Verify boot messages show BOTH FDCs found before pressure tests.
   NOTE: Some board/core combinations may not support a second custom I2C
   instance. If either FDC disappears, test each bus separately first.
*/
#include <Wire.h>
#include <math.h>

// Second, separately wired I2C interface (mbed core; SDA D2, SCL D3).
// Avoid Wire1, which is assigned to the Sense board IMU.
TwoWire secondBus(D2, D3);
TwoWire* const BUS[2] = {&Wire, &secondBus};
const uint8_t N_PADS = 5;
const uint8_t PAD_FDC[N_PADS] = {0, 0, 0, 0, 1};
const uint8_t PAD_CIN[N_PADS] = {0, 1, 2, 3, 0};
bool fdcPresent[2] = {false, false};

// ---------------- settings ----------------
const char* PAD_LABEL[N_PADS] = {"CIN1_BigToe", "CIN2_Heel", "CIN3_1stMet", "CIN4_5thMet", "CIN1_Pad5"};  // what each CIN is wired to
const uint8_t RATE_CODE = 1;          // 1 = 100 S/s (quietest), 2 = 200 S/s, 3 = 400 S/s
const int ZERO_SAMPLES = 20;          // readings averaged per pad when zeroing
const uint32_t LOOP_DELAY_MS = 50;    // slows the plot down so ~4-5 s fit on the screen
// -------------------------------------------

const uint8_t FDC_ADDR = 0x50;
const float COUNTS_PER_PF = 524288.0f;   // 2^19 counts per pF
const float CAPDAC_STEP_PF = 3.125f;     // each CAPDAC step = 3.125 pF offset
const float OVERLOAD_PF = 14.9f;         // chip reads only about +/-15 pF around the CAPDAC offset

uint8_t capdac[N_PADS] = {0};
float baselinePF[N_PADS] = {0};
bool zeroOK[N_PADS] = {false};

// ---------- I2C ----------
bool writeReg(uint8_t device, uint8_t reg, uint16_t v) {
  TwoWire &w = *BUS[device];
  w.beginTransmission(FDC_ADDR);
  w.write(reg); w.write((uint8_t)(v >> 8)); w.write((uint8_t)v);
  return w.endTransmission() == 0;
}
bool readReg(uint8_t device, uint8_t reg, uint16_t &v) {
  TwoWire &w = *BUS[device];
  w.beginTransmission(FDC_ADDR);
  w.write(reg);
  if (w.endTransmission(true) != 0) return false;
  if (w.requestFrom(FDC_ADDR, (uint8_t)2) != 2) return false;
  uint16_t hi = (uint8_t)w.read(), lo = (uint8_t)w.read();
  v = (hi << 8) | lo;
  return true;
}

// One measurement of input `in` (0..3 = CIN1..CIN4) with the current CAPDAC.
// Returns the reading in pF relative to the CAPDAC offset (valid range about +/-15), or NAN on I2C failure.
float measureOnce(int pad) {
  uint8_t device = PAD_FDC[pad];
  int in = PAD_CIN[pad];
  if (!fdcPresent[device]) return NAN;
  if (!writeReg(device, 0x08, ((uint16_t)in << 13) | (4U << 10) | ((uint16_t)capdac[pad] << 5))) return NAN;
  if (!writeReg(device, 0x0C, ((uint16_t)RATE_CODE << 10) | 0x0080)) return NAN;   // start MEAS1 once
  uint32_t t0 = millis();
  uint16_t st = 0;
  do {
    if (!readReg(device, 0x0C, st)) return NAN;
    if (millis() - t0 > 50) return NAN;
    delay(1);
  } while (!(st & 0x0008));                                                  // DONE_1
  uint16_t msb, lsb;
  if (!readReg(device, 0x00, msb) || !readReg(device, 0x01, lsb)) return NAN;
  uint32_t bits = ((uint32_t)msb << 8) | (lsb >> 8);
  int32_t raw = (bits & 0x800000UL) ? (int32_t)bits - 16777216L : (int32_t)bits;
  return raw / COUNTS_PER_PF;
}

// Zero all 5 pads: pick a CAPDAC so each reading sits near 0, then average ZERO_SAMPLES readings.
void zeroAll() {
  Serial.println("# zeroing... don't touch the sheet");
  delay(300);
  for (int in = 0; in < N_PADS; in++) {
    zeroOK[in] = false;
    for (int tries = 0; tries < 12; tries++) {           // find the CAPDAC offset
      float r = measureOnce(in);
      if (isnan(r)) break;
      int steps = (int)lroundf(r / CAPDAC_STEP_PF);
      if (fabsf(r) > OVERLOAD_PF) steps = (r > 0) ? 4 : -4;
      if (steps == 0) break;
      int v = (int)capdac[in] + steps;
      if (v < 0) v = 0;
      if (v > 31) v = 31;
      if (v == capdac[in]) break;
      capdac[in] = (uint8_t)v;
    }
    double sum = 0; int good = 0;
    for (int k = 0; k < ZERO_SAMPLES; k++) {
      float r = measureOnce(in);
      if (!isnan(r) && fabsf(r) < OVERLOAD_PF) { sum += r; good++; }
    }
    if (good > ZERO_SAMPLES / 2) {
      baselinePF[in] = (float)(sum / good);
      zeroOK[in] = true;
    }
    Serial.print("# "); Serial.print(PAD_LABEL[in]);
    if (zeroOK[in]) {
      Serial.print(" resting capacitance = ");
      Serial.print(baselinePF[in] + capdac[in] * CAPDAC_STEP_PF, 2);
      Serial.println(" pF");
    } else {
      Serial.println(" could NOT zero (no reading or out of range): check this pad's wire / tail pin");
    }
  }
  Serial.println("# zero done");
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  Wire.begin();
  Wire.setClock(400000);
  secondBus.begin();
  secondBus.setClock(400000);
  for (uint8_t d = 0; d < 2; d++) {
    uint16_t man = 0, dev = 0;
    fdcPresent[d] = readReg(d, 0xFE, man) && readReg(d, 0xFF, dev)
                 && man == 0x5449 && dev == 0x1004;
    Serial.print("# FDC #"); Serial.print(d + 1);
    if (fdcPresent[d]) {
      Serial.println(" found (TI 0x5449, FDC1004 0x1004)");
      writeReg(d, 0x0C, 0x8000);
    } else {
      Serial.println(" NOT found: check 3V3, GND, SDA/SCL and pull-ups");
    }
  }
  delay(10);
  zeroAll();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'z' || ch == 'Z') zeroAll();
  }

  long outFF[N_PADS];
  bool anyOverload = false;
  for (int in = 0; in < N_PADS; in++) {
    float r = measureOnce(in);
    if (isnan(r) || !zeroOK[in]) { outFF[in] = 0; continue; }
    if (fabsf(r) >= OVERLOAD_PF) {                        // beyond the chip's range: pin it, flag it
      anyOverload = true;
      outFF[in] = (r > 0) ? 15000 : -15000;
    } else {
      outFF[in] = lroundf((r - baselinePF[in]) * 1000.0f);
    }
  }

  for (int in = 0; in < N_PADS; in++) {
    Serial.print(PAD_LABEL[in]); Serial.print(':'); Serial.print(outFF[in]); Serial.print(',');
  }
  Serial.print("OVERLOAD:"); Serial.println(anyOverload ? 15000 : 0);

  delay(LOOP_DELAY_MS);
}
