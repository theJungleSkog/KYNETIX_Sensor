/* KYNETIX | BARE ELECTRODE SHEET TEST v1 | XIAO nRF52840 + ONE FDC1004 (hardware I2C)
   Purpose: check the electrode sheet by itself with NOTHING automatic in the code.
     - No auto-ranging while running (CAPDAC is set once at zero, then fixed)
     - No auto-restart, no auto re-zero, no zero drift following, no spike filter
   So every jump you see is what the chip actually measured.

   WIRING: same as the current one-FDC setup
     XIAO 3V3 -> FDC 3V3   XIAO GND -> FDC GND   XIAO D4 -> FDC SDA   XIAO D5 -> FDC SCL
     Tail sensor pins -> CIN1..CIN4, tail shield pins -> SHLD1, tail GND pins -> FDC GND

   OUTPUT (Serial Plotter, 115200): 5 labelled lines, about 11 times per second
     CIN1_BigToe, CIN2_Heel, CIN3_1stMet, CIN4_5thMet = change since zero, in fF (1000 fF = 1 pF)
     OVERLOAD = 0 normally. Jumps to 15000 when ANY pad is beyond the chip's range
                (more than about 15 pF change). Overloaded pads are pinned at +/-15000.

   COMMAND (type in the box at the bottom + Enter):
     z = re-zero (do NOT touch the sheet for ~1 s)

   START-UP: don't touch the sheet for ~2 s after power-on/reset.
*/
#include <Wire.h>

// ---------------- settings ----------------
const char* PAD_LABEL[4] = {"CIN1_BigToe", "CIN2_Heel", "CIN3_1stMet", "CIN4_5thMet"};  // what each CIN is wired to
const uint8_t RATE_CODE = 1;          // 1 = 100 S/s (quietest), 2 = 200 S/s, 3 = 400 S/s
const int ZERO_SAMPLES = 20;          // readings averaged per pad when zeroing
const uint32_t LOOP_DELAY_MS = 50;    // slows the plot down so ~4-5 s fit on the screen
// -------------------------------------------

const uint8_t FDC_ADDR = 0x50;
const float COUNTS_PER_PF = 524288.0f;   // 2^19 counts per pF
const float CAPDAC_STEP_PF = 3.125f;     // each CAPDAC step = 3.125 pF offset
const float OVERLOAD_PF = 14.9f;         // chip reads only about +/-15 pF around the CAPDAC offset

uint8_t capdac[4] = {0, 0, 0, 0};
float baselinePF[4] = {0, 0, 0, 0};
bool zeroOK[4] = {false, false, false, false};

// ---------- I2C ----------
bool writeReg(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(FDC_ADDR);
  Wire.write(reg); Wire.write((uint8_t)(v >> 8)); Wire.write((uint8_t)v);
  return Wire.endTransmission() == 0;
}
bool readReg(uint8_t reg, uint16_t &v) {
  Wire.beginTransmission(FDC_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(true) != 0) return false;
  if (Wire.requestFrom(FDC_ADDR, (uint8_t)2) != 2) return false;
  uint16_t hi = (uint8_t)Wire.read(), lo = (uint8_t)Wire.read();
  v = (hi << 8) | lo;
  return true;
}

// One measurement of input `in` (0..3 = CIN1..CIN4) with the current CAPDAC.
// Returns the reading in pF relative to the CAPDAC offset (valid range about +/-15), or NAN on I2C failure.
float measureOnce(int in) {
  if (!writeReg(0x08, ((uint16_t)in << 13) | (4U << 10) | ((uint16_t)capdac[in] << 5))) return NAN;
  if (!writeReg(0x0C, ((uint16_t)RATE_CODE << 10) | 0x0080)) return NAN;   // start MEAS1 once
  uint32_t t0 = millis();
  uint16_t st = 0;
  do {
    if (!readReg(0x0C, st)) return NAN;
    if (millis() - t0 > 50) return NAN;
    delay(1);
  } while (!(st & 0x0008));                                                  // DONE_1
  uint16_t msb, lsb;
  if (!readReg(0x00, msb) || !readReg(0x01, lsb)) return NAN;
  uint32_t bits = ((uint32_t)msb << 8) | (lsb >> 8);
  int32_t raw = (bits & 0x800000UL) ? (int32_t)bits - 16777216L : (int32_t)bits;
  return raw / COUNTS_PER_PF;
}

// Zero all 4 pads: pick a CAPDAC so each reading sits near 0, then average ZERO_SAMPLES readings.
void zeroAll() {
  Serial.println("# zeroing... don't touch the sheet");
  delay(300);
  for (int in = 0; in < 4; in++) {
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
  uint16_t man = 0, dev = 0;
  if (!readReg(0xFE, man) || !readReg(0xFF, dev) || man != 0x5449 || dev != 0x1004) {
    Serial.println("# FDC1004 NOT found on D4/D5: check 3V3, GND, SDA = D4, SCL = D5, then press reset");
  } else {
    Serial.println("# FDC1004 found");
  }
  writeReg(0x0C, 0x8000);   // reset the chip
  delay(10);
  zeroAll();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'z' || ch == 'Z') zeroAll();
  }

  long outFF[4];
  bool anyOverload = false;
  for (int in = 0; in < 4; in++) {
    float r = measureOnce(in);
    if (isnan(r) || !zeroOK[in]) { outFF[in] = 0; continue; }
    if (fabsf(r) >= OVERLOAD_PF) {                        // beyond the chip's range: pin it, flag it
      anyOverload = true;
      outFF[in] = (r > 0) ? 15000 : -15000;
    } else {
      outFF[in] = lroundf((r - baselinePF[in]) * 1000.0f);
    }
  }

  for (int in = 0; in < 4; in++) {
    Serial.print(PAD_LABEL[in]); Serial.print(':'); Serial.print(outFF[in]); Serial.print(',');
  }
  Serial.print("OVERLOAD:"); Serial.println(anyOverload ? 15000 : 0);

  delay(LOOP_DELAY_MS);
}