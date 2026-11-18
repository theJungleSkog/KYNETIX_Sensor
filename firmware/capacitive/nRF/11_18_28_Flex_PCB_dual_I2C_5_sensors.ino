/* KYNETIX | FLEX PCB 5 PADS, 2x FDC1004 | XIAO nRF52840 Sense | Seeed mbed 2.9.3 | FIRMWARE v3
   = your working 5-pad test code + the output the Run Lab dashboard expects
     (11_11_26_Kynetix_Flex5_Running_Dashboard_2.html). No dashboard changes needed.

   WIRING (shield groups split by chip)
     FDC #1: D4->SDA, D5->SCL, 3V3->3V3, GND->GND
             CIN1 <- tail 6 (big toe)  CIN2 <- tail 3 (1st met)  CIN3 <- tail 8 (5th met)  CIN4 unused
             SHLD1 <- tail 2 + 5 (SH_C, SH_B)
     FDC #2: D2->SDA, D3->SCL, 3V3->3V3, GND->GND
             CIN1 = lateral midfoot, CIN2 = heel  (measured by pressing, 11/18)
             SHLD1 <- tail 14 (SH_A)
     Ground lid -> GND. Shield pins -> SHLD1, never GND, never join the two SHLD1s. Never power the FDCs from 5V.

   LIBRARY: ArduinoBLE (Library Manager).

   TWO OUTPUT STYLES (type p + Enter in the Serial Monitor to switch; it re-zeros, so hands off):
     PLOTTER (default, plotMode = true): 5 labelled pads in fF + OVERLOAD, ~9 times/s, quietest setting
     DASHBOARD (plotMode = false, or type p):
       USB CSV  t_ms,S1,S2,S3,S4,S5,contact  at 115200 (close the Serial Monitor, then Connect USB)
       BLE      "Kynetix Cap Insole", 16-byte packet (works in both styles)
       Faster: each pad ~60 times/s (pads are measured one at a time so the 2 chips never overlap)
     For the website over USB: type p first (or set plotMode = false). Bluetooth works in either style.

   DASHBOARD SLOTS: S1 big toe | S2 lateral midfoot | S3 5th met | S4 1st met | S5 heel  (values in fF)
   Press each pad once and check the right line / map spot moves. If not, fix PAD_LABEL / PAD_SLOT.

   COMMANDS: z = re-zero (unloaded, hands off)   p = switch dashboard <-> plotter
   START-UP: keep the insole unloaded and hands off for ~2 s after power-on/reset.
*/
#include <Wire.h>
#include <math.h>
#include <ArduinoBLE.h>

// Second, separately wired I2C interface (mbed core; SDA D2, SCL D3).
// Avoid Wire1, which is assigned to the Sense board IMU.
TwoWire secondBus(D2, D3);
TwoWire* const BUS[2] = {&Wire, &secondBus};
const uint8_t N_PADS = 5;
const uint8_t PAD_FDC[N_PADS] = {0, 0, 0, 1, 1};   // 0 = FDC #1 (D4/D5), 1 = FDC #2 (D2/D3)
const uint8_t PAD_CIN[N_PADS] = {0, 1, 2, 0, 1};   // 0 = CIN1, 1 = CIN2, 2 = CIN3
bool fdcPresent[2] = {false, false};

// ---------------- settings ----------------
const char* PAD_LABEL[N_PADS] = {"BigToe", "1stMet", "5thMet", "LatMid", "Heel"};   // plotter names (measured by pressing)
// dashboard slot of each pad: 0 big toe, 1 lateral midfoot, 2 5th met, 3 1st met, 4 heel
const uint8_t PAD_SLOT[N_PADS] = {0, 3, 2, 1, 4};
bool plotMode = false;                 // true = labelled Serial Plotter lines, false = website CSV over USB (BLE works in both)
const uint8_t RATE_PLOT = 1;          // plotter: 100 S/s (quietest)
const uint8_t RATE_DASH = 3;          // dashboard: 400 S/s (fast enough for ground contact time)
const int ZERO_SAMPLES = 20;          // readings averaged per pad when zeroing
const uint32_t PLOT_DELAY_MS = 50;    // plotter only: ~4-5 s fit on the screen
// -------------------------------------------

const uint8_t FDC_ADDR = 0x50;
const float COUNTS_PER_PF = 524288.0f;   // 2^19 counts per pF
const float CAPDAC_STEP_PF = 3.125f;     // each CAPDAC step = 3.125 pF offset
const float OVERLOAD_PF = 14.9f;         // chip reads only about +/-15 pF around the CAPDAC offset

uint8_t capdac[N_PADS] = {0};
float baselinePF[N_PADS] = {0};
bool zeroOK[N_PADS] = {false};

// ---------- Bluetooth (same service, characteristic and packet as before) ----------
const char* SERVICE_UUID = "713d0001-5b8c-4e35-ae3f-5296c7420010";
const char* DATA_UUID    = "713d0002-5b8c-4e35-ae3f-5296c7420010";
BLEService kynetixService(SERVICE_UUID);
BLECharacteristic dataCharacteristic(DATA_UUID, BLERead | BLENotify, 16, true);
bool bleOK = false;
uint8_t sequenceNumber = 0;

void putU16(uint8_t* p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
void putU32(uint8_t* p, uint32_t v) { p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; p[2] = (v >> 16) & 0xff; p[3] = (v >> 24) & 0xff; }

uint8_t rateCode() { return plotMode ? RATE_PLOT : RATE_DASH; }

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

// One measurement of one pad with the current CAPDAC.
// Returns the reading in pF relative to the CAPDAC offset (valid range about +/-15), or NAN on I2C failure.
float measureOnce(int pad) {
  uint8_t device = PAD_FDC[pad];
  int in = PAD_CIN[pad];
  if (!fdcPresent[device]) return NAN;
  if (!writeReg(device, 0x08, ((uint16_t)in << 13) | (4U << 10) | ((uint16_t)capdac[pad] << 5))) return NAN;
  if (!writeReg(device, 0x0C, ((uint16_t)rateCode() << 10) | 0x0080)) return NAN;   // start MEAS1 once
  uint32_t t0 = millis();
  uint16_t st = 0;
  do {
    if (!readReg(device, 0x0C, st)) return NAN;
    if (millis() - t0 > 50) return NAN;
    delayMicroseconds(200);
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
  if (!plotMode) Serial.println("t_ms,S1,S2,S3,S4,S5,contact");
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

  bleOK = BLE.begin();
  if (bleOK) {
    BLE.setLocalName("Kynetix Cap Insole");
    BLE.setDeviceName("Kynetix Cap Insole");
    BLE.setConnectionInterval(6, 12);          // 7.5-15 ms: fewer missed packets
    BLE.setAdvertisedService(kynetixService);
    kynetixService.addCharacteristic(dataCharacteristic);
    BLE.addService(kynetixService);
    uint8_t initial[16] = {0};
    dataCharacteristic.writeValue(initial, sizeof(initial));
    BLE.advertise();
    Serial.println("# Bluetooth on: Kynetix Cap Insole");
  } else {
    Serial.println("# Bluetooth failed to start (USB still works)");
  }

  zeroAll();
}

void loop() {
  if (bleOK) BLE.poll();
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'z' || ch == 'Z') zeroAll();
    if (ch == 'p' || ch == 'P') {
      plotMode = !plotMode;
      Serial.println(plotMode ? "# PLOTTER output" : "# DASHBOARD output");
      zeroAll();                                        // speed setting changed: re-zero
    }
  }

  const uint32_t timestamp = millis();
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

  // dashboard values: fF, one per slot, no negatives (the dashboard decides contact itself)
  uint16_t slotVal[5] = {0, 0, 0, 0, 0};
  for (int in = 0; in < N_PADS; in++) {
    long v = outFF[in];
    if (v < 0) v = 0;
    if (v > 65535) v = 65535;
    slotVal[PAD_SLOT[in]] = (uint16_t)v;
  }
  const uint8_t contact = 0;                              // not used by the dashboard (USE_FIRMWARE_FLAG = false)

  if (bleOK && dataCharacteristic.subscribed()) {
    uint8_t packet[16];
    putU32(packet, timestamp);
    for (int s = 0; s < 5; s++) putU16(packet + 4 + 2 * s, slotVal[s]);
    packet[14] = contact;
    packet[15] = sequenceNumber++;
    dataCharacteristic.writeValue(packet, sizeof(packet));
  }

  if (plotMode) {
    for (int in = 0; in < N_PADS; in++) {
      Serial.print(PAD_LABEL[in]); Serial.print(':'); Serial.print(outFF[in]); Serial.print(',');
    }
    Serial.print("OVERLOAD:"); Serial.println(anyOverload ? 15000 : 0);
    delay(PLOT_DELAY_MS);
  } else {
    Serial.print(timestamp);
    for (int s = 0; s < 5; s++) { Serial.print(','); Serial.print(slotVal[s]); }
    Serial.print(','); Serial.println(contact);
  }
}
