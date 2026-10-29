// KYNETIX Ground Contact Timer - GCT_v1.9
// v1.9 (debug build): independent 1 Hz heartbeat task + LED blink that prints
//   no matter what the main code is doing (even if setup() hangs), plus
//   BOOT: checkpoints in setup(). Waits up to 5 s for a USB monitor at boot.
// Based on GCT_v1.7 (XIAO nRF52840 + BLE). Changes in v1.8:
//   - Prints a status banner whenever a USB monitor opens or a BLE client
//     subscribes, so boot messages are never "lost" before you connect.
//   - If the FDC1004 is not detected, retries every 2 s and prints WHY
//     (no I2C response vs. wrong chip ID) instead of going silent forever.
//   - 'b' always re-initializes the FDC1004 before re-zeroing.
// Hardware: XIAO nRF52840 + ProtoCentral FDC1004 + 2 capacitive pads.
//
// WIRING:
//   XIAO 3V3 -> breakout 3.3V (leave breakout VCC disconnected)
//   XIAO GND -> breakout GND
//   XIAO D4 (SDA) -> SDA; XIAO D5 (SCL) -> SCL
//   CIN1 -> sensor 1 upper copper plate; CIN2 -> sensor 2 upper copper plate
//   Both lower copper plates -> GND. No RC resistor. Others unconnected.
//
// CONTACT CONDITIONS, measured relative to unloaded baseline:
//   Start if EITHER pad is > 0.40 pF continuously for >= 20 ms.
//   End if BOTH pads are < 0.30 pF continuously for >= 80 ms.
//   The timer uses the timestamp of the FIRST threshold crossing, not
//   the end of its debounce window, so confirmation does not subtract time.
//   Reject confirmed events shorter than 0.10 s (likely false steps).
//   Require 0.10 s of unloaded readings before arming for another step.
//
// SPEED: Continuous round-robin FDC1004 measurements at 400 S/s total,
//        nominally 200 S/s per channel (approx 5 ms/pair). Time in micros().
//        0.01 s display resolution is NOT a guarantee of absolute accuracy.
//
// OUTPUT: Every line goes to BOTH USB Serial and BLE UART (if a client is
//         connected and subscribed). BLE device name: "KYNETIX-GCT".
//         Normal mode prints only validated finished steps (silence while
//         idle is expected); send d for live readings.
//
// COMMANDS (from USB Serial OR the BLE UART, e.g. Bluefruit Connect app):
//   b = re-init + re-zero both UNLOADED sensors, cancel unfinished step.
//   d = toggle live 10 Hz diagnostic readings (Cap1, Cap2, state).
//   s = print status banner.
//
// Requires the Adafruit nRF52 Arduino core (Seeed "afruitnrf52" board variant).

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>

// =========================== SETTINGS ===========================
const char *FW_VERSION = "GCT_v1.9";
const char *BLE_NAME = "KYNETIX-GCT";
const uint8_t FDC_ADDR = 0x50;

const float START_THRESHOLD_PF = 0.40f;
const float STOP_THRESHOLD_PF  = 0.30f;
const uint32_t START_CONFIRM_US = 20000UL;   // >0.40 pF for 20 ms.
const uint32_t END_CONFIRM_US   = 80000UL;   // Both <0.30 pF for 80 ms.
const uint32_t MIN_STEP_US      = 100000UL;  // Reject contacts shorter than 100 ms.
const uint32_t REARM_US         = 100000UL;  // Both <0.30 pF for 100 ms.
const uint32_t MAX_GAP_US       = 100000UL;  // Drop ambiguous debounce if data stall >100ms.
const uint32_t DEBUG_INTERVAL_MS = 100;      // 10 diagnostic lines per second.
const uint32_t DETECT_RETRY_MS  = 2000;      // Retry FDC1004 detection every 2 s.

const uint8_t RATE_CODE = 3;           // 400 conversions/sec TOTAL.
const int BASELINE_SAMPLES = 100;
const uint32_t TIMEOUT_MS = 50;
const float COUNTS_PER_PF = 524288.0f;
const float CAPDAC_STEP_PF = 3.125f;
const float VALID_RESIDUAL_LIMIT_PF = 14.9f;

const uint8_t REG_FDC_CONFIG = 0x0C;
const uint16_t BOTH_DONE_MASK = 0x000C;  // DONE_1 + DONE_2
const uint16_t REPEAT_BOTH_CONFIG =
    ((uint16_t)RATE_CODE << 10) | (1U << 8) | (1U << 7) | (1U << 6);

// ======================= BLE + OUTPUT ===========================
BLEUart bleuart;  // Nordic UART Service

// Collects characters into a line, then sends the whole line at once to
// USB Serial and BLE. Sending whole lines (instead of byte by byte) keeps
// BLE traffic to one or two notifications per line.
class DualConsole : public Print {
 public:
  size_t write(uint8_t c) override {
    buf_[len_++] = (char)c;
    if (c == '\n' || len_ == sizeof(buf_)) flushLine();
    return 1;
  }
  using Print::write;

 private:
  void flushLine() {
    if (len_ == 0) return;
    Serial.write((const uint8_t *)buf_, len_);
    if (Bluefruit.connected() && bleuart.notifyEnabled()) {
      bleuart.write((const uint8_t *)buf_, len_);
    }
    len_ = 0;
  }
  char buf_[160];
  size_t len_ = 0;
};
DualConsole out;

void startAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void setupBLE() {
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName(BLE_NAME);
  bleuart.begin();
  startAdvertising();
}

// ============================ STATE =============================
uint8_t capdac[2] = {0, 0};
float baselinePF[2] = {0.0f, 0.0f};
bool baselineValid[2] = {false, false};
bool deviceReady = false;
const char *fdcError = "not initialized yet";
char fdcErrorBuf[96];

enum ContactState : uint8_t {
  REARMING, IDLE, VERIFYING_START, IN_CONTACT, VERIFYING_END
};
ContactState contactState = REARMING;
uint32_t candidateStartUS = 0;
uint32_t contactStartUS = 0;
uint32_t candidateEndUS = 0;
uint32_t lowSinceUS = 0;
uint32_t lastSampleUS = 0;
uint32_t stepNumber = 0;
bool debugEnabled = false;
uint32_t lastDebugMS = 0;
uint32_t lastDetectRetryMS = 0;

// 3-sample median rejects isolated spikes without averaging over long periods.
float history1[3] = {0, 0, 0};
float history2[3] = {0, 0, 0};
uint8_t historyIndex = 0;
uint8_t historyCount = 0;

float medianOf3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

bool filterSamples(float raw1, float raw2, float &out1, float &out2) {
  history1[historyIndex] = raw1;
  history2[historyIndex] = raw2;
  historyIndex = (historyIndex + 1) % 3;
  if (historyCount < 3) historyCount++;
  if (historyCount < 3) return false;
  out1 = medianOf3(history1[0], history1[1], history1[2]);
  out2 = medianOf3(history2[0], history2[1], history2[2]);
  return true;
}

// ======================= LOW-LEVEL I2C ==========================
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
  // Sensor 0 -> CIN1/MEAS1; sensor 1 -> CIN2/MEAS2.
  // CHB=4 enables the CAPDAC reference in single-ended mode.
  uint16_t conf = ((uint16_t)sensor << 13) |
                  (4U << 10) |
                  ((uint16_t)capdac[sensor] << 5);
  return writeReg(0x08 + sensor, conf);
}

// Only used while calibrating; repeated high-speed mode is used during timing.
bool readOne(uint8_t sensor, float &residualPF) {
  uint16_t trigger = ((uint16_t)RATE_CODE << 10) | (0x0080U >> sensor);
  uint16_t doneMask = 0x0008U >> sensor;
  if (!writeReg(REG_FDC_CONFIG, trigger)) return false;

  uint32_t startMS = millis();
  while (true) {
    uint16_t status;
    if (!readReg(REG_FDC_CONFIG, status)) return false;
    if (status & doneMask) break;
    if ((uint32_t)(millis() - startMS) >= TIMEOUT_MS) return false;
    delay(1);
  }

  uint16_t msb, lsb;
  uint8_t dataReg = sensor * 2;
  if (!readReg(dataReg, msb) || !readReg(dataReg + 1, lsb)) return false;
  residualPF = decodeResult(msb, lsb) / COUNTS_PER_PF;
  return isfinite(residualPF);
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
    return fabsf(residual) < VALID_RESIDUAL_LIMIT_PF;
  }
  return false;
}

// Sets fdcError to a human-readable reason on failure.
bool initializeFDC() {
  uint16_t manufacturer, device;
  if (!readReg(0xFE, manufacturer) || !readReg(0xFF, device)) {
    fdcError = "no I2C response at 0x50 (check SDA=D4, SCL=D5, 3V3, GND)";
    return false;
  }
  if (manufacturer != 0x5449 || device != 0x1004) {
    snprintf(fdcErrorBuf, sizeof(fdcErrorBuf),
             "unexpected ID: manufacturer 0x%04X, device 0x%04X "
             "(expected 0x5449, 0x1004)", manufacturer, device);
    fdcError = fdcErrorBuf;
    return false;
  }

  if (!writeReg(REG_FDC_CONFIG, 0x8000)) {  // Software reset.
    fdcError = "reset write failed";
    return false;
  }
  delay(10);
  uint16_t status;
  if (!readReg(REG_FDC_CONFIG, status) || (status & 0x8000)) {
    fdcError = "reset did not complete";
    return false;
  }

  capdac[0] = capdac[1] = 0;
  if (!configureSensor(0) || !configureSensor(1)) {
    fdcError = "channel config write failed";
    return false;
  }
  fdcError = "";
  return true;
}

bool takeBaselines() {
  // Stop any previous continuous measurements before tuning CAPDAC.
  if (!writeReg(REG_FDC_CONFIG, 0x0000)) return false;
  baselineValid[0] = baselineValid[1] = false;

  for (uint8_t sensor = 0; sensor < 2; sensor++) {
    if (!tuneOffset(sensor)) return false;

    double sum = 0.0;
    int good = 0;
    for (int i = 0; i < BASELINE_SAMPLES; i++) {
      float residual;
      if (!readOne(sensor, residual) ||
          fabsf(residual) >= VALID_RESIDUAL_LIMIT_PF) continue;
      sum += residual + capdac[sensor] * CAPDAC_STEP_PF;
      good++;
    }
    if (good < (BASELINE_SAMPLES * 8 / 10)) return false;

    baselinePF[sensor] = (float)(sum / good);
    baselineValid[sensor] = true;
  }
  return true;
}

bool startContinuousMeasurements() {
  // Sensor configuration and CAPDAC are fixed after baseline calibration.
  // Repeating MEAS1 and MEAS2 lets the FDC1004 perform both autonomously.
  return writeReg(REG_FDC_CONFIG, REPEAT_BOTH_CONFIG);
}

// Returns true only if *both* sensors have NEW conversions.
// Reading both result register pairs clears DONE_1 and DONE_2, respectively.
bool readNewPair(float &delta1, float &delta2, uint32_t &sampleTimeUS) {
  uint16_t status;
  if (!readReg(REG_FDC_CONFIG, status)) return false;
  if ((status & BOTH_DONE_MASK) != BOTH_DONE_MASK) return false;

  // Timestamp close to when the paired results became available. This avoids
  // using the (much slower and variable) time printing/BLE takes.
  sampleTimeUS = micros();

  uint16_t msb1, lsb1, msb2, lsb2;
  if (!readReg(0x00, msb1) || !readReg(0x01, lsb1) ||
      !readReg(0x02, msb2) || !readReg(0x03, lsb2)) return false;

  float residual1 = decodeResult(msb1, lsb1) / COUNTS_PER_PF;
  float residual2 = decodeResult(msb2, lsb2) / COUNTS_PER_PF;
  if (!isfinite(residual1) || !isfinite(residual2) ||
      fabsf(residual1) >= VALID_RESIDUAL_LIMIT_PF ||
      fabsf(residual2) >= VALID_RESIDUAL_LIMIT_PF) return false;

  delta1 = residual1 + capdac[0] * CAPDAC_STEP_PF - baselinePF[0];
  delta2 = residual2 + capdac[1] * CAPDAC_STEP_PF - baselinePF[1];
  return true;
}

// =================== CONTACT DETECTION ==========================
void resetDetection() {
  contactState = REARMING;
  candidateStartUS = 0;
  contactStartUS = 0;
  candidateEndUS = 0;
  lowSinceUS = 0;
  lastSampleUS = 0;
  historyIndex = 0;
  historyCount = 0;
}

const char *stateName(ContactState s) {
  switch (s) {
    case REARMING:        return "REARM";
    case IDLE:            return "IDLE";
    case VERIFYING_START: return "START?";
    case IN_CONTACT:      return "CONTACT";
    case VERIFYING_END:   return "END?";
    default:              return "?";
  }
}

void processSample(float cap1, float cap2, uint32_t nowUS) {
  const bool aboveStart = cap1 > START_THRESHOLD_PF || cap2 > START_THRESHOLD_PF;
  const bool belowStop = cap1 < STOP_THRESHOLD_PF && cap2 < STOP_THRESHOLD_PF;

  // Only continuous valid samples may confirm threshold crossings.
  // A long I2C stall cannot silently complete a 20/80 ms debounce interval.
  if (lastSampleUS != 0 && (uint32_t)(nowUS - lastSampleUS) > MAX_GAP_US) {
    if (contactState == VERIFYING_START) contactState = IDLE;
    if (contactState == VERIFYING_END)   contactState = IN_CONTACT;
    if (contactState == REARMING) lowSinceUS = 0;
    if (debugEnabled) out.println("DEBUG: long sensor data gap; canceled pending debounce");
  }
  lastSampleUS = nowUS;

  switch (contactState) {
    case REARMING:
      if (!belowStop) {
        lowSinceUS = 0;
      } else if (lowSinceUS == 0) {
        lowSinceUS = nowUS;
      } else if ((uint32_t)(nowUS - lowSinceUS) >= REARM_US) {
        contactState = IDLE;
      }
      break;

    case IDLE:
      if (aboveStart) {
        candidateStartUS = nowUS;  // Preserve FIRST crossing timestamp.
        contactState = VERIFYING_START;
      }
      break;

    case VERIFYING_START:
      if (!aboveStart) {
        contactState = IDLE;      // Transient excursion: ignore.
      } else if ((uint32_t)(nowUS - candidateStartUS) >= START_CONFIRM_US) {
        contactStartUS = candidateStartUS;
        contactState = IN_CONTACT;
      }
      break;

    case IN_CONTACT:
      if (belowStop) {
        candidateEndUS = nowUS;   // Preserve FIRST crossing timestamp.
        contactState = VERIFYING_END;
      }
      break;

    case VERIFYING_END:
      if (!belowStop) {
        contactState = IN_CONTACT;  // Contact resumed: don't end early.
      } else if ((uint32_t)(nowUS - candidateEndUS) >= END_CONFIRM_US) {
        const uint32_t durationUS = candidateEndUS - contactStartUS;
        if (durationUS >= MIN_STEP_US) {
          stepNumber++;
          out.print("Step ");
          out.print(stepNumber);
          out.print(" time: ");
          out.print(durationUS / 1000000.0f, 2);
          out.println(" s");
        } else if (debugEnabled) {
          out.print("DEBUG: rejected short pulse (");
          out.print(durationUS / 1000000.0f, 3);
          out.println(" s)");
        }
        // Require a stable unloaded period before accepting a new step.
        contactState = REARMING;
        lowSinceUS = candidateEndUS;  // Already low for END_CONFIRM_US.
      }
      break;
  }
}

// ======================= DEBUG HEARTBEAT ==========================
// Runs in its OWN FreeRTOS task (Scheduler.startLoop), so it keeps printing
// and blinking even if setup() or loop() gets stuck (e.g. on I2C).
//   LED blinks + text prints   -> firmware and USB serial both fine.
//   LED blinks, no text        -> firmware runs; USB serial / monitor problem.
//   LED does not blink         -> firmware not running (upload/board problem).
volatile const char *bootStage = "pre-setup";
volatile uint32_t loopCount = 0;

void heartbeatLoop() {
  static uint32_t n = 0;
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  char line[160];
  int len = snprintf(line, sizeof(line),
      "[HB %lu] up %lu s | stage: %s | FDC: %s | baseline: %s | loops: %lu\n",
      (unsigned long)n++, (unsigned long)(millis() / 1000),
      (const char *)bootStage,
      deviceReady ? "OK" : "NOT DETECTED",
      (baselineValid[0] && baselineValid[1]) ? "OK" : "none",
      (unsigned long)loopCount);
  if (len > 0) Serial.write((const uint8_t *)line, (size_t)min(len, (int)sizeof(line) - 1));
  delay(1000);  // delay() yields to other tasks under FreeRTOS
}

// ===================== STATUS + COMMANDS ==========================
void printStatus() {
  out.print("KYNETIX ");
  out.println(FW_VERSION);
  if (!deviceReady) {
    out.print("FDC1004: NOT detected - ");
    out.println(fdcError);
    out.println("Retrying every 2 s...");
  } else if (!baselineValid[0] || !baselineValid[1]) {
    out.println("FDC1004 OK, but no valid baseline. Unload BOTH sensors and send b.");
  } else {
    out.print("Ready. Start >");
    out.print(START_THRESHOLD_PF, 2);
    out.print(" pF, end BOTH <");
    out.print(STOP_THRESHOLD_PF, 2);
    out.print(" pF. Steps so far: ");
    out.println(stepNumber);
  }
  out.print("Diagnostics ");
  out.println(debugEnabled ? "ON" : "OFF");
  out.println("Commands: b=recalibrate unloaded, d=toggle diagnostics, s=status.");
}

void recalibrate() {
  resetDetection();
  baselineValid[0] = baselineValid[1] = false;
  deviceReady = initializeFDC();  // Always re-init (handles a re-powered chip).
  if (!deviceReady) {
    out.print("ERROR: FDC1004 not detected - ");
    out.println(fdcError);
    return;
  }
  if (!takeBaselines() || !startContinuousMeasurements()) {
    baselineValid[0] = baselineValid[1] = false;
    out.println("ERROR: Calibration failed. Unload BOTH sensors and send b.");
    return;
  }
  out.print("Baseline ready. Start >");
  out.print(START_THRESHOLD_PF, 2);
  out.print(" pF, end BOTH <");
  out.print(STOP_THRESHOLD_PF, 2);
  out.println(" pF.");
}

void handleCommands() {
  bool requestedB = false;
  bool requestedS = false;
  auto handleChar = [&](char c) {
    if (c == 'b' || c == 'B') requestedB = true;
    if (c == 's' || c == 'S') requestedS = true;
    if (c == 'd' || c == 'D') {
      debugEnabled = !debugEnabled;
      out.print("Diagnostics ");
      out.println(debugEnabled ? "ON" : "OFF");
      lastDebugMS = millis();
    }
  };
  while (Serial.available())  handleChar((char)Serial.read());
  while (bleuart.available()) handleChar((char)bleuart.read());
  if (requestedB) recalibrate();
  if (requestedS) printStatus();
}

// Print the status banner when a USB monitor opens or a BLE client subscribes,
// so anything printed at boot (before you connected) is not lost.
void announceToNewListeners() {
  static bool usbWas = false;
  static bool bleWas = false;
  const bool usbNow = (bool)Serial;  // true once the host opens the port (DTR)
  const bool bleNow = Bluefruit.connected() && bleuart.notifyEnabled();
  if ((usbNow && !usbWas) || (bleNow && !bleWas)) printStatus();
  usbWas = usbNow;
  bleWas = bleNow;
}

// ========================== MAIN ================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);  // USB CDC; baud is ignored.

  // Start the heartbeat FIRST so it runs even if anything below hangs.
  bootStage = "waiting for USB monitor (max 5 s)";
  Scheduler.startLoop(heartbeatLoop, 1024);

  // Give the monitor up to 5 s to attach so BOOT lines aren't lost.
  // Still boots on battery (no USB host) after the timeout.
  while (!Serial && millis() < 5000) delay(10);
  Serial.println("=== BOOT: setup() started ===");

  bootStage = "BLE init";
  Serial.println("BOOT: starting BLE...");
  setupBLE();
  Serial.println("BOOT: BLE OK, advertising as KYNETIX-GCT");

  bootStage = "I2C init";
  Serial.println("BOOT: starting I2C (SDA=D4, SCL=D5)...");
  Wire.begin();           // XIAO default I2C: D4 = SDA, D5 = SCL.
  Wire.setClock(400000);  // 400 kHz I2C.
  Serial.println("BOOT: I2C OK");

  bootStage = "FDC1004 init + calibration";
  Serial.println("BOOT: initializing FDC1004 and calibrating (keep pads unloaded)...");
  recalibrate();          // IMPORTANT: both sensors must be unloaded at power-up.
  lastDetectRetryMS = millis();

  Serial.println("BOOT: setup() done, entering loop()");
  bootStage = "loop running";
}

void loop() {
  loopCount++;
  announceToNewListeners();

  if (Serial.available() || bleuart.available()) {
    handleCommands();
    return;
  }

  if (!deviceReady) {
    // Keep trying to find the chip, and say so, instead of going silent.
    if ((uint32_t)(millis() - lastDetectRetryMS) >= DETECT_RETRY_MS) {
      lastDetectRetryMS = millis();
      recalibrate();
    }
    delay(10);
    return;
  }
  if (!baselineValid[0] || !baselineValid[1]) {
    delay(10);  // Waiting for the user to unload the pads and send b.
    return;
  }

  float raw1, raw2;
  uint32_t readingTimeUS;
  if (!readNewPair(raw1, raw2, readingTimeUS)) {
    delayMicroseconds(150);
    return;
  }

  float cap1, cap2;
  if (!filterSamples(raw1, raw2, cap1, cap2)) return;

  processSample(cap1, cap2, readingTimeUS);

  // Diagnostics run ONLY at 10 Hz; detection still uses every new sensor pair.
  if (debugEnabled && (uint32_t)(millis() - lastDebugMS) >= DEBUG_INTERVAL_MS) {
    lastDebugMS = millis();
    out.print("Cap1: ");
    out.print(cap1, 3);
    out.print(" pF, Cap2: ");
    out.print(cap2, 3);
    out.print(" pF, state: ");
    out.print(stateName(contactState));
    out.print(", t: ");
    out.print(millis() / 1000.0f, 2);
    out.println(" s");
  }
}