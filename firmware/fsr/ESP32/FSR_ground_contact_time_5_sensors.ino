// FSR insole: ground contact time, 5 sensors on ESP32, 100 Hz over USB serial
// Wiring per sensor: 3.3V -> FSR -> junction -> 10k -> GND, junction -> GPIO
//
// MODE 0 -> Serial Plotter: each sensor's load + a "Contact" square wave
// MODE 1 -> Serial Monitor: one line per step with contact, swing and stride time
// MODE 2 -> CSV for MATLAB: t_ms,S1,S2,S3,S4,S5,contact (USE FOR PROTOYPE FAIR TO CONNECT TO WEBSITE)
//
// Keep the insole UNLOADED for the first 2 seconds after reset (baseline).
// Note: GPIO25/26/27 are on ADC2 and stop reading if WiFi is turned on.

const int MODE = 2;

const int PINS[]    = {32, 33, 25, 26, 27};          // S1, S2, S3, S4, S5
const char* NAMES[] = {"S1", "S2", "S3", "S4", "S5"};
const int N = 5;

const uint32_t PERIOD_US        = 10000;  // 100 Hz -> 10 ms timing resolution
const int      ON_MV            = 150;    // total load to start a contact (mV above baseline)
const int      OFF_MV           = 75;     // total load to end a contact
const uint32_t MIN_CONTACT_MS   = 80;     // shorter contacts are ignored as noise
const int      BASELINE_SAMPLES = 200;    // 2 s at 100 Hz

float baseline[N];
uint32_t next_t;

bool     inContact    = false;
uint32_t contactStart = 0;
uint32_t prevStart    = 0;
uint32_t prevEnd      = 0;
bool     havePrev     = false;
uint32_t stepCount    = 0;
uint32_t sumContactMs = 0;
int      peakTotal    = 0;

void calibrateBaseline() {
  float sum[N] = {0};
  for (int k = 0; k < BASELINE_SAMPLES; k++) {
    for (int i = 0; i < N; i++) sum[i] += analogReadMilliVolts(PINS[i]);
    delay(10);
  }
  for (int i = 0; i < N; i++) baseline[i] = sum[i] / BASELINE_SAMPLES;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  if (MODE == 1) Serial.println("Keep insole unloaded: measuring baseline for 2 s...");
  calibrateBaseline();

  if (MODE == 1) {
    Serial.print("Baseline mV:");
    for (int i = 0; i < N; i++) { Serial.print(' '); Serial.print(NAMES[i]); Serial.print('='); Serial.print(baseline[i], 0); }
    Serial.println();
    Serial.println("Ready. Step on the insole.");
  } else if (MODE == 2) {
    Serial.print("t_ms");
    for (int i = 0; i < N; i++) { Serial.print(','); Serial.print(NAMES[i]); }
    Serial.println(",contact");
  }
  next_t = micros();
}

void reportStep(uint32_t endTime) {
  uint32_t contactMs = endTime - contactStart;
  stepCount++;
  sumContactMs += contactMs;

  Serial.print("Step ");
  Serial.print(stepCount);
  Serial.print(": contact ");
  Serial.print(contactMs);
  Serial.print(" ms");
  if (havePrev) {
    Serial.print(" | swing ");
    Serial.print(contactStart - prevEnd);
    Serial.print(" ms | stride ");
    Serial.print(contactStart - prevStart);
    Serial.print(" ms");
  }
  Serial.print(" | peak ");
  Serial.print(peakTotal);
  Serial.print(" mV | avg contact ");
  Serial.print(sumContactMs / stepCount);
  Serial.println(" ms");
}

void loop() {
  if ((int32_t)(micros() - next_t) < 0) return;
  next_t += PERIOD_US;
  uint32_t now = millis();

  int load[N];
  int total = 0;
  for (int i = 0; i < N; i++) {
    int v = analogReadMilliVolts(PINS[i]) - (int)baseline[i];
    load[i] = v > 0 ? v : 0;
    total += load[i];
  }

  if (!inContact && total > ON_MV) {
    inContact = true;
    contactStart = now;
    peakTotal = total;
  } else if (inContact) {
    if (total > peakTotal) peakTotal = total;
    if (total < OFF_MV) {
      inContact = false;
      if (now - contactStart >= MIN_CONTACT_MS) {
        if (MODE == 1) reportStep(now);
        prevStart = contactStart;
        prevEnd = now;
        havePrev = true;
      }
    }
  }

  if (MODE == 0) {
    for (int i = 0; i < N; i++) {
      Serial.print(NAMES[i]); Serial.print(':'); Serial.print(load[i]); Serial.print(',');
    }
    Serial.print("Contact:");
    Serial.println(inContact ? 1000 : 0);
  } else if (MODE == 2) {
    Serial.print(now);
    for (int i = 0; i < N; i++) { Serial.print(','); Serial.print(load[i]); }
    Serial.print(',');
    Serial.println(inContact ? 1 : 0);
  }
}