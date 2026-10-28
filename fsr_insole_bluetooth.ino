// FSR insole: ground contact, 5 sensors, ESP32, 100 Hz over BLUETOOTH serial
// CSV: t_ms,S1,S2,S3,S4,S5,contact  (same format the website already reads)
// Keep the insole UNLOADED for 2 s after power-on (baseline).

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
#define OUT SerialBT            // change to Serial to send over the USB cable instead
const int PINS[]    = {32, 39, 34, 35, 36};      // S1 D32, S2 D33, S3 D34, S4 D35, S5 VP
const char* NAMES[] = {"S1", "S2", "S3", "S4", "S5"};
const int N = 5;
const int LED = 2;                                   // onboard blue LED on most DevKits

const uint32_t PERIOD_US        = 10000;  // 100 Hz
const int      ON_MV            = 150;
const int      OFF_MV           = 75;
const int      BASELINE_SAMPLES = 200;    // 2 s

float baseline[N];
uint32_t next_t;
bool inContact = false;

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
  SerialBT.begin("KYNETIX_Insole");       // name shown in the laptop's Bluetooth list
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);                // LED on = measuring baseline, keep insole unloaded
  calibrateBaseline();
  digitalWrite(LED, LOW);

  next_t = micros();
}

void loop() {
  if ((int32_t)(micros() - next_t) < 0) return;
  next_t += PERIOD_US;
  uint32_t now = millis();

  int load[N], total = 0;
  for (int i = 0; i < N; i++) {
    int v = analogReadMilliVolts(PINS[i]) - (int)baseline[i];
    load[i] = v > 0 ? v : 0;
    total += load[i];
  }
  if (!inContact && total > ON_MV) inContact = true;
  else if (inContact && total < OFF_MV) inContact = false;
  digitalWrite(LED, inContact ? HIGH : LOW);

  OUT.print(now);
  for (int i = 0; i < N; i++) { OUT.print(','); OUT.print(load[i]); }
  OUT.print(',');
  OUT.println(inContact ? 1 : 0);
}