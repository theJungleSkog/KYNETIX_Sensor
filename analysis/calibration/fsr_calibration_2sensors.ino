// 2-FSR reader with auto-zero
// PLOT = true  -> Serial Plotter, one line per sensor
// PLOT = false -> text: raw mV + zeroed signal per sensor (for calibration)
// Type  t  + Enter in the Serial Monitor to re-zero (nothing pressing the sensors)

const int   N         = 2;
const int   PINS[N]   = {32, 33};              // D32, D33
const char* NAMES[N]  = {"S1", "S2"};
const bool  PLOT      = true;
const float DEADBAND  = 15.0;                  // mV ignored around zero

float baseline[N];

float readMV(int pin, int n) {
  long sum = 0;
  for (int i = 0; i < n; i++) sum += analogReadMilliVolts(pin);
  return sum / (float)n;
}

void tare() {
  for (int k = 0; k < N; k++) baseline[k] = readMV(PINS[k], 300);
  if (!PLOT) Serial.println("Zeroed both sensors.");
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  delay(1000);          // hands off the sensors during this second
  tare();
}

void loop() {
  if (Serial.available() && Serial.read() == 't') tare();

  if (!PLOT) { Serial.print("t="); Serial.print(millis() / 1000.0, 1); Serial.print(" s"); }

  for (int k = 0; k < N; k++) {
    float mv  = readMV(PINS[k], 20);
    float sig = mv - baseline[k];
    if (sig < DEADBAND) sig = 0;

    if (PLOT) {
      Serial.print(NAMES[k]); Serial.print(':'); Serial.print(sig, 0);
      Serial.print(k < N - 1 ? ',' : '\n');
    } else {
      Serial.print("   "); Serial.print(NAMES[k]);
      Serial.print(" raw_mV="); Serial.print(mv, 0);
      Serial.print(" press_mV="); Serial.print(sig, 0);
    }
  }
  if (!PLOT) Serial.println();
  delay(10);
}