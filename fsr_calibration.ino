// One-FSR calibration: averaged mV + FSR resistance, ~8 readings/second
const int PIN = 32;              // D32 on the board
const float R_FIXED = 10000.0;   // change to 4700.0 if you used a 4.7k resistor

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
}
const bool PLOT = true;   // true = graph, false = text for writing numbers down

void loop() {
  long sum = 0;
  for (int i = 0; i < 20; i++) {          // fewer samples = smoother-moving graph (~25 updates/s)
    sum += analogReadMilliVolts(PIN);
    delay(2);
  }
  float mv = sum / 20.0;

  if (PLOT) {
    Serial.print("mV:");     Serial.print(mv, 0);
    Serial.print(",floor:"); Serial.println(160);   // dashed reference line: below this = no load
  } else {
    Serial.print("t=");  Serial.print(millis() / 1000.0, 1);
    Serial.print(" s   mV=");  Serial.print(mv, 0);
    Serial.print("   R_fsr=");
    if (mv < 160) Serial.println("open (no load)");
    else Serial.println(R_FIXED * (3300.0 - mv) / mv, 0);
  }
}