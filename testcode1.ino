// KYNETIX - capacitive pressure sensor read by RC charge timing (ESP32 Dev Module)
//
// WIRING (important):
// D25 --[ resistor ]--+-- D26
// |
// top copper tape
// silicone
// bottom copper tape
// |
// GND
// D26 and the top copper share the junction AFTER the resistor.
//
// Serial Monitor: 115200 baud. Type b then Enter to re-take the baseline.
#include "soc/gpio_reg.h"
// ===================== SETTINGS =====================
const int SEND_PIN = 25; // D25: drives the resistor
const int RECV_PIN = 26; // D26: watches the capacitor voltage
const float R_OHMS = 1.0e6; // resistor value. Change to 10.0e6 if you use 10 MOhm
const int N_SAMPLES = 32; // measurements averaged per printed reading
const uint32_t TIMEOUT_US = 2000; // give up on one measurement after 2 ms
const bool CSV_MODE = false; // false = labeled lines (easy to read)
 // true = CSV (copy/paste into Excel)
const uint32_t PERIOD_US = CSV_MODE ? 10000 : 50000; // CSV: 100 Hz, labeled: 20 Hz
// ====================================================
uint32_t timeoutTicks;
float baselineTicks = 0;
#define SEND_HIGH() REG_WRITE(GPIO_OUT_W1TS_REG, (1UL << SEND_PIN))
#define SEND_LOW() REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << SEND_PIN))
#define RECV_READ() (REG_READ(GPIO_IN_REG) & (1UL << RECV_PIN))
// One measurement: empty the capacitor, then time how long it takes to fill
// through the resistor until D26 reads HIGH. Returns ticks, or 0 if it timed out.
uint32_t chargeTime() {
 SEND_LOW();
 pinMode(RECV_PIN, OUTPUT);
 digitalWrite(RECV_PIN, LOW); // empty the capacitor
 delayMicroseconds(20);
 pinMode(RECV_PIN, INPUT); // let go of the junction
 noInterrupts();
 uint32_t t0 = ESP.getCycleCount();
 SEND_HIGH(); // start filling through the resistor
 uint32_t t = 0;
 bool ok = false;
 while ((t = ESP.getCycleCount() - t0) < timeoutTicks) {
 if (RECV_READ()) { ok = true; break; }
 }
 interrupts();
 SEND_LOW();
 return ok ? t : 0;
}
// Average n measurements. Timed-out measurements are skipped (not averaged in)
// and counted in 'timeouts'. Returns 0 if every measurement timed out.
float readAvg(int n, int &timeouts) {
 uint64_t sum = 0;
 int good = 0;
 timeouts = 0;
 for (int i = 0; i < n; i++) {
 uint32_t t = chargeTime();
 if (t == 0) timeouts++;
 else { sum += t; good++; }
 }
 return good ? (float)sum / good : 0;
}
// ticks -> picofarads. 1.39 = ln(4): ESP32 reads HIGH at ~75% of 3.3 V.
float ticksToPF(float ticks) {
 float seconds = ticks / (getCpuFrequencyMhz() * 1e6f);
 return seconds / (1.39f * R_OHMS) * 1e12f;
}
void takeBaseline() {
 int timeouts;
 Serial.println("Measuring baseline - don't touch the sensor...");
 baselineTicks = readAvg(500, timeouts);
 if (baselineTicks == 0) {
 Serial.println("ERROR: every baseline measurement timed out.");
 Serial.println(" -> D26 and the TOP copper must both connect at the junction after the resistor.");
 Serial.println(" -> BOTTOM copper must go to GND.");
 Serial.println(" -> Resistor should read ~1 MOhm; copper plates must NOT have continuity.");
 } else {
 Serial.printf("Baseline: %.1f ticks = %.2f pF (%d of 500 timed out)\n",
 baselineTicks, ticksToPF(baselineTicks), timeouts);
 if (timeouts > 0) {
 Serial.println("WARNING: some baseline measurements timed out - check solder joints.");
 }
 }
 if (CSV_MODE) Serial.println("time_s,ticks,delta_ticks,cap_pF,delta_pF,timeouts");
}
void setup() {
 Serial.begin(115200);
 delay(500);
 pinMode(SEND_PIN, OUTPUT);
 SEND_LOW();
 timeoutTicks = TIMEOUT_US * getCpuFrequencyMhz();
 Serial.println();
 Serial.println("=== KYNETIX capacitive sensor (RC timing) ===");
 Serial.printf("Resistor = %.1f MOhm | %d samples per reading | timeout = %lu us\n",
 R_OHMS / 1e6, N_SAMPLES, (unsigned long)TIMEOUT_US);
 Serial.println("Type b + Enter to re-take the baseline.");
 takeBaseline();
}
void loop() {
 static uint32_t next = micros();
 // Re-take baseline on request
 if (Serial.available()) {
 char c = Serial.read();
 if (c == 'b' || c == 'B') takeBaseline();
 next = micros();
 }
 next += PERIOD_US;
 int timeouts;
 float ticks = readAvg(N_SAMPLES, timeouts);
 float t_s = micros() / 1e6f;
 if (ticks == 0) {
 if (CSV_MODE) Serial.printf("%.3f,TIMEOUT,,,,%d\n", t_s, timeouts);
 else Serial.printf("t=%8.3f s | TIMEOUT - all %d measurements timed out (check wiring)\n",
 t_s, N_SAMPLES);
 } else {
 float deltaTicks = ticks - baselineTicks;
 float pF = ticksToPF(ticks);
 float deltaPF = pF - ticksToPF(baselineTicks);
 if (CSV_MODE) {
 Serial.printf("%.3f,%.1f,%.1f,%.3f,%.3f,%d\n",
 t_s, ticks, deltaTicks, pF, deltaPF, timeouts);
 } else {
 Serial.printf("t=%8.3f s | ticks=%9.1f | change=%+8.1f ticks | C=%7.3f pF | change=%+7.3f pF | timeouts=%d/%d\n",
 t_s, ticks, deltaTicks, pF, deltaPF, timeouts, N_SAMPLES);
 }
 }
 while ((int32_t)(micros() - next) < 0) { } // wait for next reading slot
}