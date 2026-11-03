/* KYNETIX | XIAO nRF52840 Sense | Five FSRs + BLE + USB  (v2: 10%-of-peak toe-off)
   Board package: Seeed nRF52 mbed-enabled Boards (e.g. 2.9.3)
   Library: ArduinoBLE (install from Arduino IDE Library Manager)
   Wiring: S1 Big toe=A0, S2 Lateral midfoot=A1, S3 5th met=A2,
           S4 1st met=A3, S5 Heel=A4.
   Each: 3V3 -> FSR -> ADC junction -> 10k -> GND.
   Leave all sensors unloaded for the first two seconds after power-on/reset.
   Nominal ADC millivolts are not calibrated pressure measurements.

   BLE custom service / characteristic (must match dashboard):
   713d0001-5b8c-4e35-ae3f-5296c7420010 (service)
   713d0002-5b8c-4e35-ae3f-5296c7420010 (notify characteristic)
   Packet: 16 bytes, little-endian:
   [0..3] uint32 t_ms, [4..13] 5 x uint16 S1..S5 mV above baseline,
   [14] uint8 contact (0/1), [15] uint8 sequence (wraps at 255).
   100 Hz ADC sampling target; BLE delivery is best-effort and must be validated.

   CHANGE IN v2 (only this; everything else is identical to v1):
   - Touchdown: unchanged. Contact starts when the whole-foot total rises above ON_MV.
   - Toe-off: the board keeps track of this step's highest total (its peak). The step
     now ends when the total drops below END_FRAC (10%) of that peak, instead of
     waiting for it to fall below a fixed OFF_MV. This cuts off the slow FSR release
     tail that made GCT read too long. OFF_MV is still the lowest the end level can be.
   - After toe-off, a new step only starts once the total rises ON_MV above the lowest
     level since toe-off. Otherwise the fading tail (still above ON_MV) would start a
     false second step.
   The packet, sensor order and dashboard are unchanged; the dashboard's GCT follows
   the contact flag, so it picks up the new toe-off automatically.
*/
#include <ArduinoBLE.h>

const uint8_t PINS[5] = {A0, A1, A2, A3, A4};
const uint32_t PERIOD_US = 10000;
const uint16_t ON_MV = 150, OFF_MV = 75;
const float END_FRAC = 0.10f;          // v2: toe-off when total < 10% of this step's peak
const uint16_t BASELINE_SAMPLES = 200;
const float MV_PER_COUNT = 3300.0f / 4095.0f;
const char* SERVICE_UUID = "713d0001-5b8c-4e35-ae3f-5296c7420010";
const char* DATA_UUID = "713d0002-5b8c-4e35-ae3f-5296c7420010";
BLEService kynetixService(SERVICE_UUID);
BLECharacteristic dataCharacteristic(DATA_UUID, BLERead | BLENotify, 16, true);

float baseline[5] = {0};
uint32_t nextSample = 0;
bool inContact = false;
uint32_t stepPeak = 0;                 // v2: highest total seen during the current step
uint32_t airFloor = 0;                 // v2: lowest total since the last toe-off
uint8_t sequenceNumber = 0;

int readMV(uint8_t pin) { return (int)(analogRead(pin) * MV_PER_COUNT + 0.5f); }
void putU16(uint8_t* p, uint16_t value) { p[0]=value&0xff; p[1]=value>>8; }
void putU32(uint8_t* p, uint32_t value) {
  p[0]=value&0xff; p[1]=(value>>8)&0xff;
  p[2]=(value>>16)&0xff; p[3]=(value>>24)&0xff;
}
void calibrateBaseline() {
  float sum[5]={0};
  for (int k=0; k<BASELINE_SAMPLES; ++k) {
    for (int i=0; i<5; ++i) sum[i]+=readMV(PINS[i]);
    delay(10);
  }
  for (int i=0; i<5; ++i) baseline[i]=sum[i]/BASELINE_SAMPLES;
}
void setup() {
  Serial.begin(115200); // USB debug/CSV, not required for battery-powered BLE
  analogReadResolution(12);
  calibrateBaseline();
  if (!BLE.begin()) {
    if (Serial) Serial.println("BLE initialization failed");
    while (true) { delay(1000); }
  }
  BLE.setLocalName("Kynetix Insole");
  BLE.setDeviceName("Kynetix Insole");
  BLE.setAdvertisedService(kynetixService);
  kynetixService.addCharacteristic(dataCharacteristic);
  BLE.addService(kynetixService);
  uint8_t initial[16]={0};
  dataCharacteristic.writeValue(initial, sizeof(initial));
  BLE.advertise();
  if (Serial) Serial.println("t_ms,S1,S2,S3,S4,S5,contact");
  nextSample=micros();
}
void loop() {
  BLE.poll();
  const uint32_t clockUS=micros();
  if ((int32_t)(clockUS-nextSample)<0) return;
  // Realign after a major pause to avoid bursts of stale samples.
  if ((uint32_t)(clockUS-nextSample)>50000UL) nextSample=clockUS;
  nextSample+=PERIOD_US;
  const uint32_t timestamp=millis();
  uint16_t values[5];
  uint32_t total=0;
  for (int i=0; i<5; ++i) {
    int mv=readMV(PINS[i])-(int)baseline[i];
    values[i]=(uint16_t)constrain(mv,0,3300);
    total+=values[i];
  }

  // ---- v2 contact logic: fixed start, peak-relative end ----
  if (!inContact) {
    if (total<airFloor) airFloor=total;              // follow the release tail down
    // New step: total above ON_MV AND risen ON_MV above the leftover tail,
    // so the slowly fading tail itself can't start a "ghost" step.
    if (total>ON_MV && total-airFloor>ON_MV) {
      inContact=true;
      stepPeak=total;                                // start tracking this step's peak
    }
  } else {
    if (total>stepPeak) stepPeak=total;              // running peak
    uint32_t offLevel=(uint32_t)(END_FRAC*stepPeak); // 10% of this step's peak...
    if (offLevel<OFF_MV) offLevel=OFF_MV;            // ...but never below OFF_MV
    if (total<offLevel) {                            // toe-off
      inContact=false;
      airFloor=total;
    }
  }

  uint8_t packet[16];
  putU32(packet,timestamp);
  for (int i=0;i<5;++i) putU16(packet+4+i*2,values[i]);
  packet[14]=inContact?1:0;
  packet[15]=sequenceNumber++;
  if (dataCharacteristic.subscribed()) dataCharacteristic.writeValue(packet,sizeof(packet));

  // Preserve previous CSV protocol for USB debugging.
  // Printing every 10 ms can impair timing on slow host links; verify measured rate.
  if (Serial) {
    Serial.print(timestamp);
    for (int i=0;i<5;++i) { Serial.print(','); Serial.print(values[i]); }
    Serial.print(','); Serial.println(inContact?1:0);
  }
}
