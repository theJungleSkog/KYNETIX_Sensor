/* KYNETIX | XIAO nRF52840 Sense | Five FSRs + BLE + USB
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
*/
#include <ArduinoBLE.h>

const uint8_t PINS[5] = {A0, A1, A2, A3, A4};
const uint32_t PERIOD_US = 10000;
const uint16_t ON_MV = 150, OFF_MV = 75;
const uint16_t BASELINE_SAMPLES = 200;
const float MV_PER_COUNT = 3300.0f / 4095.0f;
const char* SERVICE_UUID = "713d0001-5b8c-4e35-ae3f-5296c7420010";
const char* DATA_UUID = "713d0002-5b8c-4e35-ae3f-5296c7420010";
BLEService kynetixService(SERVICE_UUID);
BLECharacteristic dataCharacteristic(DATA_UUID, BLERead | BLENotify, 16, true);

float baseline[5] = {0};
uint32_t nextSample = 0;
bool inContact = false;
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
  if (!inContact && total>ON_MV) inContact=true;
  else if (inContact && total<OFF_MV) inContact=false;

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
