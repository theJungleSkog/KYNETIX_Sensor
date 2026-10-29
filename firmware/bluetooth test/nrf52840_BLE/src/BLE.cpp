#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>

BLEUart bleuart;  // Nordic UART Service

void startAdv() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);   // advertise that we offer UART
  Bluefruit.ScanResponse.addName();            // name goes in the scan response
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // fast, then slow (units of 0.625 ms)
  Bluefruit.Advertising.setFastTimeout(30);    // seconds in fast mode
  Bluefruit.Advertising.start(0);              // 0 = advertise forever
}

void setup() {
  Serial.begin(115200);
  // No while(!Serial) here, so it still runs on battery with no USB attached

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("XIAO-Hello");

  bleuart.begin();
  startAdv();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  static uint32_t n = 0;

  if (Bluefruit.connected() && bleuart.notifyEnabled()) {
    bleuart.print("Hello world #");
    bleuart.println(n);
  }
  //Serial.print("Hello world #");    // still mirrored to USB for debugging
  //Serial.println(n);

  n++;
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(500);
}