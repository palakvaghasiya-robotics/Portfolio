#include <WiFi.h>
#include <ArduinoOTA.h>

// ===== WIFI CREDENTIALS =====
#define WIFI_SSID     "00000000"
#define WIFI_PASSWORD "00000000"

// ===== OTA SETTINGS =====
#define OTA_HOSTNAME  "ESP32-UNIVERSAL"
#define OTA_PASSWORD  "esp32"   // keep fixed

// ===== USER APP PLACEHOLDER =====
void userSetup() {
  pinMode(2, OUTPUT);   // example
}

void userLoop() {
  // Put ANY future logic here
  static unsigned long t = 0;
  if (millis() - t > 5000) {
    t = millis();
    digitalWrite(2, !digitalRead(2));
  }
}

void setup() {
  Serial.begin(115200);

  // ---- WiFi ----
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
  }

  // ---- OTA ----
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    Serial.println("OTA Start");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("OTA End");
  });

  ArduinoOTA.begin();

  // ---- USER SETUP ----
  userSetup();
}

void loop() {
  ArduinoOTA.handle();   // NEVER REMOVE
  userLoop();            // your logic
}
