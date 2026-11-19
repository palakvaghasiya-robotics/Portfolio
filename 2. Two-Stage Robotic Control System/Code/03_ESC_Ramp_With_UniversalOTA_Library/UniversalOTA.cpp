#include "UniversalOTA.h"

// ===== CONFIG (SET ONCE) =====
#define WIFI_SSID     "00000000"
#define WIFI_PASSWORD "00000000"

#define OTA_HOSTNAME  "ESP32-UNIVERSAL"
#define OTA_PASSWORD  "esp32"

UniversalOTA OTA;

void UniversalOTA::begin() {
    Serial.begin(115200);
    connectWiFi();
    setupOTA();
}

void UniversalOTA::handle() {
    ArduinoOTA.handle();
}

void UniversalOTA::connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
    }
}

void UniversalOTA::setupOTA() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]() {
        Serial.println("OTA Start");
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("OTA End");
    });

    ArduinoOTA.begin();
}
