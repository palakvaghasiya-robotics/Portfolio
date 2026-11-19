#ifndef UNIVERSAL_OTA_H
#define UNIVERSAL_OTA_H

#include <WiFi.h>
#include <ArduinoOTA.h>

class UniversalOTA {
public:
    void begin();
    void handle();

private:
    void connectWiFi();
    void setupOTA();
};

extern UniversalOTA OTA;

#endif
