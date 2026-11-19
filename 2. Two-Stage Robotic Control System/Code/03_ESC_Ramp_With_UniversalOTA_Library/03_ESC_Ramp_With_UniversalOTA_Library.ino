#include <ESP32Servo.h>
#include "UniversalOTA.h"

// ===== ESC SETUP =====
#define ESC_PIN 18
#define MIN_THROTTLE 1000
#define MAX_THROTTLE 1600

Servo esc;
int throttle = MIN_THROTTLE;

void setup() {
  OTA.begin();                 // OTA ALWAYS STARTS
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, 1000, 2000);

  esc.writeMicroseconds(MIN_THROTTLE);
  delay(3000);                 // SimonK arming
}

void loop() {
  OTA.handle();                // OTA ALWAYS ALIVE

  // ---- RAMP UP ----
  for (int t = MIN_THROTTLE; t <= MAX_THROTTLE; t += 5) {
    esc.writeMicroseconds(t);
    delay(20);
    OTA.handle();              // VERY IMPORTANT
  }

  delay(2000);

  // ---- RAMP DOWN ----
  for (int t = MAX_THROTTLE; t >= MIN_THROTTLE; t -= 5) {
    esc.writeMicroseconds(t);
    delay(20);
    OTA.handle();              // VERY IMPORTANT
  }

  delay(5000);
}
