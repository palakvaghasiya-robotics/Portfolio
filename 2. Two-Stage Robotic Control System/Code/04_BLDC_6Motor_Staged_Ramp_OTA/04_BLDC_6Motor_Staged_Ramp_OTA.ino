#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>

/* ================= WIFI / OTA ================= */
#define WIFI_SSID     "00000000"
#define WIFI_PASSWORD "00000000"

#define OTA_HOSTNAME  "ESP32-UNIVERSAL"
#define OTA_PASSWORD  "esp32"

/* ================= BLDC / ESC ================= */
Servo esc[6];
// NOTE: GPIO 34 and 35 are INPUT-ONLY on ESP32 and cannot output PWM.
// Change these to output-capable pins (e.g. 18, 19, 21, 22) before use.
int escPins[6] = {34, 35, 32, 33, 25, 26};

const int minThrottle = 1400;
const int maxThrottle = 1800;

/* ================= TIMING ================= */
unsigned long lastStepTime = 0;
unsigned long stateStartTime = 0;
int throttle = minThrottle;

/* ================= STATE MACHINE ================= */
enum ESC_STATE {
  ARM_ALL,
  RAMP_1_2,
  ARM_3_6,
  RAMP_3_6,
  RUN_ALL,
  STOP_ALL,
  DONE
};

ESC_STATE escState = ARM_ALL;

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  /* ----- WiFi ----- */
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  /* ----- OTA ----- */
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    Serial.println("OTA Start");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("OTA End");
  });

  ArduinoOTA.begin();
  Serial.println("OTA Ready");

  /* ----- ESC INIT ----- */
  for (int i = 0; i < 6; i++) {
    esc[i].setPeriodHertz(50);
    esc[i].attach(escPins[i], 1000, 2000);
    esc[i].writeMicroseconds(minThrottle);
  }

  stateStartTime = millis();
}

/* ================= LOOP ================= */
void loop() {
  ArduinoOTA.handle();   // MUST always run

  escTask();             // BLDC control
}

/* ================= BLDC TASK ================= */
void escTask() {
  unsigned long now = millis();

  switch (escState) {

    case ARM_ALL:
      if (now - stateStartTime > 5000) {   // 5 sec arming
        throttle = minThrottle;
        escState = RAMP_1_2;
        lastStepTime = now;
        Serial.println("Ramping motors 1 & 2");
      }
      break;

    case RAMP_1_2:
      if (now - lastStepTime > 50) {
        lastStepTime = now;
        esc[0].writeMicroseconds(throttle);
        esc[1].writeMicroseconds(throttle);
        throttle += 5;
        if (throttle >= maxThrottle) {
          escState = ARM_3_6;
          stateStartTime = now;
          Serial.println("Arming motors 3-6");
        }
      }
      break;

    case ARM_3_6:
      for (int i = 2; i < 6; i++) {
        esc[i].writeMicroseconds(minThrottle);
      }
      throttle = minThrottle;
      escState = RAMP_3_6;
      lastStepTime = now;
      Serial.println("Ramping motors 3-6");
      break;

    case RAMP_3_6:
      if (now - lastStepTime > 100) {
        lastStepTime = now;
        for (int i = 2; i < 6; i++) {
          esc[i].writeMicroseconds(throttle);
        }
        throttle += 5;
        if (throttle >= maxThrottle) {
          escState = RUN_ALL;
          stateStartTime = now;
          Serial.println("All motors running");
        }
      }
      break;

    case RUN_ALL:
      if (now - stateStartTime > 5000) {
        escState = STOP_ALL;
        Serial.println("Stopping all motors");
      }
      break;

    case STOP_ALL:
      for (int i = 0; i < 6; i++) {
        esc[i].writeMicroseconds(minThrottle);
      }
      escState = DONE;
      Serial.println("ESC sequence done");
      break;

    case DONE:
      // Motors stopped, OTA still active
      break;
  }
}
