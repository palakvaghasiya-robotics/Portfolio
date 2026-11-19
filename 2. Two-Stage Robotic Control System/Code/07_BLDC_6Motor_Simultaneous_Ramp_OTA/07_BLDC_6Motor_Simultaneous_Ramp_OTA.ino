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
int escPins[6] = {2, 4, 32, 33, 25, 26};

/* --- Throttle Values --- */
int m1Throttle = 1000;     // M1 start
int mOtherThrottle = 1000; // M2-M6 start

const int M1_MAX = 2000;
const int OTHER_MAX = 1500;

/* ================= TIMING ================= */
unsigned long lastStepTime = 0;
unsigned long stateStartTime = 0;

/* ================= STATE MACHINE ================= */
enum ESC_STATE {
  ARM_ALL,
  RAMP_ALL,
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
  ArduinoOTA.begin();
  Serial.println("OTA Ready");

  /* ----- ESC INIT ----- */
  for (int i = 0; i < 6; i++) {
    esc[i].setPeriodHertz(50);
    esc[i].attach(escPins[i], 1000, 2000);
    esc[i].writeMicroseconds(1000);   // safe init
  }

  stateStartTime = millis();
}

/* ================= LOOP ================= */
void loop() {
  ArduinoOTA.handle();   // OTA must always run
  escTask();
}

/* ================= BLDC TASK ================= */
void escTask() {
  unsigned long now = millis();

  switch (escState) {

    case ARM_ALL:
      if (now - stateStartTime > 5000) {   // 5 sec arming
        m1Throttle = 1730;
        mOtherThrottle = 1000;
        escState = RAMP_ALL;
        lastStepTime = now;
        Serial.println("Simultaneous ramp started");
      }
      break;

    case RAMP_ALL:
      if (now - lastStepTime > 50) {
        lastStepTime = now;

        /* --- M1 --- */
        esc[0].writeMicroseconds(m1Throttle);
        if (m1Throttle < M1_MAX) {
          m1Throttle += 5;
        }

        /* --- M2 to M6 --- */
        for (int i = 1; i < 6; i++) {
          esc[i].writeMicroseconds(mOtherThrottle);
        }
        if (mOtherThrottle < OTHER_MAX) {
          mOtherThrottle += 5;
        }

        /* --- Finish condition --- */
        if (m1Throttle >= M1_MAX && mOtherThrottle >= OTHER_MAX) {
          escState = RUN_ALL;
          stateStartTime = now;
          Serial.println("All motors reached target speed");
        }
      }
      break;

    case RUN_ALL:
      if (now - stateStartTime > 5000) {   // run for 5 sec
        escState = STOP_ALL;
        Serial.println("Stopping motors");
      }
      break;

    case STOP_ALL:
      for (int i = 0; i < 6; i++) {
        esc[i].writeMicroseconds(1000);
      }
      escState = DONE;
      Serial.println("Motors stopped");
      break;

    case DONE:
      // OTA still active
      break;
  }
}
