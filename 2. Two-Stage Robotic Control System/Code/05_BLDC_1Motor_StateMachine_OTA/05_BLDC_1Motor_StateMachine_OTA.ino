#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>

/* ================= WIFI / OTA ================= */
#define WIFI_SSID     "00000000"
#define WIFI_PASSWORD "00000000"

#define OTA_HOSTNAME  "ESP32-UNIVERSAL"
#define OTA_PASSWORD  "esp32"

/* ================= BLDC / ESC ================= */
Servo esc;
int escPin = 35;   // ONLY ONE MOTOR
// NOTE: GPIO 35 is INPUT-ONLY on ESP32 - no PWM output. Use e.g. 18 instead.

const int minThrottle = 1400;
const int maxThrottle = 1800;

/* ================= TIMING ================= */
unsigned long lastStepTime = 0;
unsigned long stateStartTime = 0;
int throttle = minThrottle;

/* ================= STATE MACHINE ================= */
enum ESC_STATE {
  ARM,
  RAMP,
  RUN,
  STOP,
  DONE
};

ESC_STATE escState = ARM;

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
  esc.setPeriodHertz(50);
  esc.attach(escPin, 1000, 2000);
  esc.writeMicroseconds(minThrottle);

  stateStartTime = millis();
}

/* ================= LOOP ================= */
void loop() {
  ArduinoOTA.handle();   // MUST always run
  escTask();
}

/* ================= BLDC TASK ================= */
void escTask() {
  unsigned long now = millis();

  switch (escState) {

    case ARM:
      if (now - stateStartTime > 5000) {   // 5 sec arming
        throttle = minThrottle;
        escState = RAMP;
        lastStepTime = now;
        Serial.println("Ramping motor");
      }
      break;

    case RAMP:
      if (now - lastStepTime > 50) {
        lastStepTime = now;
        esc.writeMicroseconds(throttle);
        throttle += 5;

        if (throttle >= maxThrottle) {
          escState = RUN;
          stateStartTime = now;
          Serial.println("Motor running");
        }
      }
      break;

    case RUN:
      if (now - stateStartTime > 5000) {
        escState = STOP;
        Serial.println("Stopping motor");
      }
      break;

    case STOP:
      esc.writeMicroseconds(minThrottle);
      escState = DONE;
      Serial.println("ESC sequence done");
      break;

    case DONE:
      // Motor stopped, OTA still active
      break;
  }
}
