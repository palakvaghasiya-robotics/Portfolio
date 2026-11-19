#include "drive.h"

Drive::Drive(int p1, int d1, int p2, int d2, int p3, int d3, int p4, int d4)
    : pwm1(p1), dir1(d1), pwm2(p2), dir2(d2), pwm3(p3), dir3(d3), pwm4(p4), dir4(d4) {}

void Drive::begin() {
    pinMode(pwm1, OUTPUT); pinMode(dir1, OUTPUT);
    pinMode(pwm2, OUTPUT); pinMode(dir2, OUTPUT);
    pinMode(pwm3, OUTPUT); pinMode(dir3, OUTPUT);
    pinMode(pwm4, OUTPUT); pinMode(dir4, OUTPUT);
}

void Drive::move(int axisY, int axisRX, int l2, int r2) {
    // Apply deadzone
    if (abs(axisY) < 80) axisY = 0;
    if (abs(axisRX) < 80) axisRX = 0;

    // Check if all inputs are neutral (no movement)
    bool noInput = (axisY == 0 && axisRX == 0 && l2 == 0 && r2 == 0);

    if (noInput) {
        // Set target speeds to zero for ramp-down
        M1 = 0;
        M2 = 0;
        M3 = 0;
        M4 = 0;
    } else {
        // Translational Movement
        int uppwm = map(axisY, 0, -512, 0, max_pwm);     // Forward
        int downpwm = map(axisY, 0, 512, 0, max_pwm);    // Backward
        int rightpwm = map(axisRX, 0, -512, 0, max_pwm); // Right
        int leftpwm = map(axisRX, 0, 512, 0, max_pwm);   // Left

        // Rotation control mapped from L2 and R2 (0-1024 to 0-255)
        int rotationRight = map(r2, 0, 1024, 0, max_pwm);
        int rotationLeft  = map(l2, 0, 1024, 0, max_pwm);
        int rotation = constrain(rotationRight - rotationLeft, -80, 80);

        // Mixed motor values for omnidirectional X-drive
        int m1pwm = uppwm - rightpwm + rotation;
        int m2pwm = uppwm + rightpwm - rotation;
        int m3pwm = downpwm - leftpwm + rotation;
        int m4pwm = downpwm + leftpwm - rotation;

        // Determine direction
        m1 = (m1pwm > 0);
        m2 = (m2pwm > 0);
        m3 = (m3pwm < 0);
        m4 = (m4pwm < 0);

        // Absolute PWM values
        M1 = constrain(abs(m1pwm), 0, max_pwm);
        M2 = constrain(abs(m2pwm), 0, max_pwm);
        M3 = constrain(abs(m3pwm), 0, max_pwm);
        M4 = constrain(abs(m4pwm), 0, max_pwm);

        digitalWrite(dir1, m1);
        digitalWrite(dir2, m2);
        digitalWrite(dir3, m3);
        digitalWrite(dir4, m4);
    }

    // --- RAMPING LOGIC START ---
    auto ramp = [](int current, int target, int step) {
        if (current < target) return min(current + step, target);
        if (current > target) return max(current - step, target);
        return target;
    };

    prevM1 = ramp(prevM1, M1, ramp_step);
    prevM2 = ramp(prevM2, M2, ramp_step);
    prevM3 = ramp(prevM3, M3, ramp_step);
    prevM4 = ramp(prevM4, M4, ramp_step);
    // --- RAMPING LOGIC END ---

    analogWrite(pwm1, prevM1);
    analogWrite(pwm2, prevM2);
    analogWrite(pwm3, prevM3);
    analogWrite(pwm4, prevM4);
}