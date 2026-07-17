#pragma once
#include <Arduino.h>
/*
 * Switched power rails on the custom leaf PCB (v0.5+, JLC fab v6).
 *
 * Q1/Q2 P-FETs gate CAM_3V3 / LORA_3V3; 100k pull-ups hold the gates HIGH
 * (rails OFF) whenever the MCU sleeps or floats the pins — IO39/IO44 are not
 * RTC GPIOs, the pull-up IS the sleep hold. Drive LOW to power a rail.
 *
 * On boards without the flags (bench Freenove) these are no-ops — and MUST
 * be: GPIO39/40 belong to the Freenove's SD slot.
 */
static inline void leaf_cam_rail(bool on) {
#ifdef LEAF_CAM_PWR_GPIO
    pinMode(LEAF_CAM_PWR_GPIO, OUTPUT);
    digitalWrite(LEAF_CAM_PWR_GPIO, on ? LOW : HIGH);
    if (on) delay(5);   // ME6211 2V8/1V5 settle before SCCB traffic
#else
    (void)on;
#endif
}
static inline void leaf_radio_rail(bool on) {
#ifdef LEAF_RADIO_PWR_GPIO
    pinMode(LEAF_RADIO_PWR_GPIO, OUTPUT);
    digitalWrite(LEAF_RADIO_PWR_GPIO, on ? LOW : HIGH);
    if (on) delay(2);   // rail + SX1262/SD bulk caps settle
#else
    (void)on;
#endif
}
static inline void leaf_rails_all_off() {
    leaf_cam_rail(false);
    leaf_radio_rail(false);
}
static inline float leaf_read_vbat() {
#ifdef LEAF_VBAT_ADC_GPIO
    // 470k/470k divider from BAT+ (EYE values) -> x2. millivolts read is
    // factory-calibrated on the S3; fine for a trend + low-batt throttle.
    return analogReadMilliVolts(LEAF_VBAT_ADC_GPIO) * 2 / 1000.0f;
#else
    return NAN;   // bench Freenove: no divider
#endif
}
static inline void leaf_log_vbat() {
#ifdef LEAF_VBAT_ADC_GPIO
    Serial.printf("[leaf] vbat=%lumV\n", (unsigned long)(leaf_read_vbat() * 1000));
#endif
}
