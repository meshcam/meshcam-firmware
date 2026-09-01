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
// Camera-rail HOLD (2026-08-17, board 87:9C, first on-PCB captures). Switching Q1 on
// while the chip is already busy collapsed the board 12/12 times — a hard POWERON reset
// (below the brownout detector, so 0.17.4's brownout backoff never sees it): the v0.6
// 3V3 is an AP2112K LDO fed straight from a ~3.4 V LiFePO4 through spring contacts, in
// dropout with zero headroom, and the camera-side inrush through Q1 takes it under the
// S3's POR. Bringing the rail up first thing at wake, while the chip is quiet, captured
// clean every time. So: on a PIR wake the rail comes up before anything else and is
// HELD for the wake; leaf_camera_end() must not drop it mid-wake (a second capture in a
// maintenance window would re-slam it under load). Only the pre-sleep rails_all_off()
// forces it off. Board-level fix for v1.0 is a soft-start on Q1's gate (respin spec).
// One flag for the whole program. NOT `static bool` here: this header is included by
// main.cpp (which sets the hold) AND leaf_camera.cpp (whose leaf_camera_end() must
// honour it); a file-static would give each its own copy and the hold would never
// reach the camera side. A non-static inline function's local static is shared.
inline bool& leaf_cam_rail_hold_flag() { static bool f = false; return f; }
static inline void leaf_cam_rail_force(bool on) {
#ifdef LEAF_CAM_PWR_GPIO
    pinMode(LEAF_CAM_PWR_GPIO, OUTPUT);
    digitalWrite(LEAF_CAM_PWR_GPIO, on ? LOW : HIGH);
    if (on) delay(5);   // ME6211 2V8/1V5 settle before SCCB traffic
#else
    (void)on;
#endif
}
static inline void leaf_cam_rail(bool on) {
    if (!on && leaf_cam_rail_hold_flag()) return;   // held for the wake: see above
    leaf_cam_rail_force(on);
}
// Power the camera rail now and keep it up until the pre-sleep rails_all_off().
static inline void leaf_cam_rail_hold() {
    leaf_cam_rail_hold_flag() = true;
    leaf_cam_rail_force(true);
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
    leaf_cam_rail_hold_flag() = false;
    leaf_cam_rail_force(false);
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
