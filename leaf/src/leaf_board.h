#pragma once
/*
 * Leaf board pin map — Freenove ESP32-S3-WROOM CAM.
 *
 * Camera GPIOs are the ESP32S3_EYE DVP map, identical to gate-b-coexist's
 * gate_b_board.h (that's the board this project runs on). Kept here so this
 * project builds standalone. See docs/trailcam/hardware.md for the full leaf GPIO
 * budget; the non-camera reservations (PIR/ADC/BME280) live there.
 *
 *   GPIO21  PIR (AM312) ext0 deep-sleep wake      (LEAF_PIR_GPIO, in main.cpp)
 *   GPIO3   battery-divider ADC (ADC1_CH2)        (backlog)
 *   GPIO40/41  BME280 I2C SDA/SCL                 (backlog)
 *   GPIO39  spare
 *   47/48/38/14/1/2/42  SX1262 SPI + control      (radio layer, backlog)
 */

// ---- camera (ESP32S3_EYE map) ----
#define CAM_PIN_PWDN  -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK  15
#define CAM_PIN_SIOD   4   // SDA
#define CAM_PIN_SIOC   5   // SCL
#define CAM_PIN_D7    16   // Y9
#define CAM_PIN_D6    17   // Y8
#define CAM_PIN_D5    18   // Y7
#define CAM_PIN_D4    12   // Y6
#define CAM_PIN_D3    10   // Y5
#define CAM_PIN_D2     8   // Y4
#define CAM_PIN_D1     9   // Y3
#define CAM_PIN_D0    11   // Y2
#define CAM_PIN_VSYNC  6
#define CAM_PIN_HREF   7
#define CAM_PIN_PCLK  13
