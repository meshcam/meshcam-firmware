#pragma once
/*
 * Board pin maps for Gate B. Pick ONE via the PlatformIO env build flag:
 *   -DGATE_B_BOARD_FREENOVE_S3   (ESP32-S3-WROOM CAM; uses the ESP32S3_EYE DVP map)
 *   -DGATE_B_BOARD_XIAO_S3_SENSE (Seeed XIAO ESP32S3 Sense)
 *
 * Camera GPIOs are copied verbatim from arduino-esp32
 * libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h @ master
 * (CAMERA_MODEL_ESP32S3_EYE / CAMERA_MODEL_XIAO_ESP32S3). The esp32-camera library
 * has no API for "board presets" — you fill camera_config_t with explicit pins — so
 * we define them here rather than rely on the example header.
 *
 * The LORA_* pins are chosen from GPIOs the camera does NOT use on each board, for a
 * BARE SX1262 wired over SPI. Confirming this map holds with no strapping/flash/PSRAM/
 * USB collision is part of what Gate B proves.
 *
 * NOTE: LORA_* here is the human-readable map. LoRaInterface (in lib/) can't include
 * this header, so the pins it compiles against are the -DRADIO_*_PIN build_flags in
 * platformio.ini. KEEP THE TWO IN SYNC. (main.cpp uses only the CAM_PIN_* from here.)
 */

#if defined(GATE_B_BOARD_FREENOVE_S3)
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
  // ---- bare SX1262 on free, non-strapping pins ----
  #define LORA_SCK   47
  #define LORA_MOSI  48
  #define LORA_MISO  38   // moved off RTC pin 21 -> 21 reserved for PIR ext0 wake
  #define LORA_NSS   14
  #define LORA_RST    1
  #define LORA_BUSY   2
  #define LORA_DIO1  42
  // Avoided: 0/3/45/46 (strapping), 19/20 (USB-JTAG), 26-37 (flash + octal PSRAM).
  // Reserved for the deployed leaf (NOT wired for the Gate B bench test): GPIO21 = PIR
  // ext0 deep-sleep wake (needs RTC domain 0-21), GPIO3 = battery-divider ADC (ADC1_CH2),
  // GPIO40/41 = BME280 I2C SDA/SCL, GPIO39 spare. See docs/trailcam/hardware.md leaf budget.

#elif defined(GATE_B_BOARD_XIAO_S3_SENSE)
  // ---- camera (XIAO_ESP32S3 map; all internal to the B2B) ----
  #define CAM_PIN_PWDN  -1
  #define CAM_PIN_RESET -1
  #define CAM_PIN_XCLK  10
  #define CAM_PIN_SIOD  40
  #define CAM_PIN_SIOC  39
  #define CAM_PIN_D7    48   // Y9
  #define CAM_PIN_D6    11   // Y8
  #define CAM_PIN_D5    12   // Y7
  #define CAM_PIN_D4    14   // Y6
  #define CAM_PIN_D3    16   // Y5
  #define CAM_PIN_D2    18   // Y4
  #define CAM_PIN_D1    17   // Y3
  #define CAM_PIN_D0    15   // Y2
  #define CAM_PIN_VSYNC 38
  #define CAM_PIN_HREF  47
  #define CAM_PIN_PCLK  13
  // ---- bare SX1262 on castellated pads D0..D5,D8,D9 (GPIO 1-9); keep 43/44=UART for
  //      the serial monitor. The Wio-SX1262 *kit* wants the same B2B as the camera, so
  //      a bare module wired to these pads is the only way to have both. ----
  #define LORA_SCK    7
  #define LORA_MOSI   9
  #define LORA_MISO   8
  #define LORA_NSS    1
  #define LORA_RST    2
  #define LORA_BUSY   3
  #define LORA_DIO1   4

#else
  #error "Define one of GATE_B_BOARD_FREENOVE_S3 / GATE_B_BOARD_XIAO_S3_SENSE"
#endif
