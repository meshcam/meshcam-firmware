#pragma once
// Environmental sensor on the custom leaf PCB: module in J10 (or a bare chip on
// U7), dedicated I2C on GPIO40/41, addr 0x76 or 0x77.
//
// The part is whatever the "BME280" listing actually shipped — detected at
// runtime by chip-id (reg 0xD0), because the drawer modules turned out to be a
// BMP280 (0x58) and a BME680 (0x61), not a BME280 (0x60). See
// homelab docs/trailcam/board-test-log.md "2026-08-17 late".
//
// No-op unless LEAF_ENV (or the older LEAF_BME280) is defined.
void leaf_env_log();
