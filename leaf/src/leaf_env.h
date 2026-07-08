#pragma once
// BME280 (custom PCB: module in J10 or bare chip on U7; dedicated I2C on
// GPIO40/41, addr 0x76). No-op unless LEAF_BME280 is defined.
void leaf_env_log();
