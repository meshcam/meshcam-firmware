#include "leaf_env.h"
#ifdef LEAF_BME280
#include <Wire.h>
#include <Adafruit_BME280.h>
#ifndef LEAF_BME_SDA
#define LEAF_BME_SDA 40
#endif
#ifndef LEAF_BME_SCL
#define LEAF_BME_SCL 41
#endif
static Adafruit_BME280 s_bme;
static bool s_ok = false, s_tried = false;

void leaf_env_log() {
    if (!s_tried) {
        s_tried = true;
        Wire.begin(LEAF_BME_SDA, LEAF_BME_SCL);
        s_ok = s_bme.begin(0x76, &Wire);
        if (!s_ok) Serial.println("[env] BME280 not found at 0x76");
    }
    if (!s_ok) return;
    // forced mode one-shot would be lower power; default mode is fine for now
    Serial.printf("[env] T=%.2fC RH=%.1f%% P=%.1fhPa\n",
                  s_bme.readTemperature(), s_bme.readHumidity(),
                  s_bme.readPressure() / 100.0f);
}
#else
void leaf_env_log() {}
#endif
