#include "leaf_env.h"
#if defined(LEAF_ENV) || defined(LEAF_BME280)
// Own translation unit, not an .inc pulled into main.cpp: main.cpp includes
// esp_camera.h, whose sensor_t collides with Adafruit_Sensor's sensor_t.
//
// Detect-by-chip-id, ported from bme_probe.cpp (the leaf-pcb-bringup step-1/2
// probe) so production firmware reads the same three parts the bench proved:
//   0x60 BME280  T/RH/P
//   0x58 BMP280  T/P only, RH prints n/a  (the humidity-less look-alike)
//   0x61 BME680  T/RH/P via Bosch bme68x, GAS HEATER OFF — with run_gas on,
//                this module clears ctrl_meas after the forced conversion and
//                T/P/H decode to the frozen reset constants 34.80 C / 100 % /
//                613.6 hPa. MeshCam wants T/RH/P only, so the heater stays off.
// Both addresses are tried (0x76 module default, 0x77 SDO high — the drawer
// BME680 answered at 0x77).
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME680.h>   // pulled in for Bosch's bme68x.c; the Adafruit wrapper
                               // never configured this module (ctrl_meas stayed 0x00),
                               // so bme68x is driven directly below
extern "C" {
#include "bme68x.h"
}

#ifndef LEAF_BME_SDA
#define LEAF_BME_SDA 40
#endif
#ifndef LEAF_BME_SCL
#define LEAF_BME_SCL 41
#endif

enum EnvKind { ENV_NONE, ENV_BME280, ENV_BMP280, ENV_BME680 };
static Adafruit_BME280 s_bme;
static Adafruit_BMP280 s_bmp;
static struct bme68x_dev s_dev;
static struct bme68x_conf s_conf;
static struct bme68x_heatr_conf s_heat;
static EnvKind s_kind = ENV_NONE;
static uint8_t s_addr = 0;
static bool s_ok = false, s_tried = false;

static int8_t bx_read(uint8_t reg, uint8_t* data, uint32_t len, void*) {
    Wire.beginTransmission(s_addr); Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return -1;
    if (Wire.requestFrom((int)s_addr, (int)len) != (int)len) return -1;
    for (uint32_t i = 0; i < len; i++) data[i] = Wire.read();
    return 0;
}
static int8_t bx_write(uint8_t reg, const uint8_t* data, uint32_t len, void*) {
    Wire.beginTransmission(s_addr); Wire.write(reg);
    for (uint32_t i = 0; i < len; i++) Wire.write(data[i]);
    return Wire.endTransmission() == 0 ? 0 : -1;
}
static void bx_delay(uint32_t us, void*) { delayMicroseconds(us); }

static bool i2c_present(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}
static uint8_t rd_reg(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr); Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    if (Wire.requestFrom((int)addr, 1) != 1) return 0xFF;
    return Wire.read();
}

// Forced mode everywhere: the leaf takes one sample per wake and then deep-sleeps,
// so the sensor must go back to standby on its own rather than free-run into sleep.
static void begin_kind(uint8_t id) {
    if (id == 0x60) {
        s_kind = ENV_BME280;
        s_ok = s_bme.begin(s_addr, &Wire);
        if (s_ok) s_bme.setSampling(Adafruit_BME280::MODE_FORCED,
                                    Adafruit_BME280::SAMPLING_X1,   // temperature
                                    Adafruit_BME280::SAMPLING_X1,   // pressure
                                    Adafruit_BME280::SAMPLING_X1,   // humidity
                                    Adafruit_BME280::FILTER_OFF);
    } else if (id == 0x58) {
        s_kind = ENV_BMP280;
        s_ok = s_bmp.begin(s_addr, 0x58);
        if (s_ok) s_bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                                    Adafruit_BMP280::SAMPLING_X1,   // temperature
                                    Adafruit_BMP280::SAMPLING_X1,   // pressure
                                    Adafruit_BMP280::FILTER_OFF);
    } else if (id == 0x61) {
        s_kind = ENV_BME680;
        s_dev.intf = BME68X_I2C_INTF; s_dev.read = bx_read; s_dev.write = bx_write;
        s_dev.delay_us = bx_delay; s_dev.amb_temp = 25; s_dev.intf_ptr = nullptr;
        int8_t r1 = bme68x_init(&s_dev);
        s_conf.os_hum = BME68X_OS_2X; s_conf.os_temp = BME68X_OS_8X; s_conf.os_pres = BME68X_OS_4X;
        s_conf.filter = BME68X_FILTER_OFF; s_conf.odr = BME68X_ODR_NONE;
        int8_t r2 = bme68x_set_conf(&s_conf, &s_dev);
        s_heat.enable = BME68X_DISABLE; s_heat.heatr_temp = 320; s_heat.heatr_dur = 0;
        int8_t r3 = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &s_heat, &s_dev);
        // bme68x returns negatives for errors and positives for warnings (e.g.
        // W_DEFINE_SHD_HEATR_DUR); only a negative means the chip is not configured.
        s_ok = (r1 >= BME68X_OK && r2 >= BME68X_OK && r3 >= BME68X_OK);
        if (!s_ok) Serial.printf("[env] BME680 setup failed: init=%d conf=%d heatr=%d\n", r1, r2, r3);
    } else {
        Serial.printf("[env] unknown chip-id 0x%02X at 0x%02X (expect 0x60 BME280 / 0x58 BMP280 / 0x61 BME680)\n",
                      id, s_addr);
    }
}

static void env_probe() {
    Wire.begin(LEAF_BME_SDA, LEAF_BME_SCL);
    s_addr = i2c_present(0x76) ? 0x76 : (i2c_present(0x77) ? 0x77 : 0);
    if (!s_addr) { Serial.println("[env] no sensor at 0x76/0x77 (J10 empty or unpowered)"); return; }
    begin_kind(rd_reg(s_addr, 0xD0));
}

void leaf_env_log() {
    if (!s_tried) { s_tried = true; env_probe(); }
    if (!s_ok) return;

    float t = NAN, h = NAN, p = NAN;
    if (s_kind == ENV_BME280) {
        if (!s_bme.takeForcedMeasurement()) { Serial.println("[env] BME280 forced read timed out"); return; }
        t = s_bme.readTemperature(); h = s_bme.readHumidity(); p = s_bme.readPressure() / 100.0f;
    } else if (s_kind == ENV_BMP280) {
        if (!s_bmp.takeForcedMeasurement()) { Serial.println("[env] BMP280 forced read timed out"); return; }
        t = s_bmp.readTemperature(); p = s_bmp.readPressure() / 100.0f;   // no humidity sensor
    } else {
        struct bme68x_data d = {}; uint8_t nf = 0;
        if (bme68x_set_op_mode(BME68X_FORCED_MODE, &s_dev) < BME68X_OK) {
            Serial.println("[env] BME680 set_op_mode failed"); return;
        }
        uint32_t us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &s_conf, &s_dev);
        delay(us / 1000 + 5);
        if (bme68x_get_data(BME68X_FORCED_MODE, &d, &nf, &s_dev) < BME68X_OK || nf == 0) {
            Serial.println("[env] BME680 no reading (get_data error or W_NO_NEW_DATA)"); return;
        }
        t = d.temperature; h = d.humidity; p = d.pressure / 100.0f;
    }

    char hs[12];
    if (isnan(h)) strcpy(hs, "n/a"); else snprintf(hs, sizeof hs, "%.1f%%", h);
    const char* what = s_kind == ENV_BME280 ? "BME280" : s_kind == ENV_BMP280 ? "BMP280" : "BME680";
    Serial.printf("[env] %s@0x%02X T=%.2fC RH=%s P=%.1fhPa%s\n", what, s_addr, t, hs, p,
                  (isnan(t) || t < -40 || t > 85 || (!isnan(h) && (h < 0 || h > 100)) || p < 300 || p > 1100)
                      ? "  <- OUT OF RANGE" : "");
}
#else
void leaf_env_log() {}
#endif
