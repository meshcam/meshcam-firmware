// Own translation unit, not an .inc pulled into main.cpp: main.cpp includes esp_camera.h,
// whose sensor_t collides with Adafruit_Sensor's sensor_t (same reason leaf_env.cpp is separate).
#ifdef LEAF_BME_PROBE
#include <Arduino.h>
#ifndef LEAF_BAUD
#define LEAF_BAUD 460800
#endif
// BME280 bring-up probe for the CUSTOM LEAF PCB (v0.6) — step 1 + 2 of the 9-step gate
// (homelab docs/trailcam/leaf-pcb-bringup.md §5): I2C scan finds the module in J10, then
// sane T/RH/P. Standalone: no camera, no radio, no SD, NEVER sleeps, so the native USB
// console stays enumerated the whole time (the full leaf-pcb firmware deep-sleeps every
// LEAF_TEST_INTERVAL_S and drops the port with it — useless for watching a sensor).
//
// What it prints, and why in this order:
//  1. Idle line levels on SDA(40)/SCL(41) with the S3's weak pulldown enabled. The board
//     has 10k pull-ups on the always-on 3V3 rail; if a line reads LOW here the pull-up
//     is missing/open or the socket is unpowered — and the scan that follows will find
//     nothing, for that reason and not because the module is bad.
//  2. Full 7-bit bus scan. A GY-BME280 answers at 0x76 (SDO low, the module default) or
//     0x77 (SDO high). Anything else answering means the wrong module is in J10 (the
//     look-alike GY-BMP280 uses the same addresses but a different chip ID, see 3).
//  3. Chip ID register 0xD0: 0x60 = BME280 (has humidity), 0x58 = BMP280 (no humidity —
//     the Adafruit driver would refuse it and RH would be garbage), 0x61 = BME680.
//  4. Adafruit_BME280::begin at whichever address answered, then a reading every 2 s,
//     forever. Breathe on the module: RH must jump and T rise ~1 C within a few samples.
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME680.h>   // pulled in for Bosch's bme68x.c; the Adafruit wrapper itself never configured the
                               // chip on this board (ctrl_meas stayed 0x00, raw regs at reset values), so drive bme68x directly
extern "C" {
#include "bme68x.h"
}   // fallback: read the look-alike so T/P still prove the bus end-to-end

#ifndef LEAF_BME_SDA
#define LEAF_BME_SDA 40
#endif
#ifndef LEAF_BME_SCL
#define LEAF_BME_SCL 41
#endif

static Adafruit_BME280 s_bme;
static Adafruit_BMP280 s_bmp;
static struct bme68x_dev s_dev;
static struct bme68x_conf s_conf;
static struct bme68x_heatr_conf s_heat;
static uint8_t s_dev_addr;
static int8_t bx_read(uint8_t reg, uint8_t* data, uint32_t len, void*) {
    Wire.beginTransmission(s_dev_addr); Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return -1;
    if (Wire.requestFrom((int)s_dev_addr, (int)len) != (int)len) return -1;
    for (uint32_t i = 0; i < len; i++) data[i] = Wire.read();
    return 0;
}
static int8_t bx_write(uint8_t reg, const uint8_t* data, uint32_t len, void*) {
    Wire.beginTransmission(s_dev_addr); Wire.write(reg);
    for (uint32_t i = 0; i < len; i++) Wire.write(data[i]);
    return Wire.endTransmission() == 0 ? 0 : -1;
}
static void bx_delay(uint32_t us, void*) { delayMicroseconds(us); }
enum Kind { NONE, BME280, BMP280, BME680 };
static Kind s_kind = NONE;
static bool s_is_bmp = false;   // kept for the BMP280 branch below
static uint8_t s_addr = 0;
static bool s_ok = false;

static const char* lvl(int pin) { return digitalRead(pin) ? "HIGH" : "LOW"; }

static uint8_t rd_reg(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    if (Wire.requestFrom((int)addr, 1) != 1) return 0xFF;
    return Wire.read();
}

static void bus_check() {
    // Weak pulldown vs the board's 10k pull-up: a real pull-up still reads HIGH.
    pinMode(LEAF_BME_SDA, INPUT_PULLDOWN);
    pinMode(LEAF_BME_SCL, INPUT_PULLDOWN);
    delay(5);
    Serial.printf("[bus] SDA(IO%d)=%s  SCL(IO%d)=%s  (weak pulldown on; HIGH = board pull-up present)\n",
                  LEAF_BME_SDA, lvl(LEAF_BME_SDA), LEAF_BME_SCL, lvl(LEAF_BME_SCL));
    if (!digitalRead(LEAF_BME_SDA) || !digitalRead(LEAF_BME_SCL))
        Serial.println("[bus] !! a line has NO pull-up: R-pullup open, or J10 VIN unpowered / module absent"
                       " (GY-BME280 carries its own 10k pull-ups, so even a bare socket should read HIGH"
                       " if the board's own resistors are fitted)");
    pinMode(LEAF_BME_SDA, INPUT);
    pinMode(LEAF_BME_SCL, INPUT);
}

static uint8_t bus_scan() {
    uint8_t found = 0, n = 0;
    Serial.print("[scan] ");
    for (uint8_t a = 0x08; a < 0x78; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.printf("0x%02X ", a);
            n++;
            if (a == 0x76 || a == 0x77) found = a;
        }
    }
    Serial.printf("%s(%u device%s)\n", n ? "" : "nothing answered ", n, n == 1 ? "" : "s");
    return found;
}

static bool wr_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
}
// Bypass the driver: poke forced mode straight into the chip and see if it measures.
static void raw_forced_test(uint8_t addr) {
    Serial.printf("[raw] write ctrl_hum 0x72=0x01 -> ack=%d readback=0x%02X\n", wr_reg(addr,0x72,0x01), rd_reg(addr,0x72));
    Serial.printf("[raw] write ctrl_meas 0x74=0x25 (osrs_t=1,osrs_p=1,forced) -> ack=%d readback=0x%02X\n", wr_reg(addr,0x74,0x25), rd_reg(addr,0x74));
    delay(200);
    Serial.printf("[raw] after 200ms: status=0x%02X ctrl_meas=0x%02X raw P=%02X%02X%02X T=%02X%02X%02X H=%02X%02X\n",
        rd_reg(addr,0x1D), rd_reg(addr,0x74), rd_reg(addr,0x1F), rd_reg(addr,0x20), rd_reg(addr,0x21),
        rd_reg(addr,0x22), rd_reg(addr,0x23), rd_reg(addr,0x24), rd_reg(addr,0x25), rd_reg(addr,0x26));
    Serial.printf("[raw] calib sample: 0x8A..0x8D = %02X %02X %02X %02X   0xE1..0xE4 = %02X %02X %02X %02X\n",
        rd_reg(addr,0x8A), rd_reg(addr,0x8B), rd_reg(addr,0x8C), rd_reg(addr,0x8D),
        rd_reg(addr,0xE1), rd_reg(addr,0xE2), rd_reg(addr,0xE3), rd_reg(addr,0xE4));
}

static void begin_kind(uint8_t id) {
    if (id == 0x58) {
        s_kind = BMP280; s_is_bmp = true;
        s_ok = s_bmp.begin(s_addr, 0x58);
        Serial.printf("[bme] BMP280 fallback begin -> %s (T/P only; RH prints n/a)\n", s_ok ? "OK" : "FAILED");
    } else if (id == 0x61) {
        s_kind = BME680;
        raw_forced_test(s_addr);
        s_dev_addr = s_addr;
        s_dev.intf = BME68X_I2C_INTF; s_dev.read = bx_read; s_dev.write = bx_write; s_dev.delay_us = bx_delay;
        s_dev.amb_temp = 25; s_dev.intf_ptr = nullptr;
        int8_t r1 = bme68x_init(&s_dev);
        s_conf.os_hum = BME68X_OS_2X; s_conf.os_temp = BME68X_OS_8X; s_conf.os_pres = BME68X_OS_4X;
        s_conf.filter = BME68X_FILTER_OFF; s_conf.odr = BME68X_ODR_NONE;
        int8_t r2 = bme68x_set_conf(&s_conf, &s_dev);
        // Gas heater OFF on purpose. With run_gas enabled this module comes back with ctrl_meas
        // = 0x00 after the forced conversion (oversampling bits cleared), so T/P/H are
        // "skipped" and decode to the reset constants 34.80 C / 100 % / 613.6 hPa — the exact
        // frozen numbers both the Adafruit wrapper and bme68x-direct produced. Heater off,
        // ctrl_meas holds 0x6C and the readings are real. MeshCam only wants T/RH/P anyway.
        s_heat.enable = BME68X_DISABLE; s_heat.heatr_temp = 320; s_heat.heatr_dur = 0;
        int8_t r3 = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &s_heat, &s_dev);
        s_ok = (r1 == 0 && r2 == 0 && r3 == 0);
        Serial.printf("[bme] BME680 via bme68x: init=%d set_conf=%d heatr=%d -> %s (T/RH/P + gas)\n", r1, r2, r3, s_ok ? "OK" : "FAILED");
        Serial.printf("[bme] after set_conf: ctrl_hum(0x72)=0x%02X ctrl_meas(0x74)=0x%02X config(0x75)=0x%02X ctrl_gas1(0x71)=0x%02X  (expect 0x02, 0x6C|mode, 0x00, 0x10)\n",
                      rd_reg(s_addr,0x72), rd_reg(s_addr,0x74), rd_reg(s_addr,0x75), rd_reg(s_addr,0x71));
        // Chip ignoring the multi-pair write? Re-issue as plain single-register writes and read back.
        wr_reg(s_addr, 0x72, 0x02); wr_reg(s_addr, 0x74, 0x6C);
        Serial.printf("[bme] after single writes: ctrl_hum=0x%02X ctrl_meas=0x%02X\n", rd_reg(s_addr,0x72), rd_reg(s_addr,0x74));
    } else {
        s_kind = BME280;
        s_ok = s_bme.begin(s_addr, &Wire);
        Serial.printf("[bme] Adafruit BME280 begin -> %s\n", s_ok ? "OK" : "FAILED");
    }
    if (s_ok) Serial.println("[bme] reading every 2 s; breathe on the module — RH must jump (BME280/680)");
}

void setup() {
    Serial.begin(LEAF_BAUD);
    delay(2500);   // native USB-CDC: give the host time to open the port
    Serial.println("\n==================================================");
    Serial.println("[bme] leaf PCB BME280 probe (J10, I2C SDA=40 SCL=41)");
    Serial.println("==================================================");
    bus_check();
    Wire.begin(LEAF_BME_SDA, LEAF_BME_SCL, 100000);
    s_addr = bus_scan();
    if (!s_addr) {
        Serial.println("[bme] no device at 0x76/0x77 -> check J10 orientation (VIN/GND/SCL/SDA silk) and 3V3 at the module");
        return;
    }
    uint8_t id = rd_reg(s_addr, 0xD0);
    const char* what = id == 0x60 ? "BME280 (T/RH/P)"
                     : id == 0x58 ? "BMP280 (T/P only — NO humidity; wrong module)"
                     : id == 0x61 ? "BME680"
                     : id == 0xFF ? "no reply to register read"
                     : "unknown";
    Serial.printf("[bme] addr 0x%02X chip-id 0x%02X = %s\n", s_addr, id, what);
    begin_kind(id);
}

void loop() {
    static uint32_t n = 0;
    if (s_ok) {
        float t, h, p;
        float gas = NAN;
        if (s_kind == BMP280)      { t = s_bmp.readTemperature(); h = NAN; p = s_bmp.readPressure() / 100.0f; }
        else if (s_kind == BME680) {
            struct bme68x_data d = {}; uint8_t nf = 0;
            int8_t r = bme68x_set_op_mode(BME68X_FORCED_MODE, &s_dev);
            uint32_t us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &s_conf, &s_dev) + s_heat.heatr_dur * 1000;
            delay(us / 1000 + 5);
            static int dbg = 0;
            if (dbg++ < 3) Serial.printf("[b680] waited %lu ms; ctrl_meas now 0x%02X (0x00 here = conversion skipped); ", (unsigned long)(us / 1000 + 5), rd_reg(s_addr, 0x74));
            int8_t r2 = bme68x_get_data(BME68X_FORCED_MODE, &d, &nf, &s_dev);
            if (dbg <= 3) Serial.printf("[b680] set_mode=%d get_data=%d n=%u status=0x%02X ", r, r2, nf, d.status);
            static int once = 0;
            if (once++ < 1) {
                struct bme68x_calib_data* c = &s_dev.calib;
                Serial.printf("\n[calib] t1=%u t2=%d t3=%d | p1=%u p2=%d p3=%d p4=%d p5=%d p6=%d p7=%d p8=%d p9=%d p10=%u | h1=%u h2=%u h3=%d h4=%d h5=%d h6=%u h7=%d | gh1=%d gh2=%d gh3=%d res_heat_range=%u res_heat_val=%d range_sw_err=%d\n",
                    c->par_t1, c->par_t2, c->par_t3, c->par_p1, c->par_p2, c->par_p3, c->par_p4, c->par_p5, c->par_p6, c->par_p7, c->par_p8, c->par_p9, c->par_p10,
                    c->par_h1, c->par_h2, c->par_h3, c->par_h4, c->par_h5, c->par_h6, c->par_h7, c->par_gh1, c->par_gh2, c->par_gh3, c->res_heat_range, c->res_heat_val, c->range_sw_err);
                uint8_t b[23]; bx_read(0x8A, b, 23, nullptr);
                Serial.print("[calib] raw 0x8A..: "); for (int i = 0; i < 23; i++) Serial.printf("%02X ", b[i]);
                uint8_t c2[14]; bx_read(0xE1, c2, 14, nullptr);
                Serial.print(" | 0xE1..: "); for (int i = 0; i < 14; i++) Serial.printf("%02X ", c2[i]);
                uint8_t c3[5]; bx_read(0x00, c3, 5, nullptr);
                Serial.print(" | 0x00..: "); for (int i = 0; i < 5; i++) Serial.printf("%02X ", c3[i]);
                Serial.println();
                uint8_t rw[10]; bx_read(0x1D, rw, 10, nullptr);
                Serial.print("[raw] 0x1D..0x26: "); for (int i = 0; i < 10; i++) Serial.printf("%02X ", rw[i]); Serial.println();
            }
            // Raw regs: 0x1D meas_status (bit7 new_data), 0x1F-0x21 press, 0x22-0x24 temp, 0x25-0x26 hum, 0x74 ctrl_meas
            t = d.temperature; h = d.humidity; p = d.pressure / 100.0f; gas = d.gas_resistance / 1000.0f; }
        else                       { t = s_bme.readTemperature(); h = s_bme.readHumidity(); p = s_bme.readPressure() / 100.0f; }
        if (!isnan(gas)) Serial.printf("[bme] gas=%.1f kOhm  ", gas);
        char hs[16]; if (isnan(h)) strcpy(hs, "n/a"); else snprintf(hs, sizeof hs, "%.1f%%", h);
        Serial.printf("[bme] #%lu T=%.2fC (%.1fF) RH=%s P=%.1fhPa%s\n",
                      (unsigned long)++n, t, t * 9 / 5 + 32, hs, p,
                      (isnan(t) || t < -40 || t > 85 || (!isnan(h) && (h < 0 || h > 100)) || p < 300 || p > 1100)
                          ? "  <- OUT OF RANGE" : "");
    } else {
        Serial.printf("[bme] #%lu no sensor; rescanning\n", (unsigned long)++n);
        s_addr = bus_scan();
        if (s_addr) begin_kind(rd_reg(s_addr, 0xD0));
    }
    delay(2000);
}
#endif // LEAF_BME_PROBE
