// Own translation unit, not an .inc pulled into main.cpp — same reason as bme_probe.cpp:
// keeps the probe standalone (and this TU includes esp_camera.h, which is exactly the
// header main.cpp's include set collides around).
#ifdef LEAF_V1_PROBE
#include <Arduino.h>
#include <Wire.h>
#include <SD_MMC.h>
#include "esp_camera.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "mbedtls/base64.h"
#include "leaf_board.h"

// First-article overnight probe for the CUSTOM LEAF PCB v1.0 (JLC W2026082103216493,
// 5 boards, first powered 2026-08-28). Everything v1.0 changed vs v0.6 gets exercised,
// nothing RF-hazardous ever runs:
//
//  - NEVER sleeps: the native USB console stays enumerated all night, which is both the
//    v1.0 USB-ground-fix soak (v0.6's endemic flakiness, bringup §3e) and the headless
//    re-flash guarantee (sleeping firmware is a one-way door, bringup §3b).
//  - RAK3172 (U9, soldered, stock RUI3 AT firmware) on UART2: every PASS_S the LORA
//    rail (Q2, IO44 active-low) is power-cycled, the module reset via NRST (IO1), and
//    AT queries run (AT, VER, HWMODEL, HWID, SN, BAND, SYSV), then — since a 915 MHz
//    antenna went on J9 the first evening (2026-08-28); never TX into an open u.FL —
//    three P2P beacons on the grant lane at 5/14/22 dBm with SYSV after each (see
//    rak_pass). Each cycle also exercises the TPS2116/Q2 power path.
//  - CAMERA through J2's v1.0 MIRRORED net map — the first capture attempt through a
//    leaf PCB's own FFC connector ever (v0.6 shipped J2 reversed). Capture
//    ladder QVGA -> SVGA -> UXGA each pass: UXGA is the exact load that hard-reset
//    v0.6 12/12 (§16), so UXGA surviving on the TLV75733 + soft-started Q1 is the
//    power-path verdict. A reset here shows up in the soak log as a fresh boot banner.
//  - microSD as SDMMC 1-bit (CLK=47 CMD=48 D0=38 — the bench assignment, now
//    the routed v1.0 layout). Mounted once per pass while the rail is up; the UXGA
//    frame is written to /v1probe/passNNNN.jpg. No card inserted logs a clean failure.
//  - I2C scan on 40/41 + chip-ID read at 0x76/0x77 (J10 may be empty — a finding, not
//    an error).
//  - Heartbeat every HB_S with uptime, heap, PSRAM, die temp, VBAT divider (no cell
//    fitted -> expect ~0 mV; the board runs from USB through the TPS2116, itself a
//    v1.0 first), PIR line level (J3 likely empty -> floating).
//  - Serial command 'D' + newline dumps the latest saved UXGA jpg as base64 so image
//    CONTENT can be verified over the console without pulling the card.
//
// Rail-cut discipline (respin spec §7): before IO44 goes high, the UART pins are
// released to inputs and NRST driven low, so no ESP32 push-pull output back-powers the
// unpowered module through its protection diodes.

#ifndef LEAF_BAUD
#define LEAF_BAUD 460800
#endif
#define PIN_RAK_TX     14   // ESP32 out -> RAK UART2 RX
#define PIN_RAK_RX      2   // ESP32 in  <- RAK UART2 TX
#define PIN_RAK_NRST    1
#define PIN_RAK_BOOT0  42   // held LOW: boot RUI3 from flash, never the STM32 bootloader
#define PIN_RAIL_LORA  44   // Q2 gate, LOW = rail on (radio + microSD)
#define PIN_RAIL_CAM   39   // Q1 gate, LOW = rail on (camera, v1.0 RC soft-start)
#define PIN_VBAT        3
#define PIN_PIR        21
#define PIN_SD_CLK     47
#define PIN_SD_CMD     48
#define PIN_SD_D0      38

#define HB_S           30
#define PASS_S        600

static uint32_t s_pass = 0, s_rak_ok = 0, s_sd_ok = 0, s_cam_ok = 0, s_uxga_ok = 0,
                s_tx_ok = 0;
static char     s_id[8] = "??";   // last two MAC bytes — beacon identity, set in print_banner
static char     s_last_jpg[32] = "";

static void print_banner() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_id, sizeof(s_id), "%02X%02X", mac[4], mac[5]);
    Serial.printf("\n[v1-probe] 0.1.0  leaf PCB v1.0 first-article overnight probe\n");
    Serial.printf("[v1-probe] mac=%02X:%02X:%02X:%02X:%02X:%02X reset_reason=%d\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], (int)esp_reset_reason());
    Serial.printf("[v1-probe] psram=%u free=%u  heap_free=%u\n",
                  (unsigned)ESP.getPsramSize(), (unsigned)ESP.getFreePsram(),
                  (unsigned)ESP.getFreeHeap());
}

// ---- RAK3172 -------------------------------------------------------------------------

// Collect UART output until quiet for `quiet_ms` or `total_ms` elapses; print each line
// with a tag so the soak log parser can grep per-command.
static String rak_drain(const char* tag, uint32_t total_ms, uint32_t quiet_ms = 250) {
    String out;
    uint32_t start = millis(), last_rx = millis();
    while (millis() - start < total_ms) {
        while (Serial1.available()) {
            out += (char)Serial1.read();
            last_rx = millis();
        }
        if (out.length() && millis() - last_rx > quiet_ms) break;
        delay(5);
    }
    if (!out.length()) {
        Serial.printf("[rak] %s: (no response)\n", tag);
        return out;
    }
    int from = 0;
    while (from < (int)out.length()) {
        int nl = out.indexOf('\n', from);
        if (nl < 0) nl = out.length();
        String line = out.substring(from, nl);
        line.trim();
        if (line.length()) Serial.printf("[rak] %s: %s\n", tag, line.c_str());
        from = nl + 1;
    }
    return out;
}

static bool rak_cmd(const char* cmd, uint32_t timeout_ms = 1200) {
    while (Serial1.available()) Serial1.read();
    Serial1.print(cmd);
    Serial1.print("\r\n");
    String r = rak_drain(cmd, timeout_ms);
    return r.indexOf("OK") >= 0 || r.indexOf("=") >= 0;   // query replies echo "CMD=value"
}

static void rak_pass() {
    digitalWrite(PIN_RAK_NRST, LOW);          // hold in reset through rail-up
    delay(20);
    Serial1.begin(115200, SERIAL_8N1, PIN_RAK_RX, PIN_RAK_TX);
    delay(20);
    digitalWrite(PIN_RAK_NRST, HIGH);         // release: RUI3 boots, prints its banner
    String banner = rak_drain("boot", 2500);

    bool ok = rak_cmd("AT");
    rak_cmd("AT+VER=?");
    rak_cmd("AT+HWMODEL=?");
    rak_cmd("AT+HWID=?");
    rak_cmd("AT+SN=?");
    rak_cmd("AT+BAND=?");
    rak_cmd("AT+SYSV=?");                     // module's own rail reading — free voltmeter
    if (ok || banner.length()) s_rak_ok++;

    // TX rail-stress, enabled 2026-08-28 once a 915 MHz antenna went on J9 (never TX
    // into an open u.FL). Three P2P beacons per pass on the grant's DTS lane (903.9 MHz
    // SF8/BW500 — the fcc-certification.md Finding 3d config the 08-07 interop ran) at
    // 5 -> 14 -> 22 dBm, SYSV after each: the 22 dBm burst is ~120 mA through the whole
    // v1.0 power path, the exact load class that boot-looped v0.6 (bringup §3c). RUI3
    // itself rejects >22 dBm, so the grant ceiling cannot be exceeded from here.
    rak_cmd("AT+NWM=0", 4000);                // P2P mode; reboots the module first time
    rak_drain("nwm-settle", 3000);
    rak_cmd("AT+PFREQ=903900000");
    rak_cmd("AT+PSF=8");
    rak_cmd("AT+PBW=2");
    rak_cmd("AT+PCR=0");
    rak_cmd("AT+PPL=8");
    static const int POWERS[3] = {5, 14, 22};
    for (int i = 0; i < 3; i++) {
        char cmd[96], payload[40], hex[81];
        snprintf(cmd, sizeof(cmd), "AT+PTP=%d", POWERS[i]);
        rak_cmd(cmd);
        // MAC-derived id, not a hardcoded serial: boards 2-5 run this same build
        // and must be tellable apart in rx.log.
        snprintf(payload, sizeof(payload), "MC-V1-%s p%u tx%ddBm",
                 s_id, (unsigned)s_pass, POWERS[i]);
        for (size_t k = 0; k < strlen(payload); k++)
            snprintf(hex + 2 * k, 3, "%02X", (unsigned char)payload[k]);
        snprintf(cmd, sizeof(cmd), "AT+PSEND=%s", hex);
        while (Serial1.available()) Serial1.read();
        Serial1.print(cmd); Serial1.print("\r\n");
        String r = rak_drain(cmd, 3000, 400);  // wait past OK for +EVT:TXP2P DONE
        if (r.indexOf("TXP2P") >= 0) s_tx_ok++;
        rak_cmd("AT+SYSV=?");                  // did the rail sag or the module reset?
    }

    // Rail-cut discipline: nothing may drive into the soon-unpowered module.
    Serial1.end();
    pinMode(PIN_RAK_TX, INPUT);
    pinMode(PIN_RAK_RX, INPUT);
    digitalWrite(PIN_RAK_NRST, LOW);
}

// ---- I2C -----------------------------------------------------------------------------

static void i2c_pass() {
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            found++;
            Serial.printf("[i2c] device at 0x%02X", a);
            if (a == 0x76 || a == 0x77) {   // Bosch chip-ID register 0xD0
                Wire.beginTransmission(a); Wire.write(0xD0);
                if (Wire.endTransmission(false) == 0 && Wire.requestFrom((int)a, 1) == 1) {
                    uint8_t id = Wire.read();
                    Serial.printf("  chip-id 0x%02X (%s)", id,
                                  id == 0x60 ? "BME280" : id == 0x58 ? "BMP280"
                                : id == 0x61 ? "BME680" : "?");
                }
            }
            Serial.println();
        }
    }
    if (!found) Serial.println("[i2c] scan: no devices (J10 empty is expected on a bare board)");
}

// ---- camera --------------------------------------------------------------------------

static bool cam_grab(const char* tag, framesize_t size, int quality, bool save_to_sd, bool sd_up) {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) return false;
    camera_sensor_info_t* info = esp_camera_sensor_get_info(&s->id);
    if (info && size > info->max_size) size = info->max_size;   // OV2640 max is UXGA
    s->set_framesize(s, size);
    if (s->set_quality) s->set_quality(s, quality);
    for (int i = 0; i < 2; i++) {                                // AEC settle
        camera_fb_t* warm = esp_camera_fb_get();
        if (warm) esp_camera_fb_return(warm);
    }
    uint32_t t0 = millis();
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.printf("[cam] %s: fb_get returned null\n", tag);
        return false;
    }
    Serial.printf("[cam] %s: %ux%u %u bytes in %lums (starts %02X%02X)\n", tag,
                  fb->width, fb->height, (unsigned)fb->len, millis() - t0,
                  fb->buf[0], fb->buf[1]);
    if (save_to_sd && sd_up) {
        char path[32];
        snprintf(path, sizeof(path), "/v1probe/p%04u.jpg", (unsigned)s_pass);
        File f = SD_MMC.open(path, FILE_WRITE);
        if (f) {
            size_t w = f.write(fb->buf, fb->len);
            f.close();
            if (w == fb->len) {
                strncpy(s_last_jpg, path, sizeof(s_last_jpg) - 1);
                Serial.printf("[cam] %s: saved %s\n", tag, path);
            }
        } else {
            Serial.printf("[cam] %s: SD open(%s) failed\n", tag, path);
        }
    }
    esp_camera_fb_return(fb);
    return true;
}

// The v0.6 killer, replayed deliberately: rail up -> init -> QVGA -> SVGA -> UXGA.
static void cam_pass(bool sd_up) {
    digitalWrite(PIN_RAIL_CAM, LOW);          // Q1 on; v1.0 RC soft-start needs a beat
    delay(250);
    camera_config_t c = {};
    c.pin_pwdn  = CAM_PIN_PWDN;  c.pin_reset = CAM_PIN_RESET; c.pin_xclk = CAM_PIN_XCLK;
    c.pin_sccb_sda = CAM_PIN_SIOD; c.pin_sccb_scl = CAM_PIN_SIOC;
    c.pin_d7 = CAM_PIN_D7; c.pin_d6 = CAM_PIN_D6; c.pin_d5 = CAM_PIN_D5; c.pin_d4 = CAM_PIN_D4;
    c.pin_d3 = CAM_PIN_D3; c.pin_d2 = CAM_PIN_D2; c.pin_d1 = CAM_PIN_D1; c.pin_d0 = CAM_PIN_D0;
    c.pin_vsync = CAM_PIN_VSYNC; c.pin_href = CAM_PIN_HREF; c.pin_pclk = CAM_PIN_PCLK;
    c.xclk_freq_hz = 20000000;
    c.ledc_timer = LEDC_TIMER_0; c.ledc_channel = LEDC_CHANNEL_0;
    c.pixel_format = PIXFORMAT_JPEG;
    c.frame_size   = FRAMESIZE_UXGA;          // largest we'll grab; driver sizes fb from this
    c.jpeg_quality = 12;
    c.fb_count     = 1;
    c.fb_location  = CAMERA_FB_IN_PSRAM;
    c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
    esp_err_t err = esp_camera_init(&c);
    if (err != ESP_OK) {
        Serial.printf("[cam] esp_camera_init failed: 0x%x (no module, or J2 map wrong)\n", err);
        digitalWrite(PIN_RAIL_CAM, HIGH);
        return;
    }
    sensor_t* s = esp_camera_sensor_get();
    if (s) Serial.printf("[cam] sensor PID=0x%04x\n", s->id.PID);
    s_cam_ok++;
    bool any = false;
    any |= cam_grab("qvga", FRAMESIZE_QVGA, 14, false, sd_up);
    any |= cam_grab("svga", FRAMESIZE_SVGA, 12, false, sd_up);
    if (cam_grab("uxga", FRAMESIZE_UXGA, 12, true, sd_up)) { any = true; s_uxga_ok++; }
    (void)any;
    esp_camera_deinit();
    digitalWrite(PIN_RAIL_CAM, HIGH);         // rail off between passes
}

// ---- microSD -------------------------------------------------------------------------

static bool sd_mount() {
    SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0);
    if (!SD_MMC.begin("/sdcard", /*mode1bit=*/true, /*format_if_mount_failed=*/false)) {
        Serial.println("[sd] mount FAILED (no card, exFAT, or defect — insert a known FAT32 card to tell which)");
        SD_MMC.end();
        return false;
    }
    Serial.printf("[sd] mounted OK: type=%d size=%llu MB\n",
                  (int)SD_MMC.cardType(), SD_MMC.cardSize() / (1024ULL * 1024ULL));
    if (!SD_MMC.exists("/v1probe")) SD_MMC.mkdir("/v1probe");
    s_sd_ok++;
    return true;
}

// Dump the newest saved jpg as base64 (console-side: sed -n between markers, b64 -d).
static void dump_last_jpg() {
    if (!s_last_jpg[0]) { Serial.println("[dump] nothing saved yet"); return; }
    digitalWrite(PIN_RAIL_LORA, LOW);
    delay(150);
    if (!sd_mount()) { digitalWrite(PIN_RAIL_LORA, HIGH); return; }
    File f = SD_MMC.open(s_last_jpg, FILE_READ);
    if (!f) {
        Serial.printf("[dump] open %s failed\n", s_last_jpg);
    } else {
        Serial.printf("[dump] BEGIN %s %u bytes\n", s_last_jpg, (unsigned)f.size());
        uint8_t raw[600];                      // multiple of 3 -> no mid-stream padding
        unsigned char b64[820];
        int n;
        while ((n = f.read(raw, sizeof(raw))) > 0) {
            size_t olen = 0;
            mbedtls_base64_encode(b64, sizeof(b64), &olen, raw, n);
            Serial.write(b64, olen);
            Serial.println();
        }
        f.close();
        Serial.println("[dump] END");
    }
    SD_MMC.end();
    digitalWrite(PIN_RAIL_LORA, HIGH);
}

// ---- pass driver ---------------------------------------------------------------------

static void probe_pass() {
    s_pass++;
    Serial.printf("[pass %u] LORA rail ON\n", (unsigned)s_pass);
    digitalWrite(PIN_RAK_NRST, LOW);
    digitalWrite(PIN_RAIL_LORA, LOW);         // Q2 on: radio + microSD
    delay(150);

    rak_pass();
    bool sd_up = sd_mount();
    cam_pass(sd_up);                          // capture + save while the card is mounted
    i2c_pass();

    if (sd_up) SD_MMC.end();
    digitalWrite(PIN_RAIL_LORA, HIGH);        // Q2 off
    Serial.printf("[pass %u] rail OFF. tallies: rak_ok=%u tx_ok=%u/%u sd_ok=%u cam_ok=%u uxga_ok=%u of %u\n",
                  (unsigned)s_pass, (unsigned)s_rak_ok, (unsigned)s_tx_ok,
                  (unsigned)(s_pass * 3), (unsigned)s_sd_ok,
                  (unsigned)s_cam_ok, (unsigned)s_uxga_ok, (unsigned)s_pass);
}

void setup() {
    Serial.begin(LEAF_BAUD);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 5000) delay(10);

    pinMode(PIN_RAIL_CAM, OUTPUT);  digitalWrite(PIN_RAIL_CAM, HIGH);   // camera rail off
    pinMode(PIN_RAIL_LORA, OUTPUT); digitalWrite(PIN_RAIL_LORA, HIGH);  // radio rail off
    pinMode(PIN_RAK_BOOT0, OUTPUT); digitalWrite(PIN_RAK_BOOT0, LOW);
    pinMode(PIN_RAK_NRST, OUTPUT);  digitalWrite(PIN_RAK_NRST, LOW);
    pinMode(PIN_PIR, INPUT);

    print_banner();
    Wire.begin(40, 41);
    analogReadResolution(12);

    probe_pass();                   // first pass immediately so plugging in shows life
}

void loop() {
    static uint32_t last_hb = 0, last_pass = 0;
    if (Serial.available()) {
        int ch = Serial.read();
        if (ch == 'D' || ch == 'd') dump_last_jpg();
        // On-demand pass for bench work (RX listener, battery tests): don't wait
        // out the 10 min timer.
        if (ch == 'P' || ch == 'p') { probe_pass(); last_pass = millis(); }
    }
    if (millis() - last_hb >= HB_S * 1000UL) {
        last_hb = millis();
        uint32_t vbat_div_mv = analogReadMilliVolts(PIN_VBAT);
        Serial.printf("[hb] up=%lus pass=%u heap=%u psram=%u tempC=%.1f vbat_mV=%u (x2=%u) pir=%d\n",
                      millis() / 1000UL, (unsigned)s_pass, (unsigned)ESP.getFreeHeap(),
                      (unsigned)ESP.getFreePsram(), temperatureRead(),
                      (unsigned)vbat_div_mv, (unsigned)(vbat_div_mv * 2), digitalRead(PIN_PIR));
    }
    if (millis() - last_pass >= PASS_S * 1000UL) {
        last_pass = millis();
        probe_pass();
    }
    delay(50);
}
#endif // LEAF_V1_PROBE
