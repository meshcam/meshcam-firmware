/*
 * Trail-cam SURVEYOR — handheld RNS probe on the LILYGO T-Beam V1.2 (SX1276 variant).
 * Spec: docs/trailcam/surveyor.md. Radio flow: src/surveyor_radio.cpp.
 *
 * UX:
 *   button (GPIO38)  short press          -> probe (~5 KB thumb-equivalent Resource)
 *                    long press (>=1.2 s) -> big probe (2x 16 KB, the full-res path)
 *                    double press         -> toggle auto mode (probe every ~20 s idle)
 *   serial (115200)  p=probe  b=big  a=auto  d=dump CSV  s=status  wipe=clear CSV
 *
 * Every attempt (including failures, which never reach the gateway) is appended to
 * /probes.csv on LittleFS:
 *   ts,seq,part,kind,lat,lon,alt,hdop,sats,profile,bytes,ok,ms,rssi,snr
 * probe/big rows carry surveyor-side RF numbers (the gateway's proof as heard here);
 * ack rows carry the gateway-side numbers downlinked as {"kind":"probe_ack"}.
 *
 * ADR: the surveyor announces as trailcam.leaf, so the gateway's ADR machinery grants
 * it profiles exactly as it would a leaf — each spot's CSV rows record the best profile
 * that closed there. Reverts to base after 90 s of hearing nothing while off-base
 * (rendezvous rule: both ends converge on base without cooperation).
 */
#include <Arduino.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <XPowersLib.h>
#include <time.h>
#include <sys/time.h>

#include "surveyor_radio.h"

#ifndef SURVEYOR_SLUG
#define SURVEYOR_SLUG "surveyor-1"
#endif

// T-Beam V1.x pin map (confirmed against the factory Meshtastic boot log, 2026-07-10)
#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22
#define PIN_GPS_RX    34    // ESP32 RX  <- GPS TX
#define PIN_GPS_TX    12    // ESP32 TX  -> GPS RX
#define PIN_BUTTON    38    // middle button, external pull-up, active LOW

#define PROBE_BYTES        (5 * 1024)     // thumb-equivalent (ingest-api: leaf thumbs ~5 KB)
#define BIG_CHUNK_BYTES    (16 * 1024)    // the leaf's full-res chunk size
#define BIG_CHUNKS         2
#define AUTO_GAP_MS        20000          // idle gap between auto-mode probes
// Idle heartbeat: feeds gateway ADR SNR samples. JITTERED 24–37 s, never a fixed 30 s:
// the gateway's quiet-leaf profile scan toggles on an exact 30 s period, and two
// same-period clocks can phase-lock so every announce lands in the deaf half — observed
// on the bench 2026-07-10 (a whole boot of checkins unheard while the gateway sat on a
// stale sf7 grant).
static uint32_t announce_gap_ms() { return 24000 + (esp_random() % 13000); }
#define RX_WINDOW_MS       8000           // post-announce listen (grants/acks/time_sync)
#define REVERT_SILENT_MS   90000          // off-base and deaf this long -> back to base
#define CSV_PATH           "/probes.csv"

static XPowersLibInterface* s_pmu = nullptr;
// SSD1306 driver (renders unshifted on this panel; the SH1106-128 driver shifts 2 px).
// GHOST-PIXEL POSTMORTEM (2026-07-10): leftover factory-Meshtastic pixels (battery
// icon + glyph slivers) haunted the far-right columns through every reflash — they
// lived in controller RAM columns beyond every window we can address (SSD1306 driver
// writes 0-127, SH1106-128 writes 2-129, and even the raw 132-column scrub below
// missed them), and OLED RAM persists across ESP32 resets while USB keeps the 3V3
// rail up. A full POWER CYCLE cleared it. The scrub stays as best-effort hygiene for
// the addressable RAM; if edge ghosts ever reappear, pull the plug for ten seconds.
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C s_oled(U8G2_R0, U8X8_PIN_NONE,
                                                  PIN_I2C_SCL, PIN_I2C_SDA);

// Zero ALL 132 columns of every page with raw page-addressing writes (works on SH1106
// and SSD1306 alike; on a true 128-col SSD1306 the last 4 bytes wrap harmlessly).
static void oled_scrub_full_ram() {
    for (uint8_t page = 0; page < 8; page++) {
        Wire.beginTransmission(0x3C);
        Wire.write((uint8_t)0x00);            // command stream
        Wire.write((uint8_t)(0xB0 | page));   // page address
        Wire.write((uint8_t)0x00);            // column low nibble = 0
        Wire.write((uint8_t)0x10);            // column high nibble = 0
        Wire.endTransmission();
        int remaining = 132;
        while (remaining > 0) {
            Wire.beginTransmission(0x3C);
            Wire.write((uint8_t)0x40);        // data stream (column ptr auto-increments)
            for (int i = 0; i < 30 && remaining > 0; i++, remaining--)
                Wire.write((uint8_t)0x00);
            Wire.endTransmission();
        }
    }
}
static bool        s_oled_ok = false;
static TinyGPSPlus s_gps;
static Preferences s_prefs;

static uint32_t s_seq        = 0;      // NVS-persistent: correlates CSV <-> gateway telemetry
static bool     s_auto       = false;
static uint32_t s_ok_count   = 0;
static uint32_t s_try_count  = 0;
static uint32_t s_last_probe_end = 0;
static uint32_t s_last_announce  = 0;
static bool     s_clock_set  = false;  // GPS or gateway time landed
static int64_t  s_grant_until = 0;     // unix expiry of an off-base ADR grant
static char     s_last_result[28] = "no probes yet";
static char     s_last_ack[28]    = "";

// --- tiny JSON field scanners (same contract as the gateway's json_str/json_ll) --------
static bool jstr(const char* json, const char* key, char* out, size_t cap) {
    char pat[40];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return false;
    p = strchr(p + strlen(pat), ':');
    if (!p) return false;
    p++;
    while (*p == ' ') p++;
    if (*p != '"') return false;
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i + 1 < cap) out[i++] = *p++;
    out[i] = 0;
    return true;
}

static bool jnum(const char* json, const char* key, double* out) {
    char pat[40];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return false;
    p = strchr(p + strlen(pat), ':');
    if (!p) return false;
    *out = strtod(p + 1, nullptr);
    return true;
}

// --- clock: GPS is the authority when it has a fix; gateway time_sync is the fallback --
static int64_t civil_to_epoch(int y, unsigned m, unsigned d,
                              unsigned hh, unsigned mm, unsigned ss) {
    y -= m <= 2;
    const int      era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    const int64_t days = (int64_t)era * 146097 + (int64_t)doe - 719468;
    return days * 86400 + hh * 3600 + mm * 60 + ss;
}

static void set_clock(int64_t unix_s, const char* source) {
    struct timeval tv = {};
    tv.tv_sec = (time_t)unix_s;
    settimeofday(&tv, nullptr);
    s_clock_set = true;
    Serial.printf("[time] clock set from %s: %lld\n", source, (long long)unix_s);
}

static void maybe_gps_clock() {
    if (s_clock_set) return;
    if (!s_gps.date.isValid() || !s_gps.time.isValid() || s_gps.date.year() < 2026) return;
    set_clock(civil_to_epoch(s_gps.date.year(), s_gps.date.month(), s_gps.date.day(),
                             s_gps.time.hour(), s_gps.time.minute(), s_gps.time.second()),
              "GPS");
}

// --- CSV log ----------------------------------------------------------------------------
static void csv_append(const char* line) {
    File f = LittleFS.open(CSV_PATH, FILE_APPEND);
    if (!f) { Serial.println("[csv] open FAILED"); return; }
    if (f.size() == 0)
        f.println("ts,seq,part,kind,lat,lon,alt,hdop,sats,profile,bytes,ok,ms,rssi,snr");
    f.println(line);
    f.close();
    Serial.printf("[csv] %s\n", line);
}

static void csv_dump() {
    File f = LittleFS.open(CSV_PATH, "r");
    if (!f) { Serial.println("[csv] (empty)"); return; }
    Serial.println("--- probes.csv ---");
    while (f.available()) Serial.write(f.read());
    Serial.println("--- end ---");
    f.close();
}

// --- OLED --------------------------------------------------------------------------------
static void oled_draw(const char* action) {
    if (!s_oled_ok) return;
    char l[6][30];
    snprintf(l[0], 30, "%s %s%.8s", sv_radio_profile_name(),
             sv_gateway_known() ? "" : "?", sv_gateway_short());
    if (s_gps.location.isValid())
        snprintf(l[1], 30, "GPS %lus %.1f %.5f", (unsigned long)s_gps.satellites.value(),
                 s_gps.hdop.isValid() ? s_gps.hdop.hdop() : 99.9, s_gps.location.lat());
    else
        snprintf(l[1], 30, "GPS NO FIX (%lus)", (unsigned long)s_gps.satellites.value());
    snprintf(l[2], 30, "%.5f", s_gps.location.isValid() ? s_gps.location.lng() : 0.0);
    snprintf(l[3], 30, "%s", action ? action : s_last_result);
    snprintf(l[4], 30, "%s", s_last_ack[0] ? s_last_ack : "no ack yet");
    float batt = (s_pmu && s_pmu->isBatteryConnect()) ? s_pmu->getBattVoltage() / 1000.0f : 0;
    snprintf(l[5], 30, "ok %lu/%lu %.2fV%s", (unsigned long)s_ok_count,
             (unsigned long)s_try_count, batt, s_auto ? " AUTO" : "");
    s_oled.clearBuffer();
    s_oled.setFont(u8g2_font_6x10_tf);
    for (int i = 0; i < 6; i++) s_oled.drawStr(0, 10 + i * 10 + (i > 0 ? 1 : 0), l[i]);
    s_oled.sendBuffer();
}

// --- inbound gateway packets -------------------------------------------------------------
static void on_gateway_cmd(const char* json) {
    char kind[24] = "";
    if (!jstr(json, "kind", kind, sizeof(kind))) return;

    if (strcmp(kind, "radio_profile") == 0) {
        double idx = -1, ttl = 0;
        jnum(json, "idx", &idx);
        jnum(json, "ttl_s", &ttl);
        if (idx < 0 || idx >= LORA_PROFILE_COUNT) return;
        const bool changed = (uint8_t)idx != sv_radio_profile();
        if (!sv_radio_set_profile((uint8_t)idx)) {
            Serial.printf("[adr] grant idx=%d REJECTED\n", (int)idx);
            return;
        }
        s_grant_until = (int64_t)time(nullptr) + (int64_t)(ttl > 0 ? ttl : 24 * 3600);
        if (changed) {
            Serial.printf("[adr] now on %s -> confirm announce\n", sv_radio_profile_name());
            sv_radio_announce("profile");   // confirm ON the new profile (gateway reverts otherwise)
        } else {
            Serial.printf("[adr] keepalive on %s\n", sv_radio_profile_name());
        }

    } else if (strcmp(kind, "time_sync") == 0) {
        double unix_s = 0;
        if (jnum(json, "unix", &unix_s) && unix_s > 1600000000 && !s_clock_set)
            set_clock((int64_t)unix_s, "gateway");

    } else if (strcmp(kind, "probe_ack") == 0) {
        double seq = 0, rssi = 0, snr = 0, ms = 0;
        jnum(json, "seq", &seq);
        jnum(json, "rssi", &rssi);
        jnum(json, "snr", &snr);
        jnum(json, "ms", &ms);
        char row[160];
        snprintf(row, sizeof(row), "%lld,%lu,0,ack,,,,,,%s,0,1,%lu,%.0f,%.1f",
                 (long long)time(nullptr), (unsigned long)seq, sv_radio_profile_name(),
                 (unsigned long)ms, rssi, snr);
        csv_append(row);
        snprintf(s_last_ack, sizeof(s_last_ack), "ack#%lu gw %.0f/%.1f",
                 (unsigned long)seq, rssi, snr);

    } else {
        Serial.printf("[cmd] ignored kind=%s\n", kind);
    }
}

// --- the probe ----------------------------------------------------------------------------
static uint32_t probe_timeout_ms(size_t bytes) {
    const uint32_t raw = LORA_PROFILES[sv_radio_profile()].raw_bps;
    uint32_t t = (uint32_t)((uint64_t)bytes * 3000 / (raw ? raw : 122)) + 45000;
    return constrain(t, (uint32_t)60000, (uint32_t)480000);
}

static void probe(bool big) {
    s_seq++;
    s_prefs.putULong("seq", s_seq);

    // GPS snapshot at trigger time (a long transfer shouldn't smear the pin)
    const bool   fix  = s_gps.location.isValid();
    const double lat  = fix ? s_gps.location.lat() : 0.0;
    const double lon  = fix ? s_gps.location.lng() : 0.0;
    const double alt  = s_gps.altitude.isValid() ? s_gps.altitude.meters() : 0.0;
    const double hdop = s_gps.hdop.isValid() ? s_gps.hdop.hdop() : 0.0;
    const unsigned sats = s_gps.satellites.isValid() ? s_gps.satellites.value() : 0;

    const unsigned parts = big ? BIG_CHUNKS : 1;
    const size_t   bytes = big ? BIG_CHUNK_BYTES : PROBE_BYTES;
    Serial.printf("[probe] #%lu %s: %u x %u bytes on %s (fix=%d sats=%u)\n",
                  (unsigned long)s_seq, big ? "BIG" : "probe", parts, (unsigned)bytes,
                  sv_radio_profile_name(), fix, sats);

    uint8_t* body = (uint8_t*)malloc(bytes);
    if (!body) { Serial.println("[probe] OOM"); return; }

    bool all_ok = true;
    for (unsigned part = 1; part <= parts; part++) {
        char action[30];
        snprintf(action, sizeof(action), "PROBE #%lu %u/%u...",
                 (unsigned long)s_seq, part, parts);
        oled_draw(action);

        // Deterministic padding: xorshift32 seeded by seq+part (no Date/random needed,
        // incompressible enough with auto_compress off)
        uint32_t x = (s_seq << 4) ^ part ^ 0x9E3779B9u;
        for (size_t i = 0; i < bytes; i++) {
            x ^= x << 13; x ^= x >> 17; x ^= x << 5;
            body[i] = (uint8_t)x;
        }

        char hdr[256];
        snprintf(hdr, sizeof(hdr),
                 "{\"kind\":\"probe\",\"camera\":\"%s\",\"event_id\":\"prb-%lu-%u\","
                 "\"captured_at\":%lld,\"seq\":%lu,\"lat\":%.6f,\"lon\":%.6f,"
                 "\"alt\":%.1f,\"hdop\":%.2f,\"sats\":%u,\"fix\":%d,"
                 "\"part\":%u,\"parts\":%u}",
                 SURVEYOR_SLUG, (unsigned long)s_seq, part, (long long)time(nullptr),
                 (unsigned long)s_seq, lat, lon, alt, hdop, sats, fix ? 1 : 0,
                 part, parts);

        const uint32_t t0 = millis();
        const bool ok = sv_probe_send(hdr, body, bytes, probe_timeout_ms(bytes));
        const uint32_t ms = millis() - t0;
        all_ok = all_ok && ok;
        s_try_count++;
        if (ok) s_ok_count++;

        char row[220];
        snprintf(row, sizeof(row),
                 "%lld,%lu,%u,%s,%.6f,%.6f,%.1f,%.2f,%u,%s,%u,%d,%lu,%.0f,%.1f",
                 (long long)time(nullptr), (unsigned long)s_seq, part,
                 big ? "big" : "probe", lat, lon, alt, hdop, sats,
                 sv_radio_profile_name(), (unsigned)bytes, ok ? 1 : 0,
                 (unsigned long)ms, sv_last_rssi(), sv_last_snr());
        csv_append(row);
        snprintf(s_last_result, sizeof(s_last_result), "#%lu %s %.1fs %.0f/%.1f",
                 (unsigned long)s_seq, ok ? "OK" : "FAIL", ms / 1000.0f,
                 sv_last_rssi(), sv_last_snr());
        if (!ok) break;   // no point sending part 2 into a dead link
    }
    free(body);

    // Leaf-proven ordering: teardown, THEN announce (an announce first phase-locks the
    // gateway's rebroadcast into our own handshake), then listen for grants/acks.
    sv_link_teardown();
    char status[24];
    snprintf(status, sizeof(status), "probe:%lu", (unsigned long)s_seq);
    sv_radio_announce(status);
    s_last_announce = millis();
    oled_draw(nullptr);
    sv_radio_poll_commands(RX_WINDOW_MS, on_gateway_cmd);
    s_last_probe_end = millis();
    oled_draw(nullptr);
}

// --- ADR revert (rendezvous rule) ---------------------------------------------------------
static void maybe_revert_profile() {
    if (sv_radio_profile() == LORA_PROFILE_BASE) return;
    const bool deaf    = sv_ms_since_rx() > REVERT_SILENT_MS;
    const bool expired = s_grant_until && s_clock_set && time(nullptr) > s_grant_until;
    if (!deaf && !expired) return;
    Serial.printf("[adr] revert to base (%s)\n", deaf ? "deaf 90s" : "grant expired");
    sv_radio_set_profile(LORA_PROFILE_BASE);
    s_grant_until = 0;
    sv_radio_announce("checkin");   // let the gateway's scan find us on base
    s_last_announce = millis();
}

// --- button: short / long / double -------------------------------------------------------
static void button_poll() {
    static bool     was_down   = false;
    static uint32_t down_ms    = 0;
    static uint32_t release_ms = 0;
    static bool     pending    = false;   // short press waiting for a possible double
    const bool down = digitalRead(PIN_BUTTON) == LOW;
    const uint32_t now = millis();

    if (down && !was_down) {
        was_down = true;
        down_ms  = now;
        if (pending && now - release_ms < 400) {   // second press -> double
            pending = false;
            s_auto  = !s_auto;
            Serial.printf("[btn] auto mode %s\n", s_auto ? "ON" : "OFF");
            oled_draw(s_auto ? "AUTO ON" : "AUTO OFF");
            // swallow this press entirely
            while (digitalRead(PIN_BUTTON) == LOW) delay(10);
            was_down = false;
        }
    } else if (!down && was_down) {
        was_down = false;
        const uint32_t held = now - down_ms;
        if (held >= 1200) {
            pending = false;
            probe(true);
        } else if (held >= 30) {
            pending    = true;
            release_ms = now;
        }
    } else if (pending && now - release_ms >= 400) {
        pending = false;
        probe(false);
    }
}

static void serial_poll() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if      (cmd == "p" || cmd == "probe") probe(false);
    else if (cmd == "b" || cmd == "big")   probe(true);
    else if (cmd == "a" || cmd == "auto") {
        s_auto = !s_auto;
        Serial.printf("[btn] auto mode %s\n", s_auto ? "ON" : "OFF");
    }
    else if (cmd == "d" || cmd == "dump")  csv_dump();
    else if (cmd == "wipe") { LittleFS.remove(CSV_PATH); Serial.println("[csv] wiped"); }
    else if (cmd == "s" || cmd == "status") {
        Serial.printf("[status] gw=%s%s seq=%lu ok=%lu/%lu profile=%s auto=%d fix=%d "
                      "sats=%lu nmea=%lu/%lu clock=%d heap=%u\n",
                      sv_gateway_short(), sv_gateway_known() ? "" : "(?)",
                      (unsigned long)s_seq, (unsigned long)s_ok_count,
                      (unsigned long)s_try_count, sv_radio_profile_name(), s_auto,
                      s_gps.location.isValid(),
                      (unsigned long)s_gps.satellites.value(),
                      (unsigned long)s_gps.passedChecksum(),
                      (unsigned long)s_gps.charsProcessed(), s_clock_set,
                      (unsigned)ESP.getFreeHeap());
    }
    else if (cmd.length()) Serial.println("[?] p/probe b/big a/auto d/dump wipe s/status");
}

// --- bring-up: PMU first (LoRa + GPS rails are OFF until enabled), then peripherals ------
static void pmu_begin() {
    s_pmu = new XPowersAXP2101(Wire, PIN_I2C_SDA, PIN_I2C_SCL, AXP2101_SLAVE_ADDRESS);
    if (!s_pmu->init()) {
        delete s_pmu;
        s_pmu = new XPowersAXP192(Wire, PIN_I2C_SDA, PIN_I2C_SCL, AXP192_SLAVE_ADDRESS);
        if (!s_pmu->init()) {
            delete s_pmu;
            s_pmu = nullptr;
            Serial.println("[pmu] NO PMU FOUND — LoRa/GPS rails may be dead");
            return;
        }
    }
    if (s_pmu->getChipModel() == XPOWERS_AXP2101) {
        // T-Beam v1.2: ALDO2 = LoRa, ALDO3 = GPS
        s_pmu->setPowerChannelVoltage(XPOWERS_ALDO2, 3300);
        s_pmu->enablePowerOutput(XPOWERS_ALDO2);
        s_pmu->setPowerChannelVoltage(XPOWERS_ALDO3, 3300);
        s_pmu->enablePowerOutput(XPOWERS_ALDO3);
        // The 18650 holder takes UNPROTECTED 65 mm cells (protected = 68-70 mm, won't
        // fit) — this PMU *is* the protection circuit, so be explicit rather than
        // trusting silicon defaults: CC/CV to 4.20 V at a gentle 500 mA, and a 3.0 V
        // system cutoff so the cell never over-discharges in the field. The one thing
        // no chip fixes: a cell inserted BACKWARDS kills T-Beams — mind the polarity.
        s_pmu->setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);
        s_pmu->setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);
        s_pmu->setSysPowerDownVoltage(3000);
        Serial.println("[pmu] AXP2101: ALDO2 (LoRa) + ALDO3 (GPS) on; "
                       "charger 4.20V/500mA, sys cutoff 3.0V");
    } else {
        // T-Beam v1.0/1.1: LDO2 = LoRa, LDO3 = GPS, DCDC1 = OLED rail
        s_pmu->setPowerChannelVoltage(XPOWERS_LDO2, 3300);
        s_pmu->enablePowerOutput(XPOWERS_LDO2);
        s_pmu->setPowerChannelVoltage(XPOWERS_LDO3, 3300);
        s_pmu->enablePowerOutput(XPOWERS_LDO3);
        s_pmu->setPowerChannelVoltage(XPOWERS_DCDC1, 3300);
        s_pmu->enablePowerOutput(XPOWERS_DCDC1);
        Serial.println("[pmu] AXP192: LDO2 (LoRa) + LDO3 (GPS) + DCDC1 (OLED) on");
    }
    s_pmu->enableBattVoltageMeasure();
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n[surveyor] boot");

    pmu_begin();
    delay(50);   // let the rails settle before touching the radio/GPS

    s_oled_ok = s_oled.begin();
    if (s_oled_ok) oled_scrub_full_ram();
    Serial.printf("[oled] driver=SSD1306+scrub begin=%d\n", s_oled_ok);
#ifdef SURVEYOR_OLED_TEST
    // Panel-geometry test (one glance answers everything): 3 s full white flood —
    // any region that stays dark/stale is RAM the driver can't reach; then 3 s
    // border + diagonals — shows where the driver thinks the edges are.
    if (s_oled_ok) {
        const int W = s_oled.getDisplayWidth(), H = s_oled.getDisplayHeight();
        s_oled.clearBuffer();
        s_oled.drawBox(0, 0, W, H);
        s_oled.sendBuffer();
        delay(3000);
        s_oled.clearBuffer();
        s_oled.drawFrame(0, 0, W, H);
        s_oled.drawLine(0, 0, W - 1, H - 1);
        s_oled.drawLine(W - 1, 0, 0, H - 1);
        s_oled.setFont(u8g2_font_6x10_tf);
        s_oled.drawStr(34, 38, "scrubbed");
        s_oled.sendBuffer();
        delay(3000);
    }
#endif
    oled_draw("booting...");

    Serial1.setRxBufferSize(2048);   // NMEA piles up during long blocking probes
    Serial1.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    s_prefs.begin("surveyor", false);
    s_seq = s_prefs.getULong("seq", 0);

    sv_radio_begin();
    // AFTER filesystem.init(): microStore mounts LittleFS at basePath "" (root), and a
    // prior default-basePath begin() would strand the RNS store's /cache writes ("failed
    // to store identity", no path table — bit us on first bench E2E, 2026-07-10). This
    // begin is an idempotent attach at the SAME basePath, for the CSV handles.
    if (!LittleFS.begin(true, "")) Serial.println("[fs] LittleFS mount FAILED");
    sv_radio_announce("hello");   // triggers the gateway's cold-boot time_sync
    s_last_announce = millis();
    sv_radio_poll_commands(4000, on_gateway_cmd);
    oled_draw("ready");
    Serial.println("[surveyor] ready — p/b/a/d/s or the button");
}

void loop() {
    while (Serial1.available()) s_gps.encode(Serial1.read());
    maybe_gps_clock();
    button_poll();
    serial_poll();
    maybe_revert_profile();

    if (s_auto && millis() - s_last_probe_end > AUTO_GAP_MS) probe(false);

    static uint32_t announce_gap = 24000;
    if (millis() - s_last_announce > announce_gap) {
        sv_radio_announce("checkin");
        s_last_announce = millis();
        announce_gap = announce_gap_ms();
        sv_radio_poll_commands(RX_WINDOW_MS, on_gateway_cmd);
    }

    sv_radio_poll_commands(200, on_gateway_cmd);

    static uint32_t last_oled = 0;
    if (millis() - last_oled > 1000) {
        last_oled = millis();
        oled_draw(nullptr);
    }
}
