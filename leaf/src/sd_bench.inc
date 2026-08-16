// microSD write-throughput bench for the CUSTOM LEAF PCB (v0.6).
//
// WHY THIS EXISTS
// The v1.0 respin has to decide how it routes the microSD: keep SPI mode (4 pins), SDMMC
// 1-bit (3 pins — net frees one), or spend two more scarce GPIOs on SDMMC 4-bit. Spare
// digital GPIOs are contested (CHG_STAT/DONE_STAT want them), so "4-bit is
// faster" is not an argument by itself — the question is whether the SLOWEST option
// already sustains the MJPEG clip the product wants. This measures it, not guesses it.
//
// WHAT IT MEASURES, AND WHY EACH PART
//  1. Sequential write MB/s — the headline number, but not the one that decides clips.
//  2. WORST single-chunk latency — a card's internal wear-levelling/GC can stall a write
//     for tens or hundreds of ms. An MJPEG loop with fb_count=2 drops a frame when a
//     stall exceeds the frame period, so tail latency, not the mean, sets usable fps.
//  3. Per-frame append timing at three realistic JPEG sizes. The MB/s -> fps conversion
//     is not linear (per-write FAT overhead dominates small frames, card stalls dominate
//     big ones), so it is measured at each size rather than extrapolated from (1).
//  4. Sequential read MB/s — the walk-up WiFi pull (sdq_wire.inc) reads the file back.
//
// WHAT IT CANNOT MEASURE, AND WHY THAT IS STILL USEFUL
// v0.6 has NO SDMMC routing: the card is wired for SPI mode (J8 on LORA_SCK/MOSI/MISO +
// CS). The S3 puts SDMMC through the GPIO matrix, so the same three lines can be re-driven
// as CLK/CMD/D0 and 1-bit SDMMC measured on the very same card — which is the honest
// bracket for that call. But 4-bit needs DAT1/DAT2, which v0.6 does not route at all, so
// the 4-bit number is NOT obtainable here. The bracket is: SPI (today's floor) .. SDMMC
// 1-bit (measured) .. 4-bit (unmeasurable, bounded above by ~4x the 1-bit line rate).
//
// The SDMMC attempt may legitimately FAIL, and that failure is itself a finding for the
// respin: 1-bit SDMMC needs 10k pull-ups on CMD and D0, and the 2026-08-09 probe found
// v0.6 has one on CS only ("MISO no pull-up — correct for SPI, the card drives it"). The
// gateway board died of exactly this one missing pull-up. So if SDMMC only mounts with
// the chip's weak internal pull-ups forced on, v1.0 must POPULATE those resistors, and
// this run is what proves it.
//
// TWO HARDWARE FACTS THIS SKETCH DEPENDS ON
//  * The card is on the SWITCHED rail (Q2, gated by IO44) — so a real power cycle is
//    available. That matters more than it sounds: a card that has once seen CMD0 with CS
//    low is LATCHED IN SPI MODE until it loses power, so without the rail we could not
//    test both buses in one boot. Every mount below is preceded by a full power cycle.
//  * The SD lines are shared with the socketed SX1262 (NSS IO14). Its NSS is driven HIGH
//    for the whole run so it never drives MISO.
//
// A subtlety worth keeping: while the rail is off, every SD line is floated. Holding one
// HIGH would backfeed the unpowered card through its ESD diodes and it would never
// actually reset — the power cycle would silently do nothing.
//
// Results land in homelab docs/trailcam/leaf-v1.0-respin-spec.md.

#include <SPI.h>
#include <SD.h>
#include <SD_MMC.h>
#include <driver/gpio.h>
#include <esp_task_wdt.h>
#include <soc/gpio_periph.h>
#include <soc/io_mux_reg.h>
#include "leaf_rails.h"

#ifndef LEAF_SD_SPI_CS
#error "LEAF_SD_BENCH is for the custom leaf PCB — build the leaf-pcb-sd-bench env"
#endif

// SPI-mode names / SDMMC-mode names for the same three board traces.
#define SDB_SCK   47   // = SDMMC CLK
#define SDB_MOSI  48   // = SDMMC CMD
#ifndef RADIO_MISO_PIN
#define RADIO_MISO_PIN 38
#endif
#define SDB_MISO  RADIO_MISO_PIN   // = SDMMC D0
#define SDB_CS    LEAF_SD_SPI_CS   // = SDMMC DAT3 (unused in 1-bit, held high)
#ifndef RADIO_CS_PIN
#define RADIO_CS_PIN 14
#endif

// Tunables. Every stage is ALSO deadline-capped, so the slow configs (SPI at 4 MHz is
// ~0.3 MB/s) degrade to a shorter-but-still-valid sample instead of stretching the run.
#define SDB_CHUNK        16384u    // sequential-transfer chunk
#define SDB_SEQ_BYTES    (4u * 1024u * 1024u)
#define SDB_STAGE_MS     20000u    // per-stage cap
#define SDB_FRAMES       40u       // frames per frame-size in the append test
#define SDB_MOUNT_MS     6000u     // SD_MMC.begin() can busy-spin forever; contain it

static uint8_t s_buf[SDB_CHUNK];

// Frame sizes to time appends at. Deliberately spans the range a clip could use:
// roughly VGA, SVGA/HD, and UXGA JPEGs at the quality the alert path already uses.
static const uint32_t SDB_FRAME_SIZES[] = {16384, 40960, 102400};
#define SDB_NFRAMESIZE (sizeof(SDB_FRAME_SIZES) / sizeof(SDB_FRAME_SIZES[0]))

struct FrameStat {
    uint32_t size;
    uint32_t n;
    float    mean_ms;
    uint32_t max_ms;
    float    mbps;
};

struct Result {
    const char* name;
    bool     mounted;
    bool     wedged;
    uint64_t card_mb;
    int      card_type;
    float    write_mbps;
    uint32_t write_worst_us;
    float    read_mbps;
    FrameStat frames[SDB_NFRAMESIZE];
};

static Result   s_results[8];
static uint8_t  s_nresults = 0;

// --- bus / power plumbing ---------------------------------------------------------

// Every mount path here can fail for two completely different reasons, and conflating
// them cost a bench session on 2026-08-15: a SanDisk 128 GB high-endurance card failed
// all six configs and the sketch said "the lines are unusable as SDMMC", which was
// wrong — the card had initialised fine on BOTH buses and only the FILESYSTEM was
// unmountable. 128 GB cards are SDXC and ship formatted **exFAT**, which ESP-IDF's
// FATFS does not enable. So always point at the driver's own line above.
static void sdb_explain_mount_failure() {
    Serial.println("      ^ read the driver line above before blaming the wiring:");
    Serial.println("        'no valid FAT volume' / 'mount_to_vfs failed' = the CARD IS");
    Serial.println("        FINE and answered on the bus; only the filesystem is wrong.");
    Serial.println("        128 GB SDXC cards ship exFAT, which this firmware cannot");
    Serial.println("        mount. Reformat FAT32. Anything else (no response, timeout,");
    Serial.println("        0x107) is a wiring/pull-up/power problem.");
}

// Read the pin's pull-up bit straight out of IO_MUX. Without this the pull-up passes are
// only inferable from throughput, and on 2026-08-15 the +pu and pull-up-free rows came
// back within 0.15 % of each other on BOTH cards — which has two completely different
// explanations (pull-ups genuinely do not matter here, OR the SDMMC driver reconfigured
// the pins and threw our setting away, making both rows the same test). Reading the
// register after begin() settles it instead of leaving a conclusion resting on a guess.
// The IDF slot config does NOT ask for internal pull-ups on the S3 (flags = 0 in
// SDMMC_SLOT_CONFIG_DEFAULT), so anything set here is ours.
static bool sdb_pullup_on(int pin) {
    return (REG_READ(GPIO_PIN_MUX_REG[pin]) & FUN_PU) != 0;
}

static void sdb_report_pullups(const char* when) {
    Serial.printf("      pull-ups %-11s CMD(IO%d)=%s  D0(IO%d)=%s  CLK(IO%d)=%s\n",
                  when,
                  SDB_MOSI, sdb_pullup_on(SDB_MOSI) ? "on " : "off",
                  SDB_MISO, sdb_pullup_on(SDB_MISO) ? "on " : "off",
                  SDB_SCK,  sdb_pullup_on(SDB_SCK)  ? "on " : "off");
}

static void sdb_float_sd_lines() {
    const int pins[] = {SDB_SCK, SDB_MOSI, SDB_MISO, SDB_CS};
    for (unsigned i = 0; i < 4; i++) {
        gpio_reset_pin((gpio_num_t)pins[i]);
        gpio_set_direction((gpio_num_t)pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode((gpio_num_t)pins[i], GPIO_FLOATING);
    }
}

// A card latches into SPI mode until it loses power, so this is mandatory between the
// SPI configs and the SDMMC ones — and cheap enough to just do before every mount.
static void sdb_power_cycle() {
    SD.end();
    SD_MMC.end();
    SPI.end();
    sdb_float_sd_lines();
    leaf_radio_rail(false);
    delay(500);                 // let the rail bleed down through the card's own load
    leaf_radio_rail(true);
    delay(100);                 // rail + bulk caps settle, card's own power-up ramp
}

// --- benchmark stages -------------------------------------------------------------

static bool sdb_seq_write(fs::FS& fs, Result& r) {
    fs.remove("/sdbench.bin");
    File f = fs.open("/sdbench.bin", FILE_WRITE);
    if (!f) { Serial.println("      open for write FAILED"); return false; }

    uint32_t t0 = millis(), worst = 0;
    uint32_t written = 0;
    while (written < SDB_SEQ_BYTES && millis() - t0 < SDB_STAGE_MS) {
        uint32_t c0 = micros();
        size_t n = f.write(s_buf, SDB_CHUNK);
        uint32_t dt = micros() - c0;
        if (dt > worst) worst = dt;
        if (n != SDB_CHUNK) { Serial.printf("      short write (%u of %u)\n",
                                            (unsigned)n, (unsigned)SDB_CHUNK); break; }
        written += n;
    }
    // Close is inside the timing on purpose: the FAT metadata flush is part of the cost
    // of actually having the bytes on the card.
    f.close();
    uint32_t ms = millis() - t0;
    if (!written || !ms) return false;

    r.write_mbps     = (written / 1e6f) / (ms / 1000.0f);
    r.write_worst_us = worst;
    Serial.printf("      seq write : %6.3f MB/s  (%u KB in %u ms, worst %u-KB chunk %.1f ms)\n",
                  r.write_mbps, (unsigned)(written / 1024), (unsigned)ms,
                  (unsigned)(SDB_CHUNK / 1024), worst / 1000.0f);
    return true;
}

static void sdb_seq_read(fs::FS& fs, Result& r) {
    File f = fs.open("/sdbench.bin", FILE_READ);
    if (!f) { Serial.println("      open for read FAILED"); return; }
    uint32_t t0 = millis(), got = 0;
    while (f.available() && millis() - t0 < SDB_STAGE_MS) {
        size_t n = f.read(s_buf, SDB_CHUNK);
        if (!n) break;
        got += n;
    }
    f.close();
    uint32_t ms = millis() - t0;
    if (!got || !ms) return;
    r.read_mbps = (got / 1e6f) / (ms / 1000.0f);
    Serial.printf("      seq read  : %6.3f MB/s  (%u KB in %u ms)\n",
                  r.read_mbps, (unsigned)(got / 1024), (unsigned)ms);
}

// The number that actually decides clip fps. Writes whole "frames" back to back, the way
// an MJPEG loop would, and reports the WORST one as well as the mean — a single 200 ms
// card stall drops frames no matter how good the average is.
static void sdb_frames(fs::FS& fs, Result& r) {
    for (unsigned i = 0; i < SDB_NFRAMESIZE; i++) {
        uint32_t fsz = SDB_FRAME_SIZES[i];
        r.frames[i].size = fsz;

        fs.remove("/sdframes.bin");
        File f = fs.open("/sdframes.bin", FILE_WRITE);
        if (!f) continue;

        uint32_t t0 = millis(), worst = 0, n = 0, total = 0;
        bool bad = false;
        while (n < SDB_FRAMES && millis() - t0 < SDB_STAGE_MS) {
            uint32_t f0 = millis();
            uint32_t left = fsz;
            while (left) {
                uint32_t want = left > SDB_CHUNK ? SDB_CHUNK : left;
                if (f.write(s_buf, want) != want) { bad = true; break; }
                left -= want;
            }
            if (bad) break;
            f.flush();          // an AVI writer commits each frame; keeps the FAT honest
            uint32_t dt = millis() - f0;
            if (dt > worst) worst = dt;
            total += fsz;
            n++;
        }
        f.close();
        uint32_t ms = millis() - t0;
        if (!n || !ms) continue;

        r.frames[i].n       = n;
        r.frames[i].mean_ms = (float)ms / n;
        r.frames[i].max_ms  = worst;
        r.frames[i].mbps    = (total / 1e6f) / (ms / 1000.0f);
        Serial.printf("      %3u-KB frames: mean %6.1f ms (%5.1f fps)  worst %5u ms "
                      "(%4.1f fps floor)  %.3f MB/s\n",
                      (unsigned)(fsz / 1024), r.frames[i].mean_ms,
                      1000.0f / r.frames[i].mean_ms, (unsigned)worst,
                      worst ? 1000.0f / worst : 0.0f, r.frames[i].mbps);
        fs.remove("/sdframes.bin");
    }
}

static void sdb_run_suite(fs::FS& fs, Result& r) {
    if (!sdb_seq_write(fs, r)) return;
    sdb_seq_read(fs, r);
    sdb_frames(fs, r);
    fs.remove("/sdbench.bin");
}

// --- SPI mode ---------------------------------------------------------------------

static void sdb_spi(const char* name, uint32_t hz) {
    Result& r = s_results[s_nresults++];
    r = Result{};
    r.name = name;

    Serial.printf("\n[%s]\n", name);
    sdb_power_cycle();

    SPI.begin(SDB_SCK, SDB_MISO, SDB_MOSI, SDB_CS);
    if (!SD.begin(SDB_CS, SPI, hz)) {
        Serial.println("      mount FAILED at this clock");
        sdb_explain_mount_failure();
        SPI.end();
        return;
    }
    r.mounted   = true;
    r.card_type = (int)SD.cardType();
    r.card_mb   = SD.cardSize() / (1024ULL * 1024ULL);
    Serial.printf("      mounted   : type=%d, %llu MB\n", r.card_type,
                  (unsigned long long)r.card_mb);

    sdb_run_suite(SD, r);
    SD.end();
    SPI.end();
}

// --- SDMMC 1-bit ------------------------------------------------------------------
// Contained exactly like validation/sd-pullup-probe: SD_MMC.begin() BUSY-SPINS FOREVER
// when the card never answers (which is precisely the failure a missing CMD/D0 pull-up
// produces), starving the idle task until the watchdog panics the chip into a reboot
// loop. So it runs in its own task on core 0 with that core's idle task un-watched, under
// a timeout, and gets killed if it wedges.

static volatile bool s_mnt_done = false, s_mnt_ok = false;
static TaskHandle_t  s_mnt_task = nullptr;
static int           s_mnt_khz  = 0;

static void sdb_mount_task(void*) {
    SD_MMC.end();
    SD_MMC.setPins(SDB_SCK, SDB_MOSI, SDB_MISO);
    s_mnt_ok   = SD_MMC.begin("/sdcard", /*mode1bit=*/true, /*format=*/false, s_mnt_khz);
    s_mnt_done = true;
    vTaskSuspend(nullptr);
}

// 1 = mounted, 0 = failed cleanly, -1 = wedged (SDMMC host is unusable until reboot)
static int sdb_try_sdmmc_mount(int khz) {
    s_mnt_done = false; s_mnt_ok = false; s_mnt_task = nullptr; s_mnt_khz = khz;

    // Un-watching idle0 makes it log "esp_task_wdt_reset: task not found" on every one of
    // its own ticks until it is re-added — a wall of red for the few seconds a mount takes.
    // Expected and harmless; the alternative is a watchdog panic if a mount ever wedges.
    TaskHandle_t idle0 = xTaskGetIdleTaskHandleForCPU(0);
    bool unwatched = (idle0 && esp_task_wdt_delete(idle0) == ESP_OK);

    int rc = -1;
    if (xTaskCreatePinnedToCore(sdb_mount_task, "sdbmnt", 8192, nullptr, 1, &s_mnt_task, 0)
            == pdPASS) {
        uint32_t t0 = millis();
        while (!s_mnt_done && millis() - t0 < SDB_MOUNT_MS) delay(20);
        if (s_mnt_done) rc = s_mnt_ok ? 1 : 0;
        if (s_mnt_task) { vTaskDelete(s_mnt_task); s_mnt_task = nullptr; }
    }
    if (unwatched && idle0) esp_task_wdt_add(idle0);

    // begin() can early-return true with nothing behind it; only a real card counts.
    if (rc == 1 && (SD_MMC.cardType() == CARD_NONE || SD_MMC.cardSize() == 0)) rc = 0;
    return rc;
}

// Returns false if the SDMMC host wedged — everything after that is meaningless until
// a reboot, so the caller stops.
// Clearing the pull-ups BEFORE begin() does nothing — the SDMMC driver switches them
// back on during init (measured 2026-08-15: "before mount: off" -> "after mount: on",
// even though SDMMC_SLOT_CONFIG_DEFAULT sets flags = 0 on the S3). So the only way to
// ask "does this bus work with NO pull-up on CMD/D0?" is to mount normally and strip
// them afterwards, then do the transfers. That is the question v1.0 actually needs
// answered, because it decides whether the 10k resistors are a requirement or insurance.
static void sdb_strip_pullups() {
    gpio_set_pull_mode((gpio_num_t)SDB_MOSI, GPIO_FLOATING);   // CMD
    gpio_set_pull_mode((gpio_num_t)SDB_MISO, GPIO_FLOATING);   // D0
    gpio_set_pull_mode((gpio_num_t)SDB_SCK,  GPIO_FLOATING);   // CLK
}

static bool sdb_sdmmc(const char* name, int khz, bool internal_pullups,
                      bool strip_after_mount = false) {
    Result& r = s_results[s_nresults++];
    r = Result{};
    r.name = name;

    Serial.printf("\n[%s]\n", name);
    sdb_power_cycle();

    if (internal_pullups) {
        // ~45k, far weaker than the 10k a board is supposed to carry. If the card only
        // answers WITH these, v1.0 must populate real pull-ups on CMD and D0.
        gpio_set_pull_mode((gpio_num_t)SDB_MOSI, GPIO_PULLUP_ONLY);   // CMD
        gpio_set_pull_mode((gpio_num_t)SDB_MISO, GPIO_PULLUP_ONLY);   // D0
        gpio_set_pull_mode((gpio_num_t)SDB_SCK,  GPIO_PULLUP_ONLY);   // CLK
    }
    // DAT3/CS must be HIGH through init or the card interprets CMD0 as "enter SPI mode".
    // v0.6 has a real 10k here (on the switched rail), but drive it anyway — this is the
    // one line whose state changes which protocol the card speaks.
    gpio_set_pull_mode((gpio_num_t)SDB_CS, GPIO_PULLUP_ONLY);
    sdb_report_pullups("before mount:");

    int rc = sdb_try_sdmmc_mount(khz);
    sdb_report_pullups("after mount: ");
    if (rc == -1) {
        r.wedged = true;
        Serial.println("      mount WEDGED — the card never answered. On this board that");
        Serial.println("      means CMD/D0 have no pull-up strong enough for SDMMC.");
        Serial.println("      The SDMMC host is now stuck until reboot; stopping here.");
        return false;
    }
    if (rc == 0) {
        Serial.println("      mount FAILED");
        sdb_explain_mount_failure();
        return true;
    }

    r.mounted   = true;
    r.card_type = (int)SD_MMC.cardType();
    r.card_mb   = SD_MMC.cardSize() / (1024ULL * 1024ULL);
    // Say only what was done, not what was needed: the pull-up-free config runs FIRST, so
    // whether these were required is read off that row, not asserted here.
    Serial.printf("      mounted   : type=%d, %llu MB%s\n", r.card_type,
                  (unsigned long long)r.card_mb,
                  internal_pullups ? "  (chip's weak internal pull-ups forced on)" : "");

    if (strip_after_mount) {
        sdb_strip_pullups();
        sdb_report_pullups("stripped:   ");
        Serial.println("      ^ transfers below run with NO pull-up on CMD/D0. If they");
        Serial.println("        complete at full speed, the 10k resistors are insurance,");
        Serial.println("        not a requirement. Errors or a collapse = they are needed.");
    }

    sdb_run_suite(SD_MMC, r);
    SD_MMC.end();
    return true;
}

// --- summary ----------------------------------------------------------------------

static void sdb_summary() {
    Serial.println();
    Serial.println("==================================================================");
    Serial.println(" SUMMARY — sustained transfer");
    Serial.println("==================================================================");
    Serial.println(" config                        write MB/s  read MB/s  worst 16K chunk");
    for (unsigned i = 0; i < s_nresults; i++) {
        Result& r = s_results[i];
        if (!r.mounted) {
            Serial.printf(" %-28s %s\n", r.name, r.wedged ? "WEDGED (no answer)" : "did not mount");
            continue;
        }
        Serial.printf(" %-28s %9.3f  %9.3f  %11.1f ms\n",
                      r.name, r.write_mbps, r.read_mbps, r.write_worst_us / 1000.0f);
    }

    Serial.println();
    Serial.println("==================================================================");
    Serial.println(" SUMMARY — sustainable MJPEG, by measured per-frame append time");
    Serial.println(" (fps mean = 1/mean frame time; fps floor = 1/worst frame time, i.e.");
    Serial.println("  the rate at which NO frame is dropped even through a card stall)");
    Serial.println("==================================================================");
    for (unsigned i = 0; i < s_nresults; i++) {
        Result& r = s_results[i];
        if (!r.mounted) continue;
        Serial.printf(" %s\n", r.name);
        for (unsigned j = 0; j < SDB_NFRAMESIZE; j++) {
            FrameStat& fsx = r.frames[j];
            if (!fsx.n) continue;
            Serial.printf("   %3u KB/frame (~%s): %5.1f fps mean, %4.1f fps floor\n",
                          (unsigned)(fsx.size / 1024),
                          fsx.size <= 16384 ? "VGA " : (fsx.size <= 40960 ? "SVGA" : "UXGA"),
                          1000.0f / fsx.mean_ms,
                          fsx.max_ms ? 1000.0f / fsx.max_ms : 0.0f);
        }
    }
    Serial.println();
    Serial.println("Note: SDMMC 4-bit is NOT measurable on v0.6 — DAT1/DAT2 are not routed.");
    Serial.println("It is bounded above by ~4x the 1-bit LINE rate, but the card's own");
    Serial.println("program time does not scale with bus width, so the real gain is less.");
}

void setup() {
    Serial.begin(LEAF_BAUD);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) delay(50);
    delay(300);

    Serial.println();
    Serial.println("==================================================================");
    Serial.println(" LEAF v0.6 microSD THROUGHPUT BENCH");
    Serial.println(" SPI mode (what v1.0 ships today) vs SDMMC 1-bit, same card,");
    Serial.println(" power-cycled between every mount so neither result is contaminated.");
    Serial.println("==================================================================");

    for (unsigned i = 0; i < SDB_CHUNK; i++) s_buf[i] = (uint8_t)(i * 7 + 13);

    // Park the radio's NSS high for the whole run: it shares SCK/MOSI/MISO and would
    // otherwise be free to drive MISO on top of the card.
    pinMode(RADIO_CS_PIN, OUTPUT);
    digitalWrite(RADIO_CS_PIN, HIGH);

    sdb_spi("SPI  4 MHz (today's default)", 4000000);
    sdb_spi("SPI 20 MHz",                   20000000);
    sdb_spi("SPI 40 MHz",                   40000000);

    // SDMMC last: a wedged host stays wedged until reboot, and it must not be allowed to
    // invalidate the SPI numbers.
    //
    // ORDER MATTERS, and this is the second arrangement. The first run only ever tried
    // 40 MHz WITH internal pull-ups, which left the question that actually decides a
    // schematic — does the SD bus need real 10k pull-ups on CMD/D0 at 40 MHz? —
    // unanswered. "40 MHz, no pull-ups" is therefore the point of this run, and it goes
    // LAST precisely because it is the one most likely to wedge: a wedge there now costs
    // nothing, whereas a wedge in the middle would take the rest of the matrix with it.
    // A wedge on that final config is not a failed run, it IS the answer (pull-ups
    // required at 40 MHz).
    if (sdb_sdmmc("SDMMC 1-bit 20 MHz", 20000, false)) {
        if (sdb_sdmmc("SDMMC 1-bit 40 MHz", 40000, true)) {
            // The two that answer the schematic question. Pull-ups are stripped AFTER
            // mount (see sdb_strip_pullups) because clearing them earlier is undone by
            // the driver. 40 MHz goes last: it is the likeliest to fall over, and by
            // then every other row is already on the wire.
            if (sdb_sdmmc("SDMMC 20 MHz, pull-ups OFF", 20000, false, true)) {
                sdb_sdmmc("SDMMC 40 MHz, pull-ups OFF", 40000, true, true);
            }
        }
    }

    sdb_summary();
    Serial.println();
    Serial.println("Done. Tap RESET to re-run. Rail left ON.");
    Serial.println("==================================================================");
}

void loop() { delay(1000); }
