// SD slot wiring probe for the LilyGO T-Internet-POE (ESP32 classic).
//
// WHY THIS EXISTS
// The gateway's microSD upload queue could not mount a card on this board:
// SD_MMC.begin() returned 0x107 (timeout) with esp-idf's hint "Make sure SD card lines
// have pull-up resistors in place", and with an EMPTY slot it never returned at all.
// Before making "populate 10k pull-ups on CMD and D0" a hard requirement on the S3
// gateway PCB, we want evidence rather than a guess.
//
// HOW IT DECIDES
// The chip has weak (~45k) pull-ups/pull-downs it can switch onto any pin. A proper
// board has a much stronger 10k pull-up to 3V3 on the SD command/data lines. So:
//
//   switch on the chip's weak PULL-DOWN and read the pin
//     reads HIGH -> something stronger is holding it up -> a real pull-up EXISTS
//     reads LOW  -> nothing is holding it up            -> pull-up is MISSING
//
// Run it with the slot EMPTY to measure the BOARD. An inserted card can present its own
// pull-ups, which would mask a missing one — so the empty-slot round is the honest test.
//
// It then tries to actually mount a card three ways (plain, with the chip's weak
// pull-ups forced on, and at a slow clock) to separate "no pull-ups" from "bad card"
// from "signal too fast".
//
// SAFETY: SD_MMC.begin() BUSY-SPINS FOREVER on an empty slot on this board (bench,
// 2026-08-09) — it starves the idle task and the watchdog panics the chip into a reboot
// loop. So every mount attempt runs in its own task pinned to core 0, with core 0's idle
// task temporarily un-watched, under a timeout, and the task is killed if it wedges.
// Same containment as the gateway fix in wt/qjq-sd-hang-fix.

#include <Arduino.h>
#include <SD_MMC.h>
#include <driver/gpio.h>
#include <esp_task_wdt.h>

// 1-bit SD_MMC on the ESP32 classic — fixed-function pins, not remappable.
#define PIN_CLK 14
#define PIN_CMD 15
#define PIN_D0   2

#define MOUNT_TIMEOUT_MS 4000u

struct LineResult {
    int  pin;
    const char* name;
    int  with_pulldown;   // 1 = something external holds it up
    int  with_pullup;     // 0 = something external drags it down
};

static LineResult probe_line(int pin, const char* name) {
    LineResult r{pin, name, -1, -1};
    gpio_num_t g = (gpio_num_t)pin;
    gpio_reset_pin(g);
    gpio_set_direction(g, GPIO_MODE_INPUT);

    gpio_set_pull_mode(g, GPIO_PULLDOWN_ONLY);
    delay(30);
    r.with_pulldown = gpio_get_level(g);

    gpio_set_pull_mode(g, GPIO_PULLUP_ONLY);
    delay(30);
    r.with_pullup = gpio_get_level(g);

    gpio_set_pull_mode(g, GPIO_FLOATING);
    return r;
}

static void report_line(const LineResult& r) {
    const char* verdict;
    if (r.with_pulldown == 1 && r.with_pullup == 1) {
        verdict = "HELD UP  -> a real pull-up resistor IS present";
    } else if (r.with_pulldown == 0 && r.with_pullup == 1) {
        verdict = "NOT held up -> pull-up resistor is MISSING";
    } else if (r.with_pullup == 0) {
        verdict = "DRAGGED DOWN -> shorted, or an LED/other part is on this line";
    } else {
        verdict = "inconclusive";
    }
    Serial.printf("  %-3s (chip pin %2d): %s\n", r.name, r.pin, verdict);
}

// --- timeout-bounded mount ---------------------------------------------------------
static volatile bool s_done   = false;
static volatile bool s_result = false;
static TaskHandle_t  s_task   = nullptr;
static bool          s_use_internal_pullups = false;
static int           s_freq_khz = 0;

static void mount_task(void*) {
    if (s_use_internal_pullups) {
        gpio_set_pull_mode((gpio_num_t)PIN_CMD, GPIO_PULLUP_ONLY);
        gpio_set_pull_mode((gpio_num_t)PIN_D0,  GPIO_PULLUP_ONLY);
        gpio_set_pull_mode((gpio_num_t)PIN_CLK, GPIO_PULLUP_ONLY);
    }
    SD_MMC.end();
    bool ok = s_freq_khz
        ? SD_MMC.begin("/sdcard", true, false, s_freq_khz)
        : SD_MMC.begin("/sdcard", true, false);
    s_result = ok;
    s_done   = true;
    vTaskSuspend(nullptr);
}

// Returns: 1 = mounted, 0 = failed cleanly, -1 = wedged (no card / dead lines)
static int try_mount(const char* label, bool internal_pullups, int freq_khz) {
    Serial.printf("  %-34s ", label);
    Serial.flush();

    s_done = false; s_result = false; s_task = nullptr;
    s_use_internal_pullups = internal_pullups;
    s_freq_khz = freq_khz;

    TaskHandle_t idle0 = xTaskGetIdleTaskHandleForCPU(0);
    bool unwatched = (idle0 && esp_task_wdt_delete(idle0) == ESP_OK);

    int rc = -1;
    if (xTaskCreatePinnedToCore(mount_task, "sdmount", 4096, nullptr, 1, &s_task, 0)
            == pdPASS) {
        uint32_t t0 = millis();
        while (!s_done && millis() - t0 < MOUNT_TIMEOUT_MS) delay(20);
        if (s_done) rc = s_result ? 1 : 0;
        if (s_task) { vTaskDelete(s_task); s_task = nullptr; }
    }
    if (unwatched && idle0) esp_task_wdt_add(idle0);

    // begin() can return true with no card behind it (it early-returns if the host is
    // already up), so a claimed success is only believed if a real card answers.
    if (rc == 1) {
        uint64_t mb = SD_MMC.cardSize() >> 20;
        if (SD_MMC.cardType() == CARD_NONE || mb == 0) {
            Serial.println("reported success but NO REAL CARD behind it — not believed");
            rc = 0;
        } else {
            Serial.printf("WORKED  (card %llu MB)\n", (unsigned long long)mb);
        }
        SD_MMC.end();
    } else if (rc == 0) {
        Serial.println("failed (card answered wrong, or no card)");
    } else {
        Serial.println("WEDGED (no response at all — almost certainly an empty slot)");
    }
    gpio_set_pull_mode((gpio_num_t)PIN_CMD, GPIO_FLOATING);
    gpio_set_pull_mode((gpio_num_t)PIN_D0,  GPIO_FLOATING);
    gpio_set_pull_mode((gpio_num_t)PIN_CLK, GPIO_FLOATING);
    return rc;
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) delay(50);
    delay(300);

    Serial.println();
    Serial.println("==================================================");
    Serial.println(" SD SLOT TEST  (T-Internet-POE)");
    Serial.println("==================================================");

    Serial.println("STEP 1 - is the board wired with SD pull-up resistors?");
    Serial.println("         (most meaningful with the slot EMPTY)");
    LineResult cmd = probe_line(PIN_CMD, "CMD");
    LineResult d0  = probe_line(PIN_D0,  "D0");
    LineResult clk = probe_line(PIN_CLK, "CLK");
    report_line(cmd);
    report_line(d0);
    report_line(clk);

    Serial.print("  SUMMARY: ");
    if (cmd.with_pulldown == 1 && d0.with_pulldown == 1) {
        Serial.println("board HAS pull-ups on both lines that need one.");
    } else if (cmd.with_pulldown == 1 && d0.with_pulldown == 0) {
        Serial.println("CMD is fine but D0 has NO pull-up. A card needs one on BOTH; "
                       "this alone can stop a card from ever answering.");
    } else if (cmd.with_pulldown == 0) {
        Serial.println("board is MISSING the pull-up on CMD (the one that matters most).");
    } else {
        Serial.println("inconclusive.");
    }
    if (d0.with_pulldown == 0 && d0.with_pullup == 1) {
        Serial.println("           (D0 is simply floating — not shorted and not dragged "
                       "down by an LED, it just has no resistor.)");
    }

    Serial.println();
    Serial.println("STEP 2 - can it actually talk to a card?");
    // A wedged attempt leaves the SD hardware unusable for the rest of the boot (later
    // attempts come back 'host init failed (0x103)'), so anything after one is noise.
    // Stop rather than print results that cannot mean anything.
    if (try_mount("as the gateway does it:", false, 0) == -1) {
        Serial.println("  -> stopping: the SD hardware is stuck until the next reboot,");
        Serial.println("     so the remaining attempts could not tell us anything.");
    } else if (try_mount("with chip's weak pull-ups on:", true, 0) == -1) {
        Serial.println("  -> stopping (same reason).");
    } else {
        try_mount("weak pull-ups + slow clock:", true, 400);
    }

    Serial.println();
    Serial.println("Done. Change the card (or empty the slot) and tap RST to re-run.");
    Serial.println("==================================================");
}

void loop() { delay(1000); }
