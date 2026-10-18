#include "leaf_identity.h"

#ifdef LEAF_RADIO

#include <LittleFS.h>
#include <string.h>
#include "esp_attr.h"   // RTC_DATA_ATTR

// One line, space separated, human-readable on a console `cat`:
//   1 <hash_hex> <slug> <prev_slug|-> <generation>
// A single line so a torn write is detectable (a short/garbled line parses to nothing and
// is treated as "no witness", which re-arms rather than false-alarms).
static const char* WITNESS_PATH = "/identity.wit";
static const int   WITNESS_VER  = 1;

// Loaded/derived once per boot. leaf_radio_begin() is idempotent per boot, so is this.
static char     s_prev[24]   = "";
static uint32_t s_generation = 1;
static bool     s_checked    = false;

// Per-power-cycle alarm budget. RTC memory survives deep sleep and dies with power, which
// is the right lifetime: every wake is a fresh boot, so a plain static would re-arm the
// full burst every 20 s and put the token on every announce forever.
static RTC_DATA_ATTR uint32_t s_alarm_magic = 0;
static RTC_DATA_ATTR uint8_t  s_alarm_left  = 0;
#define ALARM_ARMED_MAGIC 0x1DEA7100u

struct Witness {
    bool     valid = false;
    char     hash[33] = "";
    char     slug[24] = "";
    char     prev[24] = "";
    uint32_t generation = 0;
};

static Witness read_witness() {
    Witness w;
    File f = LittleFS.open(WITNESS_PATH, "r");
    if (!f) return w;
    char line[128];
    size_t n = f.readBytesUntil('\n', line, sizeof(line) - 1);
    f.close();
    line[n] = 0;
    int ver = 0;
    unsigned long gen = 0;
    char hash[64] = "", slug[64] = "", prev[64] = "";
    if (sscanf(line, "%d %63s %63s %63s %lu", &ver, hash, slug, prev, &gen) != 5) return w;
    if (ver != WITNESS_VER || strlen(hash) > 32 || strlen(slug) > 23 || strlen(prev) > 23)
        return w;
    snprintf(w.hash, sizeof(w.hash), "%s", hash);
    snprintf(w.slug, sizeof(w.slug), "%s", slug);
    if (strcmp(prev, "-") != 0) snprintf(w.prev, sizeof(w.prev), "%s", prev);
    w.generation = (uint32_t)gen;
    w.valid = true;
    return w;
}

static void write_witness(const char* hash, const char* slug, const char* prev, uint32_t gen) {
    File f = LittleFS.open(WITNESS_PATH, "w");
    if (!f) {
        Serial.println("[identity] WARNING: cannot write the identity witness "
                       "-> a future identity change will go unnoticed");
        return;
    }
    f.printf("%d %s %s %s %lu\n", WITNESS_VER, hash, slug,
             (prev && prev[0]) ? prev : "-", (unsigned long)gen);
    f.close();
}

void leaf_identity_witness(const char* slug, const char* hash_hex, bool minted) {
    if (s_checked) return;
    s_checked = true;
    if (!slug || !slug[0] || !hash_hex || !hash_hex[0]) return;

    const Witness w = read_witness();

    if (!w.valid) {
        // First boot on this filesystem. Nothing to compare against, so nothing to claim:
        // a board that has genuinely never run and a board whose LittleFS was just
        // reformatted look identical from here, and guessing between them would be the
        // false alarm that teaches people to ignore the real one.
        write_witness(hash_hex, slug, nullptr, 1);
        Serial.printf("[identity] witness written: %s (first record on this filesystem)\n", slug);
        return;
    }

    if (strcmp(w.hash, hash_hex) == 0) {
        // Expected identity. Carry any earlier loss forward so it stays reportable.
        s_generation = w.generation;
        snprintf(s_prev, sizeof(s_prev), "%s", w.prev);
        if (s_prev[0]) {
            Serial.printf("[identity] %s (generation %lu; this board previously announced "
                          "as %s — server-side history is split until it is merged)\n",
                          slug, (unsigned long)s_generation, s_prev);
            // Fresh power cycle (RTC memory blank) with a loss still on record: re-arm the
            // on-air burst. A node whose gateway was down the day it lost its identity
            // gets another chance to say so, and one that already reported pays nothing —
            // the magic survives deep sleep, so the burst is spent once per power cycle.
            if (s_alarm_magic != ALARM_ARMED_MAGIC) {
                s_alarm_magic = ALARM_ARMED_MAGIC;
                s_alarm_left  = LEAF_IDENTITY_ALARM_ANNOUNCES;
            }
        }
        return;
    }

    // The identity changed. Say so in the loudest terms the console has.
    snprintf(s_prev, sizeof(s_prev), "%s", w.slug);
    s_generation = w.generation + 1;
    Serial.println();
    Serial.println("*** [identity] THIS BOARD'S MESH IDENTITY CHANGED ***");
    Serial.printf("*** [identity]   was %s  (%s)\n", w.slug, w.hash);
    Serial.printf("*** [identity]   now %s  (%s)\n", slug, hash_hex);
    Serial.printf("*** [identity]   NVS was %s this boot\n",
                  minted ? "EMPTY -> a new key was minted" :
                           "readable, but holds a DIFFERENT key");
    Serial.printf("*** [identity]   generation %lu\n", (unsigned long)s_generation);
    Serial.println("*** [identity]   consequences until this is repaired:");
    Serial.println("*** [identity]     - the gateway's TOFU pin still names the old key,");
    Serial.println("*** [identity]       so every queued command for this node is undeliverable");
    Serial.println("*** [identity]     - the server will auto-create a second camera row,");
    Serial.println("*** [identity]       so captures and telemetry split across two nodes");
    Serial.println("*** [identity]   the server side merges with");
    Serial.println("*** [identity]   POST /api/v1/cameras/<kept>/merge {\"source\":\"<new slug>\"}");
    Serial.println();
    write_witness(hash_hex, slug, w.slug, s_generation);

    // Arm the on-air alarm for this power cycle. An identity change is always a cold boot
    // (the key is read once, at boot), so this arms exactly where it should.
    s_alarm_magic = ALARM_ARMED_MAGIC;
    s_alarm_left  = LEAF_IDENTITY_ALARM_ANNOUNCES;
}

const char* leaf_identity_prev_slug() { return s_prev[0] ? s_prev : nullptr; }

uint32_t leaf_identity_generation() { return s_generation; }

bool leaf_identity_alarm_pending() {
    return s_prev[0] && s_alarm_magic == ALARM_ARMED_MAGIC && s_alarm_left > 0;
}

void leaf_identity_alarm_sent() {
    if (s_alarm_magic == ALARM_ARMED_MAGIC && s_alarm_left > 0) s_alarm_left--;
}

#endif  // LEAF_RADIO
