#include "leaf_nvs.h"

#include <string.h>

#include "esp_partition.h"
#include "nvs_flash.h"

bool leaf_nvs_durable_available() {
    static int cached = -1;
    if (cached < 0)
        cached = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                          ESP_PARTITION_SUBTYPE_DATA_NVS,
                                          LEAF_NVS_DURABLE) != nullptr ? 1 : 0;
    return cached == 1;
}

// What must outlive a reflash. Kept here rather than in each feature's own file so the
// list is one thing you can read, and so the migration does not depend on which features
// a given env happens to compile in — leaf_portal only exists with LEAF_PORTAL, but its
// AP key still has to move on an env that has never raised a portal.
struct DurableKey {
    const char* ns;
    const char* key;
    bool        is_string;   // false = raw bytes (the identity's private key)
};
static const DurableKey DURABLE[] = {
    {"leaf",      "rns_prv", false},   // the RNS identity — the node's whole wire name
    {"tc-portal", "ap_psk",  true},    // the AP key printed on the node's label at kitting
};

// Erase the DEFAULT nvs partition outright, and re-format it.
//
// Preferences::remove() is not enough and it is worth being precise about why. NVS is
// log-structured: remove() writes a tombstone, and the old entry's bytes sit in flash
// until that page happens to be compacted. Measured on board 0013 after the first
// version of this migration: the 64-byte private key was still recoverable from the old
// partition with a plain `esptool read-flash`. Two copies of the key, one of them in the
// partition anyone can dump, is exactly the outcome this change exists to prevent.
//
// Everything in that partition is disposable by design, which is what makes this safe:
// the WiFi/BLE PHY calibration recomputes itself, and the OTA "already applied" sha
// costs at most one redundant re-download if it is lost. Deinit before erasing, because
// initArduino() has already mounted this partition and erasing underneath a live NVS
// leaves stale state in RAM.
static void wipe_default_nvs() {
    const esp_partition_t* part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
    if (!part) return;
    nvs_flash_deinit_partition(part->label);
    if (esp_partition_erase_range(part, 0, part->size) != ESP_OK) {
        Serial.println("[nvs] WARNING: could not erase the default partition — the old "
                       "copy of the key is still in flash until it is overwritten");
        nvs_flash_init_partition(part->label);
        return;
    }
    nvs_flash_init_partition(part->label);
    Serial.printf("[nvs] default partition '%s' erased — the migrated keys now exist in "
                  "exactly one place\n", part->label);
}

void leaf_nvs_migrate_durable() {
    if (!leaf_nvs_durable_available()) return;   // pre-0.23.0 table: nothing to move into
    bool stale_copies = false;
    for (const DurableKey& d : DURABLE) {
        Preferences old;
        if (!old.begin(d.ns, /*readOnly=*/true)) continue;
        const bool in_old = d.is_string ? old.isKey(d.key) : (old.getBytesLength(d.key) > 0);
        if (!in_old) { old.end(); continue; }
        // Something durable is still sitting in the disposable partition. Whether or not
        // this boot is the one that copies it across, the partition needs wiping before
        // we are done — that check is what makes the whole thing survive losing power
        // halfway through (copy done, wipe not yet: the next boot still wipes).
        stale_copies = true;

        Preferences dst;
        if (!dst.begin(d.ns, /*readOnly=*/false, LEAF_NVS_DURABLE)) { old.end(); continue; }
        const bool already = d.is_string ? dst.isKey(d.key) : (dst.getBytesLength(d.key) > 0);
        if (!already) {
            if (d.is_string) {
                String v = old.getString(d.key, "");
                if (v.length() > 0) dst.putString(d.key, v);
            } else {
                uint8_t buf[128];
                size_t n = old.getBytesLength(d.key);
                if (n > 0 && n <= sizeof(buf)) {
                    old.getBytes(d.key, buf, n);
                    dst.putBytes(d.key, buf, n);
                }
            }
            Serial.printf("[nvs] MIGRATED %s/%s into the durable partition — it now "
                          "survives a full 0x0 reflash and an NVS-full erase\n", d.ns, d.key);
        }
        dst.end();
        old.end();
    }
    if (stale_copies) wipe_default_nvs();
}
