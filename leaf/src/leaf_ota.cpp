#include "leaf_ota.h"
#include "leaf_serial_proto.h"

#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <Update.h>
#include <mbedtls/sha256.h>

// Parse "aabbcc..." (64 chars) into 32 bytes. Returns false on any non-hex/short input.
static bool hex_to_bytes32(const char* hex, uint8_t out[32]) {
    if (strlen(hex) != 64) return false;
    for (int i = 0; i < 32; i++) {
        char b[3] = { hex[i * 2], hex[i * 2 + 1], 0 };
        char* end = nullptr;
        long v = strtol(b, &end, 16);
        if (end != b + 2) return false;
        out[i] = (uint8_t)v;
    }
    return true;
}

static void wifi_down() {
    WiFi.disconnect(true /*wifioff*/);
    WiFi.mode(WIFI_OFF);
}

bool leaf_ota_run(const TcOtaRequest& req) {
    uint8_t want_sha[32];
    if (!hex_to_bytes32(req.sha256, want_sha)) {
        Serial.println("[ota] bad/missing sha256 in command -> refused (the hash IS the integrity)");
        return false;
    }

    // Idempotence guard: a SERVER-QUEUED update_firmware has no completion signal, so
    // it re-delivers on later announces until it expires — without this the leaf would
    // re-download and re-flash the same image in a loop. The sha of the last applied
    // OTA lives in NVS (survives the reboot the OTA causes).
    Preferences prefs;
    prefs.begin("tc-ota", false);
    String applied = prefs.getString("sha", "");
    if (applied == req.sha256) {
        prefs.end();
        Serial.println("[ota] sha matches the already-applied image -> skip (queued repeat)");
        return false;
    }
    prefs.end();

    Serial.printf("[ota] joining \"%s\" ...\n", req.ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(req.ssid, req.psk);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) delay(100);
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[ota] WiFi join FAILED -> abort (firmware unchanged)");
        wifi_down();
        return false;
    }
    Serial.printf("[ota] WiFi up, ip=%s rssi=%d\n",
                  WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());

    HTTPClient http;
    // No CA store on the leaf: for https the transport is unauthenticated and the
    // command-bus sha256 is the integrity check (see header). http URLs work too.
    http.setConnectTimeout(10000);
    http.setTimeout(15000);
    if (!http.begin(req.url)) {
        Serial.println("[ota] bad URL -> abort");
        wifi_down();
        return false;
    }
    int code = http.GET();
    int len  = http.getSize();
    if (code != HTTP_CODE_OK || len <= 0) {
        Serial.printf("[ota] GET failed: http=%d len=%d -> abort\n", code, len);
        http.end(); wifi_down();
        return false;
    }
    Serial.printf("[ota] downloading %d bytes from %s\n", len, req.url);

    if (!Update.begin(len)) {   // sizes + erases the idle OTA slot
        Serial.printf("[ota] Update.begin failed: %s\n", Update.errorString());
        http.end(); wifi_down();
        return false;
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha, 0 /*sha256, not 224*/);

    WiFiClient* stream = http.getStreamPtr();
    static uint8_t buf[4096];
    int written = 0;
    uint32_t last_log = millis();
    while (written < len) {
        size_t avail = stream->available();
        if (!avail) {
            if (!http.connected()) break;
            delay(10);
            continue;
        }
        int n = stream->readBytes(buf, min(avail, sizeof(buf)));
        if (n <= 0) break;
        mbedtls_sha256_update(&sha, buf, n);
        if ((int)Update.write(buf, n) != n) {
            Serial.printf("[ota] flash write failed at %d: %s\n", written, Update.errorString());
            break;
        }
        written += n;
        if (millis() - last_log > 2000) {
            last_log = millis();
            Serial.printf("[ota] %d / %d bytes (%.0f%%)\n", written, len, 100.0 * written / len);
        }
    }

    uint8_t got_sha[32];
    mbedtls_sha256_finish(&sha, got_sha);
    mbedtls_sha256_free(&sha);
    http.end();

    if (written != len) {
        Serial.printf("[ota] short download (%d / %d) -> abort (firmware unchanged)\n", written, len);
        Update.abort();
        wifi_down();
        return false;
    }
    if (memcmp(got_sha, want_sha, 32) != 0) {
        Serial.println("[ota] sha256 MISMATCH -> abort (firmware unchanged)");
        Update.abort();
        wifi_down();
        return false;
    }
    if (!Update.end(true)) {   // finalize + set boot partition to the new slot
        Serial.printf("[ota] Update.end failed: %s\n", Update.errorString());
        wifi_down();
        return false;
    }

    // Remember what we applied BEFORE rebooting into it (see the idempotence guard).
    prefs.begin("tc-ota", false);
    prefs.putString("sha", req.sha256);
    prefs.end();

    Serial.printf("[ota] verified %d bytes, sha256 OK -> rebooting into the new firmware\n", len);
    Serial.flush();
    wifi_down();
    ESP.restart();
    return true;   // unreachable
}
