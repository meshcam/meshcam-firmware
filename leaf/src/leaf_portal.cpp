/*
 * Leaf SoftAP + captive portal. Contract, rationale and the three deliberate
 * departures from the gateway's portal: leaf_portal.h.
 */
#ifdef LEAF_PORTAL
#include "leaf_portal.h"

#include <DNSServer.h>          // bundled with arduino-esp32; answers "*" with the AP IP
#include <Preferences.h>

#include "leaf_nvs.h"   // the AP key lives in the durable partition
#include <WebServer.h>
#include <WiFi.h>
#include <esp_random.h>
#include <esp_heap_caps.h>
#include <esp_mac.h>            // esp_read_mac: works before the WiFi driver is started
#if LEAF_PORTAL_STREAM
#include "leaf_camera.h"
#endif

// Its own namespace, alongside the radio identity and "tc-ota". One key, and it is a
// secret: nothing else has any business reading it.
#define PORTAL_NVS_NS  "tc-portal"
#define PORTAL_NVS_KEY "ap_psk"

static DNSServer    s_dns;
static WebServer    s_web(80);
static bool         s_up      = false;
static bool         s_web_started = false;
static LeafPortalCmd s_cmd    = LEAF_PORTAL_CMD_NONE;
static char         s_ssid[24] = "";
static char         s_psk[LEAF_PORTAL_PSK_LEN + 1] = "";
static uint32_t     s_deadline_ms = 0;
#if LEAF_PORTAL_STREAM
// The live-view client is held SEPARATELY from the WebServer's own _currentClient. They
// share one socket, but the server hands _currentClient back the moment the peer goes
// away and can immediately accept a different browser into it, so tearing down through
// s_web.client() would eventually stop somebody else's request. This copy always names
// the socket we actually opened the stream on.
static NetworkClient s_stream_cl;
static bool     s_streaming   = false;
static uint32_t s_stream_t0   = 0;
static uint32_t s_stream_n    = 0;
static uint32_t s_stream_bytes = 0;
static int      s_stream_fs   = LEAF_STREAM_FRAMESIZE;
#endif

// Staged status, rendered by the page. Defaults are what an unpopulated field looks like
// rather than zeroes pretending to be measurements.
static char     s_node[40] = "";
static char     s_fw[24]   = "";
static char     s_card[16] = "";
static uint32_t s_boot = 0, s_pir = 0, s_cap = 0, s_fail = 0;
static float    s_vbat = NAN;

// --- key ---------------------------------------------------------------------------
// Open the AP-key namespace wherever this board can keep it. Since leaf-0.23.0 that is
// the durable `id` partition, so the key printed on the label outlives a full 0x0
// reflash and an NVS-full auto-erase, not just the four-piece write it already survived
//. Pre-0.23.0 partition tables have no `id`, and must still work.
static bool open_psk_store(Preferences& p, bool read_only) {
    if (leaf_nvs_durable_available())
        return p.begin(PORTAL_NVS_NS, read_only, LEAF_NVS_DURABLE);
    return p.begin(PORTAL_NVS_NS, read_only);
}

// Minted once per node from the hardware RNG and kept in NVS. Per-node rather than
// derived from the MAC on purpose: the MAC is in the BSSID of every beacon the AP sends,
// so anything derived from it is public by construction, and a shared batch key would
// mean one recovered camera unlocks every camera.
//
// The alphabet drops 0/O/1/I/L: this gets read off a printed label by someone standing in
// the wet, and a key you cannot transcribe is a support call.
static void portal_psk_ensure() {
    if (s_psk[0]) return;
    Preferences p;
    open_psk_store(p, /*readOnly=*/false);
    // isKey() first: getString() on a missing key logs an ESP_LOGE that reads like a
    // fault ("nvs_get_str len fail"), and the FIRST portal use on every node would print
    // it. The key being absent is the normal path exactly once per node, not an error.
    if (p.isKey(PORTAL_NVS_KEY)) p.getString(PORTAL_NVS_KEY, s_psk, sizeof(s_psk));
    if (strlen(s_psk) < 8) {                       // absent, or too short to be WPA2-legal
        static const char AL[] = "23456789ABCDEFGHJKMNPQRSTUVWXYZ";   // 31 symbols
        for (int i = 0; i < LEAF_PORTAL_PSK_LEN; i++)
            s_psk[i] = AL[esp_random() % (sizeof(AL) - 1)];
        s_psk[LEAF_PORTAL_PSK_LEN] = 0;
        p.putString(PORTAL_NVS_KEY, s_psk);
        Serial.println("[portal] AP key minted and stored (first portal use on this node)");
    }
    p.end();
}

// The network name. Derived, not stored, so it costs nothing to compute early — and it
// has to be computable early, because BLE hands the name and key to the phone app BEFORE
// the AP is raised and there is no AP to ask at that point.
//
// esp_read_mac, NOT WiFi.macAddress(). The Arduino call reads the MAC out of the running
// WiFi driver, so before mode() it hands back 00:00:00:00:00:00 — measured on the bench
// 2026-09-03, which is how "meshcam-leaf-000000" got onto the air. The efuse read has no
// such ordering requirement, which is also why the rest of this repo uses it
// (v1_probe.cpp, leaf_ble.cpp), and it is what makes calling this before begin() safe.
//
// ESP_MAC_WIFI_STA is the BASE MAC: the one hand-marked on the board, printed by the
// /dev/serial/by-id path, and reported by esptool. leaf_ble_name() reads ESP_MAC_BT,
// which is base+2 in the last byte, so the BLE name and this SSID differ by two on their
// final digit. Cosmetic, and worth unifying by passing this suffix into leaf_ble_begin(),
// which already takes one.
static void portal_ssid_ensure() {
    if (s_ssid[0]) return;
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_ssid, sizeof(s_ssid), "meshcam-leaf-%02X%02X%02X", mac[3], mac[4], mac[5]);
}

// --- page --------------------------------------------------------------------------
// Node-supplied strings (the slug, the firmware version) are ours, but the card state
// code and anything added later is not necessarily, and this is cheap.
static void html_escape(const char* in, String& out) {
    for (const char* c = in; *c; c++) {
        switch (*c) {
            case '<':  out += F("&lt;");   break;
            case '>':  out += F("&gt;");   break;
            case '&':  out += F("&amp;");  break;
            case '"':  out += F("&quot;"); break;
            case '\'': out += F("&#39;");  break;
            default:   out += *c;
        }
    }
}

static void row(String& h, const __FlashStringHelper* label, const String& value) {
    h += F("<tr><th>");
    h += label;
    h += F("</th><td>");
    h += value;
    h += F("</td></tr>");
}

// Seconds left before the window closes, or 0 if it is already past / unset. Held off
// while a phone is associated, so this counts down and then stops while someone is here.
static uint32_t portal_secs_left() {
    if (!s_deadline_ms) return 0;
    const int32_t d = (int32_t)(s_deadline_ms - millis());
    return d > 0 ? (uint32_t)d / 1000 : 0;
}

static void portal_render() {
    String h;
    h.reserve(2400);
    h += F("<!doctype html><meta charset=utf-8>"
           "<meta name=viewport content='width=device-width,initial-scale=1'>"
           "<meta http-equiv=refresh content=10>"
           "<title>MeshCam camera</title><style>"
           "body{font:16px system-ui,-apple-system,sans-serif;background:#111;color:#ddd;"
           "margin:0;padding:1.5em;max-width:26em}h1{font-size:19px;color:#8f8;margin:0}"
           "table{border-collapse:collapse;width:100%;margin-top:1.2em}"
           "th,td{text-align:left;padding:.45em .2em;border-bottom:1px solid #222;"
           "font-size:14px}th{color:#999;font-weight:400;width:9.5em}"
           "button{font:inherit;width:100%;box-sizing:border-box;padding:.7em;margin-top:1.6em;"
           "border-radius:6px;border:0;background:#2a7;color:#04120a;font-weight:600}"
           "p{color:#999;font-size:13px;line-height:1.5}</style><h1>");
    html_escape(s_node[0] ? s_node : "MeshCam camera", h);
    h += F("</h1><p>You are connected to the camera itself. Nothing here goes over the "
           "internet.");

    h += F("<table>");
    row(h, F("Firmware"), String(s_fw));
    row(h, F("Battery"), isnan(s_vbat) ? String(F("unknown")) : String(s_vbat, 2) + " V");
    row(h, F("Wakes"), String(s_boot));
    row(h, F("Motion events"), String(s_pir));
    row(h, F("Captures"), String(s_cap));
    row(h, F("Failed sends"), String(s_fail));
    if (s_card[0]) { String c; html_escape(s_card, c); row(h, F("Card"), c); }
    row(h, F("Awake for"), String(millis() / 1000) + " s");
    row(h, F("Free memory"), String((unsigned)ESP.getFreeHeap()) + " B");
    {
        const uint32_t left = portal_secs_left();
        row(h, F("Sleeps in"),
            leaf_portal_connected() ? String(F("held while you are connected"))
                                    : String(left) + " s");
    }
    h += F("</table>");

#if LEAF_PORTAL_STREAM
    // First action on the page, ahead of Done: aiming is why somebody walked out here.
    h += F("<p style=\"margin-top:1.6em\"><a href=\"/view\" style=\"display:block;"
           "text-align:center;padding:.7em;border-radius:6px;background:#2a7;color:#04120a;"
           "font-weight:600;text-decoration:none\">Live view &mdash; aim the camera</a>");
#endif
    h += F("<form method=post action=\"/sleep\">"
           "<button type=submit>Done &mdash; let the camera sleep</button></form>"
           "<p>Leaving without pressing Done is fine: the camera closes this network on "
           "its own once you disconnect.");
    s_web.send(200, "text/html", h);
}

static void portal_sleep_post() {
    s_cmd = LEAF_PORTAL_CMD_SLEEP;
    // Answer first, act later. The window teardown drops this phone's association, so
    // doing it here would kill the connection before the confirmation rendered.
    s_web.send(200, "text/html",
               F("<!doctype html><meta charset=utf-8>"
                 "<meta name=viewport content='width=device-width,initial-scale=1'>"
                 "<title>MeshCam camera</title><style>"
                 "body{font:16px system-ui,-apple-system,sans-serif;background:#111;"
                 "color:#ddd;margin:0;padding:1.5em;max-width:26em}"
                 "h1{font-size:19px;color:#8f8;margin:0}p{color:#999;font-size:14px;"
                 "line-height:1.5}</style><h1>Back to sleep</h1>"
                 "<p>This network will disappear in a moment. The camera is watching "
                 "again."));
    Serial.println("[portal] phone asked to end the window");
}

// iOS and Android decide "this network is captive" by fetching their own probe URL and
// not getting the exact body they expect. The redirect is what makes the sheet pop onto
// our page instead of onto a blank tab or a browser error.
static void portal_not_found() {
    s_web.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/");
    s_web.send(302, "text/plain", "");
}

// --- live view ----------------------------------------------------------------------
#if LEAF_PORTAL_STREAM
// Named tiers rather than a raw framesize number: this ends up in a URL a person may type,
// and the three that matter are "fits the screen", "more detail", "smoother".
static int stream_fs_parse(const String& v) {
    if (v == "qvga") return FRAMESIZE_QVGA;   // 320x240, fastest
    if (v == "svga") return FRAMESIZE_SVGA;   // 800x600, most detail
    if (v == "vga")  return FRAMESIZE_VGA;
    return LEAF_STREAM_FRAMESIZE;
}
static const char* stream_fs_name(int fs) {
    switch (fs) {
        case FRAMESIZE_QVGA: return "qvga";
        case FRAMESIZE_SVGA: return "svga";
        default:             return "vga";
    }
}

static void portal_stream_stop(const char* why) {
    if (!s_streaming) return;
    s_streaming = false;
    const uint32_t el = millis() - s_stream_t0;
    Serial.printf("[portal] live view closed (%s): %lu frames in %lu s, %.1f fps, %lu KB\n",
                  why, (unsigned long)s_stream_n, (unsigned long)(el / 1000),
                  el ? s_stream_n * 1000.0f / el : 0.0f,
                  (unsigned long)(s_stream_bytes / 1024));
    // Ours alone to close: the server let go of this socket the moment the handler
    // returned (see portal_stream_open).
    s_stream_cl.stop();
    s_stream_cl = NetworkClient();
    leaf_camera_stream_end();
}

// GET /stream — the MJPEG multipart response. This handler only opens the stream; the
// frames are written by leaf_portal_loop(). See LIVE VIEW in leaf_portal.h for why.
static void portal_stream_open() {
    // Last request wins. Tapping a different size, or reloading the page, opens a second
    // /stream while the first socket is still on the books — the server notices a dead
    // peer only on its next pass, so refusing the new one would make the size buttons
    // fail intermittently. There is one sensor and one operator, so take over.
    if (s_streaming) portal_stream_stop("replaced by a new request");
    const int fs = stream_fs_parse(s_web.arg("fs"));
    if (!leaf_camera_stream_begin(fs, LEAF_STREAM_JPEG_Q)) {
        s_web.send(503, "text/plain", "camera did not start\n");
        return;
    }
    s_stream_fs = fs;
    // TAKE THE SOCKET OUT OF THE SERVER. NetworkClient holds its file descriptor in a
    // shared_ptr and its destructor is empty, so this copy keeps the connection alive
    // after WebServer drops its own reference at the end of this handleClient() pass.
    //
    // The obvious alternative is setSSE(true), which tells WebServer to hold the socket
    // for us. It works, and it is wrong here: WebServer has exactly ONE client slot, so
    // for as long as it is parked on the stream it never calls accept() again and the
    // node answers NOTHING else. Measured on board 0015, 2026-09-03: with the stream
    // open, a second connection did not merely 404, its TCP handshake timed out. A phone
    // re-runs its captive-portal probe on a timer, and a probe that times out is how a
    // phone decides a network is dead and leaves — which would drop the stream the
    // operator is aiming by. Detaching costs nothing and keeps the server answering.
    s_stream_cl = s_web.client();
    // WebServer sets HTTP_MAX_SEND_WAIT (5 s) on its client for request parsing, and the
    // copy inherits it. A frame write is not a request: if the phone is not draining the
    // socket, 1 s is already far longer than the frame is worth, and the rest of the node
    // is stopped for every millisecond of it.
    s_stream_cl.setTimeout(1000);
    s_stream_cl.print(F("HTTP/1.1 200 OK\r\n"
                        "Content-Type: multipart/x-mixed-replace; boundary=mcframe\r\n"
                        "Cache-Control: no-cache, no-store, must-revalidate\r\n"
                        "Pragma: no-cache\r\nConnection: close\r\n\r\n"));
    s_streaming = true;
    s_stream_t0 = millis();
    s_stream_n = s_stream_bytes = 0;
    Serial.printf("[portal] live view opened at %s\n", stream_fs_name(fs));
}

// One frame per call, from the wake loop.
static void portal_stream_pump() {
    if (!s_streaming) return;
    if (!s_stream_cl.connected())                        { portal_stream_stop("client left"); return; }
    if (millis() - s_stream_t0 > LEAF_STREAM_MAX_S * 1000UL) { portal_stream_stop("time cap"); return; }

    LeafFrame f = leaf_camera_stream_frame();
    if (!f.ok()) return;    // a dropped frame is not a dropped stream: a PIR capture in the
                            // same window can be holding the sensor. Try again next pass.
    char hdr[80];
    const int hn = snprintf(hdr, sizeof(hdr),
                            "--mcframe\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                            (unsigned)f.fb->len);
    const size_t want = f.fb->len;
    bool ok = (s_stream_cl.write((const uint8_t*)hdr, hn) == (size_t)hn)
           && (s_stream_cl.write(f.fb->buf, want) == want)
           && (s_stream_cl.write((const uint8_t*)"\r\n", 2) == 2);
    s_stream_bytes += want;
    leaf_camera_release(f);
    if (!ok) { portal_stream_stop("write failed"); return; }
    s_stream_n++;
    if (s_stream_n % 50 == 0) {
        const uint32_t el = millis() - s_stream_t0;
        Serial.printf("[portal] live view: %lu frames, %.1f fps\n",
                      (unsigned long)s_stream_n, el ? s_stream_n * 1000.0f / el : 0.0f);
    }
}

// GET /view — the page the operator actually looks at.
static void portal_view() {
    const int fs = stream_fs_parse(s_web.arg("fs"));
    String h;
    h.reserve(1400);
    h += F("<!doctype html><meta charset=utf-8>"
           "<meta name=viewport content='width=device-width,initial-scale=1'>"
           "<title>MeshCam live view</title><style>"
           "body{font:16px system-ui,-apple-system,sans-serif;background:#000;color:#ddd;"
           "margin:0;padding:.8em;max-width:34em}"
           "img{width:100%;height:auto;display:block;border-radius:6px;background:#111;"
           "min-height:12em}"
           "nav{display:flex;gap:.5em;margin:.8em 0 0}"
           "a{flex:1;text-align:center;padding:.6em .2em;border-radius:6px;background:#1b1b1b;"
           "color:#8f8;text-decoration:none;font-size:14px}"
           "a.on{background:#2a7;color:#04120a;font-weight:600}"
           "p{color:#888;font-size:13px;line-height:1.5}</style>");
    // The stream URL carries the tier so a reload restarts it at the same one, and so a
    // browser that cached the old <img> src does not silently keep the previous tier.
    h += F("<img src=\"/stream?fs=");
    h += stream_fs_name(fs);
    h += F("\" alt=\"live view\"><nav>");
    static const char* const TIERS[] = {"qvga", "vga", "svga"};
    static const char* const LABEL[] = {"Fastest", "Normal", "Detail"};
    for (int i = 0; i < 3; i++) {
        h += F("<a class=\"");
        h += (strcmp(TIERS[i], stream_fs_name(fs)) == 0) ? F("on") : F("");
        h += F("\" href=\"/view?fs=");
        h += TIERS[i];
        h += F("\">");
        h += LABEL[i];
        h += F("</a>");
    }
    h += F("</nav><nav><a href=\"/\">&larr; Camera status</a></nav>"
           "<p>Point the camera while you watch. Nothing here is recorded, and the picture "
           "stops after ");
    h += String((unsigned)LEAF_STREAM_MAX_S / 60);
    h += F(" minutes to save the battery — tap a size above to start it again.");
    s_web.send(200, "text/html", h);
}
#endif  // LEAF_PORTAL_STREAM

// --- lifecycle ---------------------------------------------------------------------
bool leaf_portal_begin() {
    if (s_up) return true;
    portal_psk_ensure();
    portal_ssid_ensure();

    // AP, not AP_STA: nothing here scans and nothing here joins. See leaf_portal.h.
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(s_ssid, s_psk)) {
        Serial.println("[portal] softAP() FAILED -> portal not started");
        WiFi.mode(WIFI_OFF);
        return false;
    }

    s_dns.setErrorReplyCode(DNSReplyCode::NoError);
    s_dns.start(53, "*", WiFi.softAPIP());   // every name resolves to us, so the phone's
                                             // "am I online" probe fails and pops the page
    if (!s_web_started) {
        s_web.on("/", HTTP_GET, portal_render);
        s_web.on("/sleep", HTTP_POST, portal_sleep_post);
#if LEAF_PORTAL_STREAM
        s_web.on("/view",   HTTP_GET, portal_view);
        s_web.on("/stream", HTTP_GET, portal_stream_open);
#endif
        s_web.onNotFound(portal_not_found);
        s_web.begin();
        s_web_started = true;
    }
    s_cmd = LEAF_PORTAL_CMD_NONE;
    s_up  = true;
    // The key goes to the console every time, not just when it is minted: this is where
    // kitting reads it to print the label, and where a bench operator reads it to join.
    Serial.printf("[portal] AP up: join \"%s\" key %s, then http://%s/\n",
                  s_ssid, s_psk, WiFi.softAPIP().toString().c_str());
    return true;
}

void leaf_portal_end() {
    if (!s_up) return;
    s_up = false;
#if LEAF_PORTAL_STREAM
    // Before the AP, not after: this is what powers the sensor back down, and it wants a
    // live socket to close cleanly on.
    portal_stream_stop("window over");
#endif
    s_dns.stop();
    s_web.close();          // stop answering; the listener is re-begun by the next begin()
    s_web_started = false;
    WiFi.softAPdisconnect(true);   // drops associations AND takes the AP interface down
    WiFi.mode(WIFI_OFF);           // no lingering radio — the acceptance criterion
    s_deadline_ms = 0;
    Serial.println("[portal] AP down, WiFi radio off");
}

bool leaf_portal_active()    { return s_up; }
const char* leaf_portal_ssid() { portal_ssid_ensure(); return s_ssid; }
const char* leaf_portal_psk()  { portal_psk_ensure(); return s_psk; }

bool leaf_portal_connected() { return s_up && WiFi.softAPgetStationNum() > 0; }

LeafPortalCmd leaf_portal_take_cmd() {
    const LeafPortalCmd c = s_cmd;
    s_cmd = LEAF_PORTAL_CMD_NONE;
    return c;
}

void leaf_portal_loop() {
    if (!s_up) return;
    s_dns.processNextRequest();
#if LEAF_PORTAL_STREAM
    // Before handleClient(), not after: this is where a departed viewer is detected, and
    // the next request in the queue is very often that same browser reconnecting.
    portal_stream_pump();
#endif
    s_web.handleClient();
}

bool leaf_portal_streaming() {
#if LEAF_PORTAL_STREAM
    return s_streaming;
#else
    return false;
#endif
}

void leaf_portal_set_deadline(uint32_t deadline_abs_ms) { s_deadline_ms = deadline_abs_ms; }

void leaf_portal_set_status(const char* node, const char* fw, uint32_t boot_count,
                            uint32_t pir_wakes, uint32_t captures, uint32_t push_fails,
                            float vbat_v, const char* card) {
    if (node) strlcpy(s_node, node, sizeof(s_node));
    if (fw)   strlcpy(s_fw,   fw,   sizeof(s_fw));
    if (card) strlcpy(s_card, card, sizeof(s_card));
    s_boot = boot_count;
    s_pir  = pir_wakes;
    s_cap  = captures;
    s_fail = push_fails;
    s_vbat = vbat_v;
}

#endif  // LEAF_PORTAL
