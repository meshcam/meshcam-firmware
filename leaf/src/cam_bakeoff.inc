// Camera module bake-off harness (homelab docs/trailcam/camera-bakeoff.md).
//
// WHY THIS EXISTS
// The kit camera SKU + the night-vision stance are ONE decision (fixed IR-cut vs fixed
// no-IR-cut — no cheap DVP module has a mechanical switcher), and it is settled by looking
// at real frames from each candidate module on the SAME board in the SAME scene. The leaf
// firmware proper cannot do that: it deep-sleeps out of setup(), captures on PIR/timer,
// and hard-codes LEAF_FULL_FRAMESIZE to the bench Freenove's OV3660 (QXGA) — which is
// ABOVE the OV2640's UXGA ceiling, so a naive OV2640 grab fails for a reason that has
// nothing to do with the sensor. This harness instead:
//
//   * inits the camera, reports the sensor PID / model / driver max framesize, and grabs
//     at the sensor's OWN max — the real ceiling is recorded from fb->width x fb->height,
//     never assumed from a build flag (that is the "clamp hides the ceiling" trap);
//   * stays awake and serves HTTP: a live MJPEG stream (the TV-remote IR-cut test needs a
//     live view), single-frame captures at any framesize/quality/grayscale, sensor
//     controls, a sustained-capture load loop (so the inline USB meter can be read at
//     active-capture current), and a re-init with a different XCLK;
//   * falls back to a SoftAP if the STA join fails, and to base64-over-serial if there is
//     no WiFi at all — the run must not be blocked by the lab network.
//
// Runs on the Freenove ESP32-S3 (USB-powered). Pin map = leaf_board.h (same DVP header
// the leaf PCB uses). Nothing else of the leaf is brought up: no radio, no detector, no
// store, no sleep. Deliberately a bench tool, not firmware.
//
//   pio run -e cam-bakeoff -t upload ; pio device monitor -b 460800
//   curl http://<ip>/            -> status JSON (sensor, max, PSRAM, ...)
//   curl -o x.jpg 'http://<ip>/jpg?fs=UXGA&q=12'   ; fs=max for the sensor ceiling
//   browser  http://<ip>/stream?fs=VGA               (live view; ?gray=1 for B&W)
//   curl 'http://<ip>/set?gray=1'  /set?awb=0&wb=1  /set?hmirror=1 ...
//   curl 'http://<ip>/load?fs=max&s=20'              (20 s of back-to-back full grabs)
//   curl 'http://<ip>/reinit?xclk=10'                (re-init at 10 MHz XCLK)
//   serial: 'i' status, 'c FS Q GRAY' base64 capture, 'g0'/'g1' grayscale off/on.

#include <WiFi.h>
#include <WebServer.h>
#include <esp_camera.h>
#include <base64.h>
#include "leaf_rails.h"
#include "leaf_board.h"
#if __has_include("wire_secrets.h")
#include "wire_secrets.h"
#define BAKEOFF_HAVE_SECRETS 1
#endif

#ifndef LEAF_BAUD
#define LEAF_BAUD 460800
#endif
#ifndef BAKEOFF_AP_SSID
#define BAKEOFF_AP_SSID "meshcam-bench"
#endif
#ifndef BAKEOFF_AP_PSK
#define BAKEOFF_AP_PSK  "meshcambench"
#endif

static WebServer   g_http(80);
static bool        g_cam_ok   = false;
static int         g_xclk_hz  = 20000000;
static bool        g_gray     = false;
static framesize_t g_max_fs   = FRAMESIZE_INVALID;   // driver's max for THIS sensor
static const char* g_model    = "none";
static uint16_t    g_pid      = 0;
static esp_err_t   g_init_err = ESP_OK;

// framesize name <-> enum, from the driver's own table order (sensor.h)
static const char* FS_NAMES[] = {
    "96X96","QQVGA","128X128","QCIF","HQVGA","240X240","QVGA","320X320","CIF","HVGA",
    "VGA","SVGA","XGA","HD","SXGA","UXGA","FHD","P_HD","P_3MP","QXGA","QHD","WQXGA",
    "P_FHD","QSXGA","5MP" };
static const char* fs_name(int fs) {
    return (fs >= 0 && fs < (int)(sizeof(FS_NAMES)/sizeof(FS_NAMES[0]))) ? FS_NAMES[fs] : "?";
}
static int fs_parse(const String& s, int dflt) {
    if (s.length() == 0) return dflt;
    if (s == "max") return (int)g_max_fs;
    for (int i = 0; i < (int)(sizeof(FS_NAMES)/sizeof(FS_NAMES[0])); i++)
        if (s.equalsIgnoreCase(FS_NAMES[i])) return i;
    if (isDigit(s[0])) return s.toInt();
    return dflt;
}
static const char* model_name(int model) {
    switch (model) {
        case CAMERA_OV2640: return "OV2640";
        case CAMERA_OV3660: return "OV3660";
        case CAMERA_OV5640: return "OV5640";
        case CAMERA_OV7725: return "OV7725";
        case CAMERA_OV7670: return "OV7670";
        case CAMERA_GC2145: return "GC2145";
        case CAMERA_GC032A: return "GC032A";
        case CAMERA_GC0308: return "GC0308";
        default: return "other";
    }
}

// --- camera -----------------------------------------------------------------------
static bool cam_deinit() {
    if (g_cam_ok) { esp_camera_deinit(); g_cam_ok = false; }
    return true;
}

// Init at the driver's biggest framesize. esp32-camera clamps the framebuffer it
// allocates to the detected sensor's max, so this is how you ask "what CAN this module
// do" instead of telling it. The answer is read back from camera_sensor[] and, more
// honestly, from the pixel dimensions of the first grab.
static bool cam_init(int xclk_hz) {
    cam_deinit();
    camera_config_t c = {};
    c.pin_pwdn  = CAM_PIN_PWDN;  c.pin_reset = CAM_PIN_RESET; c.pin_xclk = CAM_PIN_XCLK;
    c.pin_sccb_sda = CAM_PIN_SIOD; c.pin_sccb_scl = CAM_PIN_SIOC;
    c.pin_d7 = CAM_PIN_D7; c.pin_d6 = CAM_PIN_D6; c.pin_d5 = CAM_PIN_D5; c.pin_d4 = CAM_PIN_D4;
    c.pin_d3 = CAM_PIN_D3; c.pin_d2 = CAM_PIN_D2; c.pin_d1 = CAM_PIN_D1; c.pin_d0 = CAM_PIN_D0;
    c.pin_vsync = CAM_PIN_VSYNC; c.pin_href = CAM_PIN_HREF; c.pin_pclk = CAM_PIN_PCLK;
    c.xclk_freq_hz = xclk_hz;
    c.ledc_timer = LEDC_TIMER_0; c.ledc_channel = LEDC_CHANNEL_0;
    c.pixel_format = PIXFORMAT_JPEG;
#ifdef LEAF_FULL_FRAMESIZE
    c.frame_size   = (framesize_t)LEAF_FULL_FRAMESIZE;   // explicit override only
#else
    c.frame_size   = FRAMESIZE_5MP;                      // ask for everything; driver clamps
#endif
    c.jpeg_quality = 12;
    c.fb_count     = 1;
    c.fb_location  = CAMERA_FB_IN_PSRAM;
    c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
    leaf_cam_rail(true);
    uint32_t t0 = millis();
    g_init_err = esp_camera_init(&c);
    g_xclk_hz  = xclk_hz;
    if (g_init_err != ESP_OK) {
        Serial.printf("[cam] esp_camera_init FAILED err=0x%x (%s) after %lu ms\n",
                      g_init_err, esp_err_to_name(g_init_err), (unsigned long)(millis() - t0));
        g_model = "none"; g_pid = 0; g_max_fs = FRAMESIZE_INVALID;
        return false;
    }
    g_cam_ok = true;
    sensor_t* s = esp_camera_sensor_get();
    camera_sensor_info_t* info = s ? esp_camera_sensor_get_info(&s->id) : nullptr;
    g_pid    = s ? s->id.PID : 0;
    g_model  = info ? model_name(info->model) : "unknown";
    g_max_fs = info ? info->max_size : FRAMESIZE_INVALID;
    Serial.printf("[cam] init OK in %lu ms: PID=0x%04x VER=0x%02x MIDH=0x%02x MIDL=0x%02x model=%s "
                  "driver-max=%s(%d) xclk=%d Hz\n",
                  (unsigned long)(millis() - t0), g_pid, s ? s->id.VER : 0, s ? s->id.MIDH : 0,
                  s ? s->id.MIDL : 0, g_model, fs_name(g_max_fs), (int)g_max_fs, xclk_hz);
    // sensible defaults for a fair A/B: everything auto, no effects, no flips
    if (s) {
        s->set_special_effect(s, 0);
        s->set_whitebal(s, 1); s->set_awb_gain(s, 1); s->set_wb_mode(s, 0);
        s->set_exposure_ctrl(s, 1); s->set_aec2(s, 0); s->set_ae_level(s, 0);
        s->set_gain_ctrl(s, 1); s->set_gainceiling(s, GAINCEILING_2X);
        s->set_brightness(s, 0); s->set_contrast(s, 0); s->set_saturation(s, 0);
        s->set_hmirror(s, 0); s->set_vflip(s, 0);
        s->set_lenc(s, 1); s->set_bpc(s, 0); s->set_wpc(s, 1); s->set_raw_gma(s, 1);
    }
    g_gray = false;
    return true;
}

static void cam_apply_gray(bool on) {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) return;
    s->set_special_effect(s, on ? 2 : 0);   // 2 = grayscale in the esp32-camera driver
    g_gray = on;
}

// Set size/quality, discard `settle` frames, then grab. Reports true dims + ms.
static camera_fb_t* cam_grab(int fs, int q, int settle, uint32_t* ms_out) {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) return nullptr;
    if (fs < 0 || fs >= FRAMESIZE_INVALID) fs = (int)g_max_fs;
    // NOT clamped on purpose: if a caller asks above the ceiling we want to SEE what the
    // driver/sensor does with it (that is a finding), and the JSON reports the real dims.
    int rc = s->set_framesize(s, (framesize_t)fs);
    if (rc != 0) Serial.printf("[cam] set_framesize(%s) rc=%d\n", fs_name(fs), rc);
    if (s->set_quality) s->set_quality(s, q);
    for (int i = 0; i < settle; i++) {
        camera_fb_t* w = esp_camera_fb_get();
        if (w) esp_camera_fb_return(w);
    }
    uint32_t t0 = millis();
    camera_fb_t* fb = esp_camera_fb_get();
    if (ms_out) *ms_out = millis() - t0;
    return fb;
}

// --- HTTP ---------------------------------------------------------------------------
static String status_json() {
    String j = "{";
    j += "\"cam_ok\":" + String(g_cam_ok ? "true" : "false");
    j += ",\"init_err\":\"" + String(esp_err_to_name(g_init_err)) + "\"";
    j += ",\"pid\":\"0x" + String(g_pid, HEX) + "\"";
    j += ",\"model\":\"" + String(g_model) + "\"";
    j += ",\"driver_max_fs\":\"" + String(fs_name(g_max_fs)) + "\"";
    j += ",\"driver_max_fs_i\":" + String((int)g_max_fs);
    j += ",\"xclk_hz\":" + String(g_xclk_hz);
    j += ",\"gray\":" + String(g_gray ? "true" : "false");
    j += ",\"psram_free\":" + String(ESP.getFreePsram());
    j += ",\"psram_total\":" + String(ESP.getPsramSize());
    j += ",\"heap_free\":" + String(ESP.getFreeHeap());
    j += ",\"uptime_s\":" + String(millis() / 1000);
    j += ",\"wifi_mode\":\"" + String(WiFi.getMode() == WIFI_AP ? "AP" : "STA") + "\"";
    j += ",\"rssi\":" + String(WiFi.RSSI());
    j += "}";
    return j;
}

static void h_root() { g_http.send(200, "application/json", status_json()); }

static void h_jpg() {
    if (!g_cam_ok) { g_http.send(503, "text/plain", "camera not initialised\n"); return; }
    int fs = fs_parse(g_http.arg("fs"), (int)g_max_fs);
    int q  = g_http.hasArg("q") ? g_http.arg("q").toInt() : 12;
    int settle = g_http.hasArg("settle") ? g_http.arg("settle").toInt() : 3;
    if (g_http.hasArg("gray")) cam_apply_gray(g_http.arg("gray").toInt() != 0);
    uint32_t ms = 0;
    camera_fb_t* fb = cam_grab(fs, q, settle, &ms);
    if (!fb) {
        Serial.printf("[cam] grab fs=%s q=%d -> NULL (fb_get returned nothing)\n", fs_name(fs), q);
        g_http.send(500, "text/plain", String("fb_get returned null at fs=") + fs_name(fs) + "\n");
        return;
    }
    Serial.printf("[cam] grab fs=%s q=%d gray=%d -> %ux%u %u bytes in %lu ms\n",
                  fs_name(fs), q, g_gray, fb->width, fb->height, (unsigned)fb->len, (unsigned long)ms);
    g_http.sendHeader("X-Width",  String(fb->width));
    g_http.sendHeader("X-Height", String(fb->height));
    g_http.sendHeader("X-Len",    String(fb->len));
    g_http.sendHeader("X-Ms",     String(ms));
    g_http.sendHeader("X-Sensor", String(g_model) + " pid=0x" + String(g_pid, HEX));
    g_http.sendHeader("X-Fs",     String(fs_name(fs)));
    g_http.sendHeader("X-Gray",   String(g_gray ? 1 : 0));
    g_http.setContentLength(fb->len);
    g_http.send(200, "image/jpeg", "");
    WiFiClient cl = g_http.client();
    cl.write(fb->buf, fb->len);
    esp_camera_fb_return(fb);
}

// MJPEG live view — for the TV-remote IR-cut test and framing. Runs until the client
// goes away; the WebServer is single-threaded so nothing else answers meanwhile.
static void h_stream() {
    if (!g_cam_ok) { g_http.send(503, "text/plain", "camera not initialised\n"); return; }
    int fs = fs_parse(g_http.arg("fs"), FRAMESIZE_VGA);
    int q  = g_http.hasArg("q") ? g_http.arg("q").toInt() : 12;
    if (g_http.hasArg("gray")) cam_apply_gray(g_http.arg("gray").toInt() != 0);
    WiFiClient cl = g_http.client();
    cl.print("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n"
             "Cache-Control: no-cache\r\nConnection: close\r\n\r\n");
    sensor_t* s = esp_camera_sensor_get();
    s->set_framesize(s, (framesize_t)fs);
    if (s->set_quality) s->set_quality(s, q);
    uint32_t n = 0, t0 = millis();
    // Capped: the WebServer is single-threaded, so an open stream tab blocks /jpg. A
    // forgotten tab must not wedge the bench — the browser just re-opens it (?s= to change).
    uint32_t max_ms = (g_http.hasArg("s") ? g_http.arg("s").toInt() : 45) * 1000UL;
    while (cl.connected() && millis() - t0 < max_ms) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) { delay(10); continue; }
        cl.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)fb->len);
        cl.write(fb->buf, fb->len);
        cl.print("\r\n");
        esp_camera_fb_return(fb);
        n++;
        if (n % 50 == 0) Serial.printf("[stream] %lu frames, %.1f fps\n", (unsigned long)n,
                                       n * 1000.0f / (millis() - t0));
    }
    Serial.printf("[stream] closed after %lu frames\n", (unsigned long)n);
}

static void h_set() {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) { g_http.send(503, "text/plain", "no sensor\n"); return; }
    String out = "";
    for (int i = 0; i < g_http.args(); i++) {
        String k = g_http.argName(i); int v = g_http.arg(i).toInt();
        int rc = -99;
        if      (k == "gray")      { cam_apply_gray(v != 0); rc = 0; }
        else if (k == "awb")       rc = s->set_whitebal(s, v);
        else if (k == "awb_gain")  rc = s->set_awb_gain(s, v);
        else if (k == "wb")        rc = s->set_wb_mode(s, v);        // 0 auto 1 sunny 2 cloudy 3 office 4 home
        else if (k == "aec")       rc = s->set_exposure_ctrl(s, v);
        else if (k == "aec2")      rc = s->set_aec2(s, v);
        else if (k == "ae")        rc = s->set_ae_level(s, v);       // -2..2
        else if (k == "aec_value") rc = s->set_aec_value(s, v);     // 0..1200 (manual)
        else if (k == "agc")       rc = s->set_gain_ctrl(s, v);
        else if (k == "agc_gain")  rc = s->set_agc_gain(s, v);      // 0..30 (manual)
        else if (k == "gainceil")  rc = s->set_gainceiling(s, (gainceiling_t)v); // 0..6 = 2x..128x
        else if (k == "bright")    rc = s->set_brightness(s, v);
        else if (k == "contrast")  rc = s->set_contrast(s, v);
        else if (k == "sat")       rc = s->set_saturation(s, v);
        else if (k == "sharp")     rc = s->set_sharpness ? s->set_sharpness(s, v) : -1;
        else if (k == "hmirror")   rc = s->set_hmirror(s, v);
        else if (k == "vflip")     rc = s->set_vflip(s, v);
        else if (k == "lenc")      rc = s->set_lenc(s, v);
        else if (k == "effect")    rc = s->set_special_effect(s, v);
        else if (k == "denoise")   rc = s->set_denoise ? s->set_denoise(s, v) : -1;
        else if (k == "q")         rc = s->set_quality(s, v);
        else if (k == "fs")        rc = s->set_framesize(s, (framesize_t)fs_parse(g_http.arg(i), (int)g_max_fs));
        out += k + "=" + String(v) + " rc=" + String(rc) + "\n";
    }
    Serial.print(out);
    g_http.send(200, "text/plain", out.length() ? out : "no args\n");
}

// Sustained back-to-back grabs for the inline USB meter (step 7). Reports fps + bytes.
static void h_load() {
    if (!g_cam_ok) { g_http.send(503, "text/plain", "camera not initialised\n"); return; }
    int fs = fs_parse(g_http.arg("fs"), (int)g_max_fs);
    int q  = g_http.hasArg("q") ? g_http.arg("q").toInt() : 12;
    int secs = g_http.hasArg("s") ? g_http.arg("s").toInt() : 20;
    sensor_t* s = esp_camera_sensor_get();
    s->set_framesize(s, (framesize_t)fs);
    if (s->set_quality) s->set_quality(s, q);
    Serial.printf("[load] %d s of back-to-back grabs at %s q=%d — READ THE USB METER NOW\n",
                  secs, fs_name(fs), q);
    uint32_t t0 = millis(), n = 0, bytes = 0, nulls = 0, w = 0, h = 0;
    while (millis() - t0 < (uint32_t)secs * 1000) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) { nulls++; delay(5); continue; }
        n++; bytes += fb->len; w = fb->width; h = fb->height;
        esp_camera_fb_return(fb);
    }
    float el = (millis() - t0) / 1000.0f;
    String r = "fs=" + String(fs_name(fs)) + " dims=" + String(w) + "x" + String(h) +
               " frames=" + String(n) + " nulls=" + String(nulls) +
               " fps=" + String(n / el, 2) + " avg_bytes=" + String(n ? bytes / n : 0) +
               " secs=" + String(el, 1) + "\n";
    Serial.print("[load] " + r);
    g_http.send(200, "text/plain", r);
}

static void h_reinit() {
    int xclk_mhz = g_http.hasArg("xclk") ? g_http.arg("xclk").toInt() : g_xclk_hz / 1000000;
    bool ok = cam_init(xclk_mhz * 1000000);
    g_http.send(ok ? 200 : 500, "application/json", status_json());
}

// --- serial fallback (no WiFi at all) -------------------------------------------------
static void serial_capture_b64(int fs, int q, int gray) {
    if (!g_cam_ok) { Serial.println("ERR no camera"); return; }
    cam_apply_gray(gray != 0);
    uint32_t ms = 0;
    camera_fb_t* fb = cam_grab(fs, q, 3, &ms);
    if (!fb) { Serial.printf("ERR fb null at %s\n", fs_name(fs)); return; }
    Serial.printf("JPG %u %u %u %s %lu\n", (unsigned)fb->len, fb->width, fb->height, g_model, (unsigned long)ms);
    // 3 KB chunks -> 4 KB base64 lines
    for (size_t off = 0; off < fb->len; off += 3072) {
        size_t n = min((size_t)3072, fb->len - off);
        Serial.println(base64::encode(fb->buf + off, n));
    }
    Serial.println("END");
    esp_camera_fb_return(fb);
}

static void serial_poll() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n'); line.trim();
    if (line.length() == 0) return;
    if (line == "i") { Serial.println(status_json()); return; }
    if (line == "g0") { cam_apply_gray(false); Serial.println("gray off"); return; }
    if (line == "g1") { cam_apply_gray(true);  Serial.println("gray on");  return; }
    if (line.startsWith("r")) { int m = line.substring(1).toInt(); cam_init((m > 0 ? m : 20) * 1000000); return; }
    if (line.startsWith("c")) {
        // c FS Q GRAY   e.g. "c UXGA 12 0" / "c max 12 1"
        int sp1 = line.indexOf(' '); int sp2 = line.indexOf(' ', sp1 + 1); int sp3 = line.indexOf(' ', sp2 + 1);
        String fs = sp1 > 0 ? line.substring(sp1 + 1, sp2 > 0 ? sp2 : line.length()) : "max";
        int q = sp2 > 0 ? line.substring(sp2 + 1, sp3 > 0 ? sp3 : line.length()).toInt() : 12;
        int g = sp3 > 0 ? line.substring(sp3 + 1).toInt() : 0;
        serial_capture_b64(fs_parse(fs, (int)g_max_fs), q ? q : 12, g);
        return;
    }
    Serial.println("cmds: i | c FS Q GRAY | g0 | g1 | r<xclk_mhz>");
}

// --- boot ---------------------------------------------------------------------------
static bool wifi_up() {
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
#ifdef BAKEOFF_HAVE_SECRETS
    struct { const char* ssid; const char* psk; } nets[] = {
#ifdef GW_WIFI_SSID2
        { GW_WIFI_SSID2, GW_WIFI_PSK2 },
#endif
#ifdef GW_WIFI_SSID
        { GW_WIFI_SSID,  GW_WIFI_PSK  },
#endif
    };
    for (auto& n : nets) {
        Serial.printf("[wifi] joining \"%s\" ...\n", n.ssid);
        WiFi.disconnect(true); delay(100);
        WiFi.begin(n.ssid, n.psk);
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(250);
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("[wifi] STA up: ip %s rssi %d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
            return true;
        }
    }
#endif
    Serial.printf("[wifi] STA failed -> SoftAP \"%s\" psk \"%s\"\n", BAKEOFF_AP_SSID, BAKEOFF_AP_PSK);
    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(BAKEOFF_AP_SSID, BAKEOFF_AP_PSK);
    Serial.printf("[wifi] AP %s ip %s\n", ok ? "up" : "FAILED", WiFi.softAPIP().toString().c_str());
    return ok;
}

void setup() {
    Serial.begin(LEAF_BAUD);
    Serial.setTimeout(200);
    delay(300);
    Serial.println("\n=== cam-bakeoff harness ===");
    Serial.printf("psram %u total / %u free, heap %u free\n", ESP.getPsramSize(), ESP.getFreePsram(), ESP.getFreeHeap());
    cam_init(g_xclk_hz);
    if (g_cam_ok) {
        // First real grab at the sensor's own ceiling: this is the honest max, from pixels.
        uint32_t ms = 0;
        camera_fb_t* fb = cam_grab((int)g_max_fs, 12, 3, &ms);
        if (fb) {
            Serial.printf("[cam] first grab at driver-max %s -> %ux%u, %u bytes, %lu ms\n",
                          fs_name(g_max_fs), fb->width, fb->height, (unsigned)fb->len, (unsigned long)ms);
            esp_camera_fb_return(fb);
        } else {
            Serial.printf("[cam] first grab at driver-max %s -> NULL\n", fs_name(g_max_fs));
        }
    }
    wifi_up();
    g_http.on("/",       h_root);
    g_http.on("/jpg",    h_jpg);
    g_http.on("/stream", h_stream);
    g_http.on("/set",    h_set);
    g_http.on("/load",   h_load);
    g_http.on("/reinit", h_reinit);
    g_http.onNotFound([]{ g_http.send(404, "text/plain", "/ /jpg /stream /set /load /reinit\n"); });
    g_http.begin();
    Serial.println("[http] listening on :80");
    Serial.println(status_json());
}

void loop() {
    g_http.handleClient();
    serial_poll();
    static uint32_t last = 0;
    if (millis() - last > 30000) { last = millis(); Serial.println("[tick] " + status_json()); }
    delay(2);
}
