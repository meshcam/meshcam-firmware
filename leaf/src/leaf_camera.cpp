#include "leaf_camera.h"
#include "leaf_board.h"

// Alert-thumbnail capture params (what goes over LoRa / the bench serial link).
#ifndef LEAF_ALERT_FRAMESIZE
#define LEAF_ALERT_FRAMESIZE FRAMESIZE_QVGA   // 320x240, ~4 KB at q14
#endif
#ifndef LEAF_ALERT_JPEG_Q
#define LEAF_ALERT_JPEG_Q 14                  // 0..63, higher = smaller/uglier
#endif
// Full-res params: the STORED ORIGINAL is sensor-max (feature request 2026-07-02) — the
// OV3660 does QXGA 2048x1536, ~200-400 KB at q12, and the 12.9 MB LittleFS partition
// holds ~40 of them. What goes over the AIR is a separate decision: fetch_full
// quality="standard" downscales this original to LEAF_STD_* below before sending.
#ifndef LEAF_FULL_FRAMESIZE
#define LEAF_FULL_FRAMESIZE FRAMESIZE_QXGA    // 2048x1536, sensor max
#endif
#ifndef LEAF_FULL_JPEG_Q
#define LEAF_FULL_JPEG_Q 12
#endif
// "standard" quality tier: the mesh-affordable size a plain fetch_full gets.
#ifndef LEAF_STD_W
#define LEAF_STD_W 800
#endif
#ifndef LEAF_STD_H
#define LEAF_STD_H 600
#endif
#ifndef LEAF_STD_JPEG_Q
#define LEAF_STD_JPEG_Q 60                    // fmt2jpg/jpge scale: 1-100, HIGHER = better.
                                              // NOTE fmt2jpg writes into a fixed 128 KB
                                              // internal buffer and silently truncates at
                                              // it — q80 at 800x600 blew past it (found
                                              // 2026-07-02: "downscale" was 131072 = the
                                              // cap, a corrupt JPEG). q60 lands ~40-70 KB.
#endif

#include "leaf_rails.h"

static bool s_inited = false;

static bool camera_init_full() {
    if (s_inited) return true;
    camera_config_t c = {};
    c.pin_pwdn  = CAM_PIN_PWDN;  c.pin_reset = CAM_PIN_RESET; c.pin_xclk = CAM_PIN_XCLK;
    c.pin_sccb_sda = CAM_PIN_SIOD; c.pin_sccb_scl = CAM_PIN_SIOC;
    c.pin_d7 = CAM_PIN_D7; c.pin_d6 = CAM_PIN_D6; c.pin_d5 = CAM_PIN_D5; c.pin_d4 = CAM_PIN_D4;
    c.pin_d3 = CAM_PIN_D3; c.pin_d2 = CAM_PIN_D2; c.pin_d1 = CAM_PIN_D1; c.pin_d0 = CAM_PIN_D0;
    c.pin_vsync = CAM_PIN_VSYNC; c.pin_href = CAM_PIN_HREF; c.pin_pclk = CAM_PIN_PCLK;
    c.xclk_freq_hz = 20000000;
    c.ledc_timer = LEDC_TIMER_0; c.ledc_channel = LEDC_CHANNEL_0;
    c.pixel_format = PIXFORMAT_JPEG;
    // Init at the LARGEST size we'll grab: the driver sizes the framebuffer from this,
    // and set_framesize() can then move freely at/below it without re-init.
    c.frame_size   = LEAF_FULL_FRAMESIZE;
    c.jpeg_quality = LEAF_FULL_JPEG_Q;
    c.fb_count     = 1;
    c.fb_location  = CAMERA_FB_IN_PSRAM;
    c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
    leaf_cam_rail(true);   // PCB: power CAM_3V3 (no-op on bench)
    esp_err_t err = esp_camera_init(&c);
    if (err != ESP_OK) { Serial.printf("[cam] esp_camera_init failed: 0x%x\n", err); return false; }
    s_inited = true;
    return true;
}

// Switch framesize/quality and discard `settle` frames (sensor + AEC need a beat after a
// mode change; on a cold sensor the first frames are washed out).
static bool set_mode(framesize_t size, int quality, int settle) {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) return false;
    s->set_framesize(s, size);
    if (s->set_quality) s->set_quality(s, quality);
    for (int i = 0; i < settle; i++) {
        camera_fb_t* warm = esp_camera_fb_get();
        if (warm) esp_camera_fb_return(warm);
    }
    return true;
}

LeafFrame leaf_camera_capture() {
    LeafFrame out;
    if (!camera_init_full()) return out;
    if (!set_mode((framesize_t)LEAF_ALERT_FRAMESIZE, LEAF_ALERT_JPEG_Q, 2)) return out;
    out.fb = esp_camera_fb_get();
    if (!out.fb) Serial.println("[cam] thumb capture (fb_get) returned null");
    return out;
}

LeafFrame leaf_camera_capture_full() {
    LeafFrame out;
    const bool cold = !s_inited;
    if (!camera_init_full()) return out;
    // Cold start needs full AEC warmup; a same-session size switch just needs to settle.
    if (!set_mode((framesize_t)LEAF_FULL_FRAMESIZE, LEAF_FULL_JPEG_Q, cold ? 2 : 1)) return out;
    out.fb = esp_camera_fb_get();
    if (!out.fb) Serial.println("[cam] full capture (fb_get) returned null");
    return out;
}

void leaf_camera_release(LeafFrame& frame) {
    if (frame.fb) { esp_camera_fb_return(frame.fb); frame.fb = nullptr; }
}

void leaf_camera_end(LeafFrame& frame) {
    leaf_camera_release(frame);
    if (s_inited) { esp_camera_deinit(); s_inited = false; leaf_cam_rail(false); }
}

// --- quality-tier transform: stored QXGA original -> "standard" ~800x600 JPEG ----------
// Decode at 1/2 scale (QXGA -> 1024x768 RGB565, 1.5 MB — a full-scale RGB888 decode would
// be 9.4 MB and not fit PSRAM), box-average down to LEAF_STD_W x LEAF_STD_H RGB888, and
// re-encode. Runs only on a fetch_full quality="standard" — never touches the camera.
#include "img_converters.h"

uint8_t* leaf_jpeg_downscale(const uint8_t* jpeg, size_t len, size_t* out_len) {
    if (out_len) *out_len = 0;
    if (!jpeg || !len) return nullptr;

    const int dw = 2048 / 2, dh = 1536 / 2;   // QXGA at JPG_SCALE_2X
    uint8_t* rgb565 = (uint8_t*)heap_caps_malloc((size_t)dw * dh * 2, MALLOC_CAP_SPIRAM);
    if (!rgb565) { Serial.println("[img] rgb565 alloc FAILED"); return nullptr; }
    if (!jpg2rgb565(jpeg, len, rgb565, JPG_SCALE_2X)) {
        // Not a QXGA original (legacy/odd file) — caller falls back to sending it as-is.
        Serial.println("[img] decode-at-2x failed (non-QXGA original?)");
        heap_caps_free(rgb565);
        return nullptr;
    }

    uint8_t* rgb = (uint8_t*)heap_caps_malloc((size_t)LEAF_STD_W * LEAF_STD_H * 3, MALLOC_CAP_SPIRAM);
    if (!rgb) { Serial.println("[img] rgb888 alloc FAILED"); heap_caps_free(rgb565); return nullptr; }

    // Box-average each output pixel from its source region (dw x dh -> STD_W x STD_H).
    for (int oy = 0; oy < LEAF_STD_H; oy++) {
        const int sy0 = (oy * dh) / LEAF_STD_H, sy1 = ((oy + 1) * dh) / LEAF_STD_H;
        for (int ox = 0; ox < LEAF_STD_W; ox++) {
            const int sx0 = (ox * dw) / LEAF_STD_W, sx1 = ((ox + 1) * dw) / LEAF_STD_W;
            uint32_t r = 0, g = 0, b = 0, n = 0;
            for (int sy = sy0; sy < sy1; sy++) {
                const uint8_t* row = rgb565 + ((size_t)sy * dw + sx0) * 2;
                for (int sx = sx0; sx < sx1; sx++) {
                    // jpg2rgb565 stores LOW byte first (_rgb565_write in to_bmp.c:
                    // o[i]=c&0xff, o[i+1]=c>>8) — reading high-first scrambles the
                    // bit-fields into psychedelic colors (bug found 2026-07-02).
                    const uint16_t px = ((uint16_t)row[1] << 8) | row[0];
                    r += (px >> 11) & 0x1F; g += (px >> 5) & 0x3F; b += px & 0x1F;
                    row += 2; n++;
                }
            }
            // fmt2jpg's PIXFORMAT_RGB888 expects B,G,R in memory (convert_line_format
            // reverses to feed the encoder) — same convention as fmt2rgb888 output.
            uint8_t* o = rgb + ((size_t)oy * LEAF_STD_W + ox) * 3;
            if (n) { o[0] = (b / n) << 3; o[1] = (g / n) << 2; o[2] = (r / n) << 3; }
            else   { o[0] = o[1] = o[2] = 0; }
        }
    }
    heap_caps_free(rgb565);

    uint8_t* out = nullptr;
    size_t   olen = 0;
    bool ok = fmt2jpg(rgb, (size_t)LEAF_STD_W * LEAF_STD_H * 3, LEAF_STD_W, LEAF_STD_H,
                      PIXFORMAT_RGB888, LEAF_STD_JPEG_Q, &out, &olen);
    heap_caps_free(rgb);
    if (!ok || !out || !olen) {
        Serial.println("[img] re-encode FAILED");
        if (out) free(out);
        return nullptr;
    }
    if (olen >= 128 * 1024) {
        // fmt2jpg's fixed internal buffer filled -> the JPEG was TRUNCATED (corrupt).
        // Fail so the caller sends the original instead of a broken image.
        Serial.printf("[img] re-encode hit the 128 KB buffer cap (truncated) -> FAILED\n");
        free(out);
        return nullptr;
    }
    if (out_len) *out_len = olen;
    return out;   // caller frees with free() (fmt2jpg allocs)
}
