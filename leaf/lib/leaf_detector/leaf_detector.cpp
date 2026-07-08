#include "leaf_detector.h"
#include "person_detect_model_data.h"

#include <tflm_esp32.h>                 // eloquentarduino TFLite-micro runtime (ESP-NN)
#include "esp_heap_caps.h"
#include "img_converters.h"            // fmt2rgb888 (JPEG decode), part of esp32-camera

// Model geometry (person_detection). If you swap the model, update these.
static const int   kInW = 96, kInH = 96;
static const int   kPersonIndex = 1;   // output class order: [not-person, person]

// person_detection needs ~80 KB of arena; 160 KB gives headroom. Prefer INTERNAL SRAM
// (gate-b measured ~316 KB free alongside camera + RNS, so it fits) and fall back to PSRAM
// on a fragmentation failure. NOTE: arena location does NOT move inference time here —
// measured 4141 ms internal vs 4145 ms PSRAM. Inference is COMPUTE-bound: this build's
// TFLite-micro CONV/DEPTHWISE_CONV kernels are the reference (non-ESP-NN-accelerated) path.
// ~4 s/inference is a real awake-time/battery cost; still a net win when it suppresses a
// multi-second high-power LoRa TX of a false trigger. FUTURE OPTIMIZATION: confirm/enable
// ESP-NN dispatch (or a lighter model / smaller input) to get this to sub-second.
static const size_t kArenaSize = 160 * 1024;

// Send-gate threshold on the dequantized person probability. Model-dependent: for this
// person_detection model, empty/motion sits ~0.25 and a framed person ~0.45+, so 0.40
// separates them. Retune against real footage when a multi-class deer model goes in.
#ifndef LEAF_DETECT_THRESHOLD
#define LEAF_DETECT_THRESHOLD 0.40f
#endif

static uint8_t*             s_arena   = nullptr;
static const tflite::Model* s_model   = nullptr;
static tflite::MicroInterpreter* s_interp = nullptr;
static TfLiteTensor*        s_input   = nullptr;
static TfLiteTensor*        s_output  = nullptr;
static bool                 s_ready   = false;

bool leaf_detector_begin() {
    if (s_ready) return true;

    s_model = tflite::GetModel(g_person_detect_model_data);
    if (s_model->version() != TFLITE_SCHEMA_VERSION) {
        Serial.printf("[det] model schema %lu != runtime %d\n",
                      (unsigned long)s_model->version(), TFLITE_SCHEMA_VERSION);
        return false;
    }

    bool arena_internal = true;
    if (!s_arena) {
        s_arena = (uint8_t*)heap_caps_malloc(kArenaSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_arena) {   // fragmentation fallback: slower PSRAM arena
            arena_internal = false;
            s_arena = (uint8_t*)heap_caps_malloc(kArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        }
        if (!s_arena) { Serial.println("[det] arena alloc FAILED"); return false; }
    }

    // person_detection uses exactly these 5 ops.
    static tflite::MicroMutableOpResolver<5> resolver;
    resolver.AddAveragePool2D();
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddReshape();
    resolver.AddSoftmax();

    static tflite::MicroInterpreter interp(s_model, resolver, s_arena, kArenaSize);
    s_interp = &interp;
    if (s_interp->AllocateTensors() != kTfLiteOk) {
        Serial.println("[det] AllocateTensors FAILED (arena too small?)");
        return false;
    }
    s_input  = s_interp->input(0);
    s_output = s_interp->output(0);

    Serial.printf("[det] ready: in=%dx%dx%d %s, arena used=%u/%u KB (%s)\n",
                  s_input->dims->data[1], s_input->dims->data[2], s_input->dims->data[3],
                  s_input->type == kTfLiteInt8 ? "int8" : "?",
                  (unsigned)(s_interp->arena_used_bytes() / 1024), (unsigned)(kArenaSize / 1024),
                  arena_internal ? "internal SRAM" : "PSRAM");
    s_ready = true;
    return true;
}

// Center-crop the (w x h) BGR888 frame to a square, box-average down to 96x96 luma, and
// write int8 = (uint8 luma) ^ 0x80 straight into the model input (person_detection's
// expected input encoding). esp32-camera's fmt2rgb888 emits bytes in B,G,R order.
static void frame_to_input(const uint8_t* bgr, int w, int h, int8_t* out) {
    const int side = (w < h) ? w : h;         // largest centered square
    const int ox = (w - side) / 2, oy = (h - side) / 2;
    for (int ry = 0; ry < kInH; ry++) {
        for (int rx = 0; rx < kInW; rx++) {
            // source block in the cropped square for this output pixel
            const int sx0 = ox + (rx * side) / kInW, sx1 = ox + ((rx + 1) * side) / kInW;
            const int sy0 = oy + (ry * side) / kInH, sy1 = oy + ((ry + 1) * side) / kInH;
            uint32_t acc = 0, n = 0;
            for (int sy = sy0; sy < sy1; sy++) {
                const uint8_t* row = bgr + (size_t)(sy * w + sx0) * 3;
                for (int sx = sx0; sx < sx1; sx++) {
                    // luma from BGR (Rec.601, integer approx)
                    const uint32_t b = row[0], g = row[1], r = row[2];
                    acc += (r * 77 + g * 150 + b * 29) >> 8;
                    row += 3; n++;
                }
            }
            const uint8_t luma = n ? (uint8_t)(acc / n) : 0;
            out[ry * kInW + rx] = (int8_t)(luma ^ 0x80);
        }
    }
}

LeafDetection leaf_detector_run(const uint8_t* jpeg, size_t len, int w, int h) {
    LeafDetection d;
    if (!leaf_detector_begin() || !jpeg || !len) return d;

    // Decode the JPEG to BGR888 in PSRAM.
    uint8_t* bgr = (uint8_t*)heap_caps_malloc((size_t)w * h * 3, MALLOC_CAP_SPIRAM);
    if (!bgr) { Serial.println("[det] rgb888 alloc FAILED"); return d; }
    if (!fmt2rgb888(jpeg, len, PIXFORMAT_JPEG, bgr)) {
        Serial.println("[det] JPEG decode FAILED");
        heap_caps_free(bgr);
        return d;
    }

    frame_to_input(bgr, w, h, s_input->data.int8);
    heap_caps_free(bgr);

    uint32_t t0 = millis();
    if (s_interp->Invoke() != kTfLiteOk) { Serial.println("[det] Invoke FAILED"); return d; }
    d.infer_ms = millis() - t0;

    // Dequantize the person score to a 0..1 probability.
    const int8_t raw = s_output->data.int8[kPersonIndex];
    d.score = (raw - s_output->params.zero_point) * s_output->params.scale;
    d.ok = true;
    d.subject = d.score >= LEAF_DETECT_THRESHOLD;
    return d;
}
