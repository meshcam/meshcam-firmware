#pragma once
// Gate B knobs.

// Tensor arena for the tiny detector. This is the dominant RAM cost of TFLite-micro,
// so allocating it (even with a stub model) is the honest proxy for "does the detector
// fit alongside camera + RNS". 512 KB in PSRAM is generous for an INT8 person/animal
// MobileNet-class model; shrink once the real model's arena need is known.
static const size_t kTensorArenaSize = 512 * 1024;

// Where the arena lives. PSRAM keeps it off the scarce internal SRAM (the bottleneck).
// Flip to test the internal-SRAM-pressure worst case.
#define GATE_B_ARENA_IN_PSRAM 1

// Camera capture format/size for the "full-res" stage (goes to PSRAM framebuffer).
#define GATE_B_CAPTURE_FRAMESIZE  FRAMESIZE_UXGA   // 1600x1200, ~ trail-cam still
#define GATE_B_CAPTURE_JPEG_Q     12               // 0-63, lower = bigger/better
#define GATE_B_FB_COUNT           2                // double-buffer like a real capture

// Thumbnail target (what would actually ride the mesh).
#define GATE_B_THUMB_BYTES        (5 * 1024)

// Simulated events to run before reporting the worst-case high-water.
#define GATE_B_EVENTS             10
#define GATE_B_EVENT_GAP_MS       2000

// PASS gate: internal-SRAM free must never dip below this across the whole run.
// (Wi-Fi/BLE-free RNS + camera should leave comfortable headroom; tune on first run.)
#define GATE_B_MIN_INTERNAL_FREE  (32 * 1024)
