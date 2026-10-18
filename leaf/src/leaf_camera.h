#pragma once
/*
 * Leaf camera path — lazy init, thumbnail + full-res capture, tear back down.
 *
 * Power discipline: the OV sensor is only ever powered during a PIR capture (or a
 * fetch_full live fallback), never on a timer check-in. The driver is initialized at the
 * FULL framesize (framebuffer sized for the largest grab), then set_framesize() switches
 * between the small alert thumbnail and the stored full-res without re-init.
 *
 * Bandwidth discipline: over LoRa (~a few kbps effective) the alert IS a small,
 * heavily-compressed JPEG captured directly at thumbnail framesize. The full-res copy is
 * grabbed after the thumb and STORED locally (leaf_store); it only travels when a gateway
 * command asks for that event_id.
 */
#include <Arduino.h>
#include "esp_camera.h"

// A captured frame. `fb` is owned by the camera driver; read fb->buf / fb->len, hand the
// bytes off, then release (leaf_camera_release for mid-session, leaf_camera_end when done).
struct LeafFrame {
    camera_fb_t* fb = nullptr;
    bool ok() const { return fb != nullptr; }
};

// Init the sensor (at full framesize), switch to the alert (thumbnail) size, grab one
// frame after AEC warmup. Returns a LeafFrame; .ok() false on any failure.
LeafFrame leaf_camera_capture();

// Grab a full-res frame (switches framesize up; discards one settle frame). Camera is
// initialized if needed, so this also serves the standalone fetch_full live fallback.
LeafFrame leaf_camera_capture_full();

// Return a framebuffer to the driver without powering the sensor down (use between the
// thumb grab and the full grab).
void leaf_camera_release(LeafFrame& frame);

// Return the framebuffer (if any) and power the sensor back down. Safe after a failure.
void leaf_camera_end(LeafFrame& frame);

// --- live view ----------------------------------------------------------------------
// A continuous low-resolution grab for the aiming stream, on the SAME sensor and the same
// lazy init as the capture path above. Deliberately not a second camera driver: the two
// never run at once in wall-clock terms, they interleave inside one service window, and
// two configs of one driver is how the framesize switch already works.
//
// The stream framesize is small on purpose. This exists so a person can see what the
// camera sees while they point it at a trail, not to take a picture — LEAF_FULL_FRAMESIZE
// is untouched and a PIR capture in the middle of a stream still stores its QXGA original.

// Stream detail tiers. VGA is the default: it fills a phone screen, and at ~20 KB a frame
// it moves fast enough over the SoftAP to look live.
#ifndef LEAF_STREAM_FRAMESIZE
#define LEAF_STREAM_FRAMESIZE FRAMESIZE_VGA
#endif
#ifndef LEAF_STREAM_JPEG_Q
#define LEAF_STREAM_JPEG_Q 14
#endif

// Enter streaming mode at the given framesize/quality (framesize_t values). Initializes
// the sensor if it is cold. Returns false if the camera would not come up.
bool leaf_camera_stream_begin(int framesize, int quality);

// One stream frame, or .ok() false. Self-healing: a PIR capture sharing this window
// deinitializes the driver and moves the framesize, and both are the normal case, so this
// re-establishes streaming mode rather than failing. Release with leaf_camera_release().
LeafFrame leaf_camera_stream_frame();

// Leave streaming mode and power the sensor back down. Safe when no stream was running.
void leaf_camera_stream_end();

// Downscale a stored QXGA original to the "standard" tier (~800x600 JPEG) for a
// quality:"standard" fetch_full. Pure transform — never touches the camera. Returns a
// malloc'd buffer (caller frees with free()), or nullptr on failure (caller should then
// send the original as-is; graceful degradation per the feature request).
uint8_t* leaf_jpeg_downscale(const uint8_t* jpeg, size_t len, size_t* out_len);
