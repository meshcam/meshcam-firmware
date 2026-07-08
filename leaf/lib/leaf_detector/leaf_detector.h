#pragma once
/*
 * Leaf detector — INT8 TFLite-micro inference to reject false PIR triggers.
 *
 * On a PIR wake the AM312 fires on any warm motion: an actual animal/person, but also
 * blowing branches, a passing car's heat, sun on foliage. Sending every one of those over
 * LoRa wastes the leaf's scarce radio energy and floods the gateway. This runs a real
 * quantized detector on the captured frame and returns whether there's a subject worth
 * sending.
 *
 * MODEL: the canonical TFLite-micro `person_detection` (MobileNet-ish, 96x96x1 grayscale,
 * INT8, ~300 KB). It detects PEOPLE — genuinely useful for a trail cam (trespasser alerts)
 * and it proves the full INT8 pipeline on real silicon within the arena budget gate-b
 * established. IMPORTANT: a person-only model cannot safely gate an ANIMAL cam (a deer
 * reads as "no person"), so the production detector must be multi-class (deer/person/empty).
 * Swapping models = replace the model array + labels + input dims; the plumbing here stays.
 * See LEAF_DETECT_GATE below for how the send policy handles this today.
 */
#include <Arduino.h>
#include "esp_camera.h"

// Load the model, allocate the tensor arena (PSRAM), AllocateTensors. Idempotent per boot.
// Returns false if setup failed (bad model / arena alloc). Prints arena high-water.
bool leaf_detector_begin();

// Result of one inference on a captured frame.
struct LeafDetection {
    bool   ok        = false;   // inference actually ran
    bool   subject   = false;   // a subject (person) was detected above threshold
    float  score     = 0.0f;    // person probability 0..1
    uint32_t infer_ms = 0;
};

// Decode the captured JPEG (w x h), center-crop + downscale to 96x96 grayscale, run
// inference. Takes raw bytes (not a framebuffer) so the camera can be powered down first.
LeafDetection leaf_detector_run(const uint8_t* jpeg, size_t len, int w, int h);
