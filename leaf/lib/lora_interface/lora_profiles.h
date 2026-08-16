#pragma once
/*
 * Shared LoRa radio-profile ladder — the over-the-air ADR contract.
 *
 * Both ends of a link must run the IDENTICAL SF/BW/CR or they are mutually deaf, so
 * profiles are negotiated by INDEX into this table, never as free parameters. The file
 * is vendored verbatim in EVERY firmware tree's lib/lora_interface (gateway, leaf,
 * surveyor, bridges — homelab prototype AND meshcam-firmware) — keep all copies
 * byte-identical, and NEVER
 * reorder or remove entries (an index in an in-flight radio_profile grant must mean the
 * same thing to old and new firmware; append only).
 *
 * The ADR scheme (gateway-0.4.0 / leaf-0.10.0; decision inputs reworked gateway-0.12.0):
 *   - LORA_PROFILE_BASE is the rendezvous: both sides fall back to it on any confirm
 *     timeout / grant expiry / power loss, so a missed switch can never strand the link.
 *   - The gateway measures the leaf's direct announces and grants faster (or slower)
 *     profiles via {"kind":"radio_profile","payload":{"idx":N,"ttl_s":S}} into the
 *     leaf's RX window.
 *   - snr_floor is the LoRa demodulation limit for the profile (Semtech SX126x datasheet
 *     figures; BW250 entries already pay the ~3 dB wider-noise-floor penalty in their
 *     floor).
 *   - rssi_floor is the receiver sensitivity for the profile (same datasheet, dBm).
 *     Since gateway-0.12.0 this is the PRIMARY grant input: the SX126x SNR readout
 *     saturates around +13 dB, which held the old SNR-only policy constant for its
 *     first 534 grants (homelab docs/trailcam/probe-analysis.md) — RSSI is the one
 *     instrument with dynamic range at real distances. SNR remains a secondary gate
 *     for interference-limited links (strong RSSI, no demod margin).
 */
#include <stdint.h>

struct LoRaProfile {
    uint8_t     sf;         // spreading factor
    float       bw_khz;     // bandwidth (RadioLib units: kHz)
    uint8_t     cr;         // coding rate denominator (4/cr)
    float       snr_floor;  // demod limit, dB — grants keep headroom above this
    float       rssi_floor; // receiver sensitivity, dBm (SX126x datasheet, conservative)
    uint16_t    raw_bps;    // raw bytes/sec (bitrate formula / 8) for goodput math
    const char* name;
};

// idx:            0 = far / robust, climbing toward near / fast. Append only.
//
// ⚠️ 0-2 are the LEGACY bw125/250 rungs and are OUTSIDE the RAK3172 grant.
//    3-5 are the certified bw500 ladder. See the block under the table.
static const LoRaProfile LORA_PROFILES[] = {
    /* 0 far  */ { 10, 125.0f, 5, -15.0f, -132.0f,  122, "sf10/bw125" },
    /* 1 base */ {  8, 125.0f, 5, -10.0f, -126.0f,  390, "sf8/bw125"  },
    /* 2 near */ {  7, 250.0f, 5,  -4.5f, -120.0f, 1367, "sf7/bw250"  },
    // --- bw500 certified ladder, added 2026-08-07 ----------------------------
    /* 3 cert-far  */ { 12, 500.0f, 5, -14.0f, -130.0f,  146, "sf12/bw500" },
    /* 4 cert-base */ { 10, 500.0f, 5,  -9.0f, -126.0f,  488, "sf10/bw500" },
    /* 5 cert-near */ {  8, 500.0f, 5,  -4.0f, -120.0f, 1562, "sf8/bw500"  },
};
#define LORA_PROFILE_COUNT ((uint8_t)(sizeof(LORA_PROFILES) / sizeof(LORA_PROFILES[0])))

/*
 * The bw500 certified ladder (idx 3-5) — why these rungs, and the rules.
 *
 * §15.247(a)(2) requires a >=500 kHz 6 dB bandwidth to be a DTS, and the RAK3172's
 * 24.1 dBm grant was issued on exactly that basis. LoRa at BW=125/250 kHz produces a
 * 6 dB BW below 500 kHz and is therefore OUTSIDE the grant — which means rungs 0-2
 * cannot legally be used on a certified-config SKU at granted power.
 * **Firmware must lock BW=500 kHz on every certified-config build.**
 * See docs/trailcam/fcc-certification.md, Findings 3c/3d and "hard firmware rule".
 *
 * The 3 rungs are the equivalence mapping already worked out in fcc-certification.md
 * (the table at "The clean answer is a bw500 DTS ladder"), not a fresh invention:
 *     sf10/bw125 (far)  -> sf12/bw500   ~2-3 dB worse,  ~2x faster
 *     sf8/bw125  (base) -> sf10/bw500   ~equal,         ~1.25x faster
 *     sf7/bw250  (near) -> sf8/bw500    ~equal class,   ~2x faster
 * sf8/bw500 is chosen for "near" over sf7/bw500 because SF8/BW500 is the exact
 * configuration in the grant's tested DTS lane, and it is what the 2026-08-07 bench
 * interop actually ran (leaf-v1.0-respin-spec.md §0a).
 *
 * ⚠️ APPEND-ONLY BREAKS THE far->near ORDERING. idx 3 is a FAR rung sitting after a
 * NEAR one. That is forced: this file's contract is that an index in an in-flight
 * radio_profile grant must mean the same thing to old and new firmware, so reordering
 * is not allowed. Any grant policy that "shifts up" by doing idx+1 is now WRONG and
 * must select on snr_floor/rssi_floor (which is why those fields exist) or clamp to
 * one family via LORA_PROFILE_MIN/MAX. Verify the gateway's policy before enabling.
 *
 * snr_floor follows this file's existing convention of folding the wider-noise-floor
 * penalty into the floor (as the bw250 row does at +3 dB): SX126x demod limits
 * SF8 -10 / SF10 -15 / SF12 -20, each +6 dB for BW500 vs BW125.
 * rssi_floor is the SX1262 datasheet BW500 sensitivity, rounded 1 dB conservative,
 * matching how rows 0-2 were derived.
 * raw_bps = SF * (BW / 2^SF) * 4/5, / 8 for bytes, floored. EVERY row now matches that
 * formula exactly — verified by compiling the table and recomputing each entry.
 *
 *   ⚠️ Row 1 (sf8/bw125) read **488** until 2026-08-07; the formula gives **390**. 488 is
 *   that row's *symbol* rate (125000 / 2^8 = 488.28), so it was a transcription slip, and
 *   it had overstated sf8/bw125 goodput by 25% since the table was written. FIXED.
 *   Consequences were small and both in the safe direction, which is why it survived:
 *     - surveyor probe_timeout_ms() divides by raw_bps, so the old value under-allowed
 *       transfer time by 25%. It carries a 3x factor, +45 s, and a 60-480 s clamp, so the
 *       correction just makes an already-padded timeout correct rather than changing
 *       behaviour dramatically.
 *     - the gateway reports raw_bps in telemetry JSON and a log line — it was simply
 *       reporting a wrong number.
 */

// The rendezvous profile. Overridable per build (-DLORA_PROFILE_BASE=0 for a far-field
// deployment where sf8 announces wouldn't be heard at all).
//
// ⚠️ STILL 1 (sf8/bw125), i.e. the LEGACY ladder, ON PURPOSE. Moving the rendezvous to
// the certified ladder is a FLAG DAY: a bw500 node and a bw125 node are mutually deaf,
// so every node has to be reflashed together (leaf-v1.0-respin-spec.md §11). Adding the
// rungs above changes nothing until a build opts in, which is what makes it safe to land.
// Certified-config builds set -DLORA_PROFILE_BASE=LORA_PROFILE_CERT_BASE.
#ifndef LORA_PROFILE_BASE
#define LORA_PROFILE_BASE 1
#endif

// The rendezvous rung of the certified ladder — the bw500 equivalent of base idx 1.
#define LORA_PROFILE_CERT_BASE 4
