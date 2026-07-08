#pragma once
/*
 * Shared LoRa radio-profile ladder — the over-the-air ADR contract.
 *
 * Both ends of a link must run the IDENTICAL SF/BW/CR or they are mutually deaf, so
 * profiles are negotiated by INDEX into this table, never as free parameters. The file
 * is vendored verbatim in gateway-heltec/lib/lora_interface AND
 * leaf-sleep-wake/lib/lora_interface — keep the two copies byte-identical, and NEVER
 * reorder or remove entries (an index in an in-flight radio_profile grant must mean the
 * same thing to old and new firmware; append only).
 *
 * The ADR scheme (gateway-0.4.0 / leaf-0.10.0):
 *   - LORA_PROFILE_BASE is the rendezvous: both sides fall back to it on any confirm
 *     timeout / grant expiry / power loss, so a missed switch can never strand the link.
 *   - The gateway measures announce SNR and grants faster (or slower) profiles via
 *     {"kind":"radio_profile","payload":{"idx":N,"ttl_s":S}} into the leaf's RX window.
 *   - snr_floor is the LoRa demodulation limit for the profile (Semtech SX126x datasheet
 *     figures; BW250 entries already pay the ~3 dB wider-noise-floor penalty in their
 *     floor). Grant policy: upgrade only when rolling SNR >= floor(next) + 12 dB,
 *     downgrade when rolling SNR <= floor(current) + 6 dB.
 */
#include <stdint.h>

struct LoRaProfile {
    uint8_t     sf;         // spreading factor
    float       bw_khz;     // bandwidth (RadioLib units: kHz)
    uint8_t     cr;         // coding rate denominator (4/cr)
    float       snr_floor;  // demod limit, dB — grants keep headroom above this
    uint16_t    raw_bps;    // raw bytes/sec (bitrate formula / 8) for goodput math
    const char* name;
};

// idx:            0 = far / robust, climbing toward near / fast. Append only.
static const LoRaProfile LORA_PROFILES[] = {
    /* 0 far  */ { 10, 125.0f, 5, -15.0f,  122, "sf10/bw125" },
    /* 1 base */ {  8, 125.0f, 5, -10.0f,  488, "sf8/bw125"  },
    /* 2 near */ {  7, 250.0f, 5,  -4.5f, 1367, "sf7/bw250"  },
};
#define LORA_PROFILE_COUNT ((uint8_t)(sizeof(LORA_PROFILES) / sizeof(LORA_PROFILES[0])))

// The rendezvous profile. Overridable per build (-DLORA_PROFILE_BASE=0 for a far-field
// deployment where sf8 announces wouldn't be heard at all).
#ifndef LORA_PROFILE_BASE
#define LORA_PROFILE_BASE 1
#endif
