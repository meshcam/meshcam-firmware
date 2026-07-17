#pragma once

#include <microReticulum/Interface.h>
#include <microReticulum/Bytes.h>
#include <microReticulum/Type.h>

#ifdef ARDUINO
#include <SPI.h>
#include <RadioLib.h>
#endif

#include <stdint.h>

#include "lora_profiles.h"

class LoRaInterface : public RNS::InterfaceImpl {
public:
	// Signal quality of the most recently received frame (NAN until first RX).
	// Static so app code can read it without holding the interface wrapper.
	static float last_rssi;
	static float last_snr;
	// Raw bytes of the most recently received frame (L2 header stripped), for
	// packet-inspection telemetry. Written before on_incoming fires.
	static uint8_t last_frame[512];
	static size_t  last_frame_len;
	// millis() of the most recent frame received (0 until first RX). App-level liveness
	// checks (e.g. the gateway's ADR quiet-leaf scan) must count EVERY frame — judging
	// liveness only from announces/chunk-completions starved during a long in-flight
	// resource and retuned the radio mid-transfer (observed 2026-07-03).
	static uint32_t last_rx_ms;
	// RNS wire-header peek of the most recent frame (bug 1, 2026-07-16). The origin
	// transmits hops=0; every transport rebroadcast increments it (and switches to
	// HEADER_2, which carries a transport-id address field). A transport relay can
	// deliver a leaf's announces across a radio-profile split, so liveness and ADR
	// must only trust DIRECT frames — counting relayed ones kept the gateway's
	// quiet-leaf scan disarmed for 21 h (07-15 outage). Set in on_incoming BEFORE
	// handle_incoming, so app callbacks fired during inbound dispatch (announce
	// handlers, resource callbacks) read the values of the frame that fired them.
	static uint8_t  last_hops;           // hops byte as transmitted (0 = direct)
	static bool     last_relayed;        // hops > 0 or HEADER_2 (in transport)
	static uint32_t last_direct_rx_ms;   // millis() of last DIRECT frame (0 = never)
	// Every ANNOUNCE-type frame demodulated, whether or not Transport's replay/dedup
	// guards let it reach an announce handler (bug 6 diagnostics: "demodulated every
	// 26 s but processed once per 112 min" was invisible without this split).
	static uint32_t announces_seen;
	// The most recently constructed instance, so app-level ADR code can retune without
	// threading the pointer through the RNS::Interface wrapper. One radio per node here.
	static LoRaInterface* active;


public:
	//z def get_address_for_if(name):
	//z def get_broadcast_for_if(name):

public:
	//p def __init__(self, owner, name, device=None, bindip=None, bindport=None, forwardip=None, forwardport=None):
	LoRaInterface(const char* name = "LoRaInterface");
	virtual ~LoRaInterface();

	virtual bool start();
	virtual void stop();
	virtual void loop();

	// ADR: retune to LORA_PROFILES[idx]. Before start() it just stages the params
	// (begin() then uses them); after start() it retunes the live radio (SX126x and
	// SX127x). BOTH ends of a link must switch or they're deaf;
	// the negotiation/fallback protocol lives in app code, not here.
	bool set_profile(uint8_t idx);
	uint8_t     profile() const      { return _profile; }
	const char* profile_name() const { return LORA_PROFILES[_profile].name; }
	uint16_t    profile_raw_bps() const { return LORA_PROFILES[_profile].raw_bps; }

	//virtual inline std::string toString() const { return "LoRaInterface[" + name() + "]"; }

private:
	virtual bool send_outgoing(const RNS::Bytes& data);
	void on_incoming(const RNS::Bytes& data);

public:
	// Split-packet protocol constants
	static constexpr uint8_t HEADER_SPLIT     = 0x08;  // bit 3: split-packet flag
	static constexpr uint8_t HEADER_SEQ_MASK  = 0x07;  // bits 2:0: sequence number
	static constexpr uint8_t SEQ_UNSET        = 0xFF;  // sentinel: no split in progress
	static constexpr int     LORA_MAX_PAYLOAD = 254;   // 255 - 1 header byte

private:
	//uint8_t buffer[Type::Reticulum::MTU] = {0};
	const uint8_t message_count = 0;
	RNS::Bytes buffer;

	uint8_t _rx_seq     = SEQ_UNSET;  // sequence of split RX in progress
	uint8_t _tx_seq_ctr = 0;          // rolling TX split sequence counter

	// Radio parameters (RadioLib units: MHz, kHz). SF/BW/CR come from the profile
	// table (lora_profiles.h) and are runtime-switchable via set_profile().
	const float frequency = 915.0;   // MHz
	float       bandwidth = LORA_PROFILES[LORA_PROFILE_BASE].bw_khz;
	int         spreading = LORA_PROFILES[LORA_PROFILE_BASE].sf;
	int         coding    = LORA_PROFILES[LORA_PROFILE_BASE].cr;
	uint8_t     _profile  = LORA_PROFILE_BASE;
#ifndef LORA_TX_DBM
#define LORA_TX_DBM 10               // dBm — LOW for desk-range bench (915 antenna is on
	                                 // as of 2026-07-02, ~3/4-wave telescopic; two radios
	                                 // meters apart at 17+ dBm can saturate the RX
	                                 // front-end). Field/range builds override via
	                                 // -DLORA_TX_DBM=17..22 (SX127x PA_BOOST caps at 20).
#endif
	const int   power     = LORA_TX_DBM;

#ifdef ARDUINO
	Module*        _module      = nullptr;
	PhysicalLayer* _radio       = nullptr;
	SX126x*        _sx126x      = nullptr;  // set on SX1262 boards; enables live retune
	SX1278*        _sx127x      = nullptr;  // set on SX1276 boards (T-Beam); same purpose.
	                                        // Typed SX1278: RadioLib declares the SF/BW/CR
	                                        // setters there, not on the SX127x base.
	int            _pa_mode_pin = -1;    // V4 FEM PA mode pin; -1 = not present
#endif

};
