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
	// (begin() then uses them); after start() it retunes the live radio (SX126x only —
	// SX127x boards return false). BOTH ends of a link must switch or they're deaf;
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
	const int   power     = 10;      // dBm — LOW for desk-range bench (915 antenna is on
	                                 // as of 2026-07-02, ~3/4-wave telescopic; two radios
	                                 // meters apart at 17+ dBm can saturate the RX
	                                 // front-end). Restore 17-22 for field/range tests.

#ifdef ARDUINO
	Module*        _module      = nullptr;
	PhysicalLayer* _radio       = nullptr;
	SX126x*        _sx126x      = nullptr;  // set on SX1262 boards; enables live retune
	int            _pa_mode_pin = -1;    // V4 FEM PA mode pin; -1 = not present
#endif

};
