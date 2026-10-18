#pragma once
/*
 * RAK3172-T LoRa modem, driven over UART with stock-RUI3 AT commands.
 *
 * This is the PHY transport for leaf/gateway PCB v1.0, which carries a soldered RAK3172-T
 * module instead of a bare SX1262 on SPI. Everything above the PHY is unchanged:
 * LoRaInterface owns the RNS framing and picks this backend with -DLORA_BACKEND_RAK3172.
 *
 * Why an AT modem at all: the module holds the FCC §15.247 grant the product line rides
 * (24.1 dBm DTS, SF8/BW500, 903-914.2 MHz), and stock RUI3 refuses >22 dBm itself — the
 * envelope cannot be exceeded by a firmware bug here. See homelab
 * docs/trailcam/fcc-certification.md Findings 3c/3d.
 *
 * Everything below was MEASURED on hardware 2026-08-07 against a RadioLib SX1262 peer
 * (homelab docs/trailcam/leaf-v1.0-respin-spec.md §0a), not read off a datasheet:
 *
 *  - P2P mode is AT+NWM=0. Changing NWM REBOOTS the module, so begin() reads it first
 *    and only sets it when it is wrong.
 *  - Config is one compound command, AT+P2P=<freq>:<sf>:<bw>:<cr>:<preamble>:<power>.
 *    bw is a CODE (0=125, 1=250, 2=500 kHz); cr is a CODE (0=4/5 .. 3=4/8); power is
 *    dBm and RUI3 rejects anything outside 5..22 with AT_PARAM_ERROR.
 *  - Preamble MUST be set. The factory default is 200 symbols; RadioLib peers in this
 *    codebase run 20, and a preamble mismatch looks exactly like a dead radio.
 *  - Sync word matches RadioLib's PRIVATE (0x1424) out of the box, IQ is normal, the
 *    header is explicit, and RUI3's CRC is one a CRC-on RadioLib receiver accepts. None
 *    of those need a command.
 *  - ⚠️ AT+PRECV=65535 IS ONE-SHOT BY DESIGN: it listens until the FIRST packet, then
 *    stops (measured 1/5 packets without a re-arm, 5/5 with one, 2026-08-07). The truly
 *    continuous value is AT+PRECV=65534 (until AT+PRECV=0), verified 2026-09-03: 12
 *    packets in 120 s from a single arm, back-to-back frames included. This driver arms
 *    with 65534 and never re-arms on RX — the one-shot-era re-arm opened a deaf gap
 *    that ate every split packet's second frame.
 *  - TX and RX are exclusive: PSEND while armed returns AT_BUSY_ERROR, so transmit()
 *    stops RX, sends, waits for +EVT:TXP2P DONE, and re-arms.
 *  - There is NO RX-done interrupt any more. RUI3 delivers frames as unsolicited AT
 *    events on the UART, so the ESP32 must be awake and polling to hear anything.
 *
 * Rail-down discipline (respin spec §11): IO14/IO1/IO42 must be low or Hi-Z BEFORE
 * LORA_PWR_EN cuts the module's rail, or the ESP32 back-powers it through its protection
 * diodes. quiesce_pins() does that, and leaf_rails.h calls it on every rail-down.
 */
#include <stdint.h>
#include <stddef.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

// --- board wiring (leaf PCB v1.0; respin spec §11 firmware contract) --------------------
#ifndef RAK_UART_TX_PIN
#define RAK_UART_TX_PIN 14      // ESP32 out -> RAK UART2 RX
#endif
#ifndef RAK_UART_RX_PIN
#define RAK_UART_RX_PIN 2       // ESP32 in  <- RAK UART2 TX
#endif
#ifndef RAK_NRST_PIN
#define RAK_NRST_PIN 1          // active low
#endif
#ifndef RAK_BOOT0_PIN
#define RAK_BOOT0_PIN 42        // held LOW: boot RUI3 from flash, never the STM32 bootloader
#endif
#ifndef RAK_UART_BAUD
#define RAK_UART_BAUD 115200    // stock RUI3, no configuration needed (§0a)
#endif
// A module that is armed but has heard nothing for this long gets a full PRECV=0 ->
// re-arm cycle rather than a bare re-arm. RUI3 can sit in a "RX_ON but deaf" state —
// P2P_RX_ON, parameters correct, packets demonstrably on the air, nothing delivered
// (observed on board 0014, 2026-08-31 07:20-07:22; a fresh PRECV cycle cleared it, and
// the v1.0 soak's RX listener has carried this workaround ever since). PRECV=0 is the
// load-bearing half: it exits RX so the follow-up arm actually lands.
#ifndef RAK_RX_RECYCLE_MS
#define RAK_RX_RECYCLE_MS 60000
#endif
// Symbols. Must match the RadioLib peers' preamble (LoRaInterface passes 20 to
// SX126x::begin), NOT RUI3's factory 200.
#ifndef RAK_PREAMBLE_SYMS
#define RAK_PREAMBLE_SYMS 20
#endif

// Bench gate: transmit one max-length frame at bring-up. See the block in begin().
#ifndef RAK_TX_SELFTEST
#define RAK_TX_SELFTEST 0
#endif

class Rak3172Modem {
public:
	// LoRa's PHY payload ceiling. AT+PSEND hex-encodes, so a full frame is 510 characters
	// of command line — inside RUI3's AT buffer, but it is the one thing here that the
	// 08-07 bench session did not exercise (it sent 5-27 byte beacons). The max-length
	// send is item 1 of the bench gate in the header of Rak3172Modem.cpp.
	static constexpr size_t MAX_FRAME = 255;

	// Power the module, prove it answers AT, put it in P2P mode, configure and arm RX.
	// Returns false if the module never answers — the caller keeps RNS up regardless.
	bool begin(double freq_mhz, float bw_khz, uint8_t sf, uint8_t cr_denom, int dbm,
	           uint16_t preamble = RAK_PREAMBLE_SYMS);
	// Stop RX and leave the pins safe for a rail cut.
	void end();
	bool online() const { return _online; }

	// Live retune. Stops RX, applies AT+P2P, re-arms if it was armed.
	bool configure(double freq_mhz, float bw_khz, uint8_t sf, uint8_t cr_denom, int dbm,
	               uint16_t preamble = RAK_PREAMBLE_SYMS);
	// TX power only (AT+PTP). Values outside RUI3's 5..22 dBm are CLAMPED, not rejected:
	// the ADR ladder is shared with SX126x nodes whose floor is -9 dBm.
	bool set_tx_dbm(int dbm);
	int  tx_dbm() const { return _dbm; }

	// Blocking send of one PHY frame. Waits for +EVT:TXP2P DONE, then re-arms RX.
	bool transmit(const uint8_t* data, size_t len, uint32_t timeout_ms);

	// Non-blocking. Drains the UART; returns the length of one received frame (and its
	// RSSI/SNR) or -1 when nothing is ready. RX stays armed across frames (65534 is
	// continuous); only the deaf-RX recycle and TX ever touch the arm state here.
	int poll(uint8_t* out, size_t cap, float& rssi, float& snr);

	// Frames dropped because both stash slots were full (diagnostic; see the .cpp).
	uint32_t rx_overruns() const { return _overruns; }

	// §15.247 conducted-power measurement (respin spec §0a, §12 q2). CW is a TEST mode,
	// not a compliant operating mode; test_stop() (AT+TOFF) ends it, and so does the
	// duration expiring. RUI3 exposes no modulated-continuous equivalent of the SX126x
	// infinite-preamble mode — use its own AT+TCONF/AT+TTX PER suite for that.
	bool cw_start(double freq_mhz, int dbm, uint32_t duration_ms);
	void test_stop();

	// Drive the control lines safe for a rail cut. Static and always callable, including
	// when the modem was never begun (a wake that never powered the radio still ends in
	// leaf_rails_all_off()).
	static void quiesce_pins();

private:
	bool     _online     = false;
	bool     _rx_armed   = false;
	uint32_t _last_arm_ms = 0;   // feeds the deaf-RX recycle above
	int      _dbm        = 14;

	// One line of AT output. A full-length +EVT:RXP2P carries 510 hex characters plus
	// the prefix and both signal fields.
	char   _line[560];
	size_t _line_len = 0;

	// Frames parsed out of the UART but not yet collected by poll(). Sized for a burst:
	// with continuous RX (65534) the module can deliver back-to-back frames the one-shot
	// arm never could — a split pair PLUS routine traffic (time_sync, an announce) can
	// all land between two poll() calls, and a dropped second half kills a reassembly.
	static constexpr int RX_SLOTS = 4;
	struct RxFrame {
		uint8_t  buf[MAX_FRAME];
		size_t   len = 0;
		float    rssi = 0, snr = 0;
	} _rx[RX_SLOTS];
	int      _rx_head = 0, _rx_count = 0;
	uint32_t _overruns = 0;

	// Line classification, in the order _classify() tests for them.
	enum Cls : uint8_t {
		CLS_NONE = 0, CLS_OTHER, CLS_OK, CLS_ERROR, CLS_RX, CLS_TXDONE, CLS_RXTIMEOUT
	};

	bool  _read_line(uint32_t timeout_ms);      // fills _line; false on timeout
	Cls   _classify();                          // side effect: stashes an RX frame
	Cls   _wait(uint32_t timeout_ms, uint8_t mask);
	bool  _cmd(const char* cmd, uint32_t timeout_ms = 1500);
	bool  _query(const char* cmd, char* out, size_t cap, uint32_t timeout_ms = 1500);
	bool  _arm_rx();
	bool  _stop_rx();
	bool  _force_rx_exit();   // unconditional AT+PRECV=0; see the .cpp
	void  _flush_input();
};
