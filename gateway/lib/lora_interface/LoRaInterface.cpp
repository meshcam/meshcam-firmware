#include "LoRaInterface.h"

#include <microReticulum/Log.h>
#include <microReticulum/Utilities/OS.h>

#include <memory>

// ---------------------------------------------------------------------------
// Board-specific pin definitions
// ---------------------------------------------------------------------------

#if defined(BOARD_TBEAM) || defined(BOARD_LORA32_V21)
// LILYGO T-Beam V1.X / LoRa32 V2.1 — SX1276
// RadioLib Module(cs, irq=DIO0, rst, gpio=DIO1)
#define RADIO_SCLK_PIN               5
#define RADIO_MISO_PIN              19
#define RADIO_MOSI_PIN              27
#define RADIO_CS_PIN                18
#define RADIO_DIO0_PIN              26   // IRQ (RxDone/TxDone)
#define RADIO_RST_PIN               23
#define RADIO_DIO1_PIN              33   // gpio (optional, passed to Module)

#elif defined(BOARD_RAK4631)
// RAK4631 (WisCore RAK4630) — SX1262
// RadioLib Module(cs, irq=DIO1, rst, busy)
#define RADIO_SCLK_PIN              43
#define RADIO_MISO_PIN              45
#define RADIO_MOSI_PIN              44
#define RADIO_CS_PIN                42
#define RADIO_DIO1_PIN              47   // IRQ (all SX1262 IRQs route to DIO1)
#define RADIO_RST_PIN               38
#define RADIO_BUSY_PIN              46

#elif defined(BOARD_HELTEC_V3)
// Heltec WiFi LoRa 32 V3 — ESP32-S3 + SX1262
// RadioLib Module(cs, irq=DIO1, rst, busy)
// Note: Heltec BSP names this pin "DIO0" but it is physically DIO1 on the SX1262
#define RADIO_SCLK_PIN               9
#define RADIO_MISO_PIN              11
#define RADIO_MOSI_PIN              10
#define RADIO_CS_PIN                 8
#define RADIO_DIO1_PIN              14   // IRQ
#define RADIO_RST_PIN               12
#define RADIO_BUSY_PIN              13

#elif defined(BOARD_HELTEC_V4)
// Heltec WiFi LoRa 32 V4 — ESP32-S3R2 + SX1262 + external FEM (GC1109 / KCT8103L)
// LoRa SPI/control pins are identical to V3; FEM adds 3 extra GPIOs
#define RADIO_SCLK_PIN               9
#define RADIO_MISO_PIN              11
#define RADIO_MOSI_PIN              10
#define RADIO_CS_PIN                 8
#define RADIO_DIO1_PIN              14   // IRQ
#define RADIO_RST_PIN               12
#define RADIO_BUSY_PIN              13
// FEM (GC1109) control — required for antenna path; verified against RNode firmware
#define RADIO_VFEM_EN               7    // LORA_PA_PWR_EN: FEM power rail (active HIGH)
#define RADIO_FEM_CE                2    // LORA_PA_CSD:    FEM chip enable  (active HIGH)
#define RADIO_PA_MODE              46    // LORA_PA_CPS:    PA mode HIGH=TX, LOW=RX

#elif defined(GATE_B_WIRED_SX1262)
// Gate B — a bare SX1262 (Waveshare Core1262) wired to an ESP32-S3 camera board.
// RADIO_*_PIN come from platformio.ini build_flags (-DRADIO_SCLK_PIN=... per env), so
// this one vendored file serves both Freenove and XIAO. They must match the radio pin
// map documented in src/gate_b_board.h.
#if !defined(RADIO_SCLK_PIN) || !defined(RADIO_CS_PIN) || !defined(RADIO_BUSY_PIN)
#error "GATE_B_WIRED_SX1262: set RADIO_{SCLK,MISO,MOSI,CS,RST,BUSY,DIO1}_PIN in build_flags"
#endif

#endif

using namespace RNS;

float LoRaInterface::last_rssi = NAN;
float LoRaInterface::last_snr  = NAN;
uint8_t LoRaInterface::last_frame[512];
size_t  LoRaInterface::last_frame_len = 0;
uint32_t LoRaInterface::last_rx_ms = 0;
uint8_t  LoRaInterface::last_hops = 0;
bool     LoRaInterface::last_relayed = false;
uint32_t LoRaInterface::last_direct_rx_ms = 0;
uint32_t LoRaInterface::announces_seen = 0;
LoRaInterface* LoRaInterface::active = nullptr;

static inline bool    isSplitPacket(uint8_t h)  { return (h & LoRaInterface::HEADER_SPLIT)   != 0; }
static inline uint8_t packetSequence(uint8_t h) { return  h & LoRaInterface::HEADER_SEQ_MASK;      }

/*
@staticmethod
def get_address_for_if(name):
	import RNS.vendor.ifaddr.niwrapper as netinfo
	ifaddr = netinfo.ifaddresses(name)
	return ifaddr[netinfo.AF_INET][0]["addr"]

@staticmethod
def get_broadcast_for_if(name):
	import RNS.vendor.ifaddr.niwrapper as netinfo
	ifaddr = netinfo.ifaddresses(name)
	return ifaddr[netinfo.AF_INET][0]["broadcast"]
*/

LoRaInterface::LoRaInterface(const char* name /*= "LoRaInterface"*/) : RNS::InterfaceImpl(name) {

	_IN = true;
	_OUT = true;
	//p self.bitrate = self.r_sf * ( (4.0/self.r_cr) / (math.pow(2,self.r_sf)/(self.r_bandwidth/1000)) ) * 1000
	// bandwidth is in kHz here (RadioLib units), formula unchanged
	_bitrate = (double)spreading * ( (4.0/coding) / (pow(2, spreading)/bandwidth) ) * 1000.0;
	// CBA alternate bitrate calculation from RNode
	//_bitrate = (uint32_t)(spreading * ( (4.0/(float)coding) / ((float)(pow(2, spreading))/((float)bandwidth/1000.0)) ) * 1000.0);
	_HW_MTU = 508;
	active = this;

}

// ADR retune. Pre-start: stage the params for begin(). Live: standby -> retune ->
// back to RX. SX126x and SX127x both support live retune (SX127x path added for the
// T-Beam surveyor, 2026-07-10); a
// failure leaves the radio re-armed on whatever the chip accepted, so callers treat
// false as "profile unavailable" and stay on the negotiated fallback path.
bool LoRaInterface::set_profile(uint8_t idx) {
	if (idx >= LORA_PROFILE_COUNT) return false;
	const LoRaProfile& p = LORA_PROFILES[idx];
	_profile  = idx;
	spreading = p.sf;
	bandwidth = p.bw_khz;
	coding    = p.cr;
	_bitrate  = (double)spreading * ( (4.0/coding) / (pow(2, spreading)/bandwidth) ) * 1000.0;
#ifdef ARDUINO
#if defined(LORA_BACKEND_RAK3172)
	if (_online && _rak) {
		if (!_rak->configure(frequency, bandwidth, spreading, coding, power)) {
			ERRORF("LoRaInterface: retune to %s failed", p.name);
			return false;
		}
		INFOF("LoRaInterface: retuned to profile %u (%s, ~%u B/s raw)",
		      (unsigned)idx, p.name, (unsigned)p.raw_bps);
	}
#else
	if (_online) {
		int s1, s2, s3;
		if (_sx126x) {
			_sx126x->standby();
			s1 = _sx126x->setSpreadingFactor(p.sf);
			s2 = _sx126x->setBandwidth(p.bw_khz);
			s3 = _sx126x->setCodingRate(p.cr);
		} else if (_sx127x) {
			_sx127x->standby();
			s1 = _sx127x->setSpreadingFactor(p.sf);
			s2 = _sx127x->setBandwidth(p.bw_khz);
			s3 = _sx127x->setCodingRate(p.cr);
		} else {
			ERROR("LoRaInterface: live profile retune unsupported on this chip");
			return false;
		}
		_radio->startReceive();
		if (s1 != RADIOLIB_ERR_NONE || s2 != RADIOLIB_ERR_NONE || s3 != RADIOLIB_ERR_NONE) {
			ERRORF("LoRaInterface: retune to %s failed (%d/%d/%d)", p.name, s1, s2, s3);
			return false;
		}
		INFOF("LoRaInterface: retuned to profile %u (%s, ~%u B/s raw)",
		      (unsigned)idx, p.name, (unsigned)p.raw_bps);
	}
#endif
#endif
	return true;
}

// Runtime TX power. Same live-retune shape as set_profile(): standby, poke the
// chip, re-arm RX. Pre-start it only stages `power` for begin(). On a rejected
// value the radio is re-armed on its previous power and we return false.
bool LoRaInterface::set_tx_dbm(int dbm) {
#ifdef ARDUINO
#if defined(LORA_BACKEND_RAK3172)
	if (_online && _rak) {
		if (!_rak->set_tx_dbm(dbm)) {
			ERRORF("LoRaInterface: AT+PTP=%d rejected", dbm);
			return false;
		}
		// RUI3 accepts 5..22 dBm only, so the modem may have clamped the ADR's request.
		// Report what the PA is actually running at, not what was asked for.
		power = _rak->tx_dbm();
		INFOF("LoRaInterface: TX power now %d dBm", power);
		return true;
	}
#else
	if (_online) {
		int st;
		if (_sx126x) {
			_sx126x->standby();
			st = _sx126x->setOutputPower(dbm);
		} else if (_sx127x) {
			_sx127x->standby();
			st = _sx127x->setOutputPower(dbm);
		} else {
			ERROR("LoRaInterface: runtime TX power unsupported on this chip");
			return false;
		}
		_radio->startReceive();
		if (st != RADIOLIB_ERR_NONE) {
			ERRORF("LoRaInterface: setOutputPower(%d) rejected (%d)", dbm, st);
			return false;
		}
		INFOF("LoRaInterface: TX power now %d dBm", dbm);
	}
#endif
#endif
	power = dbm;
	return true;
}

// --- Radio test mode (gateway cert bench) -----------------------------------------------
// Repurpose the live radio for a continuous emission the way a §15.247 DTS lab drives it.
// SX126x only (the die we certify); a no-op returning false on SX127x / radio-absent
// boards. These do NOT touch WiFi/ethernet/OTA, so the node stays reachable and
// OTA-recoverable the whole time — radio_test_stop() (or the caller's timeout) restores
// normal RX on the current profile. DIO2-as-RF-switch (set in start()) keys TXEN when the
// chip enters TX, so the emission reaches the antenna. OCP is raised to 140 mA so +22 dBm
// isn't silently clamped (RadioLib's 60 mA default); left raised on restore (an upper
// limit only, harmless for normal ops).
#if defined(ARDUINO) && !defined(LORA_BACKEND_RAK3172)
bool LoRaInterface::radio_test_cw(float freq_mhz, int8_t dbm) {
	if (!_sx126x) return false;
	_sx126x->standby();
	_sx126x->setFrequency(freq_mhz);
	_sx126x->setCurrentLimit(140.0);
	_sx126x->setOutputPower(dbm);
	int st = _sx126x->transmitDirect();   // SetTxContinuousWave (unmodulated carrier)
	INFOF("LoRaInterface: TEST CW %.3f MHz @ %d dBm (st %d)", freq_mhz, (int)dbm, st);
	return st == RADIOLIB_ERR_NONE;
}

bool LoRaInterface::radio_test_mod(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr, int8_t dbm) {
	if (!_sx126x) return false;
	_sx126x->standby();
	_sx126x->setFrequency(freq_mhz);
	_sx126x->setBandwidth(bw_khz);
	_sx126x->setSpreadingFactor(sf);
	_sx126x->setCodingRate(cr);
	_sx126x->setCurrentLimit(140.0);
	_sx126x->setOutputPower(dbm);
	// No public RadioLib wrapper for infinite preamble; issue the raw command on the
	// Module we own (the same path transmitDirect() takes internally for CW). Continuous
	// LoRa chirp = the DTS 6 dB occupied bandwidth the lab measures.
	int st = _module->SPIwriteStream(RADIOLIB_SX126X_CMD_SET_TX_INFINITE_PREAMBLE, NULL, 0);
	INFOF("LoRaInterface: TEST MOD %.3f MHz BW %.0f SF%u @ %d dBm (st %d)",
	      freq_mhz, bw_khz, (unsigned)sf, (int)dbm, st);
	return st == RADIOLIB_ERR_NONE;
}

void LoRaInterface::radio_test_stop() {
	if (!_sx126x) return;
	_sx126x->standby();
	_sx126x->setFrequency(frequency);    // back to the fixed operating channel (915)
	_sx126x->setOutputPower(power);      // back to the normal runtime TX power
	set_profile(_profile);               // reconfigure SF/BW/CR for the current profile + startReceive()
	INFO("LoRaInterface: TEST stopped -> normal RX restored");
}
#elif defined(ARDUINO)
// RAK3172: stock RUI3 exposes the whole §15.247 test suite over AT (respin spec §0a
// closed §12 q2 with it), so CW is a one-liner. AT+CW takes a DURATION, and this API has
// no such argument, so it is armed for an hour and ended by radio_test_stop() / AT+TOFF —
// long enough for a conducted-power measurement, and self-limiting if the bench walks away.
bool LoRaInterface::radio_test_cw(float freq_mhz, int8_t dbm) {
	if (!_rak) return false;
	INFOF("LoRaInterface: TEST CW %.3f MHz @ %d dBm (AT+CW)", freq_mhz, (int)dbm);
	return _rak->cw_start(freq_mhz, dbm, 3600000UL);
}

// No RUI3 equivalent. The SX126x path drives SetTxInfinitePreamble, a register-level
// command unreachable through an AT modem. RUI3's own PER suite (AT+TCONF / AT+TTX) is
// the substitute, and it is a different measurement — do not fake it here.
bool LoRaInterface::radio_test_mod(float, float, uint8_t, uint8_t, int8_t) {
	ERROR("LoRaInterface: modulated-continuous test is SX126x-only; use RUI3 AT+TCONF/AT+TTX");
	return false;
}

void LoRaInterface::radio_test_stop() {
	if (!_rak) return;
	_rak->test_stop();
	INFO("LoRaInterface: TEST stopped -> normal RX restored");
}
#else
bool LoRaInterface::radio_test_cw(float, int8_t) { return false; }
bool LoRaInterface::radio_test_mod(float, float, uint8_t, uint8_t, int8_t) { return false; }
void LoRaInterface::radio_test_stop() {}
#endif

/*virtual*/ LoRaInterface::~LoRaInterface() {
	if (active == this) active = nullptr;
	stop();
}

bool LoRaInterface::start() {
	_online = false;
	INFO("LoRa initializing...");

#ifdef ARDUINO

#if defined(LORA_BACKEND_RAK3172)
	// Leaf/gateway PCB v1.0 — RAK3172-T over UART, AT commands. No SPI, no IRQ line, and
	// no chip handle: everything the SX1262 branches do with RadioLib calls, this backend
	// does with AT commands inside Rak3172Modem.
	if (!_rak) _rak = new Rak3172Modem();
	if (!_rak->begin(frequency, bandwidth, spreading, coding, power)) {
		ERROR("RAK3172 init failed. Check the LORA rail, UART pins, and NRST.");
		return false;
	}
	// RUI3 clamps TX power to 5..22 dBm; keep `power` honest so the announce tail's
	// "tx=<dbm>" reports what the PA actually ran at.
	power = _rak->tx_dbm();
	INFOF("RAK3172 P2P up: %.3f MHz %s @ %d dBm", frequency, LORA_PROFILES[_profile].name, power);

#elif defined(BOARD_TBEAM) || defined(BOARD_LORA32_V21)
	// ESP32: T-Beam and LoRa32 use non-default SPI pins — must specify explicitly
	SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
	_module = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN, SPI);
	SX1276* chip = new SX1276(_module);
	_radio = chip;
	_sx127x = chip;
	// begin(freq MHz, bw kHz, sf, cr, syncWord, power dBm, preamble symbols, LNA gain 0=AGC)
	int state = chip->begin(frequency, bandwidth, spreading, coding,
	                        RADIOLIB_SX127X_SYNC_WORD, power, 20, 0);
	// SX127x hardware default leaves RxPayloadCrcOn=0 and RadioLib's
	// SX127x::begin() does not touch it. Upstream RNode firmware enables
	// CRC unconditionally (sx127x::enableCrc, RNode_Firmware.ino:531), so
	// real RNodes silently drop frames without a CRC. Enable it here to
	// interoperate. SX126x branches below inherit CRC-on from
	// SX126x::begin() (setCRC(2)), so no explicit call is needed there.
	if (state == RADIOLIB_ERR_NONE) state = chip->setCRC(true);

#elif defined(BOARD_RAK4631)
	// nRF52: SPI pins must be configured before SPI.begin()
	SPI.setPins(RADIO_MISO_PIN, RADIO_SCLK_PIN, RADIO_MOSI_PIN);
	SPI.begin();
	// SX1262 Module args: cs, irq=DIO1, rst, busy
	_module = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, SPI);
	SX1262* chip = new SX1262(_module);
	_radio = chip;
	_sx126x = chip;
	// DIO2 drives the antenna T/R switch on the RAK4631 SX1262 module
	chip->setDio2AsRfSwitch(true);
	// begin(freq MHz, bw kHz, sf, cr, syncWord, power dBm, preamble symbols,
	//       tcxoVoltage V, useRegulatorLDO)
	// RAK4631 SX1262 module uses a 1.6V TCXO on DIO3; DCDC regulator (LDO=false)
	int state = chip->begin(frequency, bandwidth, spreading, coding,
	                        RADIOLIB_SX126X_SYNC_WORD_PRIVATE, power, 20, 1.6, false);

#elif defined(BOARD_HELTEC_V3)
	// Heltec WiFi LoRa 32 V3 — ESP32-S3 + SX1262, 1.8V TCXO
	SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
	_module = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, SPI);
	SX1262* chip = new SX1262(_module);
	_radio = chip;
	_sx126x = chip;
	chip->setDio2AsRfSwitch(true);
	int state = chip->begin(frequency, bandwidth, spreading, coding,
	                        RADIOLIB_SX126X_SYNC_WORD_PRIVATE, power, 20, 1.8, false);

#elif defined(BOARD_HELTEC_V4)
	// Heltec WiFi LoRa 32 V4 — ESP32-S3R2 + SX1262 + external FEM, 1.8V TCXO
	SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
	// Power and enable the FEM — required for antenna path to function at all
	pinMode(RADIO_VFEM_EN, OUTPUT);
	pinMode(RADIO_FEM_CE, OUTPUT);
	pinMode(RADIO_PA_MODE, OUTPUT);
	digitalWrite(RADIO_VFEM_EN, HIGH);
	digitalWrite(RADIO_FEM_CE, HIGH);
	digitalWrite(RADIO_PA_MODE, LOW);   // start in RX mode
	_pa_mode_pin = RADIO_PA_MODE;
	_module = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, SPI);
	SX1262* chip = new SX1262(_module);
	_radio = chip;
	_sx126x = chip;
	chip->setDio2AsRfSwitch(true);
	int state = chip->begin(frequency, bandwidth, spreading, coding,
	                        RADIOLIB_SX126X_SYNC_WORD_PRIVATE, power, 20, 1.8, false);

#elif defined(GATE_B_WIRED_SX1262)
	// Gate B — bare SX1262 (Waveshare Core1262) on a cam board's free SPI pins.
	// Mirrors the Heltec V3 SX1262 path: same chip, DIO2 = RF switch, 1.8V TCXO.
	SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
	_module = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, SPI);
	SX1262* chip = new SX1262(_module);
	_radio = chip;
	_sx126x = chip;
	chip->setDio2AsRfSwitch(true);
	// Core1262 carries a 1.8V TCXO (DIO3). If your module is XTAL-based, pass 0.0 instead.
	int state = chip->begin(frequency, bandwidth, spreading, coding,
	                        RADIOLIB_SX126X_SYNC_WORD_PRIVATE, power, 20, 1.8, false);

#else
	#error "Unsupported board: define LORA_BACKEND_RAK3172, BOARD_TBEAM, BOARD_LORA32_V21, BOARD_RAK4631, BOARD_HELTEC_V3, BOARD_HELTEC_V4, or GATE_B_WIRED_SX1262"
	int state = RADIOLIB_ERR_UNKNOWN;
#endif

#if !defined(LORA_BACKEND_RAK3172)
	if (state != RADIOLIB_ERR_NONE) {
		ERRORF("LoRa init failed, code %d. Check wiring/board define.", state);
		return false;
	}

	// Enter continuous receive mode
	_radio->startReceive();

	INFO("LoRa init succeeded.");
#endif
	TRACEF("LoRa bandwidth is %.2f Kbps", Utilities::OS::round(_bitrate/1000.0, 2));
#endif

	_online = true;
	return true;
}

void LoRaInterface::stop() {

#ifdef ARDUINO
#if defined(LORA_BACKEND_RAK3172)
	// Stops RX and leaves NRST low with the UART pins floating, so the caller may cut
	// LORA_PWR_EN straight after (respin spec §11 rail-down rule).
	if (_rak) _rak->end();
#else
	if (_radio) {
		_radio->standby();
	}
#endif
#endif

	_online = false;
}

void LoRaInterface::loop() {

	if (_online) {
#ifdef ARDUINO
#if defined(LORA_BACKEND_RAK3172)
		// No IRQ line exists on this backend: RUI3 delivers frames as unsolicited AT
		// events, so a frame is only heard while this loop is being called. The modem
		// stays armed across frames (AT+PRECV=65534 is truly continuous) — no
		// per-event re-arm happens or is needed.
		uint8_t rxBuf[Rak3172Modem::MAX_FRAME];
		float rssi = NAN, snr = NAN;
		int len = _rak ? _rak->poll(rxBuf, sizeof(rxBuf), rssi, snr) : -1;
		if (len > 0) handle_rx_frame(rxBuf, len, rssi, snr);
#else
		// checkIrq() polls the hardware IRQ register — no ISR required
		if (_radio->checkIrq(RADIOLIB_IRQ_RX_DONE)) {
			int len = _radio->getPacketLength();

			uint8_t rxBuf[255];
			int state = _radio->readData(rxBuf, len);
			// Re-arm receive BEFORE processing the frame: inbound handling (which can
			// itself transmit a response and take tens of ms) used to leave the radio
			// deaf until the very end of this block — and a windowed Resource sender
			// streams parts back-to-back, so every deaf ms here was a lost part and a
			// retry round-trip later (measured 2026-07-03: ~60 s/chunk of overhead).
			_radio->startReceive();

			if (state == RADIOLIB_ERR_NONE) {
				handle_rx_frame(rxBuf, len, _radio->getRSSI(), _radio->getSNR());
			} else {
				DEBUGF("LoRaInterface: readData failed, code %d", state);
			}
			// NOTE: no trailing startReceive() here — RX was re-armed right after
			// readData(), and re-arming again now would abort a frame that started
			// arriving while we processed this one.
		}
#endif
#endif
	}
}

// One received PHY frame, whichever backend demodulated it: strip the split-packet
// header, reassemble if needed, and hand the result to Transport. Shared on purpose —
// the framing is the over-the-air contract with every other node on the mesh, and a
// second copy of it would be a second chance to get it wrong.
void LoRaInterface::handle_rx_frame(const uint8_t* rxBuf, int len, float rssi, float snr) {
#ifdef ARDUINO
	if (len <= 1) return;   // header-only frame carries nothing

	last_rssi  = rssi;
	last_snr   = snr;
	last_rx_ms = millis();

	uint8_t hdr = rxBuf[0];
	uint8_t seq = packetSequence(hdr);
	const uint32_t t_proc = millis();

	// Expire a partial whose second half never came, so a lost s2 cannot strand the
	// slot until the next split happens to reuse it.
	if (_rx_seq != SEQ_UNSET && (uint32_t)(millis() - _rx_seq_ms) > SPLIT_REASSEMBLY_MS) {
		split_buffer.clear();
		_rx_seq = SEQ_UNSET;
	}

	if (isSplitPacket(hdr)) {
		if (_rx_seq == SEQ_UNSET || _rx_seq != seq) {
			// First part of a split (or restart after a lost first part)
			_rx_seq = seq;
			_rx_seq_ms = millis();
			split_buffer.clear();
			split_buffer.append(rxBuf + 1, len - 1);
		} else {
			// Second part — sequence matches; assemble and deliver
			split_buffer.append(rxBuf + 1, len - 1);
			_rx_seq = SEQ_UNSET;
			on_incoming(split_buffer);
			split_buffer.clear();
		}
	} else {
		// Non-split: deliver immediately. It MUST NOT touch a reassembly in progress
		// — the gateway's announce and time_sync are both non-split and
		// arrive constantly, and clearing here destroyed every split packet on a busy
		// link. `buffer` is now the immediate-delivery buffer only.
		buffer.clear();
		buffer.append(rxBuf + 1, len - 1);
		on_incoming(buffer);
	}
	// One compact timeline line per frame (replaces the old RSSI/Snr pair):
	// arrival time, frame length, signal, and how long inbound processing
	// held the CPU — the raw material for transfer-turnaround analysis.
	Serial.printf("[rf] rx t=%lu len=%d rssi=%.0f snr=%.1f proc=%lu\n",
	              (unsigned long)last_rx_ms, len, last_rssi, last_snr,
	              (unsigned long)(millis() - t_proc));
#else
	(void)rxBuf; (void)len; (void)rssi; (void)snr;
#endif
}

// One PHY frame out. `tag` labels the split parts in the [rf] tx timeline ("", " s1",
// " s2") and in the error message.
bool LoRaInterface::send_frame(const uint8_t* buf, size_t len, const char* tag) {
#ifdef ARDUINO
#if defined(LORA_BACKEND_RAK3172)
	// The modem answers +EVT:TXP2P DONE only after the air time is over, so the wait has
	// to cover it: 3x the profile's own raw byte rate plus a fixed floor for the AT
	// round trip. A max-length frame on the slowest rung (sf12/bw500, 146 B/s) is ~1.7 s
	// of air, so this lands near 6.7 s there and ~2 s on the fast rungs.
	const uint16_t raw_bps = LORA_PROFILES[_profile].raw_bps;
	uint32_t timeout = 1500 + (uint32_t)((uint32_t)len * 3000UL / (raw_bps ? raw_bps : 100));
	uint32_t t_tx = millis();
	bool ok = _rak && _rak->transmit(buf, len, timeout);
	Serial.printf("[rf] tx t=%lu len=%u air=%lu%s\n", (unsigned long)t_tx,
	              (unsigned)len, (unsigned long)(millis() - t_tx), tag);
	if (!ok) ERRORF("LoRaInterface: transmit%s failed (no TXP2P DONE)", tag);
	return ok;
#else
	// V4: switch FEM to TX mode before transmitting
	if (_pa_mode_pin >= 0) { digitalWrite(_pa_mode_pin, HIGH); }
	uint32_t t_tx = millis();
	int state = _radio->transmit(const_cast<uint8_t*>(buf), len);
	// V4: return FEM to RX mode
	if (_pa_mode_pin >= 0) { digitalWrite(_pa_mode_pin, LOW); }
	Serial.printf("[rf] tx t=%lu len=%u air=%lu%s\n", (unsigned long)t_tx,
	              (unsigned)len, (unsigned long)(millis() - t_tx), tag);
	if (state != RADIOLIB_ERR_NONE) {
		ERRORF("LoRaInterface: transmit%s failed, code %d", tag, state);
		return false;
	}
	return true;
#endif
#else
	(void)buf; (void)len; (void)tag;
	return true;
#endif
}

/*virtual*/ bool LoRaInterface::send_outgoing(const Bytes& data) {
	DEBUGF("%s.on_outgoing: data: %s", toString().c_str(), data.toHex().c_str());
	bool success = true;
	try {
		if (_online) {
			TRACEF("LoRaInterface: sending %lu bytes...", data.size());
#ifdef ARDUINO
			uint8_t txBuf[255];
			uint8_t rand_nibble = (uint8_t)(Cryptography::randomnum(256)) & 0xF0;

			if ((int)data.size() <= LORA_MAX_PAYLOAD) {
				// Single-frame send
				txBuf[0] = rand_nibble;
				memcpy(txBuf + 1, data.data(), data.size());
				if (!send_frame(txBuf, 1 + data.size(), "")) success = false;
			} else {
				// Split send — two frames with matching sequence number
				uint8_t seq       = (_tx_seq_ctr++) & HEADER_SEQ_MASK;
				uint8_t split_hdr = rand_nibble | HEADER_SPLIT | seq;

				// Frame 1: first LORA_MAX_PAYLOAD bytes
				txBuf[0] = split_hdr;
				memcpy(txBuf + 1, data.data(), LORA_MAX_PAYLOAD);
				if (!send_frame(txBuf, 1 + LORA_MAX_PAYLOAD, " s1")) success = false;

				// Frame 2: remaining bytes
				size_t remainder = data.size() - LORA_MAX_PAYLOAD;
				txBuf[0] = split_hdr;
				memcpy(txBuf + 1, data.data() + LORA_MAX_PAYLOAD, remainder);
				if (!send_frame(txBuf, 1 + remainder, " s2")) success = false;
			}

#if !defined(LORA_BACKEND_RAK3172)
			_radio->startReceive();
#endif
			// (the RAK backend re-arms RX inside transmit(), because TX and RX are
			// mutually exclusive there and it had to leave RX to send at all)
#endif
			TRACE("LoRaInterface: sent bytes");
		}

		// Perform post-send housekeeping
		InterfaceImpl::handle_outgoing(data);
	}
	catch (const std::exception& e) {
		ERRORF("Could not transmit on %s. The contained exception was: %s", toString().c_str(), e.what());
		success = false;
	}
	return success;
}

/*virtual*/ void LoRaInterface::on_incoming(const Bytes& data) {
	DEBUGF("%s.on_incoming: data: %s", toString().c_str(), data.toHex().c_str());
	last_frame_len = std::min(data.size(), sizeof(last_frame));
	memcpy(last_frame, data.data(), last_frame_len);
	// Peek the RNS wire header (bug 1): byte 0 bit 6 = header type (1 = HEADER_2,
	// in transport), bits 1:0 = packet type (0b01 = ANNOUNCE); byte 1 = hops as
	// transmitted. See the header-file comment for why app code needs this split.
	if (data.size() >= 2) {
		const uint8_t flags = data.data()[0];
		last_hops    = data.data()[1];
		last_relayed = (flags & 0x40) != 0 || last_hops > 0;
		if (!last_relayed) last_direct_rx_ms = millis();
		if ((flags & 0x03) == 0x01) announces_seen++;
	}
	// Pass received data on to transport
	InterfaceImpl::handle_incoming(data);
}
