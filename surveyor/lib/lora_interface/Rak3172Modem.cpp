#include "Rak3172Modem.h"

// The whole file is inert unless a build selects this backend. lora_interface/ is a
// PlatformIO library, so every env compiles every source in it — including the SX1262
// leaf, the gateway and the surveyor, which vendor this directory byte-identically.
#if defined(ARDUINO) && defined(LORA_BACKEND_RAK3172)

// UART2 of the RAK3172 lands on the ESP32's Serial1. The leaf console is native USB-CDC
// on v1.0 boards, so Serial1 is free; nothing else in the leaf uses it.
#ifndef RAK_SERIAL
#define RAK_SERIAL Serial1
#endif

// RUI3's two special AT+PRECV values do different things (verified on hardware
// 2026-09-03): 65535 listens UNTIL THE FIRST PACKET and then stops — one-shot
// by design, and what the 08-07 "one-shot even at the continuous value" measurement was
// actually seeing. 65534 receives continuously until AT+PRECV=0: 12 packets in 120 s
// from a single arm on the RAK3272S breakout, including three frames inside one second —
// the back-to-back case a split packet's second half needs. Arm with 65534, never
// re-arm on RX (the module answers AT_BUSY_ERROR if you try while it is armed).
#define RAK_PRECV_ARM "AT+PRECV=65534"

static inline uint8_t hexval(char c) {
	if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
	if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
	if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
	return 0xFF;
}

// --- UART plumbing ----------------------------------------------------------------------

void Rak3172Modem::_flush_input() {
	while (RAK_SERIAL.available()) RAK_SERIAL.read();
	_line_len = 0;
}

bool Rak3172Modem::_read_line(uint32_t timeout_ms) {
	const uint32_t t0 = millis();
	for (;;) {
		while (RAK_SERIAL.available()) {
			int c = RAK_SERIAL.read();
			if (c == '\n' || c == '\r') {
				if (_line_len == 0) continue;      // swallow the CR of a CRLF, and blanks
				_line[_line_len] = 0;
				_line_len = 0;
				return true;
			}
			if (_line_len < sizeof(_line) - 1) _line[_line_len++] = (char)c;
			// An over-length line is truncated rather than dropped: the prefix carries the
			// event type and the signal fields, which is what diagnosing it needs.
		}
		if (millis() - t0 >= timeout_ms) return false;
		delay(1);
	}
}

// Classify the line currently in _line. An unsolicited +EVT:RXP2P is STASHED here rather
// than returned, so a frame that lands while we are waiting on a command reply survives.
Rak3172Modem::Cls Rak3172Modem::_classify() {
	const char* s = _line;
	while (*s == ' ') s++;

	if (strncmp(s, "+EVT:RXP2P:", 11) == 0) {
		// +EVT:RXP2P:<rssi>:<snr>:<payload hex>
		const char* p = s + 11;
		char* endp = nullptr;
		long rssi = strtol(p, &endp, 10);
		if (!endp || *endp != ':') return CLS_OTHER;
		p = endp + 1;
		float snr = strtof(p, &endp);
		if (!endp || *endp != ':') return CLS_OTHER;
		p = endp + 1;

		size_t hexlen = strlen(p);
		size_t n = hexlen / 2;
		if (n == 0) return CLS_OTHER;
		if (n > MAX_FRAME) n = MAX_FRAME;

		if (_rx_count >= RX_SLOTS) {
			// Both slots full. Drop the NEW frame and say so: the stashed ones may be
			// halves of a split packet whose reassembly is already in progress.
			_overruns++;
			Serial.printf("[rak] rx overrun, frame dropped (total %lu)\n",
			              (unsigned long)_overruns);
			return CLS_RX;
		}
		RxFrame& f = _rx[(_rx_head + _rx_count) % RX_SLOTS];
		for (size_t i = 0; i < n; i++) {
			uint8_t hi = hexval(p[2 * i]), lo = hexval(p[2 * i + 1]);
			if (hi == 0xFF || lo == 0xFF) { n = i; break; }
			f.buf[i] = (uint8_t)((hi << 4) | lo);
		}
		f.len  = n;
		f.rssi = (float)rssi;
		f.snr  = snr;
		_rx_count++;
		return CLS_RX;
	}
	if (strncmp(s, "+EVT:RXP2P", 10) == 0) return CLS_RXTIMEOUT;   // "RECEIVE TIMEOUT"
	if (strncmp(s, "+EVT:TXP2P", 10) == 0) return CLS_TXDONE;
	if (strcmp(s, "OK") == 0) return CLS_OK;
	if (strncmp(s, "AT_", 3) == 0) {
		// AT_ERROR / AT_PARAM_ERROR / AT_BUSY_ERROR / AT_NO_NETWORK_JOINED ...
		Serial.printf("[rak] %s\n", s);
		return CLS_ERROR;
	}
	return CLS_OTHER;
}

// Read lines until one classifies into `mask` or the deadline passes.
Rak3172Modem::Cls Rak3172Modem::_wait(uint32_t timeout_ms, uint8_t mask) {
	const uint32_t t0 = millis();
	for (;;) {
		uint32_t left = millis() - t0 >= timeout_ms ? 0 : timeout_ms - (millis() - t0);
		if (!_read_line(left)) return CLS_NONE;
		Cls c = _classify();
		if (mask & (uint8_t)(1u << c)) return c;
		if (millis() - t0 >= timeout_ms) return CLS_NONE;
	}
}

bool Rak3172Modem::_cmd(const char* cmd, uint32_t timeout_ms) {
	_flush_input();
	RAK_SERIAL.print(cmd);
	RAK_SERIAL.print("\r\n");
	Cls c = _wait(timeout_ms, (uint8_t)((1u << CLS_OK) | (1u << CLS_ERROR)));
	if (c != CLS_OK) Serial.printf("[rak] %s -> %s\n", cmd, c == CLS_ERROR ? "error" : "no reply");
	return c == CLS_OK;
}

// Run a query and copy the FIRST non-event line back (RUI3 answers a query with the bare
// value, then OK).
bool Rak3172Modem::_query(const char* cmd, char* out, size_t cap, uint32_t timeout_ms) {
	_flush_input();
	RAK_SERIAL.print(cmd);
	RAK_SERIAL.print("\r\n");
	out[0] = 0;
	const uint32_t t0 = millis();
	while (millis() - t0 < timeout_ms) {
		uint32_t left = timeout_ms - (millis() - t0);
		if (!_read_line(left)) break;
		Cls c = _classify();
		if (c == CLS_OK)    return out[0] != 0;
		if (c == CLS_ERROR) return false;
		if (c == CLS_OTHER && !out[0]) {
			// RUI3 answers a query by echoing the command with its value —
			// "AT+VER=?" -> "AT+VER=RUI_4.0.6_RAK3172-T" (measured 2026-08-07).
			// Hand back just the value, and tolerate a bare-value build.
			const char* eq = strchr(_line, '=');
			strncpy(out, eq ? eq + 1 : _line, cap - 1);
			out[cap - 1] = 0;
		}
	}
	return out[0] != 0;
}

// --- bring-up ---------------------------------------------------------------------------

// Unconditional exit from receive. _stop_rx() deliberately short-circuits when the
// driver believes it is not armed, which is exactly wrong once an arm has been refused:
// AT_BUSY_ERROR means the module IS still receiving while our flag says it is not. This
// is the only path that can settle that disagreement.
bool Rak3172Modem::_force_rx_exit() {
	_rx_armed = false;
	return _cmd("AT+PRECV=0");
}

bool Rak3172Modem::_arm_rx() {
	_last_arm_ms = millis();
	if (_cmd(RAK_PRECV_ARM)) { _rx_armed = true; return true; }
	// The arm was refused, and on RUI3 that is AT_BUSY_ERROR in practice — the module is
	// still in receive, the opposite of what _rx_armed now says. Left alone the two
	// disagree forever and the radio is deaf for good: the deaf-RX recycle in receive()
	// is gated on _rx_armed being TRUE, and _stop_rx() no-ops while it is false, so
	// AT+PRECV=0 never gets sent and every later arm is refused for the same reason.
	// That is the load-bearing half named in the RAK_RX_RECYCLE_MS note. Force the exit,
	// then arm once more.
	//
	// Found on gateway hardware 2026-09-16 (v0.4 first article): the board stopped
	// hearing leaves at ~350 s uptime and never recovered, while still transmitting its
	// own announces every 30 s and still serving HTTP — so nothing outside looked wrong.
	_force_rx_exit();
	if (_cmd(RAK_PRECV_ARM)) { _rx_armed = true; return true; }
	_rx_armed = false;
	return false;
}

bool Rak3172Modem::_stop_rx() {
	if (!_rx_armed) return true;
	_rx_armed = false;
	return _cmd("AT+PRECV=0");
}

bool Rak3172Modem::begin(double freq_mhz, float bw_khz, uint8_t sf, uint8_t cr_denom,
                         int dbm, uint16_t preamble) {
	_online = false;
	_rx_armed = false;
	_rx_head = _rx_count = 0;

	pinMode(RAK_BOOT0_PIN, OUTPUT);
	digitalWrite(RAK_BOOT0_PIN, LOW);      // boot RUI3 from flash, never the ROM bootloader
	pinMode(RAK_NRST_PIN, OUTPUT);
	digitalWrite(RAK_NRST_PIN, LOW);       // hold in reset while the UART comes up
	delay(20);
	RAK_SERIAL.begin(RAK_UART_BAUD, SERIAL_8N1, RAK_UART_RX_PIN, RAK_UART_TX_PIN);
	delay(20);
	digitalWrite(RAK_NRST_PIN, HIGH);      // release: RUI3 boots and prints its banner

	// Drain the boot banner, then handshake. Cold boot to "AT"->"OK" is well under a
	// second on stock RUI_4.0.6, but a rail that has just come up settles first.
	const uint32_t t0 = millis();
	while (millis() - t0 < 2500 && _read_line(200)) _classify();

	bool alive = false;
	for (int i = 0; i < 4 && !alive; i++) alive = _cmd("AT", 800);
	if (!alive) {
		Serial.println("[rak] module does not answer AT (rail? UART pins? NRST?)");
		return false;
	}

	// P2P mode. Setting NWM reboots the module, so only pay that when it is wrong.
	char val[48];
	if (_query("AT+NWM=?", val, sizeof(val)) && val[0] == '0') {
		// already in P2P
	} else {
		Serial.printf("[rak] NWM is \"%s\" -> switching to P2P (module reboots)\n", val);
		_cmd("AT+NWM=0", 4000);
		delay(2500);
		const uint32_t t1 = millis();
		while (millis() - t1 < 2000 && _read_line(200)) _classify();
		if (!_cmd("AT", 1000)) {
			Serial.println("[rak] no answer after the P2P-mode reboot");
			return false;
		}
	}

	// P2P-layer encryption must be off for a RadioLib peer, which knows nothing about it.
	// Best-effort: some RUI3 builds name it differently, and the factory key is zeros.
	_cmd("AT+ENCRY=0", 800);

	if (!configure(freq_mhz, bw_khz, sf, cr_denom, dbm, preamble)) return false;

	_online = true;
	if (!_arm_rx()) Serial.println("[rak] WARNING: initial RX arm failed (deaf until the next re-arm)");

	if (_query("AT+VER=?", val, sizeof(val))) Serial.printf("[rak] firmware %s\n", val);

#if RAK_TX_SELFTEST
	// Bench gate, off by default: send ONE frame at the PHY maximum. A 255-byte payload
	// hex-encodes to a 519-character AT command line, and whether RUI3's parser accepts a
	// line that long is the one thing normal traffic does not exercise — announces are
	// ~220 bytes. It matters because every RNS Resource (the alert thumbnail, every
	// fetch_full chunk) sends 255-byte split frames. The pattern is ASCII so a listening
	// peer can count what actually arrived.
	{
		uint8_t probe[MAX_FRAME];
		for (size_t i = 0; i < MAX_FRAME; i++) probe[i] = "RAKMAXLEN-255-"[i % 14];
		bool ok = transmit(probe, MAX_FRAME, 8000);
		Serial.printf("[rak] SELFTEST max-length PSEND (%u bytes, %u-char AT line): %s\n",
		              (unsigned)MAX_FRAME, (unsigned)(9 + 2 * MAX_FRAME), ok ? "OK" : "FAILED");
	}
#endif
	return true;
}

void Rak3172Modem::end() {
	if (_online) _stop_rx();
	_online = false;
	quiesce_pins();
}

// Rail-down sequencing (respin spec §11). Safe to call when begin() never ran.
void Rak3172Modem::quiesce_pins() {
	RAK_SERIAL.end();
	pinMode(RAK_UART_TX_PIN, INPUT);
	pinMode(RAK_UART_RX_PIN, INPUT);
	pinMode(RAK_BOOT0_PIN, OUTPUT);
	digitalWrite(RAK_BOOT0_PIN, LOW);
	pinMode(RAK_NRST_PIN, OUTPUT);
	digitalWrite(RAK_NRST_PIN, LOW);
}

// leaf_rails.h calls this immediately before it cuts LORA_PWR_EN, on every path that
// powers the rail down — including the wakes that never brought the radio up.
void leaf_rak_pins_quiesce() { Rak3172Modem::quiesce_pins(); }

// --- configuration ----------------------------------------------------------------------

bool Rak3172Modem::configure(double freq_mhz, float bw_khz, uint8_t sf, uint8_t cr_denom,
                             int dbm, uint16_t preamble) {
	int bw_code;
	if      (bw_khz > 400.0f) bw_code = 2;    // 500 kHz — the granted DTS lane
	else if (bw_khz > 200.0f) bw_code = 1;    // 250 kHz
	else if (bw_khz > 100.0f) bw_code = 0;    // 125 kHz
	else {
		Serial.printf("[rak] bandwidth %.0f kHz has no RUI3 P2P code (125/250/500 only)\n", bw_khz);
		return false;
	}
	if (cr_denom < 5 || cr_denom > 8) {
		Serial.printf("[rak] coding rate 4/%u out of range\n", (unsigned)cr_denom);
		return false;
	}
	if (sf < 5 || sf > 12) {
		Serial.printf("[rak] SF%u out of range\n", (unsigned)sf);
		return false;
	}
	// RUI3's own floor and ceiling. The ADR ladder is shared with SX126x nodes, whose
	// range starts at -9 dBm, so clamp rather than fail: a leaf that refuses a grant it
	// cannot express exactly would sit deaf on a profile the gateway thinks it left.
	int p = dbm;
	if (p < 5)  p = 5;
	if (p > 22) p = 22;
	if (p != dbm) Serial.printf("[rak] TX power %d dBm clamped to %d (RUI3 allows 5..22)\n", dbm, p);

	const bool was_armed = _rx_armed;
	_stop_rx();

	char cmd[80];
	// DOUBLE, not float: 903.9f * 1000000.0f is 903900032, because a 30-bit Hz count does
	// not fit float's 24-bit mantissa. Harmless for LoRa demodulation, but the channel
	// centre is a compliance-relevant number and it should read back as the one asked for.
	snprintf(cmd, sizeof(cmd), "AT+P2P=%lu:%u:%d:%u:%u:%d",
	         (unsigned long)(freq_mhz * 1000000.0 + 0.5), (unsigned)sf, bw_code,
	         (unsigned)(cr_denom - 5), (unsigned)preamble, p);
	bool ok = _cmd(cmd, 2000);
	if (ok) {
		_dbm = p;
		Serial.printf("[rak] %s\n", cmd);
	}
	if (was_armed) _arm_rx();
	return ok;
}

bool Rak3172Modem::set_tx_dbm(int dbm) {
	int p = dbm;
	if (p < 5)  p = 5;
	if (p > 22) p = 22;
	if (p != dbm) Serial.printf("[rak] TX power %d dBm clamped to %d (RUI3 allows 5..22)\n", dbm, p);

	const bool was_armed = _rx_armed;
	_stop_rx();
	char cmd[24];
	snprintf(cmd, sizeof(cmd), "AT+PTP=%d", p);
	bool ok = _cmd(cmd);
	if (ok) _dbm = p;
	if (was_armed) _arm_rx();
	return ok;
}

// --- traffic ----------------------------------------------------------------------------

bool Rak3172Modem::transmit(const uint8_t* data, size_t len, uint32_t timeout_ms) {
	if (!_online || len == 0 || len > MAX_FRAME) return false;

	// TX and RX are exclusive: PSEND while armed answers AT_BUSY_ERROR.
	_stop_rx();

	// static, not a local: 520 bytes of stack inside an RNS Resource send is the
	// wrong place to spend it, and the driver is single-threaded by construction.
	static char cmd[9 + 2 * MAX_FRAME + 1];
	memcpy(cmd, "AT+PSEND=", 9);
	static const char* HEXD = "0123456789ABCDEF";   // not HEX: Arduino Print.h owns that name
	for (size_t i = 0; i < len; i++) {
		cmd[9 + 2 * i]     = HEXD[data[i] >> 4];
		cmd[9 + 2 * i + 1] = HEXD[data[i] & 0x0F];
	}
	cmd[9 + 2 * len] = 0;

	bool sent = false;
	for (int attempt = 0; attempt < 2 && !sent; attempt++) {
		_flush_input();
		RAK_SERIAL.print(cmd);
		RAK_SERIAL.print("\r\n");
		// RUI3 answers OK as soon as it accepts the frame, then +EVT:TXP2P DONE when the
		// air time is over. Both matter: the OK proves the command parsed (a too-long
		// command line would not), the event proves the PA actually ran.
		Cls c = _wait(2000, (uint8_t)((1u << CLS_OK) | (1u << CLS_ERROR)));
		if (c != CLS_OK) {
			// AT_BUSY_ERROR. _stop_rx() is not enough here: it no-ops whenever the
			// driver already believes it is unarmed, which is one of the ways the send
			// gets refused in the first place. Force the exit.
			if (c == CLS_ERROR && attempt == 0) { _force_rx_exit(); continue; }
			Serial.println("[rak] PSEND not accepted");
			break;
		}
		if (_wait(timeout_ms, (uint8_t)(1u << CLS_TXDONE)) == CLS_TXDONE) sent = true;
		else Serial.printf("[rak] no TXP2P DONE within %lums\n", (unsigned long)timeout_ms);
	}

	_arm_rx();
	return sent;
}

int Rak3172Modem::poll(uint8_t* out, size_t cap, float& rssi, float& snr) {
	if (!_online) return -1;

	// Drain whatever the module has said since the last call. Bounded so a chattering
	// module cannot hold the caller's loop.
	for (int i = 0; i < 8 && RAK_SERIAL.available(); i++) {
		if (!_read_line(0)) break;
		Cls c = _classify();
		// CLS_RX needs NO re-arm: armed with 65534 the module keeps receiving, and an
		// AT+PRECV while it is armed only answers AT_BUSY_ERROR. The old re-arm-on-
		// every-event dance (needed when this armed with one-shot 65535) opened a deaf
		// UART-round-trip gap exactly where a split packet's second frame lands.
		if (c == CLS_RXTIMEOUT) {
			// Only a finite-timeout arm ever produces this event; 65534 never does.
			_rx_armed = false;
			_arm_rx();
		}
	}

	// Deaf-RX recycle (see RAK_RX_RECYCLE_MS). Cheap: two AT round trips a minute, and
	// only while armed — a wake short enough to never reach a minute never pays it.
	if (_rx_armed && (uint32_t)(millis() - _last_arm_ms) > RAK_RX_RECYCLE_MS) {
		_stop_rx();
		_arm_rx();
	}

	if (_rx_count == 0) return -1;
	RxFrame& f = _rx[_rx_head];
	size_t n = f.len < cap ? f.len : cap;
	memcpy(out, f.buf, n);
	rssi = f.rssi;
	snr  = f.snr;
	_rx_head = (_rx_head + 1) % RX_SLOTS;
	_rx_count--;
	return (int)n;
}

// --- RF test modes ----------------------------------------------------------------------

bool Rak3172Modem::cw_start(double freq_mhz, int dbm, uint32_t duration_ms) {
	_stop_rx();
	char cmd[48];
	snprintf(cmd, sizeof(cmd), "AT+CW=%lu:%d:%lu",
	         (unsigned long)(freq_mhz * 1000000.0 + 0.5), dbm, (unsigned long)duration_ms);
	// AT+CW returns immediately; the carrier runs in the background until AT+TOFF or the
	// duration expires (verified on hardware 2026-08-07).
	return _cmd(cmd, 2000);
}

void Rak3172Modem::test_stop() {
	_cmd("AT+TOFF", 2000);
	if (_online) _arm_rx();
}

#endif // ARDUINO && LORA_BACKEND_RAK3172
