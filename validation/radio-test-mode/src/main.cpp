// Radio test-mode firmware — SX1262 bench driver for the cert lab.
// See platformio.ini header + README.md for why this exists and the safety note.
//
// No RNS, no packet stack, no app logic: this brings up the SX1262 and exposes the
// raw emission modes a §15.247 DTS lab measures, over a line-based USB-serial console.
//
// Modes:
//   CW  -> SX126x SetTxContinuousWave (RadioLib transmitDirect): unmodulated carrier.
//          Frequency accuracy, occupied-bandwidth reference, some spurious sweeps.
//   MOD -> SX126x SetTxInfinitePreamble (cmd 0xD2): continuous LoRa chirp at the set
//          BW/SF. Modulated, 100% duty, fixed channel — the DTS 6 dB bandwidth, PSD,
//          peak conducted power, band-edge and 996369-D04 spurious source.
//   PKT -> back-to-back max-length packets: an alternative 100%-duty modulated source
//          (some labs prefer real framing over infinite preamble). ~fully saturated.
//
// Hopping is OFF by construction (one fixed channel until you change FREQ). That is
// exactly the DTS operating shape (§15.247(a)(2)), not FHSS.

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#ifndef RADIO_TCXO_V
#define RADIO_TCXO_V 1.8
#endif
#ifndef RADIO_OCP_MA
#define RADIO_OCP_MA 140
#endif

// ---- radio ---------------------------------------------------------------------
SPIClass radioSPI(FSPI);
Module radioModule(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, radioSPI);
SX1262 radio(&radioModule);

// ---- current config (defaults = leaf base profile) -----------------------------
static float   cfg_freq_mhz = 915.0;  // carrier
static float   cfg_bw_khz   = 500.0;  // DTS wants >= 500 kHz 6 dB bandwidth
static uint8_t cfg_sf        = 9;
static uint8_t cfg_cr        = 5;      // 4/5
static int8_t  cfg_pwr_dbm   = 22;     // max rated; lab also measures reduced steps

enum TxState { ST_IDLE, ST_CW, ST_MOD, ST_PKT };
static TxState state = ST_IDLE;
static bool radio_up = false;

static const char* state_name(TxState s) {
  switch (s) {
    case ST_CW:  return "CW (continuous wave)";
    case ST_MOD: return "MOD (infinite-preamble modulated)";
    case ST_PKT: return "PKT (back-to-back packets)";
    default:     return "IDLE (standby)";
  }
}

// Put the chip in standby and (re)apply the current LoRa config. Every mode entry
// goes through here so FREQ/BW/SF/CR/PWR changes always take effect on the next TX.
static bool apply_config() {
  radio.standby();
  int16_t st = radio.begin(cfg_freq_mhz, cfg_bw_khz, cfg_sf, cfg_cr,
                           RADIOLIB_SX126X_SYNC_WORD_PRIVATE, cfg_pwr_dbm,
                           /*preambleLength=*/8, /*tcxoVoltage=*/RADIO_TCXO_V);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("[radio] begin FAILED (%d) — wiring/TCXO?\n", st);
    return false;
  }
  radio.setDio2AsRfSwitch(true);          // DIO2 -> TXEN (E22 + Core1262)
  radio.setCurrentLimit((float)RADIO_OCP_MA);
  radio.setOutputPower(cfg_pwr_dbm);
  radio.standby();
  return true;
}

static void stop_tx() {
  radio.standby();
  state = ST_IDLE;
  Serial.println("[tx] stopped -> IDLE");
}

static void start_cw() {
  if (!apply_config()) return;
  int16_t st = radio.transmitDirect();    // SetTxContinuousWave
  if (st == RADIOLIB_ERR_NONE) { state = ST_CW; Serial.println("[tx] CW ON"); }
  else Serial.printf("[tx] CW failed (%d)\n", st);
}

static void start_mod() {
  if (!apply_config()) return;
  // No public RadioLib wrapper for infinite preamble; issue the raw command on the
  // Module we own (same path transmitDirect() takes internally for CW). DIO2-as-RF-
  // switch asserts TXEN automatically when the chip enters TX.
  int16_t st = radioModule.SPIwriteStream(
      RADIOLIB_SX126X_CMD_SET_TX_INFINITE_PREAMBLE, NULL, 0);
  if (st == RADIOLIB_ERR_NONE) { state = ST_MOD; Serial.println("[tx] MOD (infinite preamble) ON"); }
  else Serial.printf("[tx] MOD failed (%d)\n", st);
}

static void start_pkt() {
  if (!apply_config()) return;
  state = ST_PKT;
  Serial.println("[tx] PKT (back-to-back 255 B) ON");
}

static void print_status() {
  Serial.printf("[status] %s | %.3f MHz | BW %.0f kHz | SF%u | CR 4/%u | %d dBm"
                " | TCXO %.1f V | OCP %d mA | radio %s\n",
                state_name(state), cfg_freq_mhz, cfg_bw_khz, cfg_sf, cfg_cr,
                cfg_pwr_dbm, (float)RADIO_TCXO_V, (int)RADIO_OCP_MA,
                radio_up ? "up" : "DOWN");
}

static void print_help() {
  Serial.println(F(
    "\n=== radio test-mode — SX1262 cert bench ===\n"
    "  help | ?          this list\n"
    "  status            current config + state\n"
    "  freq <MHz>        carrier frequency (e.g. freq 902.0)\n"
    "  bw <kHz>          LoRa bandwidth (500 250 125 ...; DTS wants >=500)\n"
    "  sf <7..12>        spreading factor\n"
    "  cr <5..8>         coding rate 4/x\n"
    "  pwr <dBm>         TX power (-9..22)\n"
    "  cw                unmodulated carrier ON\n"
    "  mod               modulated continuous (infinite preamble) ON\n"
    "  pkt               back-to-back max-length packets ON\n"
    "  stop | idle       standby, stop TX\n"
    "Changing freq/bw/sf/cr/pwr while transmitting re-arms on the next cw/mod/pkt.\n"
    "SAFETY: radiates up to +22 dBm continuously — into attenuation/shield only.\n"));
}

// ---- line-based command parser -------------------------------------------------
static char linebuf[64];
static uint8_t linelen = 0;

static void handle_line(char* line) {
  // lowercase the verb; split verb + arg
  char* sp = strchr(line, ' ');
  char* arg = NULL;
  if (sp) { *sp = '\0'; arg = sp + 1; while (*arg == ' ') arg++; }
  for (char* p = line; *p; p++) *p = tolower(*p);

  bool live = (state != ST_IDLE);  // a param change should visibly re-arm if transmitting

  if (!strcmp(line, "help") || !strcmp(line, "?"))      { print_help(); }
  else if (!strcmp(line, "status"))                     { print_status(); }
  else if (!strcmp(line, "stop") || !strcmp(line, "idle")) { stop_tx(); }
  else if (!strcmp(line, "cw"))                         { start_cw(); }
  else if (!strcmp(line, "mod"))                        { start_mod(); }
  else if (!strcmp(line, "pkt"))                        { start_pkt(); }
  else if (!strcmp(line, "freq") && arg) { cfg_freq_mhz = atof(arg); Serial.printf("[cfg] freq %.3f MHz\n", cfg_freq_mhz); if (live) stop_tx(); }
  else if (!strcmp(line, "bw")   && arg) { cfg_bw_khz = atof(arg);   Serial.printf("[cfg] bw %.0f kHz\n", cfg_bw_khz);   if (live) stop_tx(); }
  else if (!strcmp(line, "sf")   && arg) { cfg_sf = (uint8_t)atoi(arg); Serial.printf("[cfg] SF%u\n", cfg_sf);          if (live) stop_tx(); }
  else if (!strcmp(line, "cr")   && arg) { cfg_cr = (uint8_t)atoi(arg); Serial.printf("[cfg] CR 4/%u\n", cfg_cr);       if (live) stop_tx(); }
  else if (!strcmp(line, "pwr")  && arg) { cfg_pwr_dbm = (int8_t)atoi(arg); Serial.printf("[cfg] %d dBm\n", cfg_pwr_dbm); if (live) stop_tx(); }
  else if (line[0]) { Serial.printf("[?] unknown '%s' — type help\n", line); }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) { /* wait for USB CDC, bounded */ }
  Serial.println("\n=== MeshCam radio test-mode ===");

  radioSPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
  radio_up = apply_config();
  Serial.println(radio_up ? "[radio] SX1262 up, standby." : "[radio] SX1262 DID NOT ANSWER — check wiring.");
  print_help();
  print_status();
}

void loop() {
  // PKT mode: fire max-length packets back to back for a saturated modulated source.
  if (state == ST_PKT) {
    static uint8_t buf[255];
    static bool filled = false;
    if (!filled) { for (int i = 0; i < 255; i++) buf[i] = (uint8_t)i; filled = true; }
    radio.transmit(buf, sizeof(buf));   // blocking; loops immediately -> ~100% duty
  }

  // Drain serial into the line buffer.
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      linebuf[linelen] = '\0';
      handle_line(linebuf);
      linelen = 0;
    } else if (linelen < sizeof(linebuf) - 1) {
      linebuf[linelen++] = c;
    }
  }
}
