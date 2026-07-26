// Gate C — W5500 SPI ethernet (+ optional SX1262 coexistence) on ESP32-S3.
// See platformio.ini header for why this gate exists and the pass criteria in README.md.
//
// Shape of the test:
//   - Bring up W5500 via the arduino-esp32 3.x ETH API (ETH_PHY_W5500 over SPI).
//   - DHCP, then an HTTP GET against TEST_URL every 5 s, forever. Count ok/fail.
//   - (eth-radio env) SX1262 in continuous RX on its own SPI bus; a 14-byte beacon
//     TX every 10 s. Count RX packets + TX results. The interesting failure modes
//     are cross-bus: missed DIO1 IRQs while the W5500 DMA is busy, heap creep, WDT.
//   - Print a one-line scoreboard every 30 s. Ten clean minutes = pass.

#include <Arduino.h>
#include <ETH.h>
#include <HTTPClient.h>

#ifndef GATE_C_NO_RADIO
#include <RadioLib.h>
#include <SPI.h>
#endif

#ifndef ETH_PHY_ADDR_W5500
#define ETH_PHY_ADDR_W5500 1
#endif

static volatile bool eth_up = false;

static uint32_t http_ok = 0, http_fail = 0;

#ifndef GATE_C_NO_RADIO
SPIClass radioSPI(FSPI);
Module radioModule(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, radioSPI);
SX1262 radio(&radioModule);
static volatile uint32_t rx_count = 0;
static uint32_t tx_ok = 0, tx_fail = 0;
static bool radio_alive = false;

void IRAM_ATTR onRadioDio1() { rx_count = rx_count + 1; }
#endif

static void onNetEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("[eth] started");
      ETH.setHostname("gate-c-w5500");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[eth] link UP");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("[eth] got IP %s  gw %s  %u Mbps%s\n",
                    ETH.localIP().toString().c_str(),
                    ETH.gatewayIP().toString().c_str(),
                    ETH.linkSpeed(), ETH.fullDuplex() ? " FD" : "");
      eth_up = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[eth] link DOWN");
      eth_up = false;
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== Gate C: W5500 ethernet"
#ifndef GATE_C_NO_RADIO
                 " + SX1262 coexistence"
#endif
                 " ===");

#ifndef GATE_C_NO_RADIO
  // Radio first: if the SX1262 doesn't answer on a quiet bus, no point blaming the W5500.
  radioSPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
  // Core1262-HF: TCXO 1.8 V on DIO3, DIO2 drives the RF switch (gate-b wiring).
  int16_t st = radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 10, 8, 1.8);
  if (st == RADIOLIB_ERR_NONE) {
    radio.setDio2AsRfSwitch(true);
    radio.setDio1Action(onRadioDio1);
    radio.startReceive();
    radio_alive = true;
    Serial.println("[radio] SX1262 up, continuous RX");
  } else {
    Serial.printf("[radio] begin FAILED (%d) — wiring? Continuing eth-only.\n", st);
  }
#endif

  Network.onEvent(onNetEvent);
  bool ok = ETH.begin(ETH_PHY_W5500, ETH_PHY_ADDR_W5500,
                      ETH_CS_PIN, ETH_INT_PIN, ETH_RST_PIN,
                      SPI3_HOST, ETH_SCK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN);
  Serial.printf("[eth] ETH.begin(W5500) -> %s\n", ok ? "ok" : "FAILED");
}

void loop() {
  static uint32_t last_get = 0, last_beacon = 0, last_score = 0;
  uint32_t now = millis();

  if (eth_up && now - last_get >= 5000) {
    last_get = now;
    HTTPClient http;
    http.setConnectTimeout(3000);
    http.begin(TEST_URL);
    int code = http.GET();
    if (code > 0 && code < 400) http_ok++; else { http_fail++; Serial.printf("[http] code %d\n", code); }
    http.end();
  }

#ifndef GATE_C_NO_RADIO
  if (radio_alive && now - last_beacon >= 10000) {
    last_beacon = now;
    uint8_t beacon[14] = "gate-c-hello!";
    int16_t st = radio.transmit(beacon, sizeof(beacon));   // blocking TX, then back to RX
    if (st == RADIOLIB_ERR_NONE) tx_ok++; else { tx_fail++; Serial.printf("[radio] tx err %d\n", st); }
    radio.startReceive();
  }
#endif

  if (now - last_score >= 30000) {
    last_score = now;
    Serial.printf("[score] up=%lus http ok/fail=%lu/%lu"
#ifndef GATE_C_NO_RADIO
                  " tx ok/fail=%lu/%lu rx_irq=%lu"
#endif
                  " heap=%u psram=%u\n",
                  now / 1000, http_ok, http_fail,
#ifndef GATE_C_NO_RADIO
                  tx_ok, tx_fail, (uint32_t)rx_count,
#endif
                  ESP.getFreeHeap(), ESP.getFreePsram());
  }

  delay(10);
}
