/*
 * Gate B — do camera capture + RNS + a TFLite-micro tensor arena coexist on ONE
 * ESP32-S3, in RAM *and* GPIO pins?  (see ../README.md and
 * docs/trailcam/design.md "Phase-3 bench-test plan")
 *
 * Runs the realistic leaf pipeline as a sequence and logs internal-SRAM + PSRAM
 * high-water (minimum-free-ever) at every stage. The thesis is that the stages are
 * SEQUENTIAL (free the framebuffer before holding the outgoing Resource), so peak
 * simultaneous use stays within budget. This firmware measures whether that holds.
 *
 * Internal SRAM (~512 KB total) is the scarce resource and the real pass/fail axis;
 * PSRAM (8 MB) holds the big buffers (framebuffer, tensor arena). We watch both.
 *
 * APIs grounded in:
 *   esp32-camera (esp_camera_init / esp_camera_fb_get / fb_return)
 *   esp-tflite-micro arena alloc pattern: heap_caps_malloc(size, MALLOC_CAP_SPIRAM)
 *   microReticulum (Reticulum + LoRaInterface) — same bring-up as Gate A
 */

#include <Arduino.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"

#include <microStore/FileSystem.h>
#include <microStore/Adapters/UniversalFileSystem.h>
#include <LoRaInterface.h>          // vendored; SEE README: must be patched to the
#include <microReticulum.h>         //   LORA_* pins in gate_b_board.h for a cam board

#include "gate_b_board.h"
#include "gate_b_config.h"

// --- heap high-water instrumentation -------------------------------------------------
static size_t g_int_low = SIZE_MAX;
static size_t g_psram_low = SIZE_MAX;

static void report(const char* stage) {
	size_t i_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
	size_t i_min  = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);  // lowest ever
	size_t p_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
	size_t p_min  = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
	if (i_min < g_int_low)   g_int_low = i_min;
	if (p_min < g_psram_low) g_psram_low = p_min;
	Serial.printf("[gateB] %-20s | INT free=%6u min=%6u | PSRAM free=%8u min=%8u\n",
	              stage, (unsigned)i_free, (unsigned)i_min, (unsigned)p_free, (unsigned)p_min);
}

// --- camera ---------------------------------------------------------------------------
static bool camera_init() {
	camera_config_t c = {};
	c.pin_pwdn  = CAM_PIN_PWDN;  c.pin_reset = CAM_PIN_RESET; c.pin_xclk = CAM_PIN_XCLK;
	c.pin_sccb_sda = CAM_PIN_SIOD; c.pin_sccb_scl = CAM_PIN_SIOC;  // (older cores: pin_sscb_*)
	c.pin_d7 = CAM_PIN_D7; c.pin_d6 = CAM_PIN_D6; c.pin_d5 = CAM_PIN_D5; c.pin_d4 = CAM_PIN_D4;
	c.pin_d3 = CAM_PIN_D3; c.pin_d2 = CAM_PIN_D2; c.pin_d1 = CAM_PIN_D1; c.pin_d0 = CAM_PIN_D0;
	c.pin_vsync = CAM_PIN_VSYNC; c.pin_href = CAM_PIN_HREF; c.pin_pclk = CAM_PIN_PCLK;
	c.xclk_freq_hz = 20000000;
	c.ledc_timer = LEDC_TIMER_0; c.ledc_channel = LEDC_CHANNEL_0;
	c.pixel_format = PIXFORMAT_JPEG;
	c.frame_size   = GATE_B_CAPTURE_FRAMESIZE;
	c.jpeg_quality = GATE_B_CAPTURE_JPEG_Q;
	c.fb_count     = GATE_B_FB_COUNT;
	c.fb_location  = CAMERA_FB_IN_PSRAM;
	c.grab_mode    = CAMERA_GRAB_LATEST;
	esp_err_t err = esp_camera_init(&c);
	if (err != ESP_OK) { Serial.printf("[gateB] esp_camera_init failed: 0x%x\n", err); return false; }
	return true;
}

// --- TFLite arena (stub model) --------------------------------------------------------
static uint8_t* g_arena = nullptr;

static bool tflite_init() {
#if GATE_B_ARENA_IN_PSRAM
	g_arena = (uint8_t*)heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
	g_arena = (uint8_t*)heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#endif
	if (!g_arena) { Serial.println("[gateB] arena alloc FAILED"); return false; }
	memset(g_arena, 0, 4096);  // touch so it's resident
	// TODO: drop in a real INT8 detector:
	//   const tflite::Model* m = tflite::GetModel(g_model_data);
	//   static tflite::MicroMutableOpResolver<8> ops;  // add the ops the model uses
	//   static tflite::MicroInterpreter interp(m, ops, g_arena, kTensorArenaSize);
	//   interp.AllocateTensors();   // <- this is what truly sizes the arena need
	return true;
}

static bool detector_says_send(camera_fb_t* /*fb*/) {
	// TODO: real inference on a downscaled frame. Stub: "send" every other frame.
	static int n = 0; return (++n % 2) == 0;
}

// --- RNS bring-up (same shape as Gate A) ----------------------------------------------
static RNS::Reticulum reticulum({RNS::Type::NONE});
static RNS::Interface lora_interface({RNS::Type::NONE});
static microStore::FileSystem filesystem{microStore::Adapters::UniversalFileSystem()};

static void rns_init() {
	filesystem.init();
	filesystem.format();
	RNS::Utilities::OS::register_filesystem(filesystem);
#ifndef GATE_B_NO_RADIO
	// Radio path. With the Core1262 unwired (it lands Jul 8), chip->begin() hits
	// RadioLib's BUSY-pin timeout, start() returns false, and we ignore it — the run
	// continues (fails SOFT, ~a few seconds + a log line, not a hard abort). Build with
	// -DGATE_B_NO_RADIO (env freenove-s3-noradio) to skip even that wait: the camera +
	// tensor-arena RAM high-water is fully measurable without the radio, so the RAM
	// verdict is available the day the board arrives. The LoRa interface's own RAM is
	// just the small segmentation buffers, so skipping it barely moves the number; the
	// dominant costs (framebuffer, 512 KB arena, thumbnail) are all still exercised.
	lora_interface = new LoRaInterface();   // README: patch LoRaInterface.cpp to LORA_* pins
	lora_interface.mode(RNS::Type::Interface::MODE_GATEWAY);
	RNS::Transport::register_interface(lora_interface);
	lora_interface.start();
#else
	Serial.println("[gateB] GATE_B_NO_RADIO — skipping SX1262 bring-up (camera+RAM only)");
#endif
	reticulum = RNS::Reticulum();
	reticulum.transport_enabled(true);
	reticulum.start();
}

// --- one simulated PIR event ----------------------------------------------------------
static void run_event(int idx) {
	camera_fb_t* fb = esp_camera_fb_get();           // a. capture full-res -> PSRAM
	if (!fb) { Serial.println("[gateB] capture failed"); return; }
	report("after capture");

	bool send = detector_says_send(fb);              // b. tiny detector decides
	report("after inference");

	size_t full_len = fb->len;
	RNS::Bytes thumb;                                // c. build thumbnail
	if (send) {
		size_t n = full_len < GATE_B_THUMB_BYTES ? full_len : (size_t)GATE_B_THUMB_BYTES;
		thumb.append(fb->buf, n);  // TODO: real thumb = downscale+re-encode (frame2jpg);
		                           //       this slice just exercises the buffer/RAM cost
	}

	esp_camera_fb_return(fb); fb = nullptr;          // d. FREE framebuffer (key ordering)
	report("after fb return");

	if (send && reticulum) {                         // e. hand to RNS (optional w/o peer)
		// With a live link this would be: new RNS::Resource(thumb, link, true, false);
		// Without a peer, building `thumb` already captures the dominant RAM cost.
	}
	report("after handoff");
	Serial.printf("[gateB] event %d: full=%u bytes  send=%d\n", idx, (unsigned)full_len, (int)send);
}

// --------------------------------------------------------------------------------------
void setup() {
	Serial.begin(115200);
	uint32_t t0 = millis();
	while (!Serial && millis() - t0 < 3000) delay(100);

	report("boot");
	if (!psramFound()) { Serial.println("[gateB] NO PSRAM — wrong board/build flags, abort"); return; }

	if (!camera_init()) { Serial.println("[gateB] camera init failed, abort"); return; }
	report("after camera_init");

	rns_init();
	report("after rns_init");

	if (!tflite_init()) { Serial.println("[gateB] tflite init failed, abort"); return; }
	report("after tflite_init");

	Serial.println("[gateB] --- steady state reached; running events ---");
}

void loop() {
	static int idx = 0;
	reticulum.loop();                 // keep RNS serviced

	if (idx < GATE_B_EVENTS) {
		run_event(idx++);
		delay(GATE_B_EVENT_GAP_MS);
		if (idx == GATE_B_EVENTS) {
			Serial.println("[gateB] ===== RESULT =====");
			Serial.printf("[gateB] internal-SRAM worst-case free: %u bytes (gate: > %u)\n",
			              (unsigned)g_int_low, (unsigned)GATE_B_MIN_INTERNAL_FREE);
			Serial.printf("[gateB] PSRAM worst-case free:        %u bytes\n", (unsigned)g_psram_low);
			Serial.printf("[gateB] VERDICT: %s\n",
			              (g_int_low > GATE_B_MIN_INTERNAL_FREE) ? "PASS (RAM fits)" : "FAIL (internal SRAM too tight)");
			Serial.println("[gateB] (also confirm: no pin-collision crash, and a deep-sleep/wake cycle rejoins)");
		}
	}
}
