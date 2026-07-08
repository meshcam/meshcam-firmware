#pragma once

// WiFi for the UDP bridge/endpoint (Heltec V3 has WiFi). The board joins the same LAN
// as the Pi so microReticulum-UDP <-> Python-RNS-UDP can reach each other.
#define A2_WIFI_SSID  "YOUR_SSID"        // <-- set
#define A2_WIFI_PASS  "YOUR_PASS"        // <-- set

// ENDPOINT role only: the Pi prints its destination hash on boot. Paste 32 hex chars.
#define A2_PI_DEST_HEX "00000000000000000000000000000000"  // <-- set for A2a

// ENDPOINT role: blob sizes to push to the Pi over UDP (C++ -> Python Resource).
static const size_t A2_SIZES[] = { 5 * 1024, 50 * 1024, 250 * 1024 };

#define A2_APP_NAME "trailcam_gatea2"
#define A2_ASPECT   "resource"
