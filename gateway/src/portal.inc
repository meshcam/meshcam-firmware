// --- provisioning portal: SoftAP + captive portal (step 8) --------------
// The consumer front-end onto net_creds_save() AND claim_start(). A buyer with no wired
// drop and no serial terminal joins an open "meshcam-gw-XXXX" network, their phone's
// captive-portal probe pops the setup form, they pick their WiFi, type its password and
// the setup code off the label, and the box joins, then claims. One form, one POST — the
// claim code deliberately rides the same channel as the WiFi credential, which is the
// trap of building this twice. Options analysis:
// homelab docs/trailcam/gateway-provisioning.md.
//
// No WiFiManager, deliberately. The gateway already runs WebServer s_web(80), so this is
// two more handlers, a wildcard DNSServer and softAP(). Everything it writes goes through
// the same guarded sinks the serial console uses.
//
// BOUNDED, which is the actual engineering content; the form is the easy part:
//   * up only while NET_ST_PROVISION holds (no wire AND no credential to try), or on the
//     rescue trigger below;
//   * down the moment a credential is accepted, after a short grace so the phone gets the
//     confirmation page, and down instantly when ethernet preempts;
//   * NEVER resurrected by a WiFi drop on a credential that has worked before. An open AP
//     that reappears every time a household router reboots is a permanent hole in a house.
//
// The AP is open (no PSK) because there is nothing to hand a buyer as a key that is not
// either printed on the box (equally public) or a support call. The exposure is bounded
// by the rules above; the form writes exactly two things and reads nothing back.
//
// Include after claim_client.inc (the form shows claim state and calls claim_start()) and
// after the net machine (it reads s_net_state and the wifi retry counters).

#ifdef GW_HAS_PORTAL

#define GW_PORTAL_LINGER_MS    3000   // AP holds this long after a save so the phone gets
                                      // the confirmation page before its association drops
#define GW_PORTAL_RESCUE_FAILS 3      // 3 x GW_WIFI_TRY_MS ~= 45 s
#define GW_PORTAL_SSID_ROWS    24     // scan rows rendered, after de-duplication

static DNSServer s_dns;
static uint32_t  s_portal_hold_until = 0;

// Re-open the portal for a credential that was typed in, has NEVER associated, and has
// now failed GW_PORTAL_RESCUE_FAILS times: that is a mistyped passphrase, and on a sealed
// unit with no wire and no serial port it is otherwise a brick. Requires the provisioned
// entry to be the ONLY credential: a build with baked ones is a bench/dev unit, and a
// bench unit out of range of its own network must never raise an AP. The s_nvs_cred_ok
// gate is what keeps this from firing on a router that is merely down.
static bool portal_rescue_due() {
    return s_nvs_ssid[0] && !s_nvs_cred_ok && WIFI_BAKED_COUNT == 0 &&
           s_wifi_fails >= GW_PORTAL_RESCUE_FAILS;
}

static bool portal_wanted() {
    if (s_net_state == NET_ST_UP) return false;   // something is carrying traffic
    if (s_portal_hold_until && (int32_t)(millis() - s_portal_hold_until) < 0) return true;
    return s_net_state == NET_ST_PROVISION || portal_rescue_due();
}

// Scanned SSIDs are attacker-supplied strings rendered into our page, and a neighbour can
// name their network anything at all, including markup.
static void html_escape(const String& in, String& out) {
    for (size_t i = 0; i < in.length(); i++) {
        const char c = in[i];
        switch (c) {
            case '<':  out += F("&lt;");   break;
            case '>':  out += F("&gt;");   break;
            case '&':  out += F("&amp;");  break;
            case '"':  out += F("&quot;"); break;
            case '\'': out += F("&#39;");  break;
            default:   out += c;
        }
    }
}

// ASYNC on purpose: a blocking scan is 2-4 s during which the loop services neither the
// radio nor the HTTP request that asked for it. The page renders whatever the last
// completed scan holds, and always offers a text field, so a scan that fails or finds
// nothing (hidden network) never blocks provisioning.
static void portal_scan_kick() {
    if (WiFi.scanComplete() == WIFI_SCAN_RUNNING) return;
    WiFi.scanDelete();
    WiFi.scanNetworks(/*async=*/true);
}

static void portal_render(const char* err) {
    if (s_web.hasArg("rescan")) portal_scan_kick();
    const int n = WiFi.scanComplete();
    String h;
    h.reserve(3200);
    h += FPSTR(GW_SETUP_CSS);
    h += F("<h1>MeshCam gateway</h1>"
           "<p>Tell this gateway which WiFi network to use.");
    if (err) { h += F("<p class=e>"); h += err; }
    h += F("<form method=post action=\"/setup\"><label>Network</label><select name=ssid>");
    int shown = 0;
    if (n > 0) {
        String seen;   // "\n<ssid>\n" per row. Mesh routers advertise one name per AP, and
        seen.reserve(256);   // five identical rows is worse than none
        for (int i = 0; i < n && shown < GW_PORTAL_SSID_ROWS; i++) {
            const String s = WiFi.SSID(i);
            if (!s.length()) continue;                 // hidden: the text field covers it
            const String key = "\n" + s + "\n";
            if (seen.indexOf(key) >= 0) continue;
            seen += key;
            shown++;
            h += F("<option>");
            html_escape(s, h);
            h += F("</option>");
        }
    }
    if (!shown) {
        h += F("<option value=\"\">");
        h += (n == WIFI_SCAN_RUNNING) ? F("scanning&hellip;") : F("(nothing found)");
        h += F("</option>");
    }
    h += F("</select>"
           "<label>&hellip;or type the name (hidden networks)</label>"
           "<input name=ssid_other autocapitalize=off autocorrect=off spellcheck=false>"
           "<label>Password</label>"
           "<input name=psk type=password autocapitalize=off autocorrect=off spellcheck=false>");
#ifndef GW_NO_CLAIM
    // The claim code rides the SAME form as the WiFi credential — the whole point of
    // designing the portal and the claim flow together. Optional: a unit being re-provisioned onto
    // new WiFi is already claimed and leaves it blank.
    h += F("<label>Setup code (on the label, like K7T9M-4QXV3)</label>"
           "<input name=claim_code autocapitalize=characters autocorrect=off "
           "spellcheck=false placeholder=\"");
    if (s_claim_state == CL_CLAIMED) h += F("already set up — leave blank");
    h += F("\">");
#endif
    h += F("<button type=submit>Save and connect</button></form>"
           "<p><a href=\"/setup?rescan=1\">Rescan</a>");
    if (s_nvs_ssid[0]) {
        // The stored SSID is the limit of what is ever shown back, never the passphrase,
        // here or anywhere else.
        h += F(" &middot; WiFi currently set to <b>");
        html_escape(String(s_nvs_ssid), h);
        h += F("</b>");
    }
#ifndef GW_NO_CLAIM
    h += F("<p>Setup state: <b>");
    h += claim_state_str();
    h += F("</b>");
#endif
    h += F("<p>If this page ever fails to open by itself, browse to http://");
    h += WiFi.softAPIP().toString();
    h += F("/");
    s_web.send(200, "text/html", h);
}

// Serves "/" and the portal side of "/setup" while the AP is up; the LAN claim form
// (claim_client.inc) takes over when it is not.
static void portal_page() {
    portal_render(nullptr);
}

static void portal_submit() {
    String ssid = s_web.arg("ssid_other");
    ssid.trim();   // typed by hand on a phone keyboard, where a trailing space is a typo.
                   // A SSID picked from the scan list is used byte-for-byte instead.
    if (!ssid.length()) ssid = s_web.arg("ssid");
    const String psk = s_web.arg("psk");   // never trimmed: spaces are legal in a passphrase
    String code = s_web.arg("claim_code");
    code.trim();

#ifdef GW_NO_CLAIM
    code = "";
#endif
    if (!ssid.length() && !code.length()) {
        portal_render("Pick a WiFi network, or enter the setup code — or both.");
        return;
    }
    // Validate EVERYTHING before saving ANYTHING, so every error page can honestly say
    // nothing was changed and a corrected resubmit starts from a clean slate.
    if (ssid.length() >= 33 || psk.length() >= 64) {
        portal_render("That network name or password is too long to store. "
                      "Nothing was changed.");
        return;
    }
#ifndef GW_NO_CLAIM
    if (code.length() && !claim_start(code.c_str())) {
        portal_render("That setup code doesn't look right — nothing was changed. Check "
                      "the label: the letter O reads as a zero, I and L as a one.");
        return;
    }
#endif
    if (ssid.length() && !net_creds_save(ssid.c_str(), psk.c_str())) {
        portal_render("That network name or password is too long to store. "
                      "Nothing was changed.");
        return;
    }

    String h;
    h.reserve(900);
    h += FPSTR(GW_SETUP_CSS);
    h += F("<h1>Saved</h1>");
    if (ssid.length()) {
        h += F("<p>Connecting to <b>");
        html_escape(ssid, h);
        h += F("</b>.<p>This setup network will disappear in a few seconds. That is the "
               "gateway joining yours. If it comes back, the password did not work: join "
               "it again and retype the password.");
    } else {
        h += F("<p>Setup code accepted.");
    }
    if (code.length())
        h += F("<p>The gateway will finish activating on its own once it is online. "
               "If your account is not set up yet, scan the label's QR code to finish "
               "in the browser.");
    s_web.send(200, "text/html", h);
    // Response first, teardown second, with a grace window: dropping the AP inside the
    // handler would kill the association before the phone had rendered anything.
    if (ssid.length()) {
        s_portal_hold_until = millis() + GW_PORTAL_LINGER_MS;
        if (!s_portal_hold_until) s_portal_hold_until = 1;   // millis() wrap: 0 = "no hold"
        Serial.printf("[gw] portal: credential accepted for \"%s\"\n", ssid.c_str());
        log_event("wifi provisioned over the portal");
        net_creds_changed();   // rewinds the rotation cursor, ticks the selector now
    }
}

// iOS and Android decide "this network is captive" from their probe URL not returning the
// exact body they expect; the redirect is what makes the popup land on the form rather
// than on a blank page or a browser error.
static void portal_not_found() {
    if (s_portal_up) {
        s_web.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/setup");
        s_web.send(302, "text/plain", "");
        return;
    }
    s_web.send(404, "text/plain", "not found\n");
}

static void portal_start() {
    // AP_STA, not AP: scanNetworks() needs a station interface, and the rescue case keeps
    // retrying the stored credential underneath the portal, so a router that merely came
    // back late closes the portal on its own with nobody touching the box.
    WiFi.mode(WIFI_AP_STA);
    uint8_t mac[6];
    WiFi.macAddress(mac);   // the STA MAC: stable per board, and the one on the unit's label
    snprintf(s_portal_ssid, sizeof(s_portal_ssid), "meshcam-gw-%02X%02X", mac[4], mac[5]);
    WiFi.softAP(s_portal_ssid);
    s_dns.setErrorReplyCode(DNSReplyCode::NoError);
    s_dns.start(53, "*", WiFi.softAPIP());   // every name resolves to us, so the OS probe
                                             // fails its "am I online" check, pops the page
    web_begin_once();                        // net_up() is false here; the portal is the
                                             // first thing that ever needs the listener
    s_portal_up = true;
    portal_scan_kick();
    Serial.printf("[gw] PROVISIONING PORTAL up: join open network \"%s\", then http://%s/\n",
                  s_portal_ssid, WiFi.softAPIP().toString().c_str());
    log_event("provisioning portal open: %s", s_portal_ssid);
}

static void portal_stop(const char* why) {
    s_portal_up         = false;
    s_portal_hold_until = 0;
    s_dns.stop();
    WiFi.scanDelete();
    // wifioff=true here clears the AP bit only, leaving the station side as it was. This
    // deliberately does NOT set a mode: on the ethernet-preempt path the tick has already
    // put WiFi in WIFI_OFF, and forcing WIFI_STA back on would undo exactly that.
    WiFi.softAPdisconnect(true);
    Serial.printf("[gw] provisioning portal down (%s)\n", why);
    log_event("provisioning portal closed (%s)", why);
}

static void portal_sync() {
    const bool want = portal_wanted();
    if (want == s_portal_up) return;
    if (want) portal_start();
    else      portal_stop(s_net_state == NET_ST_UP ? "uplink up" : "no longer needed");
}

static void portal_loop() {
    if (s_portal_up) s_dns.processNextRequest();
}

#endif  // GW_HAS_PORTAL
