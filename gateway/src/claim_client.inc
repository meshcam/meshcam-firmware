// The claim client + state machine (step 7) — how an UNCLAIMED unit becomes a
// customer's. Design of record: homelab docs/trailcam/gateway-provisioning.md,
// "Gateway claim + site binding". The whole cloud side is live (claim.getmeshcam.com);
// this file is the device half.
//
// The machine sits BESIDE the network machine, not inside it: a PoE unit is unclaimed
// too, and claims over the wire. The code arrives from any front-end (serial console
// today; portal + LAN /setup are step 8) via claim_start(), and everything after that
// is this file: sign, POST, parse, persist.
//
//        UNCLAIMED ──code entered──► CLAIMING ──"unbound"──► AWAITING_ACCOUNT
//            ▲                        │   │ 200 "claimed"          │ retry ~5 min
//            │ factory reset          │   ▼                        │ 200 "claimed"
//            │ (step 9)               │  CLAIMED ◄─────────────────┘
//            │                        │   │ two ingest 401s, >10 min apart
//            │    400/401/404/409/410 │   ▼
//            └─── CLAIM_REJECTED ◄────┘  RECLAIMING ── 200 ──► CLAIMED
//                       ▲                 │ 4xx: released/retired -> the stored code
//                       └─────────────────┘      is dead (release ROTATES it, so this
//                                                is a 404), clear the claim block
//
// Retryable (429 / 5xx / timeout / conn-fail): stay in state, back off 5 s -> 5 min,
// honouring a retry_after_s from the server. Terminal rejections wait for a human.
//
// Include AFTER json_str()/json_ll() and the claim block statics; the console and the
// upload paths reach the entry points through the forward declarations beside the
// claim block up top.

#ifndef GW_NO_CLAIM

#ifndef GW_CLAIM_URL
#define GW_CLAIM_URL "https://claim.getmeshcam.com/api/v1/claim"
#endif
#ifndef GW_HW_ID
#define GW_HW_ID "meshcam-gw"
#endif

enum ClaimState : uint8_t {
    CL_UNCLAIMED = 0, CL_CLAIMING, CL_AWAITING_ACCOUNT, CL_CLAIMED, CL_RECLAIMING,
    CL_REJECTED,
};
static ClaimState s_claim_state    = CL_UNCLAIMED;
static uint32_t   s_claim_next_ms  = 0;      // earliest next attempt (millis)
static uint32_t   s_claim_backoff_s = 5;     // doubles to 300 on retryable failures
static char       s_claim_err[24]  = "";     // last terminal error, for `claim show`
static uint32_t   s_claim_401_first_ms = 0;  // first unexplained ingest 401 (0 = none)

static const char* claim_state_str() {
    switch (s_claim_state) {
        case CL_UNCLAIMED:        return "unclaimed";
        case CL_CLAIMING:         return "claiming";
        case CL_AWAITING_ACCOUNT: return "awaiting-account";
        case CL_CLAIMED:          return "claimed";
        case CL_RECLAIMING:       return "reclaiming";
        case CL_REJECTED:         return "rejected";
    }
    return "?";
}

// --- code normalization, mirrored from the control plane -------------------------------
// Crockford base32: no I L O U. The customer may type the printed form ("K7T9M-4QXV3",
// any case, I/L for 1, O for 0); the SIGNATURE covers the 10 normalized symbols, so
// normalizing wrong here is a 401 indistinguishable from a wrong code — the single most
// likely firmware bug, called out in the design doc and pinned by the cloud tests.
static const char CLAIM_ALPHABET[] = "0123456789ABCDEFGHJKMNPQRSTVWXYZ";

static bool claim_normalize(const char* in, char out[11]) {
    int n = 0;
    for (const char* p = in; *p; p++) {
        char c = *p;
        if (c >= 'a' && c <= 'z') c -= 32;
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z'))) continue;  // strip - and spaces
        if (c == 'I' || c == 'L') c = '1';
        if (c == 'O') c = '0';
        if (!strchr(CLAIM_ALPHABET, c)) return false;   // U, or other non-alphabet letter
        if (n >= 10) return false;
        out[n++] = c;
    }
    out[n] = '\0';
    return n == 10;
}

// Check symbol = ALPHABET[sha256(first 9 symbols)[0] >> 3]. Catches a typo at the
// console instead of burning a rate-limited request on it. truncated_hash() is the
// leading bytes of the full SHA-256, so byte 0 is byte 0.
static bool claim_check_ok(const char code[11]) {
    RNS::Bytes h = RNS::Identity::truncated_hash(RNS::Bytes((const uint8_t*)code, 9));
    return code[9] == CLAIM_ALPHABET[h.data()[0] >> 3];
}

// --- entry point (console today; portal and /setup land here in step 8) ----------------
static bool claim_start(const char* entered) {
    char code[11];
    if (!claim_normalize(entered, code)) {
        Serial.println("[gw] claim: not a valid code (10 symbols, e.g. K7T9M-4QXV3)");
        return false;
    }
    if (!claim_check_ok(code)) {
        Serial.println("[gw] claim: check symbol mismatch — one symbol is mistyped");
        return false;
    }
    // Persist the code NOW, before the first attempt: a power blip mid-claim must not
    // cost the barn walk. This is also what lets RECLAIMING run unattended later.
    Preferences p;
    p.begin("gateway", /*readOnly=*/false);
    p.putString("claim_code", code);
    p.end();
    strlcpy(s_claim_code, code, sizeof(s_claim_code));
    s_claim_state     = CL_CLAIMING;
    s_claim_next_ms   = millis();
    s_claim_backoff_s = 5;
    s_claim_err[0]    = '\0';
    Serial.printf("[gw] claim: code accepted, claiming as %s...\n", s_device_id);
    return true;
}

// Two ingest 401s more than 10 minutes apart = a revoked token (one 401 is a deploy or
// a blip); re-claim with the stored code and keep running. Any ingest 2xx resets.
static void claim_note_ingest_401() {
    if (s_claim_state != CL_CLAIMED || !s_claim_code[0] || !s_claim_tok[0]) return;
    const uint32_t now = millis() ? millis() : 1;
    if (!s_claim_401_first_ms) {
        s_claim_401_first_ms = now;
        Serial.println("[gw] claim: ingest 401 noted — a second one >10 min from now re-claims");
        return;
    }
    if (now - s_claim_401_first_ms > 10u * 60u * 1000u) {
        s_claim_401_first_ms = 0;
        s_claim_state     = CL_RECLAIMING;
        s_claim_next_ms   = now;
        s_claim_backoff_s = 5;
        Serial.println("[gw] claim: second 401 >10 min apart -> token revoked, re-claiming");
        log_event("token revoked -> reclaim");
    }
}

static void claim_note_ingest_ok() { s_claim_401_first_ms = 0; }

// --- persistence -----------------------------------------------------------------------
static void claim_save(const char* site, const char* url, const char* tok,
                       const char* tid) {
    Preferences p;
    p.begin("gateway", /*readOnly=*/false);
    p.putString("claim_site", site);
    p.putString("claim_url",  url);
    p.putString("claim_tok",  tok);
    p.putString("claim_tid",  tid);
    // The cached display name belongs to the PREVIOUS site; drop it so the OLED does
    // not show the old address until the next fetch_site_name() (<= 5 min).
    p.remove("site_name");
    p.end();
    strlcpy(s_claim_site, site, sizeof(s_claim_site));
    strlcpy(s_claim_url,  url,  sizeof(s_claim_url));
    strlcpy(s_claim_tok,  tok,  sizeof(s_claim_tok));
    strlcpy(s_claim_tid,  tid,  sizeof(s_claim_tid));
    s_site_name[0] = '\0';
}

// --- one attempt -----------------------------------------------------------------------
static void claim_retry_in(uint32_t secs) {
    if (secs < 5) secs = 5;
    if (secs > 3600) secs = 3600;
    s_claim_next_ms = millis() + secs * 1000u;
}

static void claim_attempt() {
    // Sign the normalized code + device id + nonce. The nonce makes every attempt's
    // signature unique, so a captured request replays as 409, not as a claim.
    char nonce[17];
    snprintf(nonce, sizeof(nonce), "%08lx%08lx",
             (unsigned long)esp_random(), (unsigned long)esp_random());
    char msg[64];
    int mlen = snprintf(msg, sizeof(msg), "%s\n%s\n%s", s_claim_code, s_device_id, nonce);
    RNS::Bytes sig = s_identity.sign(RNS::Bytes((const uint8_t*)msg, (size_t)mlen));
    if (sig.size() != 64) {
        Serial.println("[gw] claim: signing failed (no private key?) — will not retry");
        s_claim_state = CL_REJECTED;
        strlcpy(s_claim_err, "sign_failed", sizeof(s_claim_err));
        return;
    }

    char body[560];
    snprintf(body, sizeof(body),
             "{\"claim_code\":\"%s\",\"device_id\":\"%s\",\"identity_hash\":\"%s\","
             "\"nonce\":\"%s\",\"fw_version\":\"%s\",\"hw\":\"%s\",\"sig\":\"%s\"}",
             s_claim_code, s_device_id, s_identity_hash, nonce,
             GW_FW_VERSION, GW_HW_ID, sig.toHex().c_str());

    WiFiClientSecure tls;
    tls.setInsecure();
    HTTPClient http;
    http.setConnectTimeout(8000);
    http.setTimeout(10000);
    if (!http.begin(tls, GW_CLAIM_URL)) { claim_retry_in(s_claim_backoff_s); return; }
    http.addHeader("Content-Type", "application/json");
    const int code = http.POST((uint8_t*)body, strlen(body));
    String resp = code > 0 ? http.getString() : String("");
    http.end();
    const char* r = resp.c_str();

    if (code == 200) {
        char status[24] = "";
        json_str(r, "status", status, sizeof(status));
        if (!strcmp(status, "claimed")) {
            char site[65] = "", url[129] = "", tok[81] = "", tid[41] = "";
            if (!json_str(r, "site_slug", site, sizeof(site)) || !site[0] ||
                !json_str(r, "ingest_url", url, sizeof(url)) || !url[0] ||
                !json_str(r, "device_token", tok, sizeof(tok)) || !tok[0]) {
                // A 200 we cannot parse is a server contract change, not a bad code.
                Serial.printf("[gw] claim: 200 but unparseable body -> retry: %.120s\n", r);
                claim_retry_in(s_claim_backoff_s);
                if (s_claim_backoff_s < 300) s_claim_backoff_s *= 2;
                return;
            }
            json_str(r, "token_id", tid, sizeof(tid));
            claim_save(site, url, tok, tid);
            s_claim_state  = CL_CLAIMED;
            s_claim_err[0] = '\0';
            s_claim_401_first_ms = 0;
            Serial.printf("[gw] claim: CLAIMED — site \"%s\", uploading to %s\n", site, url);
            log_event("claimed: site %s", site);
            return;
        }
        if (!strcmp(status, "unbound")) {
            // Registered and released, but no account attached yet. The customer is on
            // the QR page; poll until the next retry comes back "claimed".
            if (s_claim_state != CL_AWAITING_ACCOUNT) {
                s_claim_state = CL_AWAITING_ACCOUNT;
                Serial.println("[gw] claim: no account attached yet — finish setup in the "
                               "browser (scan the label QR); retrying every few minutes");
            }
            long long ra = json_ll(r, "retry_after_s");
            claim_retry_in(ra > 0 ? (uint32_t)ra : 300u);
            return;
        }
        Serial.printf("[gw] claim: 200 with unknown status \"%s\" -> retry\n", status);
        claim_retry_in(s_claim_backoff_s);
        if (s_claim_backoff_s < 300) s_claim_backoff_s *= 2;
        return;
    }

    if (code == 400 || code == 401 || code == 404 || code == 409 || code == 410) {
        // Terminal for this code. During RECLAIMING any of these means the STORED
        // code is dead — a release rotates the code server-side, so a released unit
        // sees 404 here, not 409 — and keeping a dead claim block would leave the
        // unit uploading into 401s forever with no way back. Clear it; the SD queue
        // holds the frames and the next code enters via console (or /setup, step 8).
        if (s_claim_state == CL_RECLAIMING) {
            claim_clear();
            strlcpy(s_claim_err, "released", sizeof(s_claim_err));
            Serial.printf("[gw] claim: re-claim rejected (HTTP %d) — this unit was "
                          "released or retired; claim block cleared, a new code is "
                          "required\n", code);
            log_event("reclaim rejected (HTTP %d): claim cleared", code);
        } else if (code == 409) {
            strlcpy(s_claim_err, "already_claimed", sizeof(s_claim_err));
            Serial.println("[gw] claim: code already claimed by another device (409)");
            log_event("claim 409: already claimed");
        } else if (code == 410) {
            strlcpy(s_claim_err, "retired", sizeof(s_claim_err));
            Serial.println("[gw] claim: this unit is retired (410)");
            log_event("claim 410: unit retired");
        } else {
            // One body, one message, deliberately (the server refuses to reveal
            // which): the code is wrong for THIS unit. 401 with a correct-looking
            // code usually means this key is not the one registered at kitting.
            strlcpy(s_claim_err, "bad_code", sizeof(s_claim_err));
            Serial.printf("[gw] claim: rejected (HTTP %d) — code unknown or not this "
                          "unit's. Check the label and `claim <code>` again\n", code);
            log_event("claim rejected (HTTP %d)", code);
        }
        s_claim_state = CL_REJECTED;
        return;
    }
    if (code == 423) {
        // Registered but not released (QC / RMA hold). Resolves server-side; keep
        // trying gently rather than making a bench operator re-type the code.
        Serial.println("[gw] claim: unit not released yet (423) — retrying in 5 min");
        claim_retry_in(300);
        return;
    }
    // 429 / 5xx / timeout / connection failure: stay in state and back off.
    long long ra = (code == 429 || code == 503) ? json_ll(r, "retry_after_s") : 0;
    Serial.printf("[gw] claim: HTTP %d -> retry in %lus\n", code,
                  (unsigned long)(ra > 0 ? (uint32_t)ra : s_claim_backoff_s));
    claim_retry_in(ra > 0 ? (uint32_t)ra : s_claim_backoff_s);
    if (s_claim_backoff_s < 300) s_claim_backoff_s *= 2;
}

// --- boot + loop hooks -----------------------------------------------------------------
// Resume from NVS: a token means CLAIMED; a code without a token is an interrupted
// claim and resumes CLAIMING on its own (the "power blip in the barn" case).
static void claim_init_state() {
    if (s_claim_tok[0]) {
        s_claim_state = CL_CLAIMED;
    } else if (s_claim_code[0]) {
        s_claim_state   = CL_CLAIMING;
        s_claim_next_ms = millis();
        Serial.println("[gw] claim: stored code but no token — resuming claim");
    }
}

static void claim_console_status() {
    Serial.printf("[gw] claim state: %s%s%s\n", claim_state_str(),
                  s_claim_err[0] ? "  last error: " : "", s_claim_err);
}

// Console `claim clear`: drop the block AND the machine state (claim_clear() alone
// leaves a CLAIMED state pointing at fallback values).
static void claim_forget() {
    claim_clear();
    s_claim_state        = CL_UNCLAIMED;
    s_claim_err[0]       = '\0';
    s_claim_401_first_ms = 0;
}

// Call gated on net_up() && !radio_busy(): the POST is blocking TLS like every other
// HTTP worker here.
static void claim_tick() {
    if (s_claim_state != CL_CLAIMING && s_claim_state != CL_AWAITING_ACCOUNT &&
        s_claim_state != CL_RECLAIMING)
        return;
    if ((int32_t)(millis() - s_claim_next_ms) < 0) return;
    claim_attempt();
}

// --- LAN /setup: the claim-code form on the uplink IP ----------------------------------
// A wired unit has no SoftAP, and a mistyped code on a PoE unit must not cost a
// serial-console trip. So while the unit is not claimed, /setup on the uplink IP serves
// a form with EXACTLY ONE field — the claim code, never the WiFi fields (an
// unauthenticated re-point endpoint on a customer's LAN is the surface the portal design refused
// to create). Once CLAIMED it 404s, forever. Entering a code here cannot steal the
// unit either way: a code claims only the unit it was registered to, and the server
// checks this device's signature against that unit's key.
static bool claim_setup_live() {
    return s_claim_state != CL_CLAIMED && s_claim_state != CL_RECLAIMING;
}

static void claim_setup_render(const char* err) {
    String h;
    h.reserve(1100);
    h += FPSTR(GW_SETUP_CSS);
    // Mid-claim states resolve on their own (the machine is polling), so the page
    // reloads itself rather than telling a customer to mash refresh.
    if (s_claim_state == CL_CLAIMING || s_claim_state == CL_AWAITING_ACCOUNT)
        h += F("<meta http-equiv=refresh content=7>");
    h += F("<h1>MeshCam gateway</h1><p>Setup state: <b>");
    h += claim_state_str();
    h += F("</b>");
    if (s_claim_err[0]) { h += F(" &middot; last error: "); h += s_claim_err; }
    if (err) { h += F("<p class=e>"); h += err; }
    if (s_claim_state == CL_AWAITING_ACCOUNT)
        h += F("<p>Code accepted — finish creating the account in your browser "
               "(scan the label's QR code). This page updates on its own.");
    else if (s_claim_state == CL_CLAIMING)
        h += F("<p>Activating&hellip;");
    h += F("<form method=post action=\"/setup\">"
           "<label>Setup code (on the label, like K7T9M-4QXV3)</label>"
           "<input name=claim_code autocapitalize=characters autocorrect=off "
           "spellcheck=false>"
           "<button type=submit>Activate</button></form>");
    s_web.send(200, "text/html", h);
}

static void claim_setup_page() {
    if (!claim_setup_live()) { s_web.send(404, "text/plain", "not found\n"); return; }
    claim_setup_render(nullptr);
}

static void claim_setup_submit() {
    if (!claim_setup_live()) { s_web.send(404, "text/plain", "not found\n"); return; }
    String code = s_web.arg("claim_code");
    code.trim();
    if (!code.length()) { claim_setup_render("Enter the setup code from the label."); return; }
    if (!claim_start(code.c_str())) {
        claim_setup_render("That code doesn't look right. The letter O reads as a zero, "
                           "I and L as a one.");
        return;
    }
    claim_setup_render(nullptr);   // shows CLAIMING; the machine takes it from here
}

#endif  // GW_NO_CLAIM
