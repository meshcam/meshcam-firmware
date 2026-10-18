// The multipart POST: ConcatStream + upload_body — the WIRE FORMAT half of the ingest
// contract.
//
// Split out so the leaf wire-format harness compiles THE SAME body builder and posts it
// at a server that inspects every byte. The decision logic around it is already covered
// (sd_queue_drain.inc + the leaf gates) by injecting outcomes; that proves what the
// gateway DOES with a 2xx/4xx/5xx, not that the bytes on the wire are correct. This file
// is the part that can only be checked against a real HTTP server.
//
// Include AFTER the host has provided:
//   log_event(fmt, ...)          — event ring
//   s_stats.uploads_ok           — counter
// and with the gw_site()/gw_url()/gw_token() accessors in scope (NVS claim
// block first, baked macro fallback).
class ConcatStream : public Stream {
public:
    ConcatStream(const String& head, const uint8_t* body, size_t blen, const String& tail)
        : _h(head), _b(body), _bl(blen), _t(tail) {}
    ConcatStream(const String& head, File* body, size_t blen, const String& tail)
        : _h(head), _f(body), _bl(blen), _t(tail) {}
    int available() override {
        size_t total = _h.length() + _bl + _t.length();
        return (int)(total - _pos);
    }
    int read() override {
        uint8_t c;
        return readBytes(&c, 1) == 1 ? c : -1;
    }
    size_t readBytes(char* out, size_t want) { return readBytes((uint8_t*)out, want); }
    size_t readBytes(uint8_t* out, size_t want) {
        size_t done = 0;
        while (done < want) {
            size_t hl = _h.length();
            if (_pos < hl) {
                out[done++] = _h[_pos++];
            } else if (_pos < hl + _bl) {
                size_t off = _pos - hl;
                size_t n = min(want - done, _bl - off);
                if (_f) {
                    size_t got = _f->read(out + done, n);
                    if (got == 0) break;   // short file (shouldn't happen): end stream
                    n = got;
                } else {
                    memcpy(out + done, _b + off, n);
                }
                done += n;
                _pos += n;
            } else {
                size_t off = _pos - hl - _bl;
                if (off >= (size_t)_t.length()) break;
                out[done++] = _t[off];
                _pos++;
            }
        }
        return done;
    }
    int peek() override { return -1; }
    size_t write(uint8_t) override { return 0; }
private:
    const String&  _h;
    const uint8_t* _b = nullptr;
    File*          _f = nullptr;
    size_t         _bl;
    const String&  _t;
    size_t         _pos = 0;
};

// One multipart POST per the ingest contract. The body is a RAM buffer (thumbs) or an
// open File already positioned at its first body byte (a LittleFS spool, or an SD queue
// entry seeked past its header). Callers decide what the outcome means for custody:
// UP_OK / UP_DROP = stop holding the frame, UP_RETRY = keep it and come back later.
enum UploadOutcome { UP_RETRY = 0, UP_OK, UP_DROP };

// Which HTTP codes mean THIS FRAME is unsendable, as opposed to "the server or the path
// is unhappy right now". Only the first kind may be dropped: a dropped frame is deleted
// from the card AND has its custody revoked, so the leaf will never re-send it either.
//
// This is an ALLOWLIST on purpose. It used to be "any 4xx is permanent", which made a
// rotated or mistyped device token (ingest answers 401 "Unknown token" — verified against
// trailcam-dev, 2026-08-29) delete every frame in the queue as fast as it could POST them,
// with all three ends believing someone else had the photo. Auth, rate limiting and
// timeouts are outages, and outages are what this queue exists to survive.
//   400  malformed request
//   413  body too large -- it will be too large next time too
//   415  unsupported media type
//   422  ingest's own validation: bad kind, bad meta, empty file
// Everything else -- 401/403/404/408/429, every 5xx, every transport error -- keeps the
// frame and retries, which is safe because ingest is idempotent per (event_id, kind).
//
// constexpr so the static_asserts below travel with it: the leaf gates inject the
// OUTCOME, so this code->outcome mapping is the one part of the drain no hardware gate
// can reach. The asserts are the test, and every build of every target runs them.
constexpr bool upload_code_is_permanent(int code) {
    return code == 400 || code == 413 || code == 415 || code == 422;
}

static_assert(upload_code_is_permanent(422), "422 (bad kind/meta/empty) must drop");
static_assert(upload_code_is_permanent(400), "400 must drop");
static_assert(!upload_code_is_permanent(401), "401 is a ROTATED TOKEN, never drop a frame");
static_assert(!upload_code_is_permanent(403), "403 is an auth/policy outage, never drop");
static_assert(!upload_code_is_permanent(429), "429 is rate limiting, never drop");
static_assert(!upload_code_is_permanent(408), "408 is a timeout, never drop");
static_assert(!upload_code_is_permanent(404), "404 is a misrouted deploy, never drop");
static_assert(!upload_code_is_permanent(500) && !upload_code_is_permanent(502),
              "5xx is the server's problem, never drop");

static UploadOutcome upload_body(const char* eid, const char* cam, const char* kind,
                                 long long captured_at, const uint8_t* buf, File* f,
                                 size_t len) {
    time_t now = time(nullptr);
    if (now < 1600000000) {
        Serial.println("[gw] clock not NTP-synced yet -> deferring upload");
        return UP_RETRY;
    }
    long long capts = captured_at;
    if (capts < 1600000000) capts = (long long)now;

    static const char* B = "----tcgw7f3a";
    String head;
    head.reserve(768);
    auto field = [&](const char* name, const String& val) {
        head += "--"; head += B; head += "\r\n";
        head += "Content-Disposition: form-data; name=\""; head += name; head += "\"\r\n\r\n";
        head += val; head += "\r\n";
    };
    field("site", gw_site());
    field("camera", cam);
    field("event_id", eid);
    field("captured_at", String(capts));
    field("kind", kind);
    head += "--"; head += B; head += "\r\n";
    head += "Content-Disposition: form-data; name=\"file\"; filename=\"";
    head += eid; head += ".jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = String("\r\n--") + B + "--\r\n";

    WiFiClientSecure tls;
    tls.setInsecure();   // no CA store; the token is the auth, the LAN path is ours
    HTTPClient http;
    http.setConnectTimeout(10000);
    http.setTimeout(30000);
    UploadOutcome out = UP_RETRY;
    if (http.begin(tls, String(gw_url()))) {
        http.addHeader("Authorization", String("Bearer ") + gw_token());
        http.addHeader("Content-Type", String("multipart/form-data; boundary=") + B);
        ConcatStream mem(head, buf, len, tail);
        ConcatStream fil(head, f,   len, tail);
        ConcatStream* body = f ? &fil : &mem;
        size_t total = head.length() + len + tail.length();
        int code = http.sendRequest("POST", body, total);
        const bool ok = code >= 200 && code < 300;
#ifndef GW_NO_CLAIM
        // A revoked device token surfaces here as 401s; two of them >10 min apart
        // trigger an unattended re-claim (claim_client.inc). 2xx clears the strike.
        if (code == 401) claim_note_ingest_401();
        else if (ok)     claim_note_ingest_ok();
#endif
        out = ok ? UP_OK : UP_RETRY;
        Serial.printf("[gw] upload %s %s (%u bytes) -> HTTP %d%s\n",
                      eid, kind, (unsigned)len, code, ok ? "" : " (will retry)");
        if (ok) s_stats.uploads_ok++;
        log_event("upload %s %s -> HTTP %d", eid, kind, code);
        if (!ok && upload_code_is_permanent(code)) {
            Serial.printf("[gw] HTTP %d is permanent -> dropping: %s\n", code,
                          http.getString().substring(0, 200).c_str());
            out = UP_DROP;   // retrying a malformed frame forever helps nobody
        }
        http.end();
    } else {
        Serial.println("[gw] http.begin failed");
    }
    return out;
}
