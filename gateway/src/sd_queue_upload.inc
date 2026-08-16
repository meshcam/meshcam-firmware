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
// and with GW_SITE / GW_INGEST_URL / GW_INGEST_TOKEN defined.
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
    field("site", GW_SITE);
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
    if (http.begin(tls, GW_INGEST_URL)) {
        http.addHeader("Authorization", "Bearer " GW_INGEST_TOKEN);
        http.addHeader("Content-Type", String("multipart/form-data; boundary=") + B);
        ConcatStream mem(head, buf, len, tail);
        ConcatStream fil(head, f,   len, tail);
        ConcatStream* body = f ? &fil : &mem;
        size_t total = head.length() + len + tail.length();
        int code = http.sendRequest("POST", body, total);
        const bool ok = code >= 200 && code < 300;
        out = ok ? UP_OK : UP_RETRY;
        Serial.printf("[gw] upload %s %s (%u bytes) -> HTTP %d%s\n",
                      eid, kind, (unsigned)len, code, ok ? "" : " (will retry)");
        if (ok) s_stats.uploads_ok++;
        log_event("upload %s %s -> HTTP %d", eid, kind, code);
        if (!ok && code >= 400 && code < 500) {
            Serial.printf("[gw] 4xx is permanent -> dropping: %s\n",
                          http.getString().substring(0, 200).c_str());
            out = UP_DROP;   // retrying a malformed frame forever helps nobody
        }
        http.end();
    } else {
        Serial.println("[gw] http.begin failed");
    }
    return out;
}
