// Drain half of the SD upload queue — the delete-only-after-2xx contract.
//
// Split out so the leaf gate harness can compile THE SAME state machine and drive it with
// scripted upload outcomes (leaf/src/sdq_gates.inc). The gateway board cannot run these
// tests at all — its TF slot will not mount a card.
//
// Include AFTER the host has provided:
//   enum UploadOutcome { UP_RETRY, UP_OK, UP_DROP }
//   upload_body(eid, cam, kind, capts, buf, File*, len) -> UploadOutcome
//   gw_fallback_camera() -> const char*
//   have_unnote(eid, kind)
// and after sd_queue.inc (this uses sdq_scan/sdq_name/sdq_read_header and s_sdq).
//
// Deliberately NOT covered by the leaf harness: the multipart body construction inside
// upload_body itself (ConcatStream, Content-Length, the HTTPClient call). Injecting the
// outcome tests the DECISIONS — keep on ambiguous, delete on 2xx, revoke-then-delete on
// 4xx, drop an unreadable entry rather than head-of-line block — which is where the data
// loss would be. The wire format needs a real server.
// Drain ONE queued frame, newest first. Returns true when it did work (so loop() can
// come straight back for the next one while the pipe is open). Deletes an entry only
// after a 2xx or a permanent 4xx — an ambiguous failure keeps the file, and re-sending
// is safe because ingest is idempotent per (event_id, kind).
static bool sdq_drain_one() {
    SdqPick newest;
    sdq_scan(&newest, nullptr, nullptr, /*with_free=*/false);
    if (!s_sdq.mounted || !newest.valid) return false;
    char path[32];
    sdq_name(path, sizeof(path), newest.key, newest.full);

    char eid[48] = "", cam[48] = "", kind[16] = "";
    long long capts = 0;
    size_t len = 0, hdr = 0;
    File f = SDQ_FS.open(path, FILE_READ);
    if (!f || !sdq_read_header(f, eid, sizeof(eid), cam, sizeof(cam), kind, sizeof(kind),
                              &capts, &len, &hdr)) {
        // Unreadable or torn: it can never become a valid POST, so drop it rather than
        // let it head-of-line block the queue forever.
        if (f) f.close();
        Serial.printf("[gw] sd queue: %s is unreadable -> dropped\n", path);
        log_event("sd queue entry %s BAD -> dropped", path);
        SDQ_FS.remove(path);
        s_sdq.read_errors++;
        if (s_sdq.depth) s_sdq.depth--;
        return true;
    }
    if (!kind[0]) snprintf(kind, sizeof(kind), "%s", newest.full ? "full" : "thumb");
    if (!cam[0])  snprintf(cam, sizeof(cam), "%s", gw_fallback_camera());
    if (!f.seek(hdr + 1)) { f.close(); s_sdq.read_errors++; return false; }

    const UploadOutcome r = upload_body(eid, cam, kind, capts, nullptr, &f, len);
    f.close();
    if (r == UP_RETRY) return false;   // keep the file; the next pass retries it
    if (r == UP_DROP) {
        // Custody revoked: a 4xx frame never landed, so the leaf must be free to send it
        // again (a truncated body is exactly the shape that 422s and then re-sends fine).
        have_unnote(eid, kind);
        log_event("sd queue DROP %s %s (permanent 4xx)", eid, kind);
    }
    SDQ_FS.remove(path);
    if (s_sdq.depth) s_sdq.depth--;
    if (newest.full) { if (s_sdq.fulls)  s_sdq.fulls--;  }
    else             { if (s_sdq.thumbs) s_sdq.thumbs--; }
    return true;
}
