#!/usr/bin/env -S uv run --quiet
# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial>=3.5", "requests>=2.32"]
# ///
"""Bench bridge: leaf prototype (USB serial) -> trailcam ingest API.

Plays the gateway role while the mesh has no relay/gateway: reads framed
capture events + telemetry off the leaf's serial console, spools images to
disk, and uploads until a 2xx (the same spool-until-acked rule the real
gateway daemon will use). Everything not `!TC `-prefixed is passed through as
ordinary console output, so ESP_LOG noise coexists with the protocol.

Framing (leaf side emits; see README.md for the firmware-facing spec):

    !TC TLM {"node":"c3-back-of-lake","kind":"camera","battery_v":3.29,...}
    !TC EVT {"camera":"c3-back-of-lake","event_id":"c3-123-1","captured_at":
             "2026-07-02T14:11:02Z","kind":"thumb","len":5123,"meta":{...}}
    <base64 of the JPEG, any line width>
    !TC END c3-123-1 1a2b3c4d          # crc32 (hex) of the decoded bytes

Usage:
    uv run serial_bridge.py --port /dev/ttyACM0            # live bench
    uv run serial_bridge.py --port - < captured-frames.txt # stdin (testing)
    uv run serial_bridge.py --dir /media/sd/DCIM --camera c3-back-of-lake

Credentials/URL come from --env-file (default ~/.config/meshcam/ingest.env,
keys TRAILCAM_INGEST_URL + TRAILCAM_INGEST_TOKEN) or the environment.
"""

import argparse
import base64
import binascii
import json
import logging
import os
import queue as queue_mod
import sys
import threading
import time
import zlib
from datetime import UTC, datetime
from pathlib import Path

import requests

log = logging.getLogger("bridge")

MAX_PAYLOAD = 2 * 1024 * 1024  # decoded bytes; a thumb is ~5 KB, full-res ~500 KB
FRAME_TIMEOUT_S = 30  # EVT with no END within this window is dropped
SPOOL_FLUSH_INTERVAL_S = 60
COMMAND_RESEND_SUPPRESS_S = 15  # suppress duplicate serial sends of one command.
# Short on purpose: a deep-sleeping leaf misses most serial writes (it wakes every
# ~20-30 s for a ~3 s command window), so an outstanding command must be re-sendable
# on every wake. SSE-reconnect re-emit bursts (seconds apart) still get suppressed.
WAKE_POLL_MIN_INTERVAL_S = 5  # rate-limit for the TLM-triggered outstanding-command poll


def load_env_file(path: Path) -> None:
    if not path.exists():
        return
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, _, v = line.partition("=")
        os.environ.setdefault(k.strip(), v.strip())


class Uploader:
    def __init__(self, ingest_url: str, token: str, default_site: str, spool: Path):
        self.ingest_url = ingest_url
        self.telemetry_url = ingest_url.removesuffix("/ingest") + "/telemetry"
        self.commands_url = ingest_url.removesuffix("/ingest") + "/commands"
        self.default_site = default_site
        self.spool = spool
        self.spool.mkdir(parents=True, exist_ok=True)
        self.http = requests.Session()
        self.http.headers["Authorization"] = f"Bearer {token}"

    # -- spool: <event_id>.<kind>.json (header) + .jpg (bytes), removed on 2xx --

    def spool_event(self, header: dict, jpeg: bytes) -> Path:
        stem = f"{header['event_id']}.{header.get('kind', 'thumb')}"
        (self.spool / f"{stem}.jpg").write_bytes(jpeg)
        (self.spool / f"{stem}.json").write_text(json.dumps(header))
        return self.spool / f"{stem}.json"

    def flush_spool(self) -> None:
        for hdr_path in sorted(self.spool.glob("*.json")):
            jpg_path = hdr_path.with_suffix(".jpg")
            try:
                header = json.loads(hdr_path.read_text())
                if self._post_photo(header, jpg_path.read_bytes()):
                    hdr_path.unlink(missing_ok=True)
                    jpg_path.unlink(missing_ok=True)
            except OSError as exc:
                log.warning("spool entry %s unreadable: %s", hdr_path.name, exc)

    def _post_photo(self, header: dict, jpeg: bytes) -> bool:
        data = {
            "site": header.get("site", self.default_site),
            "camera": header["camera"],
            "event_id": header["event_id"],
            "captured_at": header["captured_at"],
            "kind": header.get("kind", "thumb"),
        }
        if header.get("meta"):
            data["meta"] = json.dumps(header["meta"])
        try:
            r = self.http.post(
                self.ingest_url,
                data=data,
                files={"file": (f"{header['event_id']}.jpg", jpeg, "image/jpeg")},
                timeout=30,
            )
        except requests.RequestException as exc:
            log.warning("upload %s failed (will retry): %s", header["event_id"], exc)
            return False
        if r.ok:
            log.info("uploaded %s %s (%d bytes)", data["event_id"], data["kind"], len(jpeg))
            return True
        # 4xx = malformed frame, retrying forever won't help — park it loudly.
        level = logging.ERROR if r.status_code < 500 else logging.WARNING
        log.log(level, "upload %s -> %d %s", data["event_id"], r.status_code, r.text[:200])
        return r.status_code < 500 and False

    def poll_commands(self) -> list[dict]:
        """One-shot poll — fallback when the SSE stream is unavailable."""
        try:
            r = self.http.get(self.commands_url, timeout=15)
            if r.ok:
                return r.json()
            log.warning("command poll -> %d %s", r.status_code, r.text[:120])
        except requests.RequestException as exc:
            log.warning("command poll failed: %s", exc)
        return []

    def stream_commands(self, out: "queue_mod.Queue[dict]", stop: threading.Event) -> None:
        """Hold the SSE command stream; commands land in `out` the instant the
        server queues them (the server re-emits everything outstanding on each
        (re)connect, so reconnects double as re-delivery). Runs in a thread."""
        url = self.commands_url + "/stream"
        backoff = 5
        while not stop.is_set():
            try:
                # read timeout > the server's 25 s keepalive: a silent link dies
                # in ~40 s and we reconnect.
                with self.http.get(url, stream=True, timeout=(10, 40)) as r:
                    if r.status_code != 200:
                        raise requests.RequestException(f"HTTP {r.status_code}")
                    log.info("command stream connected")
                    backoff = 5
                    event = None
                    for line in r.iter_lines(decode_unicode=True):
                        if stop.is_set():
                            return
                        if not line:
                            event = None
                        elif line.startswith("event: "):
                            event = line[7:]
                        elif line.startswith("data: ") and event == "command":
                            try:
                                out.put(json.loads(line[6:]))
                            except json.JSONDecodeError:
                                log.warning("bad command frame: %s", line[:120])
            except requests.RequestException as exc:
                log.warning("command stream: %s — reconnect in %ds", exc, backoff)
            stop.wait(backoff)
            backoff = min(backoff * 2, 60)

    def ack_command(self, cmd_id: int) -> None:
        """Relay a leaf `!TC ACK <id>` receipt: flips the command delivered ->
        received server-side (stops redelivery; receipt is not completion — the
        same contract as the gateway relaying the mesh path's a=<id> announce).
        Fire-and-forget: a lost ack just means the server re-delivers and the
        leaf re-acks on the next window."""
        try:
            r = self.http.post(
                f"{self.commands_url}/{cmd_id}/ack",
                json={"status": "received", "detail": "leaf ack via serial"},
                timeout=15,
            )
            if r.ok:
                log.info("command %d receipt-acked", cmd_id)
            else:
                log.warning("command %d ack -> %d %s", cmd_id, r.status_code, r.text[:120])
        except requests.RequestException as exc:
            log.warning("command %d ack failed (server redelivery covers it): %s", cmd_id, exc)

    def post_telemetry(self, beat: dict) -> None:
        beat.setdefault("site", self.default_site)
        try:
            r = self.http.post(self.telemetry_url, json=beat, timeout=15)
            if r.ok:
                log.info("telemetry %s ok", beat.get("node"))
            else:
                log.warning("telemetry %s -> %d %s", beat.get("node"), r.status_code, r.text[:200])
        except requests.RequestException as exc:
            log.warning("telemetry failed (dropped — heartbeats are lossy): %s", exc)


class FrameParser:
    """Line-fed state machine for the !TC protocol."""

    def __init__(self, uploader: Uploader):
        self.up = uploader
        self._reset()

    def _reset(self) -> None:
        self.header: dict | None = None
        self.b64: list[str] = []
        self.started_at = 0.0

    def feed(self, line: str) -> None:
        if self.header and time.monotonic() - self.started_at > FRAME_TIMEOUT_S:
            log.error("frame %s timed out waiting for END — dropped", self.header.get("event_id"))
            self._reset()

        if line.startswith("!TC TLM "):
            self._telemetry(line[8:])
        elif line.startswith("!TC EVT "):
            if self.header:
                log.error("new EVT before END of %s — dropping prior", self.header.get("event_id"))
            self._reset()
            try:
                hdr = json.loads(line[8:])
                for k in ("camera", "event_id", "captured_at"):
                    assert k in hdr, f"missing {k}"
                self.header = hdr
                self.started_at = time.monotonic()
            except (json.JSONDecodeError, AssertionError) as exc:
                log.error("bad EVT header: %s (%s)", line[8:120], exc)
        elif line.startswith("!TC END "):
            self._finish(line[8:])
        elif line.startswith("!TC ACK "):
            try:
                self.up.ack_command(int(line[8:].split()[0]))
            except (ValueError, IndexError):
                log.error("bad ACK line: %s", line[:80])
        elif self.header is not None and line and not line.startswith("!TC"):
            self.b64.append(line.strip())
        elif line:
            print(line, file=sys.stderr)  # ordinary console output, pass through

    def _telemetry(self, payload: str) -> None:
        try:
            beat = json.loads(payload)
            assert "node" in beat, "missing node"
        except (json.JSONDecodeError, AssertionError) as exc:
            log.error("bad TLM: %s (%s)", payload[:120], exc)
            return
        self.up.post_telemetry(beat)

    def _finish(self, trailer: str) -> None:
        if self.header is None:
            log.error("END without EVT: %s", trailer[:80])
            return
        header, b64 = self.header, "".join(self.b64)
        self._reset()
        parts = trailer.split()
        if not parts or parts[0] != header["event_id"]:
            log.error("END event_id mismatch (%s != %s) — dropped", parts[:1], header["event_id"])
            return
        try:
            jpeg = base64.b64decode(b64, validate=True)
        except binascii.Error as exc:
            log.error("frame %s base64 decode failed: %s", header["event_id"], exc)
            return
        if not jpeg or len(jpeg) > MAX_PAYLOAD:
            log.error("frame %s size %d out of range", header["event_id"], len(jpeg))
            return
        if "len" in header and header["len"] != len(jpeg):
            log.error("frame %s len mismatch (%s != %d)", header["event_id"], header["len"], len(jpeg))
            return
        if len(parts) > 1 and f"{zlib.crc32(jpeg):08x}" != parts[1].lower():
            log.error("frame %s crc mismatch — dropped (leaf should re-send)", header["event_id"])
            return
        self.up.spool_event(header, jpeg)
        self.up.flush_spool()


def run_serial(args, uploader: Uploader) -> None:
    parser = FrameParser(uploader)
    uploader.flush_spool()
    if args.port == "-":
        for raw in sys.stdin:
            parser.feed(raw.rstrip("\r\n"))
        uploader.flush_spool()
        return

    import serial  # pyserial

    # Commands arrive over SSE the instant they're queued; the worker reconnects
    # forever and the server re-emits outstanding work on each connect.
    cmd_queue: "queue_mod.Queue[dict]" = queue_mod.Queue()
    stop = threading.Event()
    threading.Thread(
        target=uploader.stream_commands, args=(cmd_queue, stop), daemon=True
    ).start()
    recently_sent: dict[int, float] = {}  # command id -> last serial write (dedup)

    last_flush = time.monotonic()
    last_wake_poll = 0.0
    while True:
        try:
            with serial.Serial(args.port, args.baud, timeout=1) as port:
                log.info("listening on %s @ %d", args.port, args.baud)
                while True:
                    raw = port.readline()
                    now = time.monotonic()
                    if raw:
                        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                        parser.feed(line)
                        # A TLM line means the leaf is awake RIGHT NOW (it beacons
                        # just before opening its serial command window). An SSE
                        # push that landed mid-sleep died on the wire, so pull
                        # anything still outstanding and deliver into the open
                        # window. This is the sleepy-leaf counterpart of the
                        # always-on gateway's instant SSE delivery.
                        if line.startswith("!TC TLM ") and now - last_wake_poll > WAKE_POLL_MIN_INTERVAL_S:
                            last_wake_poll = now
                            for cmd in uploader.poll_commands():
                                cmd_queue.put(cmd)
                    if now - last_flush > SPOOL_FLUSH_INTERVAL_S:
                        uploader.flush_spool()
                        last_flush = now
                        recently_sent = {c: t for c, t in recently_sent.items()
                                         if now - t < COMMAND_RESEND_SUPPRESS_S}
                    # deliver queued work to the leaf (it answers fetch_full with
                    # an EVT kind=full frame for the same event_id). Re-delivery
                    # is server-side by design; suppress serial repeats briefly
                    # so reconnect re-emits don't spam the radio.
                    while True:
                        try:
                            cmd = cmd_queue.get_nowait()
                        except queue_mod.Empty:
                            break
                        cid = cmd.get("id", 0)
                        if now - recently_sent.get(cid, 0) < COMMAND_RESEND_SUPPRESS_S:
                            continue
                        recently_sent[cid] = now
                        port.write(("!TC CMD " + json.dumps(cmd) + "\n").encode())
                        log.info("-> leaf: %s %s", cmd.get("kind"), cmd.get("event_id"))
        except serial.SerialException as exc:
            log.warning("serial: %s — retrying in 3s (unplugged?)", exc)
            time.sleep(3)


def run_dir(args, uploader: Uploader) -> None:
    """One-shot: upload every *.jpg in a directory (SD-card pull)."""
    for f in sorted(Path(args.dir).glob("*.jpg")):
        header = {
            "camera": args.camera,
            "event_id": f"{args.camera}-{f.stem}",
            "captured_at": datetime.fromtimestamp(f.stat().st_mtime, tz=UTC).isoformat(),
            "kind": "full" if args.full else "thumb",
        }
        uploader.spool_event(header, f.read_bytes())
    uploader.flush_spool()
    left = len(list(uploader.spool.glob("*.json")))
    log.info("done; %d still spooled (will retry on next run)", left)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM0", help="serial port, or '-' for stdin")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dir", help="one-shot: upload *.jpg from this directory instead of serial")
    ap.add_argument("--camera", help="camera slug for --dir mode")
    ap.add_argument("--full", action="store_true", help="--dir mode: upload as kind=full")
    ap.add_argument("--site", default="site1")
    ap.add_argument(
        "--env-file",
        type=Path,
        default=Path.home() / ".config/meshcam/ingest.env",
        help="KEY=VALUE file providing TRAILCAM_INGEST_URL/TRAILCAM_INGEST_TOKEN",
    )
    ap.add_argument(
        "--spool-dir", type=Path, default=Path.home() / ".cache/trailcam-bridge/spool"
    )
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", stream=sys.stderr
    )
    load_env_file(args.env_file)
    url, token = os.environ.get("TRAILCAM_INGEST_URL"), os.environ.get("TRAILCAM_INGEST_TOKEN")
    if not url or not token:
        sys.exit(f"TRAILCAM_INGEST_URL/TRAILCAM_INGEST_TOKEN not set (env file: {args.env_file})")

    uploader = Uploader(url, token, args.site, args.spool_dir)
    if args.dir:
        if not args.camera:
            sys.exit("--dir mode requires --camera <slug>")
        run_dir(args, uploader)
    else:
        run_serial(args, uploader)


if __name__ == "__main__":
    main()
