# PlatformIO pre-script: patch microReticulum's single-thread deadlock in libdeps.
#
# Upstream bug (attermann/microReticulum @ c02b6e3, found on hardware 2026-07-02):
# Transport::jobs() sets _jobs_running=true and then pumps each active link's
# Resource watchdogs (tick_resources). A watchdog that needs to (re)send a packet —
# advertise retry on the sender, part-request retry on the receiver — reaches
# Transport::outbound(), whose literal translation of Python's thread-lock wait is
#
#     while (_jobs_running) { OS::sleep(0.0005); }
#
# On the single-threaded embedded port nothing else can ever clear the flag, so the
# firmware wedges solid (observed on BOTH Heltecs simultaneously, every run, the
# moment any Resource retry fired). Same latent pattern exists in Transport::inbound.
#
# Fix: when _jobs_running is already true we ARE inside jobs() on the same thread —
# execution is strictly sequential, so proceeding is safe. Idempotent: skips files
# that already carry the PATCHED marker. Runs before every build via extra_scripts.

Import("env")  # noqa: F821  (PlatformIO construct)
import os

SPIN = (
    "\twhile (_jobs_running) {{\n"
    "\t\tTRACE(\"Transport::{fn}: sleeping...\");\n"
    "\t\tOS::sleep(0.0005);\n"
    "\t}}\n"
)
GUARD = (
    "\t// PATCHED (homelab gate-a, 2026-07-02): single-threaded port — when\n"
    "\t// _jobs_running is true we ARE inside Transport::jobs() (e.g. a Resource\n"
    "\t// watchdog retry sending a packet). The literal translation of Python's\n"
    "\t// thread-lock wait spins forever (no other thread can clear the flag);\n"
    "\t// proceeding is safe because execution is strictly sequential.\n"
    "\tif (_jobs_running) {{\n"
    "\t\tTRACE(\"Transport::{fn}: re-entered from jobs(); continuing\");\n"
    "\t}}\n"
)

libdeps = env.subst("$PROJECT_LIBDEPS_DIR/$PIOENV")
target = os.path.join(libdeps, "microReticulum", "src", "microReticulum", "Transport.cpp")

if os.path.isfile(target):
    with open(target) as f:
        src = f.read()
    if "dispatch announce handlers" in src:
        print("[patch_microreticulum] already fully patched: %s" % target)
    else:
        n = 0
        if "PATCHED (homelab gate-a" in src:
            SPIN = GUARD  # spin sites already replaced; make the loop below a no-op match
        for fn in ("outbound", "inbound"):
            old, new = SPIN.format(fn=fn), GUARD.format(fn=fn)
            if src.count(old) == 1:
                src = src.replace(old, new)
                n += 1
            else:
                raise SystemExit(
                    "[patch_microreticulum] expected exactly 1 spin site for %s in %s "
                    "(found %d) — upstream changed, re-derive the patch" % (fn, target, src.count(old))
                )
        
        # ---- Patch 2: announce handlers must fire for EVERY valid announce ---------------------
        # Upstream nests the announce-handler dispatch inside the should_add path-update branch.
        # With a PERSISTED path table, replay/timebase protection routinely sets should_add=false
        # for perfectly valid announces (e.g. a peer whose RNS clock restarted after a filesystem
        # wipe) — and app-level announce handlers then NEVER fire. Python dispatches handlers for
        # every valid non-PATH_RESPONSE announce; hoist dispatch above the gate and neuter the
        # original nested copy.
        GATE = "\t\t\t\t\tif (should_add) {\n\t\t\t\t\t\tdouble now = OS::time();"
        HOISTED = (
            "\t\t\t\t\t// PATCHED (homelab gate-a, 2026-07-03): dispatch announce handlers for\n"
            "\t\t\t\t\t// EVERY valid announce, not only path-table updates (Python parity).\n"
            "\t\t\t\t\tif (packet.context() != Type::Packet::PATH_RESPONSE) {\n"
            "\t\t\t\t\t\tfor (auto& handler : _announce_handlers) {\n"
            "\t\t\t\t\t\t\ttry {\n"
            "\t\t\t\t\t\t\t\tbool execute_callback = false;\n"
            "\t\t\t\t\t\t\t\tIdentity announce_identity(Identity::recall(packet.destination_hash()));\n"
            "\t\t\t\t\t\t\t\tif (handler->aspect_filter().empty()) {\n"
            "\t\t\t\t\t\t\t\t\texecute_callback = true;\n"
            "\t\t\t\t\t\t\t\t}\n"
            "\t\t\t\t\t\t\t\telse {\n"
            "\t\t\t\t\t\t\t\t\tBytes handler_expected_hash = Destination::hash_from_name_and_identity(handler->aspect_filter().c_str(), announce_identity);\n"
            "\t\t\t\t\t\t\t\t\tif (packet.destination_hash() == handler_expected_hash) {\n"
            "\t\t\t\t\t\t\t\t\t\texecute_callback = true;\n"
            "\t\t\t\t\t\t\t\t\t}\n"
            "\t\t\t\t\t\t\t\t}\n"
            "\t\t\t\t\t\t\t\tif (execute_callback) {\n"
            "\t\t\t\t\t\t\t\t\thandler->received_announce(\n"
            "\t\t\t\t\t\t\t\t\t\tpacket.destination_hash(),\n"
            "\t\t\t\t\t\t\t\t\t\tannounce_identity,\n"
            "\t\t\t\t\t\t\t\t\t\tIdentity::recall_app_data(packet.destination_hash())\n"
            "\t\t\t\t\t\t\t\t\t);\n"
            "\t\t\t\t\t\t\t\t}\n"
            "\t\t\t\t\t\t\t}\n"
            "\t\t\t\t\t\t\tcatch (const std::exception& e) {\n"
            "\t\t\t\t\t\t\t\tERROR(\"Error while processing external announce callback.\");\n"
            "\t\t\t\t\t\t\t}\n"
            "\t\t\t\t\t\t}\n"
            "\t\t\t\t\t}\n"
        )
        NEST_ORIG = ("\t\t\t\t\t\tif (packet.context() != Type::Packet::PATH_RESPONSE) {\n"
                     "\t\t\t\t\t\t\tTRACE(\"Transport::inbound: Not path response, sending to announce handler...\");")
        NEST_OFF  = ("\t\t\t\t\t\tif (false) {   // PATCHED: dispatch hoisted above should_add\n"
                     "\t\t\t\t\t\t\tTRACE(\"Transport::inbound: Not path response, sending to announce handler...\");")
        
        if "PATCHED (homelab gate-a, 2026-07-03): dispatch announce handlers" in src:
            print("[patch_microreticulum] announce-dispatch already patched")
        else:
            assert src.count(GATE) == 1, "should_add gate not found uniquely (%d)" % src.count(GATE)
            assert src.count(NEST_ORIG) == 1, "nested dispatch not found uniquely (%d)" % src.count(NEST_ORIG)
            src = src.replace(GATE, HOISTED + GATE)
            src = src.replace(NEST_ORIG, NEST_OFF)
            n += 1
        
        with open(target, "w") as f:
            f.write(src)
        print("[patch_microreticulum] patched %d sites in %s" % (n, target))
else:
    # First build of a clean checkout: libdeps aren't fetched until after pre-scripts.
    # The file will exist on the next build; force a second `pio run` in that case.
    print("[patch_microreticulum] %s not present yet (fresh checkout?) — "
          "run pio again after the first build fetches libdeps" % target)

# ---- Patch 3: bigger Resource windows for the LoRa link (Type.h) ------------------------
# Measured on hardware (2026-07-03, gateway-0.5.0 frame timeline): every window
# round-trip costs a fixed ~2.5-3 s (receive_part's window-final processing + the
# request_next turnaround), regardless of window size. Upstream's defaults — start at
# 4 parts, cap at 10 (or 4 when the measured rate dips under 250 B/s, which our own
# protocol overhead used to guarantee) — make a 16 KB chunk pay that tax 5-7 times.
# Double the ladder so it's paid half as often. Window size is receiver-driven (the
# sender just answers the hashes each request lists), so this only changes behavior
# where resources are RECEIVED — the gateway.
typeh = os.path.join(libdeps, "microReticulum", "src", "microReticulum", "Type.h")
if os.path.isfile(typeh):
    with open(typeh) as f:
        tsrc = f.read()
    if "PATCHED (homelab trail-cam" in tsrc:
        print("[patch_microreticulum] Type.h window ladder already patched")
    else:
        WINDOWS = [
            ("static const uint8_t WINDOW               = 4;",
             "static const uint8_t WINDOW               = 8;   // PATCHED (homelab trail-cam, 2026-07-03): window ladder x2"),
            ("static const uint8_t WINDOW_MAX_SLOW      = 10;",
             "static const uint8_t WINDOW_MAX_SLOW      = 20;  // PATCHED: was 10"),
            ("static const uint8_t WINDOW_MAX_VERY_SLOW = 4;",
             "static const uint8_t WINDOW_MAX_VERY_SLOW = 8;   // PATCHED: was 4"),
        ]
        for old, new in WINDOWS:
            if tsrc.count(old) != 1:
                raise SystemExit(
                    "[patch_microreticulum] Type.h window constant not found uniquely: %r" % old)
            tsrc = tsrc.replace(old, new)
        with open(typeh, "w") as f:
            f.write(tsrc)
        print("[patch_microreticulum] Type.h window ladder patched (8/20/8)")
else:
    print("[patch_microreticulum] %s not present yet — run pio again" % typeh)

# ---- Patch 4: announce handlers get FRAME-sourced app_data + identity (2026-07-17) ------
# Upstream hands received_announce() the output of Identity::recall_app_data()/recall()
# — the known-destinations CACHE — instead of the announce packet's own contents (its
# author even left a "CBA TODO Why does app data come from recall" note). When the cache
# misses at runtime, every handler callback starves: empty app_data (no n=/health/ack
# tails -> command acks never relay) and an invalid identity (downlink can't address the
# leaf), even though perfectly valid announces keep arriving on the air. Observed live
# on the two-leaf bench 2026-07-16/17: wire frames carried the full app_data while the
# gateway logged `announce heard ()` for hours (homelab docs/trailcam/
# announce-appdata-empty.md). Python RNS parses both from the packet; do the same. The
# announce signature was already validated over exactly these bytes before dispatch.
if os.path.isfile(target):
    with open(target) as f:
        src = f.read()
    if "PATCHED (homelab machine-id, 2026-07-17)" in src:
        print("[patch_microreticulum] frame-sourced announce dispatch already patched")
    else:
        ID_OLD = ("\n\t\t\t\t\t\t\t\tIdentity announce_identity(Identity::recall(packet.destination_hash()));\n")
        ID_NEW = (
            "\n\t\t\t\t\t\t\t\t// PATCHED (homelab machine-id, 2026-07-17): Python parity — source the\n"
            "\t\t\t\t\t\t\t\t// handler's identity and app_data from THIS validated announce frame,\n"
            "\t\t\t\t\t\t\t\t// not the known-destinations cache; cache misses starved handlers with\n"
            "\t\t\t\t\t\t\t\t// empty app_data + invalid identity while valid announces kept arriving.\n"
            "\t\t\t\t\t\t\t\tIdentity announce_identity(Identity::recall(packet.destination_hash()));\n"
            "\t\t\t\t\t\t\t\tif (!announce_identity.pub()) {\n"
            "\t\t\t\t\t\t\t\t\tIdentity from_packet(false);\n"
            "\t\t\t\t\t\t\t\t\tfrom_packet.load_public_key(packet.data().left(Type::Identity::KEYSIZE/8));\n"
            "\t\t\t\t\t\t\t\t\tif (from_packet.pub()) announce_identity = from_packet;\n"
            "\t\t\t\t\t\t\t\t}\n"
            "\t\t\t\t\t\t\t\tBytes announce_app_data;\n"
            "\t\t\t\t\t\t\t\t{\n"
            "\t\t\t\t\t\t\t\t\tsize_t ad_off = Type::Identity::KEYSIZE/8 + Type::Identity::NAME_HASH_LENGTH/8 +\n"
            "\t\t\t\t\t\t\t\t\t                Type::Identity::RANDOM_HASH_LENGTH/8 + Type::Identity::SIGLENGTH/8;\n"
            "\t\t\t\t\t\t\t\t\tif (packet.context_flag() == Type::Packet::FLAG_SET) ad_off += Type::Identity::RATCHETSIZE/8;\n"
            "\t\t\t\t\t\t\t\t\tif (packet.data().size() > ad_off) announce_app_data = packet.data().mid(ad_off);\n"
            "\t\t\t\t\t\t\t\t}\n"
        )
        AD_OLD = "\n\t\t\t\t\t\t\t\t\t\tIdentity::recall_app_data(packet.destination_hash())\n"
        AD_NEW = "\n\t\t\t\t\t\t\t\t\t\tannounce_app_data\n"
        assert src.count(ID_OLD) == 1, "hoisted identity line not found uniquely (%d)" % src.count(ID_OLD)
        assert src.count(AD_OLD) == 1, "hoisted recall_app_data line not found uniquely (%d)" % src.count(AD_OLD)
        src = src.replace(ID_OLD, ID_NEW).replace(AD_OLD, AD_NEW)
        with open(target, "w") as f:
            f.write(src)
        print("[patch_microreticulum] frame-sourced announce dispatch patched")

# ---- Patch 5: bug-6 wedge — make every silent announce-drop branch loud (2026-07-17) ----
# The 07-15/16 announce-processing wedge (homelab docs/trailcam/office-leaf-outage-
# 2026-07-16.md, Bug 6) demodulated announces that were then dropped with NO log line
# at the runtime NOTICE loglevel: every candidate branch either logged at DEBUG or not
# at all. Elevate each silent drop to NOTICE so the next wedge names its gate remotely
# (main.cpp forwards NOTICE+ into the /debuglog ring via RNS::set_log_callback).
if os.path.isfile(target):
    with open(target) as f:
        src = f.read()
    if "PATCHED (homelab bug-6 dbg" in src:
        print("[patch_microreticulum] bug-6 announce-drop visibility already patched")
    else:
        A_OLD = (
            "\t\t\tauto iter = _destinations.find(packet.destination_hash());\n"
            "\t\t\tif (iter == _destinations.end() && Identity::validate_announce(packet)) {\n"
        )
        A_NEW = (
            "\t\t\tauto iter = _destinations.find(packet.destination_hash());\n"
            "\t\t\t// PATCHED (homelab bug-6 dbg, 2026-07-17): both halves of this gate drop the\n"
            "\t\t\t// announce with no log line; a wedged gateway looked healthy at NOTICE.\n"
            "\t\t\tconst bool dbg6_local = (iter != _destinations.end());\n"
            "\t\t\tconst bool dbg6_valid = dbg6_local ? false : Identity::validate_announce(packet);\n"
            "\t\t\tif (dbg6_local) {\n"
            "\t\t\t\tNOTICEF(\"bug6: announce %s DROPPED: destination registered LOCAL\", packet.destination_hash().toHex().c_str());\n"
            "\t\t\t}\n"
            "\t\t\telse if (!dbg6_valid) {\n"
            "\t\t\t\tNOTICEF(\"bug6: announce %s DROPPED: validate_announce returned false\", packet.destination_hash().toHex().c_str());\n"
            "\t\t\t}\n"
            "\t\t\tif (!dbg6_local && dbg6_valid) {\n"
        )
        B_OLD = (
            "\t\t\t\t\t\t\t\tmark_path_unknown_state(packet.destination_hash());\n"
            "\t\t\t\t\t\t\t\tshould_add = true;\n"
            "\t\t\t\t\t\t\t}\n"
            "\t\t\t\t\t\t\telse {\n"
            "\t\t\t\t\t\t\t\tshould_add = false;\n"
            "\t\t\t\t\t\t\t}\n"
        )
        B_NEW = (
            "\t\t\t\t\t\t\t\tmark_path_unknown_state(packet.destination_hash());\n"
            "\t\t\t\t\t\t\t\tshould_add = true;\n"
            "\t\t\t\t\t\t\t}\n"
            "\t\t\t\t\t\t\telse {\n"
            "\t\t\t\t\t\t\t\tshould_add = false;\n"
            "\t\t\t\t\t\t\t\t// PATCHED (homelab bug-6 dbg): the class-1 wedge branch — was silent\n"
            "\t\t\t\t\t\t\t\tNOTICEF(\"bug6: announce %s path-update REJECTED by replay guard (emitted=%llu timebase=%llu)\",\n"
            "\t\t\t\t\t\t\t\t        packet.destination_hash().toHex().c_str(),\n"
            "\t\t\t\t\t\t\t\t        (unsigned long long)announce_emitted, (unsigned long long)path_timebase);\n"
            "\t\t\t\t\t\t\t}\n"
        )
        assert src.count(A_OLD) == 1, "bug-6 A site not found uniquely (%d)" % src.count(A_OLD)
        assert src.count(B_OLD) == 1, "bug-6 B site not found uniquely (%d)" % src.count(B_OLD)
        src = src.replace(A_OLD, A_NEW).replace(B_OLD, B_NEW)
        with open(target, "w") as f:
            f.write(src)
        print("[patch_microreticulum] bug-6 announce-drop visibility patched (Transport.cpp)")

identity_cpp = os.path.join(libdeps, "microReticulum", "src", "microReticulum", "Identity.cpp")
if os.path.isfile(identity_cpp):
    with open(identity_cpp) as f:
        isrc = f.read()
    if "PATCHED (homelab bug-6 dbg" in isrc:
        print("[patch_microreticulum] bug-6 validate_announce visibility already patched")
    else:
        SIG_OLD  = 'DEBUGF("Received invalid announce for %s: Invalid signature.'
        SIG_NEW  = ('NOTICEF("bug6: announce %s DROPPED: invalid signature.'
                    '  /* PATCHED (homelab bug-6 dbg) */')
        DEST_OLD = 'DEBUGF("Received invalid announce for %s: Destination mismatch.'
        DEST_NEW = 'NOTICEF("bug6: announce %s DROPPED: destination mismatch.'
        assert isrc.count(SIG_OLD) == 1 and isrc.count(DEST_OLD) == 1, "bug-6 Identity sites changed"
        isrc = isrc.replace(SIG_OLD, SIG_NEW).replace(DEST_OLD, DEST_NEW)
        with open(identity_cpp, "w") as f:
            f.write(isrc)
        print("[patch_microreticulum] bug-6 validate_announce visibility patched (Identity.cpp)")
else:
    print("[patch_microreticulum] Identity.cpp not present yet — run pio again")

# ---- Patch 6: microStore — detect the failed-unlink compaction (bug-6 trigger) ----------
# 07-16 wedge onset fingerprint: esp_littlefs "Failed to unlink ./known_store/seg2.dat.
# Has open FD." during a threshold-triggered compact (compact_if_threshold() never
# closes active_file, so removing the active segment fails and compact() IGNORES the
# result). A surviving stale segment desyncs the store from its index. Make both remove
# sites verify the file is actually gone, and route USTORE_LOG through a hook the
# gateway implements so store events reach /debuglog without a serial cable.
fstore = os.path.join(libdeps, "microStore", "include", "microStore", "FileStore.h")
if os.path.isfile(fstore):
    with open(fstore) as f:
        fsrc = f.read()
    if "PATCHED (homelab bug-6 dbg" in fsrc:
        print("[patch_microreticulum] microStore remove-verify already patched")
    else:
        E1_OLD = "\t\t\t_filesystem.remove(src_name);  // no-op if segment had no records / did not exist\n"
        E1_NEW = (
            "\t\t\t// PATCHED (homelab bug-6 dbg, 2026-07-17): a remove() that fails while the\n"
            "\t\t\t// segment is still open (threshold compact never closes active_file) leaves a\n"
            "\t\t\t// stale segment that desyncs the store. Verify the file is actually gone.\n"
            "\t\t\t_filesystem.remove(src_name);  // no-op if segment had no records / did not exist\n"
            "\t\t\t{\n"
            "\t\t\t\tFile chk = _filesystem.open(src_name, File::ModeRead);\n"
            "\t\t\t\tif (chk) {\n"
            "\t\t\t\t\tchk.close();\n"
            "\t\t\t\t\tUSTORE_LOG(\"[ustore] ERROR: compact: segment %s SURVIVED removal (open FD?) - stale records will resurrect\\n\", src_name);\n"
            "\t\t\t\t}\n"
            "\t\t\t}\n"
        )
        E2_OLD = (
            "\t\t\tsegment_name(i, sname);\n"
            "USTORE_LOG(\"[ustore] Removing file: %s\\n\", sname);\n"
            "\t\t\t_filesystem.remove(sname);\n"
        )
        E2_NEW = (
            "\t\t\tsegment_name(i, sname);\n"
            "USTORE_LOG(\"[ustore] Removing file: %s\\n\", sname);\n"
            "\t\t\t_filesystem.remove(sname);\n"
            "\t\t\t{\n"
            "\t\t\t\tFile chk = _filesystem.open(sname, File::ModeRead);\n"
            "\t\t\t\tif (chk) {\n"
            "\t\t\t\t\tchk.close();\n"
            "\t\t\t\t\tUSTORE_LOG(\"[ustore] ERROR: finalize: segment %s SURVIVED removal (open FD?) - stale records will resurrect\\n\", sname);\n"
            "\t\t\t\t}\n"
            "\t\t\t}\n"
        )
        assert fsrc.count(E1_OLD) == 1, "bug-6 E1 site not found uniquely (%d)" % fsrc.count(E1_OLD)
        assert fsrc.count(E2_OLD) == 1, "bug-6 E2 site not found uniquely (%d)" % fsrc.count(E2_OLD)
        fsrc = fsrc.replace(E1_OLD, E1_NEW).replace(E2_OLD, E2_NEW)
        with open(fstore, "w") as f:
            f.write(fsrc)
        print("[patch_microreticulum] microStore remove-verify patched (FileStore.h)")

    mslog = os.path.join(libdeps, "microStore", "include", "microStore", "Log.h")
    with open(mslog) as f:
        lsrc = f.read()
    if "ustore_log_hook" in lsrc:
        print("[patch_microreticulum] microStore log hook already patched")
    else:
        L_OLD = ("#ifdef USTORE_ENABLE_LOG\n"
                 "  #define USTORE_LOG(...) printf(__VA_ARGS__)\n")
        L_NEW = ("#ifdef USTORE_ENABLE_LOG\n"
                 "  // PATCHED (homelab bug-6 dbg, 2026-07-17): mirror store events into a hook the\n"
                 "  // application implements (gateway: /debuglog ring) for cable-free diagnosis.\n"
                 "  extern \"C\" void ustore_log_hook(const char* fmt, ...);\n"
                 "  #define USTORE_LOG(...) do { printf(__VA_ARGS__); ustore_log_hook(__VA_ARGS__); } while (0)\n")
        assert lsrc.count(L_OLD) == 1, "microStore Log.h changed upstream"
        lsrc = lsrc.replace(L_OLD, L_NEW)
        with open(mslog, "w") as f:
            f.write(lsrc)
        print("[patch_microreticulum] microStore log hook patched (Log.h)")
else:
    print("[patch_microreticulum] microStore not present yet — run pio again")

# ---- Patch 5c: bug-6 announce-entry breadcrumb ------------------------------------------
# One NOTICE line as each ANNOUNCE enters LOCAL HANDLING. If a wedged gateway shows rf
# frames arriving but no breadcrumb, the drop is upstream of the announce block entirely
# (interface/filter/transport region); breadcrumb-without-heard isolates the gate inside.
if os.path.isfile(target):
    with open(target) as f:
        src = f.read()
    if "bug6: announce enter" in src:
        print("[patch_microreticulum] bug-6 announce-entry breadcrumb already patched")
    else:
        C_OLD = "\t\t\t++_announces_received;\n"
        C_NEW = ("\t\t\t++_announces_received;\n"
                 "\t\t\t// PATCHED (homelab bug-6 dbg): entry breadcrumb, see patch script\n"
                 "\t\t\tNOTICEF(\"bug6: announce enter %s hops=%u ctx=%u\", packet.destination_hash().toHex().c_str(), (unsigned)packet.hops(), (unsigned)packet.context());\n")
        assert src.count(C_OLD) == 1, "bug-6 C site not found uniquely (%d)" % src.count(C_OLD)
        src = src.replace(C_OLD, C_NEW)
        with open(target, "w") as f:
            f.write(src)
        print("[patch_microreticulum] bug-6 announce-entry breadcrumb patched")

# ---- Patch 7: microStore — bug-6 ROOT FIX: close active segment before threshold compact
# rotate_segment_if_needed() closes active_file before calling compact();
# compact_if_threshold() forgot to, so littlefs refuses to unlink the active segment
# ("Has open FD"), compact() commits anyway, and the surviving stale segment desyncs
# the store. Downstream: Identity::recall() fails at runtime, and (pre patch-4) the
# announce dispatch computed the aspect-filter hash from recall — handler silently
# skipped for every announce = the Bug 6 class-2 wedge. Also the hashlist_store
# segment leak main.cpp purges at boot. Close it first; reopen on the failure path.
if os.path.isfile(fstore):
    with open(fstore) as f:
        fsrc = f.read()
    if "PATCHED (homelab bug-6 fix" in fsrc:
        print("[patch_microreticulum] microStore threshold-compact fix already patched")
    else:
        F_OLD = (
            "\t\t\tif (compact()) {\n"
            "\t\t\t\t// After threshold-triggered compaction, seg0 holds the compacted data.\n"
            "\t\t\t\t// Open seg1 for new writes, mirroring what rotate_segment_if_needed() does\n"
            "\t\t\t\t// after a rotation-triggered compaction.\n"
            "\t\t\t\tcurrent_segment = 1;\n"
            "\t\t\t\topen_segment(current_segment);\n"
            "\t\t\t}\n"
        )
        F_NEW = (
            "\t\t\t// PATCHED (homelab bug-6 fix, 2026-07-17): close the active segment BEFORE\n"
            "\t\t\t// compacting, exactly as rotate_segment_if_needed() does. Compacting with it\n"
            "\t\t\t// open makes littlefs refuse the unlink (\"Has open FD\"), compact() commits\n"
            "\t\t\t// anyway, and the surviving stale segment desyncs the store (announce-wedge\n"
            "\t\t\t// trigger, hashlist segment leak).\n"
            "\t\t\tflush_buffer();\n"
            "\t\t\tif (active_file) {\n"
            "\t\t\t\tactive_file.close();\n"
            "\t\t\t}\n"
            "\t\t\tif (compact()) {\n"
            "\t\t\t\t// After threshold-triggered compaction, seg0 holds the compacted data.\n"
            "\t\t\t\t// Open seg1 for new writes, mirroring what rotate_segment_if_needed() does\n"
            "\t\t\t\t// after a rotation-triggered compaction.\n"
            "\t\t\t\tcurrent_segment = 1;\n"
            "\t\t\t\topen_segment(current_segment);\n"
            "\t\t\t}\n"
            "\t\t\telse {\n"
            "\t\t\t\t// Failed/aborted compact: re-open the segment we closed so puts keep working.\n"
            "\t\t\t\topen_segment(current_segment);\n"
            "\t\t\t}\n"
        )
        assert fsrc.count(F_OLD) == 1, "bug-6 fix site not found uniquely (%d)" % fsrc.count(F_OLD)
        fsrc = fsrc.replace(F_OLD, F_NEW)
        with open(fstore, "w") as f:
            f.write(fsrc)
        print("[patch_microreticulum] microStore threshold-compact fix patched (FileStore.h)")
