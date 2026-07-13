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
