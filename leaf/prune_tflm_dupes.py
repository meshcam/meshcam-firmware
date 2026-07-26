# PlatformIO pre-script: drop tflm_esp32's flat-mirror duplicate sources in libdeps.
#
# eloquentarduino/tflm_esp32 2.0.0 (the ONLY 2.x in the PlatformIO registry, so there
# is nothing to pin to) ships a FLAT MIRROR of its signal/ source tree: src/*.cc and
# src/kiss_fft_wrappers/*.cc duplicate the canonical src/signal/src/**/*.cpp. The
# library carries an Arduino library.properties with no srcFilter, so PlatformIO
# compiles the whole src/ tree and the linker then sees ~25 "multiple definition"
# errors for every FFT/signal symbol — all inside the library, none in leaf source. A
# warm build cache hid this until 2026-07-21; a from-clean build fails to LINK.
#
# Fix: the canonical tree is signal/, so remove the flat .cc mirror before compile.
# Scoped to ONLY the two mirror locations (src/ root + src/kiss_fft_wrappers/) and to
# .cc files whose basename matches a signal/ source, so it never touches the real
# kernels tree (tensorflow/lite/micro/kernels/circular_buffer.cpp shares a basename
# with signal/src/circular_buffer.cpp but is a different file). Idempotent; no-op for
# detector-less envs (pin-monitor/sd-probe) that never pull tflm_esp32. See homelab
# docs/trailcam/leaf-pcb-bringup.md 3c.

Import("env")  # noqa: F821  (PlatformIO construct)
import os

src = env.subst("$PROJECT_LIBDEPS_DIR/$PIOENV/tflm_esp32/src")
signal = os.path.join(src, "signal")

if os.path.isdir(signal):
    canon = set()
    for root, _dirs, files in os.walk(signal):
        for f in files:
            canon.add(os.path.splitext(f)[0])
    removed = 0
    for d in (src, os.path.join(src, "kiss_fft_wrappers")):
        if not os.path.isdir(d):
            continue
        for f in os.listdir(d):
            base, ext = os.path.splitext(f)
            p = os.path.join(d, f)
            if ext == ".cc" and base in canon and os.path.isfile(p):
                os.remove(p)
                removed += 1
    if removed:
        print("[prune_tflm_dupes] removed %d tflm_esp32 flat-mirror duplicate(s)" % removed)
    else:
        print("[prune_tflm_dupes] no flat-mirror duplicates present (already clean)")
else:
    # Like patch_microreticulum: on a truly clean checkout libdeps aren't fetched until
    # after pre-scripts run, so this no-ops the first time — run `pio run` again.
    print("[prune_tflm_dupes] tflm_esp32 not fetched yet (fresh checkout?) — "
          "re-run pio after the first build installs libdeps")
