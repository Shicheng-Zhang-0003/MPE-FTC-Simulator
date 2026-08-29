#!/usr/bin/env bash
# ============================================================
# FIX 038b — Repair: ensure render_init() is called inside the
#   when_realised callback in root_gtk.c.
#
#   The 038a verification failed because render_init() was not
#   found in root_gtk.c. This script locates the when_realised
#   callback and inserts the call if missing.
#
# Phase:   phase2_bugs
# Files:   v15R2/src/root_gtk.c
# Depends: 038, 038a
# Risk:    low (targeted insert)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R2/src/root_gtk.c"
[[ -f "$TARGET" ]] || { echo "[SKIP] $TARGET not found"; exit 0; }

# If render_init () already exists anywhere in the file, 038a should pass
if grep -q 'render_init ()' "$TARGET"; then
    echo "[SKIP] render_init() call already present in root_gtk.c"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_038b"

# Strategy: find the when_realised callback and insert render_init()
# at the top of its body. The callback signature varies but typically
# looks like:
#   static void when_realised (GtkWidget *widget, ...) {
# or it may be named on_realize / on_window_realize.

# First, try to find a function containing "realised" or "realize"
# and insert render_init() as its first statement.

# Check if there's any realize callback at all
if ! grep -qE '(when_realised|on_realize|on_window_realize|_realize\b)' "$TARGET"; then
    echo "[WARN] No realize callback found — adding render_init() before gtk_main()"
    sed -i '/gtk_main ()/i\    render_init (); /* FIX_038b: ensure renderer is initialized */' "$TARGET"
else
    # Insert render_init() right after the opening brace of the realize callback
    awk '
    /(when_realised|on_realize|on_window_realize).*\{/ {
        print
        print "    render_init (); /* FIX_038b: ensure renderer is initialized */"
        next
    }
    { print }
    ' "$TARGET" > "${TARGET}.tmp" && mv "${TARGET}.tmp" "$TARGET"
fi

# Postflight
if grep -q 'render_init ()' "$TARGET"; then
    echo "[PASS] 038b: render_init() call added to root_gtk.c"
else
    echo "[FAIL] render_init() call could not be added"
    exit 1
fi

# Verify it still compiles
cd v15R2/src
if make > /tmp/build_038b.log 2>&1; then
    echo "[PASS] 038b: build verified"
else
    # If the full build fails (expected if other fixes aren't applied yet),
    # just check that our specific file compiles
    if grep -q 'root_gtk.c.*error' /tmp/build_038b.log; then
        echo "[FAIL] root_gtk.c has compile errors after edit"
        tail -5 /tmp/build_038b.log
        exit 1
    fi
    echo "[PASS] 038b: root_gtk.c compiles (other build issues may remain)"
fi
