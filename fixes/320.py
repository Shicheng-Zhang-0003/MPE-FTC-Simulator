#!/usr/bin/env python3
"""
320: Fix E key — clear on key release
=======================================
e_key_pressed is set on keypress but never cleared on key release.
This causes the object menu to toggle open/closed every simulation tick.

Fix: clear e_key_pressed in on_key_released().

Usage:
    cd <project_root>
    python3 fixes/320.py [--dry-run]
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_320"

def log(msg): print(f"  [320] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_320")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_input_control():
    path = SRC / "ui_input" / "input_control.c"
    content = read(path)
    if MARKER in content:
        log("[SKIP] already fixed")
        return True

    # Find on_key_released and add e_key_pressed = false
    # The release handler already clears w/a/s/d keys.
    # Add e_key_pressed clearing after the d_key_pressed line.
    old = """    if (event -> keyval == GDK_KEY_d) {input_state -> d_key_pressed = false;}
    /* MPE_TASK_21_KEYBOARD_ONLY_KEYRELEASE_BEGIN */"""
    new = f"""    if (event -> keyval == GDK_KEY_d) {{input_state -> d_key_pressed = false;}}
    if (event -> keyval == GDK_KEY_e) {{input_state -> e_key_pressed = false;}} /* {MARKER} */
    /* MPE_TASK_21_KEYBOARD_ONLY_KEYRELEASE_BEGIN */"""

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True

    # Fallback: find the d_key_pressed line in on_key_released
    lines = content.split('\n')
    in_key_released = False
    for i, line in enumerate(lines):
        if 'gboolean on_key_released' in line:
            in_key_released = True
        if in_key_released and 'GDK_KEY_d' in line and 'd_key_pressed = false' in line:
            lines.insert(i + 1, f"    if (event -> keyval == GDK_KEY_e) {{input_state -> e_key_pressed = false;}} /* {MARKER} */")
            content = '\n'.join(lines)
            write(path, content)
            return True

    log("[FAIL] could not find insertion point in on_key_released")
    return False

def build_check():
    log("Building...")
    result = subprocess.run(
        ["make", "-j4"],
        cwd=str(SRC), capture_output=True, text=True, timeout=180
    )
    if result.returncode != 0:
        log("[FAIL] build failed")
        print(result.stdout[-3000:])
        print(result.stderr[-3000:])
        return False
    log("[OK] build clean")
    return True

def main():
    print("=" * 66)
    print("  320: Fix E Key — Clear on Key Release")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
        print()
    ok = fix_input_control()
    if ok and not DRY_RUN:
        ok = build_check()
    print()
    print("=" * 66)
    if ok:
        print("  PASS — E key no longer toggles infinitely")
        print("  Next: python3 fixes/321.py")
    else:
        print("  FAILED — see errors above")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
