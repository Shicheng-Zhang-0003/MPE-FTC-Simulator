#!/usr/bin/env python3
"""
322: Controller disconnect — zero all input state
===================================================
When the controller disconnects, old axis/button values remain in
gamepad_state. A future code path could accidentally use stale input.

Fix: zero all axes and buttons atomically when disconnect is detected.

Usage:
    cd <project_root>
    python3 fixes/322.py [--dry-run]
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_322"

def log(msg): print(f"  [322] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_322")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_gamepad():
    path = SRC / "ui_input" / "gamepad.c"
    content = read(path)
    if MARKER in content:
        log("[SKIP] already fixed")
        return True

    # Windows XInput disconnect handler
    old_win = """    if (XInputGetState((DWORD)pad->fd, &st) != ERROR_SUCCESS) {
        pad->connected = false;
        return;
    }"""
    new_win = f"""    if (XInputGetState((DWORD)pad->fd, &st) != ERROR_SUCCESS) {{
        pad->connected = false;
        /* {MARKER}: Zero all input state on disconnect.
         * Prevents stale axis/button values from being used
         * if a future code path doesn't check connected. */
        memset(pad->axes, 0, sizeof(pad->axes));
        memset(pad->buttons, 0, sizeof(pad->buttons));
        return;
    }}"""

    if old_win in content:
        content = content.replace(old_win, new_win, 1)
        write(path, content)
        return True

    # Linux evdev disconnect handler
    old_linux = """    if (bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        close(pad->fd); pad->fd = -1; pad->connected = false;
    }"""
    new_linux = f"""    if (bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {{
        close(pad->fd); pad->fd = -1; pad->connected = false;
        /* {MARKER}: Zero all input state on disconnect. */
        memset(pad->axes, 0, sizeof(pad->axes));
        memset(pad->buttons, 0, sizeof(pad->buttons));
    }}"""

    if old_linux in content:
        content = content.replace(old_linux, new_linux, 1)
        write(path, content)
        return True

    # If neither matched, try to find any disconnect pattern
    if 'pad->connected = false;' in content:
        # Add memset after every "pad->connected = false;" line
        lines = content.split('\n')
        modified = False
        for i, line in enumerate(lines):
            if 'pad->connected = false;' in line and MARKER not in lines[min(i+1, len(lines)-1)]:
                indent = line[:len(line) - len(line.lstrip())]
                lines.insert(i + 1, f"{indent}/* {MARKER}: Zero input state on disconnect */")
                lines.insert(i + 2, f"{indent}memset(pad->axes, 0, sizeof(pad->axes));")
                lines.insert(i + 3, f"{indent}memset(pad->buttons, 0, sizeof(pad->buttons));")
                modified = True
                break  # Only fix first occurrence to avoid duplicates
        if modified:
            content = '\n'.join(lines)
            write(path, content)
            return True

    log("[FAIL] could not find disconnect pattern")
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
    print("  322: Controller Disconnect — Zero Input State")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
        print()
    ok = fix_gamepad()
    if ok and not DRY_RUN:
        ok = build_check()
    print()
    print("=" * 66)
    if ok:
        print("  PASS — controller state zeroed on disconnect")
        print("  Next: python3 fixes/323.py")
    else:
        print("  FAILED — see errors above")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
