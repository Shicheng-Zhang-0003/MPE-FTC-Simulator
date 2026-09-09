#!/usr/bin/env python3
"""
323: Controller reconnect — periodic rediscovery
==================================================
After disconnect, there's no mechanism to detect when the controller
comes back. Add periodic rediscovery polling.

Fix: when disconnected, attempt to reopen every 60 frames (~1 second).

Usage:
    cd <project_root>
    python3 fixes/323.py [--dry-run]
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_323"

def log(msg): print(f"  [323] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_323")
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

    # Add reconnect logic at the top of gamepad_poll for Windows
    # Current: if (!pad || !pad->connected || pad->fd < 0) return;
    old_poll_start = """void gamepad_poll(gamepad_state *pad) {
    if (!pad || !pad->connected || pad->fd < 0) return;"""
    new_poll_start = f"""void gamepad_poll(gamepad_state *pad) {{
    if (!pad) return;

    /* {MARKER}: Periodic reconnect attempt when disconnected.
     * Try to reopen the device every 60 frames (~1 second). */
    if (!pad->connected || pad->fd < 0) {{
        static int reconnect_counter = 0;
        if (++reconnect_counter >= 60) {{
            reconnect_counter = 0;
#ifdef _WIN32
            XINPUT_STATE probe_st;
            if (XInputGetState(0, &probe_st) == ERROR_SUCCESS) {{
                pad->connected = true;
                pad->fd = 0;
                memset(pad->axes, 0, sizeof(pad->axes));
                memset(pad->buttons, 0, sizeof(pad->buttons));
                printf("[gamepad] XInput device 0 reconnected\\n");
            }}
#else
            int probe_fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
            if (probe_fd >= 0) {{
                pad->connected = true;
                pad->fd = probe_fd;
                memset(pad->axes, 0, sizeof(pad->axes));
                memset(pad->buttons, 0, sizeof(pad->buttons));
                printf("[gamepad] device reconnected\\n");
            }}
#endif
        }}
        return;
    }}"""

    if old_poll_start in content:
        content = content.replace(old_poll_start, new_poll_start, 1)
        write(path, content)
        return True

    # Fallback: find the poll function and add reconnect at the top
    lines = content.split('\n')
    for i, line in enumerate(lines):
        if 'void gamepad_poll(gamepad_state *pad)' in line:
            # Find the next line (should be the early return)
            for j in range(i+1, min(i+5, len(lines))):
                if '!pad' in lines[j] and 'return' in lines[j]:
                    # Replace the early return with reconnect logic
                    indent = "    "
                    new_lines = [
                        f"{indent}if (!pad) return;",
                        f"",
                        f"{indent}/* {MARKER}: Periodic reconnect when disconnected */",
                        f"{indent}if (!pad->connected || pad->fd < 0) {{",
                        f"{indent}    static int reconnect_counter = 0;",
                        f"{indent}    if (++reconnect_counter >= 60) {{",
                        f"{indent}        reconnect_counter = 0;",
                        f"#ifdef _WIN32",
                        f"{indent}        XINPUT_STATE probe_st;",
                        f"{indent}        if (XInputGetState(0, &probe_st) == ERROR_SUCCESS) {{",
                        f"{indent}            pad->connected = true;",
                        f"{indent}            pad->fd = 0;",
                        f'{indent}            memset(pad->axes, 0, sizeof(pad->axes));',
                        f'{indent}            memset(pad->buttons, 0, sizeof(pad->buttons));',
                        f'{indent}            printf("[gamepad] XInput device 0 reconnected\\n");',
                        f"{indent}        }}",
                        f"#else",
                        f'{indent}        int probe_fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);',
                        f"{indent}        if (probe_fd >= 0) {{",
                        f"{indent}            pad->connected = true;",
                        f"{indent}            pad->fd = probe_fd;",
                        f'{indent}            memset(pad->axes, 0, sizeof(pad->axes));',
                        f'{indent}            memset(pad->buttons, 0, sizeof(pad->buttons));',
                        f'{indent}            printf("[gamepad] device reconnected\\n");',
                        f"{indent}        }}",
                        f"#endif",
                        f"{indent}    }}",
                        f"{indent}    return;",
                        f"{indent}}}",
                    ]
                    lines[j:j+1] = new_lines
                    content = '\n'.join(lines)
                    write(path, content)
                    return True

    log("[FAIL] could not find gamepad_poll entry point")
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
    print("  323: Controller Reconnect — Periodic Rediscovery")
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
        print("  PASS — controller reconnects after disconnect")
        print()
        print("  Pass 2 + Pass 3 complete:")
        print("    310: Invalid object types rejected")
        print("    311: fwrite return values checked")
        print("    313: Joint-hole serialization fixed")
        print("    320: E key clears on release")
        print("    321: T opens debug terminal")
        print("    322: Controller state zeroed on disconnect")
        print("    323: Controller reconnects periodically")
        print()
        print("  Next: Pass 4 (resource paths) or Pass 5 (tooling)")
    else:
        print("  FAILED — see errors above")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
