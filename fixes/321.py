#!/usr/bin/env python3
"""
321: Fix T key — wire T to open debug terminal
================================================
Documentation says T opens the terminal, but the implementation
wires it to key 1. Wire T to actually open the terminal in debug mode.

Usage:
    cd <project_root>
    python3 fixes/321.py [--dry-run]
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_321"

def log(msg): print(f"  [301] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_321")
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

    # Find where t_key_pressed is set in on_keypress
    # Currently: if ((event -> keyval == GDK_KEY_t) || (event -> keyval == GDK_KEY_T)) {input_state -> t_key_pressed = true;}
    # We need to also trigger debug_terminal_pressed when T is pressed in debug mode.
    # The t_key_pressed field exists but isn't consumed anywhere useful.
    # Wire it to open the terminal.

    # Find the T key press handler
    old_t = """    if ((event -> keyval == GDK_KEY_t) || (event -> keyval == GDK_KEY_T)) {input_state -> t_key_pressed = true;}
    /* MPE_TASK_21_KEYBOARD_ONLY_KEYPRESS_END */"""
    new_t = f"""    if ((event -> keyval == GDK_KEY_t) || (event -> keyval == GDK_KEY_T)) {{
        input_state -> t_key_pressed = true;
        /* {MARKER}: T opens the debug terminal (matching documentation) */
        if (input_state -> is_debug_mode_active) {{
            input_state -> debug_terminal_pressed = true;
        }}
    }}
    /* MPE_TASK_21_KEYBOARD_ONLY_KEYPRESS_END */"""

    if old_t in content:
        content = content.replace(old_t, new_t, 1)
        write(path, content)
        return True

    # Fallback: line-based
    lines = content.split('\n')
    for i, line in enumerate(lines):
        if 'GDK_KEY_t' in line and 't_key_pressed = true' in line:
            # Replace this line with the expanded version
            indent = "    "
            new_lines = [
                f"{indent}if ((event -> keyval == GDK_KEY_t) || (event -> keyval == GDK_KEY_T)) {{",
                f"{indent}    input_state -> t_key_pressed = true;",
                f"{indent}    /* {MARKER}: T opens the debug terminal */",
                f"{indent}    if (input_state -> is_debug_mode_active) {{",
                f"{indent}        input_state -> debug_terminal_pressed = true;",
                f"{indent}    }}",
                f"{indent}}}",
            ]
            lines[i:i+1] = new_lines
            content = '\n'.join(lines)
            write(path, content)
            return True

    log("[FAIL] could not find T key handler")
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
    print("  321: Fix T Key — Open Debug Terminal")
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
        print("  PASS — T now opens the debug terminal")
        print("  Next: python3 fixes/322.py")
    else:
        print("  FAILED — see errors above")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
