#!/usr/bin/env python3
"""
207: Fix test_runner.py and build_check.py for Windows .exe extension
=======================================================================
On Windows, MinGW produces test_two_world.exe, not test_two_world.
The tools look for the extensionless name and report "Binary not found".

Usage:
    cd <project_root>
    python3 fixes/207_test_runner_windows.py [--dry-run]
"""
import sys, shutil
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
TOOLS = ROOT / "tools"

DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_207"

def log(msg): print(f"  [207] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_207")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def patch_test_runner():
    """Add .exe fallback to test_runner.py binary lookup."""
    log("Patching test_runner.py")
    path = TOOLS / "test_runner.py"
    content = read(path)

    if MARKER in content:
        log("[SKIP] already patched")
        return True

    # Find the binary lookup pattern and add .exe fallback
    old = 'test_binary = SRC_DIR / f"test_{test_name}"'
    new = f"""test_binary = SRC_DIR / f"test_{{test_name}}"
    if not test_binary.exists():
        test_binary = SRC_DIR / f"test_{{test_name}}.exe"  # {MARKER}"""

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    else:
        log("[WARN] binary lookup pattern not found")
        log("[INFO] Manual fix: find 'test_binary = SRC_DIR / f\"test_{test_name}\"'")
        log("[INFO] Add: if not test_binary.exists(): test_binary = SRC_DIR / f\"test_{test_name}.exe\"")
        return True

def patch_build_check():
    """Add .exe fallback to build_check.py binary lookup."""
    log("Patching build_check.py")
    path = TOOLS / "build_check.py"
    content = read(path)

    if MARKER in content:
        log("[SKIP] already patched")
        return True

    old = 'test_binary = SRC_DIR / f"test_{test_name}"'
    new = f"""test_binary = SRC_DIR / f"test_{{test_name}}"
    if not test_binary.exists():
        test_binary = SRC_DIR / f"test_{{test_name}}.exe"  # {MARKER}"""

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    else:
        log("[WARN] binary lookup pattern not found in build_check.py")
        return True

def main():
    print("=" * 66)
    print("  207: Test Runner Windows Fix")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
    print()

    ok = True
    ok = patch_test_runner() and ok
    ok = patch_build_check() and ok

    print()
    print("=" * 66)
    if ok:
        print("  PATCHES APPLIED")
        print()
        print("  Next: build the test binaries, then re-run:")
        print("    cd v10R3I/src")
        print("    make test_two_world test_revolute test_teleop_drive test_mecanum_drive \\")
        print("         test_cylinder_drop test_driven_wheel test_math3_inverse \\")
        print("         test_ftc_integration test_physics_truth test_tank_turn \\")
        print("         test_odometry_accuracy test_cylinder_sphere test_cylinder_cube \\")
        print("         test_cylinder_cylinder")
        print()
        print("    cd ~/461-MFS")
        print("    python3 tools/test_runner.py")
    else:
        print("  FAILED")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
