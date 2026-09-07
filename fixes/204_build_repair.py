#!/usr/bin/env python3
"""
204: Build repair
==================
Fixes two build errors:
1. Makefile has extraneous text after 'endif' directive
2. collision_prepare_solver uses 'dt' which is not in scope

Usage:
    cd <project_root>
    python3 fixes/204_build_repair.py [--dry-run]
"""
import sys, shutil, subprocess, re
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"

DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_204"

def log(msg): print(f"  [204] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_204")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_makefile():
    """Fix extraneous text after 'endif' in makefile."""
    log("Fix 1: Makefile extraneous text after endif")
    path = SRC / "makefile"
    if not path.exists():
        log(f"[FAIL] makefile not found at {path}")
        return False

    content = read(path)

    if "endif -lxinput9_1_0" in content:
        content = content.replace("endif -lxinput9_1_0", "endif")
        write(path, content)
        log("[OK] makefile repaired")
        return True
    else:
        log("[SKIP] makefile already clean or pattern not found")
        return True

def fix_r314_dt_parameter():
    """Fix R3-14 by adding dt parameter to collision_prepare_solver."""
    log("Fix 2: Add dt parameter to collision_prepare_solver")

    # Step 1: Update the function definition in collision_mechanics.c
    path = SRC / "physics" / "collision_mechanics.c"
    content = read(path)

    if MARKER + "_DT" in content:
        log("[SKIP] already fixed")
        return True

    # Try multiple patterns for the function definition
    patterns = [
        ("collision_prepare_solver(collision_data *m, collision_data *out) {",
         f"collision_prepare_solver(collision_data *m, collision_data *out, float dt) /* {MARKER}_DT */ {{"),
        ("collision_prepare_solver(collision_data *m, collision_data *out)\n{",
         f"collision_prepare_solver(collision_data *m, collision_data *out, float dt) /* {MARKER}_DT */\n{{"),
        ("collision_prepare_solver(collision_data *m, collision_data *out)",
         f"collision_prepare_solver(collision_data *m, collision_data *out, float dt) /* {MARKER}_DT */"),
    ]

    fixed = False
    for old, new in patterns:
        if old in content:
            content = content.replace(old, new, 1)
            write(path, content)
            log("[OK] function definition updated")
            fixed = True
            break

    if not fixed:
        log("[FAIL] could not find function definition")
        return False

    # Step 2: Update the function declaration in collision_mechanics.h
    header_path = SRC / "physics" / "collision_mechanics.h"
    if header_path.exists():
        header_content = read(header_path)

        decl_patterns = [
            ("void collision_prepare_solver(collision_data *m, collision_data *out);",
             f"void collision_prepare_solver(collision_data *m, collision_data *out, float dt); /* {MARKER}_DT */"),
            ("collision_prepare_solver(collision_data *m, collision_data *out);",
             f"collision_prepare_solver(collision_data *m, collision_data *out, float dt); /* {MARKER}_DT */"),
        ]

        for old, new in decl_patterns:
            if old in header_content:
                header_content = header_content.replace(old, new, 1)
                write(header_path, header_content)
                log("[OK] function declaration updated")
                break
        else:
            log("[WARN] function declaration not found in header")

    # Step 3: Update all callers
    for c_file in SRC.rglob("*.c"):
        if c_file == path:
            continue

        caller_content = read(c_file)

        if "collision_prepare_solver(" not in caller_content:
            continue

        # Find calls to collision_prepare_solver and add dt parameter
        call_pattern = r'collision_prepare_solver\(([^,]+),\s*([^)]+)\);'

        def add_dt(match):
            arg1 = match.group(1).strip()
            arg2 = match.group(2).strip()
            return f'collision_prepare_solver({arg1}, {arg2}, dt); /* {MARKER}_DT */'

        if re.search(call_pattern, caller_content):
            caller_content = re.sub(call_pattern, add_dt, caller_content)
            write(c_file, caller_content)
            log(f"[OK] updated caller: {c_file.name}")
        else:
            log(f"[WARN] no collision_prepare_solver call found in {c_file.name}")

    return True

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
    print("  204: Build Repair")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
    print()

    fixes = [
        fix_makefile,
        fix_r314_dt_parameter,
    ]

    failed = []
    for fix in fixes:
        try:
            if not fix():
                failed.append(fix.__name__)
        except Exception as e:
            log(f"[FAIL] {fix.__name__}: {e}")
            failed.append(fix.__name__)

    if not DRY_RUN and not failed:
        if not build_check():
            failed.append("build")

    print()
    print("=" * 66)
    if failed:
        print(f"  FAILED: {len(failed)} item(s)")
        for f in failed:
            print(f"    - {f}")
    else:
        print("  BUILD REPAIR COMPLETE")
        print("  Next: run fixes/202_robustness_critical.py again to verify")
    print("=" * 66)
    return 1 if failed else 0

if __name__ == "__main__":
    sys.exit(main())
