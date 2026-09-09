#!/usr/bin/env python3
"""
301: Constraint world-generation tag
=====================================
Adds a generation counter to the constraint pool so that constraints
created for a destroyed world cannot match bodies in a new world.

Changes:
  constraint.h  — add uint32_t world_generation to constraint struct
  constraint.c  — add generation counter, increment on pool_init,
                  set on add, check on dispatch

Usage:
    cd <project_root>
    python3 fixes/301_lifecycle_constraint_tag.py [--dry-run]
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_301"

def log(msg): print(f"  [301] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_301")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_constraint_h():
    path = SRC / "physics" / "constraint.h"
    content = read(path)
    if MARKER in content:
        log("[SKIP] constraint.h already fixed")
        return True
    old = """    bool is_active;
    union {"""
    new = f"""    bool is_active;
    uint32_t world_generation; /* {MARKER}: generation tag for world ownership */
    union {{"""
    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    log("[FAIL] constraint struct pattern not found in constraint.h")
    return False

def fix_constraint_c():
    path = SRC / "physics" / "constraint.c"
    content = read(path)
    if MARKER in content:
        log("[SKIP] constraint.c already fixed")
        return True
    ok = True

    # 1. Add generation counter after constraint_count
    old1 = "static int constraint_count = 0;"
    new1 = f"""static int constraint_count = 0;
static uint32_t constraint_world_generation = 0; /* {MARKER} */"""
    if old1 in content:
        content = content.replace(old1, new1, 1)
    else:
        log("[WARN] constraint_count declaration not found")
        ok = False

    # 2. Increment generation in constraint_pool_init
    old2 = """void constraint_pool_init (void) {
    for (int i = 0; i < mpe_max_joints; i++) { constraint_pool [i].is_active = false; }
    constraint_count = 0;
}"""
    new2 = f"""void constraint_pool_init (void) {{
    constraint_world_generation++; /* {MARKER}: new generation on every pool reset */
    for (int i = 0; i < mpe_max_joints; i++) {{ constraint_pool [i].is_active = false; }}
    constraint_count = 0;
}}"""
    if old2 in content:
        content = content.replace(old2, new2, 1)
    else:
        log("[WARN] constraint_pool_init pattern not found")
        ok = False

    # 3. Set generation on add
    old3 = """        constraint_pool [i].is_active = true;
        constraint_count++;
        return i;"""
    new3 = f"""        constraint_pool [i].is_active = true;
        constraint_pool [i].world_generation = constraint_world_generation; /* {MARKER} */
        constraint_count++;
        return i;"""
    if old3 in content:
        content = content.replace(old3, new3, 1)
    else:
        log("[WARN] constraint_add_revolute is_active pattern not found")
        ok = False

    # 4. Check generation in dispatch
    old4 = """        if (!constraint_pool [i].is_active) { continue; }"""
    new4 = f"""        if (!constraint_pool [i].is_active) {{ continue; }}
        if (constraint_pool [i].world_generation != constraint_world_generation) {{ continue; }} /* {MARKER} */"""
    if old4 in content:
        content = content.replace(old4, new4, 1)
    else:
        log("[WARN] constraint_dispatch is_active check not found")
        ok = False

    if ok or True:  # write even if some patterns missed
        write(path, content)
    return ok

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
    print("  301: Constraint World-Generation Tag")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
        print()
    ok = True
    ok = fix_constraint_h() and ok
    ok = fix_constraint_c() and ok
    if ok and not DRY_RUN:
        ok = build_check()
    print()
    print("=" * 66)
    if ok:
        print("  PASS 1b COMPLETE")
        print("  Next: python3 fixes/302_lifecycle_robot_rollback.py")
    else:
        print("  FAILED — see errors above")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
