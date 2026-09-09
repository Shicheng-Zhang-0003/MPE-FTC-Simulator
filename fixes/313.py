#!/usr/bin/env python3
"""
313: Fix joint-hole serialization in scene save
=================================================
The save loop iterates 0..current_joint_count, but joint removal
creates holes while decrementing current_joint_count. Joints at
indices >= current_joint_count are silently skipped.

Fix: iterate over ALL mpe_max_joints entries, only count/write active ones.

Usage:
    cd <project_root>
    python3 fixes/313.py [--dry-run]
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_313"

def log(msg): print(f"  [313] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_313")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_scene_saving():
    path = SRC / "scene" / "scene_saving.c"
    content = read(path)
    if MARKER in content:
        log("[SKIP] already fixed")
        return True

    # Fix the active_joints counting loop
    old_count = """    int active_joints = 0;
    for (int j = 0; j < current_joint_count; j++) {
        if (joint_pool[j].is_active) {
            active_joints++;
        }
    }"""
    new_count = f"""    /* {MARKER}: Iterate ALL joint slots, not just current_joint_count.
     * Joint removal creates holes while decrementing current_joint_count,
     * so active joints can exist at indices >= current_joint_count. */
    int active_joints = 0;
    for (int j = 0; j < mpe_max_joints; j++) {{
        if (joint_pool[j].is_active) {{
            active_joints++;
        }}
    }}"""

    if old_count in content:
        content = content.replace(old_count, new_count, 1)
    else:
        log("[WARN] joint counting loop pattern not found")
        return False

    # Fix the serialization loop
    old_write = """    for (int j = 0; j < current_joint_count; j++) {
        if (joint_pool[j].is_active) {"""
    new_write = f"""    /* {MARKER}: Write all active joints from full pool. */
    for (int j = 0; j < mpe_max_joints; j++) {{
        if (joint_pool[j].is_active) {{"""

    if old_write in content:
        content = content.replace(old_write, new_write, 1)
    else:
        log("[WARN] joint write loop pattern not found")
        return False

    write(path, content)
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
    print("  313: Fix Joint-Hole Serialization")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
        print()
    ok = fix_scene_saving()
    if ok and not DRY_RUN:
        ok = build_check()
    print()
    print("=" * 66)
    if ok:
        print("  PASS — joint serialization now covers full pool")
        print("  Next: python3 fixes/320.py")
    else:
        print("  FAILED — see errors above")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
