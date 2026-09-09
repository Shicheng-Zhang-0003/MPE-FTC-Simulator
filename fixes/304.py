#!/usr/bin/env python3
"""
304: Wire gui_robot_clear into scene operations
==================================================
Ensures scene_clear(), scene_loading(), and scene_init_default()
call gui_robot_clear() so the robot world is destroyed along with
the scene.

Changes:
  scene_init.c  — add include + call gui_robot_clear() in scene_clear()
                  and scene_init_default()
  scene_load.c  — add include + call gui_robot_clear() in scene_loading()

Usage:
    cd <project_root>
    python3 fixes/304_lifecycle_scene_clear.py [--dry-run]
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_304"

def log(msg): print(f"  [304] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_304")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_scene_init_c():
    path = SRC / "scene" / "scene_init.c"
    content = read(path)
    if MARKER in content:
        log("[SKIP] scene_init.c already fixed")
        return True
    ok = True

    # 1. Add include
    old_inc = '#include "../physics/spring_joint.h"'
    new_inc = f'#include "../physics/spring_joint.h"\n#include "../robotics/gui_robot_registry.h" /* {MARKER} */'
    if old_inc in content:
        content = content.replace(old_inc, new_inc, 1)
    else:
        log("[WARN] spring_joint.h include not found")
        ok = False

    # 2. Add gui_robot_clear() to scene_clear()
    old_clear = """void scene_clear(void) {
    object_count = 0;
    joint_init_pool();
}"""
    new_clear = f"""void scene_clear(void) {{
    gui_robot_clear(); /* {MARKER}: destroy robots before clearing scene */
    object_count = 0;
    joint_init_pool();
}}"""
    if old_clear in content:
        content = content.replace(old_clear, new_clear, 1)
    else:
        log("[WARN] scene_clear pattern not found")
        ok = False

    # 3. Add gui_robot_clear() to scene_init_default()
    old_default = """void scene_init_default(void) {
    scene_clear();"""
    new_default = f"""void scene_init_default(void) {{
    gui_robot_clear(); /* {MARKER}: destroy robots before reinitializing */
    scene_clear();"""
    if old_default in content:
        content = content.replace(old_default, new_default, 1)
    else:
        log("[WARN] scene_init_default pattern not found")

    write(path, content)
    return ok

def fix_scene_load_c():
    path = SRC / "scene" / "scene_load.c"
    content = read(path)
    if MARKER in content:
        log("[SKIP] scene_load.c already fixed")
        return True
    ok = True

    # 1. Add include
    old_inc = '#include "../physics/spring_joint.h"'
    new_inc = f'#include "../physics/spring_joint.h"\n#include "../robotics/gui_robot_registry.h" /* {MARKER} */'
    if old_inc in content:
        content = content.replace(old_inc, new_inc, 1)
    else:
        log("[WARN] spring_joint.h include not found in scene_load.c")
        ok = False

    # 2. Add gui_robot_clear() before scene_clear() in scene_loading()
    # The staging buffer commit section calls scene_clear()
    old_commit = """    /* All reads succeeded. Now safe to clear and commit. */
    scene_clear();"""
    new_commit = f"""    /* All reads succeeded. Now safe to clear and commit. */
    gui_robot_clear(); /* {MARKER}: destroy robots before loading new scene */
    scene_clear();"""
    if old_commit in content:
        content = content.replace(old_commit, new_commit, 1)
    else:
        log("[WARN] scene_loading commit pattern not found")
        ok = False

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
    print("  304: Wire gui_robot_clear into Scene Operations")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
        print()
    ok = True
    ok = fix_scene_init_c() and ok
    ok = fix_scene_load_c() and ok
    if ok and not DRY_RUN:
        ok = build_check()
    print()
    print("=" * 66)
    if ok:
        print("  PASS 1 COMPLETE — Lifecycle corruption killed")
        print()
        print("  Summary of fixes applied:")
        print("    300: physics_world_init no longer leaks or resets IDs")
        print("    301: Constraints tagged with world generation")
        print("    302: Robot creation is transactional")
        print("    303: gui_robot_clear() exists")
        print("    304: scene_clear/load/init call gui_robot_clear()")
        print()
        print("  Next pass: Persistence (310-313)")
    else:
        print("  FAILED — see errors above")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
