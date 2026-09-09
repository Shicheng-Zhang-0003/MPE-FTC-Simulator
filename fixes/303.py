#!/usr/bin/env python3
"""
303: Robot registry clear/reset
=================================
Adds gui_robot_clear() to gui_robot_registry so that scene_clear(),
scene_loading(), and reboot can actually destroy robots.

Changes:
  gui_robot_registry.h — add gui_robot_clear() declaration
  gui_robot_registry.c — implement gui_robot_clear()

Usage:
    cd <project_root>
    python3 fixes/303_lifecycle_robot_registry.py [--dry-run]
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_303"

def log(msg): print(f"  [303] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_303")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_registry_h():
    path = SRC / "robotics" / "gui_robot_registry.h"
    content = read(path)
    if MARKER in content:
        log("[SKIP] gui_robot_registry.h already fixed")
        return True
    old = """/* Query */
int gui_robot_get_count(void);
ftc_robot *gui_robot_get(int index);"""
    new = f"""/* {MARKER}: Clear all robots, their physics bodies, constraints, and proxies. */
void gui_robot_clear(void);

/* Query */
int gui_robot_get_count(void);
ftc_robot *gui_robot_get(int index);"""
    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    log("[FAIL] query section pattern not found in gui_robot_registry.h")
    return False

def fix_registry_c():
    path = SRC / "robotics" / "gui_robot_registry.c"
    content = read(path)
    if MARKER in content:
        log("[SKIP] gui_robot_registry.c already fixed")
        return True

    # Add gui_robot_clear() before gui_robot_get_count
    old = """int gui_robot_get_count(void) {"""
    new = f"""/* {MARKER}: Clear all robots and their associated state.
 * Marks physics bodies as dead, removes constraints, zeroes the registry.
 * Proxy objects remain in obj_per_scene as static (they won't move or
 * participate in physics). Full proxy removal is deferred to a later pass. */
void gui_robot_clear(void) {{
    if (!mfs_gui_robot_world) {{
        mfs_gui_robot_count = 0;
        return;
    }}
    for (int i = 0; i < mfs_gui_robot_count; i++) {{
        ftc_robot *robot = &mfs_gui_robots[i];
        /* Mark chassis as dead */
        if ((robot->chassis_body >= 0) &&
            (robot->chassis_body < mfs_gui_robot_world->body_count)) {{
            rigidbody_set_static(&mfs_gui_robot_world->bodies[robot->chassis_body], true);
        }}
        /* Mark wheels as dead and remove joints */
        for (int w = 0; w < robot->wheel_count; w++) {{
            int wi = robot->wheel_bodies[w];
            if ((wi >= 0) && (wi < mfs_gui_robot_world->body_count)) {{
                rigidbody_set_static(&mfs_gui_robot_world->bodies[wi], true);
            }}
            if (robot->wheel_joints[w] >= 0) {{
                constraint_remove(robot->wheel_joints[w]);
            }}
        }}
        /* Zero the robot struct */
        memset(robot, 0, sizeof(ftc_robot));
        /* Zero the proxy struct */
        memset(&mfs_gui_proxies[i], 0, sizeof(gui_robot_proxy));
        mfs_gui_proxies[i].chassis_proxy = -1;
        mfs_gui_proxies[i].nose_proxy = -1;
        for (int w = 0; w < FTC_MAX_WHEELS; w++) {{
            mfs_gui_proxies[i].wheel_proxies[w] = -1;
        }}
    }}
    mfs_gui_robot_count = 0;
}}

int gui_robot_get_count(void) {{"""
    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    log("[FAIL] gui_robot_get_count pattern not found")
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
    print("  303: Robot Registry Clear/Reset")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
        print()
    ok = True
    ok = fix_registry_h() and ok
    ok = fix_registry_c() and ok
    if ok and not DRY_RUN:
        ok = build_check()
    print()
    print("=" * 66)
    if ok:
        print("  PASS 1d COMPLETE")
        print("  Next: python3 fixes/304_lifecycle_scene_clear.py")
    else:
        print("  FAILED — see errors above")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
