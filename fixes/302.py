#!/usr/bin/env python3
"""
302: Robot creation rollback
==============================
Makes ftc_robot_create_with_drive() transactional:
1. Pre-checks body capacity before creating anything
2. On failure, marks already-created bodies as static (dead)
   so they don't participate in physics

Usage:
    cd <project_root>
    python3 fixes/302_lifecycle_robot_rollback.py [--dry-run]
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_302"

def log(msg): print(f"  [302] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_302")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_robot_c():
    path = SRC / "robotics" / "robot.c"
    content = read(path)
    if MARKER in content:
        log("[SKIP] already fixed")
        return True

    # 1. Add capacity pre-check after memset
    old1 = """    memset(robot, 0, sizeof(ftc_robot));
    /* memset zeroes odom_x/z/theta and wheel_radians — no separate init needed */
    robot->motor_preset = preset;"""
    new1 = f"""    memset(robot, 0, sizeof(ftc_robot));
    /* {MARKER}_CAPACITY: Pre-check capacity before creating anything.
     * Need 1 chassis + 4 wheels = 5 bodies. */
    if (world->body_count + 5 > world->body_capacity) {{
        return 1;
    }}
    /* memset zeroes odom_x/z/theta and wheel_radians — no separate init needed */
    robot->motor_preset = preset;"""
    if old1 in content:
        content = content.replace(old1, new1, 1)
    else:
        log("[WARN] memset pattern not found")

    # 2. Add rollback on wheel creation failure
    old2 = """        if (robot->wheel_bodies[i] < 0) {
            return 1;
        }"""
    new2 = f"""        if (robot->wheel_bodies[i] < 0) {{
            /* {MARKER}_ROLLBACK: Mark already-created bodies as dead. */
            if (robot->chassis_body >= 0)
                rigidbody_set_static(&world->bodies[robot->chassis_body], true);
            for (int rb_i = 0; rb_i < i; rb_i++) {{
                if (robot->wheel_bodies[rb_i] >= 0)
                    rigidbody_set_static(&world->bodies[robot->wheel_bodies[rb_i]], true);
                if (robot->wheel_joints[rb_i] >= 0)
                    constraint_remove(robot->wheel_joints[rb_i]);
            }}
            memset(robot, 0, sizeof(ftc_robot));
            return 1;
        }}"""
    if old2 in content:
        content = content.replace(old2, new2, 1)
    else:
        log("[WARN] wheel_bodies failure pattern not found")

    # 3. Add rollback on joint creation failure
    old3 = """        if (robot->wheel_joints[i] < 0) {
            return 1;
        }"""
    new3 = f"""        if (robot->wheel_joints[i] < 0) {{
            /* {MARKER}_ROLLBACK: Mark already-created bodies as dead. */
            if (robot->chassis_body >= 0)
                rigidbody_set_static(&world->bodies[robot->chassis_body], true);
            for (int rb_i = 0; rb_i <= i; rb_i++) {{
                if (robot->wheel_bodies[rb_i] >= 0)
                    rigidbody_set_static(&world->bodies[robot->wheel_bodies[rb_i]], true);
                if (robot->wheel_joints[rb_i] >= 0)
                    constraint_remove(robot->wheel_joints[rb_i]);
            }}
            memset(robot, 0, sizeof(ftc_robot));
            return 1;
        }}"""
    if old3 in content:
        content = content.replace(old3, new3, 1)
    else:
        log("[WARN] wheel_joints failure pattern not found")

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
    print("  302: Robot Creation Rollback")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
        print()
    ok = fix_robot_c()
    if ok and not DRY_RUN:
        ok = build_check()
    print()
    print("=" * 66)
    if ok:
        print("  PASS 1c COMPLETE")
        print("  Next: python3 fixes/303_lifecycle_robot_registry.py")
    else:
        print("  FAILED — see errors above")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
