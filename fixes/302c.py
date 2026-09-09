#!/usr/bin/env python3
"""
302c: Robot creation rollback — diagnostic + fix
=================================================
First diagnoses what's actually in robot.c, then applies fixes.
Uses line-based matching instead of exact multi-line strings.
"""
import sys, shutil, subprocess, re
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_302C"

def log(msg): print(f"  [302c] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_302c")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def find_line(lines, pattern, start=0):
    """Find first line containing pattern, return index or -1."""
    for i in range(start, len(lines)):
        if pattern in lines[i]:
            return i
    return -1

def fix_robot():
    path = SRC / "robotics" / "robot.c"
    content = read(path)

    if MARKER in content:
        log("[SKIP] already fixed")
        return True

    lines = content.split('\n')
    ok = True

    # --- Fix 1: Capacity pre-check after memset ---
    memset_idx = find_line(lines, "memset(robot, 0, sizeof(ftc_robot));")
    if memset_idx >= 0:
        log(f"[FOUND] memset at line {memset_idx + 1}")
        # Insert capacity check right after the memset line
        # Find the next non-comment, non-empty line after memset
        insert_idx = memset_idx + 1
        # Skip comment lines
        while insert_idx < len(lines) and (lines[insert_idx].strip().startswith("/*") or lines[insert_idx].strip() == ""):
            insert_idx += 1
        capacity_lines = [
            f"/* {MARKER}_CAPACITY: Pre-check capacity before creating anything. */",
            "if (world->body_count + 5 > world->body_capacity) {",
            "return 1;",
            "}",
        ]
        for j, cl in enumerate(capacity_lines):
            lines.insert(insert_idx + j, cl)
        log("[OK] capacity pre-check inserted")
    else:
        log("[WARN] memset pattern not found")
        ok = False

    # Re-find after insertion (indices shifted)
    # --- Fix 2: wheel_bodies failure rollback ---
    wb_idx = find_line(lines, "if (robot->wheel_bodies[i] < 0) {")
    if wb_idx >= 0:
        log(f"[FOUND] wheel_bodies check at line {wb_idx + 1}")
        # Find the "return 1;" and "}" that follow
        ret_idx = find_line(lines, "return 1;", wb_idx + 1)
        close_idx = find_line(lines, "}", ret_idx + 1) if ret_idx >= 0 else -1
        if ret_idx >= 0 and close_idx >= 0:
            # Replace "return 1;" with rollback code
            rollback_lines = [
                f"/* {MARKER}_ROLLBACK: Mark already-created bodies as dead. */",
                "if (robot->chassis_body >= 0)",
                "rigidbody_set_static(&world->bodies[robot->chassis_body], true);",
                "for (int rb_i = 0; rb_i < i; rb_i++) {",
                "if (robot->wheel_bodies[rb_i] >= 0)",
                "rigidbody_set_static(&world->bodies[robot->wheel_bodies[rb_i]], true);",
                "if (robot->wheel_joints[rb_i] >= 0)",
                "constraint_remove(robot->wheel_joints[rb_i]);",
                "}",
                "memset(robot, 0, sizeof(ftc_robot));",
                "return 1;",
            ]
            # Replace the single "return 1;" line with rollback
            lines[ret_idx:ret_idx + 1] = rollback_lines
            log("[OK] wheel_bodies rollback inserted")
        else:
            log("[WARN] could not find return/close after wheel_bodies check")
            ok = False
    else:
        log("[WARN] wheel_bodies failure pattern not found")
        ok = False

    # --- Fix 3: wheel_joints failure rollback ---
    wj_idx = find_line(lines, "if (robot->wheel_joints[i] < 0) {")
    if wj_idx >= 0:
        log(f"[FOUND] wheel_joints check at line {wj_idx + 1}")
        ret_idx = find_line(lines, "return 1;", wj_idx + 1)
        close_idx = find_line(lines, "}", ret_idx + 1) if ret_idx >= 0 else -1
        if ret_idx >= 0 and close_idx >= 0:
            rollback_lines = [
                f"/* {MARKER}_ROLLBACK: Mark already-created bodies as dead. */",
                "if (robot->chassis_body >= 0)",
                "rigidbody_set_static(&world->bodies[robot->chassis_body], true);",
                "for (int rb_i = 0; rb_i <= i; rb_i++) {",
                "if (robot->wheel_bodies[rb_i] >= 0)",
                "rigidbody_set_static(&world->bodies[robot->wheel_bodies[rb_i]], true);",
                "if (robot->wheel_joints[rb_i] >= 0)",
                "constraint_remove(robot->wheel_joints[rb_i]);",
                "}",
                "memset(robot, 0, sizeof(ftc_robot));",
                "return 1;",
            ]
            lines[ret_idx:ret_idx + 1] = rollback_lines
            log("[OK] wheel_joints rollback inserted")
        else:
            log("[WARN] could not find return/close after wheel_joints check")
            ok = False
    else:
        log("[WARN] wheel_joints failure pattern not found")
        ok = False

    new_content = '\n'.join(lines)
    write(path, new_content)
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
    print("  302c: Robot Creation Rollback (line-based matching)")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
        print()
    ok = fix_robot()
    if ok and not DRY_RUN:
        ok = build_check()
    print()
    if ok:
        print("  PASS — robot creation is now transactional")
    else:
        print("  PARTIAL — some patterns not found, check output above")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
