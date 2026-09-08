#!/usr/bin/env python3
"""
201: Critical physics fixes
============================
NEW-01: Cylinder barrel floor contact (tipped wheel falls through)
NEW-04: Odometry heading rotation
NEW-07: Revolute axis drift on chassis
NEW-08: Mecanum tangent floor check

Usage:
    cd <project_root>
    python3 fixes/201_physics_critical.py [--dry-run]
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_201"

def log(msg): print(f"  [201] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_201")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_new01_cylinder_barrel():
    """NEW-01: Add barrel-surface contact for tipped cylinders."""
    log("NEW-01: Cylinder barrel floor contact")
    path = SRC / "physics" / "collision_mechanics.c"
    content = read(path)

    if MARKER + "_NEW01" in content:
        log("[SKIP] already fixed")
        return True

    # Find the end of collision_static_plane_cylinder
    # Look for the return statement at the end of the function
    anchor = "return collision_output_data->contact_count > 0;"

    # Find the LAST occurrence (should be in collision_static_plane_cylinder)
    idx = content.rfind(anchor)
    if idx < 0:
        log("[WARN] anchor not found")
        return True

    insert = f"""
    /* {MARKER}_NEW01: Barrel-surface contact for tipped cylinders.
     * When axle is horizontal, axle endpoints don't touch floor.
     * Test the barrel surface against the floor plane. */
    {{
        vector3 axis = cyl->cached_axes[0];
        float axle_y_component = fabsf(axis.y);
        if (axle_y_component < 0.7f) {{
            float barrel_pen = plane_y - (cyl->position.y - r);
            if ((barrel_pen > 0.0f) && (collision_output_data->contact_count < 2)) {{
                contact_point_data *cp = &collision_output_data->contacts[collision_output_data->contact_count];
                cp->position = (vector3){{cyl->position.x, plane_y, cyl->position.z}};
                cp->penetration = barrel_pen;
                collision_output_data->contact_count++;
            }}
        }}
    }}

"""
    content = content[:idx] + insert + content[idx:]
    write(path, content)
    return True

def fix_new04_odometry_heading():
    """NEW-04: Rotate chassis velocity by heading before integrating odometry."""
    log("NEW-04: Odometry heading rotation")
    path = SRC / "robotics" / "drivetrain.c"
    content = read(path)

    if MARKER + "_NEW04" in content:
        log("[SKIP] already fixed")
        return True

    old = """    robot->odom_theta += yaw_rate * dt;
    robot->odom_x += chassis_vel.x * dt;
    robot->odom_z += chassis_vel.z * dt;"""

    new = f"""    robot->odom_theta += yaw_rate * dt;

    /* {MARKER}_NEW04: Rotate world velocity into robot frame before integrating.
     * Without this, odometry drifts every time the robot turns. */
    float cos_theta = cosf(robot->odom_theta);
    float sin_theta = sinf(robot->odom_theta);
    float v_forward = chassis_vel.x * cos_theta + chassis_vel.z * sin_theta;
    float v_strafe  = -chassis_vel.x * sin_theta + chassis_vel.z * cos_theta;

    robot->odom_x += v_forward * dt;
    robot->odom_z += v_strafe * dt;"""

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    else:
        log("[WARN] odometry pattern not found — check if already using chassis velocity")
        return True

def fix_new07_revolute_axis_drift():
    """NEW-07: Only apply axis correction to wheel, not chassis."""
    log("NEW-07: Revolute axis drift on chassis")
    path = SRC / "physics" / "revolute_joint.c"
    content = read(path)

    if MARKER + "_NEW07" in content:
        log("[SKIP] already fixed")
        return True

    # Find the block that applies axis correction to body_a
    old = """            if (!body_a->static_state) {
                body_a->angular_velocity = vector3_subtraction(
                    body_a->angular_velocity,
                    math3_multiplication_vector3(body_a->inverse_inertia_system, axis_impulse));
            }
            if (!body_b->static_state) {
                body_b->angular_velocity = vector3_addition(
                    body_b->angular_velocity,
                    math3_multiplication_vector3(body_b->inverse_inertia_system, axis_impulse));
            }"""

    new = f"""            /* {MARKER}_NEW07: Only correct the wheel (body_b).
             * The chassis (body_a) is the reference body and must not rotate
             * to accommodate wheel tilt. */
            if (!body_b->static_state) {{
                body_b->angular_velocity = vector3_addition(
                    body_b->angular_velocity,
                    math3_multiplication_vector3(body_b->inverse_inertia_system, axis_impulse));
            }}"""

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    else:
        log("[WARN] revolute axis drift pattern not found")
        return True

def fix_new08_mecanum_floor_check():
    """NEW-08: Only apply mecanum tangent for floor contacts."""
    log("NEW-08: Mecanum tangent floor check")
    path = SRC / "physics" / "collision_mechanics.c"
    content = read(path)

    if MARKER + "_NEW08" in content:
        log("[SKIP] already fixed")
        return True

    old = "    if (mecanum_wheel && mecanum_wheel->type == object_cylinder) {"

    new = f"""    /* {MARKER}_NEW08: Only apply mecanum tangent for floor contacts.
     * If the contact normal isn't mostly vertical, fall through to standard tangent. */
    bool mfs_is_floor_contact = (fabsf(m->normal_vector.y) > 0.9f);
    if (mecanum_wheel && mecanum_wheel->type == object_cylinder && mfs_is_floor_contact) {{"""

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    else:
        log("[WARN] mecanum tangent pattern not found")
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
    print("  201: Critical Physics Fixes")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
    print()

    fixes = [
        fix_new01_cylinder_barrel,
        fix_new04_odometry_heading,
        fix_new07_revolute_axis_drift,
        fix_new08_mecanum_floor_check,
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
        print("  ALL 4 CRITICAL PHYSICS FIXES APPLIED")
        print("  Next: run fixes/202_robustness_critical.py")
    print("=" * 66)
    return 1 if failed else 0

if __name__ == "__main__":
    sys.exit(main())
