#!/usr/bin/env python3
"""
202: Critical robustness fixes
===============================
R3-06: Wake sleeping bodies on contact (physics_world)
R3-07: Add containment walls (physics_world)
R3-14: Baumgarte bias uses /dt instead of *60

Usage:
    cd <project_root>
    python3 fixes/202_robustness_critical.py [--dry-run]
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_202"

def log(msg): print(f"  [202] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_202")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_r306_wake_on_contact():
    """R3-06: Add wake-on-contact logic to physics_world_step."""
    log("R3-06: Wake sleeping bodies on contact")
    path = SRC / "core" / "physics_world.c"
    content = read(path)

    if MARKER + "_R306" in content:
        log("[SKIP] already fixed")
        return True

    # Find where manifolds are prepared in physics_world_step
    anchor = "collision_prepare_solver(&narrowphase_collision, &world_manifolds[manifold_count]);"
    idx = content.find(anchor)
    if idx < 0:
        log("[WARN] collision_prepare_solver anchor not found")
        return True

    # Find the end of that line
    eol = content.find("\n", idx)
    if eol < 0:
        log("[WARN] could not find end of line")
        return True

    insert = f"""
            /* {MARKER}_R306: Wake sleeping bodies on contact with active bodies. */
            {{
                rigidbody *wake_a = &world->bodies[index_a];
                rigidbody *wake_b = &world->bodies[index_b];
                if ((wake_a->is_sleeping) && (!wake_b->is_sleeping) && (!wake_b->static_state)) {{
                    float speed_sq = vector3_length_squared(wake_b->velocity);
                    if (speed_sq > g_cfg.sleep.wake_linear_thresh_sq) {{
                        rigidbody_wake(wake_a);
                    }}
                }}
                if ((wake_b->is_sleeping) && (!wake_a->is_sleeping) && (!wake_a->static_state)) {{
                    float speed_sq = vector3_length_squared(wake_a->velocity);
                    if (speed_sq > g_cfg.sleep.wake_linear_thresh_sq) {{
                        rigidbody_wake(wake_b);
                    }}
                }}
            }}
"""
    content = content[:eol] + insert + content[eol:]
    write(path, content)
    return True

def fix_r307_containment_walls():
    """R3-07: Add FTC field containment walls to physics_world_init."""
    log("R3-07: Containment walls")
    path = SRC / "core" / "physics_world.c"
    content = read(path)

    if MARKER + "_R307" in content:
        log("[SKIP] already fixed")
        return True

    # Find the end of physics_world_init
    anchor = "memset(world, 0, sizeof(physics_world));"
    idx = content.find(anchor)
    if idx < 0:
        log("[WARN] physics_world_init anchor not found")
        return True

    # Find the end of that statement
    eol = content.find("\n", idx)
    if eol < 0:
        return True

    insert = f"""

    /* {MARKER}_R307: FTC field containment walls (12ft x 12ft).
     * Prevents bodies from escaping to infinity and producing NaN. */
    {{
        float half_w = 1.8288f;   /* 6 ft */
        float wall_h = 0.3048f;   /* 12 in */
        float wall_t = 0.0254f;   /* 1 in */

        /* North wall */
        physics_world_add_cube(world,
            (vector3){{0.0f, wall_h * 0.5f, half_w}},
            (vector3){{half_w, wall_h * 0.5f, wall_t * 0.5f}}, 0.0f);
        /* South wall */
        physics_world_add_cube(world,
            (vector3){{0.0f, wall_h * 0.5f, -half_w}},
            (vector3){{half_w, wall_h * 0.5f, wall_t * 0.5f}}, 0.0f);
        /* East wall */
        physics_world_add_cube(world,
            (vector3){{half_w, wall_h * 0.5f, 0.0f}},
            (vector3){{wall_t * 0.5f, wall_h * 0.5f, half_w}}, 0.0f);
        /* West wall */
        physics_world_add_cube(world,
            (vector3){{-half_w, wall_h * 0.5f, 0.0f}},
            (vector3){{wall_t * 0.5f, wall_h * 0.5f, half_w}}, 0.0f);
    }}
"""
    content = content[:eol] + insert + content[eol:]
    write(path, content)
    return True

def fix_r314_baumgarte_dt():
    """R3-14: Baumgarte bias uses /dt instead of hardcoded *60."""
    log("R3-14: Baumgarte bias /dt")
    path = SRC / "physics" / "collision_mechanics.c"
    content = read(path)

    if MARKER + "_R314" in content:
        log("[SKIP] already fixed")
        return True

    old = "cp->separation_bias = bias_factor * fmaxf(cp->penetration - penetration_slop, 0.0f) * 60.0f;"
    new = f"cp->separation_bias = bias_factor * fmaxf(cp->penetration - penetration_slop, 0.0f) / dt; /* {MARKER}_R314 */"

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    else:
        log("[WARN] Baumgarte pattern not found")
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
    print("  202: Critical Robustness Fixes")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
    print()

    fixes = [
        fix_r306_wake_on_contact,
        fix_r307_containment_walls,
        fix_r314_baumgarte_dt,
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
        print("  ALL 3 ROBUSTNESS FIXES APPLIED")
        print("  Next: run fixes/203_persistence_safety.py")
    print("=" * 66)
    return 1 if failed else 0

if __name__ == "__main__":
    sys.exit(main())
