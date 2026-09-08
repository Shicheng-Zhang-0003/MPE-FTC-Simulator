#!/usr/bin/env python3
"""
205: Fix R3-14 — add dt parameter to collision_prepare_solver
================================================================
The actual signatures are:
  .c: void collision_prepare_solver(collision_data *source, collision_data *m) {
  .h: void collision_prepare_solver(collision_data *source, collision_data *manifold_entry);

Callers:
  physics_world.c:          collision_prepare_solver(&narrowphase_collision, &world_manifolds[...]);
  physics_world.c:          collision_prepare_solver(&floor_collision, &world_manifolds[...]);
  simulation_physics_loop.c: collision_prepare_solver(&narrowphase_collision, &active_manifold[...]);
  simulation_physics_loop.c: collision_prepare_solver(&floor_collision, &active_manifold[...]);
"""
import sys, shutil, subprocess, re
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"

DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_205"

def log(msg): print(f"  [205] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_205")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_definition():
    """Add float dt to the function definition in collision_mechanics.c."""
    log("Fix 1: Function definition")
    path = SRC / "physics" / "collision_mechanics.c"
    content = read(path)

    if MARKER in content:
        log("[SKIP] already fixed")
        return True

    old = "void collision_prepare_solver(collision_data *source, collision_data *m) {"
    new = f"void collision_prepare_solver(collision_data *source, collision_data *m, float dt) {{ /* {MARKER} */"

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    else:
        log("[FAIL] definition pattern not found")
        return False

def fix_declaration():
    """Add float dt to the function declaration in collision_mechanics.h."""
    log("Fix 2: Function declaration")
    path = SRC / "physics" / "collision_mechanics.h"
    content = read(path)

    if MARKER in content:
        log("[SKIP] already fixed")
        return True

    old = "void collision_prepare_solver(collision_data *source, collision_data *manifold_entry);"
    new = f"void collision_prepare_solver(collision_data *source, collision_data *manifold_entry, float dt); /* {MARKER} */"

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    else:
        log("[FAIL] declaration pattern not found")
        return False

def fix_callers():
    """Update all callers to pass dt."""
    log("Fix 3: Update callers")

    # physics_world.c — uses dt (parameter of physics_world_step)
    pw_path = SRC / "core" / "physics_world.c"
    if pw_path.exists():
        content = read(pw_path)
        if MARKER not in content:
            # Replace all collision_prepare_solver calls with dt
            content = content.replace(
                "collision_prepare_solver(&narrowphase_collision, &world_manifolds[manifold_count]);",
                f"collision_prepare_solver(&narrowphase_collision, &world_manifolds[manifold_count], dt); /* {MARKER} */"
            )
            content = content.replace(
                "collision_prepare_solver(&floor_collision, &world_manifolds[manifold_count]);",
                f"collision_prepare_solver(&floor_collision, &world_manifolds[manifold_count], dt); /* {MARKER} */"
            )
            write(pw_path, content)
        else:
            log("[SKIP] physics_world.c already fixed")

    # simulation_physics_loop.c — uses fixed_physics_dt
    sim_path = SRC / "core" / "simulation_physics_loop.c"
    if sim_path.exists():
        content = read(sim_path)
        if MARKER not in content:
            content = content.replace(
                "collision_prepare_solver(&narrowphase_collision, &active_manifold[manifold_count]);",
                f"collision_prepare_solver(&narrowphase_collision, &active_manifold[manifold_count], fixed_physics_dt); /* {MARKER} */"
            )
            content = content.replace(
                "collision_prepare_solver(&floor_collision, &active_manifold[manifold_count]);",
                f"collision_prepare_solver(&floor_collision, &active_manifold[manifold_count], fixed_physics_dt); /* {MARKER} */"
            )
            write(sim_path, content)
        else:
            log("[SKIP] simulation_physics_loop.c already fixed")

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
    print("  205: R3-14 dt Parameter Fix")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
    print()

    ok = True
    ok = fix_definition() and ok
    ok = fix_declaration() and ok
    ok = fix_callers() and ok

    if ok and not DRY_RUN:
        ok = build_check()

    print()
    print("=" * 66)
    if ok:
        print("  R3-14 FIXED — Baumgarte bias now uses / dt")
        print("  Next: re-run fixes/202_robustness_critical.py")
    else:
        print("  FAILED — see errors above")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
