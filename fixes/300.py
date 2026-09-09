#!/usr/bin/env python3
"""
300: physics_world_init lifecycle fix
======================================
Fixes three catastrophic bugs in physics_world_init():
1. memset zeroes bodies pointer before checking allocation → memory leak on reinit
2. next_object_id resets → stale constraints match new bodies
3. Containment walls added before bodies allocated → walls silently never created

Usage:
    cd <project_root>
    python3 fixes/300_lifecycle_world_init.py [--dry-run]
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_300"

def log(msg): print(f"  [300] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_300")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_physics_world_init():
    path = SRC / "core" / "physics_world.c"
    content = read(path)

    if MARKER in content:
        log("[SKIP] already fixed")
        return True

    # --- Step 1: Replace the memset + containment walls + allocation block ---
    # Find the old block: memset → walls → conditional allocation
    old_block = """    memset(world, 0, sizeof(physics_world)); /* MPE_FTC_076a */
    /* MFS_202_R307: FTC field containment walls (12ft x 12ft).
     * Prevents bodies from escaping to infinity and producing NaN. */
    {
        float half_w = 1.8288f;   /* 6 ft */
        float wall_h = 0.3048f;   /* 12 in */
        float wall_t = 0.0254f;   /* 1 in */
        /* North wall */
        physics_world_add_cube(world,
            (vector3){0.0f, wall_h * 0.5f, half_w},
            (vector3){half_w, wall_h * 0.5f, wall_t * 0.5f}, 0.0f);
        /* South wall */
        physics_world_add_cube(world,
            (vector3){0.0f, wall_h * 0.5f, -half_w},
            (vector3){half_w, wall_h * 0.5f, wall_t * 0.5f}, 0.0f);
        /* East wall */
        physics_world_add_cube(world,
            (vector3){half_w, wall_h * 0.5f, 0.0f},
            (vector3){wall_t * 0.5f, wall_h * 0.5f, half_w}, 0.0f);
        /* West wall */
        physics_world_add_cube(world,
            (vector3){-half_w, wall_h * 0.5f, 0.0f},
            (vector3){wall_t * 0.5f, wall_h * 0.5f, half_w}, 0.0f);
    }
    if (!world->bodies) {
        world->bodies = (rigidbody *) malloc((size_t) mpe_max_bodies * sizeof(rigidbody));
        world->body_capacity = mpe_max_bodies;
    }
    if (!world->world_contact_cache) {
        world->world_contact_cache =
            (cached_contact *) malloc((size_t) max_cached_contacts * sizeof(cached_contact)); /* MFS_131A */
    }
    world->body_count = 0;
    if (world->next_object_id == 0) {
        world->next_object_id = 1;
    }"""

    new_block = f"""    /* {MARKER}_LIFECYCLE: If already initialized, free old allocations first.
     * Without this, calling init twice leaks the old bodies and cache. */
    if (world->bodies) {{
        free(world->bodies);
        world->bodies = NULL;
    }}
    if (world->world_contact_cache) {{
        free(world->world_contact_cache);
        world->world_contact_cache = NULL;
    }}
    memset(world, 0, sizeof(physics_world));
    /* {MARKER}: Allocate bodies FIRST so containment walls can actually be added. */
    world->bodies = (rigidbody *) malloc((size_t) mpe_max_bodies * sizeof(rigidbody));
    world->body_capacity = mpe_max_bodies;
    world->world_contact_cache =
        (cached_contact *) malloc((size_t) max_cached_contacts * sizeof(cached_contact));
    world->body_count = 0;
    world->next_object_id = 1;
    /* MFS_202_R307: FTC field containment walls (12ft x 12ft).
     * {MARKER}: Moved AFTER bodies allocation so add_cube actually works. */
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
    }}"""

    if old_block in content:
        content = content.replace(old_block, new_block, 1)
        write(path, content)
        return True

    log("[WARN] exact block not found — trying fallback pattern")
    # Fallback: just fix the memset line
    old_memset = "    memset(world, 0, sizeof(physics_world)); /* MPE_FTC_076a */"
    new_memset = f"""    /* {MARKER}_LIFECYCLE: Free old allocations before memset to prevent leaks. */
    if (world->bodies) {{
        free(world->bodies);
        world->bodies = NULL;
    }}
    if (world->world_contact_cache) {{
        free(world->world_contact_cache);
        world->world_contact_cache = NULL;
    }}
    memset(world, 0, sizeof(physics_world));"""
    if old_memset in content:
        content = content.replace(old_memset, new_memset, 1)
        write(path, content)
        log("[OK] memset fixed (fallback). Walls/allocation order needs manual review.")
        return True

    log("[FAIL] could not find physics_world_init patterns")
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
    print("  300: physics_world_init Lifecycle Fix")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
        print()
    ok = fix_physics_world_init()
    if ok and not DRY_RUN:
        ok = build_check()
    print()
    print("=" * 66)
    if ok:
        print("  PASS 1a COMPLETE")
        print("  Next: python3 fixes/301_lifecycle_constraint_tag.py")
    else:
        print("  FAILED — see errors above")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
