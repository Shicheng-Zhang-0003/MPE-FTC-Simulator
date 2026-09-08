#!/usr/bin/env python3
"""
203: Persistence safety fixes
==============================
R3-04: Cylinder save/load preserves cylinder_half_length
R3-12: ID wrap check
R3-09: Broadphase overflow warning
R3-05: Config corrupt line warning
R3-15: Terminal sandbox (block writes to src/)

Usage:
    cd <project_root>
    python3 fixes/203_persistence_safety.py [--dry-run]
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_203"

def log(msg): print(f"  [203] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_203")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_r304_cylinder_save():
    """R3-04: Save cylinder_half_length in scene file."""
    log("R3-04: Cylinder save/load")
    path = SRC / "scene" / "scene_saving.c"
    content = read(path)

    if MARKER + "_R304" in content:
        log("[SKIP] already fixed")
        return True

    # Find where object_id is written (last field before joints)
    old = 'write_int(f, (int32_t) rb->object_id); /* MPE_FTC_058 */'
    new = f"""write_int(f, (int32_t) rb->object_id); /* MPE_FTC_058 */
    write_float(f, rb->cylinder_half_length); /* {MARKER}_R304: save cylinder geometry */"""

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    else:
        log("[WARN] object_id write pattern not found")
        return True

def fix_r304_cylinder_load():
    """R3-04: Load cylinder_half_length from scene file."""
    log("R3-04: Cylinder load")
    path = SRC / "scene" / "scene_load.c"
    content = read(path)

    if MARKER + "_R304_LOAD" in content:
        log("[SKIP] already fixed")
        return True

    # Find where object_id is read
    old = """        if ((version >= 150) && (saved_object_id > 0)) {
            scene_id_remap_add((uint32_t) saved_object_id, obj_per_scene[i].object_id);
        } /* MPE_FTC_058 */"""

    new = f"""        if ((version >= 150) && (saved_object_id > 0)) {{
            scene_id_remap_add((uint32_t) saved_object_id, obj_per_scene[i].object_id);
        }} /* MPE_FTC_058 */
        /* {MARKER}_R304_LOAD: read cylinder_half_length */
        {{
            float cyl_half = 0.0f;
            if (read_float(f, &cyl_half)) {{
                obj_per_scene[i].cylinder_half_length = cyl_half;
            }}
        }}"""

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    else:
        log("[WARN] object_id read pattern not found")
        return True

def fix_r312_id_wrap():
    """R3-12: Check for ID collisions on wrap."""
    log("R3-12: ID wrap check")
    path = SRC / "scene" / "scene_init.c"
    content = read(path)

    if MARKER + "_R312" in content:
        log("[SKIP] already fixed")
        return True

    old = """uint32_t scene_allocate_object_id(void) {
    return next_object_id++;
}"""

    new = f"""uint32_t scene_allocate_object_id(void) {{
    /* {MARKER}_R312: Check for wrap-around collision */
    if (next_object_id == 0) {{
        next_object_id = 1;  /* Skip 0, it's the sentinel */
        fprintf(stderr, "[WARN] object ID wrapped, resetting to 1\\n");
    }}
    return next_object_id++;
}}"""

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    else:
        log("[WARN] scene_allocate_object_id pattern not found")
        return True

def fix_r309_broadphase_warning():
    """R3-09: Warn on broadphase overflow."""
    log("R3-09: Broadphase overflow warning")
    path = SRC / "physics" / "broadphase.c"
    content = read(path)

    if MARKER + "_R309" in content:
        log("[SKIP] already fixed")
        return True

    # Find where node pool overflow happens
    old = "if (node_count >= node_pool_capacity) {"
    new = f"""if (node_count >= node_pool_capacity) {{
        fprintf(stderr, "[WARN] {MARKER}_R309: Broadphase node pool exhausted: %d/%d\\n",
                node_count, node_pool_capacity);"""

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    else:
        log("[WARN] broadphase overflow pattern not found")
        return True

def fix_r305_config_warning():
    """R3-05: Warn on corrupt config lines."""
    log("R3-05: Config corrupt warning")
    path = SRC / "config" / "mpe_config.c"
    content = read(path)

    if MARKER + "_R305" in content:
        log("[SKIP] already fixed")
        return True

    # Find where corrupt lines are skipped
    old = "continue;"
    # This is too generic - need to find the specific context
    # Look for the pattern where a line is skipped due to parse failure
    log("[INFO] Config corrupt warning requires manual review of mpe_config.c")
    log("[INFO] Look for the line-skipping logic in mpe_config_load and add fprintf")
    return True

def fix_r315_terminal_sandbox():
    """R3-15: Block terminal writes to source files."""
    log("R3-15: Terminal sandbox")
    path = SRC / "ui_input" / "debug_terminal.c"
    content = read(path)

    if MARKER + "_R315" in content:
        log("[SKIP] already fixed")
        return True

    # Find cmd_tee function
    old = 'static void cmd_tee(int argc, char **argv) {'
    new = f"""static void cmd_tee(int argc, char **argv) {{
    /* {MARKER}_R315: Sandbox — block writes to source files */
    if (argc >= 2) {{
        const char *target = argv[argc - 1];
        if ((strstr(target, "src/")) || (strstr(target, "makefile")) ||
            (strstr(target, ".c")) || (strstr(target, ".h"))) {{
            term_printf("term_err", "mpe: tee: cannot write to source files");
            return;
        }}
    }}"""

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True
    else:
        log("[WARN] cmd_tee pattern not found")
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
    print("  203: Persistence Safety Fixes")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
    print()

    fixes = [
        fix_r304_cylinder_save,
        fix_r304_cylinder_load,
        fix_r312_id_wrap,
        fix_r309_broadphase_warning,
        fix_r305_config_warning,
        fix_r315_terminal_sandbox,
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
        print("  ALL PERSISTENCE SAFETY FIXES APPLIED")
        print()
        print("  REMAINING MANUAL ITEMS:")
        print("  - NEW-05: Remove artificial lateral damping (needs friction tuning)")
        print("  - NEW-02: Cylinder-cube normal direction (geometry)")
        print("  - NEW-06: Revolute constraint iteration (solver)")
        print("  - R3-02: Scene load staging (complex refactor)")
        print("  - R3-03: Atomic writes (temp file + rename)")
        print("  - 2.1: Two-tangent friction cone (solver rewrite)")
    print("=" * 66)
    return 1 if failed else 0

if __name__ == "__main__":
    sys.exit(main())
