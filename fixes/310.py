#!/usr/bin/env python3
"""
310: Reject invalid object types on scene load
================================================
Currently, unknown type_int values in scene files silently become spheres.
This means corrupted scenes produce garbage objects instead of failing cleanly.

Fix: validate type_int against known enum values before initializing.
If invalid, abort the load and preserve the live scene.

Usage:
    cd <project_root>
    python3 fixes/310.py [--dry-run]
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_310"

def log(msg): print(f"  [310] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_310")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_scene_load():
    path = SRC / "scene" / "scene_load.c"
    content = read(path)
    if MARKER in content:
        log("[SKIP] already fixed")
        return True

    # Add type validation in the staging loop.
    # After reading type_int, validate it before continuing.
    old = """        if (!read_int(f, &sb->type_int)) break;"""
    new = f"""        if (!read_int(f, &sb->type_int)) break;
        /* {MARKER}: Reject unknown object types instead of silently
         * converting them to spheres. Corrupted scene data should
         * fail the load, not produce garbage objects. */
        if ((sb->type_int != object_sphere) &&
            (sb->type_int != object_cube) &&
            (sb->type_int != object_cylinder)) {{
            fprintf(stderr, "Error LDF05: Unknown object type %d at body %d. Load aborted.\\n",
                    sb->type_int, i);
            fclose(f);
            return 0;
        }}"""

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True

    log("[WARN] exact pattern not found — trying line-based approach")
    # Line-based fallback
    lines = content.split('\n')
    for i, line in enumerate(lines):
        if 'if (!read_int(f, &sb->type_int)) break;' in line:
            indent = line[:len(line) - len(line.lstrip())]
            insert_lines = [
                f"{indent}/* {MARKER}: Reject unknown object types */",
                f"{indent}if ((sb->type_int != object_sphere) &&",
                f"{indent}    (sb->type_int != object_cube) &&",
                f"{indent}    (sb->type_int != object_cylinder)) {{",
                f'{indent}    fprintf(stderr, "Error LDF05: Unknown object type %d at body %d. Load aborted.\\n",',
                f"{indent}            sb->type_int, i);",
                f"{indent}    fclose(f);",
                f"{indent}    return 0;",
                f"{indent}}}",
            ]
            for j, il in enumerate(insert_lines):
                lines.insert(i + 1 + j, il)
            content = '\n'.join(lines)
            write(path, content)
            return True

    log("[FAIL] could not find type_int read pattern")
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
    print("  310: Reject Invalid Object Types on Scene Load")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
        print()
    ok = fix_scene_load()
    if ok and not DRY_RUN:
        ok = build_check()
    print()
    print("=" * 66)
    if ok:
        print("  PASS — invalid types now abort scene load")
        print("  Next: python3 fixes/311.py")
    else:
        print("  FAILED — see errors above")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
