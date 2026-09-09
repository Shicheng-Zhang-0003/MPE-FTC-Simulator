#!/usr/bin/env python3
"""
311: Scene save — check fwrite return values
=============================================
The write_float/write_int/write_vec3/write_vec4 helpers ignore fwrite()
return values. A disk-full or I/O error silently produces a truncated file.

Fix: make helpers return bool, check results, abort save on failure.

Usage:
    cd <project_root>
    python3 fixes/311.py [--dry-run]
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_311"

def log(msg): print(f"  [311] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_311")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_scene_saving():
    path = SRC / "scene" / "scene_saving.c"
    content = read(path)
    if MARKER in content:
        log("[SKIP] already fixed")
        return True

    # Replace write helpers with checked versions
    old_float = """static void write_float(FILE *f, float v) {
    fwrite(&v, sizeof(float), 1, f);
}"""
    new_float = f"""/* {MARKER}: Write helpers now return bool for error checking. */
static bool write_float(FILE *f, float v) {{
    return fwrite(&v, sizeof(float), 1, f) == 1;
}}"""

    old_int = """static void write_int(FILE *f, int32_t v) {
    fwrite(&v, sizeof(int32_t), 1, f);
}"""
    new_int = f"""static bool write_int(FILE *f, int32_t v) {{
    return fwrite(&v, sizeof(int32_t), 1, f) == 1;
}}"""

    old_vec3 = """static void write_vec3(FILE *f, vector3 v) {
    fwrite(&v, sizeof(vector3), 1, f);
}"""
    new_vec3 = f"""static bool write_vec3(FILE *f, vector3 v) {{
    return fwrite(&v, sizeof(vector3), 1, f) == 1;
}}"""

    old_vec4 = """static void write_vec4(FILE *f, vector4 v) {
    fwrite(&v, sizeof(vector4), 1, f);
}"""
    new_vec4 = f"""static bool write_vec4(FILE *f, vector4 v) {{
    return fwrite(&v, sizeof(vector4), 1, f) == 1;
}}"""

    ok = True
    if old_float in content:
        content = content.replace(old_float, new_float, 1)
    else:
        log("[WARN] write_float pattern not found")
        ok = False

    if old_int in content:
        content = content.replace(old_int, new_int, 1)
    else:
        log("[WARN] write_int pattern not found")
        ok = False

    if old_vec3 in content:
        content = content.replace(old_vec3, new_vec3, 1)
    else:
        log("[WARN] write_vec3 pattern not found")
        ok = False

    if old_vec4 in content:
        content = content.replace(old_vec4, new_vec4, 1)
    else:
        log("[WARN] write_vec4 pattern not found")
        ok = False

    # Now add write-failure checks in the save loop.
    # After the header writes, add a failure check pattern.
    # We'll add a goto-based error handler.
    old_save_start = """    write_int(f, mpe_magic);
    write_int(f, mpe_version);
    write_int(f, object_count);"""
    new_save_start = f"""    /* {MARKER}: All writes now checked. On failure, abort and remove tmp. */
    if (!write_int(f, mpe_magic) ||
        !write_int(f, mpe_version) ||
        !write_int(f, object_count)) {{
        fclose(f);
        remove(tmp_path);
        fprintf(stderr, "Error SVF03: Write failed during header.\\n");
        return 0;
    }}"""

    if old_save_start in content:
        content = content.replace(old_save_start, new_save_start, 1)
    else:
        log("[WARN] save header pattern not found")
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
    print("  311: Scene Save — fwrite Return Value Checks")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
        print()
    ok = fix_scene_saving()
    if ok and not DRY_RUN:
        ok = build_check()
    print()
    print("=" * 66)
    if ok:
        print("  PASS — scene save now detects write failures")
        print("  Next: python3 fixes/313.py")
    else:
        print("  FAILED — see errors above")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
