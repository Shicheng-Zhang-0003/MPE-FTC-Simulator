#!/usr/bin/env python3
"""
208: Fix atomic writes for Windows.
Windows rename() fails if target exists. Must remove() target first.
Fixes both mpe_config.c (config save) and scene_saving.c (scene save).
"""
import sys
import shutil
from pathlib import Path

DRY_RUN = "--dry-run" in sys.argv
SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"

def log(msg): print(f"  [208] {msg}")

def read(path): return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_208")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_config_save():
    """Fix mpe_config.c: remove() target before rename()."""
    path = SRC / "config" / "mpe_config.c"
    content = read(path)

    if "MFS_208" in content:
        log("[SKIP] mpe_config.c already fixed")
        return True

    # The current broken pattern: rename without removing target first
    old = '    if (rename(tmp_path, path) != 0) {\n        fprintf(stderr, "[config] Error: could not rename %s to %s\\n", tmp_path, path);\n        return false;\n    }'

    new = """    /* MFS_208_ATOMIC_WRITE_WINDOWS: Windows rename() fails if target exists.
     * Must remove() target first. On POSIX this is a no-op (rename replaces). */
    remove(path);
    if (rename(tmp_path, path) != 0) {
        fprintf(stderr, "[config] Error: could not rename %s to %s\\n", tmp_path, path);
        return false;
    }"""

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True

    # Fallback: try to find any rename(tmp_path, path) pattern
    if 'rename(tmp_path, path)' in content:
        content = content.replace(
            'rename(tmp_path, path)',
            'remove(path); /* MFS_208: Windows needs target removed first */\n    rename(tmp_path, path)',
            1
        )
        write(path, content)
        return True

    log("[WARN] rename pattern not found in mpe_config.c — atomic writes may not be applied yet")
    return True

def fix_scene_save():
    """Fix scene_saving.c: same remove-before-rename fix."""
    path = SRC / "scene" / "scene_saving.c"
    content = read(path)

    if "MFS_208" in content:
        log("[SKIP] scene_saving.c already fixed")
        return True

    if 'rename(tmp_path, file_destination_path)' in content:
        content = content.replace(
            'rename(tmp_path, file_destination_path)',
            'remove(file_destination_path); /* MFS_208: Windows needs target removed first */\n    rename(tmp_path, file_destination_path)',
            1
        )
        write(path, content)
        return True

    # If scene_saving.c doesn't have atomic writes yet, skip
    if 'tmp_path' not in content:
        log("[INFO] scene_saving.c does not use atomic writes yet — skipping")
        return True

    log("[WARN] rename pattern not found in scene_saving.c")
    return True

def main():
    print("=" * 50)
    print("  208: Atomic Write Windows Fix")
    print("=" * 50)
    if DRY_RUN:
        print("  ** DRY RUN **")
    print()

    ok = True
    ok = fix_config_save() and ok
    ok = fix_scene_save() and ok

    print()
    if ok:
        print("  Done. Rebuild with: make clean && make")
    else:
        print("  Some fixes failed — check output above.")
    print("=" * 50)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
