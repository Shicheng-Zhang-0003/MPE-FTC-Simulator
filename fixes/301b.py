#!/usr/bin/env python3
"""
301b: Fix constraint_add_revolute generation tagging
The 301 script failed to match because constraint.c uses NO indentation.
Without this fix, all constraints have generation=0 but the counter is 1,
so constraint_dispatch skips everything. THIS BREAKS ALL JOINTS.
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_301B"

def log(msg): print(f"  [301b] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_301b")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_constraint_add():
    path = SRC / "physics" / "constraint.c"
    content = read(path)

    if MARKER in content:
        log("[SKIP] already fixed")
        return True

    # The actual pattern in constraint.c (NO indentation):
    old = """constraint_pool [i].is_active = true;
constraint_count++;
return i;"""

    new = f"""constraint_pool [i].is_active = true;
constraint_pool [i].world_generation = constraint_world_generation; /* {MARKER} */
constraint_count++;
return i;"""

    if old in content:
        content = content.replace(old, new, 1)
        write(path, content)
        return True

    # Try alternate spacing
    old2 = "constraint_pool [i].is_active = true;\nconstraint_count++;\nreturn i;"
    if old2 in content:
        new2 = f"constraint_pool [i].is_active = true;\nconstraint_pool [i].world_generation = constraint_world_generation; /* {MARKER} */\nconstraint_count++;\nreturn i;"
        content = content.replace(old2, new2, 1)
        write(path, content)
        return True

    # Last resort: find is_active = true near constraint_count++
    lines = content.split('\n')
    for i, line in enumerate(lines):
        if 'constraint_pool [i].is_active = true;' in line:
            # Check if next non-empty line has constraint_count++
            for j in range(i+1, min(i+4, len(lines))):
                if 'constraint_count++' in lines[j]:
                    # Insert generation tag between them
                    indent = ''
                    lines.insert(j, f'{indent}constraint_pool [i].world_generation = constraint_world_generation; /* {MARKER} */')
                    content = '\n'.join(lines)
                    write(path, content)
                    return True

    log("[FAIL] could not find constraint_add_revolute pattern")
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
    print("  301b: Fix Constraint Generation Tagging (CRITICAL)")
    print("=" * 66)
    print("  Without this, ALL constraints are skipped after pool_init.")
    print()
    if DRY_RUN:
        print("  ** DRY RUN **")
        print()
    ok = fix_constraint_add()
    if ok and not DRY_RUN:
        ok = build_check()
    print()
    if ok:
        print("  PASS — constraints now tagged with world generation")
    else:
        print("  FAILED — constraints remain broken")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
