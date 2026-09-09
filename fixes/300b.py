#!/usr/bin/env python3
"""
300b: Fix physics_world_init ordering
The 300 fallback added free-before-memset but left walls before allocation.
Walls call physics_world_add_cube which requires world->bodies != NULL.
Must move walls AFTER allocation.
"""
import sys, shutil, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v10R3I" / "src"
DRY_RUN = "--dry-run" in sys.argv
MARKER = "MFS_300B"

def log(msg): print(f"  [300b] {msg}")

def read(path):
    return path.read_text(encoding="utf-8", errors="replace")

def write(path, content):
    if DRY_RUN:
        log(f"[DRY-RUN] would write {path.name}")
        return
    bak = path.with_suffix(path.suffix + ".pre_300b")
    if not bak.exists():
        shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] wrote {path.name}")

def fix_ordering():
    path = SRC / "core" / "physics_world.c"
    content = read(path)

    if MARKER in content:
        log("[SKIP] already fixed")
        return True

    # The current state after 300 fallback:
    # ... free + memset ...
    # /* MFS_202_R307: walls block */
    # { ... walls ... }
    # if (!world->bodies) { malloc }
    # if (!world->world_contact_cache) { malloc }
    # world->body_count = 0;
    # if (world->next_object_id == 0) { ... }

    # Strategy: find the walls block and the allocation block,
    # then reorder them.

    # Find the walls block start
    walls_start_marker = "    /* MFS_202_R307: FTC field containment walls"
    walls_start = content.find(walls_start_marker)
    if walls_start < 0:
        log("[WARN] walls block not found")
        return False

    # Find the end of the walls block (closing brace of the block)
    # The walls block is: { ... 4x physics_world_add_cube ... }
    # Find the closing } of the outer { that starts the walls block
    brace_start = content.find("    {\n", walls_start)
    if brace_start < 0:
        brace_start = content.find("    {", walls_start)
    if brace_start < 0:
        log("[WARN] walls brace not found")
        return False

    # Count braces to find the end
    depth = 0
    walls_end = brace_start
    for i in range(brace_start, len(content)):
        if content[i] == '{':
            depth += 1
        elif content[i] == '}':
            depth -= 1
            if depth == 0:
                walls_end = i + 1
                break

    walls_block = content[walls_start:walls_end]

    # Find the allocation block
    alloc_marker = "    if (!world->bodies) {"
    alloc_start = content.find(alloc_marker)
    if alloc_start < 0:
        log("[WARN] allocation block not found")
        return False

    # Find end of allocation section (through next_object_id)
    next_id_marker = "    if (world->next_object_id == 0) {"
    next_id_pos = content.find(next_id_marker, alloc_start)
    if next_id_pos < 0:
        # Try alternate: world->next_object_id = 1;
        next_id_marker = "    world->next_object_id = 1;"
        next_id_pos = content.find(next_id_marker, alloc_start)

    if next_id_pos < 0:
        log("[WARN] next_object_id not found")
        return False

    # Find end of the next_object_id block
    # Look for the closing } of the if block or end of line
    if "if (world->next_object_id == 0)" in content[next_id_pos:next_id_pos+60]:
        # It's an if block, find its closing }
        brace_pos = content.find("{", next_id_pos)
        close_pos = content.find("}", brace_pos)
        alloc_end = close_pos + 1
    else:
        # It's a simple assignment, find end of line
        eol = content.find("\n", next_id_pos)
        alloc_end = eol

    alloc_block = content[alloc_start:alloc_end]

    # Now reconstruct: remove walls from current position,
    # remove alloc from current position,
    # put alloc first, then walls

    # Remove walls block from content
    content_without_walls = content[:walls_start] + content[walls_end:]

    # Recalculate alloc position in modified content
    offset = walls_end - walls_start
    new_alloc_start = alloc_start - offset if alloc_start > walls_start else alloc_start
    new_alloc_end = alloc_end - offset if alloc_end > walls_start else alloc_end

    # Remove alloc block
    content_without_both = content_without_walls[:new_alloc_start] + content_without_walls[new_alloc_end:]

    # Insert alloc + walls at the position where walls used to be
    # But we want: alloc first, then walls
    insert_pos = walls_start  # where walls used to start
    new_content = (content_without_both[:insert_pos] +
                   alloc_block + "\n" +
                   "    /* " + MARKER + ": Walls moved AFTER allocation so add_cube works. */\n" +
                   walls_block + "\n" +
                   content_without_both[insert_pos:])

    write(path, new_content)
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
    print("  300b: Fix physics_world_init Ordering")
    print("=" * 66)
    if DRY_RUN:
        print("  ** DRY RUN **")
        print()
    ok = fix_ordering()
    if ok and not DRY_RUN:
        ok = build_check()
    print()
    if ok:
        print("  PASS — walls now come after allocation")
    else:
        print("  FAILED — see errors above")
    print("=" * 66)
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
