#!/usr/bin/env python3
"""
200: Makefile repair
Fixes stray -lxinput9_1_0 after endif in WIN_PORT_LIBS block.
"""
import sys
from pathlib import Path

DRY_RUN = "--dry-run" in sys.argv
SRC = Path(__file__).resolve().parent.parent

def log(msg): print(f"  [200] {msg}")

def main():
    makefile = SRC / "makefile"
    content = makefile.read_text(encoding="utf-8")

    if "endif -lxinput9_1_0" in content:
        log("Found stray -lxinput9_1_0 after endif")
        if not DRY_RUN:
            content = content.replace("endif -lxinput9_1_0", "endif")
            makefile.write_text(content, encoding="utf-8")
            log("[OK] makefile repaired")
        else:
            log("[DRY-RUN] would repair")
    elif "endif\n" in content:
        log("[SKIP] makefile already clean")
    else:
        log("[WARN] could not find endif pattern")

    return 0

if __name__ == "__main__":
    sys.exit(main())
