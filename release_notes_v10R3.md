# MFS-W — Native Windows Release Notes

| | |
|---|---|
| **Internal designation** | `MFS-W` (Windows port of `v15R3`) |
| **Port date** | 2026-09-07 |
| **Base tree** | MPE v15R3 (post-Milestone-2, cylinder campaign complete) |
| **License** | GPL-3.0 |
| **Platforms** | Windows x64 (primary), Linux (unchanged) |

---

## What is this?

MFS-W is the MPE FTC Simulator built as a **native Win64 executable** via
MSYS2 / MinGW-w64. No WSL, no Wine, no emulation. GTK3, libepoxy, and
OpenGL 3.3 Core all run natively; the gamepad layer uses Windows XInput.

The same source tree still builds identically on Linux — every platform
difference is behind `#ifdef` guards tagged `WIN_PORT`.

## Changes

### Platform port (11 changes, all guarded)

1. `root_gtk.c` — X11 backend force restricted to Linux; GDK uses the Win32
   backend on Windows.
2. `root_gtk.c` — gamepad left-stick Y inversion flipped on Windows
   (XInput reports stick-up as positive; evdev reports it negative).
3. `simulation.c` — POSIX `mkdir(path, mode)` / `access()` shimmed to
   `_mkdir` / `_access` via `<direct.h>` / `<io.h>`.
4. `config/mpe_config.c` — same `mkdir` shim (config dir creation).
5. `core/validation_report.c` — `access()` shim (engine.cfg presence check).
6. `core/long_run_validation.c` — `access()` shim (config restore check).
7. `ui_input/gamepad.c` — **full dual-platform rewrite**: XInput branch
   (`XInputGetState`, thumb sticks /32768, triggers /255, button masks) and
   the original evdev branch, selected by `#ifdef _WIN32`. Shared deadzone /
   inversion helpers unchanged.
8. `ui_input/input_control.c` — mouse delta clamp tightened from half-window
   to ±80 px/frame. `gdk_device_warp` is not atomic on Win32; unclamped
   deltas produced violent single-frame camera snaps.
9. `makefile` — `-lxinput9_1_0` added under `ifeq ($(OS),Windows_NT)`.
10. `ui_input/overlay.c` — per-widget CSS providers at
    `GTK_STYLE_PROVIDER_PRIORITY_USER + 1` force `#E8EEF7` text with a black
    text-shadow halo on all seven HUD labels. The Windows GTK theme otherwise
    renders label text black over the dark 3D scene.
11. `ui_input/debug_terminal.c` — new `term_normal` tag (`#CDD6E4`) routes all
    untagged terminal output; entry/prompt widgets get USER+1 CSS. Fixes
    black-on-black plain output (`ps aux`, `cat`, etc.).

### Tooling added

- `windows_port.py` — applies fixes 1–9 idempotently with `.pre_winport` backups.
- `fix_text_visibility.py` — applies fixes 10–11.
- `generate_docs.py` — regenerates the full documentation set (this file included).

## Verified on Windows (2026-09-07)

- [X] Clean build under MSYS2 MINGW64 (after two port iterations:
      `_mkdir` arity, `XINPUT_GAMEPAD_GUIDE` absent from MinGW headers)
- [X] Engine launches from the MINGW64 terminal
- [X] Engine launches from File Explorer with bundled DLLs
- [X] Grid, overlay, and robot HUD render
- [X] F310 (X mode) detected; drive / strafe / rotate correct polarity
- [X] Mouse look stable (no spikes)
- [X] All HUD and terminal text readable

## NOT yet verified on Windows (Gate 15e debt)

- [ ] F5–F11 validation suite
- [ ] Headless test suite under MinGW (`tools/test_runner.py`)
- [ ] Scene save/load round-trip
- [ ] Config persistence across relaunch
- [ ] Long idle soak
- [ ] Clean-machine zip test (no MSYS2 installed)

**Do not tag MFS-W as stable until the above are green.** See
`VALIDATION_WINDOWS.md` for the exact procedure.

## Known Windows quirks

- **Guide button** (Xbox logo): `XINPUT_GAMEPAD_GUIDE` is not defined in
  MinGW-w64's `xinput.h`; the button always reads `false`. Unused by MFS.
- **SmartScreen**: the exe is unsigned; first launch shows a warning.
- **Mouse lock**: `gdk_seat_grab` works on Win32 but recentering is slightly
  less crisp than X11. The ±80 px clamp absorbs the difference.
- **Relative paths**: shaders load from `render/shaders/*.glsl` relative to
  the working directory. Launching the exe from a shortcut with a different
  "Start in" folder produces a red screen. The release zip sets this up
  correctly; keep `render/` and `status/` beside `engine.exe`.

## Distribution

See `WINDOWS_DISTRIBUTION.md` (classroom one-pager) and
`install/windows/windows_build_instructions.md` (full build + packaging).

## Lineage

- **MFS-W v10R3I (V1.0R3-Interim)** (2026-09-07) — native Windows port *(this release)*
- **v15R3** — cylinder collision campaign, headless tests, chassis-velocity
  odometry, FTC field builder, robot config presets
- **v15R2** — config system hardening + MFS merge
- **v15R1** — centralised configuration system

---

*Release gates: `RELEASE_GATES.md` · Policy: `RELEASE_POLICY.md` ·
Platform reference: `../docs/PLATFORM_PORT.md`*
