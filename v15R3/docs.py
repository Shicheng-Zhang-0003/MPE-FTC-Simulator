#!/usr/bin/env python3
"""
MFS-W Documentation Generator — master script
==============================================
Generates/replaces every document the Windows-native tree needs:

 1  readme.md                                   (root — full replacement)
 2  v15R3/how_to_use.md                         (full replacement)
 3  v15R3/RELEASE_GATES.md                      (full replacement + Windows gates)
 4  v15R3/RELEASE_POLICY.md                     (full replacement)
 5  v15R3/release_notes_v15R3W.md               (new)
 6  v15R3/install/windows/windows_build_instructions.md  (new)
 7  v15R3/WINDOWS_DISTRIBUTION.md               (new — teacher one-pager)
 8  docs/PLATFORM_PORT.md                       (new — WIN_PORT reference)
 9  v15R3/VALIDATION_WINDOWS.md                 (new — verification checklist)
10  v15R3/evolution.txt                         (patched — MFS-W milestone)
11  scope.md                                    (patched — PLAT-003 RESOLVED)

Usage:
    cd v15R3/src
    python3 generate_docs.py            # write everything
    python3 generate_docs.py --dry-run  # preview only
"""

import sys
import shutil
from pathlib import Path

DRY_RUN = "--dry-run" in sys.argv
SRC = Path(__file__).resolve().parent      # v15R3/src
V15R3 = SRC.parent                         # v15R3/
ROOT = V15R3.parent                        # project root
DATE = "2026-09-07"

def log(msg):
    print(f"  [DOCS] {msg}")

def write_doc(path: Path, content: str, label: str) -> bool:
    if DRY_RUN:
        try:
            rel = path.relative_to(ROOT)
        except ValueError:
            rel = path
        log(f"[DRY-RUN] would write {label} -> {rel}")
        return True
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        bak = path.with_suffix(path.suffix + ".pre_docs")
        if not bak.exists():
            shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    log(f"[OK] {label}")
    return True

# =====================================================================
# 1. ROOT README
# =====================================================================
README_MD = r'''# 🧊 MINIATURE PHYSICS ENGINE — MFS-W (Windows Build)

<!-- MPE_RELEASE_FREEZE_NOTICE_BEGIN -->
> **Current development tree:** `v15R3` carries **MFS-W** — the MPE FTC
> Simulator compiled and packaged for **native Windows**. It is not a tagged
> stable release; use the release gates before promotion.
<!-- MPE_RELEASE_FREEZE_NOTICE_END -->
<!-- MPE_RELEASE_GATES_NOTICE_BEGIN -->
> **Release quality:** criteria are in [`v15R3/RELEASE_GATES.md`](v15R3/RELEASE_GATES.md).
> Windows candidate notes: [`v15R3/release_notes_v15R3W.md`](v15R3/release_notes_v15R3W.md).
> Prior RC: [`v15R3/release_notes_v15R2.md`](v15R3/release_notes_v15R2.md).
<!-- MPE_RELEASE_GATES_NOTICE_END -->

> **This is MFS, not MPE.** It retains the full MPE physics kernel plus the FTC
> robotics layer, and is built primarily for **Windows** (MSYS2/MinGW-w64).
> Linux remains supported from the same source tree — every platform difference
> lives behind `#ifdef _WIN32` guards marked `WIN_PORT`. **No WSL. No Wine.
> No emulation.** This is a real Win64 executable linking GTK3 + OpenGL natively.

**License:** GPL-3.0 · **Language:** C · **UI:** GTK3 · **Renderer:** OpenGL 3.3 Core

---

## 🚀 Quick Start (Windows — Prebuilt)

If you received a `windows_release.zip`:

1. Extract the zip anywhere (e.g. `C:\MPE\`)
2. Double-click `engine.exe`
   *(First launch may trigger SmartScreen — the exe is unsigned. Click
   **More info → Run anyway**.)*
3. Press `0` (Debug Mode), then `T` (open terminal)
4. Type `touch robot` and press Enter
5. Plug in a gamepad and drive

### 🎮 Gamepad (required for driving)

| | |
|---|---|
| Supported | Logitech F310 and any XInput-compatible controller |
| **Critical** | F310 mode switch on the back must be set to **X** (XInput), not D |
| Plug in | **Before** launching the engine |
| Left stick | Forward / backward / strafe |
| Right stick X | Rotate |

Keyboard drive keys (G/B/V/N/C/H) were removed in MFS_159 — the gamepad is the
sole drive input, matching how students drive the real robot.

---

## 🔨 Building from Source (Windows)

Full walkthrough: [`v15R3/install/windows/windows_build_instructions.md`](v15R3/install/windows/windows_build_instructions.md)

Short version — open the **MINGW64** terminal:

```bash
pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-gtk3 \
          mingw-w64-x86_64-libepoxy mingw-w64-x86_64-pkg-config make python3
cd <tree>/v15R3/src
python3 windows_port.py          # 8 platform fixes (idempotent, backed up)
python3 fix_text_visibility.py   # readable HUD + terminal text
make clean && make
./engine.exe
```

Package for distribution (DLLs + shaders next to the exe):

```bash
mkdir -p ../windows_release
cp engine.exe ../windows_release/
cp -r render ../windows_release/
mkdir -p ../windows_release/status
for dll in $(ldd engine.exe | grep mingw64 | awk '{print $3}'); do
    cp "$dll" ../windows_release/
done
```

Zip `windows_release/`. Recipients need nothing installed.

## 🔨 Building from Source (Linux)

```bash
sudo apt install build-essential pkg-config libgtk-3-dev libepoxy-dev
cd v15R3/src
make clean && make
./engine
```

The `WIN_PORT` guards leave the Linux build path byte-identical to upstream MPE.

---

## 📋 Overview

MPE is a custom-built **3D rigid-body physics engine and real-time rendering
pipeline** in pure C, with a **zero-dependency core** — the only external
requirements are GTK3 (windowing/UI) and OpenGL (render backend).

Four priorities:

- **Mathematical transparency** — every integrator, solver, and collision test
  is hand-written and inspectable.
- **Cache-efficient data layouts** — tightly packed structs, contiguous
  instance buffers.
- **Deterministic simulation** — fixed 60 Hz timestep decoupled from render
  framerate.
- **Real-time scaling** — GPU instancing and an O(N) spatial-hash broadphase.

---

## ⚙️ Physics Engine

- **Broadphase:** 3D spatial-hash grid, adaptive cell sizing, sleep system.
- **Narrowphase:** Sphere–Sphere (analytical), Sphere–OBB (closest-point),
  OBB–OBB (15-axis SAT + Sutherland–Hodgman clipping), Cylinder–Sphere,
  Cylinder–Cube, Cylinder–Cylinder (segment closest-point),
  Cylinder–floor (axle-endpoint contact).
- **Solver:** warm-starting sequential impulse, 16 iterations, static +
  kinetic friction, Baumgarte penetration correction, positional
  depenetration pass.
- **Integration:** semi-implicit Euler (linear), quaternion (angular),
  fixed 60 Hz with 5-substep spiral-of-death cap.
- **Joints:** spring joints (live-rendered) and revolute/hinge joints with
  Baumgarte axis-drift correction.

<!-- MFS_FTC_SECTIONS_BEGIN -->
## 🤖 MFS — FTC Robotics Simulator

MFS turns MPE into an FTC-oriented robot simulator. Motor commands become
torque, torque spins cylinder wheels, wheels grip the floor through anisotropic
roller friction, and sensors read back from simulated state. Nothing is faked
at the contact layer.

### Capabilities

- **Cylinder wheel bodies** with correct axle inertia (`I = ½·m·r²`)
- **Mecanum drivetrain** via real anisotropic roller friction (±45° rollers) —
  no chassis-force cheats
- **Tank drivetrain** via motor torque → wheel traction
- **DC motor model** — BackEMF, gear ratio, Kt/Kv, stall/free-speed limits,
  goBILDA presets, thermal accumulation
- **Battery model** — voltage sag under multi-motor load
- **Odometry** — chassis-velocity integration + heading for dead reckoning
- **Revolute joints** — wheels hinged to chassis with axis correction
- **Idle hold** — gearbox-style lock so a parked robot stays parked

### Robot controls (gamepad only)

| Input | Action |
|-------|--------|
| Left stick Y | Forward / backward |
| Left stick X | Strafe |
| Right stick X | Rotate |

Spawn from the debug terminal: `touch robot`

### Headless test suite

| Test | Proves |
|------|--------|
| `two_world` | Independent `physics_world` instances |
| `revolute` | Hinge joints hold anchor and allow swing |
| `teleop_drive` | Tank drive moves the robot |
| `mecanum_drive` | Strafe via real roller friction |
| `cylinder_drop` | Cylinder settles on the floor |
| `driven_wheel` | Torque → friction → translation |
| `math3_inverse` | Matrix inverse at small inertia tensors |
| `ftc_integration` | Drive / turn / strafe sequence |
| `physics_truth` | Physical-law assertions |
| `tank_turn` | Differential turning in place |
| `odometry_accuracy` | Odometry tracks physics |
| `cylinder_sphere` / `cylinder_cube` / `cylinder_cylinder` | Cylinder narrowphase |

Run with `python3 tools/test_runner.py`. *(Linux-verified; MinGW run pending —
see `v15R3/VALIDATION_WINDOWS.md`.)*
<!-- MFS_FTC_SECTIONS_END -->

---

## 🎮 Controls

### Movement & Camera

| Action | Input |
|---|---|
| Move | `W A S D` |
| Look around | Mouse (left-click to lock) |
| Jump / fly up | `Space` |
| Fly down (Debug) | `Shift` |
| Steer camera mouse-free (Debug) | `I J K L` |
| Release mouse | `Escape` |
| Re-lock mouse (Debug) | `M` |
| Toggle Game / Debug mode | `0` |

### Spawning / Selection / Menus

| Action | Input |
|---|---|
| Spawn object | Hold `Enter` |
| Spawner settings | `8` |
| Select object | Right-click (raycast) or `R` (Debug) |
| Object menu | `E` |
| Apply impulse | `F` |
| Delete object | Middle-click or `Delete` (Debug) |
| Config menu | `6` |
| World settings | `7` |
| Save / Load scene | `9` |
| Debug terminal | `T` or `1` (Debug) |
| Validation tests | `F5`–`F11` |

Full guide: [`v15R3/how_to_use.md`](v15R3/how_to_use.md)

---

## 🪟 What the Windows Port Changed

Every change is tagged `WIN_PORT` and lives behind `#ifdef _WIN32`.
Full reference: [`docs/PLATFORM_PORT.md`](docs/PLATFORM_PORT.md).

| # | File | Change |
|---|------|--------|
| 1 | `root_gtk.c` | X11 backend force guarded to Linux |
| 2 | `root_gtk.c` | Gamepad Y-inversion flipped (XInput up = positive) |
| 3 | `simulation.c` | `mkdir`/`access` → `_mkdir`/`_access` |
| 4 | `config/mpe_config.c` | `mkdir` guard |
| 5 | `core/validation_report.c` | `access()` guard |
| 6 | `core/long_run_validation.c` | `access()` guard |
| 7 | `ui_input/gamepad.c` | Dual-platform rewrite: XInput (Win) / evdev (Linux) |
| 8 | `ui_input/input_control.c` | Mouse delta clamped ±80 px/frame (Win32 warp spikes) |
| 9 | `makefile` | `-lxinput9_1_0` under `ifeq ($(OS),Windows_NT)` |
| 10 | `ui_input/overlay.c` | Per-widget CSS at USER+1 (light HUD text) |
| 11 | `ui_input/debug_terminal.c` | `term_normal` tag + widget CSS (light terminal text) |

---

## ⚠️ Known Limitations

**All platforms:**
- Scene save/load preserves bodies but not joints, robot assemblies, object
  IDs, sleep state, or cylinder half-length (R3-04).
- `physics_world` path has no containment walls (R3-07); sleeping bodies in it
  cannot be woken by contact (R3-06).
- Cylinder floor collision tests axle endpoints only — a fully tipped cylinder
  can fall through (NEW-01).
- Performance degrades above ~1136 objects (rendering-bound).
- Global state remains in the legacy GUI path; encapsulation deferred to v16.
- Full issue inventory: `the_list.txt`, `the_list_2.txt`, `the_list_3.txt`.

**Windows-specific:**
- Guide button (Xbox logo) not exposed by MinGW XInput headers — reads `false`.
- Mouse lock uses `gdk_seat_grab` on Win32; recentering is slightly less crisp
  than X11 (absorbed by the 80 px clamp).
- First launch may trigger SmartScreen (unsigned exe).
- `engine.exe` must run with `render/` and `status/` folders beside it —
  shaders load via relative paths. Red screen on launch = missing `render/`.

**Linux-specific:**
- Wayland mouse lock unsupported; X11 is forced via `GDK_BACKEND`.

---

## 🧩 Roadmap — v16 Modularisation

| Milestone | State |
|-----------|-------|
| **v15R2** | Config system + MFS merged |
| **v15R3 / MFS-W** | Windows-native port, FTC field builder, robot config presets *(current)* |
| **v15S** | Stabilisation — final merged release |
| **v16R1** | Begin splitting MFS out of the MPE mainframe |
| **v16+** | MPE kernel + module ecosystem |

Goal: run MPE standalone with no robotics present; MFS (or any future
ecosystem) drops in as a plugin module.

## 📜 Version History

- **MFS-W** (2026-09-07) — native Windows port: MSYS2/MinGW-w64 build,
  XInput gamepad, Win32 text/mouse/warp fixes, portable distribution zip.
- **v15R3** — cylinder collision campaign, headless test expansion,
  chassis-velocity odometry, FTC field builder.
- **v15R2** — configuration system hardening + MFS robotics merge.
- **v15R1** — centralised configuration system (69 tunables).
- See [`v15R3/evolution.txt`](v15R3/evolution.txt) for full lineage.
'''

# =====================================================================
# 2. how_to_use.md
# =====================================================================
HOW_TO_USE_MD = r'''# Miniature Physics Engine (MFS-W) — User Guide

### v15R3 Windows Build · also valid for Linux

---

## Starting the Engine

**Windows (prebuilt):** extract the release zip and double-click `engine.exe`.
If SmartScreen appears ("Windows protected your PC"), click **More info →
Run anyway** — the binary is unsigned but built from this source tree.

**Windows (from source):** in the MINGW64 terminal, `cd v15R3/src && ./engine.exe`

**Linux:** from the `src/` directory, run `./engine`

The window opens in Game Mode. Left-click inside the window to lock the mouse;
Escape releases it.

> ⚠️ **Windows folder layout:** `engine.exe` must sit next to the `render/`
> folder (shaders) and a `status/` folder (config + scene saves). A **red
> screen** on launch means `render/` is missing. See
> `install/windows/windows_build_instructions.md`.

---

## Modes

Toggle with `0` at any time.

**Game Mode** (default): camera gravity applies, WASD is grounded, you can
jump, and the world has boundaries (500×500×500 containment box).

**Debug Mode**: no boundaries, no camera gravity. WASD flies freely in the
look direction. Use for precise placement and inspection.

The current mode shows in the top-left status bar.

---

## Camera Controls

| Input | Game Mode | Debug Mode |
|---|---|---|
| Mouse | Look around | Look around |
| W A S D | Walk (grounded, inertia) | Fly in look direction |
| Space | Jump | Fly up |
| Shift | — | Fly down |
| I J K L | — | Steer camera (mouse-free) |
| M | — | Re-lock mouse cursor |
| Escape | Release mouse | Release mouse |

On Windows the mouse delta is clamped to ±80 px per frame to absorb Win32
cursor-warp spikes; normal mouse movement never hits the clamp.

---

## Spawning Objects

**Enter** spawns an object in front of the camera. Hold Enter ~0.3 s for
rapid fire.

Press `8` for spawner settings:

```
8 → Spawner Menu
1 → Sphere settings   (1: Mass  2: Radius)
2 → Cube settings     (1: Mass  2: Size)
3 → Toggle spawn type (sphere / cube)
```

---

## Selecting & Editing Objects

Right-click an object to select it (or `R` in Debug Mode for a camera raycast).

| Input | Action |
|---|---|
| `E` | Open/close object property menu |
| `F` | Apply impulse (launches object) |
| Middle-click | Delete object |
| `Delete` (Debug) | Remove selected object |

Object menu: `1` Mass · `2` Radius (spheres) · `3` Friction · `4` Immovable
toggle · `5` Mark for joint · `6` Link joint / colour · `7`/`8` Colour.

---

## Debug Terminal

Debug Mode → `T` (or `1`). The simulation is a virtual filesystem:

| Path | Contents |
|---|---|
| `/obj` | All rigid bodies |
| `/joint` | All spring joints |
| `/world` | Gravity, drag, friction |
| `/camera` | Camera state |
| `/spawner` | Spawner settings |

```
touch new.sph            # spawn a sphere
touch robot              # spawn the FTC robot
ln 1 2                   # spring-join objects 1 and 2
mv 3 /pos/0/10/0         # teleport object 3
chown 5.0 3              # set mass 5 kg
chmod static 3           # make immovable
kill -STOP 3             # sleep it
ps aux                   # list every body
export GRAVITY=-2.0      # change gravity
vi status/engine.cfg     # edit config in MicroVim
```

`help` lists all commands; `man <cmd>` shows usage. `Ctrl+L` clears, `Esc`
closes. Mutating commands need Debug Mode (or `sudo`).

---

## Configuration System (Key 6)

Live access to all tunables across 13 categories (World, Timestep, Sleep,
Solver, Depenetration, Broadphase, Joints, Boundary, Spawner, Body Defaults,
Camera, Render, UI). Parameters marked `[D]` require Debug Mode.

Config persists to `status/engine.cfg` (saved on exit, loaded on startup).
Terminal equivalents: `env`, `export KEY=value`, `config save|load|reset`.

`F11` randomises all tunables (torture test); restore with `config reset`.

---

## Validation Tests

| Key | Test |
|---|---|
| F5 | 10-cube stability stack |
| F6 | Sleeping cube + moving projectile (sleep/wake) |
| F7 | Editor torture: select, joint, delete, reset |
| F8 | Spawn stress: up to 300 mixed objects |
| F9 | Print validation report (incl. full config dump) |
| F10 | Long-run validation: 3600 ticks (60 s) idle stability |
| F11 | Config torture test |

---

## World Settings (Key 7) & Scene Menu (Key 9)

```
7 → 1: Spawning (launch velocity, friction)
    2: Viewpoint (move speed, jump height)
    3: World (gravity, drag, floor friction)

9 → 1: Save scene      (status/scene.dat)
    2: Load scene
    3: Clear scene
    4: Save config
    5: Reset config
    6: Exit
```

**Save/load limitations:** bodies are preserved; joints, robot assemblies,
object IDs, sleep state, and cylinder half-length are **not**. A saved cylinder
loads back as a sphere (tracked as R3-04).

---

## FTC Robot (MFS)

### Spawning

Debug terminal → `touch robot`. Spawns a 4-wheel mecanum robot at
(5, rest height, 5) with goBILDA 5203 30:1 motors. An orange nose sphere
marks the heading (+Z local).

### Driving — GAMEPAD ONLY

| Input | Action |
|---|---|
| Left stick up/down | Forward / backward |
| Left stick left/right | Strafe |
| Right stick left/right | Rotate |

> **Windows:** the controller must be XInput-compatible. For a Logitech F310,
> set the mode switch on the back to **X**. Plug the controller in **before**
> launching — hot-plug is not detected until restart.
>
> **Linux:** the controller must expose `/dev/input/js0` (evdev). If permission
> is denied: `sudo usermod -aG input $USER`, then log out and back in.

The keyboard drive keys (G/B/V/N/C/H) were **removed** in MFS_159. The gamepad
is the sole drive input so students build real driving muscle memory.

The robot HUD (top-left) shows battery voltage and average wheel RPM.

### Physics model

- Wheels: cylinders (r = 0.05 m, half-width 0.02 m), revolute-jointed
- Mecanum strafe via anisotropic ±45° roller friction at the contact solver
- Motors: BackEMF, Kt/Kv, gear ratio, thermal accumulation
- Battery: 12.8 V nominal, 0.015 Ω internal resistance, voltage sag under load
- Traction: torque → ground force clamped by floor friction (config-driven)
- Odometry: chassis-velocity integration + yaw rate (world frame)
- Timestep: fixed 60 Hz accumulator (deterministic)

---

## Physics Reference

SI units throughout. Defaults: sphere 1 kg / r 0.5 m / e 0.5; cube 2 kg /
half-extent 0.5 m; gravity −9.81 m/s²; drag 0.99; 60 Hz fixed step; 16 solver
iterations; sleep below 0.05 m/s and 0.01 rad/s after 1 s.

---

## Installation

### Windows (MSYS2 / MinGW-w64)

See `install/windows/windows_build_instructions.md` for the full walkthrough
(toolchain install, port scripts, build, DLL bundling, troubleshooting).

### Linux (Ubuntu 24.04 LTS)

```bash
sudo apt install gcc make libgtk-3-dev libepoxy-dev
cd src && make && ./engine
```

Other distributions: `install/linux/linux_install_instructions.md`.

---

## Known Limitations

- **Windows:** unsigned exe (SmartScreen warning); Guide button unreadable;
  mouse recentering slightly less crisp than X11; `render/` and `status/`
  must sit beside the exe.
- **Linux/Wayland:** mouse lock requires X11 (forced automatically).
- **Object count:** performance degrades above ~1136 objects (render-bound).
- **Scene format:** joints/IDs/sleep/cylinders not persisted (v2 planned).

For release validation follow `RELEASE_GATES.md` and `VALIDATION_WINDOWS.md`.
'''

# =====================================================================
# 3. RELEASE_GATES.md
# =====================================================================
RELEASE_GATES_MD = r'''# MFS-W Release Gates (v15R3 Windows Build)

This document defines the exit criteria for tagging the MFS-W Windows release.
Gates 1–13 are inherited from v15R2 (verified on Linux). **Gate 15 is new and
Windows-specific.** Gate 14 (FTC robotics) carries over with gamepad-only drive.

---

## Gate Rules

- **P0** — mandatory. Any P0 failure blocks the tag.
- **P1** — strongly recommended; may be deferred only if documented as a known
  limitation that does not undermine stability.
- **P2/P3** — optional; record as post-stable work.

---

## Mandatory P0 Gates (inherited — verified on Linux, v15R2)

### 1. Release Freeze
- [X] Release policy present and acknowledged
- [X] Only correctness, stability, platform-port, validation, docs, hygiene changes accepted

### 2. Build (Linux)
- [X] `make clean && make` succeeds, zero errors
- [X] Warnings reviewed and understood

### 3. Startup
- [X] Correct version string printed
- [X] Config system initialises; shaders load; window/grid/overlay render

### 4. Shader / Render Failure Visibility
- [X] Compile/link/missing-file failures reported; no silent broken render state

### 5. Input and Lifecycle
- [X] Close quits; config saved on clean exit
- [X] Mouse lock acquire/release; focus loss clears stuck state
- [X] Config menu (6) does not interfere with menus 7/8/9

### 6. Editor Stability
- [X] Select/delete/jointed-delete/marked-delete do not crash
- [X] Save/load with menus open does not crash

### 7. Physics Stability
- [X] Objects rest without jitter; cubes stack; restitution/friction work
- [X] Sleep/wake correct; no NaNs after normal use or stress

### 8. Broadphase / Solver Visibility
- [X] Node/pair/manifold/dedupe overflow visible in overlay and F9

### 9. Validation Tests (Linux)
- [X] F5–F11 pass; engine idles minutes without explosion

### 10. Configuration System
- [X] 69 tunables editable via menu and terminal
- [X] Save/load round-trip; corrupt file does not crash; clamping works

### 11. Documentation
- [X] README, user guide, and gates match the code (regenerated 2026-09-07)

### 12. Repository Hygiene
- [X] Build artifacts untracked; `.gitignore` present

### 13. Sanitizer Validation (Linux)
- [X] ASan+UBSan build available; validation passes; no severe reports

---

## P0 Gate 14 — FTC Robotics (MFS)

- [X] `ftc_robot_create` spawns chassis + 4 cylinder wheels with revolute joints
- [X] Mecanum strafe via real anisotropic roller friction (no chassis cheat)
- [X] Tank drive via motor torque → wheel traction
- [X] Motor model: BackEMF, gear ratio, Kt/Kv correct
- [X] Battery voltage sag under multi-motor load
- [X] Headless tests pass on Linux (`python3 tools/test_runner.py`)
- [X] Robot visible in GUI via proxy sync
- [X] Robot drivable via **gamepad only** (MFS_159; keyboard drive removed)
- [X] Fixed-timestep accumulator (60 Hz deterministic)
- [X] Revolute axis drift corrected (Baumgarte)
- [ ] Scene save/load preserves robot assemblies *(deferred)*
- [ ] Sensors (encoders/IMU/distance) exposed to user *(deferred)*
- [ ] FTC HAL (HardwareMap, OpMode) *(deferred)*

---

## P0 Gate 15 — Windows Platform (MFS-W)  ← NEW

### 15a. Build
- [X] MSYS2 MINGW64 toolchain builds `engine.exe` with zero errors
      (gcc, gtk3, libepoxy, pkg-config; `-lxinput9_1_0` linked)
- [X] `windows_port.py` + `fix_text_visibility.py` apply cleanly and are idempotent
- [X] Linux build unaffected by all WIN_PORT guards (same tree builds both)

### 15b. Packaging
- [X] `engine.exe` runs from File Explorer with bundled DLLs (no MSYS2 needed)
- [X] Shaders load via relative `render/` folder (no red screen)
- [X] `status/` created/used for `engine.cfg` and `scene.dat`
- [ ] Release zip verified on a **clean machine** with no MSYS2 installed

### 15c. Input
- [X] XInput gamepad detected at startup (F310 in X mode)
- [X] Left stick: forward/back correct polarity (WIN_PORT invert flip)
- [X] Left stick: strafe direction correct
- [X] Right stick X: rotate correct
- [X] Mouse look stable — no warp-induced sensitivity spikes (±80 px clamp)
- [X] Mouse lock acquire/release works under Win32 GDK

### 15d. Presentation
- [X] HUD overlay text readable (per-widget CSS, USER+1 priority)
- [X] Debug terminal text readable, including untagged output (`term_normal`)
- [X] Menus, crosshair, and robot HUD render correctly

### 15e. Verification debt (must clear before tagging)
- [ ] F5–F11 run on Windows, results recorded
- [ ] Headless test suite runs under MinGW (`python3 tools/test_runner.py`)
- [ ] Scene save (`9`) / load round-trip on Windows paths
- [ ] Config persistence: change → exit → relaunch → value survives
- [ ] 10-minute idle soak on Windows (no crash, no NaN)

---

## Recommended P1 Gates

- [X] Saving/loading scenes works (Linux); failure reported to user
- [X] CPU usage drops when scene sleeps; stress scenes remain usable
- [ ] Windows: ASan-equivalent validation (MinGW sanitizer support is limited —
      document as known limitation if skipped)

---

## Release Decision

MFS-W may be tagged only when:

1. All P0 gates 1–15 pass (15e verification debt cleared),
2. P1 gates pass or are documented as known limitations,
3. The clean-machine zip test passes,
4. The repository tree is clean,
5. `release_notes_v15R3W.md` records honest results.

If any mandatory gate fails: fix → re-run validation → re-evaluate. Never tag
around a red gate.
'''

# =====================================================================
# 4. RELEASE_POLICY.md
# =====================================================================
RELEASE_POLICY_MD = r'''# MFS-W Release Policy (v15R3 Windows Build)

This tree is in **v15R3 / MFS-W development** — the native Windows port of the
merged MPE+MFS tree.

## Cycle Goal

Deliver a Windows-native build that a classroom can run by double-clicking,
without touching a terminal, while keeping the Linux build path byte-identical.

This cycle covers:

1. Native Win64 build via MSYS2 / MinGW-w64 (no WSL, no Wine).
2. XInput gamepad support alongside the existing Linux evdev path.
3. Win32 presentation fixes (text visibility, mouse-warp stability).
4. Portable distribution packaging (DLL + shader bundling).
5. Documentation regenerated to match the dual-platform reality.

## Change Classes Accepted

During MFS-W development:

1. **Platform port code** — strictly behind `#ifdef _WIN32` / `#ifdef __linux__`,
   tagged `WIN_PORT`. Linux behaviour must not change.
2. Correctness fixes required by the port (build system, POSIX shims).
3. Packaging and distribution tooling (`windows_port.py`,
   `fix_text_visibility.py`, `generate_docs.py`, bundling scripts).
4. Documentation updates matching the new architecture.
5. Validation additions for the Windows platform (Gate 15).
6. Carried-over FTC robotics and cylinder-physics work from v15R3.

## Explicitly Rejected During This Cycle

- Any `#ifdef`-free platform assumption leaking into shared code paths.
- New physics features that have not passed the Linux headless suite first.
- Full global-state removal, multithreading, CCD, scene format v2, GTK4 —
  all remain deferred to v15S/v16.

## Platform Port Rules

1. Every platform difference gets a `WIN_PORT` marker comment.
2. The same source tree must build on both platforms without edits.
3. Fix scripts must be idempotent and create `.pre_winport` backups.
4. A Windows-only fix may never change a Linux code path; verify with a
   Linux build after every port script run.

## Release Goal

MFS-W may be tagged when:

- All P0 gates pass, **including Gate 15 (Windows Platform)** with the 15e
  verification debt cleared,
- The distribution zip runs on a clean Windows machine with nothing installed,
- The config system round-trips on Windows (save → restart → load),
- Physics behaviour at defaults matches the Linux build,
- Documentation (README, how_to_use, gates, policy, release notes, platform
  port reference) matches the shipped code.
'''

# =====================================================================
# 5. release_notes_v15R3W.md
# =====================================================================
RELEASE_NOTES_W_MD = f'''# MFS-W — Native Windows Release Notes

| | |
|---|---|
| **Internal designation** | `MFS-W` (Windows port of `v15R3`) |
| **Port date** | {DATE} |
| **Base tree** | v15R3 (post-Milestone-2, cylinder campaign complete) |
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

## Verified on Windows ({DATE})

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

- **MFS-W** ({DATE}) — native Windows port *(this release)*
- **v15R3** — cylinder collision campaign, headless tests, chassis-velocity
  odometry, FTC field builder, robot config presets
- **v15R2** — config system hardening + MFS merge
- **v15R1** — centralised configuration system

---

*Release gates: `RELEASE_GATES.md` · Policy: `RELEASE_POLICY.md` ·
Platform reference: `../docs/PLATFORM_PORT.md`*
'''

# =====================================================================
# 6. Windows build instructions
# =====================================================================
WINDOWS_BUILD_MD = r'''# Windows Build Instructions (MSYS2 / MinGW-w64)

Native Win64 build of MFS. No WSL, no Wine, no Visual Studio.

---

## 1. Install MSYS2

Download the installer from https://www.msys2.org and run it. Accept defaults
(installs to `C:\msys64`).

## 2. Install the toolchain

Open the **MINGW64** terminal (Start menu → "MSYS2 MINGW64").
Title bar must say `MINGW64` — not MSYS, not UCRT64, not CLANG64.

```bash
pacman -Syu
pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-gtk3 \
          mingw-w64-x86_64-libepoxy mingw-w64-x86_64-pkg-config \
          make python3
```

Verify:

```bash
gcc --version
pkg-config --cflags gtk+-3.0 epoxy    # should print -I flags
```

## 3. Get the source

**Recommended:** work inside the MSYS2 filesystem, not `/c/...` — the 9P
bridge to Windows drives is slow and makes compiles crawl.

```bash
# copy the extracted tree into your MSYS2 home
cp -r /c/Users/<you>/Downloads/Miniature_Physics_Engine-* ~/mpe
cd ~/mpe/v15R3/src
```

(Or `git clone` directly into `~/`.)

## 4. Apply the platform port

Two idempotent scripts, both create `.pre_winport` backups:

```bash
python3 windows_port.py          # fixes 1-9 (ifdefs, gamepad, makefile)
python3 fix_text_visibility.py   # fixes 10-11 (HUD + terminal text)
```

Preview first with `--dry-run` if desired. Re-running is safe (prints `[SKIP]`).

## 5. Build

```bash
make clean
make
```

Expect two harmless notes on first build if you skipped the scripts' latest
revision — see Troubleshooting below for the classic errors.

## 6. Run from the terminal

```bash
./engine.exe
```

## 7. Package for distribution

The exe needs GTK3's DLLs, the shaders, and a status folder beside it:

```bash
mkdir -p ../windows_release
cp engine.exe ../windows_release/
cp -r render ../windows_release/            # GLSL shaders (relative paths!)
mkdir -p ../windows_release/status          # engine.cfg / scene.dat

for dll in $(ldd engine.exe | grep mingw64 | awk '{print $3}'); do
    cp "$dll" ../windows_release/
done

ls ../windows_release | wc -l               # expect ~40-60 files
```

Nuclear option if a DLL is still missing:

```bash
cp /mingw64/bin/*.dll ../windows_release/
```

Zip `windows_release/` (Explorer → right-click → Compress to ZIP). Test the
zip on a machine **without** MSYS2 before handing it out.

---

## Test checklist (post-build)

| # | Test | Expected |
|---|------|----------|
| 1 | Double-click `engine.exe` from Explorer | Window opens, grid + HUD visible |
| 2 | HUD text | Near-white with dark halo, readable over bright scenes |
| 3 | Press `0`, then `T` | Terminal opens; plain output is light grey |
| 4 | `touch robot` | Robot spawns (blue chassis, 4 dark wheels, orange nose) |
| 5 | F310 in **X** mode, plugged in before launch | Left stick up = forward |
| 6 | Left stick sideways | Strafe (mecanum) |
| 7 | Right stick sideways | Rotate |
| 8 | Release sticks | Robot coasts to a stop and holds |
| 9 | Mouse look | Smooth — no violent single-frame snaps |
| 10 | Close window | Clean exit, `status/engine.cfg` written |

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `error: too many arguments to function 'mkdir'` | POSIX 2-arg mkdir on Windows | Run `windows_port.py` (fixes 3–4); or wrap with `#ifdef _WIN32 _mkdir(path) #endif` |
| `'XINPUT_GAMEPAD_GUIDE' undeclared` | MinGW xinput.h lacks the Guide constant | Set that button to `false` (windows_port.py handles it) |
| `warning: ignoring '#pragma comment'` | MSVC-only pragma | Harmless; remove the line (script does) |
| `undefined reference to XInputGetState` | XInput not linked | makefile needs `-lxinput9_1_0` (script fix 9); try `-lxinput` if not found |
| **Red screen** on launch | Shaders not found | `render/` folder must sit beside `engine.exe`; working directory matters |
| "DLL not found" from Explorer | GTK runtime missing | Bundle DLLs (step 7); nuclear option `cp /mingw64/bin/*.dll` |
| Drive forwards/backwards inverted | XInput Y polarity | `invert_left_y = false` on Windows (script fix 2) |
| Camera snaps violently | Win32 warp not atomic | ±80 px delta clamp (script fix 8) |
| All text black/unreadable | Windows GTK light theme | Run `fix_text_visibility.py` |
| Gamepad not detected | D mode / hot-plug | F310 switch → **X**; plug in before launch; check Windows "Game Controllers" |
| SmartScreen blocks exe | Unsigned binary | More info → Run anyway |
| Build works but slow | Source on `/c/` (9P bridge) | Move tree into `~/` inside MSYS2 |

## Rebuilding after code changes

```bash
cd ~/mpe/v15R3/src
make                        # incremental
cp engine.exe ../windows_release/   # DLLs unchanged, exe only
```
'''

# =====================================================================
# 7. WINDOWS_DISTRIBUTION.md (teacher one-pager)
# =====================================================================
WINDOWS_DIST_MD = r'''# MFS-W Classroom Distribution Guide

**For:** FTC club teachers and student mentors
**What you received:** a zip containing a self-contained physics/robot
simulator. No installation, no admin rights, no internet needed.

---

## What's in the zip

```
windows_release/
├── engine.exe        ← the simulator (double-click)
├── render/           ← shader files (do not delete)
├── status/           ← settings + saved scenes (auto-created if missing)
└── *.dll             ← ~50 runtime libraries (do not delete)
```

Extract the zip anywhere — Desktop, a USB stick, `C:\MPE\`. The whole folder
must stay together.

---

## Student quick-start (print this)

1. **Plug in the controller first.** Logitech F310: flip the switch on the
   back to **X**.
2. Double-click `engine.exe`.
   - If Windows SmartScreen appears: click **More info**, then **Run anyway**.
3. Press `0` (enters Debug Mode), then press `T` (opens the terminal).
4. Type `touch robot` and press Enter. A robot appears.
5. Drive:
   - **Left stick** — forward / back / strafe sideways
   - **Right stick (left/right)** — rotate
6. Close the terminal with `Esc`. Release the mouse with `Esc` too.
7. Close the window when done — settings save automatically.

---

## Classroom setup notes

- **One controller per machine**, plugged in **before** launching. Hot-plug
  is not detected until the simulator restarts.
- Any XInput controller works (Xbox 360/One/Series, F310 in X mode, most
  third-party pads in XInput mode).
- The robot only drives with the controller — this is deliberate. Students
  build the same stick habits they will use on the real robot.
- To reset everything to defaults: delete the `status/` folder and relaunch.

## Lesson ideas

| Exercise | How |
|---|---|
| Straight-line driving | Drive forward 3 s, note the HUD (battery V, RPM) |
| Mecanum strafe | Left stick purely sideways — watch the wheels |
| Precision parking | Spawn robot, drive to a grid line, stop within 10 cm |
| Spin control | Right stick taps — quarter turns |
| What the battery does | Watch voltage sag under full stick, recover on release |

Useful terminal commands (after `0`, `T`):

```
touch robot        spawn the robot
ps aux             list every physics object
help               full command list
```

## Troubleshooting (non-technical)

| Problem | Fix |
|---|---|
| "Windows protected your PC" | More info → Run anyway |
| Whole screen is red | The `render` folder is missing or moved — re-extract the zip |
| "DLL not found" error | Some `.dll` files were deleted — re-extract the zip |
| Robot won't move | Controller not plugged in before launch, or F310 is in D mode. Restart with controller in X mode |
| Robot drives backwards | Controller was plugged in after launch — restart |
| Text hard to read | Should not happen in this build; report it |
| Settings lost | Don't delete the `status` folder unless resetting on purpose |

## Support

This build is the MFS-W Windows port of the MPE v15R3 tree, generated
2026-09-07. Source, build instructions, and the full issue inventory live in
the project repository (`readme.md`, `install/windows/`, `the_list*.txt`).
'''

# =====================================================================
# 8. docs/PLATFORM_PORT.md
# =====================================================================
PLATFORM_PORT_MD = r'''# MPE/MFS Platform Port Reference

**Status:** Windows (Win64, MSYS2/MinGW-w64) ✅ 2026-09-07 · Linux ✅ (primary, unchanged)
**Marker:** every platform-specific change carries a `WIN_PORT` tag in source.

---

## Porting rules

1. **One tree, both platforms.** The same source builds on Linux and Windows
   with zero manual edits. Platform selection happens in the preprocessor and
   the makefile only.
2. **Guards, not forks.** All divergence lives behind `#ifdef _WIN32` /
   `#ifdef __linux__`. No `#ifdef` may change physics, solver, or math code —
   platform guards cover OS APIs, input backends, and presentation only.
3. **Linux stays byte-identical.** A Windows fix must never alter a Linux code
   path. Verify with a Linux build after every port script run.
4. **Markers.** Every guard block carries a `WIN_PORT_*` or `FIX_TEXT_VIS`
   comment so `grep -rn "WIN_PORT" src/` produces the complete inventory.
5. **Idempotent tooling.** Port scripts (`windows_port.py`,
   `fix_text_visibility.py`) detect prior application, skip cleanly, and
   create `.pre_winport` backups before first mutation.

---

## Change inventory

| # | Marker | File | Linux behaviour | Windows behaviour | Why |
|---|--------|------|-----------------|-------------------|-----|
| 1 | `WIN_PORT_X11_GUARD` | `root_gtk.c` | `g_setenv("GDK_BACKEND","x11")` forced | Guard skipped; GDK uses Win32 backend | X11 does not exist on Windows; mouse-lock workaround is Linux-only |
| 2 | `WIN_PORT_INVERT` | `root_gtk.c` | `invert_left_y = true` | `invert_left_y = false` | evdev reports stick-up negative; XInput reports it positive. Without the flip, drive is inverted |
| 3 | `WIN_PORT_SIM_INCLUDES` | `simulation.c` | `<sys/stat.h>` + `<unistd.h>` | `<direct.h>` + `<io.h>`; `mkdir(p,m)`→`_mkdir(p)`; `access`→`_access`; `F_OK` defined | Windows `mkdir` takes one argument; `unistd.h` does not exist |
| 4 | `WIN_PORT_CFG_INCLUDES` | `config/mpe_config.c` | as above | `_mkdir` shim | `ensure_parent_dir()` creates `status/` before saving config |
| 5 | `WIN_PORT_VR_INCLUDES` | `core/validation_report.c` | `<unistd.h>` | `<io.h>` + `_access` shim | F9 checks `status/engine.cfg` presence |
| 6 | `WIN_PORT_LR_INCLUDES` | `core/long_run_validation.c` | `<unistd.h>` | `<io.h>` + `_access` shim | F10 config-restore check |
| 7 | `WIN_PORT_GAMEPAD` | `ui_input/gamepad.c` | evdev: `open("/dev/input/js0")`, `struct js_event`, values /32767 | XInput: `XInputGetState(0)`, thumbs /32768, triggers /255, `wButtons` masks; Guide button hardcoded `false` | Completely different OS joystick APIs; `gamepad.h` ABI unchanged so no caller edits |
| 8 | `WIN_PORT_MOUSE_CLAMP` | `ui_input/input_control.c` | (clamp also applies — harmless) | Delta clamped ±80 px/frame | `gdk_device_warp` is not atomic on Win32; a warp can report a huge single-frame delta → violent camera snap |
| 9 | `WIN_PORT_LIBS` | `makefile` | `LIBS = … -lm` | `ifeq ($(OS),Windows_NT)` adds `-lxinput9_1_0` | XInput is a separate Windows library |
| 10 | `FIX_TEXT_VIS` | `ui_input/overlay.c` | (CSS also applies — harmless on dark themes) | Per-widget CSS providers at `USER+1`: `#E8EEF7` text + black text-shadow on all 7 HUD labels | Windows GTK theme defaults label text to black — invisible over the dark 3D scene |
| 11 | `FIX_TEXT_VIS` | `ui_input/debug_terminal.c` | as above | `term_normal` tag (`#CDD6E4`) for untagged output; entry/prompt CSS at `USER+1` | Plain terminal output (`ps aux`, `cat`) uses theme default → black-on-black |

---

## API equivalences used

| POSIX (Linux) | Win32 (MinGW) | Header |
|---|---|---|
| `mkdir(path, 0755)` | `_mkdir(path)` | `<direct.h>` |
| `access(path, F_OK)` | `_access(path, 0)` | `<io.h>` |
| evdev `js_event` | `XINPUT_STATE` | `<xinput.h>` |
| `gdk_seat_grab` (X11) | `gdk_seat_grab` (Win32 GDK) | same GTK API |
| `rename()` atomic replace | `rename()` (same semantics on NTFS for files) | `<stdio.h>` |

## Runtime layout differences

| Concern | Linux | Windows |
|---|---|---|
| Binary | `engine` | `engine.exe` |
| Runtime deps | system packages | ~50 DLLs bundled beside the exe |
| Shaders | relative `render/` from CWD (`src/`) | relative `render/` — must be copied into the release folder |
| Config/scene | `status/` beside CWD | `status/` beside the e |
| Joystick device | `/dev/input/js0` (needs `input` group) | XInput device 0 (no permissions needed) |
| Display backend | X11 forced | Win32 GDK automatic |

## Adding a new platform (future: macOS)

1. Create `install/<platform>/` build instructions.
2. Add a third branch to `gamepad.c` (e.g. IOKit HID or SDL).
3. Guard any POSIX-ism with the existing `_WIN32`/`__linux__` pattern extended
   to `__APPLE__`.
4. Add a Gate-15-equivalent platform section to `RELEASE_GATES.md`.
5. Write a `platform_port.py`-style idempotent script; never hand-edit.

## Verification

- `grep -rn "WIN_PORT" v15R3/src/` → complete guard inventory.
- Linux regression: `make clean && make && python3 tools/test_runner.py`
  after any port-script change.
- Windows regression: full checklist in
  `v15R3/install/windows/windows_build_instructions.md` +
  `v15R3/VALIDATION_WINDOWS.md`.
'''

# =====================================================================
# 9. VALIDATION_WINDOWS.md
# ===================================================================
VALIDATION_WINDOWS_MD = r'''# Windows Validation Checklist (Gate 15e)

Run every item below on the Windows build before tagging MFS-W stable.
Record results in `release_notes_v15R3W.md`. Date each run.

---

## A. In-engine validation suite (F5–F11)

Launch `engine.exe`, then:

| Key | Test | Pass criterion | Result |
|-----|------|----------------|--------|
| F5 | Stability stack | 10 cubes settle, no jitter/explosion | [ ] |
| F6 | Sleep/wake | Projectile wakes sleeping cube | ] |
| F7 | Editor torture | No crash; survivor remains | [ ] |
| F8 | Spawn stress | 300 objects, usable framerate, no NaN | [ ] |
| F9 | Validation report | Prints to console with config dump | [ ] |
| F10 | Long-run (60 s) | Console prints `result: PASS` | [ ] |
| F11 | Config torture | Survives 60 s; restore with `config reset` | [ ] |

## B. Headless test suite under MinGW

In the MINGW64 terminal:

```bash
cd ~/mpe/v15R3/src
python3 ../tools/test_runner.py       # or: python3 tools/test_runner.py from root
```

All 14 tests must PASS (two_world, revolute, teleop_drive, mecanum_drive,
cylinder_drop, driven_wheel, math3_inverse, ftc_integration, physics_truth,
tank_turn, odometry_accuracy, cylinder_sphere, cylinder_cube,
cylinder_cylinder).

- [ ] Suite builds under MinGW (makefile test targets use `find`/gcc — verify)
- [ ] All 14 tests pass
- [ ] `physics_truth` assertions identical to Linux results

Note: if test targets fail to build on MinGW, record the exact error — do not
silently skip. Gate 15euires this green or an explicitly documented
limitation.

## C. Persistence round-trips

| Test | Procedure | Pass criterion | Result |
|------|-----------|----------------|--------|
| Config save/load | Change gravity via menu `6`, exit, relaunch | New gravity active; `[config] loaded` on console | [ ] |
| Config corrupt-file | Truncate `status/engine.cfg` mid-line, relaunch | Engine starts on defaults, no crash | [ ] |
| Scene save | Spawn objects, press `9` → 1 | `status/scene.dat` written | [ ] |
| Sce load | `9` → 2 | Objects restored (positions/velocities/colours) | [ ] |
| Status dir creation | Delete `status/`, relaunch | Folder recreated, no crash | [ ] |

## D. Input soak

| Test | Procedure | Pass criterion | Result |
|------|-----------|----------------|--------|
| Gamepad cold-start | Plug F310 (X), launch | `[gamepad] XInput device 0 connected` on console | [ ] |
| Gamepad absent | Launch with no controller | Engine runs; `[gamepad] no XInput device found`; no crash | [ ] |
| Drive polarity Full stick each direction | Forward/up, strafe/left, rotate match stick | [ ] |
| Mouse soak | 2 min of fast mouse-look | No camera snaps, no drift-to-edge | [ ] |
| Focus loss | Alt-Tab mid-drive | Robot stops receiving input; no stuck keys | [ ] |

## E. Stability soak

| Test | Procedure | Pass criterion | Result |
|------|-----------|----------------|--------|
| 10-min idle | Spawn robot, leave it | No crash, no NaN, robot holds position | [ ] |
| 10-min drive | Continuous driving + strafing | No crash; battery sags; wheels stay attached | [ ] |
| Exit cleanliness | Close window during drive | Clean exit; `engine.cfg` saved | [ ] |

## F. Clean-machine test

1. Copy `windows_release.zip` to a Windows PC **without MSYS2**.
2. Extract, double-click `engine.exe`.
3. Run sections A (F5, F10 only), C, D.
- [ ] Passes with zero missing-DLL errors

---

## Recording results

Append a dated block to `release_notes_v15R3W.md`:

```
### Windows validation run — YYYY-MM-DD
A: F5-F11  → x/7 pass
B: headless → xss
C: persistence → x/5
D: input → x/5
E: soak → x/3
F: clean machine → pass/fail
Verdict: READY TO TAG / NOT READY (reasons)
```

Any FAIL blocks the tag. Fix, re-run the whole section, re-evaluate.
'''

# =====================================================================
# Patches: evolution.txt and scope.md
# =====================================================================
def patch_evolution() -> bool:
    p = V15R3 / "evolution.txt"
    if not p.exists():
        log("[WARN] evolution.tound"); return False
    c = p.read_text(encoding="utf-8", errors="replace")
    if "MFS-W" in c:
        log("[SKIP] evolution.txt already has MFS-W"); return True
    if DRY_RUN:
        log("[DRY-RUN] would patch evolution.txt"); return True
    c = c.replace("Current Head: MFS (post-Milestone-2)",
                  "Current Head: MFS-W (native Windows port of post-Milestone-2 tree)")
    if not c.endswith("\n"):
        c += "\n"
    c += f"MFS-W:          Native Windows port (MSYS2/MinGW-w64, XInput, WIN_PORT guards) \u2705 {DATE}\n"
    bak = p.with_suffix(".txt.pre_docs")
    if not bak.exists():
        shutil.copy2(p, bak)
    p.write_text(c, encoding="utf-8")
    log("[OK] evolution.txt patched (MFS-W milestone + head)")
    return True

def patch_scope() -> bool:
    p = ROOT / "scope.md"
    if not p.exists():
        log("[WARN] scope.md not found"); return False
    c = p.read_text(encoding="utf-8", errors="replace")
    if "PLAT-003 | Windows | RESOLVED" in c:
        log("[SKIP] scope.md PLAT-003 already resolved"); return True
    lines = c.split("\n")
    hit = False
    for i, ln in enumerate(lines):
        if ln.strip().startswith("| PLAT-003"):
            lines[i] = ("| PLAT-003 | Windows | RESOLVED 2026-09-07 | Native Windows build via "
                        "MSYS2/MinGW-w64 (MFS-W). All platform code behind WIN_PORT guards. "
                        "See docs/PLATFORM_PORT.md. | **Low** | root_gtk.c, gamepad.c, makefile |")
            hit = True
            break
    if not hit:
        log("[WARN] PLAT-003 row not found in scope.md"); return True
    if DRY_RUN:
        log("[DRY-RUN] would patch scope.md PLAT-003"); return True
    bak = p.with_suffix(".md.pre_docs")
    if not bak.exists():
        shutil.copy2(p, bak)
    p.write_text("\n".join(lines), encoding="utf-8")
    log("[OK] scope.md PLAT-003 marked RESOLVED")
    return True

# =====================================================================
# Main
# =====================================================================
def main():
    print("=" * 66)
    print("  MFS-W Documentation Generator")
    print("=" * 66)
    print(f"  ROOT  = {ROOT}")
    print(f"  V15R3 = {V15R3}")
    if DRY_RUN:
        print("  ** DRY RUN — no files will be written **")
    print()

    jobs = [
        ("1/11 readme.md (root)",
         lambda: write_doc(ROOT / "readme.md", README_MD, "readme.md")),
        ("2/11 how_to_use.md",
         lambda: write_doc(V15R3 / "how_to_use.md", HOW_TO_USE_MD, "how_to_use.md")),
        ("3/11 RELEASE_GATES.md",
         lambda: write_doc(V15R3 / "RELEASE_GATES.md", RELEASE_GATES_MD, "RELEASE_GATES.md")),
        ("4/11 RELEASE_POLICY.md",
         lambda: write_doc(V15R3 / "RELEASE_POLICY.md", RELEASE_POLICY_MD, "RELEASE_POLICY.md")),
        ("5/11 release_notes_v15R3W.md",
         lambda: write_doc(V15R3 / "release_notes_v15R3W.md", RELEASE_NOTES_W_MD, "release_notes_v15R3W.md")),
        ("6/11 windows_build_instructions.md",
         lambda: write_doc(V15R3 / "install" / "windows" / "windows_build_instructions.md",
                           WINDOWS_BUILD_MD, "install/windows/windows_build_instructions.md")),
        ("7/11 WINDOWS_DISTRIBUTION.md",
         lambda: write_doc(V15R3 / "WINDOWS_DISTRIBUTION.md", WINDOWS_DIST_MD, "WINDOWS_DISTRIBUTION.md")),
        ("8/11 PLATFORM_PORT.md",
         lambda: write_doc(ROOT / "docs" / "PLATFORM_PORT.md", PLATFORM_PORT_MD, "docs/PLATFORM_PORT.md")),
        ("9/11 VALIDATION_WINDOWS.md",
         lambda: write_doc(V15R3 / "VALIDATION_WINDOWS.md", VALIDATION_WINDOWS_MD, "VALIDATION_WINDOWS.md")),
        ("10/11 evolution.txt patch", patch_evolution),
        ("11/11 scope.md patch", patch_scope),
    ]

    failed = []
    for name, fn in jobs:
        log(f"--- {name} ---")
        try:
            if not fn():
                failed.append(name)
        except Exception as e:
            log(f"[FAIL] {name}: {e}")
            failed.append(name)

    print()
    print("=" * 66)
    if failed:
        print(f"  {len(failed)} job(s) need attention:")
        for f in failed:
            print(f"    - {f}")
    else:
        print("  All 11 documentation jobs complete.")
        print()
        print("  Generated tree:")
        print("    readme.md                                  (replaced)")
        print("    scope.md                                   (PLAT-003 patched)")
        print("    docs/PLATFORM_PORT.md                      (new)")
        print("    v15R3/how_to_use.md                        (replaced)")
        print("    v15R3/RELEASE_GATES.md                     (replaced, Gate 15 added)")
        print("    v15R3/RELEASE_POLICY.md                    (replaced)")
        print("    v15R3/release_notes_v15R3W.md              (new)")
        print("    v15R3/WINDOWS_DISTRIBUTION.md              (new)")
        print("    v15R3/VALIDATION_WINDOWS.md                (new)")
        print("    v15R3/evolution.txt                        (patched)")
        print("    v15R3/install/windows/windows_build_instructions.md (new)")
        print()
        print("  Backups: *.pre_docs beside every replaced file.")
    print("=" * 66)
    return 1 if failed else 0

if __name__ == "__main__":
    sys.exit(main())
