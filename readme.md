# 🧊 MINIATURE PHYSICS ENGINE — WINDOWS BUILD (MFS-W)

> **What this tree is:** This is MFS (MPE FTC Simulator) compiled and packaged
> for **native Windows**. It retains the full MPE physics kernel plus the FTC
> robotics layer. Linux remains supported from the same source tree — every
> platform difference is behind `#ifdef _WIN32` guards marked `WIN_PORT`.
>
> **No WSL. No Wine. No emulation.** This is a real Win64 executable built
> with MinGW-w64 GCC, linking GTK3 and OpenGL 3.3 natively.

**License:** GPL-3.0 · **Language:** C · **UI:** GTK3 · **Renderer:** OpenGL 3.3 Core

---

## 🚀 Quick Start (Windows — Prebuilt)

If you received a `windows_release.zip`:

1. Extract the zip anywhere (e.g. `C:\MPE\`)
2. Double-click `engine.exe`
3. Press `0` (enter Debug Mode), then `T` (open terminal)
4. Type `touch robot` and press Enter
5. Plug in a gamepad (see below) and drive

**That's it.** No installation, no dependencies, no command line.

### 🎮 Gamepad (Required for Driving)

| | |
|---|---|
| Supported | Logitech F310 (and any XInput-compatible controller) |
| **Critical** | Set the mode switch on the back of the F310 to **X** (XInput), not D |
| Left stick | Forward / backward / strafe |
| Right stick X | Rotate |
| Connection | Plug in **before** launching, or the gamepad won't be detected |

Keyboard drive keys (G/B/V/N/C/H) were removed in MFS_159 — the gamepad is
the sole drive input, matching how students will drive the real robot.

---

## 🔨 Building from Source (Windows)

### 1. Install MSYS2

Download from https://www.msys2.org and install.

### 2. Install the toolchain

Open the **MINGW64** terminal (not MSYS, not UCRT64):

```bash
pacman -Syu
pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-gtk3 \
          mingw-w64-x86_64-libepoxy mingw-w64-x86_64-pkg-config \
          make python3
```

### 3. Apply the Windows port (one-time)

From `v15R3/src/`:

```bash
python3 windows_port.py        # all 8 platform fixes
python3 fix_text_visibility.py # readable HUD/terminal text
```

Both scripts are idempotent and create `.pre_winport` backups.

### 4. Build

```bash
cd v15R3/src
make clean
make
./engine.exe
```

### 5. Package for distribution

The exe needs GTK3's DLLs and the shader files next to it:

```bash
mkdir -p ../windows_release
cp engine.exe ../windows_release/
cp -r render ../windows_release/          # shaders (GLSL)
mkdir -p ../windows_release/status        # engine.cfg / scene.dat live here

# bundle every dependent DLL
for dll in $(ldd engine.exe | grep mingw64 | awk '{print $3}'); do
    cp "$dll" ../windows_release/
done
```

Zip `windows_release/` and distribute. Recipients need nothing installed.

> ⚠️ **Working directory matters:** the engine loads shaders via relative
> paths (`render/shaders/*.glsl`) and writes config to `status/`. Both
> folders must sit next to `engine.exe`. A red screen on launch means the
> `render/` folder is missing.

---

## 🔨 Building from Source (Linux)

Unchanged from upstream MPE:

```bash
sudo apt install build-essential pkg-config libgtk-3-dev libepoxy-dev
cd v15R3/src
make clean && make
./engine
```

X11 is still forced on Linux (mouse lock requirement). The `WIN_PORT`
guards leave Linux behaviour byte-identical.

---

## 🪟 What the Windows Port Changed

Every change is tagged `WIN_PORT` in the source and lives behind
`#ifdef _WIN32` — the Linux build path is untouched.

| # | File | Change | Why |
|---|------|--------|-----|
| 1 | `root_gtk.c` | X11 backend force → `#ifdef __linux__` | GDK on Windows uses the Win32 backend |
| 2 | `root_gtk.c` | Gamepad Y-inversion flipped on Windows | XInput reports stick-up as **positive**; evdev reports it as negative |
| 3 | `simulation.c` | `mkdir`/`access` → `_mkdir`/`_access` | POSIX two-arg `mkdir` doesn't exist on Windows |
| 4 | `config/mpe_config.c` | same `mkdir` guard | config dir creation |
| 5 | `core/validation_report.c` | `access()` guard | F_OK check for engine.cfg |
| 6 | `core/long_run_validation.c` | `access()` guard | F10 config restore check |
| 7 | `ui_input/gamepad.c` | **Full dual-platform rewrite** | Windows branch uses XInput (`XInputGetState`); Linux branch keeps evdev (`/dev/input/js0`) |
| 8 | `ui_input/input_control.c` | Mouse delta clamped to ±80 px/frame | `gdk_device_warp` isn't atomic on Win32 — unclamped deltas caused violent camera snaps |
| 9 | `makefile` | `-lxinput9_1_0` under `ifeq ($(OS),Windows_NT)` | XInput link |
| 10 | `ui_input/overlay.c` | Per-widget CSS at `USER+1` priority | Windows GTK theme renders label text black-on-dark-scene |
| 11 | `ui_input/debug_terminal.c` | `term_normal` tag + widget CSS | Plain terminal output was black-on-black |

### Known Windows quirks

- **Guide button** (Xbox logo) is not exposed by MinGW's XInput headers —
  always reads `false`. Not used for anything.
- **Mouse lock** uses `gdk_seat_grab` on Win32. It works, but cursor
  recentering is slightly less crisp than X11. The 80px clamp absorbs it.
- **First launch** may be flagged by SmartScreen ("Windows protected your
  PC") — the exe is unsigned. Click *More info → Run anyway*.

---

## 🎮 Controls

### Camera & Mode

| Action | Input |
|---|---|
| Move | `W A S D` |
| Look | Mouse (left-click to lock) |
| Jump / fly up | `Space` |
| Fly down (Debug) | `Shift` |
| Release mouse | `Escape` |
| Toggle Game / Debug | `0` |

### Robot (MFS)

| Action | Input |
|---|---|
| Spawn robot | Terminal (`T`): `touch robot` |
| Drive / strafe | Left stick |
| Rotate | Right stick X |

### Everything Else

Spawning (`Enter`, `8`), selection (right-click, `E`, `F`), world settings
(`7`), scene save/load (`9`), config menu (`6`), terminal (`T`), and the
validation suite (`F5`–`F11`) are identical to upstream MPE. See
[how_to_use.md](v15R3/how_to_use.md).

---

## ⚙️ Engine (unchanged from MPE v15R2/v15R3)

- Semi-implicit Euler integration, fixed 60 Hz timestep, 5-substep cap
- Spatial-hash broadphase with adaptive cell sizing
- Warm-starting sequential impulse solver (16 iterations)
- Sphere–sphere, sphere–OBB, OBB–OBB (SAT + Sutherland–Hodgman),
  cylinder–sphere, cylinder–cube, cylinder–cylinder narrowphase
- GPU-instanced rendering, custom GLSL Phong
- 69-tunable config registry, persistent to `status/engine.cfg`
- POSIX-style debug terminal + MicroVim editor
- FTC robotics: mecanum via real anisotropic roller friction, DC motor
  model (BackEMF/Kt/Kv/gear ratio), battery sag, revolute wheel joints,
  chassis-velocity odometry

## ⚠️ Known Limitations (all platforms)

- Scene save/load preserves bodies but not joints, robot assemblies,
  object IDs, or sleep state
- `physics_world` path has no containment walls; the legacy GUI path does
- Sleeping bodies in `physics_world` cannot be woken by contact (R3-06)
- Cylinder floor collision tests axle endpoints only — a fully tipped
  cylinder can fall through (NEW-01)
- Full issue inventory: `the_list.txt`, `the_list_2.txt`, `the_list_3.txt`

---

## 📜 Version

- **This tree:** MFS-W (Windows port of v15R3, 2026-09-07)
- **Base:** v15R3 development / v15R2 config-system cycle
- **Lineage:** see `v15R3/evolution.txt`
