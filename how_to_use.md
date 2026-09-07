# Miniature Physics Engine (MFS-W) — User Guide

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
