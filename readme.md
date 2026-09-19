# 🤖 MFS — MPE FTC SIMULATOR

> **MFS** (Miniature Physics Engine FTC Simulator) is a physics-first FTC robotics simulator built on the MPE physics kernel. It simulates a ~6 kg competition-weight mecanum robot (19.2:1 goBILDA 5203s, 96 mm wheels) with **honest physics** — no cheat forces, every newton arrives through documented physics.

**License:** GPL-3.0 · **Language:** C · **UI:** GTK3 · **Renderer:** OpenGL 3.3 Core · **Base:** MPE v15R3

---

## 📋 Overview

MFS is a **custom-built 3D rigid-body physics engine and real-time rendering pipeline**, written entirely in **C**, specialized for **FTC robotics simulation**. It runs on a **zero-dependency core** — the only external requirements are **GTK3** (windowing/UI) and **OpenGL** (render backend).

MFS is built around four priorities:

- **Mathematical transparency** — every integrator, solver, and collision test is hand-written and inspectable.
- **Cache-efficient data layouts** — tightly packed structs and contiguous instance buffers.
- **Deterministic simulation** — fixed-timestep physics decoupled from render framerate.
- **Real-time scaling** — GPU instancing and an O(N) spatial-hash broadphase.

---

## 🎯 What is MFS?

MFS simulates a real FTC robot through **honest physics**:

- **DC motor model** — stall/free-speed-derived Kt/Ke with gearbox Coulomb drag, 16× electrical subcycling, battery sag with 30 A foldback, regen-aware drain.
- **Anisotropic roller contact** — rubber grip axis at the pair's own Coulomb μ plus a near-free roller axis (bearing drag), decoupled clamps. Mecum sign fix: `cross(axle_proj, floor_normal)` orients the roller frame correctly; forward/strafe/rotate all work through honest wheel torque.
- **Honest odometry** — wheel-encoder translation (slip shows as error) + IMU-model heading.
- **Robot-relative sticks** — right means the robot's right at any heading; an orange nose wireframe marks the +Z face, and the default camera starts behind the robot.
- **No cheat forces** — strafe/rotate authority arrives purely through wheel torques on the anisotropic roller frame; no chassis forces anywhere.
- **Aligned traction clamp** — `roller_friction_coeff = 0.8` matches `floor_friction_s`; drivetrain and solver agree on max traction.
- **SDK-standard roller angles** — FL +45°, FR -45°, BL -45°, BR +45°; +strafe drives +X (robot-left).

---

## ✨ What's New in v15R3 (MFS Release)

`v15R3` is the configuration-system release with live FTC/MFS robotics in `v15R3/src/robotics/`. Highlights:

- **Domain-driven architecture** — clean `core`, `physics`, `render`, `scene`, `ui_input`, `robotics` modules.
- **Warm-starting contact solver** with multi-point Sutherland–Hodgman manifolds for stable stacking.
- **Full constraint framework** — revolute, fixed, prismatic, distance, and rope constraints plus spring joints (springs + revolutes persist in scene v200; all types live in the headless suite and the TUI demo).
- **3D spatial-hash grid broadphase** with adaptive cell sizing.
- **Interactive spring-joint system** with live magenta rendering.
- **POSIX-style debug terminal** — drive the whole simulation from a shell.
- **Built-in validation suite** (F5–F11), including a 60-second long-run stability test and config torture test.
- **Shader/render failure visibility** — the engine no longer continues silently in a broken render state.
- **Physics-truth pass** — Verlet-exact free flight, post-integration Poisson gate, CCD remainder integration, strict warm-start, true cylinder SDF geometry, no velocity clamps or restitution caps; game-only damping (`nice_value`, angular scale) labeled and defaulted off/vacuum.
- **Sleep truth** — three-gate wake (first-touch pair novelty, fast-other, deep overlap): slow pushers wake sleepers at any speed, resting stacks proceed to sleep and stay settled (F10 root cause, fixed).
- **Terminal debugger + output suite** — `mpe-tui`: live ncurses inspector (bodies, joints, constraints, math, scene graph) plus pipeable `--snapshot`/`--stream` state dumps.
- **Mecum sign fix** — anisotropic roller frame cross-product order corrected (`cross(axle_proj, floor_normal)`); forward/strafe/rotate now drive correctly without cheat forces.
- **Revolute positional correction runs once/tick** — Baumgarte point-to-point correction runs once/tick (not 64×/tick), eliminating wheel-joint vibration and enabling true rest (stack test, F10 long-run pass).
- **Wheel-lock threshold lowered** — `0.5 → 0.1 rad/s` prevents fighting motor torque at low speeds.
- **Traction clamp aligned** — `roller_friction_coeff` default `1.0 → 0.8` matches `floor_friction_s`; no phantom wheelspin.
- **SDK-standard mecanum roller angles** — FL +45°, FR -45°, BL -45°, BR +45°; +strafe drives +X (robot-left) with no input flip.

---

## 🎨 Rendering System

### Hardware-Instanced Rendering

MFS eliminates per-object draw calls using **GPU instancing**:

- The CPU packs model matrices + colors into contiguous buffers.
- The GPU batches all dynamic bodies into **three instanced draws** (spheres, cubes, cylinders).
- The grid, selection outline, spring-joint overlay, and FTC nose markers share a utility shader with cached uniform locations.

### Shading

- Custom **GLSL Phong** lighting (ambient + diffuse + specular).
- **Equatorial axis rings** painted on every object (red/green/blue) so rotation is visible at a glance.

---

## ⚙️ Physics Engine

### Broadphase — Spatial Hash Grid

Objects are mapped into hashed grid buckets; collision checks are limited to local neighborhoods for **average O(N)** scaling. Cell size adapts to object radii. A sleep system removes inactive bodies from the solver.

### Narrowphase

| Pair | Method |
|---|---|
| Sphere–Sphere | Analytical distance test |
| Sphere–OBB | Closest-point projection |
| OBB–OBB | Separating Axis Theorem (15 axes) + Sutherland–Hodgman face clipping |
| Cylinder–Sphere | Exact solid-cylinder SDF (flat caps, rim, inside classification) |
| Cylinder–Cube | Exact segment-OBB via convex ternary search + face-parallel line support |
| Cylinder–Cylinder | Coaxial face-gap truth + parallel 2-point barrel support |
| Cylinder–Floor | Exact vertical half-extent + 2-point wheel support |

### Solver

- **Impulse-based sequential solver**, 64 iterations by default (configurable 1–128), with **warm starting**.
- Static + kinetic friction, rolling friction, and anisotropic mecanum-roller contact (grip axis + free roller axis, decoupled clamps).
- Positional split-impulse + depenetration passes for pile stability (no velocity-level Baumgarte bias by design).
- **Revolute joint positional correction runs once/tick** — Baumgarte point-to-point correction computed in `pre_step`, applied in velocity solve; eliminates spurious spring energy that caused wheel vibration and prevented rest.

### Integration

- **Semi-implicit (symplectic) Euler** for linear motion.
- **Quaternion-based angular integration** (no gimbal lock).
- **Fixed 60 Hz timestep** with an accumulator and 32-substep cap (spiral-of-death prevention).
- **CCD remainder integration** — swept TOI clamp, post-solve advances only remainder; Verlet-exact free-flight parabola (`-½g·dt²` correction in vacuum).

---

## 🤖 FTC Robotics (MFS Core)

A ~6 kg competition-weight mecanum robot (19.2:1 goBILDA 5203s, 96 mm wheels) simulated with no cheat forces — every newton arrives through documented physics:

- **DC motor model** — stall/free-speed-derived Kt/Ke with gearbox Coulomb drag, 16× electrical subcycling, battery sag with 30 A foldback, regen-aware drain.
- **Anisotropic roller contact** — rubber grip axis at the pair's own Coulomb μ plus a near-free roller axis (bearing drag), decoupled clamps. Mecum sign fix: `cross(axle_proj, floor_normal)` orients the roller frame correctly; forward/strafe/rotate all work through honest wheel torque.
- **Honest odometry** — wheel-encoder translation (slip shows as error) + IMU-model heading.
- **Robot-relative sticks** — right means the robot's right at any heading; an orange nose wireframe marks the +Z face, and the default camera starts behind the robot.
- **No cheat forces** — strafe/rotate authority arrives purely through wheel torques on the anisotropic roller frame; no chassis forces anywhere.
- **Aligned traction clamp** — `roller_friction_coeff = 0.8` matches `floor_friction_s`; drivetrain and solver agree on max traction.
- **SDK-standard roller angles** — FL +45°, FR -45°, BL -45°, BR +45°; +strafe drives +X (robot-left) with no input flip.

---

## 🧮 Mathematics Core

A fully custom, dependency-free math library: 3D vectors, 4×4 matrices, quaternions, and inertia tensors — designed for tightly packed, cache-friendly structs.

**Deterministic transcendentals** (`det_math.h/c`): fixed-coefficient polynomials for `pow`, `sin`, `cos`, `ln`, `exp` — bit-identical on all IEEE-754 targets; fallback counters track libm desync.

---

## 🔍 Terminal Debugger & Output Suite (`mpe-tui`)

A terminal-only companion to the GTK engine — a live inspector and a scriptable state-dump suite in one binary (needs only ncurses):

```bash
cd v15R3/src
make mpe-tui
./mpe-tui                         # live ncurses inspector (needs a TTY)
./mpe-tui --snapshot 600          # one full state dump (pipeable, diffable)
./mpe-tui --stream 600 --every 60 # dumps over time
./mpe-tui --snapshot 10 --scene tower|pendulum|springlab|f10|ftc|demo
```

Live screens: overview table, per-object characteristics + mathematics (quaternion, euler, inertia tensors, momentum, energy), joint/constraint detail with live endpoint geometry, pairwise scene graph, help. Snapshot sections (`[engine]`, `[body i]`, `[springs]`, `[constraints]`, `[pairs]`, `[islands]`, `[stats]`, `[result]`) are fixed-format and deterministic — `make tui-smoke` checks every scene dumps finite state.

---

## 🖥️ Platform & Rendering Stack

| Layer | Technology |
|---|---|
| Windowing / UI | GTK3 |
| Graphics API | OpenGL 3.3 Core (via libepoxy) |
| Lighting | Custom GLSL Phong |
| Debug visualization | Axis rings, wireframe selection, joint lines, overflow counters |

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
| Toggle Game / Debug mode | `0` |

### Spawning

| Action | Input |
|---|---|
| Spawn object | Hold `Enter` |
| Spawner settings | `8` |
| Spawn FTC robot (19.2:1) | `8` then `5` |

### Driving robots (gamepad, robot-relative)

| Action | Input |
|---|---|
| Drive forward / strafe | Left stick |
| Turn | Right stick X |
| Heading marker | Orange nose cube on the robot's front face |

### Selection & Editing

| Action | Input |
|---|---|
| Select object | Right-click (raycast) **or** `R` (Debug) |
| Open object menu | `E` |
| Apply impulse | `F` |
| Delete object | Middle-click |
| World settings | `7` |
| Save / Load scene | `9` |

### Debug Terminal & Validation

| Action | Input |
|---|---|
| Open debug terminal | `1` (Debug) |
| Stability stack test | `F5` |
| Sleep / wake test | `F6` |
| Editor torture test | `F7` |
| Spawn stress test (300 objects) | `F8` |
| Validation report | `F9` |
| Long-run validation (60 s) | `F10` |
| **Config torture test** | **F11** |

---

## 🐚 Debug Terminal

In Debug Mode, press `1` to open a **POSIX-style shell** over the physics world. The simulation is exposed as a virtual filesystem:

| Path | Contents |
|---|---|
| `/obj` | All rigid bodies |
| `/joint` | All spring joints |
| `/world` | World variables (gravity, drag, friction) |
| `/camera` | Camera state |
| `/spawner` | Spawner settings |

A few examples:

```
touch new.sph            # spawn a sphere
ln 1 2                   # spring-join objects 1 and 2
mv 3 /pos/0/10/0         # teleport object 3
chown 5.0 3              # set object 3's mass to 5 kg
chmod static 3           # make it immovable
kill -STOP 3             # put it to sleep
ps aux                   # list every body with state
export GRAVITY=-2.0      # change world gravity
```

Type `help` for the full command list, `man <command>` for usage. `Ctrl+L` clears, `Esc` closes. Mutating commands require Debug Mode; in Game Mode the terminal is read-only.

---

## 🧪 Validation Tests

MFS ships with built-in stability tests:

| Key | Test |
|---|---|
| `F5` | 10-cube stability stack |
| `F6` | Sleeping cube + moving projectile (sleep/wake) |
| `F7` | Editor torture: select, joint, delete, reset |
| `F8` | Spawn stress: up to 300 mixed objects |
| `F9` | Print validation report |
| `F10` | Long-run validation: 3600 ticks (60 s) of idle stability |
| `F11` | Config torture: 80 tunables randomised to extremes, then 3600 ticks |

`F10` monitors for NaN values, fallen objects, and residual motion, printing `PASS`/`FAIL` at the end. `F11` is a robustness verdict — `PASS` means no NaN and nothing fell through the world (speeds reported, never gated; under extremes, perpetual fall/creep can be the true outcome). Each F11 press uses the next printed seed. Torture pins solver resolution (gravity −17…−1, ≥96 iterations — proven envelope for the 10:1 validation column) while material/world extremes stay fully random.

---

## 🛠️ Build Instructions

### Dependencies (Ubuntu / Debian)

```bash
sudo apt update
sudo apt install build-essential pkg-config libgtk-3-dev libepoxy-dev
# Optional, for the mpe-tui terminal debugger:
sudo apt install libncurses-dev
```

For other distributions (Fedora, Arch, SUSE, Alpine, Gentoo, Nix), see [install/linux/linux_install_instructions.md](install/linux/linux_install_instructions.md).

### Build and run

```bash
cd v15R3/src
make clean
make
./engine
```

---

## ⚠️ Known Limitations

- **Wayland:** Mouse locking does not work under native Wayland. Run under X11, or try `GDK_BACKEND=x11 ./engine`.
- **Scene format:** v200 saves bodies (with stable IDs, sleep state, damping) plus spring and revolute joints, with a CRC32 integrity footer. Files ≤v153 still load via the legacy reader.
- **Global state:** All simulation state (bodies, IDs, joints, constraints, caches, solver scratch) is owned by `physics_world`; the file-scope sim globals are retired. App/UI state (camera, input, selection, terminal, diagnostics) remains global by design.
- **FTC odometry:** encoder odometry drift exceeds 20% under wheelspin (test `ftc_odometry` fails); chassis-velocity integration would be more accurate but hides slip.
- **FTC stress:** multi-robot stress test shows chassis lean under sustained lateral load and reduced strafe displacement (test `ftc_stress` fails); suspension model would help.

---

## 📜 Version History

- **v15R3 (MFS release)** — configuration system, physics-truth pass, full constraint framework, TUI debugger + snapshot suite, mecanum sign fix, revolute Baumgarte fix, 36/36 headless green. *(this tree)*
- **v15R2** — config-system hardening + MFS robotics (prior RC, parked in `robotics_backup/`).
- **v1.4 Alpha RC3** — domain-driven restructure, spatial-hash broadphase, physics-world encapsulation.
- **v1.4 Alpha 2** — warm-starting solver, multi-point contact manifolds.
- **v1.4 Alpha RC1** — spring joints, joint renderer, color painting, OBB raycast selection.
- **v1.3** — established instanced rendering and spatial-hash direction.

See `evolution.txt` for the full lineage back to stage 0. Release notes: [`v15R3/release_notes_v15R3.md`](v15R3/release_notes_v15R3.md).

---

### Screenshots

<img width="4424" height="1824" alt="Screenshot from 2026-07-18 17-18-52" src="https://github.com/user-attachments/assets/5d1d044d-3926-469e-ab27-9f3719452324" />
<img width="4558" height="1908" alt="Screenshot from 2026-07-18 17-20-09" src="https://github.com/user-attachments/assets/acebe348-707e-485e-835c-08cd1b1dc0fa" />

---

## 🧪 Headless Test Suite

MFS ships a headless regression suite (no GTK/OpenGL required) — **36/36 green** (29 MPE/physics + `frustum` + 7 FTC robotics):

| Test | Proves |
|------|--------|
| `two_world` | Independent `physics_world` instances |
| `revolute` | Hinge joints hold anchor and allow swing |
| `cylinder_drop` | Cylinder settles on the floor |
| `driven_wheel` | Torque → friction → translation (grounded, coupled) |
| `math3_inverse` | Matrix inverse at small inertia tensors |
| `floor_collision_diag` | Floor contact diagnostics |
| `cylinder_sphere/cube/cylinder` | Cylinder narrowphase pairs |
| `list4_cylinder_floor` | Tipped-cylinder floor regression |
| `scene_roundtrip` | Save/load v200 round-trip (springs + revolutes) |
| `static_hold` | Coulomb stick holds / yields past friction angle |
| `rolling_decay` | Contact-patch rolling resistance decay |
| `ccd_sweep` | Swept TOI: no tunneling at 144 m/s |
| `kinematic` | Velocity-driven platforms carry bodies |
| `determinism` | Twin worlds agree bitwise over 600 ticks |
| `momentum` / `angmom` | Linear / angular momentum conservation |
| `spring` | Hooke period + bounded energy |
| `projectile` | Verlet-exact free-flight parabola |
| `incline_accel` | Slope acceleration matches `g·sinθ` |
| `pendulum` | Revolute pendulum period |
| `bounce_series` | Poisson restitution series |
| `friction_stop` | Coulomb stopping distance `v²/(2μg)` |
| `stack` | 6-cube tower stands (no vibration) |
| `f10_long_run` | F10 settle gates on the validation scene |
| `sleep_contact_wake` | Slow pushers wake sleepers; resting contact doesn't churn |
| `f11_torture` | Deterministic config extremes without corruption |
| `frustum` | Frustum culling math |
| `ftc_teleop` / `ftc_mecanum` / `ftc_tank_turn` | Tank, strafe, and turn drive truth |
| `ftc_odometry` | Encoder odometry tracks ground truth (≤20%) — **known fail: >20% drift** |
| `ftc_stress` | Scripted multi-axis drive: smooth, bounded, settles — **known fail: chassis lean, weak strafe** |
| `ftc_registry` | GUI spawn/drive/count path |
| `ftc_physics_validation` | Motor/contact/idle/strafe validation |

Run with `python3 tools/test_runner.py`.

### Determinism and precision

- Fixed 1/60 s timestep, fixed solver iteration order, exact IEEE `+ - * / sqrt`.
- Per-tick transcendentals (damping retention, rotation rotors) use fixed-coefficient polynomials (`v15R3/src/core/det_math.h`), bit-identical on all IEEE-754 targets; the build disables FP contraction (`-ffp-contract=off`).
- Proven by `determinism`: twin worlds agree bitwise over 600 ticks.
- float32 world: the playable volume is bounded (±250 m), where float resolution (~0.03 mm at the corners) sits 300× below contact slop. No origin rebasing required inside the boundary box.