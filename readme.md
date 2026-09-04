# MPE FTC Simulator — README
> Status: **Milestone 2 (Physics Keystone) complete. Milestone 3 in progress.**
> Original project: Miniature Physics Engine v15R2
> Current direction: FTC robotics simulator with real physics, headless testing,
> and future FTC-style hardware abstraction.

# Note. This codebase has become MPE v15R2 and is progressing as MPE in the v15R3 development slot. MFS will in the future become a kernel plugin ecosystem specifically for FTC, the FTC components CAD editor planned, as well as the Java to C cross-compiler for executing FTC native programming within the MPE runtime itself. This split will begin in v16R1.

---

## 1. What This Project Is

This repository is an FTC-oriented robotics simulator built on top of the
Miniature Physics Engine (v15R2). The long-term goal is to provide an FTC
simulation environment comparable in spirit to what WPILib simulation provides
for FRC:

- repeatable autonomous testing
- realistic drivetrain behaviour (real cylinder wheels, anisotropic roller friction)
- motors with BackEMF, gear ratios, battery voltage sag
- headless tests suitable for CI
- eventually, a user-facing robot-programming API (HardwareMap / OpMode)

---

## 2. Current Test Status

All 8 headless tests **PASS**:

| Test | Status | What it proves |
|---|---|---|
| `two_world` | PASS | Separate `physics_world` instances are independent |
| `revolute` | PASS | Revolute joint holds anchor, allows swing |
| `teleop_drive` | PASS | Robot drives forward under motor power |
| `mecanum_drive` | PASS | Robot strafes via **real anisotropic roller friction** |
| `cylinder_drop` | PASS | Cylinder rests on floor (narrowphase works) |
| `driven_wheel` | PASS | Torque → friction → translation (grounded wheel rolls) |
| `math3_inverse` | PASS | Matrix inverse handles small inertia tensors |
| `ftc_integration` | PASS | Full drive/turn/strafe sequence |

---

## 3. Repository Layout

```text
v15R2/src/
  core/
    physics_world.c/.h          Multi-world physics state + full pipeline
    rigidbody.c/.h              Body data, integration, cylinder support
    simulation_camera.c/.h      Camera tick (extracted from simulation.c)
    simulation_physics_loop.c/.h Fixed-timestep physics loop
    validation_report.c/.h      F9 status report
    long_run_validation.c/.h    F10 validation
    physics_halt.c              Halt state
    math3D.h / math4_special.h  Linear algebra
    event_log.c/.h              Engine event ring buffer

  physics/
    collision_mechanics.c/.h    Narrowphase + solver + mecanum friction
    broadphase.c/.h             Spatial hash grid
    constraint.c/.h             Constraint pool + dispatch
    revolute_joint.c/.h         Revolute solver + Baumgarte axis drift fix
    spring_joint.c/.h           Legacy spring joints
    depenetration.c/.h          Positional depenetration

  robotics/
    robot.c/.h                  FTC robot aggregate (chassis + 4 cylinder wheels)
    drivetrain.c/.h             Tank + Mecanum IK, real traction
    motor.c/.h                  DC motor model (BackEMF, Kt, Kv, gearing)
    motor_presets.c/.h          5 goBILDA/REV presets
    battery.c/.h                Voltage sag model (12.8V, 0.015Ω, 30Ah)
    gui_robot_registry.c/.h     GUI proxy sync + fixed-timestep accumulator

  ui_input/
    simulation_input_dispatch.c  Keyboard bindings (G/V/B/N/C/H robot drive)
    simulation_menu_dispatch.c   Scene menu handling
    editor_dialog.c              Numerical input dialog
    debug_terminal.c             POSIX-style terminal (146 KB)
    microvim.c/.h                Modal text editor
    overlay.c                    HUD overlay
    input_control.c/.h           GTK key/mouse wiring
    camera.c/.h                  Camera math
    object_spawner.c/.h          Spawn logic
    object_selector.c/.h         Raycast selection
    mouse_lock.c/.h              X11 pointer grab
    config_menu.c/.h             Config menu (key 6)

  render/                       OpenGL instanced rendering + shaders
  scene/                        Scene save/load, boundary, ID remap
  config/                       69-parameter config registry

  tests/
    two_world_test.c
    revolute_test.c
    teleop_drive_test.c
    mecanum_drive_test.c
    cylinder_drop_test.c
    driven_wheel_test.c
    math3_inverse_test.c
    ftc_integration_test.c
    ftc_debug_test.c            (diagnostic, not in CI runner)

tools/
  build_check.py                Clean build + compiler diagnostics
  test_runner.py                Headless test runner (all 8 tests)
  refactor.py                   Safe Python refactoring helper
  project_audit.py              Read-only architecture audit

fixes/
  Historical fix scripts (bash + python). Do not run against active code.
```

---

## 4. Build

```bash
cd v15R2/src
make clean && make
```

Preferred project-level build check:

```bash
python3 tools/build_check.py
```

---

## 5. Test

```bash
python3 tools/test_runner.py            # all 8 tests
python3 tools/test_runner.py --list     # show available tests
python3 tools/test_runner.py mecanum    # filtered
```

All 8 tests pass. No expected failures.

---

## 6. Robot Controls (GUI)

| Key | Action |
|---|---|
| `T` (terminal) → `touch robot` | Spawn FTC robot at (5, rest_height, 5) |
| `G` | Drive forward |
| `B` | Drive backward |
| `V` | Strafe right |
| `N` | Strafe left |
| `C` | Rotate left (CCW) |
| `H` | Rotate right (CW) |

The robot uses goBILDA 5203 30:1 motors, mecanum drivetrain, cylinder wheels
with real anisotropic roller friction. An orange nose sphere indicates heading.

---

## 7. Development Rules

- Use `tools/refactor.py` for scripted source edits
- Use `tools/build_check.py` after every structural change
- Use `tools/test_runner.py` after every physics/robotics change
- Do not fake physics to make tests pass
- Do not reintroduce chassis-force cheats for mecanum

---

## 8. Current Architectural State

### Resolved (since original audit)
- `simulation.c` split from 1123 → ~130 lines (9 modules extracted)
- Cylinder narrowphase + inertia implemented
- Mecanum strafe via real anisotropic roller friction (no chassis cheat)
- Motor gear ratio pipeline fixed (no double-application)
- Revolute axis drift corrected (Baumgarte β=0.1)
- Fixed-timestep accumulator for robot physics (60 Hz)
- GUI robot proxy sync (physics_world → obj_per_scene)
- All dead code removed (rb_integrate, define_forces, frame_timer.c, etc.)

### Remaining debt
- `debug_terminal.c` is 146 KB (single translation unit)
- Global `obj_per_scene` / `object_count` still used by GUI path
- Scene save/load doesn't persist joints, robots, or cylinders
- No solver islanding (PHYS-001)
- No CCD (PHYS-002)
- No rolling resistance (PHYS-004)
- No SIMD (PERF-001)
- No frustum culling (PERF-003)

---

## 9. Roadmap

### Phase A — Stabilize ✅
Python tooling green. Bash scripts retired. Test status clear.

### Phase B — Split simulation.c ✅
9 modules extracted. Physics usable headlessly.

### Phase C — Real Drivetrain Physics ✅
Cylinder wheels. Anisotropic friction. Mecanum via roller contact.
Motor model correct. Battery sag. Revolute joints with axis correction.

### Phase D — Solver Hardening (in progress)
- [ ] Per-world contact cache
- [ ] Solver islanding
- [ ] Rolling resistance
- [ ] Warm-starting for constraints

### Phase E — Sensors & Closed-Loop Control
- [ ] Encoders, IMU, distance sensors
- [ ] PID controller
- [ ] Motor velocity/position modes

### Phase F — Mechanisms
- [ ] Prismatic joints
- [ ] Arms, slides, servos, intakes

### Phase G — FTC Platform & API
- [ ] Robot JSON loader
- [ ] HardwareMap abstraction
- [ ] OpMode lifecycle
- [ ] Gamepad model, telemetry

---

## 10. Project Philosophy

This simulator prioritises truthful physics over convenient demos.
A robot that drives correctly here should behave the same way on a real
FTC field. Motor commands become torque. Torque moves wheels. Wheels grip
the floor through contact friction. Sensors read from simulated state.
User robot code should not know whether it is running against real or
simulated hardware.
