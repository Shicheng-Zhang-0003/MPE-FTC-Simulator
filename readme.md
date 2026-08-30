# MPE FTC Simulator — Interim README

> Status: transition build
> Original project: Miniature Physics Engine v15R2
> Current direction: FTC robotics simulator with real physics, headless testing, and future FTC-style hardware abstraction.

---

## 1. What This Project Is

This repository is currently transitioning from a general-purpose miniature 3D physics engine into an FTC-oriented robotics simulator.

The long-term goal is to provide an FTC simulation environment comparable in spirit to what WPILib simulation provides for FRC:

- repeatable autonomous testing
- realistic drivetrain behavior
- motors, battery sag, joints, sensors, and mechanisms
- headless tests suitable for CI
- eventually, a user-facing robot-programming API

This is not there yet. The current codebase is an engine in transition.

---

## 2. Current Known-Good Baseline

As of the current interim state:

- The engine compiles cleanly.
- Core physics tests mostly pass.
- Python tooling has replaced bash/sed as the preferred development infrastructure.
- Old bash fix scripts remain in the repository for historical context, but should not be used for new mutation/refactor work.

Current test status:

| Test | Status | Notes |
|---|---:|---|
| `two_world` | PASS | Confirms separate `physics_world` instances work. |
| `revolute` | PASS | Constraint/joint baseline. |
| `teleop_drive` | PASS | Basic FTC-style drivetrain command path. |
| `driven_wheel` | PASS | Torque-to-wheel-to-ground propulsion path. |
| `cylinder_drop` | PASS | Cylinder contact/narrowphase baseline. |
| `math3_inverse` | PASS | Matrix inverse edge-case coverage. |
| `mecanum_drive` | XFAIL | Expected failure until real anisotropic/roller friction exists. |

`mecanum_drive` should not be fixed by reintroducing fake chassis forces. It should pass only when the wheel/contact model can produce physically meaningful lateral traction.

---

## 3. Repository Layout

Important paths:

```text
v15R2/src/
  core/
    physics_world.c/.h      Multi-world physics state
    rigidbody.c/.h          Body data and integration helpers

  physics/
    constraint.c/.h         Constraint/contact solving
    collision_mechanics.c   Collision/contact mechanics
    revolute_joint.c/.h     Revolute joint support
    spring_joint.c/.h       Spring joint support
    broadphase.c/.h         Broadphase collision support

  robotics/
    robot.c/.h              FTC robot aggregate
    drivetrain.c/.h         Drive command logic
    motor.c/.h              Motor model
    motor_presets.c/.h      FTC-ish motor presets
    battery.c/.h            Battery model
    wheel_traction.c/.h     Legacy/transition traction code

  tests/
    *_test.c                Headless physics and robotics tests

  ui_input/
    editor/input/terminal/overlay code

  render/
    Rendering support

  scene/
    Scene save/load support

  config/
    Engine tunables

tools/
  build_check.py            Clean build + compiler diagnostics + struct validation
  test_runner.py            Headless test runner with XFAIL support
  refactor.py               Safe Python refactoring helper
  project_audit.py          Read-only architecture/source audit

fixes/
  Historical bash fix scripts. Do not use for new refactors.
```

---

## 4. Build

From the source directory:

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

Preferred test command:

```bash
python3 tools/test_runner.py
```

List known tests:

```bash
python3 tools/test_runner.py --list
```

Run a filtered test:

```bash
python3 tools/test_runner.py driven_wheel
```

Current expectation:

- all non-mecanum tests should pass
- `mecanum_drive` is expected to fail until anisotropic friction is implemented

---

## 6. Development Rules Going Forward

### Use Python tools, not bash mutation scripts

The project previously used many numbered shell scripts in `fixes/` to mutate C source files using `sed`, `awk`, and inline text insertion. That approach caused structural C corruption and is now considered unsafe for continued development.

Going forward:

- use `tools/refactor.py` for scripted source edits
- use direct C edits for normal feature work
- use `tools/build_check.py` after every structural change
- use `tools/test_runner.py` after every physics/robotics change
- do not run old fix scripts as part of active development

### Do not fake physics to make tests pass

The purpose of the simulator is FTC usefulness, especially for autonomous and drivetrain tuning. A passing test is only valuable if the underlying model is physically meaningful.

Specifically:

- do not reintroduce fake mecanum chassis forces as a permanent solution
- do not lower test thresholds to hide missing physics
- mark future-physics tests as XFAIL until the real model exists

---

## 7. Current Architectural Risks

The main architectural risks are:

### 7.1 `simulation.c` god file

`simulation.c` still contains too many responsibilities:

- physics stepping
- editor behavior
- input/menu dispatch
- validation helpers
- GUI/runtime coupling

This must be split before a clean FTC simulator API can exist.

### 7.2 Global state

The codebase still contains legacy global state such as scene object arrays and selected object/editor state. The newer `physics_world` path is the correct direction, but the GUI/editor path has not fully migrated.

### 7.3 Scene persistence

The scene format needs to reliably preserve robot-relevant state:

- stable body IDs
- joints/constraints
- robot assemblies
- mechanism configuration
- possibly sleep state and material data

### 7.4 Mecanum physics

Mecanum currently lacks real anisotropic/roller contact behavior. The failing mecanum test is intentional and should remain a roadmap gate.

---

## 8. Immediate Roadmap

### Phase A — Stabilize Infrastructure

- Keep Python tooling green.
- Retire old bash fix scripts from active workflows.
- Maintain clear test status with XFAIL for known future physics.

### Phase B — Split `simulation.c`

Recommended extraction order:

1. validation/reporting helpers
2. GUI/editor input dispatch
3. physics tick wrapper
4. render/world synchronization

The goal is to make the physics core usable headlessly without dragging the GTK/editor stack along with it.

### Phase C — Complete Real Drivetrain Physics

- Implement anisotropic friction / mecanum roller behavior.
- Remove or quarantine legacy fake traction paths.
- Turn `mecanum_drive` from XFAIL to PASS only when the physical model is real.

### Phase D — FTC HAL

Only after the world/physics/runtime architecture is clean:

- simulated motors
- simulated servos
- simulated IMU
- simulated encoders
- hardware map
- OpMode-style lifecycle
- robot configuration files

---

## 9. Project Philosophy

This simulator should prioritize truthful physics over convenient demos.

A robot that drives correctly in the simulator should be useful for real FTC reasoning. That means:

- motor commands should become torque
- torque should move wheels/joints
- wheels should interact with the floor through contact/friction
- sensors should read from simulated physical state
- user robot code should not know whether it is running against real or simulated hardware

The current project is not finished, but the foundation is now moving in that direction.
