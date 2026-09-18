> ⚠️ **STALE DOCUMENT** — This audit was taken before the increment split.
> `simulation.c` has since been reduced from 1123 lines to ~130 lines.
> All 16 functions listed here have been extracted into 9 separate modules.
> This document is retained for historical reference only.

---

# simulation.c Responsibility Map (Phase A audit)

- Total lines: **1123**
- Functions detected: **16** (1060 lines inside functions)

| Function | Lines | Start | End | Globals | Risk |
|---|---:|---:|---:|---:|---:|
| `physics_halt_set` | 6 | 18 | 23 | 0 | LOW |
| `physics_halt_for_ticks` | 7 | 25 | 31 | 0 | LOW |
| `physics_is_halted` | 3 | 33 | 35 | 0 | LOW |
| `on_entry_insert_text` | 13 | 48 | 60 | 0 | LOW |
| `open_numerical_input_dialog` | 46 | 61 | 106 | 2 | MED |
| `editor_reset` | 21 | 108 | 128 | 1 | MED |
| `validation_report_print` | 42 | 130 | 171 | 9 | HIGH |
| `a3_task13_body_is_invalid` | 23 | 191 | 213 | 0 | LOW |
| `long_run_validation_report` | 39 | 215 | 253 | 5 | HIGH |
| `long_run_validation_evaluate` | 59 | 255 | 313 | 4 | HIGH |
| `long_run_validation_tick_update` | 16 | 315 | 330 | 1 | MED |
| `long_run_validation_start` | 39 | 332 | 370 | 3 | MED |
| `a3_depenetration_dispatch` | 22 | 374 | 395 | 0 | LOW |
| `a3_positional_depenetrate_manifold` | 99 | 397 | 495 | 1 | MED |
| `a3_positional_depenetration_pass` | 51 | 497 | 547 | 3 | MED |
| `physics_step_increment` | 574 | 550 | 1123 | 12 | HIGH |

## Lowest-risk extraction candidates (0-2 globals, <200 lines)
- `physics_is_halted` — 3 lines, globals: —
- `physics_halt_set` — 6 lines, globals: —
- `physics_halt_for_ticks` — 7 lines, globals: —
- `on_entry_insert_text` — 13 lines, globals: —
- `a3_depenetration_dispatch` — 22 lines, globals: —
- `a3_task13_body_is_invalid` — 23 lines, globals: —
- `long_run_validation_tick_update` — 16 lines, globals: long_run_validation
- `editor_reset` — 21 lines, globals: main_inputs
- `a3_positional_depenetrate_manifold` — 99 lines, globals: g_cfg
- `open_numerical_input_dialog` — 46 lines, globals: editor_dialog_active, main_inputs

## Largest functions (god-file core)
- `physics_step_increment` — **574 lines**, 12 globals: a3_previous_debug_mode_state, debug_last_, editor_dialog_active, frame_timer, g_cfg, long_run_validation, main_camera_fov, main_inputs …
- `a3_positional_depenetrate_manifold` — **99 lines**, 1 globals: g_cfg
- `long_run_validation_evaluate` — **59 lines**, 4 globals: debug_last_, long_run_validation, obj_per_scene, object_count
- `a3_positional_depenetration_pass` — **51 lines**, 3 globals: g_cfg, obj_per_scene, object_count
- `open_numerical_input_dialog` — **46 lines**, 2 globals: editor_dialog_active, main_inputs
- `validation_report_print` — **42 lines**, 9 globals: broadphase_get, current_joint_count, debug_last_, g_registry, g_registry_count, main_inputs, object_capacity, object_count …
- `long_run_validation_report` — **39 lines**, 5 globals: broadphase_get, g_registry, g_registry_count, long_run_validation, object_count
- `long_run_validation_start` — **39 lines**, 3 globals: g_registry, g_registry_count, long_run_validation

## Global-state inventory (file-wide reference lines)
- `obj_per_scene`: 17 lines
- `object_count`: 28 lines
- `object_capacity`: 2 lines
- `selected_object`: 7 lines
- `current_joint_count`: 1 lines
- `main_inputs`: 124 lines
- `main_camera_fov`: 50 lines
- `editor_dialog_active`: 5 lines
- `g_cfg`: 31 lines
- `g_registry`: 21 lines
- `g_registry_count`: 6 lines
- `long_run_validation`: 69 lines
- `debug_last_`: 20 lines
- `a3_previous_debug_mode_state`: 4 lines
- `frame_timer`: 2 lines
- `main_timer`: 3 lines
- `broadphase_get`: 7 lines

---

# FTC Drivetrain Physics Audit

## Overview

The FTC portion of the codebase simulates mecanum-wheeled drivetrains for FTC (FIRST Tech Challenge) robotics. The physics pipeline is:

1. **Motor dynamics** (`robotics/robot.c:ftc_robot_update`) — subcycled motor torque on wheel rigidbodies
2. **Traction** (`robotics/drivetrain.c:MPE_DRIVETRAIN_REAL`) — torque-to-force at contact, clamped by friction
3. **Contact generation** (`physics/collision_cylinder.c:collision_static_plane_cylinder`) — floor-wheel contacts
4. **Contact solver** (`physics/collision_mechanics.c`) — sequential-impulse friction with anisotropic frame
5. **Revolute constraint** (`physics/constraint.c:constraint_add_revolute`) — hinge joint constraining wheel rotation
6. **Wheel-lock** (`core/physics_world.c:wheel-lock loop`) — locks undriven wheels to axle axis
7. **Chassis damping** (`robotics/drivetrain.c`) — horizontal drag, yaw damping, idle hold

## Physics Pipeline (per tick)

```
ftc_robot_reset_drive_tracking()
    └─ driven_this_tick = false for all wheels

ftc_robot_update()
    └─ Motor subcycling (16x per tick)
        ├─ motor_update() — torque from back-EMF model
        ├─ torque_avg applied to wheel + chassis (Newton 3rd)
        └─ driven_this_tick = true for wheels with |command| > 0.05

physics_world_step()
    ├─ Broadphase + narrowphase (collision_cylinder.c)
    ├─ contact_prepare_solver() — Coulomb frame, anisotropic overlay
    ├─ constraint_solve_all() — revolute joints + hinge constraints
    ├─ constraint_correct_axis_drift_all() — Baumgarte axis stabilization
    ├─ wheel-lock loop — locks mecanum wheels to axle (|omega| < thresh)
    ├─ collision_apply_poisson_restitution()
    ├─ constraint_solve_all() — post-restitution relaxation
    └─ collision_apply_split_impulse() — positional depenetration
```

## Key Physics Parameters

| Parameter | Default | Purpose | Risk |
|---|---:|---|---|
| `world.floor_friction_s` | 0.8 | Static friction coefficient for floor contacts | HIGH |
| `world.floor_friction_k` | 0.6 | Kinetic friction coefficient for floor contacts | HIGH |
| `solver.roller_friction_coeff` | 0.8 | Mecanum roller free-axis mu (set = floor_friction_s for traction) | MED |
| `solver.wheel_lock_omega_thresh` | 0.5 | Angular velocity below which wheels are locked | MED |
| `world.rolling_resistance_coeff` | 0.03 | Coulomb rolling resistance | LOW |

## Critical Physics Findings

### 1. Anisotropic Friction Model (collision_mechanics.c:1217-1297)

**Setup**: For mecanum floor contacts, the contact solver uses an anisotropic frame:
- `tangent_vector = grip` (perpendicular to roller, in contact plane)
- `tangent2 = roller` (along wheel's roller axis)
- `mu_grip = fminf(wheel.friction_static, floor.friction_static)` — high friction
- `mu_free = fminf(wheel.friction_kinetic, floor.friction_s)` — must match grip for traction

**Critical fix applied**: `mu_free` was previously `fminf(kg, g_cfg.solver.roller_friction_coeff)` with `roller_friction_coeff = 0.03`. This caused the traction force (applied along roller direction) to exceed the friction limit by 6x, producing "slippery" behavior. Fixed to `mu_free = fminf(kg, g_cfg.world.floor_friction_s)`.

**Why this matters**: The drivetrain applies force along `rolling_dir = axle × up`, which IS the roller direction (tangent2). If `mu_free = 0.03` but traction is clamped by `floor_friction_s = 0.8`, the contact solver allows slip at 0.03×normal while drivetrain applies 0.8×normal force. The wheel slides instead of rolling.

### 2. Floor Friction Too Low (mpe_config_schema.c)

**Issue**: Default `floor_friction_s = 0.2`, `floor_friction_k = 0.1` — too low for rubber-on-foam tiles (real FTC: 0.6-1.0). The wheel's own friction (1.0/0.8) is clamped by floor friction: `mu_grip = min(1.0, 0.2) = 0.2`.

**Fix**: `floor_friction_s = 0.8`, `floor_friction_k = 0.6` — matches real FTC robot-on-foam coefficients.

### 3. Wheel-Lock Loop (physics_world.c:635-654)

**Issue**: The wheel-lock loop skipped driven wheels (`rb->driven_this_tick` check), allowing driven wheels to pitch freely when motor command was small but non-zero. Low-frequency pitching was not damped.

**Fix**: Removed `rb->driven_this_tick` check — wheel-lock now applies to ALL mecanum wheels. The lock only activates when `|omega_axle| < thresh`, so actively spinning wheels are unaffected.

### 4. Traction Clamping (drivetrain.c:100-133)

**Design**: Traction force is clamped by `max_grip = floor_friction_s × normal_per_wheel`. This is correct — rolling grip is static friction. The contact solver handles sliding via kinetic friction.

**Note**: The `drivetrain_update` function is NOT called from `ftc_robot_update` — it's only called in test files. The `ftc_robot_update` function handles motor dynamics directly (subcycled motor torque on wheel bodies). The `drivetrain_update` applies additional traction forces (chassis damping, rolling resistance). This design means tests using `drivetrain_update` get additional physics that production code does not.

### 5. Mecanum Roller Angles (robot.c:117-121)

**Configuration**: Standard FTC mecanum (axle along X, robot forward = +Z):
- FL (+X,+Z): roller at +45° from forward = +45° from axle
- FR (-X,+Z): roller at -45° from forward = -45° from axle
- BL (-X,-Z): roller at -45° from forward = -45° from axle
- BR (+X,-Z): roller at +45° from forward = +45° from axle

Matches `drivetrain_mecanum` IK: positive strafe = +X world.

### 6. Inertia Tensor (rigidbody.c:938-960)

**Cylinder inertia**: `Ixx = 0.5 × m × r²` (axle), `Iyy = Izz = (m/12)(3r² + l²)` — correct for solid cylinder. Wheel inertia is geometric only; chassis load arrives via contact/constraint impulses.

## Risk Assessment

| Component | Lines | Globals | Risk | Notes |
|---|---:|---:|---|---|
| `ftc_robot_update` | 120 | 5 | HIGH | Motor subcycling, current limiting |
| `drivetrain_update` | 250 | 8 | HIGH | Traction, damping, rolling resistance, encoder odom |
| `collision_static_plane_cylinder` | 170 | 1 | MED | Contact generation for floor-wheel |
| `collision_prepare_solver` | 370 | 4 | HIGH | Anisotropic frame, warm start, cache |
| `revolute_correct_axis_drift` | 35 | 1 | MED | Baumgarte axis stabilization |
| `wheel-lock loop` | 20 | 2 | LOW | Angular velocity lock for undriven wheels |

## Validation Status

- **475 native build**: Compiles cleanly (no errors)
- **MSYS2 cross-compilation**: Compiles cleanly (pre-existing linker error unrelated)
- **Default friction coefficients**: floor_friction_s=0.8, floor_friction_k=0.6, roller_friction_coeff=0.8
- **Wheel-lock**: Applies to all mecanum wheels (driven_this_tick removed from skip condition)
- **Anisotropic friction**: mu_free now equals mu_grip (full friction in roller direction)
