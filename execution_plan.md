# MFS-W Execution Plan: v10R3I → FTC-Team-Ready
**Generated:** 2026-09-07  
**Objective:** Transform v10R3I (Windows port of v15R3) into a stable, reliable simulator that grade 9 FTC students can use to train driving skills before the physical robot is available.

**Constraint:** The physics must be *honest* — students must learn correct driving habits, not compensate for simulation artifacts.

---

## Table of Contents
1. [Current State Assessment](#1-current-state-assessment)
2. [Execution Phases Overview](#2-execution-phases-overview)
3. [Phase 0: Emergency Stabilization (Gate 15e)](#3-phase-0-emergency-stabilization-gate-15e)
4. [Phase 1: Physics Truth (Training Quality)](#4-phase-1-physics-truth-training-quality)
5. [Phase 2: Data Safety & Robustness](#5-phase-2-data-safety--robustness)
6. [Phase 3: Architecture Debt (v15S Preparation)](#6-phase-3-architecture-debt-v15s-preparation)
7. [Phase 4: Advanced Physics (v16 Prerequisites)](#7-phase-4-advanced-physics-v16-prerequisites)
8. [Testing & Validation Strategy](#8-testing--validation-strategy)
9. [Rollout & Distribution Plan](#9-rollout--distribution-plan)
10. [Timeline & Milestones](#10-timeline--milestones)
11. [Risk Register](#11-risk-register)
12. [Decision Gates](#12-decision-gates)

---

## 1. Current State Assessment

### What Works ✅
- Windows native build via MSYS2/MinGW-w64
- GTK3 windowing + OpenGL 3.3 Core rendering
- Grid, HUD overlay, robot visual proxies
- XInput gamepad detection (F310 in X mode)
- Robot spawning via `touch robot` terminal command
- Basic mecanum drive (forward/strafe/rotate)
- Motor model (BackEMF, Kt/Kv, gear ratio)
- Battery voltage sag simulation
- Revolute wheel joints
- Config system (69 tunables, persistent to `status/engine.cfg`)
- Debug terminal (70+ commands, MicroVim editor)
- 14 headless tests (Linux-verified)

### What's Broken or Unverified 🔴
| Category | Count | Severity |
|----------|-------|----------|
| Gate 15e verification debt | 7 items | Blocking |
| Physics correctness (visible to students) | 12 items | High |
| Data safety / brick-robustness | 15 items | High |
| Solver / integration correctness | 16 items | Medium |
| Scene / persistence | 6 items | Medium |
| Architecture debt | 8 items | Low (v16) |
| **Total open** | **64** | — |

### Critical Blockers for FTC Teams
1. **Cylinder floor collision incomplete** — tipped wheels fall through
2. **Artificial lateral damping** — masks missing solver friction, teaches wrong habits
3. **Odometry ignores heading** — students learn wrong coordinate transforms
4. **Revolute axis drift on chassis** — robot drifts in circles
5. **No containment walls** — robot drives off into infinity
6. **Sleeping bodies can't wake** — pile instability
7. **Scene load destroys scene on corrupt file** — data loss

---

## 2. Execution Phases Overview

```
Phase 0: Emergency Stabilization (Gate 15e)     [1-2 days]
    ↓
Phase 1: Physics Truth (Training Quality)       [3-5 days]
    ↓
Phase 2: Data Safety & Robustness               [2-3 days]
    ↓
Phase 3: Architecture Debt (v15S Prep)          [1-2 weeks]
    ↓
Phase 4: Advanced Physics (v16 Prerequisites)   [2-4 weeks]
    ↓
v16 Release
```

**Total estimated timeline:** 4-6 weeks to v16-ready, but **Phase 0+1+2 can be completed in 1-2 weeks** for initial classroom use.

---

## 3. Phase 0: Emergency Stabilization (Gate 15e)

**Goal:** Clear all 7 Gate 15e blockers so the Windows build can be distributed to the classroom.

**Timeline:** 1-2 days

**Dependencies:** None — this is the first phase.

---

### 0.1 — Run F5–F11 on Windows

**Why:** The in-engine validation suite has never been run on the Windows build. We don't know if the physics loop, sleep system, or stress tests behave correctly on Win32.

**Steps:**
1. Launch `engine.exe` from File Explorer (not MSYS2 terminal — test the distribution build)
2. Press `0` to enter Debug Mode
3. Press `T` to open terminal
4. Type `touch robot` to spawn the robot
5. Press `F5` — observe 10-cube stability stack
   - **Pass criterion:** All 10 cubes settle within 5 seconds, no jitter, no explosion
   - **Record:** Screenshot + console output
6. Press `F6` — observe sleep/wake test
   - **Pass criterion:** Sleeping cube wakes when hit by projectile, stays asleep otherwise
   - **Record:** Screenshot + console output
7. Press `F7` — observe editor torture test
   - **Pass criterion:** No crash, survivor object remains at expected position
   - **Record:** Screenshot + console output
8. Press `F8` — observe spawn stress (300 objects)
   - **Pass criterion:** Framerate > 30 FPS, no NaN in console, no explosion
   - **Record:** Screenshot + console output + FPS counter
9. Press `F9` — observe validation report
   - **Pass criterion:** Full config dump prints, no errors
   - **Record:** Console output
10. Press `F10` — observe long-run validation (60 seconds)
    - **Pass criterion:** Console prints `result: PASS`, no NaN, no fallen objects
    - **Record:** Console output (full 60-second log)
11. Press `F11` — observe config torture test
    - **Pass criterion:** Survives 60 seconds with randomized config, no crash
    - **Record:** Console output
    - **After test:** Type `config reset` in terminal to restore defaults

**Verification:**
- All 7 tests pass
- No crashes, no NaN, no explosions
- Framerate remains > 30 FPS during F8 stress test

**Files touched:** None (observation only)

**Estimated time:** 30 minutes

---

### 0.2 — Run Headless Test Suite on Windows

**Why:** The 14 headless tests have only been verified on Linux. We need to confirm they pass on Windows/MinGW.

**Steps:**
1. Open MINGW64 terminal
2. Navigate to source: `cd ~/mpe/v15R3/src`
3. Run: `python3 ../tools/test_runner.py`
4. Observe output — all 14 tests must print `PASS`

**Expected tests:**
```
two_world           — Independent physics_world instances
revolute            — Hinge joints hold anchor and allow swing
teleop_drive        — Tank drive moves the robot
mecanum_drive       — Strafe via real roller friction
cylinder_drop       — Cylinder settles on the floor
driven_wheel        — Torque → friction → translation
math3_inverse       — Matrix inverse at small inertia tensors
ftc_integration     — Drive / turn / strafe sequence
physics_truth       — Physical-law assertions (24 assertions)
tank_turn           — Differential turning in place
odometry_accuracy   — Odometry tracks physics
cylinder_sphere     — Cylinder vs sphere collision
cylinder_cube       — Cylinder vs cube collision
cylinder_cylinder   — Cylinder vs cylinder collision
```

**Verification:**
- All 14 tests print `PASS`
- No build errors during test compilation
- `physics_truth` assertions match Linux results (compare logs)

**Files touched:** None (observation only)

**Estimated time:** 15 minutes

**If tests fail:**
- Record exact error message
- Check if it's a MinGW-specific issue (e.g., missing `libm` function, different float behavior)
- File as new issue in scope.md with `WIN-` prefix

---

### 0.3 — Scene Save/Load Round-Trip

**Why:** Students will save and load scenes. We need to verify this works on Windows paths.

**Steps:**
1. Launch `engine.exe`
2. Spawn 5 objects: `touch new.sph`, `touch new.cube`, `touch new.sph`, `touch new.cube`, `touch new.sph`
3. Move one object to a known position: `mv 3 /pos/10/5/10`
4. Press `9` → `1` (Save Scene)
5. Observe console: should print `Scene saved to status/scene.dat`
6. Press `9` → `3` (Clear Scene)
7. Observe: all objects disappear
8. Press `9` → `2` (Load Scene)
9. Observe: all 5 objects reappear at correct positions
10. Verify object 3 is at position (10, 5, 10)

**Verification:**
- Scene saves successfully
- Scene loads successfully
- Object positions match pre-save positions
- No crash, no NaN

**Files touched:** None (observation only)

**Estimated time:** 10 minutes

**Known issue:** Cylinders load back as spheres (R3-04). This is expected and tracked. Record as known limitation.

---

### 0.4 — Config Persistence

**Why:** Students will change config settings. We need to verify they persist across restarts.

**Steps:**
1. Launch `engine.exe`
2. Press `6` to open config menu
3. Navigate to World category → Gravity
4. Change gravity from `-9.81` to `-5.0`
5. Exit engine (close window)
6. Relaunch `engine.exe`
7. Press `6` → World → Gravity
8. Observe: gravity should be `-5.0`, not `-9.81`
9. Observe console: should print `[config] loaded status/engine.cfg`

**Verification:**
- Config saves on exit
- Config loads on startup
- Changed value persists

**Files touched:** None (observation only)

**Estimated time:** 10 minutes

---

### 0.5 — 10-Minute Idle Soak

**Why:** We need to verify the engine doesn't crash or produce NaN during extended idle.

**Steps:**
1. Launch `engine.exe`
2. Spawn robot: `touch robot`
3. Leave engine running for 10 minutes
4. Observe: no crash, no NaN in console, robot stays at spawn position

**Verification:**
- No crash after 10 minutes
- No NaN in console
- Robot position unchanged (within 0.01 m)

**Files touched:** None (observation only)

**Estimated time:** 10 minutes (idle)

---

### 0.6 — Clean-Machine Test

**Why:** We need to verify the distribution zip works on a machine without MSYS2 installed.

**Steps:**
1. Copy `windows_release.zip` to a USB drive
2. Copy to a different Windows PC (no MSYS2 installed)
3. Extract zip
4. Double-click `engine.exe`
5. Observe: window opens, grid renders, HUD visible
6. Press `0` → `T` → `touch robot`
7. Observe: robot spawns
8. Press `F5`, `F10` (subset of validation suite)
9. Observe: tests pass

**Verification:**
- No "DLL not found" errors
- Engine launches successfully
- Robot spawns and drives

**Files touched:** None (observation only)

**Estimated time:** 15 minutes

**If DLL errors occur:**
- Run `ldd engine.exe` in MSYS2 terminal
- Identify missing DLL
- Add to `windows_release/` folder
- Re-zip and re-test

---

### 0.7 — Folder Layout Enforcement

**Why:** The engine requires `render/` and `status/` folders beside `engine.exe`. We need to enforce this in the build process.

**Steps:**
1. Edit `makefile`
2. Add post-build step:
```makefile
# WIN_PORT_LAYOUT: Copy render/ and create status/ in release folder
release: engine
	mkdir -p ../windows_release/render
	mkdir -p ../windows_release/status
	cp -r render/* ../windows_release/render/
	cp engine.exe ../windows_release/
	@echo "Release folder ready: ../windows_release/"
```
3. Run `make release`
4. Verify `windows_release/` contains:
   - `engine.exe`
   - `render/` (all shader files)
   - `status/` (empty folder)

**Verification:**
- `make release` succeeds
- `windows_release/` contains all required files
- `engine.exe` runs from `windows_release/` folder

**Files touched:** `makefile`

**Estimated time:** 15 minutes

---

## 4. Phase 1: Physics Truth (Training Quality)

**Goal:** Fix the 12 physics defects that students will see and learn from. These produce visibly wrong behavior that students will internalize as "normal."

**Timeline:** 3-5 days

**Dependencies:** Phase 0 complete (Gate 15e cleared)

**Priority order:** Fix in order of visibility to students.

---

### 1.1 — NEW-01: Cylinder Floor Collision (Barrel Surface)

**Why:** Tipped wheels fall through the floor. This is the most visible physics bug — students will see wheels disappear.

**Root cause:** `collision_static_plane_cylinder()` only tests the two axle endpoints. When a cylinder tips over (axle horizontal), neither endpoint is below the floor plane, so no contact is detected.

**Fix:**
Add barrel-surface contact test to `collision_static_plane_cylinder()`:

```c
/* MFS_177_CYLINDER_BARREL: Test barrel surface contact.
* When cylinder tips over, the barrel (not axle endpoints) touches floor. */
bool collision_static_plane_cylinder_barrel(rigidbody *cyl, float plane_y, collision_data *out) {
    if (cyl->type != object_cylinder) return false;
    
    vector3 axis = cyl->cached_axes[0];
    float r = cyl->radius;
    float h = cyl->cylinder_half_length;
    
    /* Project cylinder center onto floor plane */
    float center_y = cyl->position.y;
    float penetration = plane_y - (center_y - r);
    
    if (penetration <= 0.0f) return false;
    
    /* Contact at projected center */
    out->object_a = cyl;
    out->object_b = collision_static_plane_body_proxy(plane_y);
    out->normal_vector = (vector3){0.0f, -1.0f, 0.0f};
    out->contact_count = 1;
    
    contact_point_data *cp = &out->contacts[0];
    cp->position = (vector3){cyl->position.x, center_y - r, cyl->position.z};
    cp->penetration = penetration;
    
    return true;
}
```

Then integrate into `collision_static_plane_body()`:
```c
bool collision_static_plane_body(rigidbody *body, float plane_y, collision_data *out) {
    if (body->type == object_cylinder) {
        /* Test axle endpoints first */
        if (collision_static_plane_cylinder(body, plane_y, out)) return true;
        /* Test barrel surface (tipped cylinder) */
        if (collision_static_plane_cylinder_barrel(body, plane_y, out)) return true;
        return false;
    }
    /* ... existing sphere/cube code ... */
}
```

**Verification:**
- Spawn cylinder, tip it over (apply torque), observe it rests on barrel
- No fall-through
- Add headless test: `cylinder_tipped_rest`

**Files touched:** `physics/collision_mechanics.c`

**Estimated time:** 2 hours

---

### 1.2 — NEW-05: Remove Artificial Lateral Damping

**Why:** The `PHYSICS LIE` comment in `drivetrain.c` admits this is fake. It masks missing solver friction and teaches students wrong driving habits (robot slides when it shouldn't).

**Root cause:** The contact solver doesn't produce enough lateral friction, so artificial damping is applied to the chassis to prevent sliding.

**Fix:**
1. **Short-term:** Increase floor friction coefficient from `0.2` to `1.2` in `mpe_config_schema.c`:
```c
{"world.floor_friction_s", "Floor Friction (Static)", "Static friction coefficient for floor contacts", p_float,
 cat_world, &g_cfg.world.floor_friction_s, 1.2, 0.0, 5.0, false},
```

2. **Medium-term:** Implement 2-tangent friction cone in solver (see Phase 4, item 2.1)

3. **Remove the lie:**
```c
/* REMOVE THIS BLOCK from drivetrain.c: */
/* MFS_132_DAMPING_TRUTH: PHYSICS LIE — artificial lateral damping */
/* chassis->force_accumulator = vector3_subtraction(
    chassis->force_accumulator,
    vector3_scaling(lat, m * 1.0f)); */
```

**Verification:**
- Robot no longer slides when stationary
- Strafe works correctly
- No artificial damping visible

**Files touched:** `config/mpe_config_schema.c`, `robotics/drivetrain.c`

**Estimated time:** 1 hour (short-term), 1 day (medium-term)

---

### 1.3 — NEW-04: Odometry Heading Rotation

**Why:** Odometry integrates world-space velocity without rotating by heading. Students will learn wrong coordinate transforms.

**Root cause:** `drivetrain_update()` integrates `chassis_vel.x` and `chassis_vel.z` directly without rotating by `odom_theta`.

**Fix:**
```c
/* MFS_178_ODOMETRY_HEADING: Rotate chassis velocity by heading before integrating */
vector3 chassis_vel = world->bodies[robot->chassis_body].velocity;
float yaw_rate = world->bodies[robot->chassis_body].angular_velocity.y;

robot->odom_theta += yaw_rate * dt;

/* Rotate velocity into robot frame */
float cos_t = cosf(robot->odom_theta);
float sin_t = sinf(robot->odom_theta);
float v_fwd = chassis_vel.x * cos_t + chassis_vel.z * sin_t;
float v_str = -chassis_vel.x * sin_t + chassis_vel.z * cos_t;

robot->odom_x += v_fwd * dt;
robot->odom_z += v_str * dt;
```

**Verification:**
- Drive forward, turn 90°, drive forward again
- Odometry should show correct path (not drift)
- Add headless test: `odometry_heading_rotation`

**Files touched:** `robotics/drivetrain.c`

**Estimated time:** 1 hour

---

### 1.4 — NEW-07: Revolute Axis Drift on Chassis

**Why:** The axis-drift correction is applied to both bodies (chassis and wheel). This causes the chassis to rotate when wheels tilt, making the robot drift in circles.

**Root cause:** In `revolute_solve()`, the axis correction is applied to both `body_a` (chassis) and `body_b` (wheel). The chassis should be the reference body and not rotate.

**Fix:**
```c
/* MFS_179_REVOLUTE_AXIS_FIX: Only apply axis correction to wheel, not chassis */
/* In revolute_solve(), remove this block: */
/* if (!body_a->static_state) {
    body_a->angular_velocity = vector3_subtraction(
        body_a->angular_velocity,
        math3_multiplication_vector3(body_a->inverse_inertia_system, axis_impulse));
} */

/* Keep only the wheel correction: */
if (!body_b->static_state) {
    body_b->angular_velocity = vector3_addition(
        body_b->angular_velocity,
        math3_multiplication_vector3(body_b->inverse_inertia_system, axis_impulse));
}
```

**Verification:**
- Robot drives in straight line (no drift)
- Wheels stay aligned with chassis
- Add headless test: `revolute_chassis_no_drift`

**Files touched:** `physics/revolute_joint.c`

**Estimated time:** 1 hour

---

### 1.5 — NEW-02: Cylinder-Cube Normal Direction

**Why:** The contact normal points to cylinder center, not closest axle point. This produces wrong bounce directions.

**Root cause:** In `collision_cylinder_cube()`, the normal is computed from `cyl->position` to `best_on_obb`, but should be from closest axle point to `best_on_obb`.

**Fix:**
```c
/* MFS_180_CYLINDER_CUBE_NORMAL: Normal from closest axle point, not center */
/* In collision_cylinder_cube(), change: */
/* out->normal_vector = vector3_scaling(
    vector3_subtraction(cyl->position, best_on_obb), ...); */

/* To: */
vector3 closest_axle_point = /* compute from axle segment */;
out->normal_vector = vector3_scaling(
    vector3_subtraction(closest_axle_point, best_on_obb),
    1.0f / vector3_length(vector3_subtraction(closest_axle_point, best_on_obb)));
```

**Verification:**
- Cylinder bounces off cube at correct angle
- Add headless test: `cylinder_cube_normal_direction`

**Files touched:** `physics/collision_mechanics.c`

**Estimated time:** 2 hours

---

### 1.6 — NEW-06: Revolute Constraint Iteration

**Why:** Point-to-point and axis constraints are solved sequentially in one pass. This produces drift.

**Root cause:** In `revolute_solve()`, the point constraint is solved first, then the axis constraint. The axis correction can violate the point constraint.

**Fix:**
```c
/* MFS_181_REVOLUTE_ITERATE: Solve constraints iteratively */
void revolute_solve(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    const int iterations = 3;
    for (int iter = 0; iter < iterations; iter++) {
        /* Solve point-to-point constraint */
        /* ... existing point constraint code ... */
        
        /* Solve axis alignment constraint */
        /* ... existing axis constraint code ... */
    }
}
```

**Verification:**
- Robot drives in straight line (no drift)
- Wheels stay aligned
- Add headless test: `revolute_constraint_stability`

**Files touched:** `physics/revolute_joint.c`

**Estimated time:** 2 hours

---

### 1.7 — NEW-08: Mecanum Tangent Assumes Floor Normal

**Why:** The mecanum tangent computation assumes the contact normal is the floor normal. If a wheel contacts a wall or another object, the tangent is wrong.

**Root cause:** In `collision_prepare_solver()`, the mecanum tangent is computed using `m->normal_vector` as the floor normal, but it could be any surface.

**Fix:**
```c
/* MFS_182_MECANUM_FLOOR_CHECK: Only apply mecanum tangent for floor contacts */
/* In collision_prepare_solver(), add check: */
bool is_floor_contact = (fabsf(m->normal_vector.y) > 0.9f); /* Normal is mostly vertical */

if (mecanum_wheel && mecanum_wheel->type == object_cylinder && is_floor_contact) {
    /* Compute mecanum tangent */
    /* ... existing mecanum tangent code ... */
} else {
    /* Use standard tangent computation */
    /* ... existing standard tangent code ... */
}
```

**Verification:**
- Mecanum strafe works on floor
- Mecanum wheel contacting wall uses standard friction
- Add headless test: `mecanum_wall_contact`

**Files touched:** `physics/collision_mechanics.c`

**Estimated time:** 2 hours

---

### 1.8 — NEW-03: Cylinder-Cube Sample Count

**Why:** Only 5 axle samples are tested. Contacts between samples are missed.

**Root cause:** In `collision_cylinder_cube()`, only 5 samples along the axle are tested. For long cylinders, contacts between samples are missed.

**Fix:**
```c
/* MFS_183_CYLINDER_CUBE_SAMPLES: Increase sample count for long cylinders */
/* In collision_cylinder_cube(), change: */
/* const int SAMPLES = 5; */

/* To: */
int SAMPLES = 5;
if (h > 0.1f) SAMPLES = 10; /* Long cylinders need more samples */
if (h > 0.2f) SAMPLES = 20; /* Very long cylinders need even more */
```

**Verification:**
- Long cylinder contacts cube at all points
- Add headless test: `cylinder_cube_long_cylinder`

**Files touched:** `physics/collision_mechanics.c`

**Estimated time:** 1 hour

---

### 1.9 — NEW-09: Mecanum Tangent Assumes Upright Wheel

**Why:** The mecanum tangent computation assumes the wheel is upright (axle horizontal). If the wheel tilts, the tangent is wrong.

**Root cause:** In `collision_prepare_solver()`, the mecanum tangent is computed assuming the axle is horizontal. If the wheel tilts, the roller direction is wrong.

**Fix:**
```c
/* MFS_184_MECANUM_TILT: Account for wheel tilt in mecanum tangent */
/* In collision_prepare_solver(), add tilt check: */
float axle_vertical_component = fabsf(axle_world.y);
bool is_upright = (axle_vertical_component < 0.1f); /* Axle is mostly horizontal */

if (mecanum_wheel && mecanum_wheel->type == object_cylinder && is_floor_contact && is_upright) {
    /* Compute mecanum tangent */
    /* ... existing mecanum tangent code ... */
} else {
    /* Use standard tangent computation */
    /* ... existing standard tangent code ... */
}
```

**Verification:**
- Mecanum strafe works with upright wheels
- Tilted wheels use standard friction
- Add headless test: `mecanum_tilted_wheel`

**Files touched:** `physics/collision_mechanics.c`

**Estimated time:** 2 hours

---

### 1.10 — list-17: Velocity Cap Bypasses Solver

**Why:** The 3.0 m/s velocity cap in `drivetrain_update()` directly overwrites `chassis->velocity`, bypassing the impulse solver. This interferes with collision responses.

**Root cause:** In `drivetrain_update()`, the velocity cap directly overwrites `chassis->velocity` after the solver has run.

**Fix:**
```c
/* MFS_185_VELOCITY_CAP: Apply velocity cap as impulse, not direct overwrite */
/* In drivetrain_update(), change: */
/* if (speed_sq > max_speed * max_speed) {
    float speed = sqrtf(speed_sq);
    float scale = max_speed / speed;
    chassis->velocity.x *= scale;
    chassis->velocity.z *= scale;
} */

/* To: */
if (speed_sq > max_speed * max_speed) {
    float speed = sqrtf(speed_sq);
    float excess_speed = speed - max_speed;
    vector3 excess_velocity = vector3_scaling(
        vector3_normalisation(chassis->velocity), excess_speed);
    /* Apply as impulse to remove excess velocity */
    chassis->velocity = vector3_subtraction(chassis->velocity, excess_velocity);
}
```

**Verification:**
- Robot doesn't exceed 3.0 m/s
- Collision responses are not affected by velocity cap
- Add headless test: `velocity_cap_collision_response`

**Files touched:** `robotics/drivetrain.c`

**Estimated time:** 1 hour

---

### 1.11 — NEW-13: Coulomb Constant Not Derived

**Why:** The idle-hold Coulomb constant (2.0 N) is hardcoded and not derived from the motor model. This produces wrong holding force.

**Root cause:** In `drivetrain_update()`, the Coulomb constant is hardcoded as `2.0f` instead of being derived from the motor's back-drive friction.

**Fix:**
```c
/* MFS_186_COULOMB_DERIVE: Derive Coulomb constant from motor model */
/* In drivetrain_update(), change: */
/* float mfs_coulomb = 2.0f; */

/* To: */
/* Derive from motor back-drive friction:
* Back-drive friction = stall_torque / gear_ratio * (1 - efficiency)
* For 5203-30: 2.55 / 30 * 0.15 = 0.01275 N·m at motor shaft
* At wheel: 0.01275 * 30 = 0.3825 N·m
* Force at contact: 0.3825 / 0.05 = 7.65 N per wheel
* For 4 wheels: 7.65 * 4 = 30.6 N total
* But this is too high, so use a fraction: 0.1 * 30.6 = 3.06 N
*/
float motor_stall_torque = 2.55f; /* N·m, from motor preset */
float gear_ratio = 30.0f; /* from motor preset */
float efficiency = 0.85f; /* from motor preset */
float wheel_radius = 0.05f; /* m */
float back_drive_friction = motor_stall_torque / gear_ratio * (1.0f - efficiency);
float wheel_torque = back_drive_friction * gear_ratio;
float wheel_force = wheel_torque / wheel_radius;
float mfs_coulomb = wheel_force * 0.1f; /* Use 10% of back-drive friction */
```

**Verification:**
- Robot holds position when stationary
- Holding force matches motor back-drive friction
- Add headless test: `idle_hold_force_derivation`

**Files touched:** `robotics/drivetrain.c`

**Estimated time:** 2 hours

---

### 1.12 — NEW-15: Gear Efficiency Not Modeled

**Why:** The motor model doesn't model gear efficiency separately from motor efficiency. This produces wrong output torque.

**Root cause:** In `motor_update()`, the efficiency is applied at the motor shaft, but not at the gear output. Gear efficiency should reduce output torque.

**Fix:**
```c
/* MFS_187_GEAR_EFFICIENCY: Model gear efficiency separately */
/* In motor_update(), change: */
/* m->output_torque = m->torque * m->gear_ratio; */

/* To: */
float gear_efficiency = 0.90f; /* 90% gear efficiency */
m->output_torque = m->torque * m->gear_ratio * gear_efficiency;
```

**Verification:**
- Output torque matches expected value (motor torque * gear ratio * gear efficiency)
- Add headless test: `gear_efficiency_torque`

**Files touched:** `robotics/motor.c`

**Estimated time:** 1 hour

---

## 5. Phase 2: Data Safety & Robustness

**Goal:** Fix the 15 data safety / brick-robustness issues. These can cause data loss, NaN, or unrecoverable states.

**Timeline:** 2-3 days

**Dependencies:** Phase 1 complete (physics truth established)

**Priority order:** Fix in order of data-loss severity.

---

### 2.1 — R3-02: Scene Load Destroys Scene

**Why:** `scene_loading()` destroys the live scene before validating the file. A corrupt file destroys the scene.

**Root cause:** In `scene_loading()`, `scene_clear()` is called before reading the file. If the file is corrupt, the scene is destroyed.

**Fix:**
```c
/* MFS_188_SCENE_LOAD_STAGING: Read into staging buffer first */
int scene_loading(const char *file_source_path) {
    FILE *f = fopen(file_source_path, "rb");
    if (!f) {
        fprintf(stderr, "Error LDF01: Could not open %s
", file_source_path);
        return 0;
    }
    
    /* Read header */
    int32_t magic, version, count;
    if ((!read_int(f, &magic)) || (magic != mpe_magic)) {
        fprintf(stderr, "Error LDF02: Invalid magic number
");
        fclose(f);
        return 0;
    }
    if ((!read_int(f, &version)) || (version != mpe_version && version != 130 && version != 140)) {
        fprintf(stderr, "Error LDF03: Version mismatch
");
        fclose(f);
        return 0;
    }
    if ((!read_int(f, &count)) || (count < 0)) {
        fclose(f);
        return 0;
    }
    
    /* Read into staging buffer */
    rigidbody staging[mpe_max_bodies];
    int loaded_count = 0;
    for (int i = 0; i < count; i++) {
        /* Read body into staging[i] */
        /* ... existing read code ... */
        loaded_count++;
    }
    
    /* Only clear and commit if all reads succeeded */
    scene_clear();
    memcpy(obj_per_scene, staging, loaded_count * sizeof(rigidbody));
    object_count = loaded_count;
    
    fclose(f);
    return 1;
}
```

**Verification:**
- Corrupt file doesn't destroy scene
- Valid file loads successfully
- Add headless test: `scene_load_corrupt_file`

**Files touched:** `scene/scene_load.c`

**Estimated time:** 2 hours

---

### 2.2 — R3-06: Sleeping Bodies Can't Wake

**Why:** Sleeping bodies in `physics_world` can never be woken by contact. This causes pile instability.

**Root cause:** In `physics_world_step()`, there's no wake-on-contact logic. The legacy path has it, but `physics_world` doesn't.

**Fix:**
```c
/* MFS_189_WAKE_ON_CONTACT: Add wake-on-contact logic to physics_world_step */
/* In physics_world_step(), after narrowphase dispatch, add: */
for (int m = 0; m < manifold_count; m++) {
    rigidbody *body_a = world_manifolds[m].object_a;
    rigidbody *body_b = world_manifolds[m].object_b;
    
    bool a_sleeping = body_a->is_sleeping;
    bool b_sleeping = body_b->is_sleeping;
    
    if (a_sleeping && b_sleeping) continue;
    
    float wake_linear_thresh_sq = g_cfg.sleep.wake_linear_thresh_sq;
    float wake_angular_thresh_sq = g_cfg.sleep.wake_angular_thresh_sq;
    
    bool a_active = (!a_sleeping) &&
        ((vector3_length_squared(body_a->velocity) > wake_linear_thresh_sq) ||
         (vector3_length_squared(body_a->angular_velocity) > wake_angular_thresh_sq));
    bool b_active = (!b_sleeping) &&
        ((vector3_length_squared(body_b->velocity) > wake_linear_thresh_sq) ||
         (vector3_length_squared(body_b->angular_velocity) > wake_angular_thresh_sq));
    
    if (a_sleeping && (!body_b->static_state) && b_active) {
        rigidbody_wake(body_a);
    }
    if (b_sleeping && (!body_a->static_state) && a_active) {
        rigidbody_wake(body_b);
    }
}
```

**Verification:**
- Sleeping body wakes when hit by moving body
- Add headless test: `sleep_wake_on_contact`

**Files touched:** `core/physics_world.c`

**Estimated time:** 2 hours

---

### 2.3 — R3-07: No Containment Walls

**Why:** `physics_world` has no containment walls. Bodies drive off into infinity and produce NaN.

**Root cause:** `physics_world_init()` doesn't add containment walls. The legacy path has boundary teleporter, but `physics_world` doesn't.

**Fix:**
```c
/* MFS_190_CONTAINMENT_WALLS: Add containment walls to physics_world_init */
void physics_world_init(physics_world *world) {
    /* ... existing init code ... */
    
    /* Add containment walls (12ft x 12ft FTC field) */
    float half_width = 1.8288f; /* 6 ft */
    float half_depth = 1.8288f; /* 6 ft */
    float wall_height = 0.3048f; /* 12 in */
    float wall_thickness = 0.0254f; /* 1 in */
    
    /* North wall */
    physics_world_add_cube(world,
        (vector3){0.0f, wall_height * 0.5f, half_depth},
        (vector3){half_width, wall_height * 0.5f, wall_thickness * 0.5f},
        0.0f);
    
    /* South wall */
    physics_world_add_cube(world,
        (vector3){0.0f, wall_height * 0.5f, -half_depth},
        (vector3){half_width, wall_height * 0.5f, wall_thickness * 0.5f},
        0.0f);
    
    /* East wall */
    physics_world_add_cube(world,
        (vector3){half_width, wall_height * 0.5f, 0.0f},
        (vector3){wall_thickness * 0.5f, wall_height * 0.5f, half_depth},
        0.0f);
    
    /* West wall */
    physics_world_add_cube(world,
        (vector3){-half_width, wall_height * 0.5f, 0.0f},
        (vector3){wall_thickness * 0.5f, wall_height * 0.5f, half_depth},
        0.0f);
}
```

**Verification:**
- Robot can't drive off field
- No NaN after extended driving
- Add headless test: `containment_walls`

**Files touched:** `core/physics_world.c`

**Estimated time:** 2 hours

---

### 2.4 — R3-01: No Determinism Contract

**Why:** There's no test that proves same-scene → same-state. This is critical for autonomous tuning.

**Root cause:** The solver is Gauss-Seidel, so results depend on manifold ordering. There's no test that verifies determinism.

**Fix:**
Add determinism test:
```c
/* MFS_191_DETERMINISM_TEST: Verify same-scene → same-state */
/* In tests/determinism_test.c: */
int main(void) {
    mpe_config_init();
    
    /* Build fixed scene */
    physics_world world;
    physics_world_init(&world);
    /* Add fixed objects */
    /* ... */
    
    /* Run 3000 ticks, hash state */
    uint64_t hash1 = run_and_hash(&world, 3000);
    
    /* Reset and run again */
    physics_world_cleanup(&world);
    physics_world_init(&world);
    /* Add same fixed objects */
    /* ... */
    
    uint64_t hash2 = run_and_hash(&world, 3000);
    
    /* Hashes must match */
    if (hash1 != hash2) {
        printf("[FAIL] Determinism test failed: hash1=%llu, hash2=%llu
", hash1, hash2);
        return 1;
    }
    
    printf("[PASS] Determinism test passed
");
    return 0;
}
```

**Verification:**
- Same scene produces same state after 3000 ticks
- Add headless test: `determinism_test`

**Files touched:** `tests/determinism_test.c` (new file)

**Estimated time:** 3 hours

---

### 2.5 — R3-03: No Atomic Writes

**Why:** `save_scene()` and `mpe_config_save()` use `fopen(path, "w")`, which truncates immediately. A crash mid-write corrupts the file.

**Root cause:** In `save_scene()` and `mpe_config_save()`, the file is truncated immediately. If the process crashes mid-write, the file is corrupted.

**Fix:**
```c
/* MFS_192_ATOMIC_WRITES: Write to temp file, then rename */
int save_scene(const char *file_destination_path) {
    char tmp_path[512];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", file_destination_path);
    
    FILE *f = fopen(tmp_path, "wb");
    if (!f) {
        fprintf(stderr, "Error SVF01: Could not open %s
", tmp_path);
        return 0;
    }
    
    /* Write to temp file */
    /* ... existing write code ... */
    
    fclose(f);
    
    /* Atomic rename */
    if (rename(tmp_path, file_destination_path) != 0) {
        fprintf(stderr, "Error SVF02: Could not rename %s to %s
", tmp_path, file_destination_path);
        return 0;
    }
    
    return 1;
}
```

**Verification:**
- Crash mid-write doesn't corrupt file
- Valid file saves successfully
- Add headless test: `atomic_write_crash`

**Files touched:** `scene/scene_saving.c`, `config/mpe_config.c`

**Estimated time:** 2 hours

---

### 2.6 — R3-04: Cylinders Corrupted on Save/Load

**Why:** Cylinders load back as spheres because `cylinder_half_length` is not saved.

**Root cause:** In `save_scene()`, `cylinder_half_length` is not saved. In `scene_loading()`, there's no cylinder branch.

**Fix:**
```c
/* MFS_193_CYLINDER_SAVE: Save cylinder_half_length */
/* In save_scene(), add: */
if (rb->type == object_cylinder) {
    write_float(f, rb->cylinder_half_length);
}

/* In scene_loading(), add cylinder branch: */
if (temp.type == object_cylinder) {
    float half_length;
    if (!read_float(f, &half_length)) break;
    rigidbody_initialisation_cylinder(&obj_per_scene[i], temp.radius, half_length, temp.mass, temp.position);
}
```

**Verification:**
- Cylinder saves and loads correctly
- Add headless test: `cylinder_save_load`

**Files touched:** `scene/scene_saving.c`, `scene/scene_load.c`

**Estimated time:** 2 hours

---

### 2.7 — R3-11: Kernel Contaminated with MFS

**Why:** The kernel (`rigidbody.h`, `physics_world.c`, `mpe_engine.h`) contains MFS-specific state (`is_mecanum`, `roller_angle_rad`, `driven_this_tick`). This blocks the v16 module split.

**Root cause:** MFS state is welded into the kernel structs.

**Fix:**
This is a large refactor. For now, document as known debt and defer to v16.

**Verification:**
- Document as known debt
- File as v16 task

**Files touched:** None (documentation only)

**Estimated time:** 1 hour (documentation)

---

### 2.8 — R3-08: File-Global Scratch Buffers

**Why:** `physics_world_step()` uses file-global scratch buffers. Two worlds stepped in the same frame clobber each other.

**Root cause:** `world_pairs[]` and `world_manifolds[]` are file-static globals.

**Fix:**
This is a large refactor. For now, document as known debt and defer to v16.

**Verification:**
- Document as known debt
- File as v16 task

**Files touched:** None (documentation only)

**Estimated time:** 1 hour (documentation)

---

### 2.9 — R3-09: Broadphase Overflow Silently Removes Objects

**Why:** When the node pool is exhausted, objects are silently removed from collision. This produces wrong physics with no warning.

**Root cause:** In `broadphase_generate_pairing()`, when the node pool is exhausted, objects are silently not inserted.

**Fix:**
```c
/* MFS_194_BROADPHASE_OVERFLOW_WARNING: Warn on overflow */
/* In broadphase_generate_pairing(), add: */
if (node_count >= node_pool_capacity) {
    fprintf(stderr, "[WARN] Broadphase node pool exhausted: %d/%d
", node_count, node_pool_capacity);
    /* Continue, but objects will not collide */
}
```

**Verification:**
- Overflow produces warning
- Add headless test: `broadphase_overflow_warning`

**Files touched:** `physics/broadphase.c`

**Estimated time:** 1 hour

---

### 2.10 — R3-10: Floor Proxy Poisons Cache

**Why:** The floor proxy uses sentinel ID `0xFFFFFFFF`, which poisons the warm-start cache.

**Root cause:** The floor proxy uses a sentinel ID that can collide with real object IDs.

**Fix:**
This is a large refactor. For now, document as known debt and defer to v16.

**Verification:**
- Document as known debt
- File as v16 task

**Files touched:** None (documentation only)

**Estimated time:** 1 hour (documentation)

---

### 2.11 — R3-12: next_object_id Can Wrap

**Why:** `next_object_id` can wrap, producing ID collisions.

**Root cause:** `scene_allocate_object_id()` increments a static uint32 with no uniqueness check.

**Fix:**
```c
/* MFS_195_ID_WRAP: Check for ID collisions */
uint32_t scene_allocate_object_id(void) {
    uint32_t new_id = next_object_id++;
    
    /* Check for collision with existing IDs */
    while (scene_object_id_exists(new_id)) {
        new_id = next_object_id++;
    }
    
    return new_id;
}
```

**Verification:**
- No ID collisions after wrap
- Add headless test: `id_wrap_collision`

**Files touched:** `scene/scene_init.c`

**Estimated time:** 1 hour

---

### 2.12 — R3-13: No Teardown for physics_world

**Why:** `physics_world_cleanup()` is not called on exit. This leaks memory.

**Root cause:** `on_main_window_destroy()` doesn't call `physics_world_cleanup()`.

**Fix:**
```c
/* MFS_196_TEARDOWN: Call physics_world_cleanup on exit */
/* In on_main_window_destroy(), add: */
physics_world_cleanup(physics_world_get_primary());
```

**Verification:**
- No memory leaks on exit
- Add headless test: `teardown_memory_leak`

**Files touched:** `root_gtk.c`

**Estimated time:** 1 hour

---

### 2.13 — R3-14: Baumgarte Bias Hardcodes 60.0f

**Why:** The Baumgarte bias hardcodes `* 60.0f` instead of `/ dt`. This produces wrong correction if timestep changes.

**Root cause:** In `collision_prepare_solver()`, the Baumgarte bias hardcodes `* 60.0f`.

**Fix:**
```c
/* MFS_197_BAUMGARTE_DT: Use / dt instead of * 60.0f */
/* In collision_prepare_solver(), change: */
/* cp->separation_bias = bias_factor * fmaxf(cp->penetration - penetration_slop, 0.0f) * 60.0f; */

/* To: */
cp->separation_bias = bias_factor * fmaxf(cp->penetration - penetration_slop, 0.0f) / dt;
```

**Verification:**
- Baumgarte bias scales with timestep
- Add headless test: `baumgarte_dt_scaling`

**Files touched:** `physics/collision_mechanics.c`

**Estimated time:** 1 hour

---

### 2.14 — R3-05: Corrupt Config Lines Silently Dropped

**Why:** Corrupt config lines are silently dropped. This produces wrong config with no warning.

**Root cause:** In `mpe_config_load()`, corrupt lines are silently skipped.

**Fix:**
```c
/* MFS_198_CONFIG_CORRUPT_WARNING: Warn on corrupt config lines */
/* In mpe_config_load(), add: */
if ((endptr == value_part) || (!isfinite(parsed))) {
    fprintf(stderr, "[WARN] Corrupt config line %d: %s
", line_number, line);
    continue;
}
```

**Verification:**
- Corrupt config line produces warning
- Add headless test: `config_corrupt_warning`

**Files touched:** `config/mpe_config.c`

**Estimated time:** 1 hour

---

### 2.15 — R3-15: tee/MicroVim Can Clobber Source

**Why:** `tee` and MicroVim can write to arbitrary paths, including source files.

**Root cause:** `cmd_tee` and MicroVim can write to any path.

**Fix:**
```c
/* MFS_199_TERMINAL_SANDBOX: Restrict terminal writes to status/ folder */
/* In cmd_tee(), add: */
if (strstr(output_filename, "src/") || strstr(output_filename, "makefile")) {
    fprintf(stderr, "[ERROR] Cannot write to source files from terminal
");
    return;
}
```

**Verification:**
- Terminal cannot write to source files
- Add headless test: `terminal_sandbox`

**Files touched:** `ui_input/debug_terminal.c`, `ui_input/microvim.c`

**Estimated time:** 2 hours

---

## 6. Phase 3: Architecture Debt (v15S Preparation)

**Goal:** Pay down architecture debt to prepare for v16 module split.

**Timeline:** 1-2 weeks

**Dependencies:** Phase 2 complete (data safety established)

**Priority order:** Fix in order of impact on v16 module split.

---

### 3.1 — Kill Global State

**Why:** All scene state is file-scope globals. This blocks multithreading, multiple worlds, and unit tests.

**Root cause:** `obj_per_scene`, `object_count`, `main_inputs`, etc. are file-scope globals.

**Fix:**
This is a large refactor. Create `engine_state` struct:
```c
typedef struct {
    rigidbody *bodies;
    int body_count;
    int body_capacity;
    input_status inputs;
    camera cam;
    frame_timer timer;
    int selected_object;
} engine_state;

extern engine_state g_engine;
```

Then replace all global references with `g_engine.bodies`, etc.

**Verification:**
- No global state references outside `g_engine`
- Unit tests can create isolated `engine_state` instances

**Files touched:** All files (large refactor)

**Estimated time:** 1 week

---

### 3.2 — Kill Legacy Physics Pipeline

**Why:** Two physics pipelines exist (legacy GUI + physics_world). This produces inconsistent behavior.

**Root cause:** `simulation_physics_loop.c` (legacy) and `physics_world.c` (encapsulated) have different behavior.

**Fix:**
Migrate GUI to `physics_world`, delete `simulation_physics_loop.c`.

**Verification:**
- No `simulation_physics_loop.c` file
- GUI uses `physics_world`

**Files touched:** `simulation.c`, `scene_init.c`, `boundary.c` (delete)

**Estimated time:** 1 week

---

### 3.3 — Kill Sleep Staticize Hack

**Why:** Sleeping bodies get `inverse_mass = 0` mid-solve. This produces corrupted properties.

**Root cause:** In `simulation_physics_loop.c`, sleeping bodies get `inverse_mass = 0` before solver, then restored after.

**Fix:**
This is eliminated by killing the legacy pipeline (3.2).

**Verification:**
- No sleep staticize hack
- Sleeping bodies keep real mass

**Files tched:** None (eliminated by 3.2)

**Estimated time:** 0 (eliminated by 3.2)

---

### 3.4 — Kill Floor Proxy

**Why:** The floor proxy is a static rigidbody, not a real body. This poisons the warm-start cache.

**Root cause:** `collision_static_plane_body_proxy()` returns a static rigidbody.

**Fix:**
This is eliminated by killing the legacy pipeline (3.2). The floor becomes a real static body in `physics_world`.

**Verification:**
- No floor proxy
- Floor is a real static body

**Files touched:** None (eminated by 3.2)

**Estimated time:** 0 (eliminated by 3.2)

---

### 3.5 — Kill Boundary Teleporter

**Why:** The boundary teleporter teleports objects instead of using real walls. This produces energy discontinuities.

**Root cause:** `boundary_apply_box()` teleports objects instead of using real walls.

**Fix:**
This is eliminated by killing the legacy pipeline (3.2). Containment walls are added to `physics_world` (see 2.3).

**Verification:**
- No boundary teleporter
- Containment walls are real static bodies

**Files touched:** None (eliminated by 3.2)

**Estimated time:** 0 (eliminated by 3.2)

---

### 3.6 — Reduce mpe_engine.h Include Bloat

**Why:** `mpe_engine.h` includes 25+ headers. This slows compilation.

**Root cause:** `mpe_engine.h` includes all headers.

**Fix:**
Split into minimal headers:
```c
/* mpe_types.h — forward declarations */
typedef struct rigidbody rigidbody;
typedef struct camera camera;
/* ... */

/* mpe_scene.h — scene state */
extern engine_state g_engine.. */

/* mpe_input.h — input state */
extern input_status main_inputs;
/* ... */
```

Then each `.c` file includes only what it needs.

**Verification:**
- Compilation time decreases
- No circular dependencies

**Files touched:** `mpe_engine.h`, all `.c` files

**Estimated time:** 2 days

---

### 3.7 — Split debug_terminal.c God File

**Why:** `debug_terminal.c` is 146 KB. This is unmaintainable.

**Root cause:** All 70+ terminal commands are in one file.

**Fix:**
Split into:
```
debug_terminal_core.   — core terminal logic
debug_terminal_commands_fs.c — filesystem commands
debug_terminal_commands_net.c — network commands
debug_terminal_commands_edit.c — edit commands
```

**Verification:**
- No file > 2000 lines
- All terminal commands still work

**Files touched:** `ui_input/debug_terminal.c` → split into 4 files

**Estimated time:** 2 days

---

### 3.8 — Split collision_mechanics.c God File

**Why:** `collision_mechanics.c` is 59 KB. This is unmaintainable.

**Root cause:** All narrowph and contact cache are in one file.

**Fix:**
Split into:
```
narrowphase_sphere.c    — sphere collision
narrowphase_cube.c      — cube collision
narrowphase_cylinder.c  — cylinder collision
solver.c                — impulse solver
contact_cache.c         — contact cache
```

**Verification:**
- No file > 2000 lines
- All collision tests still pass

**Files touched:** `physics/collision_mechanics.c` → split into 5 files

**Estimated time:** 2 days

---

## 7. Phase 4: Advanced Physics (v16 Prerequisites)

**Goal:** Implement advanced physics features needed for v16.

**Timeline:** 2-4 weeks

**Dependencies:** Phase 3 complete (architecture debt paid)

**Priority order:** Fix in order of impact on v16 features.

---

### 4.1 — 2-Tangent Friction Cone

**Why:** Single-direction friction produces wrong friction direction. This is the root cause of the artificial lateral damping (NEW-05).

**Root cause:** In `collision_resolve_iterative()`, friction is solved along a single tangent vector.

**Fi**
```c
/* MFS_200_2_TANGENT_FRICTION: Solve friction along 2 tangents */
/* In collision_resolve_iterative(), add: */
vector3 tangent1 = cp->tangent_vector;
vector3 tangent2 = vector3_cross(m->normal_vector, tangent1);
tangent2 = vector3_normalisation(tangent2);

/* Solve friction along tangent1 */
float vt1 = vector3_dot(rel_vel, tangent1);
float lambda_t1 = -vt1 * eff_mass_t1;
/* Clamp to friction cone */
/* ... */

/* Solve friction along tangent2 */
float vt2 = vector3_dot(rel_vel, tangent2);
float lambda_t2 = -vt2 * eff_mass_t2;
/* Clamp to friction cone */
/* ... */

/* Clamp combined friction to friction cone */
float friction_magnitude = sqrtf(lambda_t1 * lambda_t1 + lambda_t2 * lambda_t2);
float max_friction = cp->accumulated_normal_impulse * friction_coeff;
if (friction_magnitude > max_friction) {
    float scale = max_friction / friction_magnitude;
    lambda_t1 *= scale;
    lambda_t2 *= scale;
}
```

**Verification:**
- No artificial lateral damping needed
- Friction direction is correct
- Add headless test: `2_tangent_friction_cone`

**Files touched:** `physics/collision_mechanics.c`

**Estimated time:** 1 week

---

### 4.2 — Exponential Map Quaternion Integration

**Why:** First-order Euler quaternion integration produces orientation lag at high angular velocities.

**Root cause:** In `rb_integrate_position()`, quaternion integration is first-order Euler.

**Fix:**
```c
/* MFS_201_EXPONENTIAL_MAP: Use exponential map for quaternion integration */
/* In rb_integrate_position(), change: */
/* vtor4 orientation_change_delta = vector4_multiplication(angular_velocity_quaternion, rigid_body->orientation);
rigid_body->orientation.w += orientation_change_delta.w * 0.5f * delta_time;
... */

/* To: */
float angle = vector3_length(rigid_body->angular_velocity) * delta_time;
if (angle > 1e-6f) {
    vector3 axis = vector3_scaling(rigid_body->angular_velocity, 1.0f / (angle / delta_time));
    vector4 dq = vector4_from_axis_with_angle(axis, angle);
    rigid_body->orientation = vector4_normalisation(
        vector4_multiplication(dq, rigid_body->orientation));
}
```

**Verification:**
- Orientation matches analytical solution at high angular velocities
- Add headless test: `exponential_map_quaternion`

**Files touched:** `core/rigidbody.c`

**Estimated time:** 2 days

---

### 4.3 — Solver Islanding

**Why:** All manifolds are solved in one global loop. Unrelated contacts interfere. This blocks parallel solving.

**Root cause:** In `physics_world_step()`, all manifolds are solved in one loop.

**Fix:**
``
/* MFS_202_SOLVER_ISLANDING: Solve islands independently */
/* In physics_world_step(), add: */
/* Build contact graph */
/* Find connected components (islands) */
/* Solve each island independently */
```

**Verification:**
- Unrelated contacts don't interfere
- Add headless test: `solver_islanding`

**Files touched:** `core/physics_world.c`

**Estimated time:** 1 week

---

### 4.4 — Rolling Friction

**Why:** Spheres roll forever. This is physically wrong.

**Root cause:** No rolling friction is implented.

**Fix:**
```c
/* MFS_203_ROLLING_FRICTION: Add rolling friction */
/* In collision_resolve_iterative(), add: */
float rolling_coeff = fminf(a->rolling_friction, b->rolling_friction);
if (rolling_coeff > 0.0f) {
    vector3 angular_vel_rel = vector3_subtraction(b->angular_velocity, a->angular_velocity);
    float rolling_torque_mag = rolling_coeff * cp->accumulated_normal_impulse;
    /* Apply opposing torque about contact normal */
    /* ... */
}
```

**Verification:**
- Spheres decelerate and stop
- Add headless test: `rolling_friction`

**Files touched:** `physics/collision_mechanics.c`

**Estimated time:** 2 days

---

### 4.5 — Continuous Collision Detection (CCD)

**Why:** Fast objects tunnel through thin geometry. This is physically wrong.

**Root cause:** No CCD is implemented.

**Fix:**
```c
/* MFS_204_CCD: Add swept-sphere CCD */
/* In physics_world_step(), add: */
/* Check if any body's velocity * dt exceeds its bounding radius */
/* If so, perform raycast along velocity vector */
/* If raast hits, clamp position to contact point */
```

**Verification:**
- Fast objects don't tunnel through thin geometry
- Add headless test: `ccd_tunneling`

**Files touched:** `core/physics_world.c`

**Estimated time:** 1 week

---

## 8. Testing & Validation Strategy

### Test Pyramid

```
                    /\
                   /  \
                  / E2E \        5%  — Full robot driving tests
                 /________\
                / Integration \   20% — Robot + physics integration
           /______________\
              /  Headless Tests \  30% — Physics truth, collision, solver
             /__________________\
            /   Unit Tests      \  45% — Math, collision functions, config
           /____________________\
```

### Test Categories

| Category | Count | Coverage |
|----------|-------|----------|
| Unit tests (math, collision, config) | 50+ | Math functions, collision functions, config system |
| Headless tests (physics truth, collision, solver) | 14+ | Physics truth, collisioolver |
| Integration tests (robot + physics) | 10+ | Robot + physics integration |
| E2E tests (full robot driving) | 5+ | Full robot driving tests |

### Test Execution

```bash
# Run all tests
python3 tools/test_runner.py

# Run specific test
python3 tools/test_runner.py physics_truth

# Run with verbose output
python3 tools/test_runner.py --verbose
```

### Test Coverage Goals

| Category | Current | Target |
|----------|---------|--------|
| Unit tests | 0 | 50+ |
| Headless tests | 14 | 30+ |
| Integration tests | 0 | 10+ |
| E2E tests | 0 | 5+ |

---

## 9. Rollout & Distribution Plan

### Distribution Package

```
windows_release/
├── engine.exe              ← Main executable
├── render/                 ← Shader files
│   ├── vertex_shader.glsl
│   ├── fragment_shader.glsl
│   ├── utility_vertex.glsl
│   ├── utility_fragment.glsl
│   ├── axis_vertex.glsl
│   └── axis_fragment.glsl
├── status/                 ← Config + scene saves engine.cfg
│   └── scene.dat
├── *.dll                   ← Runtime libraries (~50 files)
└── README.txt              ← Quick-start guide
```

### Distribution Steps

1. **Build release:**
```bash
cd ~/mpe/v15R3/src
make release
```

2. **Verify release:**
```bash
cd ../windows_release
./engine.exe
```

3. **Test on clean machine:**
   - Copy `windows_release.zip` to USB drive
   - Copy to different Windows PC (no MSYS2)
   - Extract, double-click `engine.exe`
   - Verify: window opet spawns, drives

4. **Distribute:**
   - Zip `windows_release/` folder
   - Distribute to FTC teams

### Distribution Checklist

- [ ] `engine.exe` runs from File Explorer
- [ ] No "DLL not found" errors
- [ ] Robot spawns and drives
- [ ] Config persists across restarts
- [ ] Scene save/load works
- [ ] No crash after 10 minutes
- [ ] Framerate > 30 FPS during stress test

---

## 10. Timeline & Milestones

### Phase 0: Emergency Stabilization (Gate 15e)
**Timeline:** 1-2 days

| Task | Estimated Time | Status |
|------|---------------|--------|
| 0.1 Run F5–F11 on Windows | 30 min | [ ] |
| 0.2 Run headless tests on Windows | 15 min | [ ] |
| 0.3 Scene save/load round-trip | 10 min | [ ] |
| 0.4 Config persistence | 10 min | [ ] |
| 0.5 10-minute idle soak | 10 min | [ ] |
| 0.6 Clean-machine test | 15 min | [ ] |
| 0.7 Folder layout enforcement | 15 min | [ ] |
| **Total** | **~2 hours** | — |

### Phase 1: Physics Truth (Training Quality)
**Timeline:** 3-5 days

| Task | Estimated Time | Status |
|--|---------------|--------|
| 1.1 Cylinder floor collision (barrel) | 2 hours | [ ] |
| 1.2 Remove artificial lateral damping | 1 hour | [ ] |
| 1.3 Odometry heading rotation | 1 hour | [ ] |
| 1.4 Revolute axis drift on chassis | 1 hour | [ ] |
| 1.5 Cylinder-cube normal direction | 2 hours | [ ] |
| 1.6 Revolute constraint iteration | 2 hours | [ ] |
| 1.7 Mecanum tangent assumes floor | 2 hours | [ ] |
| 1.8 Cylinder-cube sample count | 1 hour | [ ] |
| 1.9 Mecanum tangent assumes upright | 2 hours | [ ] |
| 1.10 Velocity cap bypasses solver | 1 hour | [ ] |
| 1.11 Coulomb constant not derived | 2 hours | [ ] |
| 1.12 Gear efficiency not modeled | 1 hour | [ ] |
| **Total** | **~16 hours** | — |

### Phase 2: Data Safety & Robustness
**Timeline:** 2-3 days

| Task | Estimated Time | Status |
|------|---------------|--------|
| 2.1 Scene load destroys scene | 2 hours | [ ] |
| 2.2 Sleeping bodies can't wake | 2 hours | [ ] |
| 2.3 No containment walls | 2 hours | [ ] |
| 2.4 No determinism contract | 3 hou | [ ] |
| 2.5 No atomic writes | 2 hours | [ ] |
| 2.6 Cylinders corrupted on save/load | 2 hours | [ ] |
| 2.7 Kernel contaminated with MFS | 1 hour | [ ] |
| 2.8 File-global scratch buffers | 1 hour | [ ] |
| 2.9 Broadphase overflow silently removes | 1 hour | [ ] |
| 2.10 Floor proxy poisons cache | 1 hour | [ ] |
| 2.11 next_object_id can wrap | 1 hour | [ ] |
| 2.12 No teardown for physics_world | 1 hour | [ ] |
| 2.13 Baumgarte bias hardcodes 60.0f | 1 hour | [ ] |
| 2.14 Corrupt config lines silently dropped | 1 hour | [ ] |
| 2.15 tee/MicroVim can clobber source | 2 hours | [ ] |
| **Total** | **~20 hours** | — |

### Phase 3: Architecture Debt (v15S Preparation)
**Timeline:** 1-2 weeks

| Task | Estimated Time | Status |
|------|---------------|--------|
| 3.1 Kill global state | 1 week | [ ] |
| 3.2 Kill legacy physics pipeline | 1 week | [ ] |
| 3.3 Kill sleep staticize hack | 0 (eliminated by 3.2) | [ ] |
| 3.4 Kill floor proxy | 0 (eliminated by 3.2) | [ ] |
| 3.5 Kill boundary teleporter | 0 liminated by 3.2) | [ ] |
| 3.6 Reduce mpe_engine.h include bloat | 2 days | [ ] |
| 3.7 Split debug_terminal.c god file | 2 days | [ ] |
| 3.8 Split collision_mechanics.c god file | 2 days | [ ] |
| **Total** | **~2 weeks** | — |

### Phase 4: Advanced Physics (v16 Prerequisites)
**Timeline:** 2-4 weeks

| Task | Estimated Time | Status |
|------|---------------|--------|
| 4.1 2-tangent friction cone | 1 week | [ ] |
| 4.2 Exponential map quaternion integration | 2 days | [ ] |
| 4.3 Solver islanding | 1 week | [ ] |
| 4.4 Rolling friction | 2 days | [ ] |
| 4.5 Continuous collision detection (CCD) | 1 week | [ ] |
| **Total** | **~3 weeks** | — |

### Total Timeline

| Phase | Timeline | Cumulative |
|-------|----------|------------|
| Phase 0: Emergency Stabilization | 1-2 days | 1-2 days |
| Phase 1: Physics Truth | 3-5 days | 4-7 days |
| Phase 2: Data Safety | 2-3 days | 6-10 days |
| Phase 3: Architecture Debt | 1-2 weeks | 2-3 weeks |
| Phase 4: Advanced Physics | 2-4 weeks | 4-7 wes |
| **Total** | **4-7 weeks** | — |

**But:** Phase 0+1+2 can be completed in **1-2 weeks** for initial classroom use.

---

## 11. Risk Register

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| Physics bugs produce wrong driving habits | High | High | Fix Phase 1 before classroom use |
| Data loss from corrupt scene/config | Medium | High | Fix Phase 2 before classroom use |
| Performance degradation at high object counts | Medium | Medium | Optimize broadphas add frustum culling |
| GTK3/OpenGL compatibility issues on Windows | Low | High | Test on multiple Windows machines |
| Gamepad compatibility issues | Low | Medium | Test multiple gamepad models |
| Memory leaks during extended use | Medium | Medium | Run memory profiler, fix leaks |
| Architecture debt blocks v16 module split | High | High | Pay down Phase 3 before v16 |

---

## 12. Decision Gates

### Gate 1: Phase 0 Complete (Gate 15e Cleared)
**Criteria:**
- All 7 Gate 15e items pass
- No crashes, no NaN, no explosions
- Framerate > 30 FPS during stress test

**Decision:** Proceed to Phase 1

### Gate 2: Phase 1 Complete (Physics Truth Established)
**Criteria:**
- All 12 Phase 1 items pass
- No artificial damping needed
- Robot drives in straight line (no drift)
- Odometry matches physics

**Decision:** Proceed to Phase 2

### Gate 3: Phase 2 Complete (Data Safety Established)
**Criteria:**
- All 15 Phase 2 items pass
- No data loss from corrupt scene/config
- No NaN after extended use
- Determinism test passes

**Decision:** Proceed to Phase 3, or **release to classroom** if Phase 0+1+2 complete

### Gate 4: Phase 3 Complete (Architecture Debt Paid)
**Criteria:**
- All 8 Phase 3 items pass
- No global state outside `g_engine`
- No legacy physics pipeline
- No god files > 2000 lines

**Decision:** Proceed to Phase 4

### Gate 5: Phase 4 Complete (v16 Prerequisites Met)
**Criteria:**
- All 5 Phase 4 items pass
- 2-tangent friction cone implemented
- Exponential map quaternion integration implemented
- Solver islanding implemented
- Rolling friction implemented
- CCD implemented

**Decision:** Proceed to v16 release

---

## Appendix A: File Inventory

### Files to Modify

| File | Phase | Changes |
|------|-------|---------|
| `physics/collision_mechanics.c` | 1, 2, 4 | Cylinder collision, friction cone, rolling friction |
| `robotics/drivetrain.c` | 1 | Remove artificial damping, odometry heading, velocity cap |
| `physics/revolute_joint.c` | 1 | Axis drift fix, constraint iteration |
| `core/physics_world.c` | 2, 3, 4 | Wake-on-contact, containment walls, solver islanding |
| `scene/scene_load.c` | 2 | Scene load staging, cylinder save/load |
| `scene/scene_saving.c` | 2 | Atomic writes, cylinder save |
| `config/mpe_config.c` | 2 | Atomic writes, corrupt config warning |
| `core/rigidbody.c` | 4 | Exponential map quaternion integration |
| `root_gtk.c` | 2 | Teardown, X11 guard |
| `makefile` | 0 | Folder layout enforcement |
| `ui_input/debug_terminal.c` | 2, 3 | Terminal sandbox, god file split |
| `ui_input/microvim.c` | 2 | Terminal sandbox |
| `mpe_engine.h` | 3 | Include bloat reduction |
| All files | 3 | Global state elimination |

### Files to Create

| File | Phase | Purpose |
|------|-------|---------|
| `tests/determinism_test.c` | 2 | Determinism test |
| `tests/cylinder_tipped_rest.c` | 1 | Cylinder tipped rest test |
| `tests/odometry_heading_rotation.c` | 1 | Odometry heading rotation test |
| `tests/revolute_chassis_no_drift.c` | 1 | Revolute chassis no drift test |
| `tests/cylinder_cube_normal_direction.c` | 1 | Cylinder-cube normal direction test |
| `tests/revolute_constraint_stability.c` | 1 | Revolute constraint stability test |
| `tests/mecanum_wall_contact.c` | 1 | Mecanum wall contact test |
| `tests/cylinder_cube_long_cylinder.c` | 1 | Cylinder-cube long cylinder test |
| `tests/mecanum_tilted_wheel.c` | 1 | Mecanum tilted wheel test |
| `tests/velocity_cap_collision_response.c` | 1 | Velocity cap collision response test |
| `tests/idle_hold_force_derivation.c` | 1 | Idle hold force derivation test |
| `tests/gear_efficiency_torque.c` | 1 | Gear efficiency torque test |
| `tests/scene_load_corrupt_file.c` | 2 | Scene load corrupt file test |
| `tests/sleep_wake_on_contact.c` | 2 | Sleep wake on contact test |
| `tests/containment_walls.c` | 2 | Containment walls test |
| `tests/atomic_write_crash.c` | 2 | Atomic write crash test |
| `tests/cylinder_save_load.c` | 2 | Cylinder save/load test |
| `tests/broadphase_overflow_warning.c` | 2 | Broadphase overflow warning test |
| `tests/id_wrap_collision.c` | 2 | ID wrap collision test |
| `tests/teardown_memory_leak.c` | 2 | Teardown memory leak test |
| `tests/baumgarte_dt_scaling.c` | 2 | Baumgarte dt scaling test |
| `tests/config_corrupt_warning.c` | 2 | Config corrupt warning test |
| `tests/terminal_sandbox.c` | 2 | Terminal sandbox test |
| `tests/2_tangent_friction_cone.c` | 4 | 2-tangent friction cone test |
| `tests/exponential_map_quaternion.c` | 4 | Exponential map quaternion test |
| `tests/solver_islanding.c` | 4 | Solver islanding test |
| `tests/rolling_friction.c` | 4 | Rolling friction test |
| `tests/ccd_tunneling.c` | 4 | CCD tunneling test |

### Files to Delete

| File | Phase | Reason |
|------|-------|--------|
| `core/simulation_physics_loop.c` | 3 | Eliminated by physics_world migration |
| `scene/boundary.c` | 3 | Eliminated by containment walls |

---

## Appendix B: Verification Commands

### Build Verification
```bash
# Clean build
make clean
make

# Verify no warnings
make 2>&1 | grep -i warning

# Verify no errors
make 2>&1 | grep -i error
```

### Test Verification
```bash
# Run all tests
python3 tools/test_runner.py

# Run specific test
python3 tools/test_runner.py physics_truth

# Run with verbose output
python3 tools/test_runner.py --verbose
```

### Runtime Verification
```bash
# Launch engine
./engine.exe

# Verify no crash after 10 minutes
# (observe for 10 minutes)

# Verify no NaN in console
# (observe console output)
```

---

## Appendix C: Decision Log

| Date | Decision | Rationale |
|------|----------|-----------|
| 2026-09-07 | Created execution plan | Transform v10R3I to FTC-team-ready |
| — | — | — |

---

**End of Execution Plan**
