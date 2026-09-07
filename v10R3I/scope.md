# MFS-W Scope — Defect & Debt Register
**Tree:** MFS-W (Windows port of v15R3) · **Date:** 2026-09-07 · **Status:** Active

> This document supersedes the v15R2 scope.md audit. All items from
> `the_list.txt`, `the_list_2.txt`, and `the_list_3.txt` have been
> re-evaluated against the current source. Items resolved by fix campaigns
> 155–176 are marked ✅. Everything else is carried forward or newly filed.

---

## Tier 0 — Teacher Handoff Blockers (Gate 15e)

These must be green before the zip goes to the classroom.

| ID | Issue | File(s) | Status |
|---|---|---|---|
| W-01 | F5–F11 not yet run on Windows build | in-engine | 🔲 |
| W-02 | Headless suite not verified under MinGW (`test_runner.py`) | makefile, tools/ | 🔲 |
| W-03 | Scene save/load round-trip untested on Windows paths | scene_saving.c, scene_load.c | 🔲 |
| W-04 | Config persistence (change → exit → relaunch → survives) untested | mpe_config.c | 🔲 |
| W-05 | 10-minute idle soak (no crash, no NaN) not run on Windows | engine | 🔲 |
| W-06 | Clean-machine zip test (no MSYS2 installed) not done | windows_release/ | 🔲 |
| W-07 | `render/` + `status/` folder layout documented but not enforced at build time | makefile | 🔲 |

---

## Tier 1 — Training-Quality Physics (grade 9s will see these)

Defects that produce visibly wrong robot behaviour during driving practice.

| ID | Issue | Severity | File(s) | Source |
|---|---|---|---|---|
| NEW-01 | Cylinder floor collision tests axle endpoints only — tipped wheel falls through | 🔴 Critical | collision_mechanics.c | Physics audit |
| NEW-05 | Artificial lateral damping (`PHYSICS LIE`) masks missing solver friction | 🟠 High | drivetrain.c | Physics audit |
| NEW-04 | Odometry integrates world-space velocity without heading rotation | 🟠 High | drivetrain.c | Physics audit |
| NEW-07 | Revolute axis-drift correction applied to chassis (reference body rotates) | 🟠 High | revolute_joint.c | Physics audit |
| NEW-02 | Cylinder-cube normal points to centre, not closest axle point | 🟠 High | collision_mechanics.c | Physics audit |
| NEW-06 | Revolute point + axis constraints solved sequentially, no iteration | 🟡 Medium | revolute_joint.c | Physics audit |
| NEW-08 | Mecanum tangent assumes contact normal ≈ floor normal | 🟡 Medium | collision_mechanics.c | Physics audit |
| NEW-03 | Cylinder-cube uses 5 axle samples — contacts between samples missed | 🟡 Medium | collision_mechanics.c | Physics audit |
| NEW-09 | Mecanum tangent assumes upright wheel (breaks when tilted) | 🟡 Medium | collision_mechanics.c | Physics audit |
| list-17 | `drivetrain_update` velocity cap of 3.0 m/s bypasses impulse solver | 🟡 Medium | drivetrain.c | the_list.txt |
| NEW-13 | Idle-hold Coulomb constant (2.0 N) not derived from motor model | ⚪ Low | drivetrain.c | Physics audit |
| NEW-15 | Gear efficiency not modelled separately from motor efficiency | ⚪ Low | motor.c | Physics audit |

---

## Tier 2 — Brick-Robustness (engine can lose data or lock up)

From `the_list_3.txt`. None resolved.

| ID | Issue | Severity | File(s) |
|---|---|---|---|
| R3-02 | `scene_loading` destroys live scene before validating file | 🔴 Critical | scene_load.c |
| R3-06 | `physics_world` sleeping bodies can never be woken by contact | 🟠 High | physics_world.c |
| R3-07 | `physics_world` has no containment — bodies escape to infinity/NaN | 🟠 High | physics_world.c |
| R3-01 | No determinism contract; no test proves same-scene → same-state | 🟠 High | collision_mechanics.c, physics_world.c |
| R3-03 | No atomic writes for `scene.dat` or `engine.cfg` | 🟠 High | scene_saving.c, mpe_config.c |
| R3-04 | Cylinders silently corrupted to spheres on save/load | 🟠 High | scene_saving.c, scene_load.c |
| R3-11 | Kernel contaminated with MFS-specific state (`is_mecanum`, gamepad) | 🟠 High | rigidbody.h, physics_world.c, mpe_engine.h |
| R3-08 | `physics_world_step` uses file-global scratch — not truly multi-world | 🟡 Medium | physics_world.c, broadphase.c |
| R3-09 | Broadphase overflow silently removes objects from collision | 🟡 Medium | broadphase.c |
| R3-10 | Floor proxy poisons warm-start cache (sentinel ID 0xFFFFFFFF) | 🟡 Medium | collision_mechanics.c |
| R3-12 | `next_object_id` can wrap; generation field is dead | 🟡 Medium | scene_init.c |
| R3-13 | No teardown for primary physics_world or GPU resources on exit | ⚪ Low | root_gtk.c |
| R3-14 | Baumgarte separation bias hardcodes `* 60.0f` instead of `/ dt` | ⚪ Low | collision_mechanics.c |
| R3-05 | Corrupt config lines silently dropped | ⚪ Low | mpe_config.c |
| R3-15 | `tee` / MicroVim can clobber project source files | ⚪ Low | debug_terminal.c, microvim.c |

---

## Tier 3 — Solver & Integration Correctness

From `the_list_2.txt` Phase 2 and physics audit. None resolved.

| ID | Issue | File(s) | Source |
|---|---|---|---|
| 2.1 | Single-direction friction tangent (should be 2-tangent cone) | collision_mechanics.c | the_list_2 |
| 2.2 | Tangent effective mass not recomputed when tangent changes mid-solve | collision_mechanics.c | the_list_2 |
| 2.3 | Warm-start tangent mismatch across frames | collision_mechanics.c | the_list_2 |
| 2.4 | Restitution bias computed once, not updated during iterations | collision_mechanics.c | the_list_2 |
| 2.5 | Quaternion integration is first-order Euler (should be exponential map) | rigidbody.c | the_list_2 |
| 2.6 | Sleep decision inside position integration (mid-step) | physics_world.c | the_list_2 |
| 2.7 | Angular drag hidden `* 0.97f` multiplier not in config | simulation_physics_loop.c | the_list_2 |
| 2.8 | `nice_value` is a multiplicative velocity hack, not a force | rigidbody.c | the_list_2 |
| 2.9 | Velocity zeroing thresholds hardcoded | rigidbody.c | the_list_2 |
| 2.12 | No rolling friction (spheres roll forever) | collision_mechanics.c | the_list_2 |
| 2.13 | No CCD — fast objects tunnel through thin geometry | physics_world.c | the_list_2 |
| 2.14 | No solver islanding — all manifolds in one global loop | physics_world.c | the_list_2 |
| NEW-10 | Spring joint damping uses pre-force velocity (one step behind) | spring_joint.c | Physics audit |
| NEW-11 | Spring joint force clamp on net, not spring + damping independently | spring_joint.c | Physics audit |
| NEW-12 | Contact cache local-position matching fails for rotating bodies | collision_mechanics.c | Physics audit |
| NEW-14 | Inertia tensor rotated one tick stale during solving | rigidbody.c | Physics audit |

---

## Tier 4 — Scene & Persistence

| ID | Issue | File(s) | Source |
|---|---|---|---|
| 3.1 | Scene format is raw struct dump — no versioning, no checksum | scene_saving.c | the_list_2 |
| 3.2 | Object IDs reassigned on load | scene_load.c | the_list_2 |
| 3.3 | Spring joints not saved | scene_saving.c | the_list_2 |
| 3.4 | Sleep state not saved | scene_saving.c | the_list_2 |
| 3.5 | `nice_value` not saved | scene_saving.c | the_list_2 |
| list-20 | `nice_value` not restored on load | scene_load.c | the_list.txt |

---

## Tier 5 — Architecture (v15S → v16 prerequisites)

| ID | Issue | File(s) | Source |
|---|---|---|---|
| 1.1 | All scene state is file-scope globals | simulation.c, mpe_engine.h | the_list_2 |
| 1.2 | Two physics pipelines (legacy GUI + physics_world) | simulation_physics_loop.c, physics_world.c | the_list_2 |
| 1.3 | Sleep staticize hack mutates inverse_mass mid-solve | simulation_physics_loop.c | the_list_2 |
| 1.4 | Floor proxy is a static rigidbody, not a real body | collision_mechanics.c | the_list_2 |
| 1.5 | Boundary teleporter instead of real walls | boundary.c | the_list_2 |
| 1.6 | `mpe_engine.h` includes 25+ headers | mpe_engine.h | the_list_2 |
| 8.5 | `debug_terminal.c` is 146 KB god file | debug_terminal.c | the_list_2 |
| 8.5 | `collision_mechanics.c` is 59 KB god file | collision_mechanics.c | the_list_2 |

---

## Resolved by Fix Campaigns 155–176 ✅

| Original ID | Issue | Fixed by |
|---|---|---|
| list-1 | NULL deref in `ftc_robot_create_with_drive` | 161 |
| list-2 | Duplicate `g_key_pressed` clear | 161 |
| list-3 | `q_key_pressed` missing from `initialize_input` | 161 |
| list-4 | Hardcoded 0.8f friction in traction | 162 |
| list-5 | `wheel_radians[4]` vs `FTC_MAX_WHEELS = 8` | 163 |
| list-6 | No cylinder-vs-object narrowphase | 172–175 |
| list-7 | `gui_robot_apply_drive` always calls mecanum | 164 |
| list-8 | Rolling resistance outside `#if MPE_DRIVETRAIN_REAL` | 162 |
| list-9 | Dead `mecanum_active` field | 162 |
| list-10 | Cylinder init uses sphere restitution | 165 |
| list-11 | Double `rigidbody_sanitize` in scene_load | 166 |
| list-12 | `add_joint` return unchecked in editor | 166 |
| list-13 | `rb_get_kinetic_energy` inverse-of-inverse | 166 |
| list-14 | Odometry strafe sign convention | 171 |
| list-15 | Wheel lock threshold hardcoded | 166 |
| list-16 | `gamepad_poll` called with no robot | 161 |
| list-18 | No tank turn test | 167 |
| list-19 | No odometry accuracy test | 167/170/171 |
| — | Gamepad primary accessor | 155 |
| — | Gamepad init/close lifecycle | 156 |
| — | Gamepad poll + analog drive | 157 |
| — | Y-axis double inversion | 158/158a |
| — | F310 gamepad-only drive (GVBNCH removed) | 159 |
| — | Strafe direction fix | 160 |
| — | Cylinder restitution config | 165 |
| — | Cylinder integrity (sanitize + dedupe) | r3/001 |
| — | FTC field builder | 176 |
| — | Robot config struct (parameterised dimensions) | 175 |
| — | Parallel-axle cylinder-cylinder normal | 175a |
| — | Windows port (11 changes) | manual + scripts |

---

## Counts

| Tier | Open |
|---|---|
| Tier 0 — Teacher handoff | 7 |
| Tier 1 — Training quality | 12 |
| Tier 2 — Brick-robustness | 15 |
| Tier 3 — Solver correctness | 16 |
| Tier 4 — Scene persistence | 6 |
| Tier 5 — Architecture | 8 |
| **Total open** | **64** |
| Resolved (campaigns 155–176 + Windows port) | **30** |

---

## Priority Order

1. **Tier 0** — validate the Windows build. Teacher is waiting.
2. **Tier 1, items NEW-01, NEW-05, NEW-04, NEW-07** — these four produce
   visibly wrong driving that grade 9s will learn as "normal."
3. **Tier 2, items R3-02, R3-06, R3-07** — data loss and physics lockup.
4. **Tier 2, item R3-01** — determinism before MFS autonomous tuning.
5. Everything else is v15S/v16 work.
