# MFS v15R3 Release Notes

`v15R3` is the MFS (MPE FTC Simulator) release: domain-driven `core` /
`physics` / `render` / `scene` / `ui_input` / `robotics` modules, 80 runtime tunables with
schema clamp, warm-starting sequential-impulse solver (64 iterations,
4-point Sutherland–Hodgman manifolds), full constraint framework (revolute,
fixed, prismatic, distance, rope + spring joints), 3D spatial-hash
broadphase, POSIX debug terminal, F5–F11 validation matrix, and the `mpe-tui`
terminal debugger + snapshot suite.

Headless suite: **36/36 green** (`python3 tools/test_runner.py`) — 29
MPE/physics + `frustum` + 7 FTC robotics. Twin-world determinism holds
bitwise over 600 ticks (fixed 1/60 s step, fixed solver order, `-O3
-ffp-contract=off`, fixed-coefficient `det_math` transcendentals).

## Post-release fixes (applied after tag, within freeze rules)

- **Mecum sign fix** (`collision_mechanics.c:1281`): anisotropic roller frame
  cross-product order corrected from `cross(floor_normal, axle_proj)` to
  `cross(axle_proj, floor_normal)`. Forward/strafe/rotate now drive correctly
  through honest wheel torque; no cheat forces needed.
- **Revolute Baumgarte fix** (`revolute_joint.c`, `constraint.h`):
  Baumgarte point-to-point correction computed once/tick in `revolute_pre_step`,
  applied in velocity solve. Eliminates wheel-joint vibration ("invisible potmarks")
  and enables true rest (stack test, F10 long-run pass).
- **Wheel-lock threshold lowered** (`mpe_config_schema.c`):
  `wheel_lock_omega_thresh` 0.5 → 0.1 rad/s. Prevents fighting motor torque
  during low-speed driving and coasting.
- **Traction clamp aligned** (`mpe_config_schema.c`):
  `roller_friction_coeff` default 1.0 → 0.8 matches `floor_friction_s = 0.8`.
  Drivetrain and solver agree on max traction; no phantom wheelspin.
- **SDK-standard mecanum roller angles** (`robot.c`):
  FL +45°, FR -45°, BL -45°, BR +45° (was mirror-X). +Strafe drives +X
  (robot-left) with no `strafe = -strafe` input flip.

## Known limitations (P1, documented — not fixed in this release)

- **Wayland:** mouse locking needs X11 (`GDK_BACKEND=x11 ./engine`).
- **Joint creation UI + persistence scope:** scene v200 persists bodies plus
  spring and revolute joints; fixed/prismatic/distance/rope live in the
  headless suite and TUI demo only.
- **SIMD/multithreading:** solver stays scalar single-threaded by design
  (bit-determinism outranks single-digit-% CPU gains; `-march=native` is an
  opt-in `ENABLE_NATIVE=1` build). Physics steps on a worker thread with UI
  mutations serialized under a world mutex.
- **Capacity caps, counted not silent:** 16384 bodies / 1024 joints / 65536
  broadphase pairs / 8192 manifolds. Node/pair/manifold exhaustion drops
  (never corrupts) and is surfaced in the validation report and TUI dumps.
  Bodies larger than ~8 cells/axis at 1.0 m minimum cell size risk pair
  drops — keep giants subdivided.
- **Sleep trade-off:** no wake-on-separation by design (a deleted support
  leaves a sleeper floating until an explicit wake or a novelty/depth gate
  fires) — accepted for F10 long-run calm.
- **Float32 world:** playable volume ±250 m (~0.03 mm resolution at the
  corners, 300× below contact slop). No origin rebasing inside the box.
- **FTC layer:** ~6 kg competition plant (19.2:1, 96 mm wheels); odometry is
  wheel-encoder translation with IMU-model heading (translation slips
  honestly under wheelspin, heading tracks). Battery thermal is tracked
  but never derates output. No suspension model (spawn clearances only).
  Sticks are robot-relative with an orange nose marker; default camera
  starts behind the robot.
- **FTC odometry:** encoder odometry drift exceeds 20% under wheelspin (test `ftc_odometry` fails); chassis-velocity integration would be more accurate but hides slip.
- **FTC stress:** multi-robot stress test shows chassis lean under sustained lateral load and reduced strafe displacement (test `ftc_stress` fails); suspension model would help.

## Post-release development (past the tag, same freeze rules)

- **All-truth drivetrain mesh:** anisotropic roller contact (rubber grip +
  free roller axes) deleted the strafe/rotate chassis forces; traction at
  1× Coulomb on the wheels; traction/odometry constants single-sourced;
  SDK roller geometry verified by travel direction.
- **Windows (MSYS2 MINGW64) supported:** warning-free builds, 36/36-capable
  runner (`.exe` aware, retry on transient spawn blocks), MSVCRT-safe
  formatting, ncursesw-tolerant TUI build.