# MPE v15R3 Release Notes

`v15R3` is the v15 configuration-system release: domain-driven `core` /
`physics` / `render` / `scene` / `ui_input` modules, 78 runtime tunables with
schema clamp, warm-starting sequential-impulse solver (64 iterations,
4-point Sutherland–Hodgman manifolds), full constraint framework (revolute,
fixed, prismatic, distance, rope + spring joints), 3D spatial-hash
broadphase, POSIX debug terminal, F5–F11 validation matrix, and the `mpe-tui`
terminal debugger + snapshot suite.

Headless suite: **36/36 green** (`python3 tools/test_runner.py`) — 29
MPE/physics + `frustum` + 7 FTC robotics. Twin-world determinism holds
bitwise over 600 ticks (fixed 1/60 s step, fixed solver order, `-O3
-ffp-contract=off`, fixed-coefficient `det_math` transcendentals).

## Known limitations (P1, documented — not fixed in this release)

- **Wayland:** mouse locking needs X11 (`GDK_BACKEND=x11 ./engine`).
- **Joint creation UI + persistence scope:** scene v200 persists bodies plus
  spring and revolute joints; fixed/prismatic/distance/rope live in the
  headless suite and TUI demo only.
- **SIMD/multithreading:** solver is scalar single-threaded by design
  (bit-determinism outranks single-digit-% CPU gains; `-march=native` is an
  opt-in `ENABLE_NATIVE=1` build).
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
- **FTC layer:** odometry is encoder FK (`v=ω·r`, no-slip assumption) —
  full-power foam driving slips, so odometry leads ground truth under
  sustained full stick. Battery thermal is tracked but never derates output.
  No suspension model (spawn clearances only).
