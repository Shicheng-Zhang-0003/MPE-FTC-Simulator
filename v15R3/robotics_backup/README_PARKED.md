# MFS robotics — parked history (NOT the live code)

This directory is a pre-transplant snapshot, kept for history and for the
unwired diagnostic tests. It is NOT part of any build (no makefile or
`tools/test_runner.py` target references it).

- `robotics/` — battery, motor, motor presets, drivetrain, robot, GUI registry
  (pre-transplant copies: 12V/9.2A presets, NiMH model, and geometry all
  DIVERGED from live — do not copy back without diffing)
- `gamepad/` — evdev-only F310 snapshot (live `src/ui_input/gamepad.c` adds
  Win32 XInput + reconnect)
- `tests_robotics/` — 12 unwired diags (teleop, mecanum, integration/debug,
  tank turn, odometry (+diags), idle-spin diags, physics truth (+diag))

The LIVE robotics code is `v15R3/src/robotics/` (linked into the engine via
`ENGINE_SRCS`, the `mpe-tui` FTC scene, and 7 `build_ftc_*` test targets).
MPE↔FTC coupling (wheel-lock loop, mecanum roller tangent, `is_mecanum` /
`roller_angle_rad` / `driven_this_tick`, gamepad dispatch, `touch robot`)
is live in `src/`, not removed.

Drive-tuning notes for the MFS return: slop-gated zero-depth contacts
(`penetration_slop`) restored persistent wheel friction (teleop 0.92 m,
strafe +X 0.49 m in 3 s); per-iteration revolute axis-drift correction holds
wheels upright; commanding must wake the chassis (velocity integrator drains
sleeping-body forces).
