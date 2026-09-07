# Windows Validation Checklist (Gate 15e)

Run every item below on the Windows build before tagging MFS-W stable.
Record results in `release_notes_v15R3W.md`. Date each run.

---

## A. In-engine validation suite (F5–F11)

Launch `engine.exe`, then:

| Key | Test | Pass criterion | Result |
|-----|------|----------------|--------|
| F5 | Stability stack | 10 cubes settle, no jitter/explosion | [ ] |
| F6 | Sleep/wake | Projectile wakes sleeping cube | ] |
| F7 | Editor torture | No crash; survivor remains | [ ] |
| F8 | Spawn stress | 300 objects, usable framerate, no NaN | [ ] |
| F9 | Validation report | Prints to console with config dump | [ ] |
| F10 | Long-run (60 s) | Console prints `result: PASS` | [ ] |
| F11 | Config torture | Survives 60 s; restore with `config reset` | [ ] |

## B. Headless test suite under MinGW

In the MINGW64 terminal:

```bash
cd ~/mpe/v15R3/src
python3 ../tools/test_runner.py       # or: python3 tools/test_runner.py from root
```

All 14 tests must PASS (two_world, revolute, teleop_drive, mecanum_drive,
cylinder_drop, driven_wheel, math3_inverse, ftc_integration, physics_truth,
tank_turn, odometry_accuracy, cylinder_sphere, cylinder_cube,
cylinder_cylinder).

- [ ] Suite builds under MinGW (makefile test targets use `find`/gcc — verify)
- [ ] All 14 tests pass
- [ ] `physics_truth` assertions identical to Linux results

Note: if test targets fail to build on MinGW, record the exact error — do not
silently skip. Gate 15euires this green or an explicitly documented
limitation.

## C. Persistence round-trips

| Test | Procedure | Pass criterion | Result |
|------|-----------|----------------|--------|
| Config save/load | Change gravity via menu `6`, exit, relaunch | New gravity active; `[config] loaded` on console | [ ] |
| Config corrupt-file | Truncate `status/engine.cfg` mid-line, relaunch | Engine starts on defaults, no crash | [ ] |
| Scene save | Spawn objects, press `9` → 1 | `status/scene.dat` written | [ ] |
| Sce load | `9` → 2 | Objects restored (positions/velocities/colours) | [ ] |
| Status dir creation | Delete `status/`, relaunch | Folder recreated, no crash | [ ] |

## D. Input soak

| Test | Procedure | Pass criterion | Result |
|------|-----------|----------------|--------|
| Gamepad cold-start | Plug F310 (X), launch | `[gamepad] XInput device 0 connected` on console | [ ] |
| Gamepad absent | Launch with no controller | Engine runs; `[gamepad] no XInput device found`; no crash | [ ] |
| Drive polarity Full stick each direction | Forward/up, strafe/left, rotate match stick | [ ] |
| Mouse soak | 2 min of fast mouse-look | No camera snaps, no drift-to-edge | [ ] |
| Focus loss | Alt-Tab mid-drive | Robot stops receiving input; no stuck keys | [ ] |

## E. Stability soak

| Test | Procedure | Pass criterion | Result |
|------|-----------|----------------|--------|
| 10-min idle | Spawn robot, leave it | No crash, no NaN, robot holds position | [ ] |
| 10-min drive | Continuous driving + strafing | No crash; battery sags; wheels stay attached | [ ] |
| Exit cleanliness | Close window during drive | Clean exit; `engine.cfg` saved | [ ] |

## F. Clean-machine test

1. Copy `windows_release.zip` to a Windows PC **without MSYS2**.
2. Extract, double-click `engine.exe`.
3. Run sections A (F5, F10 only), C, D.
- [ ] Passes with zero missing-DLL errors

---

## Recording results

Append a dated block to `release_notes_v15R3W.md`:

```
### Windows validation run — YYYY-MM-DD
A: F5-F11  → x/7 pass
B: headless → xss
C: persistence → x/5
D: input → x/5
E: soak → x/3
F: clean machine → pass/fail
Verdict: READY TO TAG / NOT READY (reasons)
```

Any FAIL blocks the tag. Fix, re-run the whole section, re-evaluate.
