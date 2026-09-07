# MFS-W Release Gates (v15R3 Windows Build)

This document defines the exit criteria for tagging the MFS-W Windows release.
Gates 1–13 are inherited from v15R2 (verified on Linux). **Gate 15 is new and
Windows-specific.** Gate 14 (FTC robotics) carries over with gamepad-only drive.

---

## Gate Rules

- **P0** — mandatory. Any P0 failure blocks the tag.
- **P1** — strongly recommended; may be deferred only if documented as a known
  limitation that does not undermine stability.
- **P2/P3** — optional; record as post-stable work.

---

## Mandatory P0 Gates (inherited — verified on Linux, v15R2)

### 1. Release Freeze
- [X] Release policy present and acknowledged
- [X] Only correctness, stability, platform-port, validation, docs, hygiene changes accepted

### 2. Build (Linux)
- [X] `make clean && make` succeeds, zero errors
- [X] Warnings reviewed and understood

### 3. Startup
- [X] Correct version string printed
- [X] Config system initialises; shaders load; window/grid/overlay render

### 4. Shader / Render Failure Visibility
- [X] Compile/link/missing-file failures reported; no silent broken render state

### 5. Input and Lifecycle
- [X] Close quits; config saved on clean exit
- [X] Mouse lock acquire/release; focus loss clears stuck state
- [X] Config menu (6) does not interfere with menus 7/8/9

### 6. Editor Stability
- [X] Select/delete/jointed-delete/marked-delete do not crash
- [X] Save/load with menus open does not crash

### 7. Physics Stability
- [X] Objects rest without jitter; cubes stack; restitution/friction work
- [X] Sleep/wake correct; no NaNs after normal use or stress

### 8. Broadphase / Solver Visibility
- [X] Node/pair/manifold/dedupe overflow visible in overlay and F9

### 9. Validation Tests (Linux)
- [X] F5–F11 pass; engine idles minutes without explosion

### 10. Configuration System
- [X] 69 tunables editable via menu and terminal
- [X] Save/load round-trip; corrupt file does not crash; clamping works

### 11. Documentation
- [X] README, user guide, and gates match the code (regenerated 2026-09-07)

### 12. Repository Hygiene
- [X] Build artifacts untracked; `.gitignore` present

### 13. Sanitizer Validation (Linux)
- [X] ASan+UBSan build available; validation passes; no severe reports

---

## P0 Gate 14 — FTC Robotics (MFS)

- [X] `ftc_robot_create` spawns chassis + 4 cylinder wheels with revolute joints
- [X] Mecanum strafe via real anisotropic roller friction (no chassis cheat)
- [X] Tank drive via motor torque → wheel traction
- [X] Motor model: BackEMF, gear ratio, Kt/Kv correct
- [X] Battery voltage sag under multi-motor load
- [X] Headless tests pass on Linux (`python3 tools/test_runner.py`)
- [X] Robot visible in GUI via proxy sync
- [X] Robot drivable via **gamepad only** (MFS_159; keyboard drive removed)
- [X] Fixed-timestep accumulator (60 Hz deterministic)
- [X] Revolute axis drift corrected (Baumgarte)
- [ ] Scene save/load preserves robot assemblies *(deferred)*
- [ ] Sensors (encoders/IMU/distance) exposed to user *(deferred)*
- [ ] FTC HAL (HardwareMap, OpMode) *(deferred)*

---

## P0 Gate 15 — Windows Platform (MFS-W)  ← NEW

### 15a. Build
- [X] MSYS2 MINGW64 toolchain builds `engine.exe` with zero errors
      (gcc, gtk3, libepoxy, pkg-config; `-lxinput9_1_0` linked)
- [X] `windows_port.py` + `fix_text_visibility.py` apply cleanly and are idempotent
- [X] Linux build unaffected by all WIN_PORT guards (same tree builds both)

### 15b. Packaging
- [X] `engine.exe` runs from File Explorer with bundled DLLs (no MSYS2 needed)
- [X] Shaders load via relative `render/` folder (no red screen)
- [X] `status/` created/used for `engine.cfg` and `scene.dat`
- [ ] Release zip verified on a **clean machine** with no MSYS2 installed

### 15c. Input
- [X] XInput gamepad detected at startup (F310 in X mode)
- [X] Left stick: forward/back correct polarity (WIN_PORT invert flip)
- [X] Left stick: strafe direction correct
- [X] Right stick X: rotate correct
- [X] Mouse look stable — no warp-induced sensitivity spikes (±80 px clamp)
- [X] Mouse lock acquire/release works under Win32 GDK

### 15d. Presentation
- [X] HUD overlay text readable (per-widget CSS, USER+1 priority)
- [X] Debug terminal text readable, including untagged output (`term_normal`)
- [X] Menus, crosshair, and robot HUD render correctly

### 15e. Verification debt (must clear before tagging)
- [ ] F5–F11 run on Windows, results recorded
- [ ] Headless test suite runs under MinGW (`python3 tools/test_runner.py`)
- [ ] Scene save (`9`) / load round-trip on Windows paths
- [ ] Config persistence: change → exit → relaunch → value survives
- [ ] 10-minute idle soak on Windows (no crash, no NaN)

---

## Recommended P1 Gates

- [X] Saving/loading scenes works (Linux); failure reported to user
- [X] CPU usage drops when scene sleeps; stress scenes remain usable
- [ ] Windows: ASan-equivalent validation (MinGW sanitizer support is limited —
      document as known limitation if skipped)

---

## Release Decision

MFS-W may be tagged only when:

1. All P0 gates 1–15 pass (15e verification debt cleared),
2. P1 gates pass or are documented as known limitations,
3. The clean-machine zip test passes,
4. The repository tree is clean,
5. `release_notes_v15R3W.md` records honest results.

If any mandatory gate fails: fix → re-run validation → re-evaluate. Never tag
around a red gate.
