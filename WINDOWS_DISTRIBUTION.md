# MFS-W Classroom Distribution Guide

**For:** FTC club teachers and student mentors
**What you received:** a zip containing a self-contained physics/robot
simulator. No installation, no admin rights, no internet needed.

---

## What's in the zip

```
windows_release/
├── engine.exe        ← the simulator (double-click)
├── render/           ← shader files (do not delete)
├── status/           ← settings + saved scenes (auto-created if missing)
└── *.dll             ← ~50 runtime libraries (do not delete)
```

Extract the zip anywhere — Desktop, a USB stick, `C:\MPE\`. The whole folder
must stay together.

---

## Student quick-start (print this)

1. **Plug in the controller first.** Logitech F310: flip the switch on the
   back to **X**.
2. Double-click `engine.exe`.
   - If Windows SmartScreen appears: click **More info**, then **Run anyway**.
3. Press `0` (enters Debug Mode), then press `T` (opens the terminal).
4. Type `touch robot` and press Enter. A robot appears.
5. Drive:
   - **Left stick** — forward / back / strafe sideways
   - **Right stick (left/right)** — rotate
6. Close the terminal with `Esc`. Release the mouse with `Esc` too.
7. Close the window when done — settings save automatically.

---

## Classroom setup notes

- **One controller per machine**, plugged in **before** launching. Hot-plug
  is not detected until the simulator restarts.
- Any XInput controller works (Xbox 360/One/Series, F310 in X mode, most
  third-party pads in XInput mode).
- The robot only drives with the controller — this is deliberate. Students
  build the same stick habits they will use on the real robot.
- To reset everything to defaults: delete the `status/` folder and relaunch.

## Lesson ideas

| Exercise | How |
|---|---|
| Straight-line driving | Drive forward 3 s, note the HUD (battery V, RPM) |
| Mecanum strafe | Left stick purely sideways — watch the wheels |
| Precision parking | Spawn robot, drive to a grid line, stop within 10 cm |
| Spin control | Right stick taps — quarter turns |
| What the battery does | Watch voltage sag under full stick, recover on release |

Useful terminal commands (after `0`, `T`):

```
touch robot        spawn the robot
ps aux             list every physics object
help               full command list
```

## Troubleshooting (non-technical)

| Problem | Fix |
|---|---|
| "Windows protected your PC" | More info → Run anyway |
| Whole screen is red | The `render` folder is missing or moved — re-extract the zip |
| "DLL not found" error | Some `.dll` files were deleted — re-extract the zip |
| Robot won't move | Controller not plugged in before launch, or F310 is in D mode. Restart with controller in X mode |
| Robot drives backwards | Controller was plugged in after launch — restart |
| Text hard to read | Should not happen in this build; report it |
| Settings lost | Don't delete the `status` folder unless resetting on purpose |

## Support

This build is the MFS-W Windows port of the MPE v15R3 tree, generated
2026-09-07. Source, build instructions, and the full issue inventory live in
the project repository (`readme.md`, `install/windows/`, `the_list*.txt`).
