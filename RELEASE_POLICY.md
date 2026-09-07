# MFS-W Release Policy (v15R3 Windows Build)

This tree is in **v15R3 / MFS-W development** — the native Windows port of the
merged MPE+MFS tree.

## Cycle Goal

Deliver a Windows-native build that a classroom can run by double-clicking,
without touching a terminal, while keeping the Linux build path byte-identical.

This cycle covers:

1. Native Win64 build via MSYS2 / MinGW-w64 (no WSL, no Wine).
2. XInput gamepad support alongside the existing Linux evdev path.
3. Win32 presentation fixes (text visibility, mouse-warp stability).
4. Portable distribution packaging (DLL + shader bundling).
5. Documentation regenerated to match the dual-platform reality.

## Change Classes Accepted

During MFS-W development:

1. **Platform port code** — strictly behind `#ifdef _WIN32` / `#ifdef __linux__`,
   tagged `WIN_PORT`. Linux behaviour must not change.
2. Correctness fixes required by the port (build system, POSIX shims).
3. Packaging and distribution tooling (`windows_port.py`,
   `fix_text_visibility.py`, `generate_docs.py`, bundling scripts).
4. Documentation updates matching the new architecture.
5. Validation additions for the Windows platform (Gate 15).
6. Carried-over FTC robotics and cylinder-physics work from v15R3.

## Explicitly Rejected During This Cycle

- Any `#ifdef`-free platform assumption leaking into shared code paths.
- New physics features that have not passed the Linux headless suite first.
- Full global-state removal, multithreading, CCD, scene format v2, GTK4 —
  all remain deferred to v15S/v16.

## Platform Port Rules

1. Every platform difference gets a `WIN_PORT` marker comment.
2. The same source tree must build on both platforms without edits.
3. Fix scripts must be idempotent and create `.pre_winport` backups.
4. A Windows-only fix may never change a Linux code path; verify with a
   Linux build after every port script run.

## Release Goal

MFS-W may be tagged when:

- All P0 gates pass, **including Gate 15 (Windows Platform)** with the 15e
  verification debt cleared,
- The distribution zip runs on a clean Windows machine with nothing installed,
- The config system round-trips on Windows (save → restart → load),
- Physics behaviour at defaults matches the Linux build,
- Documentation (README, how_to_use, gates, policy, release notes, platform
  port reference) matches the shipped code.
