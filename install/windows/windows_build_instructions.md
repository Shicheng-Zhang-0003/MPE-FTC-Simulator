# Windows Build Instructions (MSYS2 / MinGW-w64)

Native Win64 build of MFS. No WSL, no Wine, no Visual Studio.

---

## 1. Install MSYS2

Download the installer from https://www.msys2.org and run it. Accept defaults
(installs to `C:\msys64`).

## 2. Install the toolchain

Open the **MINGW64** terminal (Start menu → "MSYS2 MINGW64").
Title bar must say `MINGW64` — not MSYS, not UCRT64, not CLANG64.

```bash
pacman -Syu
pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-gtk3 \
          mingw-w64-x86_64-libepoxy mingw-w64-x86_64-pkg-config \
          make python3
```

Verify:

```bash
gcc --version
pkg-config --cflags gtk+-3.0 epoxy    # should print -I flags
```

## 3. Get the source

**Recommended:** work inside the MSYS2 filesystem, not `/c/...` — the 9P
bridge to Windows drives is slow and makes compiles crawl.

```bash
# copy the extracted tree into your MSYS2 home
cp -r /c/Users/<you>/Downloads/Miniature_Physics_Engine-* ~/mpe
cd ~/mpe/v15R3/src
```

(Or `git clone` directly into `~/`.)

## 4. Apply the platform port

Two idempotent scripts, both create `.pre_winport` backups:

```bash
python3 windows_port.py          # fixes 1-9 (ifdefs, gamepad, makefile)
python3 fix_text_visibility.py   # fixes 10-11 (HUD + terminal text)
```

Preview first with `--dry-run` if desired. Re-running is safe (prints `[SKIP]`).

## 5. Build

```bash
make clean
make
```

Expect two harmless notes on first build if you skipped the scripts' latest
revision — see Troubleshooting below for the classic errors.

## 6. Run from the terminal

```bash
./engine.exe
```

## 7. Package for distribution

The exe needs GTK3's DLLs, the shaders, and a status folder beside it:

```bash
mkdir -p ../windows_release
cp engine.exe ../windows_release/
cp -r render ../windows_release/            # GLSL shaders (relative paths!)
mkdir -p ../windows_release/status          # engine.cfg / scene.dat

for dll in $(ldd engine.exe | grep mingw64 | awk '{print $3}'); do
    cp "$dll" ../windows_release/
done

ls ../windows_release | wc -l               # expect ~40-60 files
```

Nuclear option if a DLL is still missing:

```bash
cp /mingw64/bin/*.dll ../windows_release/
```

Zip `windows_release/` (Explorer → right-click → Compress to ZIP). Test the
zip on a machine **without** MSYS2 before handing it out.

---

## Test checklist (post-build)

| # | Test | Expected |
|---|------|----------|
| 1 | Double-click `engine.exe` from Explorer | Window opens, grid + HUD visible |
| 2 | HUD text | Near-white with dark halo, readable over bright scenes |
| 3 | Press `0`, then `T` | Terminal opens; plain output is light grey |
| 4 | `touch robot` | Robot spawns (blue chassis, 4 dark wheels, orange nose) |
| 5 | F310 in **X** mode, plugged in before launch | Left stick up = forward |
| 6 | Left stick sideways | Strafe (mecanum) |
| 7 | Right stick sideways | Rotate |
| 8 | Release sticks | Robot coasts to a stop and holds |
| 9 | Mouse look | Smooth — no violent single-frame snaps |
| 10 | Close window | Clean exit, `status/engine.cfg` written |

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `error: too many arguments to function 'mkdir'` | POSIX 2-arg mkdir on Windows | Run `windows_port.py` (fixes 3–4); or wrap with `#ifdef _WIN32 _mkdir(path) #endif` |
| `'XINPUT_GAMEPAD_GUIDE' undeclared` | MinGW xinput.h lacks the Guide constant | Set that button to `false` (windows_port.py handles it) |
| `warning: ignoring '#pragma comment'` | MSVC-only pragma | Harmless; remove the line (script does) |
| `undefined reference to XInputGetState` | XInput not linked | makefile needs `-lxinput9_1_0` (script fix 9); try `-lxinput` if not found |
| **Red screen** on launch | Shaders not found | `render/` folder must sit beside `engine.exe`; working directory matters |
| "DLL not found" from Explorer | GTK runtime missing | Bundle DLLs (step 7); nuclear option `cp /mingw64/bin/*.dll` |
| Drive forwards/backwards inverted | XInput Y polarity | `invert_left_y = false` on Windows (script fix 2) |
| Camera snaps violently | Win32 warp not atomic | ±80 px delta clamp (script fix 8) |
| All text black/unreadable | Windows GTK light theme | Run `fix_text_visibility.py` |
| Gamepad not detected | D mode / hot-plug | F310 switch → **X**; plug in before launch; check Windows "Game Controllers" |
| SmartScreen blocks exe | Unsigned binary | More info → Run anyway |
| Build works but slow | Source on `/c/` (9P bridge) | Move tree into `~/` inside MSYS2 |

## Rebuilding after code changes

```bash
cd ~/mpe/v15R3/src
make                        # incremental
cp engine.exe ../windows_release/   # DLLs unchanged, exe only
```
