Yes. I went substantially deeper this time.

I treated **v10R3I as the actual product**, not as an MPE physics branch, and audited the codebase specifically for anything that can cause:

* training sessions to fail,
* driver input to become inconsistent,
* state to be lost or corrupted,
* the simulator to behave differently after reset/reload/restart,
* Windows builds/packages to stop working,
* UI/debug tools to interfere with training,
* resources or world state to become stale,
* numerical failures to escape containment,
* regressions to go unnoticed,
* or future development to silently reintroduce any of the above.

I also cross-checked the current code against the existing `scope.md`, execution plan, build system, validation scripts, release documentation, and the actual source tree.

**There is considerably more here than the 64 issues currently recorded in `scope.md`.**

Some of the existing 64 are legitimately MPE work and should stay out of v10S. But there is a second class of problems that the existing audit doesn't adequately capture: **state-machine, lifecycle, tooling, UI, persistence, and MFS↔MFS-world integration failures.**

Those are the ones I'm concentrating on.

---

# 1. The most important finding

The biggest problem with v10R3I right now isn't one individual bug.

It's this:

> **The simulator has several independent pieces of state that do not share a single lifecycle.**

You have:

```text
GTK application
    │
    ├── legacy scene
    │      ├── obj_per_scene
    │      ├── object_count
    │      ├── joints
    │      └── legacy physics
    │
    ├── FTC robot world
    │      ├── physics_world
    │      ├── robot bodies
    │      ├── revolute constraints
    │      └── robot state
    │
    ├── GUI robot proxies
    │      └── stored as legacy scene objects
    │
    ├── controller state
    │
    ├── camera/input state
    │
    ├── config state
    │
    ├── persistence state
    │
    ├── debug terminal state
    │
    └── rendering state
```

And operations such as:

```text
scene_clear()
scene_load()
scene_init_default()
reboot
reset
config reset
robot spawn
robot reload
```

do **not** necessarily reset all of those subsystems together.

That is exactly the kind of thing that produces the nightmare bugs you're talking about:

> "It worked before I reloaded the scene."

> "The robot was fine until I rebooted."

> "It only happens after spawning the second robot."

> "The controller works if plugged in before launch but not after."

> "The simulator says it saved, but it didn't."

> "This scene works until I load another scene."

These are much more dangerous to an FTC training simulator than an imperfect collision equation.

---

# 2. P0 — Things I would consider unacceptable in v10S

## P0-1 — Robot state is not part of the scene lifecycle

### `robotics/gui_robot_registry.c`

You have:

```c
ftc_robot mfs_gui_robots[MFS_MAX_GUI_ROBOTS];
int mfs_gui_robot_count = 0;
physics_world *mfs_gui_robot_world = NULL;
gui_robot_proxy mfs_gui_proxies[MFS_MAX_GUI_ROBOTS];
```

There is no corresponding:

```text
gui_robot_clear()
gui_robot_remove()
gui_robot_reset()
gui_robot_world_cleanup()
```

The scene system can do:

```c
scene_clear();
scene_load();
scene_init_default();
```

while the robot world continues existing.

That means **the GUI scene and robot simulation can become logically unrelated**.

This is much worse than a physics error.

---

# 3. The robot proxies can become attached to the wrong scene objects

This is one of the nastiest things I found.

Robot proxies are stored as **indices into `obj_per_scene`**:

```c
proxy->chassis_proxy
proxy->nose_proxy
proxy->wheel_proxies[i]
```

Then every tick:

```c
if ((proxy->chassis_proxy >= 0) &&
    (proxy->chassis_proxy < object_count))
```

and the proxy object is overwritten with robot state.

Now imagine:

```text
spawn robot
    ↓
proxy indices = 1..6
    ↓
scene_load()
    ↓
old scene destroyed
    ↓
new scene gets objects 0..N
```

The robot registry **still contains indices 1..6**.

So the robot can subsequently overwrite whatever objects happen to occupy those indices.

That's not merely "robot disappeared."

It can become:

> robot proxy A is now visually hijacking unrelated objects from the newly loaded scene.

And because the proxy is actually a rigidbody in the legacy scene, those objects aren't merely render handles.

---

# 4. Robot proxies are actual physics objects in the other world

This deserves special attention.

`gui_robot_spawn()` does:

```c
scene_add_cube(..., 0.0f);
scene_add_object(..., 0.0f);
```

for the robot's visual representation.

Those objects are therefore part of:

```text
obj_per_scene
```

and the legacy physics system.

Meanwhile the **real robot** exists in:

```text
physics_world
```

So you have:

```text
Legacy world
    └── static robot proxy

Robot world
    └── actual robot
```

Those worlds don't collide with each other.

That means a game object simulated by the legacy world can collide with the **static proxy**, but it cannot collide with the actual robot.

And the proxy is:

* a cube for the chassis,
* a sphere for each wheel,
* a sphere for the nose.

Therefore you can get:

```text
game piece
     ↓
collides with robot proxy
     ↓
legacy physics responds
```

while the actual robot body isn't involved in that collision at all.

This produces **two different physical robots occupying the same visual space**.

For pure driver training this may be tolerable if game pieces are intentionally irrelevant.

But if MFS is ever used to practice interaction with field elements, this is a **training-integrity bug**, not merely a physics-truth issue.

---

# 5. Robot creation isn't transactional

## `robot.c`

`ftc_robot_create_with_drive()` progressively creates:

```text
chassis
wheel 0
joint 0
wheel 1
joint 1
wheel 2
joint 2
wheel 3
joint 3
```

But if anything fails halfway:

```c
if (robot->wheel_bodies[i] < 0) {
    return 1;
}
```

or:

```c
if (robot->wheel_joints[i] < 0) {
    return 1;
}
```

it simply returns.

There is **no rollback**.

So a failed robot creation can leave:

```text
chassis
wheel 0
wheel 1
...
```

inside the physics world.

And potentially constraints too.

The caller sees:

```text
spawn failed
```

but the world has been mutated.

That is a classic transactional-resource bug.

### v10S requirement

Robot creation must be:

```text
BEGIN
    allocate bodies
    allocate constraints
    allocate proxies
    initialize motors
    initialize metadata
COMMIT

or

ROLLBACK EVERYTHING
```

---

# 6. The constraint pool is global

## `physics/constraint.c`

This is one of the biggest architectural reliability hazards:

```c
static constraint constraint_pool[mpe_max_joints];
static int constraint_count = 0;
```

There is no world ownership.

A constraint contains:

```c
body_id_a
body_id_b
```

and then:

```c
constraint_dispatch(...)
```

searches the supplied body's ID list.

This means the constraints are **implicitly associated with a world by matching IDs**, not by actually belonging to a world.

That's fragile enough already.

Now combine it with:

```c
physics_world_init()
```

resetting:

```c
next_object_id = 1;
```

and you've got a particularly dangerous failure mode.

---

# 7. `physics_world_init()` leaks and resets IDs incorrectly

## `core/physics_world.c`

You currently do:

```c
memset(world, 0, sizeof(physics_world));
```

before attempting to preserve/free existing resources.

So if:

```c
physics_world_init(&world);
```

is called twice:

```text
old bodies pointer
old cache pointer
       ↓
memset
       ↓
pointers lost
       ↓
malloc new buffers
```

The old allocations are leaked.

Worse, `next_object_id` is also zeroed and therefore restarted.

So a world that was:

```text
ID 1
ID 2
ID 3
...
```

can become:

```text
ID 1
ID 2
ID 3
```

again after reinitialization.

Combine that with the global constraint pool and stale constraints become capable of matching newly created bodies.

**That is a real catastrophic-state possibility.**

---

# 8. `physics_world_cleanup()` doesn't clean constraints

You have:

```c
physics_world_cleanup()
```

for bodies/cache.

But constraints live elsewhere and aren't associated with the world.

So:

```text
destroy world
    ↓
bodies gone
    ↓
constraints remain
```

The next world can inherit stale constraints.

This absolutely needs to be fixed before v10S.

Not because revolute joints need to become physically perfect.

Because **destroying a robot must destroy its joints.**

---

# 9. No proper robot reset/removal mechanism

The registry only has:

```c
gui_robot_spawn()
gui_robot_tick()
gui_robot_apply_drive()
gui_robot_get_count()
gui_robot_get()
```

No removal.

That means the following is effectively permanent:

```text
touch robot
touch robot
touch robot
touch robot
```

until:

```c
MFS_MAX_GUI_ROBOTS
```

is reached.

And `mfs_gui_robot_count` never decreases.

For a training application, you want explicit lifecycle commands such as:

```text
robot spawn
robot reset
robot delete
robot list
robot respawn
```

even if the underlying physics remains ugly.

---

# 10. Controller hot-plug is genuinely incomplete

The current documentation openly says:

> controller must be plugged in before launch.

That's not something I'd accept in v10S.

The gamepad code detects a disconnect, but the architecture doesn't provide a robust reconnect path.

The current flow is effectively:

```text
startup
    ↓
open controller
    ↓
poll
    ↓
disconnect
    ↓
connected = false
    ↓
done
```

There is no robust:

```text
disconnect
    ↓
periodic device rediscovery
    ↓
reopen
    ↓
reset axes/buttons
    ↓
resume
```

For classroom hardware, this matters enormously.

Someone will:

* unplug USB,
* move the laptop,
* Windows reconnect the controller,
* accidentally bump the cable,
* change USB ports.

The simulator should recover.

---

# 11. Controller state should be zeroed on disconnect

Even though the drive path currently checks `connected`, the underlying `gamepad_state` retains old axis/button values.

That's dangerous API design.

A disconnected controller should have:

```text
axes = 0
buttons = 0
connected = false
```

as one atomic logical state.

That guarantees future code can't accidentally use stale input.

---

# 12. The `E` key is effectively broken

## `ui_input/input_control.c`

Key press:

```c
if (event->keyval == GDK_KEY_e)
    input_state->e_key_pressed = true;
```

But `on_key_released()` does **not** clear `e_key_pressed`.

It is only cleared during focus loss.

So:

```text
press E
    ↓
e_key_pressed = true
    ↓
simulation tick
    ↓
object menu toggles
    ↓
e_key_pressed remains true
    ↓
next tick
    ↓
object menu toggles again
    ↓
...
```

This is exactly the sort of small UI state-machine bug that can make the simulator feel haunted.

It needs to be fixed immediately.

---

# 13. The T key doesn't actually do what the documentation says

The current input system has:

```c
t_key_pressed
```

but terminal opening is wired to:

```text
1
```

in debug mode.

The documentation repeatedly says:

```text
T → terminal
```

while the implementation uses:

```text
1 → terminal
```

The T key state is effectively dead.

That's a **training/onboarding failure**, not documentation trivia.

If the intended workflow is:

```text
0
T
touch robot
```

then **T must actually open the terminal.**

---

# 14. Debug/test keys are not sufficiently isolated

F5–F11 perform destructive operations:

```text
F5 stability
F6 sleep/wake
F7 editor torture
F8 spawn stress
F9 report
F10 long run
F11 config torture
```

But they're not consistently restricted to an explicit diagnostic mode.

A student accidentally hitting F8/F10 can change the scene.

That is unacceptable for a classroom simulator.

I would make:

```text
Debug Mode
    ↓
diagnostic keys enabled
```

and:

```text
Game Mode
    ↓
F5-F11 inert
```

Even better, the final training build should distinguish:

```text
TRAINING
DEBUG
```

rather than letting diagnostic machinery sit one accidental keystroke away.

---

# 15. Key repeat isn't handled as an input edge

Several actions are effectively:

```text
keydown → bool=true
tick → consume bool → false
```

That doesn't guarantee a single logical press.

GTK key auto-repeat can generate multiple keypress events.

This is particularly dangerous for:

* `0` debug toggle,
* `6` config menu,
* `7`/`8` menus,
* destructive diagnostic keys,
* scene operations,
* object operations.

You need an explicit distinction between:

```text
held
pressed_this_frame
released_this_frame
```

or a key-down edge tracker.

---

# 16. `scene_load()` is still one of the largest data-loss hazards

## `scene/scene_load.c`

This happens:

```c
scene_clear();
```

before the entire file has been validated.

Therefore:

```text
load corrupted file
       ↓
current scene destroyed
       ↓
file parsing fails
       ↓
partial/new scene remains
```

This is exactly the kind of bug that makes users afraid to use persistence.

The correct architecture is:

```text
open file
    ↓
validate header
    ↓
validate counts
    ↓
read entire scene into temporary state
    ↓
validate every object
    ↓
validate every joint
    ↓
validate EOF/format
    ↓
COMMIT replacement
```

Never mutate the live scene until the candidate scene has passed validation.

---

# 17. Truncated scenes can return success

This is even worse.

The object loop uses:

```c
if (!read_float(...))
    break;
```

Then:

```c
object_count = loaded_count;
```

and eventually:

```c
return 1;
```

So a truncated scene can result in:

```text
half a scene loaded
```

while the caller receives:

```text
SUCCESS
```

Likewise, a partially read joint list can terminate early and still result in success.

This needs to become:

```text
read failure → transaction failure → old scene preserved
```

---

# 18. Invalid object types silently become spheres

You have:

```c
if (temp.type == object_cube) {
    ...
} else {
    rigidbody_initialisation_sphere(...)
}
```

So:

```text
type = 999
```

becomes:

```text
sphere
```

That is dangerous corruption handling.

Unknown enum values should be:

```text
invalid scene
```

not:

```text
guess what the user meant
```

---

# 19. Cylinder serialization is still broken

Current save stores:

```text
type
mass
radius
half_extensions
...
```

but doesn't store:

```text
cylinder_half_length
is_mecanum
roller_angle
```

And loading uses:

```c
if (temp.type == object_cube)
    initialize_cube(...)
else
    initialize_sphere(...)
```

Therefore cylinders become spheres.

This is already in the scope document, but I'm elevating it because persistence is a **user-visible state integrity problem**, not an MPE physics problem.

---

# 20. Robot state isn't saved at all

Scene saving operates on:

```text
obj_per_scene
```

The actual robot lives in:

```text
physics_world
```

Therefore:

```text
save scene
close
reopen
load scene
```

does not restore:

* robot body state,
* wheel state,
* motor state,
* battery,
* odometry,
* revolute constraints,
* robot registry.

The documentation admits this, but for v10S you need to make a deliberate decision:

### Either

**A.** robot state is explicitly *not persistent* and the UI must make that clear,

or

**B.** the training scene format includes robot state.

What you cannot have is an operation called "save scene" that appears to save the training state while silently excluding the actual robot.

For training I'd probably choose **A for v10S** unless persistence of robot sessions is actually needed.

But then loading a scene must explicitly reset/clear the robot world.

---

# 21. Scene save isn't atomic

## `scene_saving.c`

Current:

```c
fopen(path, "wb");
```

then directly write the destination.

If anything happens during the write:

```text
crash
disk full
power loss
process killed
```

the existing file can become:

```text
empty
partial
corrupt
```

Same problem exists in config saving.

Correct pattern:

```text
scene.dat.tmp
    ↓
write
    ↓
flush
    ↓
close
    ↓
rename(scene.dat.tmp, scene.dat)
```

On Windows, the replacement semantics need to be handled explicitly.

---

# 22. `fwrite()` results aren't checked

The scene writer's helpers:

```c
write_float()
write_int()
write_vec3()
write_vec4()
```

ignore `fwrite()`'s return value.

Therefore:

```text
disk write fails
```

doesn't necessarily propagate to:

```text
save_scene() == failure
```

The API can claim success after an incomplete write.

This is exactly the opposite of what persistence should do.

---

# 23. Scene save has a joint-hole bug

This one is subtle.

You do:

```c
for (int j = 0; j < current_joint_count; j++)
```

and only inspect active entries.

But joint removal apparently creates holes while decrementing the count.

Example:

```text
active:
0
1
2
3
4
5
6
7
8
9
```

remove 9:

fine.

But remove 3:

```text
active:
0
1
2
4
5
6
7
8
9
```

and:

```text
current_joint_count = 9
```

The save loop only checks:

```text
0..8
```

so joint `9` is never serialized.

This means scene persistence can silently lose the final surviving joint depending on removal history.

That's a classic sparse-pool iteration bug.

---

# 24. Config saving has the same atomicity problem

## `config/mpe_config.c`

```c
FILE *file = fopen(path, "w");
```

immediately truncates the existing configuration.

Then hundreds of writes occur.

There is no:

* temporary file,
* complete-write verification,
* fsync,
* atomic replacement.

So a shutdown/write failure can destroy the user's configuration.

---

# 25. Config parser silently accepts malformed data

The code intentionally ignores:

```text
unknown keys
missing '='
bad numeric values
bad sections
```

That's reasonable for forward compatibility **only if malformed recognized settings are distinguishable from unknown settings**.

Currently:

```text
gravity = 9.81garbage
```

can parse as `9.81`.

Because `strtod()` is used but the end pointer isn't checked for trailing garbage.

So a damaged line can silently become a valid-but-wrong configuration.

v10S should require:

```text
number
optional whitespace
end-of-line
```

for recognized values.

---

# 26. Configuration corruption isn't transactional

Suppose:

```text
gravity = bad
drag = 0.99
friction = 0.2
...
```

The loader modifies settings as it goes.

If later validation fails, some settings have already changed.

The same transaction principle applies:

```text
read file
    ↓
parse into temporary config
    ↓
validate whole config
    ↓
commit
```

This matters because configuration can materially alter the training environment.

---

# 27. Long-run validation does not validate the robot

`long_run_validation.c` iterates:

```c
obj_per_scene
```

It does **not** validate:

```text
mfs_gui_robot_world
```

So the current "long run" test can pass while the FTC robot:

* NaNs,
* flies away,
* loses constraints,
* gets stuck,
* loses input,
* accumulates impossible battery state,
* or becomes disconnected from its proxies.

For v10S this needs to become a **robot soak test**, separate from the MPE physics truth suite.

---

# 28. Existing NaN protection is incomplete

`rigidbody_sanitize()` checks:

* position,
* velocity,
* angular velocity,
* orientation,
* mass,
* dimensions,
* friction,
* restitution,
* axes,
* inertia.

But it does not comprehensively validate:

```text
force_accumulator
torque_accumulator
acceleration
angular_acceleration
object_type
object_id
object_generation
sleep_timer
mecanum metadata
roller angle
```

A NaN force accumulator is especially dangerous because it can poison integration before the next sanitizer gets a chance to repair it.

For v10S, the sanitizer should become a **state validator**, not merely a partial emergency repair mechanism.

---

# 29. Sanitization repairs things silently

This is useful for an emergency release build, but it can also hide bugs.

For example:

```text
bad state
    ↓
sanitize
    ↓
state silently repaired
    ↓
training continues
```

Now you don't know that a bug occurred.

v10S needs two modes:

```text
RELEASE MODE
    defensive repair + log

VALIDATION MODE
    fail loudly + identify first bad field
```

Otherwise future regressions can keep getting masked.

---

# 30. Finite-but-insane values aren't rejected

A loaded scene can contain:

```text
mass = 1e30
radius = 1e30
half_extent = 1e30
friction = 1e30
```

Those are technically finite.

`rigidbody_sanitize()` largely accepts them.

Then inertia calculations can overflow to infinity.

That can lead to:

```text
inverse inertia = 0
```

and eventually bad physics.

Scene validation needs **semantic bounds**, not merely `isfinite()`.

---

# 31. Object IDs can wrap

Both:

```c
scene_allocate_object_id()
```

and:

```c
world->next_object_id++
```

are ultimately finite-width counters.

There is no real wrap policy.

Eventually:

```text
UINT32_MAX → 0
```

and ID zero is already treated as special by constraints.

You don't need to wait until billions of objects to solve this. v10S should define:

```text
ID exhaustion
```

as a controlled failure.

---

# 32. Robot/world object IDs aren't globally coordinated

You have:

```text
legacy scene object IDs
robot physics world IDs
```

and both worlds start around low IDs.

Because the constraint system is global, this is another reason the current global constraint architecture is dangerous.

Even if the robot world is supposed to be isolated, the constraint pool doesn't know that.

---

# 33. Broadphase overflow is silent

The broadphase already tracks overflow counters.

That's good.

But from a training-integrity standpoint:

```text
broadphase full
    ↓
some objects aren't represented
    ↓
collision isn't generated
    ↓
simulation continues
```

That's dangerous.

For a diagnostic build:

```text
overflow = failure
```

For normal training:

```text
overflow = logged + visible warning
```

At minimum.

Silently producing incomplete collision is exactly the sort of "weird thing happened once" bug you want eliminated.

---

# 34. Large-object clamping has the same issue

The broadphase deliberately clamps objects that span too many cells.

Again:

```text
large object
    ↓
clamp
    ↓
collision coverage changes
```

That's acceptable as an engine limitation.

But it needs to be observable.

Otherwise the driver sees a strange collision and has no explanation.

---

# 35. `physics_world` has static scratch buffers

You have:

```c
static broadphase_pair world_pairs[...];
static collision_data world_manifolds[...];
```

inside `physics_world.c`.

Therefore multiple worlds are not truly independent from an execution-state perspective.

The current tests have a two-world test, but that's not enough.

You need to test:

```text
world A step
world B step
world A step
world B step
```

and:

```text
world A contains robot
world B contains robot
```

with randomized IDs and simultaneous lifetime.

This is especially important because MFS has made `physics_world` a real runtime subsystem rather than a test-only abstraction.

---

# 36. The current `physics_world` and legacy physics aren't equivalent

The two pipelines differ in things such as:

* depenetration,
* sleeping,
* constraint handling,
* solver behavior,
* global/static state,
* boundary handling.

That isn't automatically bad.

But it means **you cannot assume a bug fixed in one pipeline is fixed in the other.**

For v10S you need separate regression contracts:

```text
legacy world contract
FTC world contract
```

rather than one generic "physics passed."

---

# 37. Robot containment is missing

`physics_world` has no equivalent of the legacy:

```c
boundary_apply_box(...)
```

So the robot can simply leave the useful simulation region.

At:

```text
3 m/s
```

a driver can move a significant distance during a long session.

Eventually:

```text
robot leaves field
```

and the training session becomes useless.

This isn't MPE physics truth.

It's **MFS training infrastructure**.

You need either:

* a field boundary,
* robot reset,
* out-of-bounds recovery,
* or all three.

I'd strongly prefer:

```text
out of bounds
    ↓
robot automatically reset to starting pose
    ↓
controller zeroed
    ↓
visible "ROBOT RESET" notification
```

rather than allowing it to vanish into infinity.

---

# 38. Robot reset must also reset controller/motor state

A reset should not merely teleport the chassis.

It must reset:

```text
chassis velocity
wheel velocity
angular velocity
motor current
motor torque
battery state?        ← policy decision
odometry
wheel encoders
constraints
input commands
accumulator
```

Otherwise:

```text
reset
    ↓
robot immediately shoots away again
```

because old motor commands or velocities survive.

---

# 39. GUI robot fixed timestep has a hidden state lifecycle issue

`gui_robot_tick()` uses:

```c
static float robot_accumulator = 0.0f;
```

That accumulator belongs to the entire process, not the robot/world.

If the robot world is reset/reinitialized, the accumulator remains.

This is small, but it's another example of state that isn't owned by the subsystem it belongs to.

Make it part of the robot simulation state.

---

# 40. Robot proxy synchronization is one-way and delayed

The sequence is:

```text
legacy physics
    ↓
robot input
    ↓
robot physics
    ↓
proxy synchronization
```

So the legacy world has already stepped before the robot's proxy gets updated.

If legacy physics interacts with the proxy, it can be one frame behind.

That is another reason the proxy-as-rigidbody approach should eventually disappear.

For v10S, if you keep it, it should be treated as:

```text
render-only representation
```

as much as possible.

---

# 41. Renderer has unchecked allocation failures

## `render/new_render.c`

These:

```c
sphere_instances = malloc(...);
cube_instances = malloc(...);
```

aren't checked.

If allocation fails:

```text
render_init_status = RENDER_OK
```

and later:

```c
target_array[idx] = ...
```

can dereference NULL.

Likewise:

* grid allocation,
* sphere mesh allocation,
* index buffers,
* mesh resources.

This is unlikely under normal classroom conditions, but v10S should not have a known crash-on-OOM path.

---

# 42. Mesh initialization is not transactional

`sphere_meshing.c` allocates multiple buffers:

```text
vertex_data
wireframe_indices
element_indices
```

without checking each allocation before writing.

For example:

```c
float *vertex_data = malloc(...);
```

is immediately followed by:

```c
vertex_data[...]
```

No NULL check.

Same for the index arrays.

This is a real memory-safety hole.

---

# 43. Shader file reading doesn't fail transactionally

`shader_loading.c` reports:

```text
Error reading vertex shader
```

but continues.

It then feeds potentially incomplete data to OpenGL.

Also:

```c
ftell()
```

isn't checked for `-1`.

So an unusual filesystem failure can turn into:

```text
negative size
```

then an invalid allocation.

---

# 44. Renderer assumes working directory

Shaders are opened as:

```text
render/shaders/vertex_shader.glsl
```

not relative to the executable.

So:

```text
double-click engine.exe
```

may work because Windows chooses the expected CWD.

But:

```text
launch from shortcut
launch from another directory
launch from script
launch from IDE
launch from PowerShell
```

can produce:

```text
red screen
```

This is absolutely a v10S issue.

The executable should resolve:

```text
executable_directory/render/shaders/...
```

or establish its resource root once at startup.

---

# 45. The Windows release directory in the repository does not contain `engine.exe`

I checked the actual archive contents.

`v10R3I/windows_release/` contains the DLLs and render resources, but no executable.

That may be intentional because the binary is gitignored/not committed.

But then the repository's "release package" is not itself reproducible.

v10S should have:

```text
build Windows executable
        ↓
stage release directory
        ↓
verify required files
        ↓
dependency scan
        ↓
zip
        ↓
clean-machine test
```

as an automated process.

---

# 46. Windows build instructions are stale/broken

`install/windows/windows_build_instructions.md` still says:

```text
cd ~/mpe/v15R3/src
```

and references:

```text
windows_port.py
fix_text_visibility.py
```

which aren't present where the document says they are.

That's not cosmetic.

A teammate trying to rebuild v10S from source can literally follow the instructions and fail.

---

# 47. `docs.py` is itself stale

`v10R3I/docs.py` is still fundamentally a **v15R3 documentation generator**.

It generates paths such as:

```text
v15R3/how_to_use.md
v15R3/RELEASE_GATES.md
```

and contains huge numbers of old version assumptions.

This is a particularly dangerous future-regression mechanism.

Someone eventually runs the documentation generator and can overwrite current v10S documentation with v15R3-era content.

That needs to be fixed or removed.

---

# 48. Validation scripts target nonexistent trees

The following still target:

```text
v15R2/src
v15R3/src
```

instead of:

```text
v10R3I/src
```

This affects:

* `tools/build_check.py`
* `tools/project_audit.py`
* `tools/test_runner.py`
* `run_all.sh`
* `verify.sh`
* `validation/V01.sh`
* `validation/V02.sh`
* `validation/V03.py`
* `validation/V04.sh`
* `tools/refactor.py`
* migration tooling

This is one of the most important **development-regression hazards**.

A test suite that points at the wrong source tree isn't merely stale documentation.

It gives you false confidence.

---

# 49. The Makefile has an actual syntax defect

Current structure:

```make
ifeq ($(OS),Windows_NT)
...
else
...
endif -lxinput9_1_0
```

The `-lxinput9_1_0` ends up after the `endif`.

That's wrong.

The correct conceptual structure is:

```make
ifeq ($(OS),Windows_NT)
LIBS = ... -lxinput9_1_0
else
LIBS = ...
endif
```

You said you already have a working build, and I believe you.

But this is still a **source-level reproducibility defect**.

---

# 50. The build system has no actual platform matrix

The same source tree is intended to support:

```text
Linux
Windows MinGW
```

but v10S needs explicit validation of:

```text
Linux GCC
Windows MinGW64 GCC
ASan/UBSan Linux
ASan/UBSan-compatible Windows strategy
Release optimization
Debug optimization
```

The current build system has no authoritative matrix saying:

```text
this source tree + this compiler + this flags = supported
```

That becomes important as MFS continues developing while MPE changes independently.

---

# 51. The tests are not integrated as a regression gate

There are many tests, which is good.

But the current situation is closer to:

```text
lots of individual tests
```

than:

```text
one authoritative v10S gate
```

You need a top-level invariant:

```text
v10S candidate
    ↓
build
    ↓
unit tests
    ↓
robot tests
    ↓
persistence tests
    ↓
state-transition tests
    ↓
soak
    ↓
package
    ↓
Windows smoke test
```

with **one nonzero failure result**.

---

# 52. `math3_inverse_test` isn't a strong gate

We already found that the inverse diagnostic has branches that effectively report failure without necessarily making the test fail in the strongest possible way.

For v10S, every test must have:

```text
PASS → exit 0
FAIL → exit != 0
```

No diagnostic test should be allowed to "look failed" while CI continues.

---

# 53. Physics truth tolerances are irrelevant to v10S — but numerical stability isn't

This is an important distinction.

Things like:

```text
free-fall error < 0.5 m
bounce error < 30%
```

aren't what I care about here.

But these are:

```text
NaN
Inf
memory corruption
exploding velocity
invalid quaternion
invalid inertia
invalid body count
constraint references nonexistent body
broadphase overflow
```

Those are **v10S correctness requirements**, even if MPE later replaces the underlying physics.

---

# 54. There is no first-fault numerical containment

Suppose body 27 becomes:

```text
NaN
```

The sanitizer may repair some fields.

But the system doesn't appear to establish a strong concept of:

```text
FIRST BAD STATE
```

You want a debug facility that records:

```text
frame
body ID
world
field
old value
new value
source subsystem
```

For example:

```text
NUMERICAL FAULT
world=FTC
body=17
field=angular_velocity.y
value=NaN
frame=18342
source=constraint_solve
```

That turns future debugging from archaeology into engineering.

---

# 55. No robot-specific invariant monitor

v10S should continuously be capable of asserting:

```text
robot.chassis_body valid
every wheel body valid
every wheel joint valid
wheel count sane
all motor states finite
battery finite
battery 0..1
odometry finite
wheel encoders finite
body IDs unique
constraints reference existing bodies
```

This is currently missing.

---

# 56. Motor state isn't comprehensively validated

The motor system assumes things such as:

```text
resistance > 0
stall_current >= 0
gear_ratio > 0
efficiency in (0,1]
```

because presets provide sane values.

But v10S needs a motor invariant function anyway.

A bad motor should not be able to turn:

```text
NaN → battery → torque → robot
```

into a whole-world failure.

---

# 57. Battery state needs invariant checking

Battery currently clamps:

```text
charge_fraction >= 0
terminal_voltage >= 0
```

but there isn't a complete invariant framework around:

```text
nominal_voltage
internal_resistance
capacity
charge_fraction
current draw
```

For a training simulator, the battery model doesn't have to be perfect.

It **must never become invalid.**

---

# 58. Input → motor command needs an explicit neutral guarantee

The desired invariant should be:

```text
controller disconnected
    ↓
all commands = 0
```

and:

```text
robot reset
    ↓
all commands = 0
```

and:

```text
window focus lost
    ↓
all commands = 0
```

The current controller path mostly achieves the first one indirectly, but it should be an explicit subsystem contract rather than an accidental consequence of `gamepad_is_connected()`.

---

# 59. Focus-loss handling is good, but incomplete as a general input architecture

You already clear a lot of state in:

```c
on_focus_out()
```

which is good.

But this is compensating for the fact that the input model doesn't distinguish:

```text
physical key state
logical held state
one-shot event
```

v10S should formalize that.

Otherwise every new keybinding risks introducing another stuck state.

The `E` bug is proof that the current approach is vulnerable.

---

# 60. Mouse grab/reacquire isn't fully error checked

Calls such as:

```c
gdk_seat_grab(...)
```

don't appear to have their result treated as authoritative.

So the application can set:

```text
is_mouse_locked = true
```

even when the actual pointer grab failed.

That creates:

```text
internal state says locked
OS says unlocked
```

which then causes bizarre camera behavior.

The state should be updated from the actual result.

---

# 61. Render cleanup doesn't actually delete GL resources

`render_cleanup()` frees CPU instance arrays, but doesn't delete:

* shader programs,
* VAOs,
* VBOs,
* mesh resources.

At normal process exit Windows will reclaim the context/resources.

But the function is named:

```text
render_cleanup
```

and `render_init_status` can be reset to uninitialized.

That means a future attempt to reinitialize rendering in the same process can leak or reuse stale GL handles.

For v10S:

```text
render_init()
render_cleanup()
render_init()
```

should be a valid lifecycle test.

---

# 62. Shader/program initialization is not rollback-safe

You create:

```text
instanced shader
utility shader
```

and if the second fails, the first isn't necessarily cleaned up before entering failed state.

Same general problem:

> initialization needs to be transactional.

This should be applied consistently to:

* renderer,
* robot,
* physics world,
* scene,
* controller,
* config.

---

# 63. Scene state and selection state are separate

The selection system has:

```text
selected_object
selected_object_id
```

and the scene clear/reset operations don't appear to establish one canonical invalid state for every selection field.

This is dangerous because indices are ephemeral.

The rule should be:

```text
scene mutation
    ↓
invalidate selection
    ↓
re-resolve only by ID if appropriate
```

not:

```text
old index happens to still point somewhere.
```

---

# 64. `marked_joint_object_index` is index-based

This is another example.

You use:

```c
marked_joint_object_index
```

rather than a stable object ID.

But object deletion changes indices.

So:

```text
mark object 5
delete object 2
```

can cause the previously marked object to become object 4 while the marker still says 5.

That's an editor-state consistency bug.

For v10S, every persistent selection/reference should use:

```text
object ID
```

and resolve to an index at the moment it's needed.

---

# 65. Debug terminal has dangerous file capabilities

The terminal/editor machinery can operate on project files.

The existing scope correctly identifies:

```text
tee / MicroVim can clobber source
```

but the problem is broader.

For a training build, debug tools should have an explicit sandbox:

```text
status/
scenes/
config/
logs/
```

rather than being able to wander around the project tree.

This isn't primarily a security concern.

It's:

> **preventing a classroom user from accidentally destroying the simulator itself.**

---

# 66. MicroVim has destructive failure modes

We found several.

### Failed open can destroy current buffer

The editor's load flow can reset the editing state before successfully loading the new file.

### Save isn't atomic

Same problem as scenes/config.

### Save failure can be treated like success

This is particularly dangerous for:

```text
:wq
:x
```

because the user can leave the editor believing their file was saved when the write failed.

### Long lines are constrained by fixed line storage

A source/config file with a line exceeding the editor's supported length can be rewritten incorrectly.

For a debug terminal this might sound minor.

But it's precisely the sort of tool that can corrupt the codebase while you're debugging the simulator.

---

# 67. Debug terminal documentation is stale too

For example the terminal help says things like:

```text
Drive with WASD...
```

while the current system is deliberately gamepad-only.

There are also old v15R2/v15R3 descriptions embedded in terminal help.

That creates conflicting instructions depending on whether someone reads:

```text
README
terminal
Windows guide
how_to_use
overlay
```

v10S needs **one source of truth for controls.**

---

# 68. Version identity is internally inconsistent

Current tree:

```text
v10R3I
```

but the engine says:

```c
#define a3_version_string "v15R3"
```

and the overlay says:

```text
Miniature Physics Engine v15R2
```

while documentation alternates between:

```text
v10R3I
v15R2
v15R3
MFS-W
```

That's going to make bug reports horrible.

Imagine someone reports:

> "I'm running v15R2 and the robot disappears after reboot."

You don't even know what executable they actually have.

v10S needs a single canonical version identity exposed everywhere.

---

# 69. The release gate documents don't represent the actual product

The current `RELEASE_GATES.md` is largely inherited from the MPE/v15 lineage.

For v10S, I would replace that with gates built around:

```text
TRAINING
RELIABILITY
PERSISTENCE
INPUT
WINDOWS
PACKAGING
REGRESSION
```

and explicitly mark:

```text
physics-truth issues deferred to MPE
```

This will stop the project from accidentally drifting back toward "fix everything in MFS."

---

# 70. `run_all.sh` is particularly dangerous because it can create false confidence

It expects:

```text
v15R3/src
```

which doesn't exist.

So the developer can run the supposed master validation script and get:

```text
cannot find source
```

rather than actually testing v10R3I.

That needs to become impossible.

---

# 71. Path handling isn't centralized

You have hardcoded:

```text
status/
render/
render/shaders/
```

all over the code.

That creates different behavior depending on:

```text
CWD
installation location
Windows path
Linux path
debug launch
Explorer launch
terminal launch
```

v10S needs one:

```text
mfs_paths
```

or equivalent runtime path resolver.

Then:

```text
resource_root
config_root
scene_root
log_root
shader_root
```

all derive from it.

---

# 72. `status/` creation is scattered

`simulation.c` does:

```c
mkdir("status", ...)
```

during the tick.

Config has its own directory creation.

Scene saving doesn't have the same guarantee.

This should be centralized:

```text
mfs_runtime_init()
    ↓
resolve paths
    ↓
create directories
    ↓
validate writable
```

before the engine enters the training state.

---

# 73. Startup should verify its resource environment before becoming interactive

Currently the application can get far enough into startup before discovering:

```text
shader missing
DLL missing
config broken
controller absent
```

Some of these should be nonfatal.

But the simulator should explicitly classify them:

```text
FATAL
    renderer cannot initialize

RECOVERABLE
    controller missing

WARNING
    config corrupt → defaults loaded

INFO
    no saved scene
```

That gives you deterministic startup behavior.

---

# 74. There is no persistent diagnostic log for classroom failures

You have an in-memory event log.

That's good.

But if the simulator crashes, the event log disappears.

For v10S I would persist a small:

```text
status/mfs.log
```

with:

```text
version
startup
platform
renderer initialization
controller connect/disconnect
scene load/save
robot spawn/reset
numerical fault
resource failure
shutdown
```

This will make field debugging dramatically easier.

---

# 75. Event log itself isn't thread-safe

Currently it is single-threaded, so this isn't immediately dangerous.

But v10S should make that an explicit assumption.

If future Windows controller/resource code becomes asynchronous, the static event ring isn't protected.

I'd either:

```text
declare single-threaded permanently
```

or make it safe before adding async subsystems.

---

# 76. The test suite doesn't exercise failure paths enough

Most tests are:

```text
construct valid world
run valid simulation
measure result
```

You need a second category:

```text
failure injection
```

Examples:

```text
invalid scene
truncated scene
bad object type
duplicate ID
missing shader
unwritable config
controller disconnect
controller reconnect
robot allocation failure
constraint allocation failure
scene capacity exhaustion
broadphase overflow
render allocation failure
world reinitialization
world cleanup
reboot after robot spawn
load after robot spawn
save after robot spawn
```

That's where most of the v10S bugs are going to live.

---

# 77. The existing test architecture doesn't test GUI lifecycle

This is a huge gap.

You can have:

```text
headless physics = 100% pass
```

while:

```text
actual MFS = broken
```

because GUI-specific state is responsible for:

* robot registry,
* proxies,
* controller,
* input,
* scene menus,
* rendering,
* GTK lifecycle.

You need a **headless application-state integration layer** even if GTK itself isn't tested headlessly.

---

# 78. Need a state-transition test matrix

This is what I think is missing most from the existing testing strategy.

You need to test transitions, not just features.

For example:

| From         | Operation             | Expected                        |
| ------------ | --------------------- | ------------------------------- |
| Fresh        | Spawn robot           | valid robot                     |
| Robot        | Clear scene           | robot policy explicitly applied |
| Robot        | Reboot                | no stale robot                  |
| Robot        | Load scene            | robot policy explicitly applied |
| Robot        | Save scene            | documented behavior             |
| Robot        | Controller disconnect | neutral                         |
| Disconnected | Reconnect             | driving resumes                 |
| Robot        | Reset                 | zero velocity/input             |
| Robot        | Spawn second          | independent robot               |
| Robot        | Failed spawn          | no partial bodies               |
| World        | Cleanup               | no stale constraints            |
| World        | Reinit                | no leaks/ID collision           |
| Config       | Corrupt load          | old config preserved            |
| Config       | Save failure          | old config preserved            |

That matrix is much more valuable for v10S than another 500-line collision test.

---

# 79. One particularly important future-regression problem: the architecture is easy to accidentally break

The code currently mixes:

```text
legacy MPE assumptions
MFS additions
MFS Windows port
historical v15R2 patches
historical v15R3 patches
```

with comments such as:

```text
MPE_TASK_...
MFS_...
A3_PATCH_...
MPE_FTC_...
FIX ...
```

This is understandable given the project's history.

But it means a future change can accidentally modify an old subsystem without realizing it's still active.

The v10S hardening pass should establish explicit subsystem ownership.

Not necessarily modularize everything.

Just establish:

```text
THIS function owns this state.
THIS function initializes it.
THIS function resets it.
THIS function destroys it.
```

---

# 80. Things I explicitly do **not** want to drag into v10S

This is just as important.

I would **not** turn v10S into MPE v16.

I would leave these alone unless they cause actual training failures:

### Deferred physics truth

* exact cylinder-floor manifold
* exact cylinder-cylinder collision
* 2D friction cone
* exponential-map quaternion integration
* CCD
* solver islanding
* perfect revolute Jacobian solver
* physically exact mecanum contact
* elimination of artificial lateral damping
* elimination of the 3 m/s safety cap
* removal of all heuristic damping
* exact rolling friction
* perfect warm-start tangent matching
* elimination of every legacy physics hack

Those belong in **MPE**.

But:

```text
NaN containment
state corruption
stale world references
robot reset
controller recovery
persistence
build
Windows
resource paths
```

belong in **MFS**.

---

# 81. What I think the v10S progression should actually look like

I would now split the work into **seven hardening passes**.

## Pass 1 — Kill lifecycle corruption

Highest priority.

Fix:

```text
physics_world_init()
physics_world_cleanup()
constraint ownership
robot lifecycle
robot rollback
robot reset
robot registry reset
proxy lifecycle
scene lifecycle
selection lifecycle
```

Goal:

> No operation can leave behind a subsystem that the operation logically destroyed.

---

## Pass 2 — Kill state/persistence corruption

Fix:

```text
scene transactional load
scene transactional save
config transactional load
config transactional save
cylinder serialization
joint-hole serialization
invalid scene rejection
duplicate ID rejection
semantic bounds
```

Goal:

> A failed operation never destroys previously valid state.

---

## Pass 3 — Kill input/UI inconsistencies

Fix:

```text
E key
T key
key repeat
edge/held state model
controller reconnect
controller state zeroing
focus loss
mouse grab result
debug-only test keys
neutral command guarantee
```

Goal:

> Input always has one unambiguous meaning.

---

## Pass 4 — Kill resource/platform inconsistencies

Fix:

```text
resource root
shader paths
status paths
Windows package generation
DLL dependency verification
startup resource validation
render allocation failures
GL cleanup
Linux/Windows path abstraction
```

Goal:

> Same executable behavior regardless of launch directory or machine setup.

---

## Pass 5 — Kill development-system lies

Fix every stale:

```text
v15R2
v15R3
v15R3/src
v15R2/src
windows_port.py
fix_text_visibility.py
```

reference that isn't intentionally historical.

Then make:

```text
build_check
project_audit
test_runner
run_all
verify
validation scripts
docs generator
```

all target v10S correctly.

Goal:

> The project's own tooling must be trustworthy.

---

## Pass 6 — Add the MFS training invariant suite

This is the new major testing layer.

Something like:

```text
mfs_training_test
```

with:

### Startup

```text
launch
renderer
resource paths
config
```

### Robot

```text
spawn
drive
strafe
rotate
combined motion
stop
reset
```

### Controller

```text
disconnect
neutral
reconnect
resume
```

### Scene

```text
save
load
clear
reboot
```

### Stress

```text
10 min
30 min
60 min
```

### State

```text
no NaN
no Inf
no invalid IDs
no orphan constraints
no stale proxies
no invalid motors
no invalid battery
no invalid odometry
```

---

# 82. Pass 7 — Windows release certification

This becomes a hard release gate.

### Build

```text
clean MSYS2 MINGW64
make clean
make
```

### Test

```text
all headless tests
```

### GUI

```text
launch from terminal
launch from Explorer
launch from different CWD
```

### FTC

```text
F310 X mode
forward
reverse
strafe
rotate
combined
stop
disconnect
reconnect
```

### Persistence

```text
config save
config reload
scene save
scene reload
```

### Soak

```text
30–60 minutes
```

### Packaging

```text
fresh directory
only packaged files
no MSYS2
no development tree
```

### Final

```text
delete development environment
run package
```

If that passes, **then** v10S is actually ready.

---

# 83. The new v10S bug taxonomy I recommend

I would stop treating the existing `scope.md` categories as the complete definition.

Use:

### T0 — Training blockers

Anything that can prevent an FTC user from training.

```text
crash
freeze
controller failure
robot unavailable
robot disappears
broken startup
broken package
broken reset
```

### T1 — State integrity

```text
save/load corruption
stale world state
orphan constraints
wrong object references
configuration corruption
```

### T2 — Input/UI integrity

```text
stuck keys
repeat actions
wrong mappings
focus bugs
mouse lock bugs
debug commands leaking into training
```

### T3 — Platform/release integrity

```text
Windows build
DLLs
paths
CWD
packaging
resource lookup
```

### T4 — Numerical containment

```text
NaN
Inf
overflow
invalid IDs
invalid body state
```

### T5 — Regression infrastructure

```text
stale tests
stale scripts
false-positive tests
missing failure-path tests
missing GUI integration tests
```

### T6 — Physics truth

Explicitly deferred to MPE.

---

# 84. And the really important end-state

I would not define v10S as:

> "all known bugs fixed."

That's impossible, and it would pull MFS back toward MPE.

I'd define it as:

> **Every subsystem has an explicit owner, initialization path, reset path, failure path, and validation path. A failed operation cannot corrupt previously valid state. A disconnected input device cannot produce motion. A malformed file cannot destroy a valid scene. A missing resource cannot crash the simulator. A robot cannot survive in a logically destroyed world. And every training-critical invariant is automatically tested.**

Then MPE can go absolutely feral on the physics afterward. 😂

---

# The current priority ranking

If I were actually taking the repository and starting v10S **today**, my order would be:

| Priority | Work                                  |
| -------- | ------------------------------------- |
| 🔴 P0    | Robot/world/constraint lifecycle      |
| 🔴 P0    | Scene transactional load              |
| 🔴 P0    | Scene/config atomic writes            |
| 🔴 P0    | Controller reconnect + neutral state  |
| 🔴 P0    | Input state bugs (`E`, `T`, repeat)   |
| 🔴 P0    | Robot proxy lifecycle                 |
| 🔴 P0    | Robot creation rollback               |
| 🔴 P0    | Windows/tooling path consistency      |
| 🔴 P0    | Resource-root/CWD independence        |
| 🟠 P1    | Robot invariant monitor               |
| 🟠 P1    | Failure-injection tests               |
| 🟠 P1    | GUI state-transition tests            |
| 🟠 P1    | Persistence round-trip suite          |
| 🟠 P1    | 30–60 min robot soak                  |
| 🟠 P1    | Renderer allocation/cleanup hardening |
| 🟠 P1    | Debug terminal sandboxing             |
| 🟠 P1    | Version/documentation unification     |
| 🟡 P2    | Broadphase overflow policy            |
| 🟡 P2    | Numerical first-fault diagnostics     |
| 🟡 P2    | Full resource/error logging           |
| 🟢 P3    | Deferred architecture cleanup         |
| 🟢 P3    | MPE physics-truth work                |

And **only after that** would I call the build `v10S`.

The really interesting part is that this audit gives us a much better target than the existing `execution_plan.md`: **the existing plan has identified a lot of the physics debt, but it hasn't fully mapped the state-machine/lifecycle surface that MFS itself now has.**

That missing surface is where I think the next generation of bugs would otherwise come from.

