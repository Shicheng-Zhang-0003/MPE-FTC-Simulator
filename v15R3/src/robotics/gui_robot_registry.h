/* FTC GUI robot registry (475 single-world: robots live in the primary
 * physics_world and render directly; no proxies). */
#ifndef gui_robot_registry_h
#define gui_robot_registry_h

#include "robot.h"
#include "drivetrain.h"
#include "../core/physics_world.h"

#define MFS_MAX_GUI_ROBOTS 4

/* Registry state (single primary world; bodies render directly). */
extern ftc_robot mfs_gui_robots[MFS_MAX_GUI_ROBOTS];
extern int mfs_gui_robot_count;
extern physics_world *mfs_gui_robot_world;

/* Spawn a robot into the primary world. Returns index or -1. */
int gui_robot_spawn(float x, float y, float z, motor_preset_id preset);

/* Per-tick: motor/drivetrain updates only (main loop steps physics). */
void gui_robot_tick(float dt);

/* Apply keyboard drive input to all registered robots. */
void gui_robot_apply_drive(float forward, float strafe, float rotate);

/* Query */
int gui_robot_get_count(void);
ftc_robot *gui_robot_get(int index);

#endif /* gui_robot_registry_h */
