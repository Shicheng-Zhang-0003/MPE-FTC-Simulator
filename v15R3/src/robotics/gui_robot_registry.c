/* FTC GUI robot registry (475 transplant, single-world).
 * 475 renders physics_world bodies directly, so no obj_per_scene proxies:
 * spawn creates the robot in the primary world, tick only runs motors
 * (the main loop owns physics_world_step). Teleop stick shaping (8/s ramp)
 * lives here, not in the motor model, so autonomy/tests bypass it. */
#include "gui_robot_registry.h"
#include "../core/physics_world.h"
#include "../scene/boundary.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

ftc_robot mfs_gui_robots[MFS_MAX_GUI_ROBOTS];
int mfs_gui_robot_count = 0;
physics_world *mfs_gui_robot_world = NULL;

int gui_robot_spawn(float x, float y, float z, motor_preset_id preset) {
    if (mfs_gui_robot_count >= MFS_MAX_GUI_ROBOTS) return -1;
    if (!mfs_gui_robot_world) mfs_gui_robot_world = physics_world_get_primary();
    if (!mfs_gui_robot_world) return -1;
    if (!mfs_gui_robot_world->bodies) physics_world_init(mfs_gui_robot_world);
    ftc_robot *robot = &mfs_gui_robots[mfs_gui_robot_count];
    int rc = ftc_robot_create(mfs_gui_robot_world, robot, x, y, z, preset);
    if (rc != 0) return -1;
    return mfs_gui_robot_count++;
}

/* Per-frame: run motor/drivetrain updates only (no stepping, no proxies). */
void gui_robot_tick(float dt) {
    if (mfs_gui_robot_count <= 0 || !mfs_gui_robot_world) return;
    if (dt <= 0.0f) return;
    const float step = 1.0f / 60.0f;
    float acc = dt > step * 5.0f ? step * 5.0f : dt;
    static float carry = 0.0f;
    carry += acc;
    while (carry >= step) {
        for (int i = 0; i < mfs_gui_robot_count; i++)
            drivetrain_update(mfs_gui_robot_world, &mfs_gui_robots[i], step);
        carry -= step;
    }
}

static float mfs_smooth_fwd = 0.0f, mfs_smooth_strafe = 0.0f, mfs_smooth_rot = 0.0f;
static float mfs_shape_axis(float target, float cur, float dt) {
    float max_d = 8.0f * dt;
    float d = target - cur;
    if (d > max_d) d = max_d;
    else if (d < -max_d) d = -max_d;
    return cur + d;
}

void gui_robot_apply_drive(float forward, float strafe, float rotate) {
    if (mfs_gui_robot_count <= 0 || !mfs_gui_robot_world) return;
    const float shape_dt = 1.0f / 60.0f;
    mfs_smooth_fwd = mfs_shape_axis(forward, mfs_smooth_fwd, shape_dt);
    mfs_smooth_strafe = mfs_shape_axis(strafe, mfs_smooth_strafe, shape_dt);
    mfs_smooth_rot = mfs_shape_axis(rotate, mfs_smooth_rot, shape_dt);
    for (int i = 0; i < mfs_gui_robot_count; i++) {
        if (mfs_gui_robots[i].drivetrain_type == FTC_DRIVETRAIN_TANK) {
            /* TRUTH: normalize like mecanum (preserve turn authority at full
             * stick) instead of per-side clamping, which distorts turns. */
            float l = mfs_smooth_fwd + mfs_smooth_rot, r = mfs_smooth_fwd - mfs_smooth_rot;
            float m = (fabsf(l) > fabsf(r)) ? fabsf(l) : fabsf(r);
            if (m > 1.0f) {
                l /= m;
                r /= m;
            }
            drivetrain_tank(&mfs_gui_robots[i], l, r);
        } else
            drivetrain_mecanum(&mfs_gui_robots[i], mfs_smooth_fwd, mfs_smooth_strafe, mfs_smooth_rot);
    }
}

int gui_robot_get_count(void) { return mfs_gui_robot_count; }

ftc_robot *gui_robot_get(int index) {
    if (index < 0 || index >= mfs_gui_robot_count) return NULL;
    return &mfs_gui_robots[index];
}
