/* MPE_FTC_074: Drivetrain implementation */
/* MPE_FTC_082 TEMPORARY — replace with anisotropic friction (MPE_FTC_095): Fixed syntax error (stray '}') + real mecanum chassis forces */
#include "drivetrain.h"
#include "../core/math3D.h"

void drivetrain_tank (ftc_robot *robot, float left_power, float right_power) {
    if (!robot) {return;}
    if (left_power > 1.0f) {left_power = 1.0f;}
    if (left_power < -1.0f) {left_power = -1.0f;}
    if (right_power > 1.0f) {right_power = 1.0f;}
    if (right_power < -1.0f) {right_power = -1.0f;}
    /* Wheel layout: [0]=front-left, [1]=front-right, [2]=back-left, [3]=back-right */
    float commands [FTC_MAX_WHEELS];
    for (int i = 0; i < robot->wheel_count; i++) {
        bool is_left = (i % 2 == 0);  /* 0,2 = left; 1,3 = right */
        commands [i] = is_left ? left_power : right_power;
    }
    ftc_robot_set_wheel_commands (robot, commands, robot->wheel_count);
    robot->mecanum_active = false; /* MPE_FTC_082 TEMPORARY — replace with anisotropic friction (MPE_FTC_095) */
}

/* MPE_FTC_075 + MPE_FTC_082: Mecanum drive with inverse kinematics
 *
 * Since the wheel model uses spheres (no natural rolling direction),
 * mecanum strafe cannot work through wheel friction alone. We set
 * per-wheel motor commands for forward drive (which the wheel_traction
 * raycast converts to forward force), AND we compute a direct chassis
 * force for the strafe/rotate components. drivetrain_update() applies
 * that chassis force after ftc_robot_update(). */
void drivetrain_mecanum (ftc_robot *robot, float forward, float strafe, float rotate) {
    if (!robot) {return;}
    /* Clamp inputs */
    if (forward > 1.0f) {forward = 1.0f;}
    if (forward < -1.0f) {forward = -1.0f;}
    if (strafe > 1.0f) {strafe = 1.0f;}
    if (strafe < -1.0f) {strafe = -1.0f;}
    if (rotate > 1.0f) {rotate = 1.0f;}
    if (rotate < -1.0f) {rotate = -1.0f;}

    /* Mecanum IK: per-wheel velocity targets
       Wheel layout: [0]=FL, [1]=FR, [2]=BL, [3]=BR
       FL: forward + strafe - rotate
       FR: forward - strafe + rotate
       BL: forward - strafe - rotate
       BR: forward + strafe + rotate */
    float wheel_targets [4];
    wheel_targets [0] = forward + strafe - rotate;
    wheel_targets [1] = forward - strafe + rotate;
    wheel_targets [2] = forward - strafe - rotate;
    wheel_targets [3] = forward + strafe + rotate;

    /* Normalize if any target exceeds 1.0 */
    float max_mag = 0.0f;
    for (int i = 0; i < 4; i++) {
        float mag = fabsf (wheel_targets [i]);
        if (mag > max_mag) {max_mag = mag;}
    }
    if (max_mag > 1.0f) {
        for (int i = 0; i < 4; i++) {wheel_targets [i] /= max_mag;}
    }

    /* Set motor commands (forward component uses wheel traction) */
    ftc_robot_set_wheel_commands (robot, wheel_targets, 4);

    /* MPE_FTC_082 TEMPORARY — replace with anisotropic friction (MPE_FTC_095): Compute direct chassis force for strafe + rotate.
     * Local space: X = lateral (strafe), Y = up, Z = forward.
     * Force scale: tuned so full input ≈ 80 N, enough to move an 8 kg
     * chassis at ~0.5 m/s² against friction. */
    const float force_scale = 80.0f;   /* N per unit input */
    const float torque_scale = 8.0f;   /* N·m per unit input */
    robot->mecanum_chassis_force = (vector3) {
        strafe * force_scale,
        0.0f,
        forward * force_scale * 0.5f   /* forward partly via wheels */
    };
    robot->mecanum_chassis_torque = rotate * torque_scale;
    robot->mecanum_active = true;
}

void drivetrain_update (physics_world *world, ftc_robot *robot, float dt) {
    if ((!world) || (!robot) || (dt <= 0.0f)) {return;}
    ftc_robot_update (world, robot, dt);

    /* MPE_FTC_082 TEMPORARY — replace with anisotropic friction (MPE_FTC_095): Apply mecanum chassis forces after motor update */
    if (robot->mecanum_active) {
        int idx = robot->chassis_body;
        if ((idx >= 0) && (idx < world->body_count)) {
            rigidbody *chassis = &world->bodies [idx];
            /* Transform local force to world space using chassis orientation */
            vector3 world_force = vector4_rotate_to_vector3 (
                chassis->orientation, robot->mecanum_chassis_force);
            rb_apply_forces_perfect (chassis, world_force);
            /* Yaw torque (around local Y axis) */
            vector3 local_torque = {0.0f, robot->mecanum_chassis_torque, 0.0f};
            vector3 world_torque = vector4_rotate_to_vector3 (
                chassis->orientation, local_torque);
            chassis->torque_accumulator = vector3_addition (
                chassis->torque_accumulator, world_torque);
            rigidbody_wake (chassis);
        }
        robot->mecanum_active = false;
    }
}
