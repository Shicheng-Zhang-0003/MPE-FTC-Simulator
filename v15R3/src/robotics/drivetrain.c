/* MPE_FTC_074: Drivetrain implementation (inverse kinematics + odometry).
 * Traction comes from real rolling contact via the solver's anisotropic
 * roller friction; no synthetic chassis forces anywhere in this file. */
#include "drivetrain.h"
#include "../core/math3d.h"
#include "../config/mpe_config.h"
#include <math.h>

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
    /* MFS_162_DEAD_FIELD: mecanum_active removed */
}

/* MPE_FTC_075: Mecanum drive inverse kinematics (per-wheel commands;
 * traction resolved by anisotropic roller contact in the solver). */
void drivetrain_mecanum (ftc_robot *robot, float forward, float strafe, float rotate) {
    if (!robot) {return;}
    if (forward > 1.0f) {forward = 1.0f;}
    if (forward < -1.0f) {forward = -1.0f;}
    if (strafe > 1.0f) {strafe = 1.0f;}
    if (strafe < -1.0f) {strafe = -1.0f;}
    if (rotate > 1.0f) {rotate = 1.0f;}
    if (rotate < -1.0f) {rotate = -1.0f;}

    /* Mecanum IK: [0]=FL [1]=FR [2]=BL [3]=BR
       FL: f+s-r / FR: f-s+r / BL: f-s-r / BR: f+s+r (+strafe=+X). */
    int n = robot->wheel_count < 4 ? robot->wheel_count : 4;
    float wheel_targets [FTC_MAX_WHEELS] = {0};
    /* PHYS-FIX: generic count; mecanum requires 4 wheels. */
    if (robot->wheel_count != 4) {
        /* Degrade to tank-ish: same command all wheels */
        for (int i = 0; i < robot->wheel_count; i++) wheel_targets[i] = forward;
    } else {
        wheel_targets [0] = forward + strafe - rotate;
        wheel_targets [1] = forward - strafe + rotate;
        wheel_targets [2] = forward - strafe - rotate;
        wheel_targets [3] = forward + strafe + rotate;
        float max_mag = 0.0f;
        for (int i = 0; i < 4; i++) {
            float mag = fabsf (wheel_targets [i]);
            if (mag > max_mag) {max_mag = mag;}
        }
        if (max_mag > 1.0f) {
            for (int i = 0; i < 4; i++) {wheel_targets [i] /= max_mag;}
        }
        n = 4;
    }
    ftc_robot_set_wheel_commands (robot, wheel_targets, n);
}

void drivetrain_update (physics_world *world, ftc_robot *robot, float dt) {
    if ((!world) || (!robot) || (dt <= 0.0f)) {return;}
    ftc_robot_update (world, robot, dt);

    /* TRUTH: exactly ONE rolling-resistance model exists: the solver's
     * contact-patch pass (collision_apply_rolling_resistance), which uses
     * true per-contact normal force. The second torque loop that lived here
     * (c_rr * assumed-Mg/4 load, blind to load transfer and airborne state
     * except a height gate) double-counted dissipation and is deleted.
     * Zero-power is BRAKE via BackEMF + gearbox Coulomb drag; a FLOAT
     * (coast) mode needs controller support (future). */

    /* Odometry: encoder-based, not ground truth. wheel_radians are
     * the encoder truth; pose comes from mecanum inverse kinematics rotated
     * by estimated heading (gyro yaw). Slip/collisions now show as drift,
     * as on a real bot. Ground-truth chassis_vel path deleted. */
{
    float w_rad[FTC_MAX_WHEELS] = {0};
    float w_omega[FTC_MAX_WHEELS] = {0};
    float wheel_r = 0.05f;
    for (int i = 0; i < robot->wheel_count && i < FTC_MAX_WHEELS; i++) {
        int wi = robot->wheel_bodies[i];
        if (wi >= 0 && wi < world->body_count) {
            rigidbody *rw = &world->bodies[wi];
            vector3 axle = rw->cached_axes[0];
            if (vector3_length_squared(axle) < 0.0001f)
                axle = vector4_rotate_to_vector3(rw->orientation, (vector3){1.0f, 0.0f, 0.0f});
            float omega = vector3_dot(rw->angular_velocity, axle);
            w_omega[i] = omega;
            robot->wheel_radians[i] += omega * dt;
            w_rad[i] = robot->wheel_radians[i];
            wheel_r = rw->radius;
        }
    }
    (void)w_rad;
    /* Gyro yaw (IMU proxy — defensible; translation is encoder-only). */
    if (robot->chassis_body >= 0 && robot->chassis_body < world->body_count) {
        vector3 up = {0.0f, 1.0f, 0.0f};
        float yaw_rate = vector3_dot(world->bodies[robot->chassis_body].angular_velocity, up);
        robot->odom_theta += yaw_rate * dt;
    }
    /* Mecanum inverse kinematics (matches drivetrain_mecanum forward map). */
    float vx_local = 0.0f, vz_local = 0.0f;
    if (robot->wheel_count == 4) {
        float w0 = w_omega[0]*wheel_r, w1 = w_omega[1]*wheel_r;
        float w2 = w_omega[2]*wheel_r, w3 = w_omega[3]*wheel_r;
        /* f=(sum)/4 -> vz, s=(FL-FR-BL+BR)/4 -> vx */
        vz_local = (w0 + w1 + w2 + w3) * 0.25f;
        vx_local = (w0 - w1 - w2 + w3) * 0.25f;
    } else if (robot->wheel_count > 0) {
        float sum = 0.0f;
        for (int i = 0; i < robot->wheel_count; i++) sum += w_omega[i]*wheel_r;
        vz_local = sum / (float)robot->wheel_count;
    }
    float c = cosf(robot->odom_theta), s = sinf(robot->odom_theta);
    robot->odom_x += (vx_local * c + vz_local * s) * dt;
    robot->odom_z += (-vx_local * s + vz_local * c) * dt;
}

}
