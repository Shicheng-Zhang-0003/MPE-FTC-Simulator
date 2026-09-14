/* MPE_FTC_074: Drivetrain implementation */
/* MPE_FTC_082 TEMPORARY — replace with anisotropic friction (MPE_FTC_095): Fixed syntax error (stray '}') + real mecanum chassis forces */
#include "drivetrain.h"
#include "../core/math3D.h"
#include "../config/mpe_config.h"

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
       BR: forward + strafe + rotate
       Convention (MFS_311_MECANUM_GRIP_FIX): positive strafe = +X world motion.
       The old code negated strafe here AND in simulation_input_dispatch — two
       stacked fixes that cancelled out and hid the real force-direction bug. */

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


    
}

void drivetrain_update (physics_world *world, ftc_robot *robot, float dt) {
    if ((!world) || (!robot) || (dt <= 0.0f)) {return;}
    ftc_robot_update (world, robot, dt);

/* MPE_DRIVETRAIN_REAL — FIX 117 (Path A / partial 095 keystone):
 * real traction physics. Motor torque is converted to a ground traction
 * force per wheel, clamped by friction (MFS_311 grip-direction fix).
 * No artificial lateral/yaw damping or velocity cap: corrected traction
 * keeps the robot at rest when uncommanded. Flip MPE_DRIVETRAIN_REAL to 0
 * to revert to motor-torque-only drive. */
#define MPE_DRIVETRAIN_REAL 1
#if MPE_DRIVETRAIN_REAL
    {
        float gravity_mag = 9.81f;
        if (g_cfg.world.gravity < 0.0f) { gravity_mag = -g_cfg.world.gravity; }

        /* Total robot mass -> per-wheel normal load */
        float total_mass = 0.0f;
        int chassis_ok = ((robot->chassis_body >= 0) &&
                          (robot->chassis_body < world->body_count));
        if (chassis_ok) { total_mass += world->bodies[robot->chassis_body].mass; }
        for (int i = 0; i < robot->wheel_count; i++) {
            int wi = robot->wheel_bodies[i];
            if ((wi >= 0) && (wi < world->body_count)) {
                total_mass += world->bodies[wi].mass;
            }
        }

        /* MFS_310_TRUE_TRACTION: no synthetic chassis-force injection.
         * Traction now comes from real rolling contact: ftc_robot_update spins
         * each wheel with motor torque, and the physics solver's anisotropic
         * roller friction (grip axis full mu, roller-free axis ~0) converts
         * wheel spin into ground reaction. Removing the old phantom force lets
         * the robot's speed be bounded by its own wheel free speed and its
         * pitch/yaw dynamics by real reaction moments at the contact points. */

        /* MFS_317_BRAKE_MODE: when the stick is released, all commands go to
         * zero and a real 30:1 FTC gearbox binds (it cannot be back-driven),
         * so the robot slides to a stop in ~0.1-0.2 s on the field tiles.
         * Apply a grip-limited horizontal deceleration opposing the chassis
         * velocity, capped at the one-step stop bound so it can never flip
         * direction within a step. This is what makes "release the stick"
         * feel planted instead of the old iceberg-coast. */
        if (chassis_ok) {
            int mfs_all_idle = 1;
            for (int i = 0; i < robot->wheel_count; i++) {
                if (fabsf(robot->wheel_motors[i].command) > 0.05f) { mfs_all_idle = 0; break; }
            }
            if (mfs_all_idle) {
                rigidbody *mfs_chassis = &world->bodies[robot->chassis_body];
                float mfs_hx = mfs_chassis->velocity.x;
                float mfs_hz = mfs_chassis->velocity.z;
                float mfs_hspeed = sqrtf(mfs_hx * mfs_hx + mfs_hz * mfs_hz);
                if (mfs_hspeed > 0.02f) {
                    float mfs_friction_k = g_cfg.world.floor_friction_k;
                    if (mfs_friction_k <= 0.0f) { mfs_friction_k = 0.8f; }
                    float mfs_n_total = total_mass * gravity_mag;
                    float mfs_f_brake = mfs_friction_k * mfs_n_total;
                    float mfs_f_one_step = (dt > 0.0f) ? (total_mass * mfs_hspeed / dt) : mfs_f_brake;
                    if (mfs_f_brake > mfs_f_one_step) { mfs_f_brake = mfs_f_one_step; }
                    if (mfs_f_brake > 0.0f) {
                        vector3 mfs_brake = vector3_scaling(
                            (vector3){-mfs_hx / mfs_hspeed, 0.0f, -mfs_hz / mfs_hspeed}, mfs_f_brake);
                        mfs_chassis->force_accumulator = vector3_addition(mfs_chassis->force_accumulator, mfs_brake);
                    }
                }
            }
        }
    }

/* MFS_132_ROLLING_RESISTANCE: apply small opposing torque to spinning
* wheels in contact with the floor. Simulates realistic coast-down.
* Only applies when motor command is near-zero (free-rolling). */
{
float c_rr = g_cfg.world.rolling_resistance_coeff; /* MFS_141: real config param, default 0.02 */
if ((c_rr > 0.0f) && (robot->wheel_count > 0)) {
float total_mass = 0.0f;
int chassis_ok = ((robot->chassis_body >= 0) &&
(robot->chassis_body < world->body_count));
if (chassis_ok) { total_mass += world->bodies[robot->chassis_body].mass; }
for (int i = 0; i < robot->wheel_count; i++) {
int wi = robot->wheel_bodies[i];
if ((wi >= 0) && (wi < world->body_count)) {
total_mass += world->bodies[wi].mass;
}
}
float g_mag = 9.81f;
if (g_cfg.world.gravity < 0.0f) { g_mag = -g_cfg.world.gravity; }
float n_per_wheel = (total_mass * g_mag) / (float) robot->wheel_count;
for (int i = 0; i < robot->wheel_count; i++) {
int wi = robot->wheel_bodies[i];
if ((wi < 0) || (wi >= world->body_count)) { continue; }
rigidbody *wheel = &world->bodies[wi];
/* Only apply when motor command is near-zero (free-rolling) */
if (fabsf(robot->wheel_motors[i].command) > 0.05f) { continue; }
float r = wheel->radius;
if (r <= 0.001f) { continue; }
/* Rolling resistance force opposing rotation about axle */
vector3 axle = wheel->cached_axes[0];
float omega_axle = vector3_dot(wheel->angular_velocity, axle);
if (fabsf(omega_axle) < 0.01f) { continue; }
float f_rr = c_rr * n_per_wheel;
float torque_rr = f_rr * r;
/* Apply opposing torque about axle */
float sign = (omega_axle > 0.0f) ? -1.0f : 1.0f;
vector3 rr_torque = vector3_scaling(axle, sign * torque_rr);
wheel->torque_accumulator = vector3_addition(wheel->torque_accumulator, rr_torque);
} /* MFS_169: driven_this_tick is now set in ftc_robot_update when motor torque is applied */
}
}

#endif /* MPE_DRIVETRAIN_REAL */



    /* MFS_171: Chassis-velocity odometry.
 * Integrates chassis velocity in world space directly.
 * Physically equivalent to a perfect IMU + accelerometer.
 * Avoids wheel axle sign convention issues entirely.
 * Wheel encoder values are still tracked for diagnostics. */
{
    /* Track wheel encoder values */
    for (int mfs_i = 0; mfs_i < robot->wheel_count && mfs_i < FTC_MAX_WHEELS; mfs_i++) {
        int wi = robot->wheel_bodies[mfs_i];
        if ((wi >= 0) && (wi < world->body_count)) {
            rigidbody *w = &world->bodies[wi];
            vector3 axle = w->cached_axes[0];
            if (vector3_length_squared(axle) < 0.0001f) {
                axle = vector4_rotate_to_vector3(w->orientation, (vector3){1.0f, 0.0f, 0.0f});
            }
            float omega = vector3_dot(w->angular_velocity, axle);
            robot->wheel_radians[mfs_i] += omega * dt;
        }
    }
    /* Integrate chassis velocity (world space) */
    vector3 chassis_vel = world->bodies[robot->chassis_body].velocity;
    float yaw_rate = world->bodies[robot->chassis_body].angular_velocity.y;
    robot->odom_theta += yaw_rate * dt;
    robot->odom_x += chassis_vel.x * dt;
    robot->odom_z += chassis_vel.z * dt;
}

}
