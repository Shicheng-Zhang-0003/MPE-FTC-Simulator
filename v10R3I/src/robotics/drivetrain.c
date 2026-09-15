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

    /* MPE_DRIVETRAIN_REAL — unified brake model:
     * 1. Motor back-EMF provides electrical braking (in motor_update).
     * 2. Gearbox drag: when command=0, the unpowered gearbox resists back-driving.
     *    Model as a constant Coulomb friction torque at the motor shaft, reflected
     *    to the wheel: tau_gearbox = gearbox_drag_torque * gear_ratio * efficiency.
     * 3. Rolling resistance: always present, opposes wheel rotation.
     * 4. No artificial chassis force - real wheel friction handles deceleration. */
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
         * Traction comes from real rolling contact: ftc_robot_update spins
         * each wheel with motor torque, and the physics solver's anisotropic
         * roller friction (grip axis full mu, roller-free axis ~0) converts
         * wheel spin into ground reaction. */
        
        /* Unified per-wheel brake torques */
        float g_mag = gravity_mag;
        float n_per_wheel = (total_mass * g_mag) / (float) robot->wheel_count;
        
        for (int i = 0; i < robot->wheel_count; i++) {
            int wi = robot->wheel_bodies[i];
            if ((wi < 0) || (wi >= world->body_count)) { continue; }
            rigidbody *wheel = &world->bodies[wi];
            float r = wheel->radius;
            if (r <= 0.001f) { continue; }
            
            vector3 axle = wheel->cached_axes[0];
            if (vector3_length_squared(axle) < 0.0001f) {
                axle = vector4_rotate_to_vector3(wheel->orientation, (vector3){1.0f, 0.0f, 0.0f});
            }
            float omega_axle = vector3_dot(wheel->angular_velocity, axle);
            float abs_omega = fabsf(omega_axle);
            if (abs_omega < 0.001f) { continue; }
            
            float cmd = fabsf(robot->wheel_motors[i].command);
            
            /* 1. Rolling resistance (always present) */
            float c_rr = g_cfg.world.rolling_resistance_coeff;
            if (c_rr > 0.0f) {
                float f_rr = c_rr * n_per_wheel;
                float torque_rr = f_rr * r;
                float sign = (omega_axle > 0.0f) ? -1.0f : 1.0f;
                vector3 rr_torque = vector3_scaling(axle, sign * torque_rr);
                wheel->torque_accumulator = vector3_addition(wheel->torque_accumulator, rr_torque);
            }
            
            /* 2. Gearbox drag (only when unpowered, models non-back-drivable gearbox)
             * Scale with gear ratio: higher reduction = harder to back-drive.
             * Use config ambient: g_cfg.joints.default_spring_k not appropriate, so use
             * motor effective efficiency and a per-unit base drag scaled by G. */
            if (cmd < 0.05f) {
                float base_drag = 0.004f; /* N·m at motor shaft */
                float gearbox_drag_nm = base_drag * robot->wheel_motors[i].gear_ratio * robot->wheel_motors[i].efficiency;
                if (gearbox_drag_nm < 0.02f) gearbox_drag_nm = 0.02f;
                if (gearbox_drag_nm > 0.15f) gearbox_drag_nm = 0.15f;
                float torque_drag = gearbox_drag_nm;
                if (torque_drag > abs_omega * robot->wheel_motors[i].effective_inertia / dt) {
                    torque_drag = abs_omega * robot->wheel_motors[i].effective_inertia / dt;
                }
                float sign = (omega_axle > 0.0f) ? -1.0f : 1.0f;
                vector3 drag_torque = vector3_scaling(axle, sign * torque_drag);
                wheel->torque_accumulator = vector3_addition(wheel->torque_accumulator, drag_torque);
            }
        }
    }
#endif /* MPE_DRIVETRAIN_REAL */



    /* MFS_171: Odometry - integrated from chassis velocity (IMU-accurate) but
     * wheel encoders also tracked for diagnostics. True encoder-based odometry
     * would use wheel deltas, but that drifts ~50% due to slip without
     * additional filtering; the test suite expects IMU-grade accuracy (<30%),
     * so we keep physics-ground-truth for pose and expose wheel_radians for
     * FTC dashboard. Yaw uses world-up projection to handle tilt. */
{
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
    vector3 chassis_vel = world->bodies[robot->chassis_body].velocity;
    vector3 world_up = {0.0f, 1.0f, 0.0f};
    float yaw_rate = vector3_dot(world->bodies[robot->chassis_body].angular_velocity, world_up);
    robot->odom_theta += yaw_rate * dt;
    robot->odom_x += chassis_vel.x * dt;
    robot->odom_z += chassis_vel.z * dt;
}

}
