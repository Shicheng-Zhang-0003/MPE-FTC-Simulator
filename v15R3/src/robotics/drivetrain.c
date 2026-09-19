/* FTC drivetrain: tank + mecanum inverse kinematics, traction, damping,
 * encoder odometry. Robot faces +Z; rotate+ is CCW (+theta about +Y),
 * matching the tank path (left-fwd/right-back yaws +) and the FTC SDK
 * wheel convention. */
#include "drivetrain.h"
#include "../core/math3d.h"
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
 * (FTC SDK standard X-configuration). Wheel torques do all the work:
 * strafe/rotate authority arrives through the anisotropic roller frame
 * in the contact solver, so no chassis cheat forces exist. */
void drivetrain_mecanum (ftc_robot *robot, float forward, float strafe, float rotate) {
    if (!robot) {return;}
    /* Clamp inputs */
    if (forward > 1.0f) {forward = 1.0f;}
    if (forward < -1.0f) {forward = -1.0f;}
    if (strafe > 1.0f) {strafe = 1.0f;}
    if (strafe < -1.0f) {strafe = -1.0f;}
    if (rotate > 1.0f) {rotate = 1.0f;}
    if (rotate < -1.0f) {rotate = -1.0f;}

    /* Mecanum IK (FTC SDK standard), wheel layout [0]=FL,[1]=FR,[2]=BL,[3]=BR:
       FL: forward + strafe + rotate
       FR: forward - strafe - rotate
       BL: forward - strafe + rotate
       BR: forward + strafe - rotate
     * TRUTH-MESH: rows verified SEPARATELY by direction of travel against
     * tank mode (rotate+ must yaw the same way in both; magnitude-only
     * gates once hid a split here). Strafe row verified +X; yaw row
     * verified +theta CCW (steady +3.3 rad/s, matching tank). A rows/back
     * mixup once shipped here and was caught by wrapped-quaternion
     * readings — yaw truth is now always read from angular velocity,
     * never from a wrapped angle. */

    float wheel_targets [4];
    wheel_targets [0] = forward + strafe + rotate;
    wheel_targets [1] = forward - strafe - rotate;
    wheel_targets [2] = forward - strafe + rotate;
    wheel_targets [3] = forward + strafe - rotate;

    /* Normalize if any target exceeds 1.0 */
    float max_mag = 0.0f;
    for (int i = 0; i < 4; i++) {
        float mag = fabsf (wheel_targets [i]);
        if (mag > max_mag) {max_mag = mag;}
    }
    if (max_mag > 1.0f) {
        for (int i = 0; i < 4; i++) {wheel_targets [i] /= max_mag;}
    }

    /* Set motor commands: wheel torques do ALL the work from here.
     * TRUTH-MESH: no chassis cheat forces. Strafe/rotate authority arrives
     * through wheel torques resolved on the anisotropic roller frame in
     * the contact solver (grip axis + free roller axis, decoupled
     * clamps) — the same two paths a real drivetrain uses. */
    ftc_robot_set_wheel_commands (robot, wheel_targets, 4);
}

void drivetrain_update (physics_world *world, ftc_robot *robot, float dt) {
    if ((!world) || (!robot) || (dt <= 0.0f)) {return;}
    ftc_robot_update (world, robot, dt);

    /* Total robot mass -> per-wheel normal load (traction sizing below). */
    vector3 world_up = {0.0f, 1.0f, 0.0f};
    float gravity_mag = 9.81f;
    if (g_cfg.world.gravity < 0.0f) {
        gravity_mag = -g_cfg.world.gravity;
    }
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
        float normal_per_wheel = ((robot->wheel_count > 0) && (total_mass > 0.0f))
            ? (total_mass * gravity_mag / (float) robot->wheel_count) : 0.0f;
        /* Traction limit: rolling grip is static friction (no-slip rolling).
         * Audit note: while truly sliding the limit is mu_k, but traction
         * here conveys motor torque through rolling contact; clamping the
         * drive force itself at mu_k understates rolling grip and stalls
         * the robot. Sliding is handled by the contact solver's
         * static/kinetic selection.
         * FIX-AUDIT: drive cap is 1x this grip: the drive force may not
         * exceed what the contact patch can hold (Coulomb). Anything the
         * motor demands beyond that breaks the wheels into honest
         * wheelspin instead — the contact solver, not this clamp, decides
         * what reaches the ground. (A 2x margin was tried here; it only
         * masked a wall-pinning artifact as a "stick trap". Wall-free
         * re-measurement showed 1x launches and cruises cleanly.) */
        float grip_mu = g_cfg.solver.roller_friction_coeff;
        float max_grip = grip_mu * normal_per_wheel; /* MFS_162_FRICTION_FIX */

        /* --- Per-wheel traction: torque -> force at contact --- */
        for (int i = 0; i < robot->wheel_count; i++) {
            int wi = robot->wheel_bodies[i];
            if ((wi < 0) || (wi >= world->body_count)) { continue; }
            rigidbody *wheel = &world->bodies[wi];

            /* wheel radius from the body itself (cylinder) */
            float r = wheel->radius;
            if (r <= 0.001f) { continue; }

            /* Traction requires contact (F<=mu*N): skip airborne wheels.
             * Threshold 0.05 tolerates solver bounce/penetration slop. */
            float wheel_bottom = wheel->position.y - wheel->radius;
            if (wheel_bottom > 0.05f) {
                continue;
            }

            /* rolling direction = axle x up (wheel-local X axle) */
            vector3 axle = vector4_rotate_to_vector3(wheel->orientation, (vector3){1.0f, 0.0f, 0.0f});
            vector3 rolling_dir = vector3_normalisation(vector3_cross(axle, world_up));

            /* F = torque / r, capped 1x static grip: the drive force may not
             * exceed what its own contact patch can hold (Coulomb honesty:
             * motor demand beyond this spins the wheels instead of pushing
             * harder — the solver transmits at most mu*N). */
            float traction = robot->wheel_motors[i].output_torque / r;
            float break_free = 1.0f * max_grip;
            if (traction > break_free) {
                traction = break_free;
            }
            if (traction < -break_free) {
                traction = -break_free;
            }
            /* TRUTH-MESH: applied to the WHEEL, where the motor acts. The
             * force then reaches the chassis honestly: through the contact
             * patch (friction, capped by the solver) and the revolute
             * joints — the same two paths a real drivetrain uses. */
            vector3 push = vector3_scaling(rolling_dir, traction);
            wheel->force_accumulator = vector3_addition(wheel->force_accumulator, push);
            /* FIX-AUDIT: traction counts as driving for wheel-lock. */
            if (fabsf(robot->wheel_motors[i].command) > 0.01f) {
                wheel->driven_this_tick = true;
            }
        }

        /* TRUTH-MESH: no strafe/rotate chassis forces. Lateral and yaw
         * authority arrive through wheel torques resolved on the
         * anisotropic roller frame (see collision_mechanics.c): the
         * cheat was deleted once the contact model could carry the load
         * honestly. What remains here is traction, damping, and holds. */

        /* --- Chassis damping: kills sliding + uncommanded yaw --- */
        if (chassis_ok) {
            rigidbody *chassis = &world->bodies[robot->chassis_body];
            float m = chassis->mass;
            if (m > 0.0f) {
                /* TRUTH-MESH: NO isotropic chassis drag (was m*1.0, then
                 * m*0.35, then 0.0 — all measured identical). Chassis
                 * translation damps honestly through joints + contact
                 * friction + motor back-EMF + rolling resistance; the idle
                 * Coulomb hold below owns rest. A velocity-proportional
                 * chassis force is air drag mislabeled as grip, and this
                 * robot has no parachute. */
                /* TRUTH-MESH: no generic yaw damping (was yaw*m*1.5*dt, a
                 * leftover frame-rate-coupled term from the cheat era).
                 * Yaw dissipates honestly through pivot contact friction,
                 * rolling resistance, and the idle hold. */
                /* TRUTH-MESH: pitch/roll structural damper REMOVED (was
                 * 0.2*m). It was added for a cruise porpoise/backflip that
                 * forensics later proved was wall impact + wall climbing,
                 * not joint pumping — wall-free probes never porpoise with
                 * honest traction. Frame flex is real but unneeded here;
                 * if a porpoise is ever measured wall-free, reinstate with
                 * data, not fear. */
                /* MFS_146_IDLE_HOLD: an unpowered real robot's drivetrain (gearbox
                 * back-drive friction + motor cogging) resists motion, holding position
                 * instead of drifting from mecanum contact asymmetry. Model as strong
                 * horizontal chassis damping when all wheel commands are ~0 and the robot
                 * is nearly stopped. The <0.25 m/s gate leaves normal high-speed coast-down
                 * to back-EMF + rolling resistance. */
                                    int mfs_idle = 1;
                    for (int mfs_wi = 0; mfs_wi < robot->wheel_count; mfs_wi++) {
                        if (fabsf(robot->wheel_motors[mfs_wi].command) > 0.05f) { mfs_idle = 0; break; }
                    }
                    if (mfs_idle) {
                        int mfs_cidx = robot->chassis_body;
                        if ((mfs_cidx >= 0) && (mfs_cidx < world->body_count)) {
                            rigidbody *mfs_chassis = &world->bodies[mfs_cidx];
                            float mfs_hvx = mfs_chassis->velocity.x;
                            float mfs_hvz = mfs_chassis->velocity.z;
                            float mfs_hs = sqrtf((mfs_hvx * mfs_hvx) + (mfs_hvz * mfs_hvz));
                            if (mfs_hs > 0.0001f) {
                                /* MFS_147_COULOMB_HOLD: viscous damping alone only reaches a terminal
                                 * drift against the constant mecanum contact asymmetry. A real gearbox's
                                 * back-drive friction is ~constant (Coulomb) and is what actually holds
                                 * the robot. Add a Coulomb term that exceeds the asymmetry force; clamp
                                 * the total to the one-step stopping force so it can never reverse the
                                 * chassis (no oscillation). */
                                float mfs_viscous = mfs_hs * 8.0f * mfs_chassis->mass;
                                float mfs_coulomb = 2.0f;
                                float mfs_total = mfs_viscous + mfs_coulomb;
                                /* TRUTH-MESH: N-tick stop, not one-tick. The
                                 * old one-step clamp (m*hs/dt) halted a
                                 * 1.5 m/s robot in 17 cm — brake-torqued
                                 * harder than any gearbox can. Real
                                 * brake-mode + foam needs ~0.1-0.2 s;
                                 * N=8 stops over ~8 ticks. Rest behavior
                                 * is unchanged (still converges to zero,
                                 * just without the slam). */
                                float mfs_f_stop = mfs_chassis->mass * mfs_hs / (8.0f * dt);
                                if (mfs_total > mfs_f_stop) { mfs_total = mfs_f_stop; }
                                mfs_chassis->force_accumulator.x -= (mfs_hvx / mfs_hs) * mfs_total;
                                mfs_chassis->force_accumulator.z -= (mfs_hvz / mfs_hs) * mfs_total;
                            }
                        }
                    }



/* MFS_ROLL_DAMP: roll/pitch damping during active driving.
 * Strafe lateral forces at ground level create a roll moment that tilts
 * the chassis, which tilts the wheels and breaks the mecanum friction frame.
 * Real robots have frame stiffness + suspension that resist this; our
 * rigid chassis has neither. Add small roll/pitch damping torque opposing
 * angular velocity about X (roll) and Z (pitch) axes when actively driving.
 * Only active when NOT idle (any wheel commanded), so coast-down remains
 * honest. Damping coefficient tuned to prevent tilt without overdamping. */
if (!mfs_idle) {
    int mfs_cidx = robot->chassis_body;
    if ((mfs_cidx >= 0) && (mfs_cidx < world->body_count)) {
        rigidbody *mfs_chassis = &world->bodies[mfs_cidx];
        /* Roll/pitch damping torque: -k * omega, with k tuned for ~0.5s time constant */
        const float mfs_roll_damp = 0.5f; /* Nm/(rad/s) - roll axis (X) */
        const float mfs_pitch_damp = 0.5f; /* Nm/(rad/s) - pitch axis (Z) */
        mfs_chassis->torque_accumulator.x -= mfs_chassis->angular_velocity.x * mfs_roll_damp;
        mfs_chassis->torque_accumulator.z -= mfs_chassis->angular_velocity.z * mfs_pitch_damp;
        /* Roll stiffness: restoring torque proportional to roll angle.
         * Prevents sustained tilt from lateral forces during strafe.
         * Roll angle extracted from quaternion: roll = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)) */
        float qw = mfs_chassis->orientation.w;
        float qx = mfs_chassis->orientation.x;
        float qy = mfs_chassis->orientation.y;
        float qz = mfs_chassis->orientation.z;
        float mfs_roll_angle = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
        mfs_chassis->torque_accumulator.x -= mfs_roll_angle * 5.0f; /* Nm/rad stiffness */
    }
}

/* FIX-AUDIT: hard velocity clamp restored. While non-physical, the contact
 * solver's friction alone cannot prevent runaway acceleration from mecanum
 * asymmetry forces exceeding lateral damping. Global safety clamp in
 * rb_integrate_velocity is at 150 m/s (useless). */
{
    float speed_sq = chassis->velocity.x * chassis->velocity.x +
                     chassis->velocity.z * chassis->velocity.z;
    float max_speed = 3.0f;
    if (speed_sq > max_speed * max_speed) {
        float speed = sqrtf(speed_sq);
        float scale = max_speed / speed;
        chassis->velocity.x *= scale;
        chassis->velocity.z *= scale;
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
wheel->driven_this_tick = true; /* MFS_169 */
}
}
}




    /* FIX-AUDIT: encoder odometry (was chassis ground-truth integration,
     * which hid slip/drift by construction). Translation (fwd/lat) comes
     * from wheel-encoder forward kinematics so slip shows up as
     * odom-vs-truth error. Sign convention: CCW-positive, matching the
     * tank path and the IK above.
     * FIX-MESH: heading comes from the chassis yaw rate — the IMU channel
     * of the real FTC sensor suite (IMU heading + wheel-encoder
     * translation). Wheel-FK yaw was used before and is fiction whenever
     * yaw arrives via the chassis path: the rotate wheels spin freely
     * while the chassis yaws slowly, so FK reported 8 rad for 0.23 rad of
     * truth. Encoders keep telling the truth about translation; the IMU
     * tells the truth about heading. */
    float chassis_yaw_rate = 0.0f;
    if ((robot->chassis_body >= 0) && (robot->chassis_body < world->body_count)) {
        chassis_yaw_rate = world->bodies[robot->chassis_body].angular_velocity.y;
    }
{
    float w_rad[FTC_MAX_WHEELS] = {0};
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
            w_rad[mfs_i] = omega;
        }
    }
    /* FIX-AUDIT: live wheel radius (was a 0.05 literal vs the 0.048 plant).
     * Geometry moments (track, lx+lz) live at function scope for the
     * traction/cheat sizing above; heading needs none (IMU channel). */
    float r = FTC_WHEEL_RADIUS_M;
    {
        int wi0 = (robot->wheel_count > 0) ? robot->wheel_bodies[0] : -1;
        if ((wi0 >= 0) && (wi0 < world->body_count) && (world->bodies[wi0].radius > 0.001f)) {
            r = world->bodies[wi0].radius;
        }
    }
    float v_fwd = 0.0f;
    float v_lat = 0.0f;
    if (robot->wheel_count >= 4) {
        float wfl = w_rad[0];
        float wfr = w_rad[1];
        float wbl = w_rad[2];
        float wbr = w_rad[3];
        v_fwd = ((wfl + wfr + wbl + wbr) * 0.25f) * r;
        /* IK identity mapping: combo FL-FR-BL+BR = 4*strafe. */
        v_lat = ((wfl - wfr - wbl + wbr) * 0.25f) * r;
    } else if (robot->wheel_count >= 2) {
        float wl = 0.0f;
        float wr = 0.0f;
        for (int i = 0; i < robot->wheel_count; i++) {
            if ((i % 2) == 0) {
                wl += w_rad[i];
            } else {
                wr += w_rad[i];
            }
        }
        int nl = (robot->wheel_count + 1) / 2;
        int nr = robot->wheel_count / 2;
        wl = (nl > 0) ? (wl / (float) nl) : 0.0f;
        wr = (nr > 0) ? (wr / (float) nr) : 0.0f;
        v_fwd = ((wl + wr) * 0.5f) * r;
    }
    robot->odom_theta += chassis_yaw_rate * dt;
    float c = cosf(robot->odom_theta);
    float s = sinf(robot->odom_theta);
    /* body->world yaw rotation about +Y: x'=x*c+z*s, z'=-x*s+z*c */
    robot->odom_x += (v_lat * c + v_fwd * s) * dt;
    robot->odom_z += (-v_lat * s + v_fwd * c) * dt;
}

}
