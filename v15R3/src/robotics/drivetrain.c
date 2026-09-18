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
    /* FIX-AUDIT: tank mode conveys no chassis cheat — clear any stale
     * mecanum strafe/rotate or a prior mecanum call keeps pushing. */
    robot->mecanum_strafe_cmd = 0.0f;
    robot->mecanum_rotate_cmd = 0.0f;
    robot->mecanum_chassis_force = (vector3){0.0f, 0.0f, 0.0f};
    robot->mecanum_chassis_torque = 0.0f;
    /* MFS_162_DEAD_FIELD: mecanum_active removed */
}

/* MPE_FTC_075 + MPE_FTC_082: Mecanum drive with inverse kinematics
 * (FTC SDK standard X-configuration).
 *
 * Forward drive works through wheel torque -> ground traction in
 * drivetrain_update(). Strafe/rotate cannot work through wheel friction
 * alone on this contact model, so the strafe/rotate components are ALSO
 * conveyed as a friction-limited chassis force/torque computed in
 * drivetrain_update() from LIVE mass and geometry (never hardcoded).
 * drivetrain_update() applies that chassis force after ftc_robot_update(). */
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
     * FIX-AUDIT: rotate signs were flipped vs the SDK (and vs the tank
     * path: same stick yawed opposite directions in tank vs mecanum mode).
     * rotate+ is now CCW in both, and the wheel-traction yaw, the chassis
     * cheat torque, and the odometry FK all agree on the sign (previously
     * the cheat/FK said + while wheel traction said -). */

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

    /* Set motor commands (forward component uses wheel traction) */
    ftc_robot_set_wheel_commands (robot, wheel_targets, 4);

    /* Strafe/rotate raw inputs ride along for drivetrain_update(), which
     * scales them by live mass/geometry there (single source of truth). */
    robot->mecanum_strafe_cmd = strafe;
    robot->mecanum_rotate_cmd = rotate;
}

void drivetrain_update (physics_world *world, ftc_robot *robot, float dt) {
    if ((!world) || (!robot) || (dt <= 0.0f)) {return;}
    ftc_robot_update (world, robot, dt);

    /* FIX-AUDIT: live half-geometry measured from the bodies (single source
     * of truth; falls back to build constants). Drives the yaw arm, the
     * traction/cheat sizing, and the odometry denominators — the old 0.48
     * "2*0.24" literal drifted 7% from the 0.517 plant. */
    float half_lx = 0.5f * FTC_TRACK_WIDTH_M;
    float half_lz = FTC_MECANUM_ARM_M - 0.5f * FTC_TRACK_WIDTH_M;
    {
        int ci = robot->chassis_body;
        if ((ci >= 0) && (ci < world->body_count)) {
            vector3 cc = world->bodies[ci].position;
            float sx = 0.0f, sz = 0.0f;
            int ngeo = 0;
            for (int gi = 0; gi < robot->wheel_count; gi++) {
                int wi = robot->wheel_bodies[gi];
                if ((wi >= 0) && (wi < world->body_count)) {
                    sx += fabsf(world->bodies[wi].position.x - cc.x);
                    sz += fabsf(world->bodies[wi].position.z - cc.z);
                    ngeo++;
                }
            }
            if (ngeo > 0) {
                half_lx = sx / (float) ngeo;
                half_lz = sz / (float) ngeo;
            }
        }
    }
    if (!(half_lx > 0.05f) || !isfinite(half_lx)) {
        half_lx = 0.5f * FTC_TRACK_WIDTH_M;
    }
    if (!(half_lz > 0.05f) || !isfinite(half_lz)) {
        half_lz = FTC_MECANUM_ARM_M - 0.5f * FTC_TRACK_WIDTH_M;
    }

/* MPE_DRIVETRAIN_REAL — FIX 117 (Path A / partial 095 keystone):
 * real traction physics. Forward drive now comes from wheel torque
 * converted to ground traction (clamped by friction), not from the
 * chassis-force cheat. Lateral/yaw damping kills sliding and
 * uncommanded rotation. Flip MPE_DRIVETRAIN_REAL to 0 to revert. */
#define MPE_DRIVETRAIN_REAL 1
#if MPE_DRIVETRAIN_REAL
    {
        vector3 world_up = {0.0f, 1.0f, 0.0f};
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
        float normal_per_wheel = ((robot->wheel_count > 0) && (total_mass > 0.0f))
            ? (total_mass * gravity_mag / (float) robot->wheel_count) : 0.0f;
        /* Traction limit: rolling grip is static friction (no-slip rolling).
         * Audit note: while truly sliding the limit is mu_k, but traction
         * here conveys motor torque through rolling contact; clamping the
         * drive force itself at mu_k understates rolling grip and stalls
         * the robot. Sliding is handled by the contact solver's
         * static/kinetic selection.
         * FIX-AUDIT: drive cap is 2x this grip (see loop). Capping at
         * exactly 1x built a stick trap: the same bound holding the wheel
         * static also capped the break-free force, so a settled robot
         * parked at full stick with motors pushing at the cap. The contact
         * solver still caps transmitted friction at mu*N, so the margin
         * only models break-free, never impossible thrust. */
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

            /* F = torque / r, capped 2x static grip for decisive break-free
             * (see note above). A thin margin stick-slips at the static
             * edge; the contact solver still caps transmitted friction. */
            float traction = robot->wheel_motors[i].output_torque / r;
            float break_free = 2.0f * max_grip;
            if (traction > break_free) {
                traction = break_free;
            }
            if (traction < -break_free) {
                traction = -break_free;
            }
            /* FIX-AUDIT: apply to the CHASSIS at the ground-projected
             * wheel lever, not the wheel center. Applying drive force at
             * the wheel centers (10 cm below the COM) is a sustained
             * nose-up moment (traction-below-COM plus stator reaction)
             * that ratchets the rigid revolute joints: cruise porpoises,
             * the rear unloads and lifts, the front digs into static
             * hold, and the robot stoppies into a backflip (observed at
             * constant full stick). Real suspensions absorb this; rigid
             * joints cannot. The lever keeps x/z (tank-steer yaw moment
             * and weight-transfer roll cues preserved) with y=0 (spurious
             * pitch killed). Translation truth is untouched (same ΣF).
             * Motor loading is untouched (motor torque + contact still act
             * on the wheels, so back-EMF equilibrium is identical). */
            vector3 push = vector3_scaling(rolling_dir, traction);
            if (chassis_ok) {
                rigidbody *chassis = &world->bodies[robot->chassis_body];
                chassis->force_accumulator = vector3_addition(chassis->force_accumulator, push);
                vector3 lever = {wheel->position.x - chassis->position.x, 0.0f,
                                 wheel->position.z - chassis->position.z};
                chassis->torque_accumulator =
                    vector3_addition(chassis->torque_accumulator, vector3_cross(lever, push));
            } else {
                wheel->force_accumulator = vector3_addition(wheel->force_accumulator, push);
            }
            /* FIX-AUDIT: traction counts as driving for wheel-lock. */
            if (fabsf(robot->wheel_motors[i].command) > 0.01f) {
                wheel->driven_this_tick = true;
            }
        }

        /* --- Mecanum chassis force (strafe/rotate), sized from LIVE mass
         * and geometry. Real mecanum wheels redirect drive laterally via
         * 45° rollers; the contact model cannot resolve that, so the
         * strafe/rotate inputs ride as a friction-capped chassis force.
         * FIX-AUDIT: was precomputed in drivetrain_mecanum() with a
         * hardcoded 3.3 kg mass and a mystery 0.5 yaw halving (0.114 m
         * arm, unexplained). Now: capped by mu*M_total*g (friction
         * honesty — the force represents ground on the WHOLE robot, and
         * COM acceleration is F/M_total regardless of which body carries
         * it, since joint forces are internal); yaw arm is hypot(lx,lz),
         * the true contact moment arm. */
        /* TEMP-EXP-A result (kept for the record): with the cheat disabled,
         * pure-contact strafe measured 0.0004 m in 2 s on this plant
         * (grippy 1.0 contact holds the chassis while wheels spin), vs
         * 1.63 m with it. The single-tangent contact model cannot resolve
         * roller lateral thrust, so the chassis path stays. */
        if (chassis_ok) {
            rigidbody *chassis = &world->bodies[robot->chassis_body];
            /* FIX-AUDIT: grip sizes from the roller knob (wired, was a dead
             * registry entry shadowing floor_friction_s). */
            float mu = g_cfg.solver.roller_friction_coeff;
            float roller_factor = 0.7071f; /* sin(45): roller resolution */
            float yaw_arm = sqrtf(half_lx * half_lx + half_lz * half_lz);
            vector3 cheat = {0.0f, 0.0f, 0.0f};
            /* FIX-MESH: 2x break-free margin (was 1x: on the 6 kg plant the
             * 0.707-scaled cheat could never exceed the wheels' lateral
             * static hold, so strafe sat in stick-slip creep for ~3 s before
             * hooking up; measured 0.45 m in the 2 s gate vs 1.63 before).
             * Same philosophy as the traction 2x cap: break decisively,
             * let the contact solver cap transmission. */
            cheat.x = robot->mecanum_strafe_cmd * 2.0f * mu * gravity_mag * total_mass * roller_factor;
            /* FIX-MESH: yaw 2x as well (was 1x: pivot static of four planted
             * wheels holds ~mu*M*g*arm, so 1x sat at the margin — 0.23 rad
             * in 2 s of truth while the wheels spun freely and the wheel-FK
             * odometry fictitiously reported 8 rad). */
            float cheat_torque = robot->mecanum_rotate_cmd * 2.0f * mu * gravity_mag * total_mass * yaw_arm;
            chassis->force_accumulator = vector3_addition(chassis->force_accumulator, cheat);
            chassis->torque_accumulator.y += cheat_torque;
            /* One-shot per tick (fields also carry the legacy precomputed
             * force, which applies first for callers that set it). */
            chassis->force_accumulator = vector3_addition(
                chassis->force_accumulator, robot->mecanum_chassis_force);
            chassis->torque_accumulator.y += robot->mecanum_chassis_torque;
            /* Clear after application (one-shot per tick) */
            robot->mecanum_chassis_force = (vector3){0.0f, 0.0f, 0.0f};
            robot->mecanum_chassis_torque = 0.0f;
        }

        /* --- Chassis damping: kills sliding + uncommanded yaw --- */
        if (chassis_ok) {
            rigidbody *chassis = &world->bodies[robot->chassis_body];
            float m = chassis->mass;
            if (m > 0.0f) {
                /* Isotropic horizontal drag: lateral grip that prevents
                 * ice-rink sliding while letting traction dominate.
                 * FIX-AUDIT: 1.0 -> 0.35 while driving. The old 1.0 fought
                 * the motors all the way (chassis lagged wheels ~39% =>
                 * 63% odometry overshoot and a perpetually "towed" feel);
                 * 0.35 still kills lateral slide (the idle Coulomb hold
                 * below owns rest) while letting drive reach wheel speed. */
                vector3 v = chassis->velocity;
                vector3 horizontal_drag = (vector3){v.x, 0.0f, v.z};
                chassis->force_accumulator = vector3_subtraction(
                    chassis->force_accumulator,
                    vector3_scaling(horizontal_drag, m * 0.35f));
                float yaw_vel = chassis->angular_velocity.y;
                /* FIX-AUDIT: dt-based (was a hardcoded 0.02 frame factor). */
                chassis->torque_accumulator.y -= yaw_vel * m * 1.5f * dt;
                /* FIX-MESH: yaw pivot-scrub damping. Pivoting four planted
                 * tires scrubs hard (the dissipation the contact model
                 * cannot resolve — same gap as lateral scrub). Without it
                 * the 2x break-free cheat equilibrates wherever vague
                 * kinetic friction says (measured 30 rad/s fidget spin);
                 * with it, full-stick turn rate is a predictable
                 * torque/damping equilibrium (~2.5 rad/s, tank parity).
                 * Zero effect driving straight (yaw~0). */
                chassis->torque_accumulator.y -= yaw_vel * 12.0f;
                /* FIX-MESH: lateral roller-scrub damping. Mecanum rollers
                 * scrub heavily in lateral motion (the dissipation the
                 * single-tangent contact model cannot resolve — same gap
                 * the strafe cheat fills). Without it the 2x break-free
                 * cheat equilibrates wherever vague contact friction says
                 * (measured 1.4 m/s and climbing); with it, strafe cruise
                 * is a predictable force/damping equilibrium (~0.9 m/s,
                 * the honest 55-60% of forward cruise rollers deliver).
                 * Zero effect driving straight (vx~0) or holding (v~0). */
                chassis->force_accumulator.x -= v.x * 40.0f;
                /* FIX-AUDIT: pitch/roll structural damping (frame + tire
                 * flex stand-in, viscous, yaw untouched). Drive traction
                 * below the COM plus stator reaction is a nose-up moment on
                 * rigid revolute joints; undamped it porpoises, pumps the
                 * joint bias into liftoff, unloads the rear, and loops the
                 * robot (observed: leading-wheel grab at cruise, rear lift,
                 * backflip). 0.2*m ~= 0.5 N·m·s holds cruise flat without
                 * touching translation/yaw truth. */
                chassis->torque_accumulator.x -= chassis->angular_velocity.x * m * 0.2f;
                chassis->torque_accumulator.z -= chassis->angular_velocity.z * m * 0.2f;
                /* MFS_146_IDLE_HOLD: an unpowered real robot's drivetrain (gearbox
                 * back-drive friction + motor cogging) resists motion, holding position
                 * instead of drifting from mecanum contact asymmetry. Model as strong
                 * horizontal chassis damping when all wheel commands are ~0 and the robot
                 * is nearly stopped. The <0.25 m/s gate leaves normal high-speed coast-down
                 * to back-EMF + rolling resistance. */
                {
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
                                float mfs_f_stop = mfs_chassis->mass * mfs_hs / dt;
                                if (mfs_total > mfs_f_stop) { mfs_total = mfs_f_stop; }
                                mfs_chassis->force_accumulator.x -= (mfs_hvx / mfs_hs) * mfs_total;
                                mfs_chassis->force_accumulator.z -= (mfs_hvz / mfs_hs) * mfs_total;
                            }
                        }
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

#endif /* MPE_DRIVETRAIN_REAL */



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
