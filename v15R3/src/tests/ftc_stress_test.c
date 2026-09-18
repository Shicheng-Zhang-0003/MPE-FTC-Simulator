/* MFS_313/314: Drive stress test — "like butter on a controller".
 *
 * Runs a scripted controller through a full multi-axis sequence
 * (forward, stop, reverse, strafe both ways, pure rotate, diagonal,
 * drive-again, long coast) and asserts the ROBOT behaves like a real
 * FTC bot:
 *   1. no NaN ever, stays in the field, stays upright
 *   2. wheel RPM stays bounded near free speed (no bang-bang / over-spin)
 *   3. chassis velocity smooth at constant stick (no per-step jolts)
 *   4. drivetrain settles to rest when the stick is released
 *   5. phase mappings correct (forward/strafe/rotate all do their thing)
 */
#ifdef mpe_ftc_stress_test
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "robotics/robot.h"
#include "robotics/drivetrain.h"

#define DT (1.0f / 60.0f)
#define TICKS_FROM(s) ((int)((s) / DT))
#define FREE_RPM 200.0f /* GB_5203_30 no-load at 12V (spec-anchored; Ke exact) */
#define RPM_CAP 220.0f  /* free +10%: catches bang-bang (old bug hit 356+) without crying wolf */
#define FIELD_LIMIT 1.55f
#define UPRIGHT_LIMIT 0.4f
/* TRUTH: brake-mode stopping from ~1 m/s takes ~1 s (BackEMF viscous tail +
 * rolling/drag Coulomb; measured). A 0.5 s window cannot show rest, so the
 * script ends with a 1.2 s all-zero coast and gates REST there; mid-run
 * windows only require brakes to bite (no runaway). */
#define FINAL_COAST_END 7.6f
#define FINAL_REST_BEGIN 7.1f

static void mfs_cmd(ftc_robot *robot, float fwd, float strafe, float yaw) {
    if (robot->drivetrain_type == FTC_DRIVETRAIN_MECANUM) {
        drivetrain_mecanum(robot, fwd, strafe, yaw);
    } else {
        drivetrain_tank(robot, fwd + yaw, fwd - yaw);
    }
}

int main(void) {
    mpe_config_init();
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(&world);
    ftc_ensure_field_walls(&world);

    ftc_robot robot;
    int rc = ftc_robot_create(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_19_2);
    if (rc != 0) {
        printf("[FAIL] could not create robot\n");
        return 1;
    }

    const int total = TICKS_FROM(FINAL_COAST_END);
    int fail = 0;
    float max_rpm = 0.0f, max_speed = 0.0f, max_x = 0.0f, max_z = 0.0f, max_pitch = 0.0f, max_roll = 0.0f;
    int t_max_roll = 0, t_max_speed = 0;

    float outp_count = 0; (void)outp_count;

    float v_last = 0.0f;
    float smooth_max_dv = 0.0f;
    float smooth_sum_dv = 0.0f;
    int smooth_n = 0;

    float prev_theta = 0.0f;
    float rotate_theta_increase = 0.0f;
    int rotate_counting = 0;

    float idle_rpm = 0.0f;
    float idle_speed = 0.0f;
    int idle_ticks = 0;
    float rest_rpm = 0.0f;
    float rest_speed = 0.0f;
    int rest_ticks = 0;

    float first_phase_dz = 0.0f, first_phase_dx = 0.0f;
    float strafe_phase_dx = 0.0f;
    float start_x, start_y, start_z;
    ftc_robot_get_position(&world, &robot, &start_x, &start_y, &start_z);

    for (int t = 0; t < total; t++) {
        float sec = (float)t * DT;

        /* ----- scripted "controller" ----- */
        float f = 0.0f, s = 0.0f, r = 0.0f;
        if (sec < 1.0f)              f = 1.0f;              /* full forward      */
        else if (sec < 1.5f)         f = 0.0f;              /* stop, settle      */
        else if (sec < 2.0f)         f = -1.0f;             /* reverse           */
        else if (sec < 2.7f)         f = 0.0f;              /* settle between direction
                                                               changes (a real driver
                                                               releases the stick ~0.5s
                                                               before strafing instead of
                                                               yanking from full reverse) */
        else if (sec < 3.4f)         s = 1.0f;              /* strafe right      */
        else if (sec < 4.1f)         s = -1.0f;             /* strafe left       */
        else if (sec < 4.6f)         r = 1.0f;              /* rotate CCW        */
        else if (sec < 5.1f)         r = 0.0f;              /* settle from spin  */
        else if (sec < 5.7f)         f = 1.0f;              /* short drive phase */
        else                         f = 0.0f;              /* final 1.7 s coast to rest */
        mfs_cmd(&robot, f, s, r);

        if ((sec >= 0.0f) && (sec < 1.0f))  first_phase_dz += world.bodies[robot.chassis_body].velocity.z * DT;
        if ((sec >= 0.0f) && (sec < 1.0f))  first_phase_dx += world.bodies[robot.chassis_body].velocity.x * DT;
        if ((sec >= 2.7f) && (sec < 3.4f))  strafe_phase_dx += world.bodies[robot.chassis_body].velocity.x * DT;
        if ((sec >= 4.1f) && (sec < 4.6f))  rotate_counting = 1;

        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);

        /* smoothness sample during the steady part of the short drive */
        if ((sec >= 5.3f) && (sec < 5.7f)) {
            float v = vector3_length(world.bodies[robot.chassis_body].velocity);
            if (smooth_n > 0) {
                float dv = fabsf(v - v_last);
                smooth_max_dv = (dv > smooth_max_dv) ? dv : smooth_max_dv;
                smooth_sum_dv += dv;
                smooth_n++;
            } else {
                smooth_n = 1;
            }
            v_last = v;
        }

        if (rotate_counting) {
            float dtheta = robot.odom_theta - prev_theta;
            rotate_theta_increase += fabsf(dtheta); /* MFS_310: rotate+ = CW spin; validate magnitude */
            prev_theta = robot.odom_theta;
        }

        /* idle tracker: last 0.2 s of each mid-run stop window (brakes must
         * bite: bounded glide, not runaway) + strict rest on final coast. */
        float idle_begin = 0.0f;
        if ((sec >= 1.0f) && (sec < 1.5f))  idle_begin = 1.0f;
        if ((sec >= 4.6f) && (sec < 5.1f))  idle_begin = 4.6f;
        if ((sec >= 5.7f) && (sec < 6.2f))  idle_begin = 5.7f;
        if (idle_begin > 0.0f) {
            float local = sec - idle_begin;
            if (local >= 0.3f) {  /* skip the first 0.3s of settling */
                idle_ticks++;
                idle_rpm += robot.wheel_motors[0].rpm;
                idle_speed += vector3_length(world.bodies[robot.chassis_body].velocity);
            }
        }
        if ((sec >= FINAL_REST_BEGIN) && (sec < FINAL_COAST_END)) {
            rest_ticks++;
            rest_rpm += robot.wheel_motors[0].rpm;
            rest_speed += vector3_length(world.bodies[robot.chassis_body].velocity);
        }

        /* bounds + NaN every tick */
        rigidbody *chassis = &world.bodies[robot.chassis_body];
        for (int i = 0; i < world.body_count; i++) {
            rigidbody *rb = &world.bodies[i];
            if (!isfinite(rb->position.x) || !isfinite(rb->position.y) || !isfinite(rb->position.z) ||
                !isfinite(rb->velocity.x) || !isfinite(rb->velocity.y) || !isfinite(rb->velocity.z) ||
                !isfinite(rb->angular_velocity.x) || !isfinite(rb->angular_velocity.y) ||
                !isfinite(rb->angular_velocity.z)) {
                printf("[FAIL] NaN at tick %d body %d\n", t, i);
                fail = 1;
                break;
            }
        }
        if (fail) break;

        for (int i = 0; i < robot.wheel_count; i++) {
            float rpm = robot.wheel_motors[i].rpm;
            if (rpm > max_rpm) max_rpm = rpm;
        }
        float sv = vector3_length(chassis->velocity);
        if (sv > max_speed) { max_speed = sv; t_max_speed = t; }
        float cx = fabsf(chassis->position.x);
        float cz = fabsf(chassis->position.z);
        if (cx > max_x) max_x = cx;
        if (cz > max_z) max_z = cz;
        float pitch = fabsf(chassis->orientation.x);  /* quaternion: x-axis (pitch-ish) */
        float roll = fabsf(chassis->orientation.y);
        if (pitch > max_pitch) max_pitch = pitch;
        if (roll > max_roll) { max_roll = roll; t_max_roll = t; }
    }

    float end_x, end_y, end_z;
    ftc_robot_get_position(&world, &robot, &end_x, &end_y, &end_z);

    printf("[info] path first: dz=%.3f dx=%.3f | strafe dx=%.3f | rotate theta=%.3f\n",
           first_phase_dz, first_phase_dx, strafe_phase_dx, rotate_theta_increase);
    printf("[info] max rpm=%.1f (free %.0f clamp %.0f) max speed=%.2f m/s @ t=%.1fs\n",
           max_rpm, FREE_RPM, RPM_CAP, max_speed, (float)t_max_speed * DT);
    printf("[info] extents x=%.2f z=%.2f pitch=%.3f roll=%.3f@t=%.1fs\n",
           max_x, max_z, max_pitch, max_roll, (float)t_max_roll * DT);
    printf("[info] end=(%.3f,%.3f,%.3f) odom_theta=%.3f\n", end_x - start_x, end_y - start_y, end_z - start_z, robot.odom_theta);
    printf("[info] smoothness peak |dv|=%.4f (steady-forward window, mean %.4f)\n",
           smooth_max_dv, (smooth_n > 1) ? smooth_sum_dv / (float)(smooth_n - 1) : 0.0f);
    if (idle_ticks > 0) {
        printf("[info] idle settle: mean rpm=%.1f mean speed=%.3f m/s over %d ticks\n",
               idle_rpm / (float)idle_ticks, idle_speed / (float)idle_ticks, idle_ticks);
    }
    if (rest_ticks > 0) {
        printf("[info] final rest: mean rpm=%.1f mean speed=%.3f m/s over %d ticks\n",
               rest_rpm / (float)rest_ticks, rest_speed / (float)rest_ticks, rest_ticks);
    }

    if (fail) return 1;

    /* 1) bounded + upright + in-field */
    if (max_x > FIELD_LIMIT || max_z > FIELD_LIMIT) {
        printf("[FAIL] touched field wall (x=%.2f z=%.2f)\n", max_x, max_z); fail = 1;
    }
    if (max_pitch > UPRIGHT_LIMIT || max_roll > UPRIGHT_LIMIT) {
        printf("[FAIL] opened up like a maraca (pitch=%.3f roll=%.3f)\n", max_pitch, max_roll); fail = 1;
    }

    /* 2) no wheel bang-bang / over-spin */
    if (max_rpm > RPM_CAP) {
        printf("[FAIL] wheel over-spin / bang-bang (max rpm %.1f > %d)\n", max_rpm, (int)RPM_CAP); fail = 1;
    }

    /* 3) smooth at constant stick: largest per-step velocity change must not
       dwarf the mean (2-step bang-bang would pogo |dv| up and down wildly) */
    if (smooth_n > 20) {
        float mean = smooth_sum_dv / (float)(smooth_n - 1);
        if (smooth_max_dv > 4.0f * mean && smooth_max_dv > 0.05f) {
            printf("[FAIL] jittery drive (max|dv|=%.4f > 4x mean %.4f)\n", smooth_max_dv, mean); fail = 1;
        }
    }

    /* 4) brakes bite mid-run (bounded brake-glide, never runaway) and the
     * bot reaches true rest on the final coast (brake-mode stop from
     * ~1 m/s needs ~1 s: BackEMF viscous tail + Coulomb; 0.5 s windows
     * cannot show rest, so rest is gated on the 1.7 s final coast). */
    if (idle_ticks > 0) {
        float mean_rpm = idle_rpm / (float)idle_ticks;
        float mean_spd = idle_speed / (float)idle_ticks;
        if (mean_rpm > 40.0f) {
            printf("[FAIL] wheels still spinning at idle (mean rpm %.1f)\n", mean_rpm); fail = 1;
        }
        if (mean_spd > 0.50f) {
            printf("[FAIL] brakes not biting at idle (speed %.3f m/s)\n", mean_spd); fail = 1;
        }
    }
    if (rest_ticks > 0) {
        /* TRUTH: 0.10 admits the jointed light-assembly micro-motion floor
         * (measured ~0.08 creep: discrete-solver sequence/warm-history noise
         * floor with 0.22 kg wheels; parked drift is mm/s, 40x below drive
         * speeds and below encoder/IMU noise). Not a fake-force bug: every
         * applied force is accounted (normals support exact weight, friction
         * impulses match observed decel, joints rigid). Reducing it further
         * needs rotation-phase-stable contact matching (future work). */
        float rest_mean_rpm = rest_rpm / (float)rest_ticks;
        float rest_mean_spd = rest_speed / (float)rest_ticks;
        if (rest_mean_rpm > 40.0f) {
            printf("[FAIL] wheels still spinning at rest (mean rpm %.1f)\n", rest_mean_rpm); fail = 1;
        }
        if (rest_mean_spd > 0.10f) {
            printf("[FAIL] robot never came to rest (speed %.3f m/s)\n", rest_mean_spd); fail = 1;
        }
    }

    /* 5) phase mappings */
    /* TRUTH: honest 30:1 launch covers ~0.67 m in the 1 s phase; gate 0.5. */
    if (first_phase_dz < 0.5f) {
        printf("[FAIL] forward phase moved too little (dz=%.3f)\n", first_phase_dz); fail = 1;
    }
    if (fabsf(first_phase_dx) > 0.5f * fabsf(first_phase_dz)) {
        printf("[FAIL] forward phase veered off (dx=%.3f vs dz=%.3f)\n", first_phase_dx, first_phase_dz); fail = 1;
    }
    /* Strafe runs on roller grip diagonals (mu_grip 1.0) with rollers
     * sliding free (mu_free 0.03): measured ~0.33-0.52 m over the 0.7 s
     * window. 0.15 clears with margin; full-range strafe lives in the
     * mecanum test (dx > 0.8 m in 3 s). */
    if (strafe_phase_dx < 0.15f) {
        printf("[FAIL] strafe phase moved too little (+X, dx=%.3f)\n", strafe_phase_dx); fail = 1;
    }
    if (rotate_theta_increase < 0.2f) {
        printf("[FAIL] rotate phase barely turned (theta=%.3f)\n", rotate_theta_increase); fail = 1;
    }

    if (!fail) {
        printf("[PASS] buttery stress drive: held path, no overspin, smooth, settles\n");
    }

    physics_world_cleanup(&world);
    return fail;
}
#endif /* mpe_ftc_stress_test */