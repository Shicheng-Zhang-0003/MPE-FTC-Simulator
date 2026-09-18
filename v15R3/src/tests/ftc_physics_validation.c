/* FTC Physics Validation Test — validates mecanum drivetrain physics.
 * Tests traction, friction, wheel-lock, and chassis damping.
 * Built via `make test_ftc_physics_validation`.
 */
#ifdef mpe_ftc_physics_validation_test
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "robotics/robot.h"
#include "robotics/drivetrain.h"
#include "robotics/motor_presets.h"

#define FTC_DT (1.0f / 60.0f)
#define FTC_TICKS 600
#define FTC_TRANSIENT 60

int main(void) {
    mpe_config_init();
    printf("\n=== FTC PHYSICS VALIDATION ===\n");

    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(&world);

    /* Create FTC robot with mecanum drive */
    ftc_robot robot;
    memset(&robot, 0, sizeof(ftc_robot));
    ftc_robot_create(&world, &robot, 0.0f, 0.0f, 0.0f, MOTOR_GB_5203_19_2);

    printf("Robot created: %d wheels, chassis=%d\n", robot.wheel_count, robot.chassis_body);
    printf("Floor friction: s=%.3f k=%.3f\n", g_cfg.world.floor_friction_s, g_cfg.world.floor_friction_k);
    printf("Roller friction: %.3f\n", g_cfg.solver.roller_friction_coeff);
    printf("Wheel-lock thresh: %.3f\n", g_cfg.solver.wheel_lock_omega_thresh);

    /* Test 1: Forward drive - validate traction and no NaN */
    printf("\n--- Test 1: Forward drive (300 ticks) ---\n");
    float commands[4] = {0.5f, 0.5f, 0.5f, 0.5f};
    ftc_robot_set_wheel_commands(&robot, commands, 4);

    int nan_count = 0, slip_count = 0;
    float max_speed = 0.0f;
    for (int t = 0; t < 300; t++) {
        ftc_robot_update(&world, &robot, FTC_DT);
        physics_world_step(&world, FTC_DT);

        /* Check for NaN */
        if (!isfinite(world.bodies[robot.chassis_body].position.x) ||
            !isfinite(world.bodies[robot.chassis_body].position.y) ||
            !isfinite(world.bodies[robot.chassis_body].position.z)) {
            nan_count++;
        }

        /* Track max speed */
        vector3 vel = world.bodies[robot.chassis_body].velocity;
        float speed = sqrtf(vel.x * vel.x + vel.z * vel.z);
        if (speed > max_speed) max_speed = speed;

        /* Check for slip (wheel angular velocity >> expected) */
        for (int i = 0; i < robot.wheel_count; i++) {
            int wi = robot.wheel_bodies[i];
            if (wi >= 0 && wi < world.body_count) {
                vector3 axle = world.bodies[wi].cached_axes[0];
                float omega = fabsf(vector3_dot(world.bodies[wi].angular_velocity, axle));
                if (omega > 50.0f) slip_count++;
            }
        }
    }

    printf("  NaN ticks: %d / 300\n", nan_count);
    printf("  Max speed: %.3f m/s\n", max_speed);
    printf("  Slip ticks: %d / 300\n", slip_count);

    /* Test 2: Idle hold - validate chassis stops */
    printf("\n--- Test 2: Idle hold (120 ticks) ---\n");
    memset(commands, 0, sizeof(commands));
    ftc_robot_set_wheel_commands(&robot, commands, 4);

    for (int t = 0; t < 120; t++) {
        ftc_robot_update(&world, &robot, FTC_DT);
        physics_world_step(&world, FTC_DT);
    }

    vector3 final_vel = world.bodies[robot.chassis_body].velocity;
    float final_speed = sqrtf(final_vel.x * final_vel.x + final_vel.z * final_vel.z);
    printf("  Final speed: %.6f m/s (should be ~0)\n", final_speed);

    /* Test 3: Strafe drive - validate lateral traction */
    printf("\n--- Test 3: Strafe drive (300 ticks) ---\n");
    float strafe_cmds[4] = {0.5f, -0.5f, -0.5f, 0.5f};
    ftc_robot_set_wheel_commands(&robot, strafe_cmds, 4);

    float max_strafe = 0.0f;
    for (int t = 0; t < 300; t++) {
        ftc_robot_update(&world, &robot, FTC_DT);
        physics_world_step(&world, FTC_DT);

        vector3 vel = world.bodies[robot.chassis_body].velocity;
        float speed = sqrtf(vel.x * vel.x + vel.z * vel.z);
        if (speed > max_strafe) max_strafe = speed;
    }

    printf("  Max strafe speed: %.3f m/s\n", max_strafe);

    /* Verdict */
    int pass = 1;
    if (nan_count > 0) { printf("FAIL: NaN detected\n"); pass = 0; }
    if (slip_count > 10) { printf("FAIL: Excessive wheel slip\n"); pass = 0; }
    if (max_speed < 0.1f) { printf("FAIL: Robot didn't move forward\n"); pass = 0; }
    if (final_speed > 0.5f) { printf("FAIL: Robot didn't stop\n"); pass = 0; }
    if (max_strafe < 0.01f) { printf("FAIL: Robot didn't strafe\n"); pass = 0; }

    printf("\n=== RESULT: %s ===\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
#endif
