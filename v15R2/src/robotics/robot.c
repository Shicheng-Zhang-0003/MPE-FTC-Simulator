/* MPE_FTC_073: FTC robot object implementation */
/* MPE_FTC_094_CLEANUP: wheel_traction removed — real cylinder friction */
#include "robot.h"
#include "../physics/constraint.h"
#include <math.h>
#include <string.h>

/* Robot dimensions (metres, approximate FTC 18" x 18" chassis) */
#define CHASSIS_HALF_X 0.225f
#define CHASSIS_HALF_Y 0.075f
#define CHASSIS_HALF_Z 0.225f
#define CHASSIS_MASS 8.0f /* ~18 lb robot */
#define WHEEL_RADIUS 0.05f /* 100mm wheels */
#define WHEEL_MASS 0.2f
#define WHEEL_HALF_WIDTH 0.02f /* 40mm wide wheels */
#define WHEEL_OFFSET_X 0.24f /* slightly outside chassis */
#define WHEEL_OFFSET_Z 0.20f
#define WHEEL_Y_OFFSET (-CHASSIS_HALF_Y - WHEEL_RADIUS + 0.01f)

/* MPE_FTC_095: chassis-centre height where the wheels just touch floor y=0 */
float ftc_robot_rest_height(void) {
    return WHEEL_RADIUS - WHEEL_Y_OFFSET;
}

int ftc_robot_create(physics_world *world, ftc_robot *robot, float x, float y, float z, motor_preset_id preset) {
    if ((!world) || (!robot)) {
        return 1;
    }
    memset(robot, 0, sizeof(ftc_robot));
    robot->motor_preset = preset;
    robot->mecanum_active = false; /* MPE_FTC_082 */
    robot->axle_axis_x = 1.0f; /* axles point along X (left-right) */
    robot->axle_axis_y = 0.0f;
    robot->axle_axis_z = 0.0f;
    battery_init(&robot->battery);

    /* Chassis: a box at the given position */
    robot->chassis_body = physics_world_add_cube(
        world, (vector3){x, y, z}, (vector3){CHASSIS_HALF_X, CHASSIS_HALF_Y, CHASSIS_HALF_Z}, CHASSIS_MASS);
    if (robot->chassis_body < 0) {
        return 1;
    }

    uint32_t chassis_id = world->bodies[robot->chassis_body].object_id;

    /* 4 wheels at corners */
    float wheel_positions[4][3] = {
        {x - WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z - WHEEL_OFFSET_Z}, /* front-left */
        {x + WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z - WHEEL_OFFSET_Z}, /* front-right */
        {x - WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z + WHEEL_OFFSET_Z}, /* back-left */
        {x + WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z + WHEEL_OFFSET_Z}, /* back-right */
    };
    robot->wheel_count = 4;

    for (int i = 0; i < robot->wheel_count; i++) {
        /* Create wheel as a sphere (rolling approximation) */
        robot->wheel_bodies[i] =
            physics_world_add_cylinder(world, WHEEL_RADIUS, WHEEL_HALF_WIDTH, WHEEL_MASS,
                                     (vector3){wheel_positions[i][0], wheel_positions[i][1], wheel_positions[i][2]});
        if (robot->wheel_bodies[i] < 0) {
            return 1;
        }

        uint32_t wheel_id = world->bodies[robot->wheel_bodies[i]].object_id;

        /* Revolute joint: chassis (body_a) to wheel (body_b), axle along X */
        vector3 anchor_on_chassis = {wheel_positions[i][0] - x, WHEEL_Y_OFFSET, wheel_positions[i][2] - z};
        vector3 anchor_on_wheel = {0.0f, 0.0f, 0.0f}; /* wheel centre */
        vector3 axle_axis = {robot->axle_axis_x, robot->axle_axis_y, robot->axle_axis_z};

        robot->wheel_joints[i] =
            constraint_add_revolute(chassis_id, wheel_id, anchor_on_chassis, anchor_on_wheel, axle_axis);
        if (robot->wheel_joints[i] < 0) {
            return 1;
        }

        /* MFS_MECANUM_REAL: Mark wheel as mecanum with roller angle.
         * Standard layout: front-left +45°, front-right -45°, back-left -45°, back-right +45° */
        float roller_angle = 0.0f;
        if (i == 0) roller_angle = 0.785398f;       /* front-left: +45° */
        if (i == 1) roller_angle = -0.785398f;      /* front-right: -45° */
        if (i == 2) roller_angle = -0.785398f;      /* back-left: -45° */
        if (i == 3) roller_angle = 0.785398f;       /* back-right: +45° */
        
        rigidbody_set_mecanum(&world->bodies[robot->wheel_bodies[i]], true, roller_angle);

        /* Set up motor for this wheel */
        motor_preset_apply(&robot->wheel_motors[i], preset);
    }

    return 0;
}

void ftc_robot_update(physics_world *world, ftc_robot *robot, float dt) {
    if ((!world) || (!robot) || (dt <= 0.0f)) {
        return;
    }

    /* Sum currents for battery sag */
    float total_current = 0.0f;
    for (int i = 0; i < robot->wheel_count; i++) {
        total_current += fabsf(robot->wheel_motors[i].current);
    }
    float terminal_voltage = battery_get_voltage(&robot->battery, total_current);
    battery_drain(&robot->battery, total_current, dt);

    /* Update each wheel motor */
    for (int i = 0; i < robot->wheel_count; i++) {
        int wheel_idx = robot->wheel_bodies[i];
        if ((wheel_idx < 0) || (wheel_idx >= world->body_count)) {
            continue;
        }
        rigidbody *wheel = &world->bodies[wheel_idx];

        /* Read wheel angular velocity about the axle axis */
        vector3 axle = {robot->axle_axis_x, robot->axle_axis_y, robot->axle_axis_z};
        float wheel_speed = wheel->angular_velocity.x * axle.x + wheel->angular_velocity.y * axle.y +
                            wheel->angular_velocity.z * axle.z;

        /* Update motor electrical state */
        motor_update(&robot->wheel_motors[i], wheel_speed, dt, terminal_voltage);

        /* Apply motor torque to wheel body */
        float torque = robot->wheel_motors[i].output_torque;
        wheel->torque_accumulator.x += axle.x * torque;
        wheel->torque_accumulator.y += axle.y * torque;
        wheel->torque_accumulator.z += axle.z * torque;
        /* MPE_FTC_076: raycast traction augments contact friction */
        //Retired 076
        rigidbody_wake(wheel); /* MPE_FTC_078: keep driven wheels awake so motor torque is applied */
    }
}

void ftc_robot_set_wheel_commands(ftc_robot *robot, const float *commands, int count) {
    if (!robot) {
        return;
    }
    int n = (count < robot->wheel_count) ? count : robot->wheel_count;
    for (int i = 0; i < n; i++) {
        float cmd = commands[i];
        if (cmd > 1.0f) {
            cmd = 1.0f;
        }
        if (cmd < -1.0f) {
            cmd = -1.0f;
        }
        robot->wheel_motors[i].command = cmd;
    }
}

void ftc_robot_get_position(physics_world *world, ftc_robot *robot, float *px, float *py, float *pz) {
    if ((!world) || (!robot)) {
        return;
    }
    int idx = robot->chassis_body;
    if ((idx < 0) || (idx >= world->body_count)) {
        return;
    }
    if (px) {
        *px = world->bodies[idx].position.x;
    }
    if (py) {
        *py = world->bodies[idx].position.y;
    }
    if (pz) {
        *pz = world->bodies[idx].position.z;
    }
}
