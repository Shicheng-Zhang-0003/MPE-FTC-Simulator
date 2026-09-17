/* MPE_FTC_073: FTC robot object implementation */
/* MPE_FTC_094_CLEANUP: wheel_traction removed — real cylinder friction */
#include "robot.h"
#include "../physics/constraint.h"
#include "../config/mpe_config.h"
#include <math.h>
#include <string.h>

/* Robot dimensions (metres, approximate FTC 18" x 18" chassis) */
#define CHASSIS_HALF_X 0.225f
#define CHASSIS_HALF_Y 0.075f
#define CHASSIS_HALF_Z 0.225f
#define CHASSIS_MASS 8.92f /* chassis + 4 motors/gearboxes (~0.23 kg each); total 9.8 kg */
#define WHEEL_RADIUS 0.05f /* 100mm wheels (96/104mm class, parameterized later) */
#define WHEEL_MASS 0.22f /* bare wheel + rollers (sprung motor mass lives on chassis) */
#define WHEEL_HALF_WIDTH 0.02f /* 40mm wide wheels */
#define WHEEL_CLEARANCE_M 0.005f /* vertical gap wheel-top to chassis-bottom */
#define WHEEL_CLEARANCE_SIDE_M 0.010f /* lateral gap (tilt travel under load) */
/* NOTE: track is 0.51 m outboard of an 18 in (0.457 m) frame: wheels sit
 * outside the rails (gap above) rather than inboard like many real builds.
 * Modeling choice for joint clearance; narrows if inboard geometry lands. */
#define WHEEL_OFFSET_X (CHASSIS_HALF_X + WHEEL_HALF_WIDTH + WHEEL_CLEARANCE_SIDE_M)
#define WHEEL_OFFSET_Z 0.20f
/* Wheels clear the chassis: 5 mm vertical gap (was 10 mm overlap). */
#define WHEEL_Y_OFFSET (-CHASSIS_HALF_Y - WHEEL_RADIUS - WHEEL_CLEARANCE_M)

/* MPE_FTC_095: chassis-centre height where the wheels just touch floor y=0 */
float ftc_robot_rest_height(void) {
    return WHEEL_RADIUS - WHEEL_Y_OFFSET;
}

int ftc_robot_create_with_drive(physics_world *world, ftc_robot *robot, float x, float y, float z,
                                motor_preset_id preset, ftc_drivetrain_type drivetrain_type) {
/* MFS_161_NULL_FIX: null-check FIRST, before any dereference */
if ((!world) || (!robot)) {
return 1;
}
memset(robot, 0, sizeof(ftc_robot));
/* memset zeroes odom_x/z/theta and wheel_radians — no separate init needed */
/* MFS_302C_CAPACITY: Pre-check capacity before creating anything. */
if (world->body_count + 5 > world->body_capacity) {
return 1;
}
    robot->motor_preset = preset;
    robot->drivetrain_type = drivetrain_type;
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

    int chassis_body_snapshot = robot->chassis_body;
    int wheel_count_snapshot = 0;
    for (int i = 0; i < 4; i++) {
        /* Create wheel as cylinder (axle along X) */
        int added = physics_world_add_cylinder(world, WHEEL_RADIUS, WHEEL_HALF_WIDTH, WHEEL_MASS,
                                     (vector3){wheel_positions[i][0], wheel_positions[i][1], wheel_positions[i][2]});
        if (added < 0) {
/* MFS_302C_ROLLBACK: Properly rollback body_count and constraints instead of leaking ghosts. */
int saved_count = chassis_body_snapshot >=0 ? chassis_body_snapshot : 0;
world->body_count = saved_count;
for (int rb_i = 0; rb_i < i; rb_i++) {
if (robot->wheel_joints[rb_i] >= 0)
constraint_remove(world, robot->wheel_joints[rb_i]);
}
memset(robot, 0, sizeof(ftc_robot));
return 1;
        }
        robot->wheel_bodies[i] = added;
        wheel_count_snapshot++;

        uint32_t wheel_id = world->bodies[robot->wheel_bodies[i]].object_id;

        /* Revolute joint: chassis (body_a) to wheel (body_b), axle along X */
        vector3 anchor_on_chassis = {wheel_positions[i][0] - x, WHEEL_Y_OFFSET, wheel_positions[i][2] - z};
        vector3 anchor_on_wheel = {0.0f, 0.0f, 0.0f}; /* wheel centre */
        vector3 axle_axis = {robot->axle_axis_x, robot->axle_axis_y, robot->axle_axis_z};

        robot->wheel_joints[i] =
            constraint_add_revolute(world, chassis_id, wheel_id, anchor_on_chassis, anchor_on_wheel, axle_axis);
        if (robot->wheel_joints[i] < 0) {
/* MFS_302C_ROLLBACK: Rollback body_count and remove constraints. */
int saved_count = chassis_body_snapshot >=0 ? chassis_body_snapshot : 0;
world->body_count = saved_count;
for (int rb_i = 0; rb_i <= i; rb_i++) {
if (robot->wheel_joints[rb_i] >= 0)
constraint_remove(world, robot->wheel_joints[rb_i]);
}
memset(robot, 0, sizeof(ftc_robot));
return 1;
        }

        /* MFS_MECANUM_REAL: Mark wheel as mecanum with roller angle.
         * Standard FTC mecanum (axle along X, robot forward = +Z):
         *   FL (-X,+Z): roller at -45° from forward (+Z) = +45° from axle (+X)
         *   FR (+X,+Z): roller at +45° from forward = -45° from axle
         *   BL (-X,-Z): roller at +45° from forward = -45° from axle
         *   BR (+X,-Z): roller at -45° from forward = +45° from axle
         * Matches drivetrain_mecanum IK: positive strafe = +X world. */
        float roller_angle = 0.0f;
        if (i == 0) roller_angle = -0.78539816339f;      /* front-left: -45° from forward */
        if (i == 1) roller_angle = 0.78539816339f;       /* front-right: +45° from forward */
        if (i == 2) roller_angle = 0.78539816339f;       /* back-left: +45° from forward */
        if (i == 3) roller_angle = -0.78539816339f;      /* back-right: -45° from forward */
        
        if (robot->drivetrain_type == FTC_DRIVETRAIN_MECANUM) {
                rigidbody_set_mecanum(&world->bodies[robot->wheel_bodies[i]], true, roller_angle);
                /* Mecanum wheels need high friction on grip axis (perpendicular to rollers) */
                world->bodies[robot->wheel_bodies[i]].friction_static = 1.0f;
                world->bodies[robot->wheel_bodies[i]].friction_kinetic = 0.8f;
                /* TRUTH: rubber on foam is dead (e~=0). Default cylinder
                 * restitution (0.3, bouncy) chatters micro-impacts into a
                 * creep pump via Poisson pay on fresh contacts. */
                world->bodies[robot->wheel_bodies[i]].restitution = 0.0f;
            } else {
                rigidbody_set_mecanum(&world->bodies[robot->wheel_bodies[i]], false, 0.0f);
                world->bodies[robot->wheel_bodies[i]].restitution = 0.0f;
            }

        /* Set up motor for this wheel */
        motor_preset_apply(&robot->wheel_motors[i], preset);
        /* PHYS-FIX P0-2: wheel body keeps true geometric inertia only.
         * Old code baked chassis_mass/4*r^2 into the wheel rigidbody while
         * keeping full chassis mass -> ~1.93x double-counted inertia.
         * Chassis load arrives via contact/constraint impulses naturally.
         * effective_inertia (wheel + reflected rotor only) is kept for the
         * brake-torque clamp, never written into the rigidbody. */
        motor_compute_effective_inertia(&robot->wheel_motors[i], WHEEL_MASS, WHEEL_RADIUS, 0.0f);
    }

    return 0;
}


int ftc_robot_create(physics_world *world, ftc_robot *robot, float x, float y, float z, motor_preset_id preset) {
    return ftc_robot_create_with_drive(world, robot, x, y, z, preset, FTC_DRIVETRAIN_MECANUM);
}

/* TRUTH: motor electrical substeps. Explicit Euler on motor+wheel at 60 Hz
 * is unstable (electrical time constant ~1.6 ms at 71:1 vs 16.7 ms tick):
 * one tick of stall torque overshoots free speed 5x (bang-bang). Subcycling
 * the stiff motor dynamics at 16x (~1 ms, honest time-integration, not a
 * torque falsification) keeps it stable with true light-wheel inertia. */
#define FTC_MOTOR_SUBSTEPS 16

void ftc_robot_update(physics_world *world, ftc_robot *robot, float dt) {
    if ((!world) || (!robot) || (dt <= 0.0f)) {
        return;
    }

    /* Sum previous-tick currents for battery sag (explicit 1-tick lag). */
    float total_current = 0.0f;
    for (int i = 0; i < robot->wheel_count; i++) {
        total_current += fabsf(robot->wheel_motors[i].current);
    }
    /* Bus current foldback (controller limit, NOT a fuse: a real 20A breaker
     * trips open on I^2t; trip/reset state machine is future work). Scales
     * drive voltage share when the pack would exceed 20A so stall behaviour
     * collapses instead of producing impossible thrust. */
    float current_scale = 1.0f;
    const float FTC_BUS_LIMIT_A = 20.0f;
    if (total_current > FTC_BUS_LIMIT_A && total_current > 0.0f) {
        current_scale = FTC_BUS_LIMIT_A / total_current;
    }
    float terminal_voltage = battery_get_voltage(&robot->battery, total_current);
    battery_drain(&robot->battery, total_current, dt);

    /* Command ramp lives in the teleop layer (gui_robot_apply_drive); this
     * path uses command directly so autonomy / PID / characterization see
     * the true commanded voltage without 125 ms lag. */
    bool any_driven = false;
    for (int i = 0; i < robot->wheel_count; i++) {
        int wheel_idx = robot->wheel_bodies[i];
        if ((wheel_idx < 0) || (wheel_idx >= world->body_count)) {
            continue;
        }
        rigidbody *wheel = &world->bodies[wheel_idx];

        /* Read wheel angular velocity about the actual rotated axle axis in world space */
        vector3 axle = wheel->cached_axes[0];
        if (vector3_length_squared(axle) < 0.0001f) {
            axle = vector4_rotate_to_vector3(wheel->orientation, (vector3){1.0f, 0.0f, 0.0f});
        }
        float wheel_speed = vector3_dot(wheel->angular_velocity, axle);

        /* command already shaped by teleop layer; keep target in sync */
        robot->wheel_motors[i].command = robot->wheel_motors[i].target_command;
        bool driven = (fabsf(robot->wheel_motors[i].command) > 0.05f);
        if (driven) {
            any_driven = true;
        }

        /* True rigidbody axle inertia (integration uses rigidbody I). */
        float I_true = wheel->inertia_tensor_local.matrix[0][0];
        if (!(I_true > 1e-9f) || !isfinite(I_true)) I_true = robot->wheel_motors[i].effective_inertia;
        if (!(I_true > 1e-9f) || !isfinite(I_true)) I_true = 0.5f * WHEEL_MASS * WHEEL_RADIUS * WHEEL_RADIUS;

        /* Subcycled motor dynamics: advance local wheel speed under motor
         * torque + Coulomb gearbox drag, average the torque for the world
         * step. Contact/other torques act in the world integration (split). */
        float dt_sub = dt / (float) FTC_MOTOR_SUBSTEPS;
        float w_local = wheel_speed;
        float impulse = 0.0f;
        float drag = robot->wheel_motors[i].gearbox_drag;
        for (int k = 0; k < FTC_MOTOR_SUBSTEPS; k++) {
            motor_update(&robot->wheel_motors[i], w_local, dt_sub, terminal_voltage);
            float tq = robot->wheel_motors[i].output_torque * current_scale;
            /* Coulomb gearbox drag: opposes motion, holds stiction at rest
             * (no drift from exact zero), never reverses within a substep. */
            float tq_net;
            if (w_local > 0.0f) {
                tq_net = tq - drag;
                if (w_local + tq_net * dt_sub / I_true < 0.0f) {
                    tq_net = -w_local * I_true / dt_sub;
                }
            } else if (w_local < 0.0f) {
                tq_net = tq + drag;
                if (w_local + tq_net * dt_sub / I_true > 0.0f) {
                    tq_net = -w_local * I_true / dt_sub;
                }
            } else {
                if (fabsf(tq) <= drag) {
                    tq_net = 0.0f;
                } else {
                    tq_net = tq - ((tq > 0.0f) ? drag : -drag);
                }
            }
            impulse += tq_net * dt_sub;
            w_local += tq_net * dt_sub / I_true;
        }
        float torque_avg = impulse / dt;

        /* Motor torque on the wheel + equal-and-opposite stator reaction on
         * the chassis (Newton 3rd; old code torqued wheels only, forcing the
         * revolute to transmit the entire reaction as constraint error). */
        wheel->torque_accumulator = vector3_addition(
            wheel->torque_accumulator,
            vector3_scaling(axle, torque_avg));
        if (robot->chassis_body >= 0 && robot->chassis_body < world->body_count) {
            rigidbody *chassis = &world->bodies[robot->chassis_body];
            chassis->torque_accumulator = vector3_subtraction(
                chassis->torque_accumulator, vector3_scaling(axle, torque_avg));
        }
        /* Drive activity wakes (parked robots sleep honestly via error-gated
         * joint wakes; the old blanket chassis wake vetoed sleep always). */
        if (driven) {
            rigidbody_wake(wheel);
        }
    }
    if (any_driven && robot->chassis_body >= 0 && robot->chassis_body < world->body_count) {
        rigidbody_wake(&world->bodies[robot->chassis_body]);
    }
}

void ftc_robot_set_wheel_commands(ftc_robot *robot, const float *commands, int count) {
    if (!robot || !commands) {
        return;
    }
    /* PHYS-FIX: stale-motor guard. Old code left wheels 2..3 at previous
     * commands when count < wheel_count. Zero unspecified wheels. */
    for (int i = 0; i < robot->wheel_count && i < FTC_MAX_WHEELS; i++) {
        float cmd = 0.0f;
        if (i < count) {
            cmd = commands[i];
            if (cmd > 1.0f) cmd = 1.0f;
            if (cmd < -1.0f) cmd = -1.0f;
        }
        robot->wheel_motors[i].target_command = cmd;
        robot->wheel_motors[i].command = cmd;
    }
}

int ftc_ensure_field_walls(physics_world *world) {
    if (!world) return -1;
    /* FTC foam tiles grip hard; 475 defaults (0.2/0.1) are validation-ice.
     * Without this, mecanum grip clamps to 0.2 and strafe never breaks free. */
    g_cfg.world.floor_friction_s = 1.0f;
    g_cfg.world.floor_friction_k = 0.8f;
    return physics_world_add_boundary_walls(world, 1.8288f, 1.8288f, 0.5f, 0.0508f);
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
