/* MPE_FTC_073: FTC robot object implementation */
/* MPE_FTC_094_CLEANUP: wheel_traction removed — real cylinder friction */
#include "robot.h"
#include "../physics/constraint.h"
#include "../config/mpe_config.h"
#include <math.h>
#include <string.h>

/* Robot dimensions — FTC 18"×18" chassis (0.457m × 0.457m) */
#define CHASSIS_HALF_X 0.2285f  /* 18" / 2 = 9" = 0.2286m */
#define CHASSIS_HALF_Y 0.040f   /* ~3" tall plate; realistic for 1/8" aluminium + belly pan */
#define CHASSIS_HALF_Z 0.2285f  /* square chassis */
#define CHASSIS_MASS 5.2f       /* bare drivetrain + battery + hubs: total ~6.0 kg
                                 * (~13 lb competition weight with 4x207 g wheels).
                                 * FIX-MESH: was 2.5 kg (3.3 total) — too light to
                                 * drive like a true FTC bot (twitchy, easily
                                 * excited joints). Heavier plant + same torque =
                                 * calmer launch, planted cruise, honest traction
                                 * loads. All mass-derived sizing (traction cap,
                                 * cheat, drag, idle hold) reads live mass. */
#define WHEEL_RADIUS 0.048f     /* 96mm goBILDA mecanum (official SKU 3213-3606-0002) */
#define WHEEL_MASS 0.207f       /* 207g per official goBILDA spec */
#define WHEEL_HALF_WIDTH 0.02f  /* 40mm wide (20mm half-width) */
#define WHEEL_CLEARANCE_M 0.005f /* vertical gap wheel-top to chassis-bottom */
#define WHEEL_CLEARANCE_SIDE_M 0.010f /* lateral gap (tilt travel under load) */
/* 18"×18" frame: wheels sit just outside the rails for clearance. */
#define WHEEL_OFFSET_X (CHASSIS_HALF_X + WHEEL_HALF_WIDTH + WHEEL_CLEARANCE_SIDE_M)
#define WHEEL_OFFSET_Z 0.20f
/* Wheels clear the chassis: 5 mm vertical gap (was 10 mm overlap). */
#define WHEEL_Y_OFFSET (-CHASSIS_HALF_Y - WHEEL_RADIUS - WHEEL_CLEARANCE_M)

/* MPE_FTC_095: chassis-centre height where the wheels just touch floor y=0 */
float ftc_robot_rest_height(void) {
    return WHEEL_RADIUS - WHEEL_Y_OFFSET;
}

/* FIX-AUDIT: index slots where 0 is a valid body/joint must never read 0
 * for "empty" after memset. Central reset keeps the sentinel invariant
 * (wheel_joints/bodies/chassis = -1) at every init and failure exit. */
static void ftc_robot_invalidate(ftc_robot *robot) {
    memset(robot, 0, sizeof(ftc_robot));
    for (int s_i = 0; s_i < FTC_MAX_WHEELS; s_i++) {
        robot->wheel_joints[s_i] = -1;
        robot->wheel_bodies[s_i] = -1;
    }
    robot->chassis_body = -1;
}

int ftc_robot_create_with_drive(physics_world *world, ftc_robot *robot, float x, float y, float z,
                                motor_preset_id preset, ftc_drivetrain_type drivetrain_type) {
/* MFS_161_NULL_FIX: null-check FIRST, before any dereference */
if ((!world) || (!robot)) {
return 1;
}
/* FIX-AUDIT: joint/body slots are indices where 0 is VALID. memset leaves
 * 0, so a mid-spawn failure rollback (`>= 0`) used to remove constraint 0
 * (an unrelated joint). ftc_robot_invalidate zeroes scalars (odom, radians)
 * and sentinels the slots to -1. */
ftc_robot_invalidate(robot);
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

    /* 4 wheels at corners.
     * FIX-AUDIT: front is +Z (was -Z with a "+Z forward" comment, so
     * full stick drove the robot back-first and stick-up meant reverse).
     * Indices stay [0]=FL,[1]=FR,[2]=BL,[3]=BR; roller X-pattern per index
     * is unchanged, only the Z side moved. */
    float wheel_positions[4][3] = {
        {x - WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z + WHEEL_OFFSET_Z}, /* front-left */
        {x + WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z + WHEEL_OFFSET_Z}, /* front-right */
        {x - WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z - WHEEL_OFFSET_Z}, /* back-left */
        {x + WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z - WHEEL_OFFSET_Z}, /* back-right */
    };
    robot->wheel_count = 4;

    int chassis_body_snapshot = robot->chassis_body;
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
ftc_robot_invalidate(robot);
return 1;
        }
        robot->wheel_bodies[i] = added;

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
ftc_robot_invalidate(robot);
return 1;
        }

/* MFS_MECANUM_REAL: Mark wheel as mecanum with roller angle.
          * Standard FTC mecanum (axle along X, robot forward = +Z):
          *   FL (-X,+Z): roller at +45° from forward (+Z) = +45° from axle (+X)
          *   FR (+X,+Z): roller at -45° from forward = -45° from axle
          *   BL (-X,-Z): roller at -45° from forward = -45° from axle
          *   BR (+X,-Z): roller at +45° from forward = +45° from axle
          * Matches drivetrain_mecanum IK: positive strafe = +X world. */
         float roller_angle = 0.0f;
         if (i == 0) roller_angle = 0.78539816339f;      /* front-left: +45° from forward */
         if (i == 1) roller_angle = -0.78539816339f;       /* front-right: -45° from forward */
         if (i == 2) roller_angle = -0.78539816339f;       /* back-left: -45° from forward */
         if (i == 3) roller_angle = 0.78539816339f;      /* back-right: +45° from forward */
        
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
    /* Reset drive tracking for this tick; drivetrain_update sets
     * driven_this_tick=true on wheels with active commands. */
    for (int i = 0; i < robot->wheel_count; i++) {
        int wi = robot->wheel_bodies[i];
        if ((wi >= 0) && (wi < world->body_count)) {
            world->bodies[wi].driven_this_tick = false;
        }
    }

    /* Sum previous-tick currents for battery sag (explicit 1-tick lag).
     * FIX-AUDIT: two sums. Foldback and terminal-voltage sag key on |I|
     * (a braking motor still loads the bus); coulomb drain keys on signed
     * I so regenerative braking credits charge instead of draining. The old
     * fabsf single sum counted braking as drain. */
    float abs_current = 0.0f;
    float signed_current = 0.0f;
    for (int i = 0; i < robot->wheel_count; i++) {
        abs_current += fabsf(robot->wheel_motors[i].current);
        signed_current += robot->wheel_motors[i].current;
    }
    /* Bus current foldback (controller limit, NOT a fuse: a real 30A
     * controller limit scales back drive; trip/reset state machine is
     * future work). Scales drive torque share when the pack would exceed
     * 30A so stall behaviour collapses instead of producing impossible
     * thrust. */
    float current_scale = 1.0f;
    const float FTC_BUS_LIMIT_A = 30.0f;
    if (abs_current > FTC_BUS_LIMIT_A && abs_current > 0.0f) {
        current_scale = FTC_BUS_LIMIT_A / abs_current;
    }
    float terminal_voltage = battery_get_voltage(&robot->battery, abs_current);
    battery_drain(&robot->battery, signed_current, dt);

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

        /* FIX-AUDIT: honor the motor ramp. command slews toward
         * target_command at command_ramp_per_s (teleop shaping lives in
         * gui_robot_apply_drive; this is the actuator-level limit so direct
         * API users get smooth steps too). Default 1e6 = snap, so existing
         * autonomy/tests are bit-identical. */
        {
            float ramp = robot->wheel_motors[i].command_ramp_per_s;
            float cur = robot->wheel_motors[i].command;
            float tgt = robot->wheel_motors[i].target_command;
            if (!(ramp > 0.0f) || !isfinite(ramp)) {
                ramp = 1e6f;
            }
            float max_step = ramp * dt;
            float d = tgt - cur;
            if (d > max_step) {
                d = max_step;
            } else if (d < -max_step) {
                d = -max_step;
            }
            robot->wheel_motors[i].command = cur + d;
        }
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
        /* FIX-AUDIT: publish the APPLIED average, not the last substep's
         * instantaneous value. The 16x subcycle spins its local copy far
         * past the world wheel speed on light wheels, so the last substep
         * reads cruise (~0.07 N·m) while the world received mostly stall
         * (~2+ N·m early substeps). Traction (next stage) and telemetry
         * both consumed the misleading instantaneous value. */
        robot->wheel_motors[i].output_torque = torque_avg;

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
        if (driven) {
            wheel->driven_this_tick = true;
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
    /* FIX-AUDIT: write the TARGET only; command slews toward it in
     * ftc_robot_update() at command_ramp_per_s. The old double-write
     * (target AND command) snap-applied every call, which kept the ramp
     * permanently dead and made full stick an instant stall-torque shock
     * (joint-pump liftoff/backflip on grippy floor). Constant commands
     * converge in ~83 ms at the 12/s default, so scripted tests are
     * unaffected. */
    for (int i = 0; i < robot->wheel_count && i < FTC_MAX_WHEELS; i++) {
        float cmd = 0.0f;
        if (i < count) {
            cmd = commands[i];
            if (cmd > 1.0f) cmd = 1.0f;
            if (cmd < -1.0f) cmd = -1.0f;
        }
        robot->wheel_motors[i].target_command = cmd;
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
