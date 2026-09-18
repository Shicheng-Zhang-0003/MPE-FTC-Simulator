/* MPE_FTC_073: FTC robot object */
#ifndef robot_h
#define robot_h

#include "motor.h"
#include "motor_presets.h"
#include "battery.h"
#include "../core/physics_world.h"

#define FTC_MAX_WHEELS 8

/* FIX-AUDIT: single-source build geometry (was scattered 0.48/0.05/3.3
 * literals drifting from the plant). Only the wheel radius is still read
 * in code (odometry fallback); track/arm remain as the documented build
 * geometry for tests and tooling. */
#define FTC_TRACK_WIDTH_M 0.517f /* 2 * (0.2285 + 0.02 + 0.01) */
#define FTC_MECANUM_ARM_M 0.4585f /* (0.2585 + 0.20): standard lx+lz yaw arm */
#define FTC_WHEEL_RADIUS_M 0.048f /* 96 mm goBILDA mecanum */

/* MFS_DRIVETRAIN_TYPE: explicit wheel/traction model selection. */
typedef enum {
    FTC_DRIVETRAIN_MECANUM = 0,
    FTC_DRIVETRAIN_TANK = 1
} ftc_drivetrain_type;

typedef struct {
    /* Body indices in physics_world */
    int chassis_body;
    int wheel_bodies[FTC_MAX_WHEELS];
    int wheel_joints[FTC_MAX_WHEELS]; /* revolute joint indices */
    int wheel_count;

    /* Motor + electrical */
    motor wheel_motors[FTC_MAX_WHEELS];
    motor_preset_id motor_preset;
    battery battery;

    /* Axle direction in chassis-local space (for reading wheel speed) */
    float axle_axis_x, axle_axis_y, axle_axis_z;
    /* TRUTH-MESH: chassis-force cheat fields REMOVED (were one-shot strafe/
     * rotate forces + raw command carries). Lateral/yaw authority now
     * arrives through wheel torques on the anisotropic roller frame, so
     * nothing outside the contact solver pushes the chassis. */
    /* FIX-AUDIT: per-robot teleop shaping state. The stick smoother used to
     * be process-global singletons shared across all robots. */
    float smooth_fwd, smooth_strafe, smooth_rot;
    /* MFS_162_DEAD_FIELD: mecanum_active removed */
    ftc_drivetrain_type drivetrain_type; /* MFS_DRIVETRAIN_TYPE */

    /* MFS_151_ODOMETRY: Wheel encoders and pose estimation */
    float wheel_radians[FTC_MAX_WHEELS]; /* MFS_163_BOUNDS_FIX: was [4], OOB if wheel_count > 4 */
    float odom_x, odom_z, odom_theta;
} ftc_robot;

/* Create a 4-wheel robot at the given position. Returns 0 on success. */
/* MPE_FTC_095: chassis-centre height where wheels rest on the floor */
float ftc_robot_rest_height(void);
int ftc_robot_create_with_drive(physics_world *world, ftc_robot *robot, float x, float y, float z,
                                motor_preset_id preset, ftc_drivetrain_type drivetrain_type);

int ftc_robot_create(physics_world *world, ftc_robot *robot, float x, float y, float z, motor_preset_id preset);

/* Update all motors for one tick. Reads wheel angular velocity,
   computes electrical state, applies torque to wheel bodies. */
void ftc_robot_update(physics_world *world, ftc_robot *robot, float dt);

/* Set wheel motor commands (-1..1). */
void ftc_robot_set_wheel_commands(ftc_robot *robot, const float *commands, int count);

/* Get the chassis body's position (for validation). */
void ftc_robot_get_position(physics_world *world, ftc_robot *robot, float *px, float *py, float *pz);

/* Ensure 12ft FTC field walls exist (idempotent-ish; adds 4 static walls). */
int ftc_ensure_field_walls(physics_world *world);

#endif /* robot_h */
