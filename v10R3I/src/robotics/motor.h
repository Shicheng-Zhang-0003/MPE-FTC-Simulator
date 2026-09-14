/* MPE_FTC_070: DC motor electrical model */
#ifndef motor_h
#define motor_h

typedef struct {
    /* Electrical (derive from spec sheet: stall_torque, free_speed, stall_current) */
    float resistance; /* ohms */
    float kt; /* N·m/A torque constant */
    float kv; /* V/(rad/s) back-EMF constant */
    float stall_current; /* A */
    float free_speed_rad_s; /* rad/s at no load */

    /* Mechanical */
    float gear_ratio; /* output/input */
    float efficiency; /* 0..1 */

    /* MFS_314_DRIVE_RAMP: input shaping. The controller writes target_command;
     * the applied command ramps toward it, so stick snaps become smooth
     * acceleration — "buttery" on a controller rather than wheel-slam. */
    float target_command;      /* -1..1 requested by the controller each tick */
    float command_ramp_per_s;  /* how fast command can change (units/s) */
    float command;             /* -1..1 applied (ramped toward target) */
    float current; /* A (computed each tick) */
    float back_emf; /* V (computed each tick) */
    float torque; /* N·m at motor shaft */
    float output_torque; /* N·m at wheel after gearing */
    float rpm; /* current output speed */
    float temperature; /* simplified thermal model */

    /* MFS_313_TORQUE_STABILITY: effective inertia seen by the motor at the
     * wheel (kg·m²) = wheel roll inertia + rotor inertia·G² + the driven
     * chassis load reflected through the contact patch (m_share·r²). Without
     * the load term, full stall torque on a 2.5e-4 kg·m² wheel accelerates it
     * past free speed within one step, so the back-EMF flips sign and the
     * wheel bangs between +free and -free (the ±steering/slam instability).
     * NOTE: this is used for MOTION (integration) but is NOT odometry truth. */
    float effective_inertia;
} motor;

/* Derive motor params from the four spec-sheet numbers. */
void motor_from_spec(motor *m, float stall_torque_nm, float free_speed_rpm, float stall_current_a,
                     float nominal_voltage, float gear_ratio, float efficiency);

/* MFS_313_TORQUE_STABILITY: compute effective_inertia (kg·m²) so one step of
 * stall torque stays below free speed (no bang-bang). wheel_mass + radius are
 * the wheel cylinder; driven_mass is the chassis load THIS wheel accelerates
 * (e.g. total robot mass / 4 for a symmetric skid-steer or mecanum robot). */
void motor_compute_effective_inertia(motor *m, float wheel_mass, float wheel_radius, float driven_mass);

/* Advance one tick. wheel_angular_vel = output shaft speed (rad/s). */
void motor_update(motor *m, float wheel_angular_vel, float dt, float battery_voltage);

#endif /* motor_h */
