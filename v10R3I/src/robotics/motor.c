/* MPE_FTC_070: DC motor electrical model implementation */
#include "motor.h"
#include <math.h>

#define MOTOR_RPM_TO_RAD_S 0.10472f /* 2*pi/60 */
/* MFS_313_TORQUE_STABILITY: bare rotor inertia of a 5203-class 60mm motor is
 * ~0.2 g·cm² = 2e-7 kg·m²; reflected through the gearbox it is I_rotor·G². */
#define MOTOR_ROTOR_INERTIA_KGM2 2.0e-7f

void motor_from_spec(motor *m, float stall_torque_nm, float free_speed_rpm, float stall_current_a,
                     float nominal_voltage, float gear_ratio, float efficiency) {
    if (!m) {
        return;
    }
    (void)stall_torque_nm; /* MFS_MOTOR_KTVK_FIX: Kt derived from Kv, spec torque unused */
    m->stall_current = stall_current_a;
    m->free_speed_rad_s = free_speed_rpm * MOTOR_RPM_TO_RAD_S;
    m->gear_ratio = (gear_ratio > 0.0f) ? gear_ratio : 1.0f;
    m->efficiency = (efficiency > 0.0f && efficiency <= 1.0f) ? efficiency : 0.85f;

    /* Back-EMF constant. At free speed, current ~ 0, so BackEMF ~ V_nominal.
     * m->kv = ke = V / omega_motor_shaft_free  (V·s/rad). It is the back-EMF
     * constant of the MOTOR SHAFT (omega already includes the gearbox). */
    float motor_free_speed = m->free_speed_rad_s * m->gear_ratio;
    m->kv = (motor_free_speed > 0.0f) ? (nominal_voltage / motor_free_speed) : 0.0f;

    /* MFS_309_MOTOR_KT_FIX: In SI, the torque constant Kt (N·m/A) is numerically
     * equal to the back-EMF constant ke (V·s/rad). The old code computed
     * kt = 1/kv, treating kv as a radiating speed constant (rad/s/V). That was
     * wrong by ~1/kv² (~2900x) and, combined with the gear ratio applied again
     * in motor_update, handed the wheels ~23,000 N·m at stall — guaranteeing
     * wheelspin and killing chassis motion. The spec stall_torque is unused; the
     * electrical specs (V, omega_free) are self-consistent and energy-conserving. */
    m->kt = m->kv;

    /* R = V_nominal / stall_current */
    m->resistance = (stall_current_a > 0.0f) ? (nominal_voltage / stall_current_a) : 1.0f;

    m->command = 0.0f;
    m->target_command = 0.0f;
    /* MFS_314_DRIVE_RAMP: full stick deflection (0->1) in ~125 ms feels
     * planted but responsive — the "buttery" compromise between a dead
     * laggy car and the old instant crack-open. */
    m->command_ramp_per_s = 8.0f;
    m->effective_inertia = 0.0f;
    m->current = 0.0f;
    m->back_emf = 0.0f;
    m->torque = 0.0f;
    m->output_torque = 0.0f;
    m->rpm = 0.0f;
    m->temperature = 25.0f;
}

void motor_compute_effective_inertia(motor *m, float wheel_mass, float wheel_radius, float driven_mass) {
    if (!m) {
        return;
    }
    float r = (wheel_radius > 0.0f) ? wheel_radius : 0.05f;
    float i_wheel = (wheel_mass > 0.0f) ? (0.5f * wheel_mass * r * r) : 0.0f;
    float i_rotor_reflected = MOTOR_ROTOR_INERTIA_KGM2 * m->gear_ratio * m->gear_ratio;
    float i_load = (driven_mass > 0.0f) ? (driven_mass * r * r) : 0.0f;
    m->effective_inertia = i_wheel + i_rotor_reflected + i_load;
}

void motor_update(motor *m, float wheel_angular_vel, float dt, float battery_voltage) {
    if ((!m) || (dt <= 0.0f)) {
        return;
    }

    float motor_shaft_vel = wheel_angular_vel * m->gear_ratio;

    /* BackEMF opposes applied voltage */
    m->back_emf = m->kv * motor_shaft_vel;

    /* Applied voltage from command */
    float applied_voltage = battery_voltage * m->command;

    /* Current = (V - BackEMF) / R, clamped to stall */
    float raw_current = (applied_voltage - m->back_emf) / m->resistance;
    if (raw_current > m->stall_current) {
        raw_current = m->stall_current;
    }
    if (raw_current < -m->stall_current) {
        raw_current = -m->stall_current;
    }
    m->current = raw_current;

    /* Torque = Kt * I * efficiency */
    m->torque = m->kt * m->current * m->efficiency;

    /* Output torque at wheel (after gearing) */
    m->output_torque = m->torque * m->gear_ratio; /* MFS_122: restore gearing */

    /* Speed tracking */
    m->rpm = fabsf(wheel_angular_vel) / MOTOR_RPM_TO_RAD_S;

    /* Simplified thermal: heat from I^2*R, cooling to ambient */
    float heat_generated = m->current * m->current * m->resistance * dt;
    float cooling = (m->temperature - 25.0f) * 0.01f * dt;
    m->temperature += heat_generated * 0.1f - cooling;
    if (m->temperature < 25.0f) {
        m->temperature = 25.0f;
    }
}
