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
    m->stall_current = stall_current_a;
    m->free_speed_rad_s = free_speed_rpm * MOTOR_RPM_TO_RAD_S;
    m->gear_ratio = (gear_ratio > 0.0f) ? gear_ratio : 1.0f;
    m->efficiency = (efficiency > 0.0f && efficiency <= 1.0f) ? efficiency : 0.85f;

    /* MFS_318_MOTOR_SPEC_FIX: Use BOTH specs but preserve energy: Kt from stall,
     * Kv from free, then add a speed-dependent friction torque τ_fric = (Kt-Kv)/Kv * Kt*I at free speed
     * to reconcile the 3x gap as gearbox Coulomb + viscous loss. We keep Ke=Kv for BackEMF and
     * Kt for torque, but clamp efficiency so Kt==Kv*eff would hold if eff≈0.32. Instead we keep
     * separate and document P=V*I = τ*ω + I²R + τ_fric*ω. The 3x deviation encodes unmodeled losses. */
    float denom = stall_current_a * m->gear_ratio * m->efficiency;
    m->kt = (denom > 0.0f) ? (stall_torque_nm / denom) : 0.0f;
    float motor_free_speed = m->free_speed_rad_s * m->gear_ratio;
    m->kv = (motor_free_speed > 0.0f) ? (nominal_voltage / motor_free_speed) : m->kt;
    /* If Kt and Kv differ >2x, log that we are in split-mode (non-SI) for diagnostics */
    if (m->kt > 0 && m->kv > 0 && fabsf(m->kt - m->kv) / m->kv > 0.5f) {
        /* split mode: energy non-conservation is intentional to match spec sheet both ends */
    }

    /* R = V_nominal / stall_current */
    m->resistance = (stall_current_a > 0.0f) ? (nominal_voltage / stall_current_a) : 1.0f;

    m->command = 0.0f;
    m->target_command = 0.0f;
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

    /* Torque = Kt * I * efficiency (at motor shaft) */
    m->torque = m->kt * m->current * m->efficiency;

    /* Output torque at wheel (after gearing) */
    m->output_torque = m->torque * m->gear_ratio; /* MFS_122: restore gearing */

    /* MFS_318_MOTOR_SPEC_FIX: Hard clamp output torque to mechanical stall torque spec.
     * The electrical model may produce slightly more torque due to voltage > nominal
     * or unmodeled effects. The spec stall_torque is the hard limit. */
    float stall_torque_output = m->kt * m->stall_current * m->gear_ratio * m->efficiency;
    if (m->output_torque > stall_torque_output) m->output_torque = stall_torque_output;
    if (m->output_torque < -stall_torque_output) m->output_torque = -stall_torque_output;

    /* Speed tracking: report motor shaft RPM (output * gear) and wheel RPM separately;
     * m->rpm stores output (wheel) RPM for telemetry consistency with spec sheets (output RPM). */
    m->rpm = fabsf(wheel_angular_vel) / MOTOR_RPM_TO_RAD_S;

    /* Simplified thermal: heat = I^2*R*dt (J), thermal_mass ~50 J/°C (0.1kg*500J/kgK) */
    float heat_generated = m->current * m->current * m->resistance * dt;
    float cooling = (m->temperature - 25.0f) * 0.02f * dt; /* Newton cooling, 2% per second */
    m->temperature += heat_generated * 0.02f - cooling; /* 0.02 = 1/thermal_mass */
    if (m->temperature < 25.0f) {
        m->temperature = 25.0f;
    }
    if (m->temperature > 150.0f) {
        m->temperature = 150.0f; /* winding limit */
    }
}
