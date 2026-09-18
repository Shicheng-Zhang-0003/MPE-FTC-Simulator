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

    /* TRUTH: both datasheet ends exact. Kt from stall (output stall =
     * Kt*Istall*G*eff); Ke from free speed (V = I0*R + Ke*G*w_free), so free
     * speed is exact at no-load current I0 instead of 17-20% high. The
     * Kt/Ke gap is the lumped brush+gearbox loss, modeled explicitly below
     * as Coulomb drag (not an energy hole): slope dT/dw matches datasheet
     * within ~3%. R = V/Istall. */
    float denom = stall_current_a * m->gear_ratio * m->efficiency;
    m->kt = (denom > 0.0f) ? (stall_torque_nm / denom) : 0.0f;
    m->no_load_current = 0.25f;
    if (m->no_load_current < 0.05f) m->no_load_current = 0.05f;
    if (m->no_load_current > 0.40f) m->no_load_current = 0.40f;
    m->resistance = (stall_current_a > 0.0f) ? (nominal_voltage / stall_current_a) : 1.0f;
    {
        float motor_free = m->free_speed_rad_s * m->gear_ratio;
        float ke = m->kt;
        if ((motor_free > 0.0f) && (stall_current_a > 0.0f)) {
            ke = (nominal_voltage - m->no_load_current * m->resistance) / motor_free;
            if (!(ke > 0.0f) || !isfinite(ke)) ke = m->kt;
        }
        m->kv = ke;
    }
    /* Coulomb gearbox drag at the output: Kt*I0*G*eff. Makes net output
     * exactly zero at free speed (all input goes to losses, as measured). */
    m->gearbox_drag = m->kt * m->no_load_current * m->gear_ratio * m->efficiency;
    if (!(m->gearbox_drag >= 0.0f) || !isfinite(m->gearbox_drag)) m->gearbox_drag = 0.0f;

    m->command = 0.0f;
    m->target_command = 0.0f;
    /* FIX-AUDIT: actuator slew (was snap 1e6 with the ramp mechanics dead:
     * full stick meant instant stall torque, which excited the joint pump
     * into liftoff/backflip on grippy floor). 12/s reaches full stick in
     * ~83 ms — actuator-realistic, and tests are unaffected (shortest
     * scripted phase is 500 ms). gui_robot_apply_drive keeps its own 8/s
     * stick shaping on top for the gamepad path. */
    m->command_ramp_per_s = 12.0f;
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

    /* Current = (V - BackEMF) / R, clamped to stall. At free speed this
     * yields exactly I0 (no deadband): free-running draws current honestly. */
    float raw_current = (applied_voltage - m->back_emf) / m->resistance;
    if (raw_current > m->stall_current) {
        raw_current = m->stall_current;
    }
    if (raw_current < -m->stall_current) {
        raw_current = -m->stall_current;
    }
    m->current = raw_current;

    /* Torque = Kt * I (motor shaft, SI); gearbox eff applied once at output.
     * This is the ELECTROMAGNETIC gross output; Coulomb gearbox drag
     * (m->gearbox_drag) is applied by the caller (ftc_robot_update), which
     * owns wheel inertia for a no-reverse clamp + static stiction. Net is
     * exactly zero at free speed by construction (see motor_from_spec). */
    m->torque = m->kt * m->current;
    m->output_torque = m->torque * m->gear_ratio * m->efficiency;

    /* Hard clamp to mechanical stall spec (over-voltage edge case). */
    float stall_torque_output = m->kt * m->stall_current * m->gear_ratio * m->efficiency;
    if (m->output_torque > stall_torque_output) m->output_torque = stall_torque_output;
    if (m->output_torque < -stall_torque_output) m->output_torque = -stall_torque_output;

    /* Speed tracking: signed output (wheel) RPM for telemetry consistency
     * with spec sheets (output RPM, not shaft RPM).
     * FIX-AUDIT: was fabsf (direction destroyed — telemetry could never show
     * reverse; odometry-adjacent tooling read it as unsigned). */
    m->rpm = wheel_angular_vel / MOTOR_RPM_TO_RAD_S;

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
