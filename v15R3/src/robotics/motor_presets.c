/* MPE_FTC_071: FTC motor presets — goBILDA 5203 Yellow Jacket series.
 * All specs verified against official goBILDA product pages (2025).
 * Common: 12V nominal, 9.2A stall (all geared), 0.25A no-load.
 * Stall torque from kg·cm → N·m: multiply by 0.0980665. */
#include "motor_presets.h"

typedef struct {
    motor_preset_id id;
    const char *name;
    float stall_torque;   /* N·m */
    float free_speed_rpm; /* RPM @ 12V */
    float stall_current;  /* A @ 12V */
    float gear_ratio;
    float efficiency;
} motor_preset_spec;

static const motor_preset_spec presets[MOTOR_COUNT] = {
    /* 13.7:1 — 435 RPM, 18.7 kg·cm = 1.835 N·m */
    {MOTOR_GB_5203_13_7, "goBILDA 5203 13.7:1", 1.835f, 435.0f, 9.2f, 13.7f, 0.85f},
    /* 19.2:1 — 312 RPM, 24.3 kg·cm = 2.384 N·m */
    {MOTOR_GB_5203_19_2, "goBILDA 5203 19.2:1", 2.384f, 312.0f, 9.2f, 19.2f, 0.85f},
    /* 26.9:1 — 223 RPM, 38 kg·cm = 3.728 N·m */
    {MOTOR_GB_5203_26_9, "goBILDA 5203 26.9:1", 3.728f, 223.0f, 9.2f, 26.9f, 0.85f},
    /* 50.9:1 — 117 RPM, 68.4 kg·cm = 6.710 N·m */
    {MOTOR_GB_5203_50_9, "goBILDA 5203 50.9:1", 6.710f, 117.0f, 9.2f, 50.9f, 0.85f},
    /* 71.2:1 — 84 RPM, 93.6 kg·cm = 9.182 N·m */
    {MOTOR_GB_5203_71_2, "goBILDA 5203 71.2:1", 9.182f, 84.0f, 9.2f, 71.2f, 0.85f},
    /* REV Core Hex — 125 RPM, 3.2 N·m, 4.4A (REV-41-1300, 72:1) */
    {MOTOR_REV_CORE_HEX, "REV Core Hex REV-41-1300", 3.20f, 125.0f, 4.4f, 72.0f, 0.80f},
};

void motor_preset_apply(motor *m, motor_preset_id id) {
    if ((!m) || (id < 0) || (id >= MOTOR_COUNT)) {
        return;
    }
    const motor_preset_spec *spec = &presets[id];
    motor_from_spec(m, spec->stall_torque, spec->free_speed_rpm, spec->stall_current, 12.0f, spec->gear_ratio,
                    spec->efficiency);
}

const char *motor_preset_name(motor_preset_id id) {
    if ((id < 0) || (id >= MOTOR_COUNT)) {
        return "unknown";
    }
    return presets[id].name;
}
