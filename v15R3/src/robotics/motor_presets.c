/* MPE_FTC_071: FTC motor presets (goBILDA 5203 Yellow Jacket, 12V nominal).
 * PHYS-FIX: old table (1.63Nm/340RPM/17A @12.8V) was -32% torque, +9% speed,
 * +85% current vs spec. Anchored on 19.2:1 datasheet (24.3 kg.cm=2.38Nm,
 * 312RPM, 9.2A stall); siblings scale by ratio (torque~G, speed~1/G). */
#include "motor_presets.h"

typedef struct {
    motor_preset_id id;
    const char *name;
    float stall_torque;
    float free_speed_rpm;
    float stall_current;
    float gear_ratio;
    float efficiency;
} motor_preset_spec;

static const motor_preset_spec presets[MOTOR_COUNT] = {
    {MOTOR_GB_5203_19_2, "goBILDA 5203 19.2:1", 2.38f, 312.0f, 9.2f, 19.2f, 0.85f},
    {MOTOR_GB_5203_30, "goBILDA 5203 30:1", 3.72f, 200.0f, 9.2f, 30.0f, 0.85f},
    {MOTOR_GB_5203_43_7, "goBILDA 5203 43.7:1", 5.42f, 137.0f, 9.2f, 43.7f, 0.85f},
    {MOTOR_GB_5203_71, "goBILDA 5203 71.2:1", 8.83f, 84.0f, 9.2f, 71.2f, 0.85f},
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
