/* MPE_FTC_071: FTC motor presets */
#ifndef motor_presets_h
#define motor_presets_h
#include "motor.h"

typedef enum {
    MOTOR_GB_5203_13_7, /* goBILDA Yellow Jacket 13.7:1 (435 RPM, 18.7 kg·cm) */
    MOTOR_GB_5203_19_2, /* goBILDA Yellow Jacket 19.2:1 (312 RPM, 24.3 kg·cm) */
    MOTOR_GB_5203_26_9, /* goBILDA Yellow Jacket 26.9:1 (223 RPM, 38 kg·cm) */
    MOTOR_GB_5203_50_9, /* goBILDA Yellow Jacket 50.9:1 (117 RPM, 68.4 kg·cm) */
    MOTOR_GB_5203_71_2, /* goBILDA Yellow Jacket 71.2:1 (84 RPM, 93.6 kg·cm) */
    MOTOR_REV_CORE_HEX, /* REV Core Hex Motor */
    MOTOR_COUNT
} motor_preset_id;

void motor_preset_apply(motor *m, motor_preset_id id);
const char *motor_preset_name(motor_preset_id id);

#endif /* motor_presets_h */
