/* MPE_FTC_072: Battery model implementation */
#include "battery.h"

void battery_init(battery *b) {
    if (!b) {
        return;
    }
    b->nominal_voltage = 12.8f;
    b->internal_resistance = 0.05f;
    b->capacity_ah = 3.0f;
    b->charge_fraction = 1.0f;
}

float battery_get_voltage(const battery *b, float total_current_draw) {
    if (!b) {
        return 12.8f;
    }
    float open_circuit = b->nominal_voltage * b->charge_fraction;
    float sag = b->internal_resistance * total_current_draw;
    float terminal = open_circuit - sag;
    if (terminal < 0.0f) {
        terminal = 0.0f;
    }
    return terminal;
}

void battery_drain(battery *b, float total_current_draw, float dt) {
    if ((!b) || (dt <= 0.0f)) {
        return;
    }
    float amp_hours_used = (total_current_draw * dt) / 3600.0f;
    b->charge_fraction -= amp_hours_used / b->capacity_ah;
    if (b->charge_fraction < 0.0f) {
        b->charge_fraction = 0.0f;
    }
}
