/* MPE_FTC_072: Battery model implementation */
#include "battery.h"
#include <math.h>

/* PHYS-FIX: real FTC pack is 12V NiMH 3000mAh, 20A fuse — not 12.8V LiPo.
 * Old model hid empty (9V floor) and stretched life 4x (0.25 factor). */
void battery_init(battery *b) {
    if (!b) {
        return;
    }
    b->nominal_voltage = 12.0f;
    b->internal_resistance = 0.030f; /* NiMH + wiring, higher than LiPo */
    b->capacity_ah = 3.0f;
    b->charge_fraction = 1.0f;
}

float battery_get_voltage(const battery *b, float total_current_draw) {
    if (!b) {
        return 12.0f;
    }
    /* NiMH: flatter mid, steeper knee. >20%: 11.4-12.0; <20%: fall to 9.0. */
    float effective_charge = b->charge_fraction;
    float open_circuit;
    if (effective_charge > 0.2f) {
        open_circuit = 11.4f + 0.6f * (effective_charge - 0.2f) / 0.8f;
    } else {
        open_circuit = 9.0f + effective_charge * 12.0f; /* 9.0-11.4 at 0.2 */
    }
    /* TRUTH: single resistance model (V = OCV - I*R). Over-current foldback
     * lives once in the drive layer (bus current limiter); the old extra
     * R-slope here punished the same current twice. Breaker trip/reset
     * dynamics are future work (documented, not faked). */
    float sag = b->internal_resistance * total_current_draw;
    float terminal = open_circuit - sag;
    if (terminal < 0.0f) terminal = 0.0f; /* empty means empty, no 9V gift */
    if (terminal > open_circuit) terminal = open_circuit;
    return terminal;
}

void battery_drain(battery *b, float total_current_draw, float dt) {
    if ((!b) || (dt <= 0.0f)) {
        return;
    }
    /* True coulomb counting + mild Peukert. No 0.25 life extender. */
    float peukert_current = powf(fmaxf(total_current_draw, 0.0f), 1.1f);
    float amp_hours_used = (peukert_current * dt) / 3600.0f;
    b->charge_fraction -= amp_hours_used / b->capacity_ah;
    if (b->charge_fraction < 0.0f) {
        b->charge_fraction = 0.0f;
    }
    if (b->charge_fraction > 1.0f) b->charge_fraction = 1.0f;
}
