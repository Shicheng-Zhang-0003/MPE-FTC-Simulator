/* MPE_FTC_072: Battery model implementation */
#include "battery.h"
#include <math.h>

void battery_init(battery *b) {
    if (!b) {
        return;
    }
    b->nominal_voltage = 12.8f;
    b->internal_resistance = 0.015f; /* FIX 112: realistic LiPo internal resistance */
    b->capacity_ah = 3.5f; /* MFS_BATTERY_FIX: realistic FTC 12V LiPo capacity (~3-4 Ah, not 30 Ah) */
    b->charge_fraction = 1.0f;
}

float battery_get_voltage(const battery *b, float total_current_draw) {
    if (!b) {
        return 12.8f;
    }
    /* Non-linear discharge: LiPo stays ~12.8V until 20% then drops quickly.
     * Approx with piecewise: >20% charge => voltage ~12.6-12.8, <20% => linear drop to 10V. */
    float effective_charge = b->charge_fraction;
    float open_circuit;
    if (effective_charge > 0.2f) {
        open_circuit = 12.3f + 0.5f * (effective_charge - 0.2f) / 0.8f; /* 12.3-12.8 */
    } else {
        open_circuit = 10.0f + effective_charge * 11.5f; /* 10-12.3 at 0.2 */
    }
    float sag = b->internal_resistance * total_current_draw;
    float terminal = open_circuit - sag;
    if (terminal < 9.0f) {
        terminal = 9.0f; /* brownout, not 0 */
    }
    if (terminal > open_circuit) terminal = open_circuit;
    return terminal;
}

void battery_drain(battery *b, float total_current_draw, float dt) {
    if ((!b) || (dt <= 0.0f)) {
        return;
    }
    /* Use RMS-ish average: total_current is sum of |I| peaks, but average draw is lower
     * due to PWM duty and not all motors at stall simultaneously. Scale by 0.25 to get
     * realistic 2-3 minute match life instead of 50s. Also apply Peukert (exponent 1.1). */
    float avg_current = total_current_draw * 0.25f;
    float peukert_current = powf(fmaxf(avg_current, 0.0f), 1.1f);
    float amp_hours_used = (peukert_current * dt) / 3600.0f;
    b->charge_fraction -= amp_hours_used / b->capacity_ah;
    if (b->charge_fraction < 0.0f) {
        b->charge_fraction = 0.0f;
    }
    if (b->charge_fraction > 1.0f) b->charge_fraction = 1.0f;
}
