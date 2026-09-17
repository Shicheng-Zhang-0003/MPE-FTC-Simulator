/* MPE_FTC_072: Battery model with voltage sag */
#ifndef battery_h
#define battery_h

/* FTC pack: 12V NiMH 3000mAh (goBILDA 12V nested, 20A fuse). Peukert 1.1
 * coulomb counting; terminal floored at 0 (empty means empty). Breaker
 * trip/reset dynamics intentionally unmodeled (see bus foldback note in
 * robot.c); over-current protection beyond foldback is future work. */
typedef struct {
    float nominal_voltage; /* V (12.0 NiMH) */
    float internal_resistance; /* ohms (~0.03 pack + wiring) */
    float capacity_ah; /* amp-hours (3.0) */
    float charge_fraction; /* 0..1 */
} battery;

void battery_init(battery *b);
/* Returns terminal voltage under load. total_current = sum of all motor currents. */
float battery_get_voltage(const battery *b, float total_current_draw);
/* Drain battery over time based on current draw. */
void battery_drain(battery *b, float total_current_draw, float dt);

#endif /* battery_h */
