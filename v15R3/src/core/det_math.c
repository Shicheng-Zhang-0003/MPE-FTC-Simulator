/* det_math.c — single definition site for the deterministic-math fallback
 * counters declared extern in det_math.h.
 *
 * FIX-AUDIT: the counters were previously `static` in the header, giving
 * every translation unit its own copy. A zero-fallback assert in one TU
 * (e.g. the determinism test) could not observe fallbacks taken in
 * rigidbody.c / physics_world.c / collision_mechanics.c. Exactly one
 * definition lives here; all TUs share it. Portable C99 + fenv only.
 */
#include "det_math.h"

unsigned long det_fallback_pow_count = 0;
unsigned long det_fallback_trig_count = 0;
