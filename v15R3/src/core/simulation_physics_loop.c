/* MFS_INCREMENT_SPLIT_2: Fixed-timestep physics loop.
 * Extracted from physics_step_increment in simulation.c.
 * Owns: the accumulator, broadphase, narrowphase, solver, integration,
 *        sleep staticize/restore, boundary, depenetration.
 * LEGACY GUI PATH: operates on global world->bodies / world->body_count so the
 * GTK render/editor loop keeps working. Canonical stepping is
 * physics_world_step() in core/physics_world.c (used by all headless
 * tests). Do not add new simulation state here — add it to physics_world.
 */
#include "../mpe_engine.h"
#include "../physics/depenetration.h"
#include "../physics/constraint.h"
#include "../physics/islands.h"
#include "../core/det_math.h"
#include <math.h>

/* FIX-AUDIT: a ~100-line static legacy pair helper (simulation_process_pair,
 * narrowphase duplicate of physics_world_process_pair) lived here with zero
 * callers and its own wake/manifold logic — a trap for any future caller
 * (stale behavior, -Wunused-function). Deleted: the canonical path is
 * physics_world_step() below. */

void simulation_physics_tick(float frame_delta_time) {
    (void) frame_delta_time; /* Physics thread runs at fixed 60Hz; this is called from UI thread but physics runs independently */
    const float fixed_physics_dt = 1.0f / 60.0f;

    /* All simulation state lives in the primary world; this tick only
     * borrows it (scratch included). Worlds that failed init degrade
     * to skipped ticks, never crashes (low-memory contract). */
    physics_world *world = physics_world_get_primary();
    if ((!world) || (!world->bodies) || (!world->pair_buffer) || (!world->manifolds) ||
        (!world->manifold_awake) || (!world->manifold_order) || (!world->manifold_sort_keys) ||
        (!world->pair_skipped)) {
        return;
    }

    /* Run exactly one fixed physics step. The physics thread ensures
     * this is called at 60Hz. No accumulator, no substeps here. */
    physics_world_step(world, fixed_physics_dt);
}