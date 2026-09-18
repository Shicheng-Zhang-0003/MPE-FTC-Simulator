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


/* One broadphase pair through narrowphase + wake-on-contact + solver
 * prep (legacy GUI path). Shared by the main pair loop and the sleep-wake
 * revisit pass below; see physics_world.c for the race it closes. */
static void simulation_process_pair(physics_world *world, int object_index_a, int object_index_b, float dt,
                                    collision_data *manifolds, int max_manifolds, int *manifold_count_ptr) {
    rigidbody *body_a = &world->bodies[object_index_a];
    rigidbody *body_b = &world->bodies[object_index_b];
    /* FTC transplant: skip joint-connected bodies (wheel↔chassis). */
    if (constraint_bodies_connected(world, body_a->object_id, body_b->object_id)) return;
            collision_data narrowphase_collision = {0};
            bool collided = false;
            if (body_a->type == object_sphere && body_b->type == object_sphere)
                collided = collision_dual_sphere(body_a, body_b, &narrowphase_collision);
            else if (body_a->type == object_sphere && body_b->type == object_cube)
                collided = collision_sphere_cube(body_a, body_b, &narrowphase_collision);
            else if (body_a->type == object_cube && body_b->type == object_sphere) {
                collided = collision_sphere_cube(body_b, body_a, &narrowphase_collision);
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            } else if (body_a->type == object_cube && body_b->type == object_cube)
                collided = collision_dual_cube(body_a, body_b, &narrowphase_collision);
            /* MFS_173D: Cylinder collision dispatch */
            else if (body_a->type == object_cylinder && body_b->type == object_sphere)
                collided = collision_cylinder_sphere(body_a, body_b, &narrowphase_collision);
            else if (body_a->type == object_sphere && body_b->type == object_cylinder) {
                collided = collision_cylinder_sphere(body_b, body_a, &narrowphase_collision);
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            } else if (body_a->type == object_cylinder && body_b->type == object_cube)
                collided = collision_cylinder_cube(body_a, body_b, &narrowphase_collision);
            else if (body_a->type == object_cube && body_b->type == object_cylinder) {
                collided = collision_cylinder_cube(body_b, body_a, &narrowphase_collision);
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            } else if (body_a->type == object_cylinder && body_b->type == object_cylinder)
                collided = collision_cylinder_cylinder(body_a, body_b, &narrowphase_collision);
            if (collided) {
                if ((*manifold_count_ptr) >= max_manifolds) {
                    debug_last_manifold_overflow_count++;
                    return;
                }
                    bool a3_a_was_sleeping = body_a->is_sleeping;
                    bool a3_b_was_sleeping = body_b->is_sleeping;
                    if (a3_a_was_sleeping && a3_b_was_sleeping) { return; }
                    /* TRUTH: three-gate wake — new edge, fast other, deep
                     * overlap (see physics_world.c). Persistent resting
                     * contact alone must not veto sleep. */
                    bool a3_new_edge = !contact_cache_has_pair(world, body_a->object_id, body_b->object_id);
                    if ((a3_a_was_sleeping) && (!body_b->static_state) && a3_new_edge) {
                        rigidbody_wake(body_a);
                    }
                    if ((a3_b_was_sleeping) && (!body_a->static_state) && a3_new_edge) {
                        rigidbody_wake(body_b);
                    }
                    float a3_wake_lin = g_cfg.sleep.wake_linear_thresh_sq;
                    float a3_wake_ang = g_cfg.sleep.wake_angular_thresh_sq;
                    bool a3_a_fast = (!a3_a_was_sleeping) &&
                                     ((vector3_length_squared(body_a->velocity) > a3_wake_lin) ||
                                      (vector3_length_squared(body_a->angular_velocity) > a3_wake_ang));
                    bool a3_b_fast = (!a3_b_was_sleeping) &&
                                     ((vector3_length_squared(body_b->velocity) > a3_wake_lin) ||
                                      (vector3_length_squared(body_b->angular_velocity) > a3_wake_ang));
                    if ((a3_a_was_sleeping) && (!body_b->static_state) && a3_b_fast) {
                        rigidbody_wake(body_a);
                    }
                    if ((a3_b_was_sleeping) && (!body_a->static_state) && a3_a_fast) {
                        rigidbody_wake(body_b);
                    }
                    /* Wake on significant overlap growth, even against static
                     * geometry (see physics_world.c). */
                    {
                        float deepest = 0.0f;
                        for (int wi = 0; wi < narrowphase_collision.contact_count; wi++) {
                            if (narrowphase_collision.contacts[wi].penetration > deepest) {
                                deepest = narrowphase_collision.contacts[wi].penetration;
                            }
                        }
                        if (deepest > g_cfg.depenetration.wake_depth_thresh) {
                            if (a3_a_was_sleeping) {
                                rigidbody_wake(body_a);
                            }
                            if (a3_b_was_sleeping) {
                                rigidbody_wake(body_b);
                            }
                        }
                    }
                    collision_prepare_solver(world, &narrowphase_collision, &manifolds[(*manifold_count_ptr)],
                                             dt);
                    (*manifold_count_ptr)++;
                    /* Support flag on manifold presence (see world path). */
                    if (world->has_contact) {
                        if ((object_index_a >= 0) && (object_index_a < world->body_count)) {
                            world->has_contact[object_index_a] = 1;
                        }
                        if ((object_index_b >= 0) && (object_index_b < world->body_count)) {
                            world->has_contact[object_index_b] = 1;
                        }
                    }
            }
}

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