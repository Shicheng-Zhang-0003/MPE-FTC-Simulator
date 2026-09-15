/* MPE_FTC_059C: physics world — full pipeline.
 * Supersedes MPE_FTC_056 (free-body step).
 * Pipeline mirrors the legacy loop in simulation.c, minus:
 *   - sleep staticize hack (sleeping bodies keep real mass here),
 *   - positional depenetration pass,
 *   - joints/constraints (Phase 1).
 * NOTE: the contact warm-start cache is still engine-global. Worlds
 * must seed non-overlapping object_id ranges (see tests/two_world_test.c).
 * A per-world cache is tracked as future work.
 */
#include "physics_world.h"
#include "../physics/collision_mechanics.h"
#include "../physics/broadphase.h"
#include "../physics/constraint.h" /* MPE_FTC_067 */
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include <stdlib.h>
#include <math.h>
#include <string.h> /* MPE_FTC_076a */

static physics_world g_physics_world = {.bodies = NULL, .body_count = 0, .body_capacity = 0, .next_object_id = 1};

#define MFS_WORLD_MAGIC 0x4D503231u /* "MP21" */

static broadphase_pair world_pairs[mpe_max_broadphase_pairs];
static collision_data world_manifolds[a3_max_manifolds];

void physics_world_init(physics_world *world) {
    if (!world) {
        return;
    }
    /* MFS_300_LIFECYCLE: Free old allocations before memset to prevent leaks.
     * MFS_310B: only free() if *we* previously initialized this world. A fresh
     * stack/global world contains garbage in ->bodies, and free()-ing a garbage
     * pointer crashed every test binary that declared `physics_world world;`. */
    if (world->magic == MFS_WORLD_MAGIC) {
        free(world->bodies);
        world->bodies = NULL;
        free(world->world_contact_cache);
        world->world_contact_cache = NULL;
    }
    memset(world, 0, sizeof(physics_world));
    world->magic = MFS_WORLD_MAGIC;
    constraint_pool_init(); /* MFS: bump generation so new world's joints don't see stale pool */

    if (!world->bodies) {
        world->bodies = (rigidbody *) malloc((size_t) mpe_max_bodies * sizeof(rigidbody));
        world->body_capacity = mpe_max_bodies;
    }
    if (!world->world_contact_cache) {
        world->world_contact_cache =
            (cached_contact *) malloc((size_t) max_cached_contacts * sizeof(cached_contact)); /* MFS_131A */
    }
    world->body_count = 0;
    world->magic = MFS_WORLD_MAGIC;
    if (world->next_object_id == 0) {
        world->next_object_id = 1;
    }
    /* MFS_300B: Walls moved AFTER allocation so add_cube works. */
    /* MFS_202_R307: FTC field containment walls (12ft x 12ft).
     * Prevents bodies from escaping to infinity and producing NaN.
     * Hardened: thickness 2 in (was 1 in) to reduce tunneling without CCD,
     * height 0.5m to prevent jump-over; still static so no solver cost. */
    {
        float half_w = 1.8288f;   /* 6 ft */
        float wall_h = 0.5f;   /* 20 in, hardened */
        float wall_t = 0.0508f;   /* 2 in, hardened */

        /* North wall */
        physics_world_add_cube(world,
            (vector3){0.0f, wall_h * 0.5f, half_w},
            (vector3){half_w, wall_h * 0.5f, wall_t * 0.5f}, 0.0f);
        /* South wall */
        physics_world_add_cube(world,
            (vector3){0.0f, wall_h * 0.5f, -half_w},
            (vector3){half_w, wall_h * 0.5f, wall_t * 0.5f}, 0.0f);
        /* East wall */
        physics_world_add_cube(world,
            (vector3){half_w, wall_h * 0.5f, 0.0f},
            (vector3){wall_t * 0.5f, wall_h * 0.5f, half_w}, 0.0f);
        /* West wall */
        physics_world_add_cube(world,
            (vector3){-half_w, wall_h * 0.5f, 0.0f},
            (vector3){wall_t * 0.5f, wall_h * 0.5f, half_w}, 0.0f);
    }



}

void physics_world_cleanup(physics_world *world) {
    if (!world) {
        return;
    }
    /* MFS_310B: only free buffers we allocated. */
    if (world->magic == MFS_WORLD_MAGIC) {
        free(world->bodies);
        world->bodies = NULL;
        free(world->world_contact_cache); /* MFS_131A */
        world->world_contact_cache = NULL;
    }
    memset(world, 0, sizeof(physics_world));
}

int physics_world_add_sphere(physics_world *world, float radius, float mass, vector3 position) {
    if ((!world) || (!world->bodies) || (world->body_count >= world->body_capacity)) {
        return -1;
    }
    rigidbody *rb = &world->bodies[world->body_count];
    rigidbody_initialisation_sphere(rb, radius, mass, position);
    rb->object_id = world->next_object_id++;
    rb->object_generation = 1;
    rigidbody_sanitize(rb);
    return world->body_count++;
}

int physics_world_add_cube(physics_world *world, vector3 position, vector3 half_extensions, float mass) {
    if ((!world) || (!world->bodies) || (world->body_count >= world->body_capacity)) {
        return -1;
    }
    rigidbody *rb = &world->bodies[world->body_count];
    rigidbody_initialisation_cube(rb, position, half_extensions, mass);
    rb->object_id = world->next_object_id++;
    rb->object_generation = 1;
    rigidbody_sanitize(rb);
    return world->body_count++;
}

/* MPE_FTC_091 */
int physics_world_add_cylinder(physics_world *world, float radius, float half_length, float mass,
                             vector3 position) {
    if ((!world) || (!world->bodies) || (world->body_count >= world->body_capacity)) {
        return -1;
    }
    rigidbody *rb = &world->bodies[world->body_count];
    rigidbody_initialisation_cylinder(rb, radius, half_length, mass, position);
    rb->object_id = world->next_object_id++;
    rb->object_generation = 1;
    rigidbody_sanitize(rb);
    return world->body_count++;
}

void physics_world_clear(physics_world *world) {
    if (!world) {
        return;
    }
    world->body_count = 0;
    world->world_contact_cache_count = 0; /* MFS_131A */
}

void physics_world_step(physics_world *world, float dt) {
    if ((!world) || (!world->bodies) || (dt <= 0.0f) || (world->body_count <= 0)) {
        return;
    }

    for (int i = 0; i < world->body_count; i++) {
        rigidbody_sanitize(&world->bodies[i]);
    }

    int pair_count = 0;
    if (world->body_count >= 2) {
        pair_count =
            broadphase_generate_pairing(world->bodies, world->body_count, world_pairs, mpe_max_broadphase_pairs);
    }

    int manifold_count = 0;
    for (int p = 0; p < pair_count; p++) {
        int index_a = world_pairs[p].object_index_a;
        int index_b = world_pairs[p].object_index_b;
        if ((index_a < 0) || (index_a >= world->body_count)) {
            continue;
        }
        if ((index_b < 0) || (index_b >= world->body_count)) {
            continue;
        }
        rigidbody *body_a = &world->bodies[index_a];
        rigidbody *body_b = &world->bodies[index_b];
        if ((body_a->is_sleeping) && (body_b->is_sleeping)) {
            continue;
        }
        collision_data narrowphase_collision = {0};
        bool collided = false;
        if ((body_a->type == object_sphere) && (body_b->type == object_sphere)) {
            collided = collision_dual_sphere(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_sphere) && (body_b->type == object_cube)) {
            collided = collision_sphere_cube(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_cube) && (body_b->type == object_sphere)) {
            collided = collision_sphere_cube(body_b, body_a, &narrowphase_collision);
            if (collided) {
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            }
        } else if ((body_a->type == object_cube) && (body_b->type == object_cube)) {
            collided = collision_dual_cube(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_cylinder) && (body_b->type == object_sphere)) { /* MFS_173C_REPAIRED */
            collided = collision_cylinder_sphere(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_sphere) && (body_b->type == object_cylinder)) {
            collided = collision_cylinder_sphere(body_b, body_a, &narrowphase_collision);
            if (collided) {
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            }
        } else if ((body_a->type == object_cylinder) && (body_b->type == object_cube)) {
            collided = collision_cylinder_cube(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_cube) && (body_b->type == object_cylinder)) {
            collided = collision_cylinder_cube(body_b, body_a, &narrowphase_collision);
            if (collided) {
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            }
        } else if ((body_a->type == object_cylinder) && (body_b->type == object_cylinder)) {
            collided = collision_cylinder_cylinder(body_a, body_b, &narrowphase_collision);
        }
        if ((collided) && (manifold_count >= 0) && (manifold_count < a3_max_manifolds)) { /* MPE_FTC_078 */
            collision_prepare_solver(&narrowphase_collision, &world_manifolds[manifold_count], dt); /* MFS_205 */
            /* MFS_202_R306: Wake sleeping bodies on contact with active bodies. */
            {
                rigidbody *wake_a = &world->bodies[index_a];
                rigidbody *wake_b = &world->bodies[index_b];
                if ((wake_a->is_sleeping) && (!wake_b->is_sleeping) && (!wake_b->static_state)) {
                    float speed_sq = vector3_length_squared(wake_b->velocity);
                    if (speed_sq > g_cfg.sleep.wake_linear_thresh_sq) {
                        rigidbody_wake(wake_a);
                    }
                }
                if ((wake_b->is_sleeping) && (!wake_a->is_sleeping) && (!wake_a->static_state)) {
                    float speed_sq = vector3_length_squared(wake_a->velocity);
                    if (speed_sq > g_cfg.sleep.wake_linear_thresh_sq) {
                        rigidbody_wake(wake_b);
                    }
                }
            }

            manifold_count++;
        }
    }

    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];
        if ((rb->static_state) || (rb->is_sleeping)) {
            continue;
        }
        collision_data floor_collision = {0};
        if ((collision_static_plane_body(rb, 0.0f, &floor_collision)) && (manifold_count < a3_max_manifolds)) {
            collision_prepare_solver(&floor_collision, &world_manifolds[manifold_count], dt); /* MFS_205 */
            manifold_count++;
        }
    }

    vector3 gravity = {0.0f, g_cfg.world.gravity, 0.0f};
    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];
        if ((rb->static_state) || (rb->is_sleeping)) {
            continue;
        }
        rb_apply_forces_perfect(rb, vector3_scaling(gravity, rb->mass));
    }

    float linear_damping = powf(g_cfg.world.drag, dt);
    float angular_damping = powf(g_cfg.world.drag * g_cfg.world.angular_damping_factor, dt);
    constraint_apply_motors(world->bodies, world->body_count, dt); /* MPE_FTC_067 */
    for (int i = 0; i < world->body_count; i++) {
        rb_integrate_velocity(&world->bodies[i], dt, linear_damping, angular_damping);
    }

    int solver_iterations = g_cfg.timestep.solver_iterations;
    for (int iter = 0; iter < solver_iterations; iter++) {
        for (int m = 0; m < manifold_count; m++) {
            collision_resolve_iterative(&world_manifolds[m]);
        }
        constraint_solve_all(world->bodies, world->body_count, dt);
    }

    /* MFS_150_WHEEL_LOCK: torque-based lock for idle MECANUM wheels only.
     * Replaces velocity snap with a brake torque opposing axle spin, limited to I*ω/dt.
     * Mecanum wheels: brake on AXLE axis only (prevents forward/back creep).
     * Free-slide axis is handled by anisotropic friction (near-zero mu).
     * Regular wheels use rolling friction (rb_integrate_velocity) instead. */
    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];
        if (rb->is_mecanum && !rb->driven_this_tick) {
            vector3 axle = rb->cached_axes[0];
            if (vector3_length_squared(axle) < 0.0001f) {
                axle = vector4_rotate_to_vector3(rb->orientation, (vector3){1.0f, 0.0f, 0.0f});
            }
            float axle_omega = vector3_dot(rb->angular_velocity, axle);
            float thresh = g_cfg.solver.wheel_lock_omega_thresh;
            if (thresh <= 0.0f) thresh = 1.5f;
            /* Mecanum wheels: use same threshold as default to catch idle creep up to ~1.5 rad/s */
            if (fabsf(axle_omega) < thresh && fabsf(axle_omega) > 0.001f) {
                float brake_tau = 0.08f; /* softer for mecanum */
                float eff_I = rb->inertia_tensor_local.matrix[0][0];
                if (eff_I < 0.0001f) eff_I = 0.000625f;
                float max_tau = fabsf(axle_omega) * eff_I / dt;
                if (brake_tau > max_tau) brake_tau = max_tau;
                float sign = (axle_omega > 0.0f) ? -1.0f : 1.0f;
                float inv_xx = rb->inverse_inertia_system.matrix[0][0];
                if (inv_xx < 0.0001f) inv_xx = 1.0f / eff_I;
                float delta_omega = sign * brake_tau * dt * inv_xx;
                if (fabsf(delta_omega) > fabsf(axle_omega)) delta_omega = -axle_omega;
                rb->angular_velocity = vector3_addition(rb->angular_velocity, vector3_scaling(axle, delta_omega));
            }
        }
    }

    contact_cache_save(world, world_manifolds, manifold_count);

    /* Clear driven_this_tick flag at end of step */
    for (int i = 0; i < world->body_count; i++) {
        world->bodies[i].driven_this_tick = false;
    }

    for (int i = 0; i < world->body_count; i++) {
        rb_integrate_position(&world->bodies[i], dt);
        /* Hard clamp to field bounds to prevent tunneling without CCD.
         * Clamp X/Z so body edge stays inside wall. Y not clamped here (floor handles it).
         * Skip mecanum wheels (robot wheels) -- their position is constrained by revolute
         * joints to the chassis, and clamping them directly fights the joint, causing
         * the chassis to stop prematurely ("invisible wall" symptom). */
        if (!world->bodies[i].static_state && !world->bodies[i].is_mecanum) {
            float half_w = 1.8288f;
            float margin_x = 0.1f, margin_z = 0.1f;
            if (world->bodies[i].type == object_cube) {
                margin_x = world->bodies[i].half_extensions.x + 0.02f;
                margin_z = world->bodies[i].half_extensions.z + 0.02f;
            } else if (world->bodies[i].type == object_cylinder) {
                /* Cylinder axle along X, so X margin is half_length, Z margin is radius */
                margin_x = world->bodies[i].cylinder_half_length + 0.02f;
                margin_z = world->bodies[i].radius + 0.02f;
                /* If axle not along X (tilted), use max */
                float alt = world->bodies[i].radius + 0.02f;
                if (alt > margin_x) margin_x = alt;
            } else {
                margin_x = margin_z = world->bodies[i].radius + 0.02f;
            }
            float lim_x = half_w - margin_x;
            float lim_z = half_w - margin_z;
            if (world->bodies[i].position.x > lim_x) { world->bodies[i].position.x = lim_x; if (world->bodies[i].velocity.x > 0) world->bodies[i].velocity.x = 0; }
            if (world->bodies[i].position.x < -lim_x) { world->bodies[i].position.x = -lim_x; if (world->bodies[i].velocity.x < 0) world->bodies[i].velocity.x = 0; }
            if (world->bodies[i].position.z > lim_z) { world->bodies[i].position.z = lim_z; if (world->bodies[i].velocity.z > 0) world->bodies[i].velocity.z = 0; }
            if (world->bodies[i].position.z < -lim_z) { world->bodies[i].position.z = -lim_z; if (world->bodies[i].velocity.z < 0) world->bodies[i].velocity.z = 0; }
        }
        rigidbody_sanitize(&world->bodies[i]);
    }
}

physics_world *physics_world_get_primary(void) {
    return &g_physics_world;
}
