/* MPE_FTC_062: revolute (hinge) constraint solver.
 *
 * revolute_solve() enforces per solver iteration (velocity-level only):
 *   1. point-to-point: the two anchors coincide (3 DOF removed), solved
 *      with a 3x3 effective-mass impulse + Baumgarte positional bias.
 *   2. axis alignment: relative angular velocity perpendicular to the
 *      hinge axis is removed (2 DOF removed), leaving spin about the axis.
 *   3. angle limits: persistent relative-angle tracking with velocity-level
 *      enforcement (limits_enabled, limit_min_rad, limit_max_rad).
 * revolute_correct_axis_drift() applies the positional Baumgarte
 * correction that keeps hinge axes aligned (prevents wheel tilt under
 * load). It MUST run exactly once per tick after the iteration loop:
 * the error is positional, so per-iteration application multiplies the
 * correction by the iteration count and pumps energy into the joint.
 * revolute_apply_motor() drives relative spin about the axis toward a
 * target speed by adding torque to the torque accumulator (integrated once
 * per tick), clamped to a max torque. It is intentionally NOT inside the
 * iterative contact loop, so it cannot over-apply.
 *
 * Known simplifications (documented):
 *   - Jointed bodies are kept awake (FTC robots are always active).
 *   - Single pass per tick; an accumulated-impulse iterative variant is a
 *     future stiffness upgrade.
 *   */
#include "revolute_joint.h"
#include "../config/mpe_config.h"
#include <math.h>
#include <stdio.h>

static math3 skew_symmetric(vector3 v) {
    math3 m = {{{0.0f}}};
    m.matrix[0][1] = -v.z;
    m.matrix[0][2] = v.y;
    m.matrix[1][0] = v.z;
    m.matrix[1][2] = -v.x;
    m.matrix[2][0] = -v.y;
    m.matrix[2][1] = v.x;
    return m;
}

static math3 math3_addition(math3 a, math3 b) {
    math3 r;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r.matrix[i][j] = a.matrix[i][j] + b.matrix[i][j];
        }
    }
    return r;
}

void revolute_solve(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    vector3 r_a = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);

    /* ---- point-to-point velocity solve (with Baumgarte bias per iteration) ---- */
    vector3 anchor_a_world = vector3_addition(body_a->position, r_a);
    vector3 anchor_b_world = vector3_addition(body_b->position, r_b);
    vector3 position_error = vector3_subtraction(anchor_b_world, anchor_a_world);
    /* TRUTH: error-gated wake. Unconditional waking pins every passive
     * hinge awake forever (defeats island sleep). A sleeping member rejoins
     * only when the joint has real work (anchor gap beyond slop); settled
     * hinges keep their sleep. */
    if (vector3_length_squared(position_error) >
        g_cfg.solver.penetration_slop * g_cfg.solver.penetration_slop) {
        if (body_a->is_sleeping) {
            rigidbody_wake(body_a);
        }
        if (body_b->is_sleeping) {
            rigidbody_wake(body_b);
        }
    }
    float inv_mass_a = rigidbody_effective_inv_mass(body_a);
    float inv_mass_b = rigidbody_effective_inv_mass(body_b);
    if ((inv_mass_a <= 0.0f) && (inv_mass_b <= 0.0f)) {
        return;
    }

    vector3 vel_a_at_anchor = vector3_addition(body_a->velocity, vector3_cross(body_a->angular_velocity, r_a));
    vector3 vel_b_at_anchor = vector3_addition(body_b->velocity, vector3_cross(body_b->angular_velocity, r_b));
    vector3 relative_velocity = vector3_subtraction(vel_b_at_anchor, vel_a_at_anchor);

    /* Use pre-computed Baumgarte bias from pre_step (once per tick) */
    vector3 bias = p->positional_bias;

    /* Solve for zero relative velocity + Baumgarte bias at anchors */
    float inv_mass_sum = inv_mass_a + inv_mass_b;
    math3 k = {{{0.0f}}};
    for (int i = 0; i < 3; i++) {
        k.matrix[i][i] = inv_mass_sum;
    }
    math3 skew_a = skew_symmetric(r_a);
    math3 skew_b = skew_symmetric(r_b);
    /* k = inv_mass_sum*I - skew(r_a)*Ia^-1*skew(r_a) - skew(r_b)*Ib^-1*skew(r_b)
     * (the subtracted terms are positive semi-definite, so k stays SPD) */
    math3 term_a = math3_multiplication(skew_a, math3_multiplication(rigidbody_effective_inv_inertia(body_a), skew_a));
    math3 term_b = math3_multiplication(skew_b, math3_multiplication(rigidbody_effective_inv_inertia(body_b), skew_b));
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            k.matrix[i][j] -= term_a.matrix[i][j];
            k.matrix[i][j] -= term_b.matrix[i][j];
        }
    }
    math3 k_inv = math3_inverse(k);
    vector3 rhs = vector3_scaling(vector3_addition(relative_velocity, bias), -1.0f);
    vector3 impulse = math3_multiplication_vector3(k_inv, rhs);

    body_a->velocity = vector3_subtraction(body_a->velocity, vector3_scaling(impulse, inv_mass_a));
    body_b->velocity = vector3_addition(body_b->velocity, vector3_scaling(impulse, inv_mass_b));
    body_a->angular_velocity =
        vector3_subtraction(body_a->angular_velocity,
                            math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a),
                                                         vector3_cross(r_a, impulse)));
    body_b->angular_velocity =
        vector3_addition(body_b->angular_velocity,
                         math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b),
                                                      vector3_cross(r_b, impulse)));

    /* ---- axis alignment: kill relative angular velocity off the hinge ----
     * TRUTH: lock against BOTH per-body hinge axes (averaged), not axis_a
     * alone: with distinct axes (constraint_set_revolute_axes) the old
     * single-sided lock under-constrained body B's hinge direction. */
    vector3 axis_a_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    vector3 axis_b_world = (vector3_length_squared(p->axis_b) > 1e-12f)
        ? vector4_rotate_to_vector3(body_b->orientation, vector3_normalisation(p->axis_b))
        : axis_a_world;
    vector3 axis_sum = vector3_addition(axis_a_world, axis_b_world);
    vector3 axis_world = (vector3_length_squared(axis_sum) > 1e-12f)
        ? vector3_normalisation(axis_sum)
        : axis_a_world;
    vector3 relative_angular = vector3_subtraction(body_b->angular_velocity, body_a->angular_velocity);
    float along_axis = vector3_dot(relative_angular, axis_world);
    vector3 perpendicular_angular = vector3_subtraction(relative_angular, vector3_scaling(axis_world, along_axis));

    math3 angular_mass = math3_addition(rigidbody_effective_inv_inertia(body_a),
                                        rigidbody_effective_inv_inertia(body_b));
    math3 angular_mass_inv = math3_inverse(angular_mass);
    vector3 angular_impulse =
        vector3_scaling(math3_multiplication_vector3(angular_mass_inv, perpendicular_angular), -1.0f);
    body_a->angular_velocity = vector3_subtraction(
        body_a->angular_velocity,
        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), angular_impulse));
    body_b->angular_velocity = vector3_addition(
        body_b->angular_velocity,
        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), angular_impulse));

    /* ---- angle limits: persistent relative-angle tracking + velocity-level enforcement ----
     * TRUTH: angle integration lives in revolute_pre_step() (once per tick).
     * Old code integrated here (per solver iteration, 64x/tick). */
    if (p->limits_enabled) {
        /* Initialize reference frame on first solve after limits enabled. */
        if (!p->angle_initialized) {
            /* Compute initial relative orientation around hinge axis. */
            vector4 q_a_inv = {body_a->orientation.w, -body_a->orientation.x, -body_a->orientation.y, -body_a->orientation.z};
            vector4 q_rel = vector4_multiplication(q_a_inv, body_b->orientation);
            q_rel = vector4_normalisation(q_rel);
            if (q_rel.w < 0.0f) {
                q_rel.w = -q_rel.w;
                q_rel.x = -q_rel.x;
                q_rel.y = -q_rel.y;
                q_rel.z = -q_rel.z;
            }
            /* Extract rotation angle around hinge axis (axis_a in A's local space). */
            float half_angle = atan2f(sqrtf(q_rel.x * q_rel.x + q_rel.y * q_rel.y + q_rel.z * q_rel.z), q_rel.w);
            vector3 rot_axis = {q_rel.x, q_rel.y, q_rel.z};
            float rot_axis_len = sqrtf(vector3_length_squared(rot_axis));
            if (rot_axis_len > 1e-6f) {
                rot_axis = vector3_scaling(rot_axis, 1.0f / rot_axis_len);
            } else {
                rot_axis = axis_world;
            }
            /* Project rotation onto hinge axis. */
            float axis_dot = vector3_dot(rot_axis, axis_world);
            float initial_angle = 2.0f * half_angle * axis_dot;
            /* Normalize to [-pi, pi] for consistency. */
            while (initial_angle > math_pi) initial_angle -= 2.0f * math_pi;
            while (initial_angle < -math_pi) initial_angle += 2.0f * math_pi;
            p->accumulated_angle = initial_angle;
            p->reference_axis_a = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
            p->reference_axis_b = vector4_rotate_to_vector3(body_b->orientation,
                (vector3_length_squared(p->axis_b) > 1e-12f) ? vector3_normalisation(p->axis_b)
                                                            : vector3_normalisation(p->axis_a));
            p->angle_initialized = true;
        }

        /* Enforce limits at velocity level: if at limit and trying to go beyond, kill along-axis velocity. */
        float min_limit = p->limit_min_rad;
        float max_limit = p->limit_max_rad;
        bool at_min = (p->accumulated_angle <= min_limit + 1e-4f) && (along_axis < 0.0f);
        bool at_max = (p->accumulated_angle >= max_limit - 1e-4f) && (along_axis > 0.0f);

        if (at_min || at_max) {
            /* Compute effective mass along hinge axis for limit impulse. */
            float axis_mass_inv = vector3_dot(axis_world, math3_multiplication_vector3(
                math3_addition(rigidbody_effective_inv_inertia(body_a), rigidbody_effective_inv_inertia(body_b)), axis_world));
            float axis_mass = (axis_mass_inv > 1e-12f) ? (1.0f / axis_mass_inv) : 0.0f;

            if (axis_mass > 0.0f) {
                /* Velocity-level clamp: apply impulse to kill along-axis component. */
                float lambda = -along_axis * axis_mass;
                vector3 limit_impulse = vector3_scaling(axis_world, lambda);
                body_a->angular_velocity = vector3_subtraction(
                    body_a->angular_velocity,
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), limit_impulse));
                body_b->angular_velocity = vector3_addition(
                    body_b->angular_velocity,
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), limit_impulse));
                /* Clamp accumulated angle to limit to prevent numerical drift. */
                if (at_min) p->accumulated_angle = min_limit;
                if (at_max) p->accumulated_angle = max_limit;
            }
        }
    }
}

/* TRUTH: once-per-tick MEASURED hinge angle (called before the solver
 * loop). Old code dead-reckoned (accumulated += along*dt): the estimate
 * drifted from the true hinge pose (drift-correction velocities, wrap,
 * 1 rad/tick clamp), so limits clamped free motion early or let stops
 * blow through, firing limit impulses at the wrong configuration.
 * This reads the angle straight from relative orientation every tick
 * (same math as limit init); accumulated_angle is readout state. */
void revolute_pre_step(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    if ((!p) || (!body_a) || (!body_b) || (!(dt > 0.0f))) {
        return;
    }

    /* ---- Compute Baumgarte positional bias (once per tick) ---- */
    vector3 r_a = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);
    vector3 anchor_a_world = vector3_addition(body_a->position, r_a);
    vector3 anchor_b_world = vector3_addition(body_b->position, r_b);
    vector3 position_error = vector3_subtraction(anchor_b_world, anchor_a_world);
    float position_error_len = vector3_length(position_error);
    float slop = g_cfg.solver.penetration_slop;
    if (position_error_len <= slop) {
        p->positional_bias = vector3_zero();
    } else {
        /* Baumgarte bias velocity (per Catto's stabilized Baumgarte) */
        const float baumgarte_beta = g_cfg.joints.revolute_beta;
        float bias_speed = baumgarte_beta * position_error_len / dt;
        float max_bias_speed = g_cfg.joints.revolute_max_bias;
        if ((bias_speed > max_bias_speed) && (bias_speed > 0.0f)) {
            p->positional_bias = vector3_scaling(position_error, (baumgarte_beta / dt) * (max_bias_speed / bias_speed));
        } else {
            p->positional_bias = vector3_scaling(position_error, baumgarte_beta / dt);
        }
    }

    /* ---- Initialize reference axes for axis drift correction (once, first call) ----
     * These store the world-space hinge axes at initialization, used by
     * revolute_correct_axis_drift to anchor the hinge to its original orientation.
     * Done regardless of limits_enabled, so wheel joints also benefit. */
    if (!p->angle_initialized) {
        p->reference_axis_a = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
        p->reference_axis_b = vector4_rotate_to_vector3(body_b->orientation,
            (vector3_length_squared(p->axis_b) > 1e-12f) ? vector3_normalisation(p->axis_b)
                                                        : vector3_normalisation(p->axis_a));
        p->angle_initialized = true;
    }

    /* ---- Angle tracking for limits (unchanged) ---- */
    if (!p->limits_enabled || !p->angle_initialized) {
        return;
    }
    vector3 axis_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    vector4 q_a_inv = {body_a->orientation.w, -body_a->orientation.x, -body_a->orientation.y, -body_a->orientation.z};
    vector4 q_rel = vector4_normalisation(vector4_multiplication(q_a_inv, body_b->orientation));
    if (q_rel.w < 0.0f) {
        q_rel.w = -q_rel.w;
        q_rel.x = -q_rel.x;
        q_rel.y = -q_rel.y;
        q_rel.z = -q_rel.z;
    }
    float half_angle = atan2f(sqrtf(q_rel.x * q_rel.x + q_rel.y * q_rel.y + q_rel.z * q_rel.z), q_rel.w);
    vector3 rot_axis = {q_rel.x, q_rel.y, q_rel.z};
    float rot_len = sqrtf(vector3_length_squared(rot_axis));
    if (rot_len > 1e-6f) {
        rot_axis = vector3_scaling(rot_axis, 1.0f / rot_len);
    } else {
        rot_axis = axis_world;
    }
    float measured = 2.0f * half_angle * vector3_dot(rot_axis, axis_world);
    while (measured > math_pi) measured -= 2.0f * math_pi;
    while (measured < -math_pi) measured += 2.0f * math_pi;
    if (isfinite(measured)) {
        p->accumulated_angle = measured;
    }
}

/* Prismatic: single-axis slide with optional limits and motor. */
void prismatic_solve(prismatic_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }

    vector3 r_a = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);
    vector3 world_a = vector3_addition(body_a->position, r_a);
    vector3 world_b = vector3_addition(body_b->position, r_b);

    /* Slide axis in world space (from body A). */
    vector3 axis_a_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    vector3 axis_b_world = (vector3_length_squared(p->axis_b) > 1e-12f)
        ? vector4_rotate_to_vector3(body_b->orientation, vector3_normalisation(p->axis_b))
        : axis_a_world;

    /* Relative velocity along slide axis. */
    vector3 vel_a = vector3_addition(body_a->velocity, vector3_cross(body_a->angular_velocity, r_a));
    vector3 vel_b = vector3_addition(body_b->velocity, vector3_cross(body_b->angular_velocity, r_b));
    vector3 rel_vel = vector3_subtraction(vel_b, vel_a);
    float rel_n = vector3_dot(rel_vel, axis_a_world);

    /* Current position along axis (measured, not dead-reckoned). */
    vector3 delta = vector3_subtraction(world_b, world_a);
    float current_pos = vector3_dot(delta, axis_a_world);
    /* TRUTH: error-gated wake (see revolute_solve). Off-axis gap, limit
     * breach, or a live drive motor rejoins sleepers; a settled slider
     * keeps its sleep. */
    {
        vector3 off = vector3_subtraction(delta, vector3_scaling(axis_a_world, current_pos));
        bool motor_live = (p->motor_enabled) && (isfinite(p->motor_target_speed)) &&
            (fabsf(p->motor_target_speed) > 1e-6f);
        bool limit_hit = (p->limits_enabled) && (p->position_initialized) &&
            ((current_pos < p->limit_min) || (current_pos > p->limit_max));
        if ((vector3_length_squared(off) >
             g_cfg.solver.penetration_slop * g_cfg.solver.penetration_slop) ||
            (motor_live) || (limit_hit)) {
            if (body_a->is_sleeping) rigidbody_wake(body_a);
            if (body_b->is_sleeping) rigidbody_wake(body_b);
        }
    }

    float inv_a = rigidbody_effective_inv_mass(body_a);
    float inv_b = rigidbody_effective_inv_mass(body_b);
    if ((inv_a <= 0.0f) && (inv_b <= 0.0f)) return;

    /* Initialize position tracking once (reference = initial slide pos). */
    if (!p->position_initialized) {
        p->accumulated_position = current_pos;
        p->reference_axis_a = axis_a_world;
        p->reference_axis_b = axis_b_world;
        p->position_initialized = true;
    }

    /* ---- Perpendicular constraint: full 2-D (not 1-D slip dir).
     * TRUTH: old code killed only instantaneous perp_vel direction; when
     * |perp|~0 it applied zero and orthogonal drift grew free (wobble).
     * Build orthonormal basis (u,v) ⊥ axis and solve both. */
    {
        vector3 ref = (fabsf(axis_a_world.y) < 0.99f) ? (vector3){0.0f, 1.0f, 0.0f} : (vector3){1.0f, 0.0f, 0.0f};
        vector3 bu = vector3_subtraction(ref, vector3_scaling(axis_a_world, vector3_dot(ref, axis_a_world)));
        float bu_len_sq = vector3_length_squared(bu);
        if (bu_len_sq > 1e-12f) {
            bu = vector3_scaling(bu, 1.0f / sqrtf(bu_len_sq));
            vector3 bv = vector3_cross(axis_a_world, bu);
            vector3 basis[2] = {bu, bv};
            for (int bi = 0; bi < 2; bi++) {
                vector3 dir = basis[bi];
                float rel_d = vector3_dot(rel_vel, dir);
                /* Positional bias: keep anchors coincident off-axis.
                 * TRUTH: slop deadband (see revolute_solve). */
                float err_d = vector3_dot(delta, dir);
                float bias_d = 0.0f;
                if (err_d * err_d >
                    g_cfg.solver.penetration_slop * g_cfg.solver.penetration_slop) {
                    bias_d = g_cfg.joints.revolute_beta * err_d / dt;
                    float max_b = g_cfg.joints.revolute_max_bias;
                    if (bias_d > max_b) {
                        bias_d = max_b;
                    } else if (bias_d < -max_b) {
                        bias_d = -max_b;
}
}

vector3 ra_d = vector3_cross(r_a, dir);
                vector3 rb_d = vector3_cross(r_b, dir);
                float k_d = inv_a + inv_b +
                            vector3_dot(ra_d, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a),
                                                                          ra_d)) +
                            vector3_dot(rb_d, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b),
                                                                          rb_d));
                if (k_d > 1e-12f) {
                    float lambda_d = -(rel_d + bias_d) / k_d;
                    vector3 imp = vector3_scaling(dir, lambda_d);
                    body_a->velocity = vector3_subtraction(body_a->velocity, vector3_scaling(imp, inv_a));
                    body_b->velocity = vector3_addition(body_b->velocity, vector3_scaling(imp, inv_b));
                    body_a->angular_velocity = vector3_subtraction(
                        body_a->angular_velocity,
                        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a),
                                                     vector3_cross(r_a, imp)));
                    body_b->angular_velocity = vector3_addition(
                        body_b->angular_velocity,
                        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b),
                                                     vector3_cross(r_b, imp)));
                }
            }
        }
    }

    /* TRUTH: no axial equality — a slider slides freely. Old
     * err=current-accumulated (≈0 by construction) made the lock vacuous
     * when limits were off (free by accident) and double-paid rel_n when
     * limits were on (axis block + limit block). Only limits constrain. */

    /* ---- Limits enforcement (measured position, single pay) ---- */
    if (p->limits_enabled) {
        /* TRUTH: limits key off measured current_pos, not dead-reckoned
         * accumulated (which drifted 64x/tick). Accumulated tracks for API
         * readout via pre_step; enforcement uses measurement. */
        bool at_min = (current_pos <= p->limit_min + 1e-4f) && (rel_n < 0.0f);
        bool at_max = (current_pos >= p->limit_max - 1e-4f) && (rel_n > 0.0f);
        if (at_min || at_max) {
            vector3 ra_axis = vector3_cross(r_a, axis_a_world);
            vector3 rb_axis = vector3_cross(r_b, axis_a_world);
            float k_axis = inv_a + inv_b +
                           vector3_dot(ra_axis,
                                       math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), ra_axis)) +
                           vector3_dot(rb_axis,
                                       math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), rb_axis));
            if (k_axis > 1e-12f) {
                float lambda_limit = -rel_n / k_axis;
                vector3 limit_impulse = vector3_scaling(axis_a_world, lambda_limit);
                body_a->velocity = vector3_subtraction(body_a->velocity, vector3_scaling(limit_impulse, inv_a));
                body_b->velocity = vector3_addition(body_b->velocity, vector3_scaling(limit_impulse, inv_b));
                body_a->angular_velocity = vector3_subtraction(
                    body_a->angular_velocity,
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a),
                                                 vector3_cross(r_a, limit_impulse)));
                body_b->angular_velocity = vector3_addition(
                    body_b->angular_velocity,
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b),
                                                 vector3_cross(r_b, limit_impulse)));
            }
        }
    }
}

/* Positional point-to-point drift correction: MUST be called exactly once per tick,
 * AFTER the velocity iteration loop — never inside it. The error term is
 * positional (anchor gap, unchanged by velocity iterations), so per-iteration
 * application multiplies the correction by the iteration count (64x at defaults):
 * a spurious spring that pumps energy into wheel joints, causing vibration
 * and preventing rest (measured: coasting robot never sleeps). */
void revolute_correct_positional_drift(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    vector3 r_a = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);

    /* ---- point-to-point positional correction ---- */
    vector3 anchor_a_world = vector3_addition(body_a->position, r_a);
    vector3 anchor_b_world = vector3_addition(body_b->position, r_b);
    vector3 position_error = vector3_subtraction(anchor_b_world, anchor_a_world);
    float position_error_len_sq = vector3_length_squared(position_error);
    float slop_sq = g_cfg.solver.penetration_slop * g_cfg.solver.penetration_slop;
    if (position_error_len_sq <= slop_sq) {
        return; /* Inside slop: no positional correction needed */
    }

    float inv_mass_a = rigidbody_effective_inv_mass(body_a);
    float inv_mass_b = rigidbody_effective_inv_mass(body_b);
    if ((inv_mass_a <= 0.0f) && (inv_mass_b <= 0.0f)) {
        return;
    }

    /* Baumgarte bias velocity (clamped per Catto's stabilized Baumgarte).
     * Scale by solver iterations to match the total per-tick correction of the
     * old 64-iteration version (beta applied per iteration), but applied as a
     * single impulse to avoid energy pumping from repeated applications.
     * Max bias also scaled by iterations to match old per-tick effective cap. */
    const float baumgarte_beta = g_cfg.joints.revolute_beta * (float) g_cfg.timestep.solver_iterations;
    const float max_bias_speed = g_cfg.joints.revolute_max_bias * (float) g_cfg.timestep.solver_iterations;
    float bias_speed = baumgarte_beta * sqrtf(position_error_len_sq) / dt;
    vector3 bias;
    if ((bias_speed > max_bias_speed) && (bias_speed > 0.0f)) {
        bias = vector3_scaling(position_error, (baumgarte_beta / dt) * (max_bias_speed / bias_speed));
    } else {
        bias = vector3_scaling(position_error, baumgarte_beta / dt);
    }

    /* Solve for positional correction impulse */
    float inv_mass_sum = inv_mass_a + inv_mass_b;
    math3 k = {{{0.0f}}};
    for (int i = 0; i < 3; i++) {
        k.matrix[i][i] = inv_mass_sum;
    }
    math3 skew_a = skew_symmetric(r_a);
    math3 skew_b = skew_symmetric(r_b);
    math3 term_a = math3_multiplication(skew_a, math3_multiplication(rigidbody_effective_inv_inertia(body_a), skew_a));
    math3 term_b = math3_multiplication(skew_b, math3_multiplication(rigidbody_effective_inv_inertia(body_b), skew_b));
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            k.matrix[i][j] -= term_a.matrix[i][j];
            k.matrix[i][j] -= term_b.matrix[i][j];
        }
    }
    math3 k_inv = math3_inverse(k);
    /* Only bias velocity (no relative velocity term - that's handled in revolute_solve) */
    vector3 rhs = vector3_scaling(bias, -1.0f);
    vector3 impulse = math3_multiplication_vector3(k_inv, rhs);

    /* Apply positional correction impulse */
    body_a->velocity = vector3_subtraction(body_a->velocity, vector3_scaling(impulse, inv_mass_a));
    body_b->velocity = vector3_addition(body_b->velocity, vector3_scaling(impulse, inv_mass_b));
    body_a->angular_velocity =
        vector3_subtraction(body_a->angular_velocity,
                            math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a),
                                                         vector3_cross(r_a, impulse)));
    body_b->angular_velocity =
        vector3_addition(body_b->angular_velocity,
                         math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b),
                                                      vector3_cross(r_b, impulse)));
}

/* TRUTH: once-per-tick slide tracking (called before the solver loop). */
void prismatic_pre_step(prismatic_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    if ((!p) || (!body_a) || (!body_b) || (!(dt > 0.0f))) {
        return;
    }
    vector3 r_a = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);
    vector3 world_a = vector3_addition(body_a->position, r_a);
    vector3 world_b = vector3_addition(body_b->position, r_b);
    vector3 axis_w = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    float cur = vector3_dot(vector3_subtraction(world_b, world_a), axis_w);
    if (!isfinite(cur)) {
        return;
    }
    if (!p->position_initialized) {
        p->accumulated_position = cur;
        p->position_initialized = true;
    } else {
        /* Track for readout; enforcement uses measured cur (see solve). */
        vector3 vel_a = vector3_addition(body_a->velocity, vector3_cross(body_a->angular_velocity, r_a));
        vector3 vel_b = vector3_addition(body_b->velocity, vector3_cross(body_b->angular_velocity, r_b));
        float rel = vector3_dot(vector3_subtraction(vel_b, vel_a), axis_w);
        if (isfinite(rel)) {
            float d = rel * dt;
            if (d > 1.0f) {
                d = 1.0f;
            } else if (d < -1.0f) {
                d = -1.0f;
            }
            p->accumulated_position += d;
        }
    }
}

/* Rope: inequality distance constraint (pulls only, no push). */
void rope_solve(rope_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) return;
    if (!isfinite(p->rest_length) || p->rest_length < 0.0f) return;

    float inv_a = rigidbody_effective_inv_mass(body_a);
    float inv_b = rigidbody_effective_inv_mass(body_b);
    vector3 r_a = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);
    vector3 world_a = vector3_addition(body_a->position, r_a);
    vector3 world_b = vector3_addition(body_b->position, r_b);
    vector3 delta = vector3_subtraction(world_b, world_a);
    float dist = vector3_length(delta);

    if (dist < 1e-9f) return;

    /* Only pull when stretched beyond rest_length (inequality). Slack ropes
     * transmit nothing: no wake (see revolute_solve error gating). */
    if (dist <= p->rest_length) return;
    if (body_a->is_sleeping) rigidbody_wake(body_a);
    if (body_b->is_sleeping) rigidbody_wake(body_b);

    vector3 n = vector3_scaling(delta, 1.0f / dist);
    float err = dist - p->rest_length;

    vector3 vel_a = vector3_addition(body_a->velocity, vector3_cross(body_a->angular_velocity, r_a));
    vector3 vel_b = vector3_addition(body_b->velocity, vector3_cross(body_b->angular_velocity, r_b));
    float rel_n = vector3_dot(vector3_subtraction(vel_b, vel_a), n);

    /* TRUTH: slop deadband (see revolute_solve). Rope pulls only past
     * rest+slop; inside tolerance there is nothing to enforce. */
    float bias = 0.0f;
    if (err * err > g_cfg.solver.penetration_slop * g_cfg.solver.penetration_slop) {
        bias = g_cfg.joints.revolute_beta * err / dt;
        float max_b = g_cfg.joints.revolute_max_bias;
        if (bias > max_b) bias = max_b;
        else if (bias < -max_b) bias = -max_b;
    }

    vector3 ra_n = vector3_cross(r_a, n);
    vector3 rb_n = vector3_cross(r_b, n);
    float k = inv_a + inv_b +
        vector3_dot(ra_n, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), ra_n)) +
        vector3_dot(rb_n, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), rb_n));
    if (k <= 1e-12f) return;

    float lambda = -(rel_n + bias) / k;
    /* TRUTH: n=(B-A)/dist, impulse=n*lambda, B+=, A-=. Stretched+separating
     * (err>0, rel_n>0) needs lambda<0 (pull B toward A). Old clamp killed
     * exactly pull (lambda<0) and kept push — rope was a strut. Keep pull
     * (negative), kill push (positive). */
    if (lambda > 0.0f) {
        lambda = 0.0f; /* Only pull, never push. */
    }

    vector3 impulse = vector3_scaling(n, lambda);
    body_a->velocity = vector3_subtraction(body_a->velocity, vector3_scaling(impulse, inv_a));
    body_b->velocity = vector3_addition(body_b->velocity, vector3_scaling(impulse, inv_b));
    body_a->angular_velocity = vector3_subtraction(
        body_a->angular_velocity,
        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), vector3_cross(r_a, impulse)));
    body_b->angular_velocity = vector3_addition(
        body_b->angular_velocity,
        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), vector3_cross(r_b, impulse)));
}

/* Prismatic motor: adds drive force to the force accumulator (call once per tick). */
void prismatic_apply_motor(prismatic_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    if ((!p) || (!p->motor_enabled) || (!body_a) || (!body_b) || (!(dt > 0.0f))) {
        return;
    }
    if (!isfinite(p->motor_target_speed) || !isfinite(p->motor_max_force) || p->motor_max_force <= 0.0f) {
        return;
    }
    /* TRUTH: respect slide limits. */
    if (p->limits_enabled && p->position_initialized) {
        vector3 axw0 = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
        vector3 r_a0 = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
        vector3 r_b0 = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);
        float cur0 =
            vector3_dot(vector3_subtraction(vector3_addition(body_b->position, r_b0),
                                            vector3_addition(body_a->position, r_a0)),
                        axw0);
        if ((cur0 <= p->limit_min + 1e-4f && p->motor_target_speed < 0.0f) ||
            (cur0 >= p->limit_max - 1e-4f && p->motor_target_speed > 0.0f)) {
            return;
        }
    }
    vector3 axis_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    if (vector3_length_squared(axis_world) < 1e-12f) {
        return;
    }
    axis_world = vector3_normalisation(axis_world);
    vector3 r_ama = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_amb = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);
    vector3 vel_a = vector3_addition(body_a->velocity, vector3_cross(body_a->angular_velocity, r_ama));
    vector3 vel_b = vector3_addition(body_b->velocity, vector3_cross(body_b->angular_velocity, r_amb));
    float current_speed = vector3_dot(vector3_subtraction(vel_b, vel_a), axis_world);
    float speed_error = p->motor_target_speed - current_speed;
    float motor_gain = g_cfg.joints.revolute_motor_gain;
    float desired_force = speed_error * motor_gain;
    if (desired_force > p->motor_max_force) {
        desired_force = p->motor_max_force;
    }
    if (desired_force < -p->motor_max_force) {
        desired_force = -p->motor_max_force;
    }
    /* TRUTH: no-overshoot clamp, mirroring revolute_apply_motor. P-only
     * explicit Euler pumps energy when gain*dt/m_eff >> 2; bound one tick's
     * velocity change by the remaining error: |F| <= |err|*m_eff_axis/dt. */
    {
        vector3 ra_x = vector3_cross(r_ama, axis_world);
        vector3 rb_x = vector3_cross(r_amb, axis_world);
        float k_axis = rigidbody_effective_inv_mass(body_a) + rigidbody_effective_inv_mass(body_b) +
            vector3_dot(ra_x, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), ra_x)) +
            vector3_dot(rb_x, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), rb_x));
        if ((k_axis > 1e-12f) && (dt > 0.0f)) {
            float max_no_overshoot = fabsf(speed_error) / (dt * k_axis);
            if (desired_force > max_no_overshoot) {
                desired_force = max_no_overshoot;
            } else if (desired_force < -max_no_overshoot) {
                desired_force = -max_no_overshoot;
            }
        }
    }
    vector3 drive_force = vector3_scaling(axis_world, desired_force);
    body_a->force_accumulator = vector3_subtraction(body_a->force_accumulator, drive_force);
    body_b->force_accumulator = vector3_addition(body_b->force_accumulator, drive_force);
}

/* Fixed weld: point-to-point (same K-matrix as revolute) plus full angular
 * lock (kill all relative spin, not just off-axis). Deterministic, no bias
 * beyond the shared Baumgarte cap. */
void fixed_solve(fixed_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    vector3 r_a = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);
    vector3 world_a = vector3_addition(body_a->position, r_a);
    vector3 world_b = vector3_addition(body_b->position, r_b);
    vector3 err = vector3_subtraction(world_b, world_a);
    /* TRUTH: error-gated wake (see revolute_solve). */
    if (vector3_length_squared(err) >
        g_cfg.solver.penetration_slop * g_cfg.solver.penetration_slop) {
        if (body_a->is_sleeping) {
            rigidbody_wake(body_a);
        }
        if (body_b->is_sleeping) {
            rigidbody_wake(body_b);
        }
    }
    float inv_a = rigidbody_effective_inv_mass(body_a);
    float inv_b = rigidbody_effective_inv_mass(body_b);
    if ((inv_a <= 0.0f) && (inv_b <= 0.0f)) {
        return;
    }
    vector3 vel_a = vector3_addition(body_a->velocity, vector3_cross(body_a->angular_velocity, r_a));
    vector3 vel_b = vector3_addition(body_b->velocity, vector3_cross(body_b->angular_velocity, r_b));
    vector3 rel = vector3_subtraction(vel_b, vel_a);
    const float beta = g_cfg.joints.revolute_beta;
    float err_len = vector3_length(err);
    /* TRUTH: slop deadband (see revolute_solve). */
    vector3 bias = ((err_len > 1e-9f) &&
                    (err_len * err_len >
                     g_cfg.solver.penetration_slop * g_cfg.solver.penetration_slop))
        ? vector3_scaling(err, fminf(beta / dt, g_cfg.joints.revolute_max_bias / fmaxf(err_len, 1e-9f)))
        : vector3_zero();
    math3 skew_a = skew_symmetric(r_a);
    math3 skew_b = skew_symmetric(r_b);
    math3 k = {{{0.0f}}};
    float inv_sum = inv_a + inv_b;
    for (int i = 0; i < 3; i++) {
        k.matrix[i][i] = inv_sum;
    }
    math3 term_a = math3_multiplication(skew_a, math3_multiplication(rigidbody_effective_inv_inertia(body_a), skew_a));
    math3 term_b = math3_multiplication(skew_b, math3_multiplication(rigidbody_effective_inv_inertia(body_b), skew_b));
    /* k = inv_sum*I - term_a - term_b */
    for (int c = 0; c < 3; c++) {
        for (int r = 0; r < 3; r++) {
            k.matrix[c][r] -= term_a.matrix[c][r] + term_b.matrix[c][r];
        }
    }
    math3 k_inv = math3_inverse(k);
    vector3 rhs = vector3_scaling(vector3_addition(rel, bias), -1.0f);
    vector3 impulse = math3_multiplication_vector3(k_inv, rhs);
    body_a->velocity = vector3_subtraction(body_a->velocity, vector3_scaling(impulse, inv_a));
    body_b->velocity = vector3_addition(body_b->velocity, vector3_scaling(impulse, inv_b));
    body_a->angular_velocity = vector3_subtraction(
        body_a->angular_velocity,
        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), vector3_cross(r_a, impulse)));
    body_b->angular_velocity = vector3_addition(
        body_b->angular_velocity,
        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), vector3_cross(r_b, impulse)));
    /* Full angular lock: remove all relative spin. */
    vector3 rel_w = vector3_subtraction(body_b->angular_velocity, body_a->angular_velocity);
    math3 m = math3_addition(rigidbody_effective_inv_inertia(body_a), rigidbody_effective_inv_inertia(body_b));
    math3 m_inv = math3_inverse(m);
    vector3 ang_imp = vector3_scaling(math3_multiplication_vector3(m_inv, rel_w), -1.0f);
    body_a->angular_velocity = vector3_subtraction(
        body_a->angular_velocity, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), ang_imp));
    body_b->angular_velocity = vector3_addition(
        body_b->angular_velocity, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), ang_imp));
}

void revolute_apply_motor(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    (void) dt;
    if ((!p) || (!p->motor_enabled) || (!body_a) || (!body_b)) {
        return;
    }
    if (!isfinite(p->motor_target_speed) || !isfinite(p->motor_max_torque) || p->motor_max_torque <= 0.0f) {
        return;
    }
    /* TRUTH: respect angle limits — driving into a limit fights the limit
     * impulse every tick (jitter + energy). Refuse to drive past stops. */
    if (p->limits_enabled && p->angle_initialized) {
        float along_probe;
        {
            vector3 axw = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
            vector3 relw = vector3_subtraction(body_b->angular_velocity, body_a->angular_velocity);
            along_probe = vector3_dot(relw, axw);
        }
        bool at_min = (p->accumulated_angle <= p->limit_min_rad + 1e-4f) && (p->motor_target_speed < 0.0f);
        bool at_max = (p->accumulated_angle >= p->limit_max_rad - 1e-4f) && (p->motor_target_speed > 0.0f);
        if ((at_min && along_probe <= 0.0f) || (at_max && along_probe >= 0.0f)) {
            /* At stop and driving further out: hold, don't push. Allow
             * driving back inward (opposite sign passes through). */
            if ((at_min && p->motor_target_speed < 0.0f) || (at_max && p->motor_target_speed > 0.0f)) {
                return;
            }
        }
    }
    vector3 axis_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    if (vector3_length_squared(axis_world) < 1e-12f) {
        return;
    }
    axis_world = vector3_normalisation(axis_world);
    vector3 relative_angular = vector3_subtraction(body_b->angular_velocity, body_a->angular_velocity);
    float current_speed = vector3_dot(relative_angular, axis_world);
    if (!isfinite(current_speed)) {
        return;
    }
    float speed_error = p->motor_target_speed - current_speed;
    if (!isfinite(speed_error)) {
        return;
    }
    float motor_gain = g_cfg.joints.revolute_motor_gain;
    if (!isfinite(motor_gain) || motor_gain < 0.0f) {
        motor_gain = 8.0f;
    }
    /* TRUTH: P-only with shared gain goes explicit-Euler unstable at
     * gain*dt/I >> 2 (gain=100, I=0.1, dt=1/60 => 16). Clamp torque so the
     * per-tick velocity change cannot exceed the remaining error (no
     * overshoot): |Δw| <= |err|. Δw = torque*dt*inv_eff. */
    if (motor_gain > 50.0f) {
        motor_gain = 50.0f;
    }
    float desired_torque = speed_error * motor_gain;
    if (desired_torque > p->motor_max_torque) {
        desired_torque = p->motor_max_torque;
    }
    if (desired_torque < -p->motor_max_torque) {
        desired_torque = -p->motor_max_torque;
    }
    /* Stability: no overshoot in one tick. */
    {
        float inv_sum = vector3_dot(axis_world,
                                    math3_multiplication_vector3(math3_addition(rigidbody_effective_inv_inertia(body_a),
                                                                                rigidbody_effective_inv_inertia(body_b)),
                                                                 axis_world));
        if (inv_sum > 1e-12f && dt > 0.0f) {
            float max_no_overshoot = fabsf(speed_error) / (dt * inv_sum);
            if (desired_torque > max_no_overshoot) {
                desired_torque = max_no_overshoot;
            } else if (desired_torque < -max_no_overshoot) {
                desired_torque = -max_no_overshoot;
            }
        }
    }
    if (!isfinite(desired_torque)) {
        return;
    }
    vector3 drive_torque = vector3_scaling(axis_world, desired_torque);
    body_a->torque_accumulator = vector3_subtraction(body_a->torque_accumulator, drive_torque);
    body_b->torque_accumulator = vector3_addition(body_b->torque_accumulator, drive_torque);
}

/* Distance: 1D constraint along the anchor axis. Preserves free rotation and
 * tangential motion; only the separation error is corrected. */
void distance_solve(distance_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    if (!isfinite(p->rest_length) || p->rest_length < 0.0f) {
        return;
    }
    float inv_a = rigidbody_effective_inv_mass(body_a);
    float inv_b = rigidbody_effective_inv_mass(body_b);
    vector3 r_a = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);
    vector3 world_a = vector3_addition(body_a->position, r_a);
    vector3 world_b = vector3_addition(body_b->position, r_b);
    vector3 delta = vector3_subtraction(world_b, world_a);
    float dist = vector3_length(delta);
    if (dist < 1e-9f) {
        return;
    }
    /* TRUTH: error-gated wake (see revolute_solve): only a stretched /
     * compressed rod (beyond slop) rejoins sleepers. */
    if (fabsf(dist - p->rest_length) > g_cfg.solver.penetration_slop) {
        if (body_a->is_sleeping) {
            rigidbody_wake(body_a);
        }
        if (body_b->is_sleeping) {
            rigidbody_wake(body_b);
        }
    }
    vector3 n = vector3_scaling(delta, 1.0f / dist);
    float err = dist - p->rest_length;
    vector3 vel_a = vector3_addition(body_a->velocity, vector3_cross(body_a->angular_velocity, r_a));
    vector3 vel_b = vector3_addition(body_b->velocity, vector3_cross(body_b->angular_velocity, r_b));
    float rel_n = vector3_dot(vector3_subtraction(vel_b, vel_a), n);
    /* TRUTH: slop deadband (see revolute_solve). */
    float bias = 0.0f;
    if (err * err > g_cfg.solver.penetration_slop * g_cfg.solver.penetration_slop) {
        bias = g_cfg.joints.revolute_beta * err / dt;
        float max_b = g_cfg.joints.revolute_max_bias;
        if (bias > max_b) {
            bias = max_b;
        } else if (bias < -max_b) {
            bias = -max_b;
        }
    }
    vector3 ra_n = vector3_cross(r_a, n);
    vector3 rb_n = vector3_cross(r_b, n);
    float k = inv_a + inv_b +
        vector3_dot(ra_n, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), ra_n)) +
        vector3_dot(rb_n, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), rb_n));
    if (k <= 1e-12f) {
        return;
    }
    float lambda = -(rel_n + bias) / k;
    vector3 impulse = vector3_scaling(n, lambda);
    body_a->velocity = vector3_subtraction(body_a->velocity, vector3_scaling(impulse, inv_a));
    body_b->velocity = vector3_addition(body_b->velocity, vector3_scaling(impulse, inv_b));
    body_a->angular_velocity = vector3_subtraction(
        body_a->angular_velocity,
        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), vector3_cross(r_a, impulse)));
    body_b->angular_velocity = vector3_addition(
        body_b->angular_velocity,
        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), vector3_cross(r_b, impulse)));
}

/* TRUTH: fixed-weld angular positional correction (once per tick, AFTER the
 * velocity loop — never inside, same energy-pump rule as revolute axis
 * drift). Velocity-only lock kills relative spin but lets orientation error
 * integrate (weld flexes over seconds). This Baumgarte drives the relative
 * quaternion error q_err = q_a^-1 * q_b toward identity with angular impulse,
 * proportional to the rotation vector (axis*sin(half-angle) scaled). */
void fixed_correct_angular_drift(fixed_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    (void) p;
    if ((!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    vector4 q_a_inv =
        (vector4){body_a->orientation.w, -body_a->orientation.x, -body_a->orientation.y, -body_a->orientation.z};
    vector4 q_err = vector4_multiplication(q_a_inv, body_b->orientation);
    q_err = vector4_normalisation(q_err);
    /* Shortest-arc: w<0 flips to the complementary rotation. */
    if (q_err.w < 0.0f) {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }
    vector3 err_vec = {q_err.x, q_err.y, q_err.z};
    if (vector3_length_squared(err_vec) < 1e-12f) {
        return;
    }
    const float beta = 0.1f;
    /* Correction angular velocity in A-frame, rotated to world via A. */
    vector3 corr_local = vector3_scaling(err_vec, 2.0f * beta / dt);
    vector3 corr_world = vector4_rotate_to_vector3(body_a->orientation, corr_local);
    math3 m = math3_addition(rigidbody_effective_inv_inertia(body_a),
                             rigidbody_effective_inv_inertia(body_b));
    /* Guard near-singular (both infinite mass): nothing to correct. */
    math3 m_inv = math3_inverse(m);
    vector3 impulse = vector3_scaling(math3_multiplication_vector3(m_inv, corr_world), -1.0f);
    if (!body_a->static_state) {
        body_a->angular_velocity = vector3_subtraction(
            body_a->angular_velocity,
            math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), impulse));
    }
    if (!body_b->static_state) {
        body_b->angular_velocity = vector3_addition(
            body_b->angular_velocity,
            math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), impulse));
    }
}

/* Positional axis-drift correction: MUST be called exactly once per tick,
 * AFTER the velocity iteration loop — never inside it. The error term is
 * positional (orientation difference, unchanged by velocity iterations),
 * so per-iteration application multiplies the correction by the iteration
 * count (64x at defaults): a spurious torsional spring that pumps energy
 * and destroys hinge truth (e.g. 9x-too-fast pendulum). */
void revolute_correct_axis_drift(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    /* Use reference axes (stored at init) as the target orientation for each body's hinge axis.
     * Compare each body's CURRENT hinge axis with its REFERENCE axis (stored at init).
     * This anchors each body's hinge axis to its original world-space direction,
     * preventing drift even when both bodies tilt together. */
    vector3 ref_axis_a = p->reference_axis_a;
    vector3 ref_axis_b = p->reference_axis_b;
    /* Fallback to current orientation if reference axes not initialized (shouldn't happen). */
    if (vector3_length_squared(ref_axis_a) < 1e-12f) {
        ref_axis_a = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    }
    if (vector3_length_squared(ref_axis_b) < 1e-12f) {
        vector3 hinge_b = (vector3_length_squared(p->axis_b) > 1e-12f) ? vector3_normalisation(p->axis_b)
                                                                       : vector3_normalisation(p->axis_a);
        ref_axis_b = vector4_rotate_to_vector3(body_b->orientation, hinge_b);
    }
    /* Current hinge axes in world space */
    vector3 axis_a_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    vector3 axis_b_world = vector4_rotate_to_vector3(body_b->orientation,
        (vector3_length_squared(p->axis_b) > 1e-12f) ? vector3_normalisation(p->axis_b)
                                                     : vector3_normalisation(p->axis_a));
    /* Axis error: cross product of current axis with reference axis.
     * This gives the rotation vector needed to align current axis with reference axis. */
    vector3 axis_error_a = vector3_cross(axis_a_world, ref_axis_a);
    vector3 axis_error_b = vector3_cross(axis_b_world, ref_axis_b);
    float axis_error_len_sq_a = vector3_length_squared(axis_error_a);
    float axis_error_len_sq_b = vector3_length_squared(axis_error_b);
    const float axis_baumgarte_beta = 0.1f; /* MFS_127: reduced from 0.2 to reduce oscillation */
    const float axis_correction_scale = axis_baumgarte_beta / dt;
    math3 drift_angular_mass = math3_addition(rigidbody_effective_inv_inertia(body_a),
                                              rigidbody_effective_inv_inertia(body_b));
    math3 drift_angular_mass_inv = math3_inverse(drift_angular_mass);
    /* Correct body A's axis drift */
    if (axis_error_len_sq_a > 0.000001f) {
        vector3 axis_correction_a = vector3_scaling(axis_error_a, axis_correction_scale);
        vector3 axis_impulse_a =
            vector3_scaling(math3_multiplication_vector3(drift_angular_mass_inv, axis_correction_a), -1.0f);
        if (!body_a->static_state) {
            body_a->angular_velocity = vector3_subtraction(
                body_a->angular_velocity,
                math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), axis_impulse_a));
        }
        if (!body_b->static_state) {
            body_b->angular_velocity = vector3_addition(
                body_b->angular_velocity,
                math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), axis_impulse_a));
        }
    }
    /* Correct body B's axis drift */
    if (axis_error_len_sq_b > 0.000001f) {
        vector3 axis_correction_b = vector3_scaling(axis_error_b, axis_correction_scale);
        vector3 axis_impulse_b =
            vector3_scaling(math3_multiplication_vector3(drift_angular_mass_inv, axis_correction_b), -1.0f);
        if (!body_a->static_state) {
            body_a->angular_velocity = vector3_subtraction(
                body_a->angular_velocity,
                math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), axis_impulse_b));
        }
        if (!body_b->static_state) {
            body_b->angular_velocity = vector3_addition(
                body_b->angular_velocity,
                math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), axis_impulse_b));
        }
    }
}