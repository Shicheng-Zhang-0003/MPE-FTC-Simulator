/* MPE_FTC_076: Wheel traction raycast implementation
 *
 * Raycasts downward from the wheel center. If it hits the ground (or any
 * static body), applies a horizontal drive force at the contact point in
 * the wheel's forward direction. This augments contact friction and makes
 * drivetrains more stable.
 *
 * Simplifications:
 *   - Only checks against one ground body (the first static body found).
 *   - No suspension model (wheel is assumed to be at the correct height).
 *   - Force is applied to the wheel body, not the chassis.
 */
#include "wheel_traction.h"
#include <math.h>

#define TRACTION_RAY_LENGTH 2.0f
#define TRACTION_RAY_OFFSET 0.5f

static bool ray_sphere_intersect(vector3 ray_origin, vector3 ray_dir, rigidbody *sphere, float *t_hit) {
    vector3 oc = vector3_subtraction(ray_origin, sphere->position);
    float a = vector3_dot(ray_dir, ray_dir);
    float b = 2.0f * vector3_dot(oc, ray_dir);
    float c = vector3_dot(oc, oc) - sphere->radius * sphere->radius;
    float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f) {
        return false;
    }
    float sqrt_disc = sqrtf(discriminant);
    float t1 = (-b - sqrt_disc) / (2.0f * a);
    float t2 = (-b + sqrt_disc) / (2.0f * a);
    *t_hit = (t1 > 0.0f) ? t1 : t2;
    return (*t_hit > 0.0f);
}

bool wheel_traction_apply(physics_world *world, int wheel_body_index, int ground_body_index,
                          vector3 wheel_forward_world, float drive_force_magnitude) {
    if ((!world) || (wheel_body_index < 0) || (wheel_body_index >= world->body_count)) {
        return false;
    }
    if ((ground_body_index < 0) || (ground_body_index >= world->body_count)) {
        return false;
    }

    rigidbody *wheel = &world->bodies[wheel_body_index];
    rigidbody *ground = &world->bodies[ground_body_index];

    /* Raycast down from wheel center */
    vector3 ray_origin = wheel->position;
    ray_origin.y += TRACTION_RAY_OFFSET;
    vector3 ray_dir = {0.0f, -1.0f, 0.0f};

    float t_hit = 0.0f;
    bool hit = false;
    if (ground->type == object_sphere) {
        hit = ray_sphere_intersect(ray_origin, ray_dir, ground, &t_hit);
    } else if (ground->type == object_cube) {
        /* For cubes, use a simplified plane test (y = ground top) */
        float ground_top_y = ground->position.y + ground->half_extensions.y;
        if (ray_origin.y > ground_top_y) {
            t_hit = ray_origin.y - ground_top_y;
            hit = true;
        }
    }

    if ((!hit) || (t_hit > TRACTION_RAY_LENGTH)) {
        return false;
    }

    /* Apply drive force at the wheel in the forward direction */
    vector3 drive_force = vector3_scaling(wheel_forward_world, drive_force_magnitude);
    rb_apply_forces_perfect(wheel, drive_force);

    /* Wake the wheel so the force is integrated */
    rigidbody_wake(wheel);

    return true;
}
