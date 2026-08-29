/* MPE_FTC_076: Wheel traction raycast */
#ifndef wheel_traction_h
#define wheel_traction_h
#include "../core/physics_world.h"

/* Raycast down from wheel, apply drive force at ground contact.
 * wheel_forward = unit vector in wheel's driving direction (chassis-local).
 * Returns true if ground was hit and force was applied. */
bool wheel_traction_apply(physics_world *world, int wheel_body_index, int ground_body_index,
                          vector3 wheel_forward_world, float drive_force_magnitude);

#endif /* wheel_traction_h */
