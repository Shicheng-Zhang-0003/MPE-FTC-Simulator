#ifndef boundary_h
#define boundary_h
#include "../core/math3d.h"
#include "../core/rigidbody.h"

void boundary_apply_floor(rigidbody *rigid_body, float floor_y_coordinate);
void boundary_apply_box(rigidbody *rigid_body, vector3 minimum_bounds, vector3 maximum_bounds);
/* TRUTH: every safety-net trip is counted (never silent). A nonzero count
 * means the solver lost a body past emergency slop that tick. */
int boundary_trip_count(void);
void boundary_trip_reset(void);
#endif
