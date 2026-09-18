#ifndef wireframe_h
#define wireframe_h
#include <epoxy/gl.h>
#include "../core/math3d.h"
#include "../core/math4_special.h"
#include "../core/rigidbody.h"

void wireframe_render_selected_object(GLuint shader_program, math4 view_matrix, math4 projection_matrix);
void wireframe_render_object(GLuint shader_program, math4 view_matrix, math4 projection_matrix, rigidbody *rigid_body,
                             vector3 wireframe_colour);
/* FIX-STICK: orange nose marker on every registered FTC robot's +Z face so
 * heading reads at a glance (robot-relative driving needs orientation).
 * Render-only: no bodies, no physics, no-ops with zero robots. */
void wireframe_render_robot_noses(GLuint shader_program, math4 view_matrix, math4 projection_matrix);
#endif
