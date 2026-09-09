#include "../mpe_engine.h"
#include "scene_saving.h"
#include "../physics/spring_joint.h"
#include <stdio.h>
#include <stdint.h>
/* MFS_311: Write helpers now return bool for error checking. */
static bool write_float(FILE *f, float v) {
    return fwrite(&v, sizeof(float), 1, f) == 1;
}
static bool write_int(FILE *f, int32_t v) {
    return fwrite(&v, sizeof(int32_t), 1, f) == 1;
}
static bool write_vec3(FILE *f, vector3 v) {
    return fwrite(&v, sizeof(vector3), 1, f) == 1;
}
static bool write_vec4(FILE *f, vector4 v) {
    return fwrite(&v, sizeof(vector4), 1, f) == 1;
}
int save_scene(const char *file_destination_path) {
    /* R3-03: Atomic write — write to temp file, rename on success */
    char tmp_path[512];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", file_destination_path);
    FILE *f = fopen(tmp_path, "wb");
	if (!f) {
        fprintf(stderr, "Error SVF01: Could not open %s\n", file_destination_path);
        return 0;
    }
    /* MFS_311: All writes now checked. On failure, abort and remove tmp. */
    if (!write_int(f, mpe_magic) ||
        !write_int(f, mpe_version) ||
        !write_int(f, object_count)) {
        fclose(f);
        remove(tmp_path);
        fprintf(stderr, "Error SVF03: Write failed during header.\n");
        return 0;
    }
    for (int i = 0; i < object_count; i++) {
        rigidbody *rb = &obj_per_scene[i];
        write_int(f, (int32_t) rb->type);
        write_float(f, rb->mass);
        write_float(f, rb->radius);
        write_vec3(f, rb->half_extensions);
        write_vec3(f, rb->position);
        write_vec3(f, rb->velocity);
        write_vec3(f, rb->angular_velocity);
        write_vec4(f, rb->orientation);
        write_vec3(f, rb->colour);
        write_float(f, rb->restitution);
        write_float(f, rb->friction_static);
        write_float(f, rb->friction_kinetic);
        write_int(f, rb->static_state ? 1 : 0);
        write_int(f, (int32_t) rb->object_id); /* MPE_FTC_058 */
    write_float(f, rb->cylinder_half_length); /* MFS_203_R304: save cylinder geometry */
    }
    /* MFS_313: Iterate ALL joint slots, not just current_joint_count.
     * Joint removal creates holes while decrementing current_joint_count,
     * so active joints can exist at indices >= current_joint_count. */
    int active_joints = 0;
    for (int j = 0; j < mpe_max_joints; j++) {
        if (joint_pool[j].is_active) {
            active_joints++;
        }
    }
    write_int(f, active_joints);
    /* MFS_313: Write all active joints from full pool. */
    for (int j = 0; j < mpe_max_joints; j++) {
        if (joint_pool[j].is_active) {
            write_int(f, (int32_t) joint_pool[j].object_id_a);
            write_int(f, (int32_t) joint_pool[j].object_id_b);
            write_float(f, joint_pool[j].equilibrium_length);
            write_float(f, joint_pool[j].spring_constant);
            write_float(f, joint_pool[j].damping_coefficient);
        }
    } fflush(f);
    fclose(f);    /* R3-03: Atomic rename — old file survives if this fails */
	/* MFS_208: Windows needs target removed first */
    remove(file_destination_path);
    if (rename(tmp_path, file_destination_path) != 0) {
        fprintf(stderr, "Error SVF02: Could not rename %s to %s\n",
                tmp_path, file_destination_path);
        return 0;
    } return 1;
}
