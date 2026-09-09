#include "../mpe_engine.h"
#include "scene_load.h"
#include "scene_init.h"
#include "scene_id_remap.h" /* MPE_FTC_058 */
#include "../physics/spring_joint.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
static int read_float(FILE *f, float *v) {
    return fread(v, sizeof(float), 1, f) == 1;
}
static int read_int(FILE *f, int32_t *v) {
    return fread(v, sizeof(int32_t), 1, f) == 1;
}
static int read_vec3(FILE *f, vector3 *v) {
    return fread(v, sizeof(vector3), 1, f) == 1;
}
static int read_vec4(FILE *f, vector4 *v) {
    return fread(v, sizeof(vector4), 1, f) == 1;
}
int scene_loading(const char *file_source_path) {
    FILE *f = fopen(file_source_path, "rb");
    if (!f) {
        fprintf(stderr, "Error LDF01: Could not open %s\n", file_source_path);
        return 0;
    }
    int32_t magic, version, count;
    if ((!read_int(f, &magic)) || (magic != mpe_magic)) {
        fprintf(stderr, "Error LDF02: Invalid magic number\n");
        fclose(f);
        return 0;
    }
    if ((!read_int(f, &version)) || (version != mpe_version && version != 130 && version != 140)) {
        fprintf(stderr, "Error LDF03: Version mismatch\n");
        fclose(f);
        return 0;
    }
    if ((!read_int(f, &count)) || (count < 0)) {
        fclose(f);
        return 0;
    }
	    /* R3-02: Read into staging buffer first. Only commit after full validation. */
    if (count > mpe_max_bodies) {
        count = mpe_max_bodies;
    }

    /* Staging buffer: holds raw body data until all reads succeed */
    typedef struct {
        int32_t type_int;
        float mass;
        float radius;
        vector3 half_extensions;
        vector3 position;
        vector3 velocity;
        vector3 angular_velocity;
        vector4 orientation;
        vector3 colour;
        float restitution;
        float friction_static;
        float friction_kinetic;
        int32_t static_int;
        int32_t saved_object_id;
        float cylinder_half_length;
    } staged_body;

    static staged_body staging[mpe_max_bodies];
    int staged_count = 0;

    for (int i = 0; i < count; i++) {
        staged_body *sb = &staging[i];
        sb->saved_object_id = 0;
        sb->cylinder_half_length = 0.0f;

        if (!read_int(f, &sb->type_int)) break;
        /* MFS_310: Reject unknown object types instead of silently
         * converting them to spheres. Corrupted scene data should
         * fail the load, not produce garbage objects. */
        if ((sb->type_int != object_sphere) &&
            (sb->type_int != object_cube) &&
            (sb->type_int != object_cylinder)) {
            fprintf(stderr, "Error LDF05: Unknown object type %d at body %d. Load aborted.\n",
                    sb->type_int, i);
            fclose(f);
            return 0;
        }
        if (!read_float(f, &sb->mass)) break;
        if (!read_float(f, &sb->radius)) break;
        if (!read_vec3(f, &sb->half_extensions)) break;
        if (!read_vec3(f, &sb->position)) break;
        if (!read_vec3(f, &sb->velocity)) break;
        if (!read_vec3(f, &sb->angular_velocity)) break;
        if (!read_vec4(f, &sb->orientation)) break;
        if (!read_vec3(f, &sb->colour)) break;
        if (!read_float(f, &sb->restitution)) break;
        if (!read_float(f, &sb->friction_static)) break;
        if (!read_float(f, &sb->friction_kinetic)) break;
        if (!read_int(f, &sb->static_int)) break;

        if (version >= 150) {
            if (!read_int(f, &sb->saved_object_id)) break;
        }

        /* MFS_203_R304_LOAD: read cylinder_half_length */
        {
            float cyl_half = 0.0f;
            if (read_float(f, &cyl_half)) {
                sb->cylinder_half_length = cyl_half;
            }
        }

        staged_count++;
    }

    /* R3-02: If we didn't read all expected bodies, the file is corrupt.
     * Do NOT clear the live scene. Return failure. */
    if (staged_count < count) {
        fprintf(stderr, "Error LDF04: Scene file truncated at body %d/%d. Live scene preserved.\n",
                staged_count, count);
        fclose(f);
        return 0;
    }

    /* All reads succeeded. Now safe to clear and commit. */
    scene_clear();
    scene_id_remap_reset(); /* MPE_FTC_058 */
    contact_cache_clear(NULL); /* A3_PATCH_22_SCENE_LOAD_RESET */
    contact_cache_clear(physics_world_get_primary()); /* MFS_131 */
    joint_init_pool();

    if (!scene_ensure_pool_capacity(staged_count)) {
        fclose(f);
        return 0;
    }

    int loaded_count = 0;
    for (int i = 0; i < staged_count; i++) {
        staged_body *sb = &staging[i];
        object_type type = (object_type) sb->type_int;

        if (type == object_cube) {
            rigidbody_initialisation_cube(&obj_per_scene[i], sb->position, sb->half_extensions, sb->mass);
        } else if (type == object_cylinder) {
            rigidbody_initialisation_cylinder(&obj_per_scene[i], sb->radius, sb->cylinder_half_length, sb->mass, sb->position);
        } else {
            rigidbody_initialisation_sphere(&obj_per_scene[i], sb->radius, sb->mass, sb->position);
        }

        obj_per_scene[i].type = type;
        obj_per_scene[i].velocity = sb->velocity;
        obj_per_scene[i].angular_velocity = sb->angular_velocity;
        obj_per_scene[i].orientation = vector4_normalisation(sb->orientation);
        obj_per_scene[i].colour = sb->colour;
        obj_per_scene[i].restitution = sb->restitution;
        obj_per_scene[i].friction_static = sb->friction_static;
        obj_per_scene[i].friction_kinetic = sb->friction_kinetic;
        obj_per_scene[i].static_state = (sb->static_int != 0);

        if (obj_per_scene[i].static_state) {
            rigidbody_set_static(&obj_per_scene[i], true);
        } else {
            rigidbody_set_static(&obj_per_scene[i], false);
        }

        rigidbody_sanitize(&obj_per_scene[i]);
        rigidbody_update_axes(&obj_per_scene[i]);

        obj_per_scene[i].object_id = scene_allocate_object_id();
        if ((version >= 150) && (sb->saved_object_id > 0)) {
            scene_id_remap_add((uint32_t) sb->saved_object_id, obj_per_scene[i].object_id);
        } /* MPE_FTC_058 */

        obj_per_scene[i].cylinder_half_length = sb->cylinder_half_length;
        obj_per_scene[i].object_generation = 1;
        loaded_count++;
    }

    object_count = loaded_count; /* A3_PATCH_42_CRITICAL_LIFECYCLE */
    int32_t active_joints = 0;
    if (read_int(f, &active_joints) && (active_joints > 0)) {
        for (int j = 0; j < active_joints; j++) {
            int32_t id_a, id_b;
            float eq, k, c;
            if (!read_int(f, &id_a))
                break;
            if (!read_int(f, &id_b))
                break;
            if (!read_float(f, &eq))
                break;
            if (!read_float(f, &k))
                break;
            if (!read_float(f, &c))
                break;
            add_joint_by_ids(scene_id_remap_resolve((uint32_t) id_a), scene_id_remap_resolve((uint32_t) id_b), eq, k,
                             c); /* MPE_FTC_058 */
        }
    }
    fclose(f);
    return 1;
}
