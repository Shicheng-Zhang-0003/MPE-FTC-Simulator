/* MPE headless smoke: drop a sphere + cube, step N ticks, report finiteness.
 * Build with `-Dmpe_headless` via `make headless` (see makefile).
 * FIX-AUDIT: the guard used to be lowercase `mpe_headless` while the comment
 * promised `-DMPE_HEADLESS` and no `headless` target existed, so the file
 * compiled to nothing and linked inertly. Guard, comment, and makefile
 * target now agree (lowercase mpe_ convention, matching every test). */
#ifdef mpe_headless
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "config/mpe_config.h"
#include "core/physics_world.h"

int main(int argc, char *argv[]) {
    long ticks = 3600;
    if (argc > 1) {
        char *end = NULL;
        long parsed = strtol(argv[1], &end, 10);
        /* FIX-AUDIT: unchecked atoi used to accept negatives (zero-iteration
         * PASS) and garbage (silent default). Clamp to a sane range. */
        if (end != argv[1] && *end == '\0' && parsed >= 0 && parsed <= 1000000) {
            ticks = parsed;
        } else {
            fprintf(stderr, "[headless] invalid tick count '%s', using %ld\n", argv[1], ticks);
        }
    }
    (void) argc; /* tick count is optional; no other arguments exist */
    mpe_config_init();
    physics_world world;
    physics_world_init(&world);
    physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 5.0f, 0.0f});
    physics_world_add_cube(&world, (vector3){2.0f, 5.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 2.0f);
    const float dt = 1.0f / 60.0f;
    for (long t = 0; t < ticks; t++) {
        physics_world_step(&world, dt);
    }
    int invalid = 0;
    for (int i = 0; i < world.body_count; i++) {
        rigidbody *rb = &world.bodies[i];
        if ((!isfinite(rb->position.x)) || (!isfinite(rb->position.y)) || (!isfinite(rb->position.z))) {
            invalid++;
        }
    }
    printf("[headless] ticks=%ld bodies=%d invalid=%d result=%s\n", ticks, world.body_count, invalid,
           (invalid == 0) ? "PASS" : "FAIL");
    physics_world_cleanup(&world);
    return (invalid == 0) ? 0 : 1;
}
#endif /* mpe_headless */
