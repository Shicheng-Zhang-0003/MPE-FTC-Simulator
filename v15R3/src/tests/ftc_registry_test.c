/* FTC registry spawn test (GUI path without the GUI).
 * Regression: the engine ran fine but robots never spawned — the terminal
 * rejected `touch robot` and no key bound gui_robot_spawn, so the registry
 * stayed empty forever. This test drives the exact registry entry points
 * the UI now calls: spawn twice (field walls ensured once), drive via the
 * registry, tick motors, step the world, verify motion + count. */
#ifdef mpe_ftc_registry_test
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "robotics/gui_robot_registry.h"
#include "robotics/drivetrain.h"

int main(void) {
    mpe_config_init();
    physics_world *world = physics_world_get_primary();
    physics_world_init(world);
    constraint_pool_init(world);

    int first_count = world->body_count;
    int idx0 = gui_robot_spawn(0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_19_2);
    if (idx0 != 0) {
        printf("[FAIL] first gui_robot_spawn returned %d (want 0)\n", idx0);
        return 1;
    }
    if (gui_robot_get_count() != 1) {
        printf("[FAIL] registry count %d (want 1)\n", gui_robot_get_count());
        return 1;
    }
    /* 4 field walls + 5 robot bodies per spawn. */
    if (world->body_count < first_count + 9) {
        printf("[FAIL] bodies %d, want >= %d (walls + robot)\n", world->body_count, first_count + 9);
        return 1;
    }

    /* Second spawn must reuse the existing field (no duplicate walls).
     * Offset laterally: identical spawn points would interpenetrate. */
    int bodies_before_second = world->body_count;
    int idx1 = gui_robot_spawn(1.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_19_2);
    if (idx1 != 1) {
        printf("[FAIL] second gui_robot_spawn returned %d (want 1)\n", idx1);
        return 1;
    }
    if (world->body_count != bodies_before_second + 5) {
        printf("[FAIL] second spawn added %d bodies (want exactly 5, no new walls)\n",
               world->body_count - bodies_before_second);
        return 1;
    }

    /* Drive robot 0 forward through the registry path and verify motion. */
    float sx, sy, sz;
    ftc_robot_get_position(world, gui_robot_get(0), &sx, &sy, &sz);
    const float dt = 1.0f / 60.0f;
    for (int t = 0; t < 120; t++) {
        gui_robot_apply_drive(1.0f, 0.0f, 0.0f);
        gui_robot_tick(dt);
        physics_world_step(world, dt);
        for (int i = 0; i < world->body_count; i++) {
            rigidbody *rb = &world->bodies[i];
            if (!isfinite(rb->position.x) || !isfinite(rb->position.y) || !isfinite(rb->position.z)) {
                printf("[FAIL] NaN in body %d at tick %d\n", i, t);
                return 1;
            }
        }
    }
    float ex, ey, ez;
    ftc_robot_get_position(world, gui_robot_get(0), &ex, &ey, &ez);
    float moved = sqrtf((ex - sx) * (ex - sx) + (ez - sz) * (ez - sz));
    printf("[info] registry robot drove %.4f m\n", moved);
    if (moved < 0.5f) {
        printf("[FAIL] registry robot barely moved (%.4f m)\n", moved);
        return 1;
    }
    if (gui_robot_get(99) != NULL) {
        printf("[FAIL] out-of-range registry index did not return NULL\n");
        return 1;
    }
    printf("[PASS] registry spawn/drive/count works\n");
    physics_world_cleanup(world);
    return 0;
}
#endif /* mpe_ftc_registry_test */
