#include "mpe_engine.h"
#include "core/validation_report.h"
#include "physics/depenetration.h"
#include "core/long_run_validation.h"
#include <gtk/gtk.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h> /* MPE_TASK_39 access() */
//World Status right now
frame_timer main_timer;
rigidbody *obj_per_scene = NULL;
int object_count = 0;
int object_capacity = 0;
//World Physics Globals

int debug_last_object_count = 0;
int debug_last_broadphase_pair_count = 0;
int debug_last_manifold_count = 0;
float debug_last_frame_time = 0.0f;
/* MPE_TASK_12_SLEEPING_COUNT_GLOBAL_BEGIN */
int debug_last_sleeping_object_count = 0;
/* MPE_TASK_12_SLEEPING_COUNT_GLOBAL_END */
/* MPE_TASK_09_MANIFOLD_OVERFLOW_COUNTER_BEGIN */
int debug_last_manifold_overflow_count = 0;
/* MPE_TASK_09_MANIFOLD_OVERFLOW_COUNTER_END */




/* MPE_TASK_20A_DEPENETRATION_HELPERS_BEGIN */
/* MPE_TASK_20A_DEPENETRATION_HELPERS_END */

gboolean physics_step_increment(gpointer user_data_pointer) {
    GtkWidget *parent_window = NULL;
    if (user_data_pointer) {
        parent_window = gtk_widget_get_toplevel(GTK_WIDGET(user_data_pointer));
    }
    if (editor_dialog_is_active()) {
        return TRUE;
    }
    /* MPE_TASK_V15R2_PHYSICS_HALT_CHECK_BEGIN */
    if (physics_halt_tick_update()) {
        gtk_widget_queue_draw(GTK_WIDGET(user_data_pointer));
        overlay_update();
        return TRUE;
    }
    /* MPE_TASK_V15R2_PHYSICS_HALT_CHECK_END */
    /* MPE_TASK_18_MODE_WATCH_BEGIN */
    static bool a3_previous_debug_mode_state = false;
    static bool a3_debug_mode_watch_ready = false;
    if (!a3_debug_mode_watch_ready) {
        a3_debug_mode_watch_ready = true;
        a3_previous_debug_mode_state = main_inputs.is_debug_mode_active;
    } else if (main_inputs.is_debug_mode_active != a3_previous_debug_mode_state) {
        a3_previous_debug_mode_state = main_inputs.is_debug_mode_active;
        debug_terminal_sync_mode();
    }
    /* MPE_TASK_18_MODE_WATCH_END */
    if (main_inputs.is_debug_mode_active) {
        if (main_inputs.left_arrow_pressed) {
            g_cfg.ui.change_rate_debug -= 0.01f;
            main_inputs.left_arrow_pressed = false;
        }
        if (main_inputs.right_arrow_pressed) {
            g_cfg.ui.change_rate_debug += 0.01f;
            main_inputs.right_arrow_pressed = false;
        }
    } else {
        if (main_inputs.left_arrow_pressed) {
            g_cfg.ui.change_rate_game -= 0.2f;
            main_inputs.left_arrow_pressed = false;
        }
        if (main_inputs.right_arrow_pressed) {
            g_cfg.ui.change_rate_game += 0.2f;
            main_inputs.right_arrow_pressed = false;
        }
    }
    static int status_dir_checked = 0;
    if (!status_dir_checked) {
        mkdir("status", 0755);
        status_dir_checked = 1;
    }
    frame_timer_update(&main_timer);
    float frame_delta_time = main_timer.delta_time;
    debug_last_frame_time = frame_delta_time;
    //Camera Movements
    if (!main_inputs.is_debug_mode_active) {
        if (main_inputs.w_key_pressed) {
            camera_move_forward(&main_camera_fov, frame_delta_time);
        }
        if (main_inputs.a_key_pressed) {
            camera_move_left(&main_camera_fov, frame_delta_time);
        }
        if (main_inputs.s_key_pressed) {
            camera_move_backward(&main_camera_fov, frame_delta_time);
        }
        if (main_inputs.d_key_pressed) {
            camera_move_right(&main_camera_fov, frame_delta_time);
        }
    } //Perspective Steering
    float perspective_steering_sensitivity = g_cfg.camera.steer_sensitivity; /* MPE_TASK_32 */
    if (main_inputs.is_mouse_locked) {
        main_camera_fov.yaw += main_inputs.mouse_delta_x * perspective_steering_sensitivity;
        main_camera_fov.pitch += main_inputs.mouse_delta_y * perspective_steering_sensitivity;
        main_inputs.mouse_delta_x = 0.0f;
        main_inputs.mouse_delta_y = 0.0f;
    } //IJKL Emulation (Debug Mode)
    if (main_inputs.is_debug_mode_active) {
        float debug_speed = main_camera_fov.movement_speed * frame_delta_time;
        if (main_inputs.w_key_pressed) {
            main_camera_fov.position = vector3_addition(main_camera_fov.position,
                                                        vector3_scaling(main_camera_fov.forward_vector, debug_speed));
        }
        if (main_inputs.s_key_pressed) {
            main_camera_fov.position = vector3_subtraction(
                main_camera_fov.position, vector3_scaling(main_camera_fov.forward_vector, debug_speed));
        }
        if (main_inputs.a_key_pressed) {
            main_camera_fov.position = vector3_subtraction(main_camera_fov.position,
                                                           vector3_scaling(main_camera_fov.side_vector, debug_speed));
        }
        if (main_inputs.d_key_pressed) {
            main_camera_fov.position =
                vector3_addition(main_camera_fov.position, vector3_scaling(main_camera_fov.side_vector, debug_speed));
        }
        if (main_inputs.space_key_pressed) {
            main_camera_fov.position.y += debug_speed;
        }
        /* MPE_TASK_22_SHIFT_DOWN_BEGIN */
        if (main_inputs.shift_key_pressed) {
            main_camera_fov.position.y -= debug_speed;
        }
        /* MPE_TASK_22_SHIFT_DOWN_END */
        float ijkl_speed = g_cfg.camera.ijkl_speed * frame_delta_time; /* MPE_TASK_32 */
        if (main_inputs.i_key_pressed) {
            main_camera_fov.pitch += ijkl_speed;
        }
        if (main_inputs.k_key_pressed) {
            main_camera_fov.pitch -= ijkl_speed;
        }
        if (main_inputs.j_key_pressed) {
            main_camera_fov.yaw -= ijkl_speed;
        }
        if (main_inputs.l_key_pressed) {
            main_camera_fov.yaw += ijkl_speed;
        }
    }
    if (main_camera_fov.pitch > 89.0f) {
        main_camera_fov.pitch = 89.0f;
    }
    if (main_camera_fov.pitch < -89.0f) {
        main_camera_fov.pitch = -89.0f;
    }
    camera_update_vectors(&main_camera_fov);
    //Character Logic
    if (!main_inputs.is_debug_mode_active) {
        float horizontal_friction = g_cfg.camera.horizontal_friction; /* MPE_TASK_32 */
        main_camera_fov.horizontal_velocity.x -=
            main_camera_fov.horizontal_velocity.x * horizontal_friction * frame_delta_time;
        main_camera_fov.horizontal_velocity.z -=
            main_camera_fov.horizontal_velocity.z * horizontal_friction * frame_delta_time;
        main_camera_fov.position.x += main_camera_fov.horizontal_velocity.x * frame_delta_time;
        main_camera_fov.position.z += main_camera_fov.horizontal_velocity.z * frame_delta_time;
        main_camera_fov.vertical_velocity += g_cfg.world.gravity * frame_delta_time;
        main_camera_fov.position.y += main_camera_fov.vertical_velocity * frame_delta_time;
        if (main_camera_fov.position.y <= 2.0f) {
            main_camera_fov.position.y = 2.0f;
            main_camera_fov.vertical_velocity = 0.0f;
            if (main_inputs.space_key_pressed) {
                float jump_velocity = sqrtf(2.0f * fabsf(g_cfg.world.gravity) * g_cfg.camera.jump_height);
                main_camera_fov.vertical_velocity = jump_velocity;
                main_inputs.space_key_pressed = false;
            }
        }
        if (main_camera_fov.position.x < -250.0f) {
            main_camera_fov.position.x = -250.0f;
        }
        if (main_camera_fov.position.x > 250.0f) {
            main_camera_fov.position.x = 250.0f;
        }
        if (main_camera_fov.position.z < -250.0f) {
            main_camera_fov.position.z = -250.0f;
        }
        if (main_camera_fov.position.z > 250.0f) {
            main_camera_fov.position.z = 250.0f;
        }
    } //Mouse, Escape, E, F key bindings and actions
    if (main_inputs.escape_key_pressed) {
        if (main_inputs.is_mouse_locked) {
            mouse_lock_disable(gtk_widget_get_toplevel(GTK_WIDGET(user_data_pointer)));
            main_inputs.is_mouse_locked = false;
        }
        main_inputs.escape_key_pressed = false;
    }
    if (main_inputs.right_mouse_button_clicked) {
        selector_ray_tracing();
        main_inputs.right_mouse_button_clicked = false;
    }
    if (main_inputs.middle_mouse_button_clicked) {
        if (selected_object >= 0) {
            scene_remove_object_by_index(selected_object);
        }
        main_inputs.middle_mouse_button_clicked = false;
    }
    if (main_inputs.e_key_pressed) {
        if (selected_object >= 0) {
            if (main_inputs.object_menu_level > 0) {
                main_inputs.object_menu_level = 0;
            } else {
                main_inputs.object_menu_level = 1;
            }
        }
        main_inputs.e_key_pressed = false;
        config_menu_close(); /* MPE_TASK_35 */
    }
    if (main_inputs.f_key_pressed) {
        if (selected_object >= 0) {
            selector_apply_force_impulse(250.0f);
        } //Increased as cube friction is far higher
        main_inputs.f_key_pressed = false;
    }
    /* MPE_TASK_21_KEYBOARD_ONLY_ACTIONS_BEGIN */
    if (main_inputs.r_key_pressed) {
        if (main_inputs.is_debug_mode_active) {
            selector_ray_tracing();
        }
        main_inputs.r_key_pressed = false;
    }

    if (main_inputs.delete_key_pressed) {
        if ((main_inputs.is_debug_mode_active) && (selected_object >= 0) && (selected_object < object_count)) {
            scene_remove_object_by_index(selected_object);
        }
        main_inputs.delete_key_pressed = false;
    }

    if (main_inputs.m_key_pressed) {
        if ((main_inputs.is_debug_mode_active) && (!main_inputs.is_mouse_locked) && (parent_window)) {
            mouse_lock_enable(parent_window);
            main_inputs.is_mouse_locked = true;
        }
        main_inputs.m_key_pressed = false;
    }

    if (main_inputs.t_key_pressed) {
        if (main_inputs.is_debug_mode_active) {
            if (debug_terminal_is_open()) {
                debug_terminal_focus_entry();
            } else {
                debug_terminal_open(parent_window);
            }
        }
        main_inputs.t_key_pressed = false;
    }
    /* MPE_TASK_21_KEYBOARD_ONLY_ACTIONS_END */
    if (main_inputs.stability_test_pressed) {
        scene_spawn_stability_stack();
        main_inputs.stability_test_pressed = false;
    }
    if (main_inputs.sleep_wake_test_pressed) {
        scene_spawn_sleep_wake_test();
        main_inputs.sleep_wake_test_pressed = false;
    }
    if (main_inputs.editor_torture_pressed) {
        scene_editor_torture_test();
        main_inputs.editor_torture_pressed = false;
    }
    if (main_inputs.spawn_stress_pressed) {
        scene_spawn_stress_test();
        main_inputs.spawn_stress_pressed = false;
    }
    if (main_inputs.validation_report_pressed) {
        validation_report_print();
        main_inputs.validation_report_pressed = false;
    }
    /* MPE_TASK_18_TERMINAL_OPEN_BEGIN */
    if (main_inputs.debug_terminal_pressed) {
        if (main_inputs.is_debug_mode_active) {
            debug_terminal_open(parent_window);
        }
        main_inputs.debug_terminal_pressed = false;
    }
    /* MPE_TASK_18_TERMINAL_OPEN_END */
    /* MPE_TASK_13_LONG_RUN_START_CALL_BEGIN */
    if (main_inputs.long_run_validation_pressed) {
        scene_spawn_long_run_validation();
        long_run_validation_start(a3_long_run_validation_ticks);
        long_run_validation_restore_config = 1; /* MPE_TASK_39_FIX */
        main_inputs.long_run_validation_pressed = false;
    }
    /* MPE_TASK_13_LONG_RUN_START_CALL_END */
    /* MPE_TASK_39_CONFIG_TORTURE_CALL_BEGIN */
    if (main_inputs.config_torture_pressed) {
        /* MPE_TASK_39_FIX_SAVE_CONFIG */
        mpe_config_save("status/engine.cfg.backup");
        scene_spawn_config_torture_test();
        long_run_validation_start(a3_long_run_validation_ticks);
        long_run_validation_restore_config = 1; /* MPE_TASK_39_FIX */
        main_inputs.config_torture_pressed = false;
    }
    /* MPE_TASK_39_CONFIG_TORTURE_CALL_END */

    //Holding down shift, spawn gun
    static float enter_hold_timer = 0.0f;
    static float enter_spawn_interval_timer = 0.0f;
    static bool enter_previously_held = false;
    /* MPE_TASK_22_ENTER_SPAWN_CONDITION_BEGIN */
    if ((main_inputs.enter_spawn_held) && (!editor_dialog_is_active()) && (!main_inputs.is_menu_open) &&
        (main_inputs.spawner_menu_level == 0) && (main_inputs.velocity_menu_level == 0) &&
        (main_inputs.object_menu_level == 0)) {
        /* MPE_TASK_22_ENTER_SPAWN_CONDITION_END */
        if (!enter_previously_held) {
            if (main_inputs.current_spawn_type == 0) {
                spawner_launch_sphere(g_cfg.spawner.radius, g_cfg.spawner.mass, g_cfg.spawner.speed);
            } else {
                vector3 cube_spawn_position =
                    vector3_addition(main_camera_fov.position,
                                     vector3_scaling(main_camera_fov.forward_vector, g_cfg.spawner.cube_extent + 1.0f));
                spawner_launch_cube(
                    cube_spawn_position,
                    (vector3){g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent},
                    g_cfg.spawner.cube_mass);
            }
            enter_hold_timer = 0.0f;
            enter_spawn_interval_timer = 0.0f;
        } else {
            enter_hold_timer += frame_delta_time;
            if (enter_hold_timer > 0.3f) {
                enter_spawn_interval_timer += frame_delta_time;
                if (enter_spawn_interval_timer >= 0.02f) {
                    if (main_inputs.current_spawn_type == 0) {
                        spawner_launch_sphere(g_cfg.spawner.radius, g_cfg.spawner.mass, g_cfg.spawner.speed);
                    } else {
                        vector3 cube_spawn_position = vector3_addition(
                            main_camera_fov.position,
                            vector3_scaling(main_camera_fov.forward_vector, g_cfg.spawner.cube_extent + 1.0f));
                        spawner_launch_cube(
                            cube_spawn_position,
                            (vector3){g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent},
                            g_cfg.spawner.cube_mass);
                    }
                    enter_spawn_interval_timer = 0.0f;
                }
            }
        }
        enter_previously_held = true;
    } else {
        enter_hold_timer = 0.0f;
        enter_previously_held = false;
    } //Scene Saving, 9 Key bindings
    if (main_inputs.menu_1_pressed) {
        save_scene("status/scene.dat");
        main_inputs.menu_1_pressed = false;
        main_inputs.is_menu_open = false;
    } /* MPE_TASK_35_SAVE */
    if (main_inputs.menu_2_pressed) {
        scene_loading("status/scene.dat");
        editor_reset();
        main_inputs.menu_2_pressed = false;
        main_inputs.is_menu_open = false;
    } /* MPE_TASK_35_LOAD */
    if (main_inputs.menu_3_pressed) {
        scene_clear();
        clear_selection();
        contact_cache_clear();
        editor_reset();
        main_inputs.menu_3_pressed = false;
        main_inputs.is_menu_open = false;
    } /* MPE_TASK_35_CLEAR */
    if (main_inputs.menu_4_pressed) {
        mpe_config_save("status/engine.cfg");
        main_inputs.menu_4_pressed = false;
        main_inputs.is_menu_open = false;
    } /* MPE_TASK_35_SAVE_CFG */
    if (main_inputs.menu_5_pressed) {
        mpe_config_reset_defaults();
        contact_cache_clear();
        main_inputs.menu_5_pressed = false;
        main_inputs.is_menu_open = false;
    } /* MPE_TASK_35_RESET_CFG */
    if (main_inputs.menu_6_pressed) {
        main_inputs.menu_6_pressed = false;
        gtk_main_quit();
    } /* MPE_TASK_35_EXIT */
    editor_update_menus(parent_window);
    config_menu_update(parent_window); /* MPE_TASK_35 */
    // v1.4 Simulation Contract: Fixed Timestep Accumulator
    static broadphase_pair persistent_collision_pairs[mpe_max_broadphase_pairs];
    static float physics_time_accumulator = 0.0f;
    const float fixed_physics_dt = 1.0f / 60.0f;
    const int max_substeps_per_frame = 5; // Spiral of death prevention
    physics_time_accumulator += frame_delta_time;
    if (physics_time_accumulator > fixed_physics_dt * max_substeps_per_frame) {
        physics_time_accumulator = fixed_physics_dt * max_substeps_per_frame;
    }
    float linear_damping_factor = powf(g_cfg.world.drag, fixed_physics_dt);
    float angular_damping_factor = powf(g_cfg.world.drag * 0.97f, fixed_physics_dt);
    /* MPE_TASK_09_MANIFOLD_OVERFLOW_FRAME_RESET_BEGIN */
    debug_last_manifold_overflow_count = 0;
    /* MPE_TASK_09_MANIFOLD_OVERFLOW_FRAME_RESET_END */
    while (physics_time_accumulator >= fixed_physics_dt) {
        /* MPE_TASK_14_SANITIZE_ONCE_BEGIN */
        for (int sanitize_index = 0; sanitize_index < object_count; sanitize_index++) {
            rigidbody_sanitize(&obj_per_scene[sanitize_index]);
        }
        /* MPE_TASK_14_SANITIZE_ONCE_END */
        int detected_collision_count = 0;
        detected_collision_count = broadphase_generate_pairing(obj_per_scene, object_count, persistent_collision_pairs,
                                                               mpe_max_broadphase_pairs);
        debug_last_broadphase_pair_count = detected_collision_count;
        static collision_data active_manifold[a3_max_manifolds];
        int manifold_count = 0;
        contact_cache_stats_reset();
        apply_force_all_joints();
        for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {
            vector3 constant_gravity_acceleration = {0, g_cfg.world.gravity, 0};
            rigidbody *rigid_body = &obj_per_scene[object_iterator_index];
            if (rigid_body->is_sleeping) {
                continue;
            }
            rb_apply_forces_perfect(rigid_body, vector3_scaling(constant_gravity_acceleration, rigid_body->mass));
        }

        for (int velocity_integration_index = 0; velocity_integration_index < object_count;
             velocity_integration_index++) {
            rb_integrate_velocity(&obj_per_scene[velocity_integration_index], fixed_physics_dt, linear_damping_factor,
                                  angular_damping_factor);
        }

        for (int collision_index = 0; collision_index < detected_collision_count; collision_index++) {
            rigidbody *rigid_body_a = &obj_per_scene[persistent_collision_pairs[collision_index].object_index_a];
            rigidbody *rigid_body_b = &obj_per_scene[persistent_collision_pairs[collision_index].object_index_b];
            /* MPE_TASK_13_SLEEP_PAIR_SKIP_BEGIN */
            if ((rigid_body_a->is_sleeping) && (rigid_body_b->is_sleeping)) {
                continue;
            }
            /* MPE_TASK_13_SLEEP_PAIR_SKIP_END */
            collision_data narrowphase_collision = {0};
            bool collided = false;
            if (rigid_body_a->type == object_sphere && rigid_body_b->type == object_sphere)
                collided = collision_dual_sphere(rigid_body_a, rigid_body_b, &narrowphase_collision);
            else if (rigid_body_a->type == object_sphere && rigid_body_b->type == object_cube)
                collided = collision_sphere_cube(rigid_body_a, rigid_body_b, &narrowphase_collision);
            else if (rigid_body_a->type == object_cube && rigid_body_b->type == object_sphere) {
                collided = collision_sphere_cube(rigid_body_b, rigid_body_a, &narrowphase_collision);
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = rigid_body_a;
                narrowphase_collision.object_b = rigid_body_b;
            } else if (rigid_body_a->type == object_cube && rigid_body_b->type == object_cube)
                collided = collision_dual_cube(rigid_body_a, rigid_body_b, &narrowphase_collision);
            /* MPE_TASK_09_OBJECT_MANIFOLD_CONDITION_BEGIN */
            if (collided) {
                if (manifold_count < a3_max_manifolds) {
                    /* MPE_TASK_09_OBJECT_MANIFOLD_CONDITION_END */
                    /* MPE_TASK_13_SLEEP_WAKE_FIX_BEGIN */
                    bool a3_a_was_sleeping = rigid_body_a->is_sleeping;
                    bool a3_b_was_sleeping = rigid_body_b->is_sleeping;

                    if (a3_a_was_sleeping && a3_b_was_sleeping) {
                        continue;
                    }

                    float a3_wake_linear_threshold_sq = g_cfg.sleep.wake_linear_thresh_sq;
                        /* MPE_TASK_30 */ /* 0.1 m/s */
                    float a3_wake_angular_threshold_sq = g_cfg.sleep.wake_angular_thresh_sq;
                        /* MPE_TASK_30 */ /* 0.05 rad/s */

                    bool a3_a_is_active =
                        (!a3_a_was_sleeping) &&
                        ((vector3_length_squared(rigid_body_a->velocity) > a3_wake_linear_threshold_sq) ||
                         (vector3_length_squared(rigid_body_a->angular_velocity) > a3_wake_angular_threshold_sq));

                    bool a3_b_is_active =
                        (!a3_b_was_sleeping) &&
                        ((vector3_length_squared(rigid_body_b->velocity) > a3_wake_linear_threshold_sq) ||
                         (vector3_length_squared(rigid_body_b->angular_velocity) > a3_wake_angular_threshold_sq));

                    if (a3_a_was_sleeping && (!rigid_body_b->static_state) && a3_b_is_active) {
                        rigidbody_wake(rigid_body_a);
                    }

                    if (a3_b_was_sleeping && (!rigid_body_a->static_state) && a3_a_is_active) {
                        rigidbody_wake(rigid_body_b);
                    }
                    /* MPE_TASK_13_SLEEP_WAKE_FIX_END */
                    collision_prepare_solver(&narrowphase_collision, &active_manifold[manifold_count]);
                    manifold_count++;
                }
                /* MPE_TASK_09_OBJECT_MANIFOLD_OVERFLOW_BEGIN */
            } else {
                debug_last_manifold_overflow_count++;
            }
            /* MPE_TASK_09_OBJECT_MANIFOLD_OVERFLOW_END */
        } /* A3_PATCH_16_FLOOR_MANIFOLD */
        for (int floor_object_index = 0; floor_object_index < object_count; floor_object_index++) {
            rigidbody *floor_rigid_body = &obj_per_scene[floor_object_index];
            if ((floor_rigid_body->static_state) || (floor_rigid_body->is_sleeping)) {
                continue;
            }

            collision_data floor_collision = {0};

            if (collision_static_plane_body(floor_rigid_body, 0.0f, &floor_collision)) {
                /* MPE_TASK_09_FLOOR_MANIFOLD_OVERFLOW_BEGIN */
                if (manifold_count < a3_max_manifolds) {
                    collision_prepare_solver(&floor_collision, &active_manifold[manifold_count]);
                    manifold_count++;
                } else {
                    debug_last_manifold_overflow_count++;
                }
                /* MPE_TASK_09_FLOOR_MANIFOLD_OVERFLOW_END */
            }
        }

        debug_last_manifold_count = manifold_count;
        /* MPE_TASK_13_SLEEP_STATICIZE_BEGIN */
        math3 a3_sleep_zero_matrix = {{{0.0f}}};
        for (int sleep_staticize_index = 0; sleep_staticize_index < object_count; sleep_staticize_index++) {
            rigidbody *sleep_staticize_body = &obj_per_scene[sleep_staticize_index];
            if ((sleep_staticize_body->is_sleeping) && (!sleep_staticize_body->static_state)) {
                sleep_staticize_body->velocity = vector3_zero();
                sleep_staticize_body->angular_velocity = vector3_zero();
                sleep_staticize_body->force_accumulator = vector3_zero();
                sleep_staticize_body->torque_accumulator = vector3_zero();
                sleep_staticize_body->inverse_mass = 0.0f;
                sleep_staticize_body->inverse_inertia_system = a3_sleep_zero_matrix;
            }
        }
        /* MPE_TASK_13_SLEEP_STATICIZE_END */
        int solver_iterations = g_cfg.timestep.solver_iterations; /* MPE_TASK_29 from config */
        for (int iter = 0; iter < solver_iterations; iter++) {
            for (int m = 0; m < manifold_count; m++) {
                collision_resolve_iterative(&active_manifold[m]);
            }
        }
        contact_cache_save(active_manifold, manifold_count);
        /* MPE_TASK_13_SLEEP_RESTORE_BEGIN */
        for (int sleep_restore_index = 0; sleep_restore_index < object_count; sleep_restore_index++) {
            rigidbody *sleep_restore_body = &obj_per_scene[sleep_restore_index];
            if ((sleep_restore_body->is_sleeping) && (!sleep_restore_body->static_state)) {
                if ((sleep_restore_body->mass > 0.0f) && (isfinite(sleep_restore_body->mass))) {
                    sleep_restore_body->inverse_mass = 1.0f / sleep_restore_body->mass;
                } else {
                    sleep_restore_body->inverse_mass = 0.0f;
                }
                math3 sleep_rotation_matrix = vector4_to_math3(sleep_restore_body->orientation);
                math3 sleep_rotation_transpose = math3_transposition(sleep_rotation_matrix);
                sleep_restore_body->inverse_inertia_system = math3_multiplication(
                    sleep_rotation_matrix,
                    math3_multiplication(sleep_restore_body->inverse_inertia_tensor_local, sleep_rotation_transpose));
                sleep_restore_body->velocity = vector3_zero();
                sleep_restore_body->angular_velocity = vector3_zero();
            }
        }
        /* MPE_TASK_13_SLEEP_RESTORE_END */
        /* MPE_TASK_20A_BOUNDARY_MOVED_DECL_BEGIN */
        bool a3_boundary_moved_any = false;
        /* MPE_TASK_20A_BOUNDARY_MOVED_DECL_END */
        for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {
            rigidbody *rigid_body = &obj_per_scene[object_iterator_index];
            rb_integrate_position(rigid_body, fixed_physics_dt); /* A3_PATCH_44_SEMI_IMPLICIT */
            rigidbody_sanitize(rigid_body);
            /* MPE_TASK_20A_BOUNDARY_TRACK_BEGIN */
            vector3 a3_pre_boundary_position = rigid_body->position;
            if (!main_inputs.is_debug_mode_active) {
                boundary_apply_box(rigid_body, (vector3){-250, 0, -250}, (vector3){250, 500, 250});
            } else {
                boundary_apply_floor(rigid_body, 0.0f);
            }
            if (vector3_length_squared(vector3_subtraction(rigid_body->position, a3_pre_boundary_position)) >
                0.000001f) {
                a3_boundary_moved_any = true;
            }
            /* MPE_TASK_20A_BOUNDARY_TRACK_END */
        }
        /* MPE_TASK_20A_POST_BOUNDARY_DEPENETRATION_BEGIN */
        a3_positional_depenetration_pass(persistent_collision_pairs, &detected_collision_count, a3_boundary_moved_any);
        /* MPE_TASK_20A_POST_BOUNDARY_DEPENETRATION_END */
        physics_time_accumulator -= fixed_physics_dt;
    }
    gtk_widget_queue_draw(GTK_WIDGET(user_data_pointer));
    /* MPE_TASK_12_SLEEPING_COUNT_BEGIN */
    int a3_sleeping_object_count = 0;
    for (int sleep_count_index = 0; sleep_count_index < object_count; sleep_count_index++) {
        if (obj_per_scene[sleep_count_index].is_sleeping) {
            a3_sleeping_object_count++;
        }
    }
    debug_last_sleeping_object_count = a3_sleeping_object_count;
    /* MPE_TASK_12_SLEEPING_COUNT_END */
    debug_last_object_count = object_count;
    /* MPE_TASK_13_LONG_RUN_TICK_CALL_BEGIN */
    long_run_validation_tick_update();
    /* MPE_TASK_13_LONG_RUN_TICK_CALL_END */
    overlay_update();
    return TRUE;
}
