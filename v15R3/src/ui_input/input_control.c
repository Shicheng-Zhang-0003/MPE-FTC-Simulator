#include "../mpe_engine.h"
#include "input_control.h"
#include "camera.h"
#include "mouse_lock.h"
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include <stdbool.h>
extern camera main_camera_fov;
extern input_status main_inputs;
extern int selected_object;
void initialize_input (input_status *input_state) {
    //Keyboard
    input_state -> w_key_pressed = false;
    input_state -> a_key_pressed = false;
    input_state -> s_key_pressed = false;
    input_state -> d_key_pressed = false;
    input_state -> space_key_pressed = false;
    input_state -> shift_key_pressed = false;
    input_state -> escape_key_pressed = false;
    input_state -> f_key_pressed = false;
    /* q/m/delete/t keybinds REMOVED (dead or deleted; see header). */
    input_state -> i_key_pressed = false;
    input_state -> j_key_pressed = false;
    input_state -> k_key_pressed = false;
    input_state -> l_key_pressed = false;
     /* MPE_TASK_21_KEYBOARD_ONLY_INIT_BEGIN (r only) */
input_state -> r_key_pressed = false;
/* MPE_TASK_21_KEYBOARD_ONLY_INIT_END */
//Menu
    input_state -> is_menu_open = false;
    input_state -> menu_1_pressed = false;
    input_state -> menu_2_pressed = false;
    input_state -> menu_3_pressed = false;
    input_state -> menu_4_pressed = false;
    input_state -> menu_5_pressed = false;
    input_state -> menu_6_pressed = false; /* MPE_TASK_35_FOCUS */
    //Spawn
    input_state -> spawner_menu_level = 0;
    input_state -> velocity_menu_level = 0;
    input_state -> object_menu_level = 0;
    input_state -> current_spawn_type = 0; // 0: Sphere, 1: Cube
    input_state -> up_arrow_pressed = false;
    input_state -> down_arrow_pressed = false;
    input_state -> enter_key_pressed = false;
    input_state -> e_key_pressed = false;
input_state -> stability_test_pressed = false;
input_state -> sleep_wake_test_pressed = false;
input_state -> editor_torture_pressed = false;
input_state -> spawn_stress_pressed = false;
input_state -> validation_report_pressed = false;
    /* MPE_TASK_13_LONG_RUN_INIT_BEGIN */
input_state -> long_run_validation_pressed = false;
/* MPE_TASK_13_LONG_RUN_INIT_END */
/* MPE_TASK_39_CONFIG_TORTURE_INIT_BEGIN */
input_state -> config_torture_pressed = false;
/* MPE_TASK_39_CONFIG_TORTURE_INIT_END */
/* MPE_TASK_18_TERMINAL_INPUT_INIT_BEGIN */
input_state -> debug_terminal_pressed = false;
/* MPE_TASK_18_TERMINAL_INPUT_INIT_END */
/* MPE_TASK_22_ENTER_SPAWN_INIT_BEGIN */
input_state -> enter_spawn_held = false;
/* MPE_TASK_22_ENTER_SPAWN_INIT_END */
//Mouse
    input_state -> is_mouse_locked = false;
    input_state -> is_debug_mode_active = false;
    input_state -> right_mouse_button_clicked = false;
    input_state -> middle_mouse_button_clicked = false;
    input_state -> mouse_delta_x = 0.0f;
    input_state -> mouse_delta_y = 0.0f;
    input_state -> mouse_delta_accum_x = 0.0f;
    input_state -> mouse_delta_accum_y = 0.0f;
    input_state -> suppress_mouse_delta = false;
    input_state -> last_mouse_x = 0;
    input_state -> last_mouse_y = 0;
    input_state -> mouse_centered = false;
    input_state -> mouse_lock_state = 0;
    input_state -> marked_joint_object_index = -1;
} gboolean on_keypress (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored) {
    (void) widget;
    input_status *input_state = (input_status *) user_data_stored;
    if (event -> keyval == GDK_KEY_w) {input_state -> w_key_pressed = true;}
    if (event -> keyval == GDK_KEY_a) {input_state -> a_key_pressed = true;}
    if (event -> keyval == GDK_KEY_s) {input_state -> s_key_pressed = true;}
    if (event -> keyval == GDK_KEY_d) {input_state -> d_key_pressed = true;}
    /* E toggles the object menu: edge-triggered via e_key_held latch so
     * key auto-repeat while held cannot re-toggle (menu flicker). */
    if (event -> keyval == GDK_KEY_e) {
        if (!input_state -> e_key_held) {
            input_state -> e_key_pressed = true;
            input_state -> e_key_held = true;
        }
    }
    /* Backspace deletes the selected object (trackpads lack middle-click). */
    if (event -> keyval == GDK_KEY_BackSpace) {input_state -> delete_key_pressed = true;}
    if (event -> keyval == GDK_KEY_f) {input_state -> f_key_pressed = true;}
/* q/m/t/left/right REMOVED (dead or deleted keybinds; see header). Delete is
 * back as Backspace (see below). */
/* MPE_TASK_21_KEYBOARD_ONLY_KEYPRESS_BEGIN (r only) */
if ((event -> keyval == GDK_KEY_r) || (event -> keyval == GDK_KEY_R)) {input_state -> r_key_pressed = true;}
/* MPE_TASK_21_KEYBOARD_ONLY_KEYPRESS_END */
if (event -> keyval == GDK_KEY_F5) {input_state -> stability_test_pressed = true;}
if (event -> keyval == GDK_KEY_F6) {input_state -> sleep_wake_test_pressed = true;}
if (event -> keyval == GDK_KEY_F7) {input_state -> editor_torture_pressed = true;}
if (event -> keyval == GDK_KEY_F8) {input_state -> spawn_stress_pressed = true;}
if (event -> keyval == GDK_KEY_F9) {input_state -> validation_report_pressed = true;}
/* MPE_TASK_13_LONG_RUN_KEY_BEGIN */
if (event -> keyval == GDK_KEY_F10) {input_state -> long_run_validation_pressed = true;}
/* MPE_TASK_13_LONG_RUN_KEY_END */
/* MPE_TASK_39_CONFIG_TORTURE_KEY_BEGIN */
if (event -> keyval == GDK_KEY_F11) {input_state -> config_torture_pressed = true;}
/* MPE_TASK_39_CONFIG_TORTURE_KEY_END */
    if (event -> keyval == GDK_KEY_i) {input_state -> i_key_pressed = true;}
    if (event -> keyval == GDK_KEY_j) {input_state -> j_key_pressed = true;}
    if (event -> keyval == GDK_KEY_k) {input_state -> k_key_pressed = true;}
     if (event -> keyval == GDK_KEY_l) {input_state -> l_key_pressed = true;}
     if ((event -> keyval == GDK_KEY_9) && (!config_menu_is_open ())) {input_state -> spawner_menu_level = 0; input_state -> velocity_menu_level = 0; input_state -> object_menu_level = 0; input_state -> is_menu_open = !(input_state -> is_menu_open);}
     if ((event -> keyval == GDK_KEY_8) && (!config_menu_is_open ())) {input_state -> is_menu_open = false; input_state -> velocity_menu_level = 0; input_state -> object_menu_level = 0; if (input_state -> spawner_menu_level > 0) {input_state -> spawner_menu_level = 0;} else {input_state -> spawner_menu_level = 1;}}
     if ((event -> keyval == GDK_KEY_7) && (!config_menu_is_open ())) {input_state -> is_menu_open = false; input_state -> spawner_menu_level = 0; input_state -> object_menu_level = 0; if (input_state -> velocity_menu_level > 0) {input_state -> velocity_menu_level = 0;} else {input_state -> velocity_menu_level = 1;}}
 /* MPE_TASK_35_CONFIG_MENU_KEY_BEGIN */
if ((event -> keyval == GDK_KEY_6) && (!input_state -> is_menu_open) && (input_state -> object_menu_level == 0)) {
input_state -> spawner_menu_level = 0;
input_state -> velocity_menu_level = 0;
input_state -> object_menu_level = 0;
if (config_menu_is_open ()) {config_menu_close ();}
else {config_menu_level_force_open ();}
}
/* MPE_TASK_35_CONFIG_MENU_KEY_END */
    /* MPE_TASK_18_TERMINAL_KEY_BEGIN */
if ((event -> keyval == GDK_KEY_1) && (!config_menu_is_open ()) &&
(input_state -> is_debug_mode_active) &&
(!input_state -> is_menu_open) &&
(input_state -> spawner_menu_level == 0) &&
(input_state -> velocity_menu_level == 0) &&
(input_state -> object_menu_level == 0)) {
input_state -> debug_terminal_pressed = true;
}
/* MPE_TASK_18_TERMINAL_KEY_END */
if (input_state -> is_menu_open) {
        if (event -> keyval == GDK_KEY_1) {input_state -> menu_1_pressed = true;}
        if (event -> keyval == GDK_KEY_2) {input_state -> menu_2_pressed = true;}
        if (event -> keyval == GDK_KEY_3) {input_state -> menu_3_pressed = true;}
        if (event -> keyval == GDK_KEY_4) {input_state -> menu_4_pressed = true;}
        if (event -> keyval == GDK_KEY_5) {input_state -> menu_5_pressed = true;}
        if (event -> keyval == GDK_KEY_6) {input_state -> menu_6_pressed = true;} /* MPE_TASK_35 */
    } // Menu Navigation
    /* MPE_TASK_35_CONFIG_MENU_NAV_BEGIN */
    if (config_menu_is_open ()) {
        if (event -> keyval == GDK_KEY_0) {config_menu_key_press (0);}
        if (event -> keyval == GDK_KEY_1) {config_menu_key_press (1);}
        if (event -> keyval == GDK_KEY_2) {config_menu_key_press (2);}
        if (event -> keyval == GDK_KEY_3) {config_menu_key_press (3);}
        if (event -> keyval == GDK_KEY_4) {config_menu_key_press (4);}
        if (event -> keyval == GDK_KEY_5) {config_menu_key_press (5);}
        if (event -> keyval == GDK_KEY_7) {config_menu_key_press (7);}
        if (event -> keyval == GDK_KEY_8) {config_menu_key_press (8);}
        if (event -> keyval == GDK_KEY_9) {config_menu_key_press (9);}
    }
    /* MPE_TASK_35_CONFIG_MENU_NAV_END */
    if ((input_state -> spawner_menu_level > 0) || (input_state -> velocity_menu_level > 0) || (input_state -> object_menu_level > 0)) {
        if (event -> keyval == GDK_KEY_Up) {input_state -> up_arrow_pressed = true;}
        if (event -> keyval == GDK_KEY_Down) {input_state -> down_arrow_pressed = true;}
        /* Left/Right REMOVED (pre-dialog change-rate relics). */
        if ((event -> keyval == GDK_KEY_Return) || (event -> keyval == GDK_KEY_KP_Enter)) {input_state -> enter_key_pressed = true;}
    } // Spawner Menu Logic
    if (input_state -> spawner_menu_level == 1) {
        if (event -> keyval == GDK_KEY_1) {input_state -> spawner_menu_level = 2;}
        if (event -> keyval == GDK_KEY_2) {input_state -> spawner_menu_level = 5;}
        if (event -> keyval == GDK_KEY_3) {input_state -> spawner_menu_level = 8;}
        if (event -> keyval == GDK_KEY_4) {input_state -> spawner_menu_level = 9;}
        /* FTC robot spawn (key 5): goBILDA 5203 19.2:1 motor. */
        if (event -> keyval == GDK_KEY_5) {gui_robot_spawn(0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_19_2);}
    } else if (input_state -> spawner_menu_level == 2) {
        if (event -> keyval == GDK_KEY_1) {input_state -> spawner_menu_level = 3;}
        if (event -> keyval == GDK_KEY_2) {input_state -> spawner_menu_level = 4;}
    } else if (input_state -> spawner_menu_level == 5) {
        if (event -> keyval == GDK_KEY_1) {input_state -> spawner_menu_level = 6;}
        if (event -> keyval == GDK_KEY_2) {input_state -> spawner_menu_level = 7;}
    } else if (input_state -> spawner_menu_level == 9) {
        if (event -> keyval == GDK_KEY_1) {input_state -> spawner_menu_level = 10;}
        if (event -> keyval == GDK_KEY_2) {input_state -> spawner_menu_level = 11;}
        if (event -> keyval == GDK_KEY_3) {input_state -> spawner_menu_level = 12;}
    } // Velocity Menu Logic
    if (input_state -> velocity_menu_level == 1) {
        if (event -> keyval == GDK_KEY_1) {input_state -> velocity_menu_level = 2;}
        if (event -> keyval == GDK_KEY_2) {input_state -> velocity_menu_level = 10;}
        if (event -> keyval == GDK_KEY_3) {input_state -> velocity_menu_level = 20;}
    } else if (input_state -> velocity_menu_level == 2) {
        if (event -> keyval == GDK_KEY_1) {input_state -> velocity_menu_level = 3;}
        if (event -> keyval == GDK_KEY_2) {input_state -> velocity_menu_level = 4;}
    } else if (input_state -> velocity_menu_level == 20) {
        if (event -> keyval == GDK_KEY_1) {input_state -> velocity_menu_level = 21;}
        if (event -> keyval == GDK_KEY_2) {input_state -> velocity_menu_level = 22;}
        if (event -> keyval == GDK_KEY_3) {input_state -> velocity_menu_level = 23;}
        /* World-physics shortcuts imported from menu 6 (game-mode-safe,
         * non-debug-only params only; debug-only physics stays in 6). */
        if (event -> keyval == GDK_KEY_4) {input_state -> velocity_menu_level = 24;}
        if (event -> keyval == GDK_KEY_5) {input_state -> velocity_menu_level = 25;}
    } else if (input_state -> velocity_menu_level == 10) {
        if (event -> keyval == GDK_KEY_1) {input_state -> velocity_menu_level = 11;}
        if (event -> keyval == GDK_KEY_2) {input_state -> velocity_menu_level = 12;}
    } // Selected Object Menu Logic
    if (input_state -> object_menu_level == 1) {
        if (event -> keyval == GDK_KEY_1) {input_state -> object_menu_level = 2;}
        if (event -> keyval == GDK_KEY_2) {input_state -> object_menu_level = 3;}
        if (event -> keyval == GDK_KEY_3) {input_state -> object_menu_level = 4;}
        if (event -> keyval == GDK_KEY_4) {input_state -> object_menu_level = 5;}
        if (event -> keyval == GDK_KEY_5) {input_state -> object_menu_level = 6;}
        if (event -> keyval == GDK_KEY_6) {
            if (input_state -> marked_joint_object_index != -1 && input_state -> marked_joint_object_index != selected_object) {
                input_state -> object_menu_level = 7;
            } else {
                input_state -> object_menu_level = 8;
            }
        }
        if ((event -> keyval == GDK_KEY_7) && (!config_menu_is_open ())) {
            if (input_state -> marked_joint_object_index != -1 && input_state -> marked_joint_object_index != selected_object) {
                input_state -> object_menu_level = 8;
            }
        }
    } else if (input_state -> object_menu_level == 8) {
        if (event -> keyval == GDK_KEY_1) {input_state -> object_menu_level = 81;}
        if (event -> keyval == GDK_KEY_2) {input_state -> object_menu_level = 82;}
        if (event -> keyval == GDK_KEY_3) {input_state -> object_menu_level = 83;}
        if (event -> keyval == GDK_KEY_4) {input_state -> object_menu_level = 84;}
        if (event -> keyval == GDK_KEY_5) {input_state -> object_menu_level = 85;}
        if (event -> keyval == GDK_KEY_6) {input_state -> object_menu_level = 86;}
        if ((event -> keyval == GDK_KEY_7) && (!config_menu_is_open ())) {input_state -> object_menu_level = 87;}
        if ((event -> keyval == GDK_KEY_8) && (!config_menu_is_open ())) {input_state -> object_menu_level = 88;}
    } /* MPE_TASK_22_ENTER_SPAWN_KEYPRESS_BEGIN */
if (((event -> keyval == GDK_KEY_Return) || (event -> keyval == GDK_KEY_KP_Enter)) &&
(!input_state -> is_menu_open) &&
(input_state -> spawner_menu_level == 0) &&
(input_state -> velocity_menu_level == 0) &&
(input_state -> object_menu_level == 0)) {
input_state -> enter_spawn_held = true;
}
/* MPE_TASK_22_ENTER_SPAWN_KEYPRESS_END */
if (event -> keyval == GDK_KEY_space) {input_state -> space_key_pressed = true;}
    if (event -> keyval == GDK_KEY_Shift_L) {input_state -> shift_key_pressed = true;}
    if (event -> keyval == GDK_KEY_Escape) {input_state -> escape_key_pressed = true;}
    if ((event -> keyval == GDK_KEY_0) && (!config_menu_is_open ())) {input_state -> is_debug_mode_active = !input_state -> is_debug_mode_active;}
    return FALSE;
} gboolean on_key_released (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored) {
    (void) widget;
    input_status *input_state = (input_status *) user_data_stored;
    if (event -> keyval == GDK_KEY_w) {input_state -> w_key_pressed = false;}
    if (event -> keyval == GDK_KEY_a) {input_state -> a_key_pressed = false;}
    if (event -> keyval == GDK_KEY_s) {input_state -> s_key_pressed = false;}
    if (event -> keyval == GDK_KEY_d) {input_state -> d_key_pressed = false;}
 /* MPE_TASK_21_KEYBOARD_ONLY_KEYRELEASE_BEGIN (r only) */
if ((event -> keyval == GDK_KEY_r) || (event -> keyval == GDK_KEY_R)) {input_state -> r_key_pressed = false;}
 /* MPE_TASK_21_KEYBOARD_ONLY_KEYRELEASE_END */
     if (event -> keyval == GDK_KEY_e) {input_state -> e_key_held = false;}
    if (event -> keyval == GDK_KEY_BackSpace) {input_state -> delete_key_pressed = false;}
    if (event -> keyval == GDK_KEY_space) {input_state -> space_key_pressed = false;} /* MFS_154 */
    if (event -> keyval == GDK_KEY_Shift_L) {input_state -> shift_key_pressed = false;} /* MFS_154 */
/* MPE_TASK_22_ENTER_SPAWN_KEYRELEASE_BEGIN */
if ((event -> keyval == GDK_KEY_Return) || (event -> keyval == GDK_KEY_KP_Enter)) {
input_state -> enter_spawn_held = false;
}
/* MPE_TASK_22_ENTER_SPAWN_KEYRELEASE_END */
    return FALSE;
} gboolean on_mouse_movements (GtkWidget *widget, GdkEventMotion *event, gpointer user_data_stored) {
    (void) user_data_stored;
    input_status *input_state = &main_inputs;

    /* Only process motion when locked (state 2). Ignore during transitions. */
    if (input_state -> mouse_lock_state != 2) {
        return FALSE;
    }

    /* Use widget-space coordinates for HiDPI consistency.
     * event->x/y are in widget space (affected by scale factor).
     * event->x_root/y_root are in screen space (not scaled). */
    int current_mouse_x = (int) event -> x;
    int current_mouse_y = (int) event -> y;

    /* Suppress delta during menu/overlay interactions. */
    if (input_state -> suppress_mouse_delta) {
        input_state -> last_mouse_x = current_mouse_x;
        input_state -> last_mouse_y = current_mouse_y;
        input_state -> mouse_centered = true;
        return FALSE;
    }

    /* First event after lock: baseline only, no delta. */
    if (!input_state -> mouse_centered) {
        input_state -> last_mouse_x = current_mouse_x;
        input_state -> last_mouse_y = current_mouse_y;
        input_state -> mouse_centered = true;
        return FALSE;
    }

    /* Compute relative delta in widget space. */
    int rel_dx = current_mouse_x - input_state -> last_mouse_x;
    int rel_dy = current_mouse_y - input_state -> last_mouse_y;
    input_state -> last_mouse_x = current_mouse_x;
    input_state -> last_mouse_y = current_mouse_y;

    /* Clamp single-event spikes (ballistics bursts, pointer jumps). */
    if (rel_dx > 80) {rel_dx = 80;}
    if (rel_dx < -80) {rel_dx = -80;}
    if (rel_dy > 80) {rel_dy = 80;}
    if (rel_dy < -80) {rel_dy = -80;}

    if ((rel_dx != 0) || (rel_dy != 0)) {
        input_state -> mouse_delta_accum_x += (float) rel_dx;
        input_state -> mouse_delta_accum_y += -(float) rel_dy;  // Invert Y for camera pitch

        /* Clamp per-frame accumulated total against flings. */
        if (input_state -> mouse_delta_accum_x > 240.0f) {input_state -> mouse_delta_accum_x = 240.0f;}
        if (input_state -> mouse_delta_accum_x < -240.0f) {input_state -> mouse_delta_accum_x = -240.0f;}
        if (input_state -> mouse_delta_accum_y > 240.0f) {input_state -> mouse_delta_accum_y = 240.0f;}
        if (input_state -> mouse_delta_accum_y < -240.0f) {input_state -> mouse_delta_accum_y = -240.0f;}
    }

    /* Edge-triggered recenter: only when cursor nears widget edge.
     * Uses widget space (event->x/y vs allocated size) for HiDPI correctness. */
    {
        int widget_width = gtk_widget_get_allocated_width (widget);
        int widget_height = gtk_widget_get_allocated_height (widget);
        double edge_x = event -> x;
        double edge_y = event -> y;
        double margin_x = (double) widget_width / 5.0;
        double margin_y = (double) widget_height / 5.0;
        if ((margin_x >= 1.0) && (margin_y >= 1.0)) {
            if ((edge_x < margin_x) ||
                (edge_x > (double) widget_width - margin_x) ||
                (edge_y < margin_y) ||
                (edge_y > (double) widget_height - margin_y)) {
                mouse_lock_reset_centre (widget);
                input_state -> mouse_centered = false;
            }
        }
    }

    return FALSE;
} gboolean on_button_press (GtkWidget *widget, GdkEventButton *event, gpointer user_data_stored) {
    input_status *input_state = (input_status *) user_data_stored;
    /* Left click only locks the mouse (no selection action attached). */
    if (event -> button == 2) {input_state -> middle_mouse_button_clicked = true;}
    if (event -> button == 3) {input_state -> right_mouse_button_clicked = true;}
    if (event -> button == 1 && input_state -> mouse_lock_state == 0) {
        input_state -> mouse_lock_state = 1;  // locking
        input_state -> mouse_delta_x = 0.0f;
        input_state -> mouse_delta_y = 0.0f;
        input_state -> mouse_delta_accum_x = 0.0f;
        input_state -> mouse_delta_accum_y = 0.0f;
        input_state -> last_mouse_x = 0;
        input_state -> last_mouse_y = 0;
        input_state -> mouse_centered = false;
        input_state -> suppress_mouse_delta = false;
        mouse_lock_enable (gtk_widget_get_toplevel (widget));
        input_state -> is_mouse_locked = true;
        input_state -> mouse_lock_state = 2;  // locked
    }
    return FALSE;
} gboolean on_button_release (GtkWidget *widget, GdkEventButton *event, gpointer user_data_stored) {
    (void) widget;
    input_status *input_state = (input_status *) user_data_stored;
    if (event -> button == 2) {input_state -> middle_mouse_button_clicked = false;}
    if (event -> button == 3) {input_state -> right_mouse_button_clicked = false;}
    return FALSE;
} gboolean on_focus_out (GtkWidget *widget, GdkEventFocus *event, gpointer user_data_stored) {
    (void) widget;
    (void) event;
    input_status *input_state = (input_status *) user_data_stored;
    input_state -> w_key_pressed = false;
    input_state -> a_key_pressed = false;
    input_state -> s_key_pressed = false;
    input_state -> d_key_pressed = false;
    input_state -> space_key_pressed = false;
    input_state -> shift_key_pressed = false;
    input_state -> escape_key_pressed = false;
    input_state -> f_key_pressed = false;
 /* q/m/delete/t REMOVED (dead or deleted keybinds). */
    input_state -> i_key_pressed = false;
    input_state -> j_key_pressed = false;
    input_state -> k_key_pressed = false;
    input_state -> l_key_pressed = false;
     /* MPE_TASK_21_KEYBOARD_ONLY_FOCUS_BEGIN (r only) */
input_state -> r_key_pressed = false;
/* MPE_TASK_21_KEYBOARD_ONLY_FOCUS_END */
input_state -> up_arrow_pressed = false;
    input_state -> down_arrow_pressed = false;
    input_state -> enter_key_pressed = false;
    input_state -> e_key_pressed = false;
input_state -> stability_test_pressed = false;
input_state -> sleep_wake_test_pressed = false;
input_state -> editor_torture_pressed = false;
input_state -> spawn_stress_pressed = false;
input_state -> validation_report_pressed = false;
    /* MPE_TASK_13_LONG_RUN_FOCUS_BEGIN */
input_state -> long_run_validation_pressed = false;
/* MPE_TASK_13_LONG_RUN_FOCUS_END */
/* MPE_TASK_39_CONFIG_TORTURE_FOCUS_BEGIN */
input_state -> config_torture_pressed = false;
/* MPE_TASK_39_CONFIG_TORTURE_FOCUS_END */
/* MPE_TASK_18_TERMINAL_FOCUS_BEGIN */
input_state -> debug_terminal_pressed = false;
/* MPE_TASK_18_TERMINAL_FOCUS_END */
/* MPE_TASK_22_ENTER_SPAWN_FOCUS_BEGIN */
input_state -> enter_spawn_held = false;
/* MPE_TASK_22_ENTER_SPAWN_FOCUS_END */
/* Full mouse state reset on focus loss. */
    input_state -> mouse_delta_x = 0.0f;
    input_state -> mouse_delta_y = 0.0f;
    input_state -> mouse_delta_accum_x = 0.0f;
    input_state -> mouse_delta_accum_y = 0.0f;
    input_state -> right_mouse_button_clicked = false;
    input_state -> middle_mouse_button_clicked = false;
    input_state -> suppress_mouse_delta = false;
    input_state -> last_mouse_x = 0;
    input_state -> last_mouse_y = 0;
    input_state -> mouse_centered = false;

    if (input_state -> is_mouse_locked) {
        input_state -> mouse_lock_state = 3;  // unlocking
        GtkWidget *a3_toplevel_widget = gtk_widget_get_toplevel (widget);
        mouse_lock_disable (a3_toplevel_widget);
        input_state -> is_mouse_locked = false;
        input_state -> mouse_lock_state = 0;  // unlocked
    }

    return FALSE;
}
