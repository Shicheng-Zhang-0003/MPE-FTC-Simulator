#include "mpe_engine.h"
#include "core/validation_report.h"
#include "physics/depenetration.h"
#include "core/simulation_camera.h"
#include "core/simulation_physics_loop.h"
#include "core/long_run_validation.h"
#include "robotics/gui_robot_registry.h"

#include "ui_input/simulation_dispatch.h"
#include <gtk/gtk.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdatomic.h>
#include <time.h>
#ifdef _WIN32 /* WIN_PORT: _mkdir/_access */
#include <direct.h>
#include <io.h>
#else
#include <sys/stat.h>
#include <unistd.h> /* MPE_TASK_39 access() */
#endif
#ifdef _WIN32
#define mkdir(p, m) _mkdir(p)
#ifndef F_OK
#define F_OK 0
#endif
#endif

/* ------------------------------------------------------------------ */
/* Physics thread state                                               */
/* ------------------------------------------------------------------ */

static GThread *physics_thread = NULL;
static GMutex physics_mutex;
static GCond physics_cond;
/* FIX-AUDIT: world mutex. The physics worker steps bodies[] while the GTK
 * callback used to write accumulators (gui_robot_tick) and read bodies
 * (sleeping-count scan, validation, overlay) on the UI thread with no
 * lock — torn reads and heap races under F11/torture. physics_mutex only
 * guarded the wake flag. world_mutex serializes: worker holds it across
 * physics_world_step; the UI callback holds it across gui_robot_tick +
 * signal + post-step reads. Short critical sections only (no GTK calls
 * that re-enter the main loop inside). */
static GMutex world_mutex;
static bool world_mutex_ready = false;
static _Atomic bool physics_thread_running = false;
static _Atomic bool physics_thread_should_stop = false;
static _Atomic int physics_tick_requested = 0;

/* ------------------------------------------------------------------ */
/* Frame timer and debug state                                        */
/* ------------------------------------------------------------------ */

frame_timer main_timer;

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

/* ------------------------------------------------------------------ */
/* Physics thread function                                            */
/* ------------------------------------------------------------------ */

static gpointer physics_thread_func(gpointer user_data) {
    (void) user_data;

    const float fixed_physics_dt = 1.0f / 60.0f;
    const gint64 tick_interval_us = (gint64)(fixed_physics_dt * 1000000.0);
    gint64 next_wakeup = g_get_monotonic_time() + tick_interval_us;

    while (!physics_thread_should_stop) {
        /* Wait until next fixed timestep */
        g_mutex_lock(&physics_mutex);
        gboolean timed_out = FALSE;
        while (!physics_tick_requested && !physics_thread_should_stop) {
            gint64 now = g_get_monotonic_time();
            gint64 wait_us = next_wakeup - now;
            if (wait_us > 0) {
                timed_out = !g_cond_wait_until(&physics_cond, &physics_mutex, now + wait_us);
            } else {
                timed_out = TRUE;
            }
        }
        bool tick_now = physics_tick_requested || timed_out;
        physics_tick_requested = 0;
        g_mutex_unlock(&physics_mutex);

        if (tick_now || physics_thread_should_stop) {
            if (physics_thread_should_stop) break;

            /* Run one fixed physics tick (serialized vs the UI thread). */
            physics_world *world = physics_world_get_primary();
            if (world && world->bodies && world->body_count > 0) {
                g_mutex_lock(&world_mutex);
                physics_world_step(world, fixed_physics_dt);
                g_mutex_unlock(&world_mutex);
            }

            /* Schedule next wakeup: fixed_physics_dt from NOW (not from last wakeup)
             * to prevent drift accumulation. */
            next_wakeup = g_get_monotonic_time() + tick_interval_us;
        }

        /* Yield to main thread - prevent CPU starvation on heavy scenes */
        g_usleep(200);
    }

    physics_thread_running = false;
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Public API                                                         */
/* ------------------------------------------------------------------ */

void physics_thread_start(void) {
    if (physics_thread_running) return;

    g_mutex_init(&physics_mutex);
    g_cond_init(&physics_cond);
    if (!world_mutex_ready) {
        g_mutex_init(&world_mutex);
        world_mutex_ready = true;
    }
    physics_thread_should_stop = false;
    physics_tick_requested = 0;
    physics_thread_running = true;

    physics_thread = g_thread_new("MPE-Physics", physics_thread_func, NULL);
}

void physics_thread_stop(void) {
    if (!physics_thread_running) return;

    physics_thread_should_stop = true;
    physics_tick_requested = 1; /* Wake it up to exit */
    g_cond_signal(&physics_cond);

    if (physics_thread) {
        g_thread_join(physics_thread);
        physics_thread = NULL;
    }

    g_mutex_clear(&physics_mutex);
    g_cond_clear(&physics_cond);
}

bool physics_thread_is_running(void) {
    return physics_thread_running;
}

void physics_thread_wake(void) {
    if (!physics_thread_running) return;
    physics_tick_requested = 1;
    g_cond_signal(&physics_cond);
}

void world_lock(void) {
    if (world_mutex_ready) {
        g_mutex_lock(&world_mutex);
    }
}

void world_unlock(void) {
    if (world_mutex_ready) {
        g_mutex_unlock(&world_mutex);
    }
}

/* ------------------------------------------------------------------ */
/* GTK callback - now ONLY does render/input/UI, NO physics          */
/* ------------------------------------------------------------------ */

gboolean physics_step_increment(gpointer user_data_pointer) {
    GtkWidget *parent_window = NULL;
    if (user_data_pointer) {
        parent_window = gtk_widget_get_toplevel(GTK_WIDGET(user_data_pointer));
    }

    /* Guard checks */
    if (editor_dialog_is_active()) {
        /* Still need to pump GTK events, but no physics */
        return TRUE;
    }
    if (physics_halt_tick_update()) {
        gtk_widget_queue_draw(GTK_WIDGET(user_data_pointer));
        overlay_update();
        return TRUE;
    }

    /* Mode watch */
    static bool a3_previous_debug_mode_state = false;
    static bool a3_debug_mode_watch_ready = false;
    if (!a3_debug_mode_watch_ready) {
        a3_debug_mode_watch_ready = true;
        a3_previous_debug_mode_state = main_inputs.is_debug_mode_active;
    } else if (main_inputs.is_debug_mode_active != a3_previous_debug_mode_state) {
        a3_previous_debug_mode_state = main_inputs.is_debug_mode_active;
        debug_terminal_sync_mode();
    }

    /* Status dir + frame timer (for camera/UI only) */
    static int status_dir_checked = 0;
#ifdef _WIN32
    if (!status_dir_checked) { _mkdir("status"); status_dir_checked = 1; }
#else
    if (!status_dir_checked) { mkdir("status", 0755); status_dir_checked = 1; }
#endif
    frame_timer_update(&main_timer);
    float frame_delta_time = main_timer.delta_time;
    debug_last_frame_time = frame_delta_time;

    /* Camera + character (uses frame_delta_time for smooth movement) */
    simulation_camera_tick(frame_delta_time);

    /* Input dispatch (mouse/keyboard bindings, menus, spawn) */
    simulation_input_dispatch(parent_window);

    /* Menu handling */
    simulation_menu_dispatch(parent_window);
    editor_update_menus(parent_window);
    config_menu_update(parent_window);

    /* FTC robots: motor updates before the physics step.
     * FIX-AUDIT: the whole UI-side world critical section (motor writes,
     * wake signal, post-step reads) runs under world_mutex, serialized
     * against the worker's physics_world_step. */
    world_lock();
    gui_robot_tick(frame_delta_time);

    /* Signal physics thread to run one tick */
    physics_thread_wake();

    /* Post-physics bookkeeping (UI only) */
    gtk_widget_queue_draw(GTK_WIDGET(user_data_pointer));
    int a3_sleeping_object_count = 0;
    physics_world *world = physics_world_get_primary();
    if (world) {
        for (int sleep_count_index = 0; sleep_count_index < world->body_count; sleep_count_index++) {
            if (world->bodies[sleep_count_index].is_sleeping) {
                a3_sleeping_object_count++;
            }
        }
        debug_last_sleeping_object_count = a3_sleeping_object_count;
        debug_last_object_count = world->body_count;
    }
    long_run_validation_tick_update();
    overlay_update();
    world_unlock();

    return TRUE;
}

/* ------------------------------------------------------------------ */
/* a3_positional_depenetration_pass now lives in physics/depenetration.c. */