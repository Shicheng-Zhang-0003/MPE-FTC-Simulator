/* WIN_PORT_GAMEPAD: dual-platform gamepad (XInput / evdev) */
#include "gamepad.h"
#include <string.h>
#include <stdio.h>

#ifdef _WIN32
/* ---- Windows XInput ---- */
#include <windows.h>
#include <xinput.h>

static gamepad_state g_primary_gamepad;

gamepad_state *gamepad_get_primary(void) { return &g_primary_gamepad; }

bool gamepad_init(gamepad_state *pad, const char *device_path) {
    (void)device_path;
    if (!pad) return false;
    memset(pad, 0, sizeof(gamepad_state));
    pad->fd = -1;
    pad->deadzone = 0.15f;
    XINPUT_STATE st;
    if (XInputGetState(0, &st) == ERROR_SUCCESS) {
        pad->connected = true;
        pad->fd = 0;
        printf("[gamepad] XInput device 0 connected\n");
        return true;
    }
    pad->connected = false;
    fprintf(stderr, "[gamepad] no XInput device found\n");
    fprintf(stderr, "[gamepad] hint: set F310 switch to X mode\n");
    return false;
}

void gamepad_close(gamepad_state *pad) {
    if (!pad) return;
    pad->connected = false;
    pad->fd = -1;
}

void gamepad_poll(gamepad_state *pad) {
    if (!pad) return;

    /* MFS_323: Periodic reconnect attempt when disconnected.
     * Try to reopen the device every 60 frames (~1 second). */
    if (!pad->connected || pad->fd < 0) {
        static int reconnect_counter = 0;
        if (++reconnect_counter >= 60) {
            reconnect_counter = 0;
#ifdef _WIN32
            XINPUT_STATE probe_st;
            if (XInputGetState(0, &probe_st) == ERROR_SUCCESS) {
                pad->connected = true;
                pad->fd = 0;
                memset(pad->axes, 0, sizeof(pad->axes));
                memset(pad->buttons, 0, sizeof(pad->buttons));
                printf("[gamepad] XInput device 0 reconnected\n");
            }
#else
            int probe_fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
            if (probe_fd >= 0) {
                pad->connected = true;
                pad->fd = probe_fd;
                memset(pad->axes, 0, sizeof(pad->axes));
                memset(pad->buttons, 0, sizeof(pad->buttons));
                printf("[gamepad] device reconnected\n");
            }
#endif
        }
        return;
    }
    XINPUT_STATE st;
    if (XInputGetState((DWORD)pad->fd, &st) != ERROR_SUCCESS) {
        pad->connected = false;
        /* MFS_322: Zero all input state on disconnect.
         * Prevents stale axis/button values from being used
         * if a future code path doesn't check connected. */
        memset(pad->axes, 0, sizeof(pad->axes));
        memset(pad->buttons, 0, sizeof(pad->buttons));
        return;
    }
    pad->axes[gamepad_axis_left_x]        = (float)st.Gamepad.sThumbLX / 32768.0f;
    pad->axes[gamepad_axis_left_y]        = (float)st.Gamepad.sThumbLY / 32768.0f;
    pad->axes[gamepad_axis_right_x]       = (float)st.Gamepad.sThumbRX / 32768.0f;
    pad->axes[gamepad_axis_right_y]       = (float)st.Gamepad.sThumbRY / 32768.0f;
    pad->axes[gamepad_axis_left_trigger]  = (float)st.Gamepad.bLeftTrigger  / 255.0f;
    pad->axes[gamepad_axis_right_trigger] = (float)st.Gamepad.bRightTrigger / 255.0f;
    WORD b = st.Gamepad.wButtons;
    pad->buttons[gamepad_button_a]       = (b & XINPUT_GAMEPAD_A) != 0;
    pad->buttons[gamepad_button_b]       = (b & XINPUT_GAMEPAD_B) != 0;
    pad->buttons[gamepad_button_x]       = (b & XINPUT_GAMEPAD_X) != 0;
    pad->buttons[gamepad_button_y]       = (b & XINPUT_GAMEPAD_Y) != 0;
    pad->buttons[gamepad_button_lb]      = (b & XINPUT_GAMEPAD_LEFT_SHOULDER) != 0;
    pad->buttons[gamepad_button_rb]      = (b & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0;
    pad->buttons[gamepad_button_back]    = (b & XINPUT_GAMEPAD_BACK) != 0;
    pad->buttons[gamepad_button_start]   = (b & XINPUT_GAMEPAD_START) != 0;
    pad->buttons[gamepad_button_guide]   = false;
    pad->buttons[gamepad_button_stick_l] = (b & XINPUT_GAMEPAD_LEFT_THUMB) != 0;
    pad->buttons[gamepad_button_stick_r] = (b & XINPUT_GAMEPAD_RIGHT_THUMB) != 0;
}

#else
/* ---- Linux evdev ---- */
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <linux/joystick.h>

static gamepad_state g_primary_gamepad;

gamepad_state *gamepad_get_primary(void) { return &g_primary_gamepad; }

bool gamepad_init(gamepad_state *pad, const char *device_path) {
    if (!pad) return false;
    memset(pad, 0, sizeof(gamepad_state));
    pad->fd = -1;
    pad->deadzone = 0.15f;
    if (!device_path) device_path = "/dev/input/js0";
    strncpy(pad->device_path, device_path, sizeof(pad->device_path) - 1);
    pad->device_path[sizeof(pad->device_path) - 1] = '\0';
    pad->fd = open(device_path, O_RDONLY | O_NONBLOCK);
    if (pad->fd < 0) {
        fprintf(stderr, "[gamepad] could not open %s: %s\n",
                device_path, strerror(errno));
        fprintf(stderr, "[gamepad] hint: sudo usermod -aG input $USER\n");
        pad->connected = false;
        return false;
    }
    pad->connected = true;
    printf("[gamepad] opened %s\n", device_path);
    return true;
}

void gamepad_close(gamepad_state *pad) {
    if (!pad) return;
    if (pad->fd >= 0) { close(pad->fd); pad->fd = -1; }
    pad->connected = false;
}

void gamepad_poll(gamepad_state *pad) {
    if (!pad || !pad->connected || pad->fd < 0) return;
    struct js_event ev;
    ssize_t bytes;
    while ((bytes = read(pad->fd, &ev, sizeof(ev))) == sizeof(ev)) {
        __u8 type = ev.type & ~JS_EVENT_INIT;
        if (type == JS_EVENT_AXIS) {
            if (ev.number < gamepad_axis_count)
                pad->axes[ev.number] = (float)ev.value / 32767.0f;
        } else if (type == JS_EVENT_BUTTON) {
            if (ev.number < gamepad_button_count)
                pad->buttons[ev.number] = (ev.value != 0);
        }
    }
    if (bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        close(pad->fd); pad->fd = -1; pad->connected = false;
    }
}
#endif /* _WIN32 */

/* ---- shared helpers ---- */
static float apply_deadzone(float v, float dz) {
    if (v >  dz) return (v - dz) / (1.0f - dz);
    if (v < -dz) return (v + dz) / (1.0f - dz);
    return 0.0f;
}

float gamepad_get_axis(const gamepad_state *pad, int axis) {
    if (!pad || axis < 0 || axis >= gamepad_axis_count) return 0.0f;
    float v = pad->axes[axis];
    if (axis == gamepad_axis_left_y  && pad->invert_left_y)  v = -v;
    if (axis == gamepad_axis_left_x  && pad->invert_left_x)  v = -v;
    if (axis == gamepad_axis_right_x && pad->invert_right_x) v = -v;
    return apply_deadzone(v, pad->deadzone);
}

bool gamepad_get_button(const gamepad_state *pad, int button) {
    if (!pad || button < 0 || button >= gamepad_button_count) return false;
    return pad->buttons[button];
}

bool gamepad_is_connected(const gamepad_state *pad) {
    return pad ? pad->connected : false;
}

void gamepad_set_deadzone(gamepad_state *pad, float dz) {
    if (!pad) return;
    if (dz < 0.0f) dz = 0.0f;
    if (dz > 0.9f) dz = 0.9f;
    pad->deadzone = dz;
}
