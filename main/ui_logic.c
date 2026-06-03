/**
 * @file ui_logic.c
 * @brief UI Logic Layer - Bridge between system metrics and LVGL display
 * @author Rallybox Development Team
 * @developer Akhil
 *
 * This module implements the application logic layer that:
 * 1. Bridges system_monitor metrics to LVGL UI components
 * 2. Manages file browser (file manager) for SD cards
 * 3. Handles WiFi configuration and credential management
 * 4. Implements text display components for logs and settings
 * 5. Manages screen transitions and navigation
 *
 * Architecture:
 * - **Update Functions**: ui_logic_update_*() called periodically from main task
 * - **Event Handlers**: lv_event_t callbacks for button presses, selections
 * - **File Manager**: Embedded directory browser with file preview
 *
 * Color Scheme (Material Design):
 * - Background (BG): 0xff0f1419 (very dark blue-grey)
 * - Surface: 0xff182028 (dark surface)
 * - Accent: 0xff6ba6a8 (teal)
 * - Text: 0xfff2f4f5 (light grey-white)
 * - Success: 0xff77b255 (green)
 * - Warning: 0xffd4a14a (amber)
 * - Danger: 0xffd77474 (red)
 *
 * Components (2KB Text Buffer Limits):
 * - File Manager: Max 64 items, 4 KB preview per file
 * - Text Display: 8 KB textarea for logs/reports
 * - File Preview: 160 lines × 72 chars (4096 bytes max)
 *
 * @note All display operations lock the display mutex to prevent tearing
 * @note File operations may block briefly for I/O; called from UI task only
 * @note WiFi credentials stored permanently in NVS (auto-restored on boot)
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <dirent.h>
#include <sys/stat.h>
#include <stdint.h>
#include <errno.h>
#include "lvgl.h"                   ///< LVGL graphics library
#include "GPS_points.h"
#include "ui.h"                     ///< Generated UI definitions (EEZ Studio)
#include "screens.h"                ///< Screen switching functions
#include "esp_log.h"                ///< ESP-IDF logging
#include "esp_heap_caps.h"          ///< PSRAM/internal heap alloc caps
#include "esp_http_client.h"        ///< HTTP transport config for OTA
#include "esp_ota_ops.h"            ///< OTA validation error codes
#include "esp_partition.h"
#include "esp_https_ota.h"          ///< HTTPS OTA update support
#include "esp_wifi.h"               ///< Wi-Fi power save tuning for OTA
#include "esp_mac.h"                ///< Base MAC for stable device ID
#include "esp_system.h"             ///< Restart after successful OTA
#include "nvs_flash.h"              ///< Non-volatile storage API
#include "nvs.h"                    ///< NVS namespace handle
#include "esp_timer.h"              ///< System timer for timestamps
#include "driver/uart.h"            ///< UART driver for GNSS monitor
#include "sd_card.h"                ///< SD card module
#if __has_include("esp_crt_bundle.h")
#include "esp_crt_bundle.h"
#define UI_HAS_CRT_BUNDLE 1
#else
#define UI_HAS_CRT_BUNDLE 0
#endif
#if CONFIG_RALLYBOX_GNSS_ENABLED
#include "gnss.h"                   ///< GNSS backend runtime control
#endif
#include "system_monitor.h"         ///< System monitoring/status
#include "debug_flags.h"
#include "psa/crypto.h"
#if CONFIG_RALLYBOX_RACEBOX_ENABLED
#include "racebox.h"                ///< RaceBox BLE scan/connect APIs
#endif
#include "freertos/FreeRTOS.h"      ///< Real-time OS kernel
#include "freertos/semphr.h"        ///< Mutex/semaphore support
#include "freertos/task.h"          ///< Task management

static const char* TAG = "UI_Logic";   ///< Log tag for this module

#define BLUETOOTH_TAB_INDEX 2
#define GNSS_TAB_INDEX 3
#define BLUETOOTH_DROPDOWN_REFRESH_MS 2000
#define BLUETOOTH_DUMP_PENDING_MAX 4096
#define BLUETOOTH_DUMP_LINE_MAX 192
#define RACEBOX_AUTO_CONNECT_TIMEOUT_MS 12000U

#define GNSS_DUMP_PENDING_MAX 4096
#define GNSS_DUMP_LINE_MAX 192
#define UART_MONITOR_MAX_CHARS 1600
#define UART_MONITOR_LINES 20
#define UART_STREAM_BUFFER_SIZE CONFIG_RALLYBOX_STREAM_BUFFER_BYTES
#define UART_STREAM_CHUNK_MAX 384

#define UI_COLOR_BG 0xff0f1419
#define UI_COLOR_SURFACE 0xff182028
#define UI_COLOR_SURFACE_ALT 0xff22303b
#define UI_COLOR_BORDER 0xff31404c
#define UI_COLOR_ACCENT 0xff6ba6a8
#define UI_COLOR_ACCENT_ALT 0xff4f8d8f
#define UI_COLOR_TEXT 0xfff2f4f5
#define UI_COLOR_MUTED 0xff9fadb8
#define UI_COLOR_SUCCESS 0xff77b255
#define UI_COLOR_WARNING 0xffd4a14a
#define UI_COLOR_DANGER 0xffd77474
#define UI_LED_ACTIVITY_WINDOW_US 1500000ULL
#define UI_LED_BLINK_PERIOD_US 350000ULL
#define UI_OTA_PROGRESS_PERCENT_STEP 20
#define UI_OTA_PROGRESS_KB_STEP 1024
#define UI_OTA_PROGRESS_UI_INTERVAL_MS 4000U
#define UI_OTA_WAITING_STATUS_INTERVAL_MS 8000U
#define UI_OTA_MANIFEST_FILENAME "ota-manifest.json"
#define UI_OTA_MANIFEST_MAX_BYTES 2048U
#define UI_OTA_HASH_CHUNK_BYTES 4096U
#define UI_OTA_SHA256_HEX_LEN 64U

#define MENU_PANEL_LEFT_X 11
#define MENU_PANEL_RIGHT_X 584
#define MENU_PANEL_TOP_Y 96
#define MENU_PANEL_BOTTOM_Y 354
#define MENU_PANEL_WIDTH 561
#define MENU_PANEL_HEIGHT 248
#define MENU_PANEL_PAD_X 18
#define MENU_PANEL_CONTENT_WIDTH (MENU_PANEL_WIDTH - (MENU_PANEL_PAD_X * 2))
#define MENU_PANEL_HALF_BUTTON_WIDTH 250
#define MENU_PANEL_THIRD_BUTTON_WIDTH 167
#define MENU_PANEL_ACTION_COLUMN_X 376
#define MENU_PANEL_ACTION_TOP_Y 136
#define MENU_PANEL_ACTION_BOTTOM_Y 184
#define MENU_PANEL_ACTION_BUTTON_HEIGHT 42
#define MENU_PANEL_DETAIL_WIDTH 340
#define MENU_PANEL_COUNT_RIGHT_X 280

#define FILE_MANAGER_MAX_ITEMS 64
#define FILE_PREVIEW_MAX_BYTES 4096
#define FILE_PREVIEW_MAX_LINES 160
#define FILE_PREVIEW_MAX_LINE_LENGTH 72
#define UI_OTA_RESTART_DELAY_MS 5000

#ifndef CONFIG_RALLYBOX_OTA_S3_BUCKET
#define CONFIG_RALLYBOX_OTA_S3_BUCKET ""
#endif

#ifndef CONFIG_RALLYBOX_OTA_S3_REGION
#define CONFIG_RALLYBOX_OTA_S3_REGION ""
#endif

#ifndef CONFIG_RALLYBOX_OTA_S3_OBJECT_PREFIX
#define CONFIG_RALLYBOX_OTA_S3_OBJECT_PREFIX "firmware/RallyBox-Dashboard/"
#endif

#ifndef CONFIG_RALLYBOX_OTA_FILENAME
#define CONFIG_RALLYBOX_OTA_FILENAME "RallyBox-Dashboard.bin"
#endif

#ifndef CONFIG_RALLYBOX_GPX_S3_VIRTUAL_HOSTED_STYLE
#define CONFIG_RALLYBOX_GPX_S3_VIRTUAL_HOSTED_STYLE 1
#endif

#ifndef CONFIG_RALLYBOX_GPX_S3_USE_HTTPS
#define CONFIG_RALLYBOX_GPX_S3_USE_HTTPS 1
#endif

/* ─────────────────────────────────────────────────────────────────────────
 * FORWARD DECLARATIONS
 *
 * These functions are implemented later in this file and called from
 * event handlers or other functions defined before their implementation.
 * ───────────────────────────────────────────────────────────────────────── */

 /// Event handler for on-screen keyboard display
void action_show_keyboard(lv_event_t* e);

/// Event handler for WiFi password/SSID keyboard modal
void action_password_ssid_keyboad(lv_event_t* e);

/// Display error message dialog with error code details
static void ui_handle_error(const char* msg, esp_err_t err);

/// Display a modal message dialog
void ui_show_message(const char* title, const char* msg, lv_color_t color);

/// Remove/forget saved WiFi credentials from NVS
static void action_remove_saved_wifi(lv_event_t* e);

// File manager functions
void file_manager_load_files(const char* mount_point, lv_obj_t* list_obj);
void file_manager_file_selected(lv_event_t* e);
void notepad_viewer_load_file(const char* filepath, lv_obj_t* preview_container);
lv_obj_t* create_file_manager_screen(lv_obj_t* parent);
lv_obj_t* create_settings_screen(lv_obj_t* parent);
lv_obj_t* create_minimal_keyboard(lv_obj_t* parent);

// Tab switching callbacks
static void file_manager_tab_sd1_cb(lv_event_t* e);
static void file_manager_tab_sd2_cb(lv_event_t* e);
static void file_modal_close_cb(lv_event_t* e);
static void file_modal_delete_cb(lv_event_t* e);
static void file_manager_initial_load_async(void* user_data);
static void file_manager_show_placeholder(const char* text);
static lv_obj_t* ui_get_first_child(lv_obj_t* obj);
static void ui_update_storage_dependent_controls(const system_status_t* status);
static void ui_refresh_bluetooth_controls(const system_status_t* status);
static bool ui_is_bluetooth_tab_active(void);
static bool ui_is_gnss_tab_active(void);
static bool ui_is_files_tab_active(void);
static bool ui_is_settings_tab_active(void);
static void ui_refresh_bluetooth_dropdown(bool force);
static void ui_style_surface(lv_obj_t* obj, lv_color_t bg_color, lv_coord_t radius);
static void ui_create_main_status_panels(void);
static void ui_update_main_status_panels(const system_status_t* status);
static void action_main_panel_racebox_click(lv_event_t* e);
static void action_main_panel_gnss_click(lv_event_t* e);
static void action_bluetooth_refresh(lv_event_t* e);
static void action_bluetooth_connect(lv_event_t* e);
static void action_gnss_start_stop(lv_event_t* e);
static void ui_refresh_gnss_dump(void);
static void ui_refresh_bluetooth_dump(void);
static void ui_focus_bluetooth_connect_button(void);
static void action_bluetooth_close(lv_event_t* e);
static void ui_show_dashboard_tab(bool animate);
static void ui_open_bluetooth_panel(bool request_scan);
static void ui_prepare_bluetooth_panel(bool request_scan, bool show_messages);
static void ui_begin_racebox_auto_connect(void);
static void ui_cancel_racebox_auto_connect(void);
static void ui_fallback_to_bluetooth_panel(void);
static bool ui_racebox_status_has_failure(const char* text);
static void ui_update_racebox_auto_connect(const system_status_t* status);
static void ui_refresh_settings_controls(void);
static void ui_settings_filter_checkbox_cb(lv_event_t* e);
static void ui_settings_stationary_threshold_cb(lv_event_t* e);
static void action_settings_ota_update(lv_event_t* e);
static const char* ui_get_build_timestamp(void);
static void ui_refresh_ota_button_state(void);
static bool ui_is_ota_progress_message(const char* msg);
static void ui_refresh_ota_progress_widgets(void);
static bool ui_build_ota_object_request(const char* object_name, char* out_host, size_t out_host_len, char* out_path, size_t out_path_len, char* out_url, size_t out_url_len);
static bool ui_build_ota_request(char* out_host, size_t out_host_len, char* out_path, size_t out_path_len, char* out_url, size_t out_url_len);
static bool ui_build_ota_url(char* out_url, size_t out_url_len);
static bool ui_is_wifi_ready_for_ota(void);
static void ui_queue_deferred_message(const char* title, const char* msg, uint32_t color, bool restart_requested);
static void ui_process_deferred_message(void);
static void ui_restart_timer_cb(lv_timer_t* timer);
static void ui_ota_update_task(void* param);
static void style_modern_button(lv_obj_t* button, lv_color_t primary_color, lv_color_t secondary_color);

typedef struct
{
    bool available;
    int size_bytes;
    char sha256[(UI_OTA_SHA256_HEX_LEN + 1U)];
    char url[512];
} ui_ota_source_metadata_t;

static bool ui_fetch_ota_source_metadata(ui_ota_source_metadata_t* metadata);
static bool ui_compute_partition_sha256_hex(const esp_partition_t* partition, size_t image_size, char* out_hex, size_t out_hex_len);

typedef struct
{
    SemaphoreHandle_t mutex;
    char* data;
    size_t capacity;
    size_t head;
    size_t tail;
    bool active;
    bool overflowed;
    char partial_line[GNSS_DUMP_LINE_MAX];
    size_t partial_len;
} ui_stream_buffer_t;

typedef struct
{
    char lines[UART_MONITOR_LINES][GNSS_DUMP_LINE_MAX];
    uint8_t start;
    uint8_t count;
    bool dirty;
} ui_line_window_t;

static bool ui_stream_init(ui_stream_buffer_t* stream);
static void ui_stream_start(ui_stream_buffer_t* stream);
static void ui_stream_stop(ui_stream_buffer_t* stream);
static void ui_stream_push_bytes(ui_stream_buffer_t* stream, const uint8_t* data, size_t len);
static size_t ui_stream_pop_chunk(ui_stream_buffer_t* stream, char* out, size_t max_len, bool* had_overflow);
static void ui_window_reset(ui_line_window_t* window, lv_obj_t* panel);
static void ui_window_append_line(ui_line_window_t* window, const char* line);
static void ui_window_consume_chunk(ui_stream_buffer_t* stream, ui_line_window_t* window, const char* chunk, size_t len);
static void ui_window_render_if_dirty(ui_line_window_t* window, lv_obj_t* panel);

#if CONFIG_RALLYBOX_RACEBOX_ENABLED
typedef struct
{
    bool has_data;
    bool solution_valid;
    uint8_t fix_status;
    uint8_t satellites;
    double longitude_deg;
    double latitude_deg;
    float altitude_m;
    float speed_kph;
    float heading_deg;
    float pdop;
    float horizontal_accuracy_m;
    float vertical_accuracy_m;
    uint32_t packets;
} racebox_nav_snapshot_t;

typedef struct
{
    SemaphoreHandle_t mutex;
    uint8_t frame[520];
    size_t frame_len;
    racebox_nav_snapshot_t nav;
} racebox_decode_state_t;

static racebox_visible_device_t s_bt_visible_devices[RACEBOX_VISIBLE_DEVICES_MAX] = { 0 };
static size_t s_bt_visible_count = 0;
static uint64_t s_bt_last_dropdown_refresh_us = 0;
static char s_bt_last_options[1536] = { 0 };
static ui_stream_buffer_t s_bt_stream = { 0 };
static ui_line_window_t s_bt_window = { 0 };
static char s_bt_render_buf[UART_MONITOR_LINES * (BLUETOOTH_DUMP_LINE_MAX + 1)] = { 0 };
static lv_timer_t* s_bt_dump_timer = NULL;
static lv_obj_t* s_bt_dump_panel = NULL;
static bool s_bt_prev_connected = false;
static volatile bool s_bt_suspend_preserve_feed = false;
static lv_obj_t* s_bt_close_button = NULL;
static bool s_bt_auto_connect_pending = false;
static uint32_t s_bt_auto_connect_started_ms = 0;
static racebox_decode_state_t s_rb_decode = { 0 };
#endif

static ui_stream_buffer_t s_gnss_stream = { 0 };
static ui_line_window_t s_gnss_window = { 0 };
static char s_gnss_render_buf[UART_MONITOR_LINES * (GNSS_DUMP_LINE_MAX + 1)] = { 0 };
static bool s_gnss_listening = false;
static volatile uint32_t s_gnss_rx_count = 0;
static volatile uint32_t s_bt_rx_count = 0;
static char s_gnss_status_text[96] = "Idle. Enter UART settings, then press START";
static lv_timer_t* s_gnss_dump_timer = NULL;
static uint8_t s_bt_ui_last_connected = 0xFF;
static uint8_t s_bt_ui_last_initialized = 0xFF;

static lv_obj_t* s_panel_sd1 = NULL;
static lv_obj_t* s_panel_sd2 = NULL;
static lv_obj_t* s_panel_racebox = NULL;
static lv_obj_t* s_panel_racebox_led = NULL;
static lv_obj_t* s_panel_racebox_state = NULL;
static lv_obj_t* s_panel_racebox_count = NULL;
static lv_obj_t* s_panel_racebox_points = NULL;
static lv_obj_t* s_panel_racebox_detail = NULL;
static lv_obj_t* s_panel_racebox_fill_bar = NULL;
static lv_obj_t* s_panel_racebox_fill_label = NULL;
static lv_obj_t* s_panel_gnss = NULL;
static lv_obj_t* s_panel_gnss_led = NULL;
static lv_obj_t* s_panel_gnss_state = NULL;
static lv_obj_t* s_panel_gnss_count = NULL;
static lv_obj_t* s_panel_gnss_points = NULL;
static lv_obj_t* s_panel_gnss_detail = NULL;
static lv_obj_t* s_panel_gnss_fill_bar = NULL;
static lv_obj_t* s_panel_gnss_fill_label = NULL;

static lv_obj_t* s_btn_ble_sd1 = NULL;
static lv_obj_t* s_btn_ble_sd2 = NULL;
static lv_obj_t* s_btn_ble_web = NULL;
static lv_obj_t* s_btn_ble_reset = NULL;
static lv_obj_t* s_btn_gnss_sd1 = NULL;
static lv_obj_t* s_btn_gnss_sd2 = NULL;
static lv_obj_t* s_btn_gnss_web = NULL;
static lv_obj_t* s_btn_gnss_reset = NULL;
static volatile bool s_gpx_operation_in_progress = false;
static bool s_wifi_connected = false;
static char s_wifi_ip[16] = "0.0.0.0";
static uint32_t s_log_line_count = 1999;
static uint32_t s_log_file_index = 0;
static char current_log_filename[128] = "";

static lv_obj_t* g_files_tab = NULL;
static lv_obj_t* g_file_manager_screen = NULL;
static lv_obj_t* g_settings_tab = NULL;
static lv_obj_t* g_settings_screen = NULL;
static lv_obj_t* s_settings_rallybox_id_value = NULL;
static lv_obj_t* s_settings_firmware_version_value = NULL;
static lv_obj_t* s_settings_build_value = NULL;
static lv_obj_t* s_settings_ota_button = NULL;
static lv_obj_t* s_settings_ota_status_value = NULL;
static lv_obj_t* s_settings_ota_progress_row = NULL;
static lv_obj_t* s_settings_ota_progress_bar = NULL;
static lv_obj_t* s_settings_ota_progress_value = NULL;
static lv_obj_t* s_settings_filter_stationary_checkbox = NULL;
static lv_obj_t* s_settings_filter_impossible_checkbox = NULL;
static lv_obj_t* s_settings_stationary_hint_label = NULL;
static lv_obj_t* s_settings_impossible_hint_label = NULL;
static lv_obj_t* s_settings_stationary_radius_value = NULL;
static bool s_settings_ui_syncing = false;
static uint8_t s_settings_stationary_radius_m = 3;
static volatile bool s_ota_update_in_progress = false;
static char s_settings_ota_status_text[224] = "Ready to install uploaded OTA firmware.";
static uint32_t s_settings_ota_status_color = UI_COLOR_MUTED;
static uint32_t s_ota_progress_last_ui_update_ms = 0;
static bool s_settings_ota_progress_visible = false;
static int s_settings_ota_progress_percent = 0;
static char s_settings_ota_progress_value_text[32] = "0%";
static char s_settings_ota_status_text_cached[224] = "";
static uint32_t s_settings_ota_status_color_cached = UINT32_MAX;
static int s_settings_ota_progress_percent_cached = -1;
static bool s_settings_ota_progress_visible_cached = false;
static char s_settings_ota_progress_value_text_cached[32] = "";
static int8_t s_ota_button_disabled_cached = -1;

typedef struct
{
    bool pending;
    bool restart_requested;
    uint32_t color;
    char title[48];
    char message[224];
} ui_deferred_message_t;

static SemaphoreHandle_t s_ui_deferred_message_mutex = NULL;
static ui_deferred_message_t s_ui_deferred_message = { 0 };

typedef struct
{
    char path[256];
    char filename[256];
    char mount_point[32];
    bool is_dir;
    uint32_t size;
} file_entry_t;

typedef struct
{
    char full_path[512];
    char filename[256];
    char mount_point[32];
    bool is_dir;
    uint32_t size;
} file_item_context_t;

typedef struct
{
    char active_mount[32];
    lv_obj_t* list_container;
    lv_obj_t* status_label;
    lv_obj_t* sd1_button;
    lv_obj_t* sd2_button;
} file_manager_view_t;

static file_entry_t selected_file = { 0 };
static file_item_context_t g_file_items[FILE_MANAGER_MAX_ITEMS] = { 0 };
static file_manager_view_t g_file_manager_view = { 0 };
static bool s_files_sd1_available = true;
static bool s_files_sd2_available = true;
static uint32_t s_last_bt_count = 0;
static uint32_t s_last_gnss_count = 0;
static uint64_t s_bt_last_rx_change_us = 0;
static uint64_t s_gnss_last_rx_change_us = 0;
static bool s_rb_smooth_valid = false;
static float s_rb_speed_smooth = 0.0f;
static float s_rb_heading_smooth = 0.0f;
static bool s_gnss_smooth_valid = false;
static float s_gnss_speed_smooth = 0.0f;
static float s_gnss_heading_smooth = 0.0f;

static float ui_smooth_scalar(float prev, float current, float alpha)
{
    return prev + alpha * (current - prev);
}

static float ui_smooth_heading(float prev, float current, float alpha)
{
    float delta = current - prev;

    if (delta > 180.0f)
    {
        delta -= 360.0f;
    }
    else if (delta < -180.0f)
    {
        delta += 360.0f;
    }

    prev = prev + alpha * delta;
    while (prev < 0.0f)
    {
        prev += 360.0f;
    }
    while (prev >= 360.0f)
    {
        prev -= 360.0f;
    }
    return prev;
}

static lv_color_t ui_get_fill_bar_color(uint8_t fill_percent)
{
    if (fill_percent >= 90)
    {
        return lv_color_hex(UI_COLOR_DANGER);
    }
    if (fill_percent >= 70)
    {
        return lv_color_hex(UI_COLOR_WARNING);
    }
    return lv_color_hex(UI_COLOR_SUCCESS);
}

static lv_color_t ui_get_racebox_fix_led_color(bool solution_valid, uint8_t fix_status)
{
    if (!solution_valid)
    {
        return lv_color_hex(UI_COLOR_WARNING);
    }

    if (fix_status >= 3U)
    {
        return lv_color_hex(UI_COLOR_SUCCESS);
    }

    if (fix_status == 2U)
    {
        return lv_color_hex(UI_COLOR_ACCENT);
    }

    return lv_color_hex(UI_COLOR_WARNING);
}

static lv_color_t ui_get_gnss_fix_led_color(uint8_t fix_quality)
{
    switch (fix_quality)
    {
        case 4:
            return lv_color_hex(UI_COLOR_SUCCESS);
        case 2:
        case 3:
        case 5:
            return lv_color_hex(UI_COLOR_ACCENT);
        case 1:
            return lv_color_hex(UI_COLOR_ACCENT_ALT);
        case 6:
            return lv_color_hex(UI_COLOR_WARNING);
        default:
            return lv_color_hex(UI_COLOR_WARNING);
    }
}

static void ui_update_fix_led(lv_obj_t* led, bool receiving, lv_color_t active_color, uint64_t now_us)
{
    if (led == NULL)
    {
        return;
    }

    if (!receiving)
    {
        lv_led_set_color(led, lv_color_hex(UI_COLOR_MUTED));
        lv_led_off(led);
        return;
    }

    lv_led_set_color(led, active_color);

    if (((now_us / UI_LED_BLINK_PERIOD_US) & 1ULL) != 0ULL)
    {
        lv_led_off(led);
        return;
    }

    lv_led_on(led);
}

static void gnss_dump_timer_cb(lv_timer_t* t)
{
    LV_UNUSED(t);
    ui_refresh_gnss_dump();
}

#if CONFIG_RALLYBOX_RACEBOX_ENABLED
static void bluetooth_dump_timer_cb(lv_timer_t* t)
{
    LV_UNUSED(t);
    ui_refresh_bluetooth_dump();
}

static bool racebox_decode_ensure_mutex(void)
{
    if (s_rb_decode.mutex == NULL)
    {
        s_rb_decode.mutex = xSemaphoreCreateMutex();
    }
    return s_rb_decode.mutex != NULL;
}

static uint16_t racebox_le_u16(const uint8_t* p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t racebox_le_u32(const uint8_t* p)
{
    return (uint32_t)p[0] |
        ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24);
}

static int32_t racebox_le_i32(const uint8_t* p)
{
    return (int32_t)((uint32_t)p[0] |
        ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24));
}

static void racebox_ubx_checksum(const uint8_t* data, size_t len, uint8_t* ck_a, uint8_t* ck_b)
{
    size_t i;
    uint8_t a = 0;
    uint8_t b = 0;

    for (i = 0; i < len; ++i)
    {
        a = (uint8_t)(a + data[i]);
        b = (uint8_t)(b + a);
    }

    if (ck_a) *ck_a = a;
    if (ck_b) *ck_b = b;
}

static void racebox_decode_nav_payload_locked(const uint8_t* payload, size_t payload_len)
{
    uint8_t fix_status;
    uint8_t fix_flags;
    uint8_t lat_lon_flags;
    int32_t lon_1e7;
    int32_t lat_1e7;
    int32_t msl_alt_mm;
    int32_t speed_mmps;
    int32_t heading_1e5;
    uint16_t pdop_1e2;
    uint32_t h_acc_mm;
    uint32_t v_acc_mm;
    float heading;

    if (payload == NULL || payload_len < 80)
    {
        return;
    }

    fix_status = payload[20];
    fix_flags = payload[21];
    lat_lon_flags = payload[66];
    lon_1e7 = racebox_le_i32(payload + 24);
    lat_1e7 = racebox_le_i32(payload + 28);
    msl_alt_mm = racebox_le_i32(payload + 36);
    speed_mmps = racebox_le_i32(payload + 48);
    heading_1e5 = racebox_le_i32(payload + 52);
    pdop_1e2 = racebox_le_u16(payload + 64);
    h_acc_mm = racebox_le_u32(payload + 40);
    v_acc_mm = racebox_le_u32(payload + 44);
    heading = (float)heading_1e5 / 100000.0f;

    while (heading < 0.0f)
    {
        heading += 360.0f;
    }
    while (heading >= 360.0f)
    {
        heading -= 360.0f;
    }

    s_rb_decode.nav.has_data = true;
    s_rb_decode.nav.fix_status = fix_status;
    s_rb_decode.nav.satellites = payload[23];
    s_rb_decode.nav.longitude_deg = ((double)lon_1e7) / 10000000.0;
    s_rb_decode.nav.latitude_deg = ((double)lat_1e7) / 10000000.0;
    s_rb_decode.nav.altitude_m = ((float)msl_alt_mm) / 1000.0f;
    s_rb_decode.nav.speed_kph = ((float)speed_mmps) * 0.0036f;
    s_rb_decode.nav.heading_deg = heading;
    s_rb_decode.nav.pdop = ((float)pdop_1e2) / 100.0f;
    s_rb_decode.nav.horizontal_accuracy_m = ((float)h_acc_mm) / 1000.0f;
    s_rb_decode.nav.vertical_accuracy_m = ((float)v_acc_mm) / 1000.0f;
    s_rb_decode.nav.solution_valid = ((fix_flags & 0x01U) != 0U) &&
        (fix_status >= 2U) && ((lat_lon_flags & 0x01U) == 0U);
    s_rb_decode.nav.packets++;
}

static void racebox_decode_stream_bytes(const uint8_t* data, size_t len)
{
    size_t i;

    if (data == NULL || len == 0)
    {
        return;
    }

    if (!racebox_decode_ensure_mutex())
    {
        return;
    }

    if (xSemaphoreTake(s_rb_decode.mutex, pdMS_TO_TICKS(5)) != pdTRUE)
    {
        return;
    }

    for (i = 0; i < len; ++i)
    {
        if (s_rb_decode.frame_len >= sizeof(s_rb_decode.frame))
        {
            s_rb_decode.frame_len = 0;
        }
        s_rb_decode.frame[s_rb_decode.frame_len++] = data[i];

        while (s_rb_decode.frame_len >= 8)
        {
            size_t start = 0;
            uint16_t payload_len;
            size_t packet_len;
            uint8_t ck_a;
            uint8_t ck_b;

            while ((start + 1) < s_rb_decode.frame_len)
            {
                if (s_rb_decode.frame[start] == 0xB5 && s_rb_decode.frame[start + 1] == 0x62)
                {
                    break;
                }
                start++;
            }

            if (start > 0)
            {
                memmove(s_rb_decode.frame,
                    s_rb_decode.frame + start,
                    s_rb_decode.frame_len - start);
                s_rb_decode.frame_len -= start;
            }

            if (s_rb_decode.frame_len < 8)
            {
                break;
            }

            if (!(s_rb_decode.frame[0] == 0xB5 && s_rb_decode.frame[1] == 0x62))
            {
                memmove(s_rb_decode.frame, s_rb_decode.frame + 1, s_rb_decode.frame_len - 1);
                s_rb_decode.frame_len -= 1;
                continue;
            }

            payload_len = racebox_le_u16(s_rb_decode.frame + 4);
            if (payload_len > 504)
            {
                memmove(s_rb_decode.frame, s_rb_decode.frame + 1, s_rb_decode.frame_len - 1);
                s_rb_decode.frame_len -= 1;
                continue;
            }

            packet_len = (size_t)payload_len + 8;
            if (s_rb_decode.frame_len < packet_len)
            {
                break;
            }

            racebox_ubx_checksum(s_rb_decode.frame + 2, (size_t)payload_len + 4, &ck_a, &ck_b);
            if (ck_a == s_rb_decode.frame[6 + payload_len] &&
                ck_b == s_rb_decode.frame[7 + payload_len])
            {
                if (s_rb_decode.frame[2] == 0xFF && s_rb_decode.frame[3] == 0x01)
                {
                    racebox_decode_nav_payload_locked(s_rb_decode.frame + 6, payload_len);
                }

                memmove(s_rb_decode.frame,
                    s_rb_decode.frame + packet_len,
                    s_rb_decode.frame_len - packet_len);
                s_rb_decode.frame_len -= packet_len;
            }
            else
            {
                memmove(s_rb_decode.frame, s_rb_decode.frame + 1, s_rb_decode.frame_len - 1);
                s_rb_decode.frame_len -= 1;
            }
        }
    }

    xSemaphoreGive(s_rb_decode.mutex);
}

static bool racebox_get_nav_snapshot(racebox_nav_snapshot_t* out)
{
    if (out == NULL)
    {
        return false;
    }

    memset(out, 0, sizeof(*out));
    if (!racebox_decode_ensure_mutex())
    {
        return false;
    }

    if (xSemaphoreTake(s_rb_decode.mutex, pdMS_TO_TICKS(2)) != pdTRUE)
    {
        return false;
    }

    *out = s_rb_decode.nav;
    xSemaphoreGive(s_rb_decode.mutex);
    return out->has_data;
}

static void racebox_ui_rx_cb(const uint8_t* data, size_t len, void* user_ctx)
{
    LV_UNUSED(user_ctx);
    if (len > 0)
    {
        s_bt_rx_count++;
        racebox_decode_stream_bytes(data, len);
    }
    ui_stream_push_bytes(&s_bt_stream, data, len);
}
#endif

static void gnss_dump_append_line(const char* line)
{
    if (line == NULL || line[0] == '\0')
    {
        return;
    }
    ui_stream_push_bytes(&s_gnss_stream, (const uint8_t*)line, strlen(line));
    ui_stream_push_bytes(&s_gnss_stream, (const uint8_t*)"\n", 1);
}

static void gnss_ui_sentence_cb(const char* sentence, void* user_ctx)
{
    LV_UNUSED(user_ctx);

    if (sentence && sentence[0] != '\0')
    {
        s_gnss_rx_count++;
        gnss_dump_append_line(sentence);
    }
}

static void ui_setup_quadrant_panel(lv_obj_t* panel, lv_coord_t x, lv_coord_t y)
{
    if (panel == NULL)
    {
        return;
    }

    lv_obj_set_pos(panel, x, y);
    lv_obj_set_size(panel, MENU_PANEL_WIDTH, MENU_PANEL_HEIGHT);
    ui_style_surface(panel, lv_color_hex(UI_COLOR_SURFACE), 18);
    lv_obj_set_style_border_color(panel, lv_color_hex(UI_COLOR_BORDER), 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_pad_all(panel, 0, 0);
    lv_obj_set_scrollbar_mode(panel, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
}

static void ui_setup_dashboard_button(lv_obj_t* button,
    lv_obj_t* parent,
    lv_coord_t x,
    lv_coord_t y,
    lv_coord_t w,
    lv_coord_t h,
    const char* text,
    lv_color_t primary,
    lv_color_t secondary)
{
    lv_obj_t* label;

    if (button == NULL || parent == NULL)
    {
        return;
    }

    lv_obj_set_parent(button, parent);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, w, h);
    lv_obj_clear_flag(button, LV_OBJ_FLAG_SCROLLABLE);
    style_modern_button(button, primary, secondary);

    label = lv_obj_get_child(button, 0);
    if (label)
    {
        lv_label_set_text(label, text);
        lv_obj_center(label);
    }
}

static void ui_create_main_status_panels(void)
{
    lv_obj_t* title;
    lv_obj_t* subtitle;

    if (objects.menu == NULL)
    {
        return;
    }

    if (objects.obj2) lv_obj_add_flag(objects.obj2, LV_OBJ_FLAG_HIDDEN);
    if (objects.obj3) lv_obj_add_flag(objects.obj3, LV_OBJ_FLAG_HIDDEN);
    if (objects.sd_card_icon_2_2) lv_obj_add_flag(objects.sd_card_icon_2_2, LV_OBJ_FLAG_HIDDEN);
    if (objects.sd_card_icon_2_3) lv_obj_add_flag(objects.sd_card_icon_2_3, LV_OBJ_FLAG_HIDDEN);

    if (s_panel_sd1 == NULL)
    {
        s_panel_sd1 = lv_obj_create(objects.menu);
        ui_setup_quadrant_panel(s_panel_sd1, MENU_PANEL_LEFT_X, MENU_PANEL_TOP_Y);

        title = lv_label_create(s_panel_sd1);
        lv_obj_set_pos(title, 18, 16);
        lv_label_set_text(title, "SD Card 1");
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(title, lv_color_hex(UI_COLOR_TEXT), 0);

        subtitle = lv_label_create(s_panel_sd1);
        lv_obj_set_pos(subtitle, 18, 122);
        lv_obj_set_width(subtitle, MENU_PANEL_CONTENT_WIDTH);
        lv_label_set_long_mode(subtitle, LV_LABEL_LONG_WRAP);
        lv_label_set_text(subtitle, "Test mount/access or format the SD1 media.");
        lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(subtitle, lv_color_hex(UI_COLOR_MUTED), 0);
    }

    if (s_panel_sd2 == NULL)
    {
        s_panel_sd2 = lv_obj_create(objects.menu);
        ui_setup_quadrant_panel(s_panel_sd2, MENU_PANEL_RIGHT_X, MENU_PANEL_TOP_Y);

        title = lv_label_create(s_panel_sd2);
        lv_obj_set_pos(title, 18, 16);
        lv_label_set_text(title, "SD Card 2");
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(title, lv_color_hex(UI_COLOR_TEXT), 0);

        subtitle = lv_label_create(s_panel_sd2);
        lv_obj_set_pos(subtitle, 18, 122);
        lv_obj_set_width(subtitle, MENU_PANEL_CONTENT_WIDTH);
        lv_label_set_long_mode(subtitle, LV_LABEL_LONG_WRAP);
        lv_label_set_text(subtitle, "Test mount/access or format the SD2 media.");
        lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(subtitle, lv_color_hex(UI_COLOR_MUTED), 0);
    }

    if (objects.sdcard_storage_1_1)
    {
        lv_obj_set_parent(objects.sdcard_storage_1_1, s_panel_sd1);
        lv_obj_set_pos(objects.sdcard_storage_1_1, 18, 54);
        lv_obj_set_width(objects.sdcard_storage_1_1, 340);
        lv_obj_set_style_text_font(objects.sdcard_storage_1_1, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(objects.sdcard_storage_1_1, lv_color_hex(UI_COLOR_TEXT), 0);
    }
    if (objects.read_write_label_1_1)
    {
        lv_obj_set_parent(objects.read_write_label_1_1, s_panel_sd1);
        lv_obj_set_pos(objects.read_write_label_1_1, 18, 90);
        lv_obj_set_width(objects.read_write_label_1_1, 360);
        lv_obj_set_style_text_font(objects.read_write_label_1_1, &lv_font_montserrat_16, 0);
    }
    if (objects.active_checkbox_sd_card_1_1)
    {
        lv_obj_set_parent(objects.active_checkbox_sd_card_1_1, s_panel_sd1);
        lv_obj_set_pos(objects.active_checkbox_sd_card_1_1, 388, 18);
        lv_obj_set_style_text_font(objects.active_checkbox_sd_card_1_1, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_color(objects.active_checkbox_sd_card_1_1, lv_color_hex(UI_COLOR_TEXT), 0);
    }
    ui_setup_dashboard_button(objects.test_sd_card1_mount,
        s_panel_sd1,
        18,
        168,
        MENU_PANEL_HALF_BUTTON_WIDTH,
        52,
        "Test SD",
        lv_color_hex(0xff2e7d32),
        lv_color_hex(0xff1b5e20));
    ui_setup_dashboard_button(objects.format_sd_card1,
        s_panel_sd1,
        293,
        168,
        MENU_PANEL_HALF_BUTTON_WIDTH,
        52,
        "Fmt SD",
        lv_color_hex(0xffef6c00),
        lv_color_hex(0xffe65100));

    if (objects.sd_card_title_2_1)
    {
        lv_obj_set_parent(objects.sd_card_title_2_1, s_panel_sd2);
        lv_obj_set_pos(objects.sd_card_title_2_1, 18, 54);
        lv_obj_set_width(objects.sd_card_title_2_1, 340);
        lv_obj_set_style_text_font(objects.sd_card_title_2_1, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(objects.sd_card_title_2_1, lv_color_hex(UI_COLOR_TEXT), 0);
    }
    if (objects.read_write_label_2_1)
    {
        lv_obj_set_parent(objects.read_write_label_2_1, s_panel_sd2);
        lv_obj_set_pos(objects.read_write_label_2_1, 18, 90);
        lv_obj_set_width(objects.read_write_label_2_1, 360);
        lv_obj_set_style_text_font(objects.read_write_label_2_1, &lv_font_montserrat_16, 0);
    }
    if (objects.active_checkbox_sd_card_2_1)
    {
        lv_obj_set_parent(objects.active_checkbox_sd_card_2_1, s_panel_sd2);
        lv_obj_set_pos(objects.active_checkbox_sd_card_2_1, 388, 18);
        lv_obj_set_style_text_font(objects.active_checkbox_sd_card_2_1, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_color(objects.active_checkbox_sd_card_2_1, lv_color_hex(UI_COLOR_TEXT), 0);
    }
    ui_setup_dashboard_button(objects.test_sd_card1_mount_2,
        s_panel_sd2,
        18,
        168,
        MENU_PANEL_HALF_BUTTON_WIDTH,
        52,
        "Test SD",
        lv_color_hex(0xff2e7d32),
        lv_color_hex(0xff1b5e20));
    ui_setup_dashboard_button(objects.format_sd_card2,
        s_panel_sd2,
        293,
        168,
        MENU_PANEL_HALF_BUTTON_WIDTH,
        52,
        "Fmt SD",
        lv_color_hex(0xffef6c00),
        lv_color_hex(0xffe65100));

    if (s_panel_racebox == NULL)
    {
        s_panel_racebox = lv_obj_create(objects.menu);
        ui_setup_quadrant_panel(s_panel_racebox, MENU_PANEL_LEFT_X, MENU_PANEL_BOTTOM_Y);
        lv_obj_add_flag(s_panel_racebox, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_CLICK_FOCUSABLE);
        lv_obj_set_style_border_color(s_panel_racebox, lv_color_hex(UI_COLOR_ACCENT), LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(s_panel_racebox, 2, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(s_panel_racebox, 240, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_add_event_cb(s_panel_racebox, action_main_panel_racebox_click, LV_EVENT_CLICKED, NULL);

        s_panel_racebox_led = lv_led_create(s_panel_racebox);
        lv_obj_set_pos(s_panel_racebox_led, 18, 16);
        lv_obj_set_size(s_panel_racebox_led, 18, 18);
        lv_led_set_color(s_panel_racebox_led, lv_color_hex(UI_COLOR_MUTED));
        lv_led_off(s_panel_racebox_led);

        title = lv_label_create(s_panel_racebox);
        lv_obj_set_pos(title, 48, 16);
        lv_label_set_text(title, "RaceBox BLE");
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(title, lv_color_hex(UI_COLOR_TEXT), 0);

        s_panel_racebox_fill_bar = lv_bar_create(s_panel_racebox);
        lv_obj_set_pos(s_panel_racebox_fill_bar, 392, 18);
        lv_obj_set_size(s_panel_racebox_fill_bar, 118, 10);
        lv_bar_set_range(s_panel_racebox_fill_bar, 0, 100);
        lv_bar_set_value(s_panel_racebox_fill_bar, 0, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(s_panel_racebox_fill_bar, lv_color_hex(UI_COLOR_SURFACE_ALT), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(s_panel_racebox_fill_bar, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_radius(s_panel_racebox_fill_bar, 6, LV_PART_MAIN);
        lv_obj_set_style_border_width(s_panel_racebox_fill_bar, 0, LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_panel_racebox_fill_bar, lv_color_hex(UI_COLOR_ACCENT), LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(s_panel_racebox_fill_bar, LV_OPA_COVER, LV_PART_INDICATOR);
        lv_obj_set_style_radius(s_panel_racebox_fill_bar, 6, LV_PART_INDICATOR);

        s_panel_racebox_fill_label = lv_label_create(s_panel_racebox);
        lv_obj_set_pos(s_panel_racebox_fill_label, 518, 12);
        lv_label_set_text(s_panel_racebox_fill_label, "0%");
        lv_obj_set_style_text_font(s_panel_racebox_fill_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_panel_racebox_fill_label, lv_color_hex(UI_COLOR_MUTED), 0);

        s_panel_racebox_state = lv_label_create(s_panel_racebox);
        lv_obj_set_pos(s_panel_racebox_state, 18, 56);
        lv_label_set_text(s_panel_racebox_state, "Status: Disconnected");
        lv_obj_set_style_text_font(s_panel_racebox_state, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(s_panel_racebox_state, lv_color_hex(UI_COLOR_DANGER), 0);

        s_panel_racebox_count = lv_label_create(s_panel_racebox);
        lv_obj_set_pos(s_panel_racebox_count, 18, 86);
        lv_label_set_text(s_panel_racebox_count, "Packets: 0");
        lv_obj_set_style_text_font(s_panel_racebox_count, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(s_panel_racebox_count, lv_color_hex(UI_COLOR_MUTED), 0);

        s_panel_racebox_points = lv_label_create(s_panel_racebox);
        lv_obj_set_pos(s_panel_racebox_points, MENU_PANEL_COUNT_RIGHT_X, 86);
        lv_obj_set_width(s_panel_racebox_points, 240);
        lv_label_set_text(s_panel_racebox_points, "Points: 0");
        lv_obj_set_style_text_font(s_panel_racebox_points, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(s_panel_racebox_points, lv_color_hex(UI_COLOR_MUTED), 0);

        s_panel_racebox_detail = lv_label_create(s_panel_racebox);
        lv_obj_set_pos(s_panel_racebox_detail, 18, 114);
        lv_obj_set_width(s_panel_racebox_detail, MENU_PANEL_DETAIL_WIDTH);
        lv_label_set_long_mode(s_panel_racebox_detail, LV_LABEL_LONG_WRAP);
        lv_label_set_text(s_panel_racebox_detail, "SRC:RB | Device: -- | RSSI: -- dBm");
        lv_obj_set_style_text_font(s_panel_racebox_detail, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_panel_racebox_detail, lv_color_hex(UI_COLOR_MUTED), 0);
    }

    if (s_panel_gnss == NULL)
    {
        s_panel_gnss = lv_obj_create(objects.menu);
        ui_setup_quadrant_panel(s_panel_gnss, MENU_PANEL_RIGHT_X, MENU_PANEL_BOTTOM_Y);
        lv_obj_add_flag(s_panel_gnss, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_CLICK_FOCUSABLE);
        lv_obj_set_style_border_color(s_panel_gnss, lv_color_hex(UI_COLOR_ACCENT), LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(s_panel_gnss, 2, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(s_panel_gnss, 240, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_add_event_cb(s_panel_gnss, action_main_panel_gnss_click, LV_EVENT_CLICKED, NULL);

        s_panel_gnss_led = lv_led_create(s_panel_gnss);
        lv_obj_set_pos(s_panel_gnss_led, 18, 16);
        lv_obj_set_size(s_panel_gnss_led, 18, 18);
        lv_led_set_color(s_panel_gnss_led, lv_color_hex(UI_COLOR_MUTED));
        lv_led_off(s_panel_gnss_led);

        title = lv_label_create(s_panel_gnss);
        lv_obj_set_pos(title, 48, 16);
        lv_label_set_text(title, "GNSS");
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(title, lv_color_hex(UI_COLOR_TEXT), 0);

        s_panel_gnss_fill_bar = lv_bar_create(s_panel_gnss);
        lv_obj_set_pos(s_panel_gnss_fill_bar, 392, 18);
        lv_obj_set_size(s_panel_gnss_fill_bar, 118, 10);
        lv_bar_set_range(s_panel_gnss_fill_bar, 0, 100);
        lv_bar_set_value(s_panel_gnss_fill_bar, 0, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(s_panel_gnss_fill_bar, lv_color_hex(UI_COLOR_SURFACE_ALT), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(s_panel_gnss_fill_bar, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_radius(s_panel_gnss_fill_bar, 6, LV_PART_MAIN);
        lv_obj_set_style_border_width(s_panel_gnss_fill_bar, 0, LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_panel_gnss_fill_bar, lv_color_hex(UI_COLOR_ACCENT), LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(s_panel_gnss_fill_bar, LV_OPA_COVER, LV_PART_INDICATOR);
        lv_obj_set_style_radius(s_panel_gnss_fill_bar, 6, LV_PART_INDICATOR);

        s_panel_gnss_fill_label = lv_label_create(s_panel_gnss);
        lv_obj_set_pos(s_panel_gnss_fill_label, 518, 12);
        lv_label_set_text(s_panel_gnss_fill_label, "0%");
        lv_obj_set_style_text_font(s_panel_gnss_fill_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_panel_gnss_fill_label, lv_color_hex(UI_COLOR_MUTED), 0);

        s_panel_gnss_state = lv_label_create(s_panel_gnss);
        lv_obj_set_pos(s_panel_gnss_state, 18, 56);
        lv_label_set_text(s_panel_gnss_state, "Status: Disconnected");
        lv_obj_set_style_text_font(s_panel_gnss_state, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(s_panel_gnss_state, lv_color_hex(UI_COLOR_DANGER), 0);

        s_panel_gnss_count = lv_label_create(s_panel_gnss);
        lv_obj_set_pos(s_panel_gnss_count, 18, 86);
        lv_label_set_text(s_panel_gnss_count, "Sentences: 0");
        lv_obj_set_style_text_font(s_panel_gnss_count, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(s_panel_gnss_count, lv_color_hex(UI_COLOR_MUTED), 0);

        s_panel_gnss_points = lv_label_create(s_panel_gnss);
        lv_obj_set_pos(s_panel_gnss_points, MENU_PANEL_COUNT_RIGHT_X, 86);
        lv_obj_set_width(s_panel_gnss_points, 240);
        lv_label_set_text(s_panel_gnss_points, "Points: 0");
        lv_obj_set_style_text_font(s_panel_gnss_points, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(s_panel_gnss_points, lv_color_hex(UI_COLOR_MUTED), 0);

        s_panel_gnss_detail = lv_label_create(s_panel_gnss);
        lv_obj_set_pos(s_panel_gnss_detail, 18, 114);
        lv_obj_set_width(s_panel_gnss_detail, MENU_PANEL_DETAIL_WIDTH);
        lv_label_set_long_mode(s_panel_gnss_detail, LV_LABEL_LONG_WRAP);
        lv_label_set_text(s_panel_gnss_detail, "SRC:UART | Fix: -- | Sats: -- | Speed: --");
        lv_obj_set_style_text_font(s_panel_gnss_detail, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_panel_gnss_detail, lv_color_hex(UI_COLOR_MUTED), 0);
    }
}

static void action_main_panel_racebox_click(lv_event_t* e)
{
    LV_UNUSED(e);
    ui_begin_racebox_auto_connect();
}

static void action_main_panel_gnss_click(lv_event_t* e)
{
    LV_UNUSED(e);

    if (s_gnss_listening)
    {
        return;
    }

    action_gnss_start_stop(NULL);
}

static void ui_focus_bluetooth_connect_button(void)
{
    if (objects.bluetooth_connect_button == NULL)
    {
        return;
    }

    if (objects.bluetooth_refresh_button)
    {
        lv_obj_clear_state(objects.bluetooth_refresh_button, LV_STATE_FOCUSED);
    }

    lv_obj_add_state(objects.bluetooth_connect_button, LV_STATE_FOCUSED);
    lv_obj_scroll_to_view(objects.bluetooth_connect_button, LV_ANIM_ON);
}

static void ui_show_dashboard_tab(bool animate)
{
    if (objects.obj0)
    {
        lv_tabview_set_active(objects.obj0, 0, animate ? LV_ANIM_ON : LV_ANIM_OFF);
    }
}

static void action_bluetooth_close(lv_event_t* e)
{
    LV_UNUSED(e);
    ui_show_dashboard_tab(true);
}

static void ui_cancel_racebox_auto_connect(void)
{
#if CONFIG_RALLYBOX_RACEBOX_ENABLED
    s_bt_auto_connect_pending = false;
    s_bt_auto_connect_started_ms = 0;
#endif
}

static bool ui_text_contains_ignore_case(const char* text, const char* needle)
{
    size_t needle_len;
    const char* pos;

    if (text == NULL || needle == NULL || needle[0] == '\0')
    {
        return false;
    }

    needle_len = strlen(needle);
    for (pos = text; *pos != '\0'; ++pos)
    {
        if (strncasecmp(pos, needle, needle_len) == 0)
        {
            return true;
        }
    }

    return false;
}

static bool ui_racebox_status_has_failure(const char* text)
{
    return ui_text_contains_ignore_case(text, "fail") ||
        ui_text_contains_ignore_case(text, "error") ||
        ui_text_contains_ignore_case(text, "timeout") ||
        ui_text_contains_ignore_case(text, "unavailable");
}

static void ui_fallback_to_bluetooth_panel(void)
{
    system_status_t status;

    ui_cancel_racebox_auto_connect();
    ui_open_bluetooth_panel(false);
    status = system_monitor_get_status();
    ui_refresh_bluetooth_controls(&status);
}

static void ui_begin_racebox_auto_connect(void)
{
#if CONFIG_RALLYBOX_RACEBOX_ENABLED
    system_status_t status = system_monitor_get_status();
    esp_err_t ret;

    if (status.racebox_connected)
    {
        ui_cancel_racebox_auto_connect();
        return;
    }

    s_bt_auto_connect_pending = true;
    s_bt_auto_connect_started_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    if (!status.racebox_initialized)
    {
        ret = racebox_init();
        if (ret != ESP_OK)
        {
            ui_fallback_to_bluetooth_panel();
        }
        return;
}

    ret = racebox_request_scan();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ui_fallback_to_bluetooth_panel();
    }
#else
    ui_open_bluetooth_panel(false);
#endif
}

static void ui_update_racebox_auto_connect(const system_status_t* status)
{
#if CONFIG_RALLYBOX_RACEBOX_ENABLED
    uint32_t now_ms;

    if (!s_bt_auto_connect_pending || status == NULL)
    {
        return;
    }

    if (status->racebox_connected)
    {
        ui_cancel_racebox_auto_connect();
        return;
    }

    if (ui_racebox_status_has_failure(status->racebox_status_text))
    {
        ui_fallback_to_bluetooth_panel();
        return;
    }

    now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if ((now_ms - s_bt_auto_connect_started_ms) >= RACEBOX_AUTO_CONNECT_TIMEOUT_MS)
    {
        ESP_LOGW(TAG, "RaceBox auto-connect timed out after %u ms", (unsigned)RACEBOX_AUTO_CONNECT_TIMEOUT_MS);
        ui_fallback_to_bluetooth_panel();
    }
#else
    LV_UNUSED(status);
#endif
}

static void ui_open_bluetooth_panel(bool request_scan)
{
    if (objects.obj0)
    {
        lv_tabview_set_active(objects.obj0, BLUETOOTH_TAB_INDEX, LV_ANIM_ON);
    }

    ui_prepare_bluetooth_panel(request_scan, false);
}

static void ui_prepare_bluetooth_panel(bool request_scan, bool show_messages)
{
    system_status_t status;

    ui_refresh_bluetooth_dropdown(true);
    ui_focus_bluetooth_connect_button();

    if (!request_scan)
    {
        return;
    }

#if CONFIG_RALLYBOX_RACEBOX_ENABLED
    status = system_monitor_get_status();
    if (status.racebox_connected)
    {
        if (show_messages)
        {
            ui_show_message("Bluetooth", "RaceBox is already connected. Refresh is only used for discovery when disconnected.", lv_color_hex(UI_COLOR_SURFACE));
        }
        ui_refresh_bluetooth_dropdown(true);
        ui_focus_bluetooth_connect_button();
        return;
    }

    if (!status.racebox_initialized)
    {
        esp_err_t ret = racebox_init();
        if (ret != ESP_OK)
        {
            ui_handle_error("Failed to initialize RaceBox BLE", ret);
            return;
        }

        ui_refresh_bluetooth_dropdown(true);
        ui_focus_bluetooth_connect_button();
        if (show_messages)
        {
            ui_show_message("Bluetooth", "Initializing BLE stack... tap Refresh again in a moment.", lv_color_hex(UI_COLOR_SURFACE));
        }
        return;
    }

    {
        esp_err_t ret = racebox_request_scan();
        if (ret != ESP_OK)
        {
            ui_handle_error("Failed to refresh BLE scan", ret);
            return;
        }
}
#endif

    ui_refresh_bluetooth_dropdown(true);
    ui_focus_bluetooth_connect_button();
    if (show_messages)
    {
        ui_show_message("Bluetooth", "Refreshing scan... devices will appear shortly.", lv_color_hex(UI_COLOR_SURFACE));
    }
}

static void ui_update_main_status_panels(const system_status_t* status)
{
    char state_line[160];
    char count_line[96];
    char points_line[48];
    char detail[192];
    char gnss_state_line[160];
    char gnss_count_line[96];
    char gnss_points_line[48];
    char gnss_detail[192];
    char ble_fill_text[24];
    char gnss_fill_text[24];
    uint32_t bt_count;
    uint32_t gnss_count;
    size_t ble_track_points = 0;
    size_t ble_track_capacity = 0;
    size_t gnss_track_points = 0;
    size_t gnss_track_capacity = 0;
    uint8_t ble_fill_percent = 0;
    uint8_t gnss_fill_percent = 0;
    uint64_t now_us;
    bool bt_count_changed;
    bool gnss_count_changed;
    bool bt_receiving;
    bool gnss_receiving;
    bool moving;
    float speed_kph;
    float heading_deg;
    lv_color_t state_color;
    lv_color_t gnss_state_color;
#if CONFIG_RALLYBOX_RACEBOX_ENABLED
    racebox_nav_snapshot_t rb_nav = { 0 };
    bool rb_has_data = racebox_get_nav_snapshot(&rb_nav);
#endif

    if (status == NULL)
    {
        return;
    }

    ui_update_storage_dependent_controls(status);

    if (s_panel_racebox == NULL || s_panel_gnss == NULL)
    {
        ui_create_main_status_panels();
    }

    if (s_panel_racebox_state == NULL || s_panel_racebox_count == NULL || s_panel_racebox_points == NULL ||
        s_panel_gnss_state == NULL || s_panel_gnss_count == NULL || s_panel_gnss_points == NULL ||
        s_panel_racebox_detail == NULL || s_panel_gnss_detail == NULL)
    {
        return;
    }

    bt_count = s_bt_rx_count;
    gnss_count = s_gnss_rx_count;
    now_us = (uint64_t)esp_timer_get_time();
    bt_count_changed = (bt_count != s_last_bt_count);
    gnss_count_changed = (gnss_count != s_last_gnss_count);
    if (bt_count_changed)
    {
        s_bt_last_rx_change_us = now_us;
    }
    if (gnss_count_changed)
    {
        s_gnss_last_rx_change_us = now_us;
    }
    (void)gps_points_get_usage(GPS_POINTS_FEED_BLE, &ble_track_points, &ble_track_capacity, &ble_fill_percent);
    (void)gps_points_get_usage(GPS_POINTS_FEED_GNSS, &gnss_track_points, &gnss_track_capacity, &gnss_fill_percent);
    bt_receiving = status->racebox_connected &&
        (s_bt_last_rx_change_us != 0ULL) &&
        ((now_us - s_bt_last_rx_change_us) <= UI_LED_ACTIVITY_WINDOW_US);
    gnss_receiving = s_gnss_listening &&
        (s_gnss_last_rx_change_us != 0ULL) &&
        ((now_us - s_gnss_last_rx_change_us) <= UI_LED_ACTIVITY_WINDOW_US);

    if (status->racebox_connected)
    {
        snprintf(state_line, sizeof(state_line), "Status: %s",
            bt_receiving ? "Connected / Receiving" : "Connected / Idle");
        state_color = bt_receiving ? lv_color_hex(UI_COLOR_SUCCESS) : lv_color_hex(UI_COLOR_WARNING);
    }
    else
    {
        snprintf(state_line, sizeof(state_line), "Status: Disconnected");
        state_color = lv_color_hex(UI_COLOR_DANGER);
    }

    snprintf(count_line,
        sizeof(count_line),
        "Packets: %lu",
        (unsigned long)bt_count);
    snprintf(points_line, sizeof(points_line), "Points: %u", (unsigned)ble_track_points);
    snprintf(ble_fill_text,
        sizeof(ble_fill_text),
        "%u%%",
        (unsigned)ble_fill_percent);

#if CONFIG_RALLYBOX_RACEBOX_ENABLED
    if (status->racebox_connected && rb_has_data)
    {
        const char* fix_text = "none";
        speed_kph = rb_nav.speed_kph;
        heading_deg = rb_nav.heading_deg;
        if (speed_kph < 0.0f)
        {
            speed_kph = -speed_kph;
        }

        if (rb_nav.fix_status == 3)
        {
            fix_text = "3D";
        }
        else if (rb_nav.fix_status == 2)
        {
            fix_text = "2D";
        }

        if (!s_rb_smooth_valid)
        {
            s_rb_speed_smooth = speed_kph;
            s_rb_heading_smooth = heading_deg;
            s_rb_smooth_valid = true;
        }
        else
        {
            s_rb_speed_smooth = ui_smooth_scalar(s_rb_speed_smooth, speed_kph, 0.25f);
            s_rb_heading_smooth = ui_smooth_heading(s_rb_heading_smooth, heading_deg, 0.25f);
        }

        snprintf(detail, sizeof(detail),
            "SRC:RB | Fix:%s | S:%u | %.1f km/h | %.0f deg | %s\nPDOP:%.2f | hAcc:%.1fm | vAcc:%.1fm",
            rb_nav.solution_valid ? fix_text : "none",
            (unsigned)rb_nav.satellites,
            (double)s_rb_speed_smooth,
            (double)s_rb_heading_smooth,
            s_rb_speed_smooth > 2.0f ? "Moving" : "Still",
            (double)rb_nav.pdop,
            (double)rb_nav.horizontal_accuracy_m,
            (double)rb_nav.vertical_accuracy_m);
}
    else
    {
        s_rb_smooth_valid = false;
        snprintf(detail, sizeof(detail), "SRC:RB | Device: %s | RSSI: %d dBm",
            status->racebox_device_name[0] ? status->racebox_device_name : "--",
            (int)status->racebox_rssi);
    }
#else
    snprintf(detail, sizeof(detail), "SRC:RB | Device: %s | RSSI: %d dBm",
        status->racebox_device_name[0] ? status->racebox_device_name : "--",
        (int)status->racebox_rssi);
#endif

    lv_label_set_text(s_panel_racebox_state, state_line);
    lv_obj_set_style_text_color(s_panel_racebox_state, state_color, 0);
    lv_label_set_text(s_panel_racebox_count, count_line);
    lv_label_set_text(s_panel_racebox_points, points_line);
    lv_label_set_text(s_panel_racebox_detail, detail);
    ui_update_fix_led(s_panel_racebox_led,
        bt_receiving,
        ui_get_racebox_fix_led_color(
#if CONFIG_RALLYBOX_RACEBOX_ENABLED
            status->racebox_connected && rb_has_data && rb_nav.solution_valid,
            rb_nav.fix_status
#else
            false,
            0
#endif
        ),
        now_us
    );
    if (s_panel_racebox_fill_bar)
    {
        lv_obj_set_style_bg_color(s_panel_racebox_fill_bar,
            ui_get_fill_bar_color(ble_fill_percent),
            LV_PART_INDICATOR);
        lv_bar_set_value(s_panel_racebox_fill_bar, ble_fill_percent, LV_ANIM_OFF);
    }
    if (s_panel_racebox_fill_label)
    {
        lv_obj_set_style_text_color(s_panel_racebox_fill_label,
            ui_get_fill_bar_color(ble_fill_percent),
            0);
        lv_label_set_text(s_panel_racebox_fill_label, ble_fill_text);
    }

    if (s_gnss_listening)
    {
        snprintf(gnss_state_line, sizeof(gnss_state_line), "Status: %s",
            gnss_receiving ? "Connected / Receiving" : "Connected / Idle");
        gnss_state_color = gnss_receiving ? lv_color_hex(UI_COLOR_SUCCESS) : lv_color_hex(UI_COLOR_WARNING);
    }
    else
    {
        snprintf(gnss_state_line, sizeof(gnss_state_line), "Status: Disconnected");
        gnss_state_color = lv_color_hex(UI_COLOR_DANGER);
    }

    snprintf(gnss_count_line,
        sizeof(gnss_count_line),
        "Sentences: %lu",
        (unsigned long)gnss_count);
    snprintf(gnss_points_line, sizeof(gnss_points_line), "Points: %u", (unsigned)gnss_track_points);
    snprintf(gnss_fill_text,
        sizeof(gnss_fill_text),
        "%u%%",
        (unsigned)gnss_fill_percent);

    speed_kph = status->gnss_speed_kph;
    if (speed_kph < 0.0f)
    {
        speed_kph = -speed_kph;
    }
    heading_deg = status->gnss_heading_deg;
    while (heading_deg < 0.0f)
    {
        heading_deg += 360.0f;
    }
    while (heading_deg >= 360.0f)
    {
        heading_deg -= 360.0f;
    }

    if (status->gnss_fix_valid)
    {
        if (!s_gnss_smooth_valid)
        {
            s_gnss_speed_smooth = speed_kph;
            s_gnss_heading_smooth = heading_deg;
            s_gnss_smooth_valid = true;
        }
        else
        {
            s_gnss_speed_smooth = ui_smooth_scalar(s_gnss_speed_smooth, speed_kph, 0.25f);
            s_gnss_heading_smooth = ui_smooth_heading(s_gnss_heading_smooth, heading_deg, 0.25f);
        }
    }
    else
    {
        s_gnss_smooth_valid = false;
    }

    moving = s_gnss_speed_smooth > 2.0f;
    if (status->gnss_fix_valid)
    {
        snprintf(gnss_detail, sizeof(gnss_detail), "SRC:UART | Fix:%u | S:%u | %.1f km/h\nHead:%.0f deg | %s",
            (unsigned)status->gnss_fix_quality,
            (unsigned)status->gnss_satellites,
            (double)s_gnss_speed_smooth,
            (double)s_gnss_heading_smooth,
            moving ? "Moving" : "Still");
    }
    else
    {
        snprintf(gnss_detail, sizeof(gnss_detail), "SRC:UART | Fix:none | S:%u\nWaiting for lock",
            (unsigned)status->gnss_satellites);
    }

    lv_label_set_text(s_panel_gnss_state, gnss_state_line);
    lv_obj_set_style_text_color(s_panel_gnss_state, gnss_state_color, 0);
    lv_label_set_text(s_panel_gnss_count, gnss_count_line);
    lv_label_set_text(s_panel_gnss_points, gnss_points_line);
    lv_label_set_text(s_panel_gnss_detail, gnss_detail);
    ui_update_fix_led(s_panel_gnss_led,
        gnss_receiving,
        ui_get_gnss_fix_led_color(status->gnss_fix_quality),
        now_us);
    if (s_panel_gnss_fill_bar)
    {
        lv_obj_set_style_bg_color(s_panel_gnss_fill_bar,
            ui_get_fill_bar_color(gnss_fill_percent),
            LV_PART_INDICATOR);
        lv_bar_set_value(s_panel_gnss_fill_bar, gnss_fill_percent, LV_ANIM_OFF);
    }
    if (s_panel_gnss_fill_label)
    {
        lv_obj_set_style_text_color(s_panel_gnss_fill_label,
            ui_get_fill_bar_color(gnss_fill_percent),
            0);
        lv_label_set_text(s_panel_gnss_fill_label, gnss_fill_text);
    }

    s_last_bt_count = bt_count;
    s_last_gnss_count = gnss_count;
}

static esp_err_t gnss_monitor_start(int baud_rate, int tx_gpio, int rx_gpio)
{
#if CONFIG_RALLYBOX_GNSS_ENABLED
    if (s_gnss_listening)
    {
        return ESP_ERR_INVALID_STATE;
    }

    gnss_set_sentence_callback(gnss_ui_sentence_cb, NULL);
    return gnss_start_with_config(baud_rate, tx_gpio, rx_gpio, true);
#else
    LV_UNUSED(baud_rate);
    LV_UNUSED(tx_gpio);
    LV_UNUSED(rx_gpio);
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static void gnss_monitor_stop(void)
{
#if CONFIG_RALLYBOX_GNSS_ENABLED
    gnss_set_sentence_callback(NULL, NULL);
    gnss_stop();
#endif
}

static bool ui_stream_init(ui_stream_buffer_t* stream)
{
    void* mem;

    if (stream == NULL)
    {
        return false;
    }

    if (stream->mutex == NULL)
    {
        stream->mutex = xSemaphoreCreateMutex();
    }

    if (stream->data == NULL || stream->capacity == 0)
    {
        mem = heap_caps_malloc(UART_STREAM_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (mem == NULL)
        {
            mem = heap_caps_malloc(UART_STREAM_BUFFER_SIZE, MALLOC_CAP_8BIT);
            if (mem)
            {
                ESP_LOGW(TAG, "Stream buffer allocated in internal RAM (%u bytes)", (unsigned)UART_STREAM_BUFFER_SIZE);
            }
        }
        if (mem == NULL)
        {
            ESP_LOGE(TAG, "Failed to allocate stream buffer (%u bytes)", (unsigned)UART_STREAM_BUFFER_SIZE);
            return false;
        }

        stream->data = (char*)mem;
        stream->capacity = UART_STREAM_BUFFER_SIZE;
        stream->head = 0;
        stream->tail = 0;
        stream->partial_len = 0;
        stream->overflowed = false;
        stream->active = false;
    }

    return stream->mutex != NULL && stream->data != NULL && stream->capacity > 1;
}

static void ui_stream_start(ui_stream_buffer_t* stream)
{
    if (!ui_stream_init(stream))
    {
        return;
    }

    if (xSemaphoreTake(stream->mutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        stream->head = 0;
        stream->tail = 0;
        stream->partial_len = 0;
        stream->overflowed = false;
        stream->active = true;
        xSemaphoreGive(stream->mutex);
    }
}

static void ui_stream_stop(ui_stream_buffer_t* stream)
{
    if (!ui_stream_init(stream))
    {
        return;
    }

    if (xSemaphoreTake(stream->mutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        stream->active = false;
        stream->head = 0;
        stream->tail = 0;
        stream->partial_len = 0;
        stream->overflowed = false;
        xSemaphoreGive(stream->mutex);
    }
}

static void ui_stream_push_bytes(ui_stream_buffer_t* stream, const uint8_t* data, size_t len)
{
    size_t i;
    size_t next_head;

    if (stream == NULL || data == NULL || len == 0)
    {
        return;
    }

    if (!ui_stream_init(stream))
    {
        return;
    }

    if (xSemaphoreTake(stream->mutex, pdMS_TO_TICKS(5)) != pdTRUE)
    {
        return;
    }

    if (!stream->active)
    {
        xSemaphoreGive(stream->mutex);
        return;
    }

    for (i = 0; i < len; ++i)
    {
        next_head = (stream->head + 1) % stream->capacity;
        if (next_head == stream->tail)
        {
            stream->tail = 0;
            stream->head = 0;
            stream->partial_len = 0;
            stream->overflowed = true;
            next_head = 1;
        }

        stream->data[stream->head] = (char)data[i];
        stream->head = next_head;
    }

    xSemaphoreGive(stream->mutex);
}

static size_t ui_stream_pop_chunk(ui_stream_buffer_t* stream, char* out, size_t max_len, bool* had_overflow)
{
    size_t copied = 0;

    if (stream == NULL || out == NULL || max_len == 0)
    {
        return 0;
    }

    if (had_overflow)
    {
        *had_overflow = false;
    }

    if (!ui_stream_init(stream))
    {
        return 0;
    }

    if (xSemaphoreTake(stream->mutex, pdMS_TO_TICKS(5)) != pdTRUE)
    {
        return 0;
    }

    if (had_overflow && stream->overflowed)
    {
        *had_overflow = true;
        stream->overflowed = false;
    }

    while (stream->tail != stream->head && copied < max_len)
    {
        out[copied++] = stream->data[stream->tail];
        stream->tail = (stream->tail + 1) % stream->capacity;
    }

    xSemaphoreGive(stream->mutex);
    return copied;
}

static void ui_window_reset(ui_line_window_t* window, lv_obj_t* panel)
{
    if (window == NULL)
    {
        return;
    }

    window->start = 0;
    window->count = 0;
    window->dirty = true;

    if (panel)
    {
        lv_textarea_set_text(panel, "");
    }
}

static void ui_window_append_line(ui_line_window_t* window, const char* line)
{
    uint8_t slot;

    if (window == NULL || line == NULL || line[0] == '\0')
    {
        return;
    }

    if (window->count < UART_MONITOR_LINES)
    {
        slot = (uint8_t)((window->start + window->count) % UART_MONITOR_LINES);
        window->count++;
    }
    else
    {
        slot = window->start;
        window->start = (uint8_t)((window->start + 1) % UART_MONITOR_LINES);
    }

    snprintf(window->lines[slot], sizeof(window->lines[slot]), "%s", line);
    window->dirty = true;
}

static void ui_window_consume_chunk(ui_stream_buffer_t* stream, ui_line_window_t* window, const char* chunk, size_t len)
{
    size_t i;

    if (stream == NULL || window == NULL || chunk == NULL || len == 0)
    {
        return;
    }

    for (i = 0; i < len; ++i)
    {
        char ch = chunk[i];

        if (ch == '\r')
        {
            continue;
        }

        if (ch == '\n')
        {
            if (stream->partial_len > 0)
            {
                stream->partial_line[stream->partial_len] = '\0';
                ui_window_append_line(window, stream->partial_line);
                stream->partial_len = 0;
            }
            continue;
        }

        if (!(isprint((unsigned char)ch) || ch == '\t' || ch == ' '))
        {
            continue;
        }

        if (stream->partial_len < (sizeof(stream->partial_line) - 1))
        {
            stream->partial_line[stream->partial_len++] = ch;
        }
    }
}

static void ui_window_render_if_dirty(ui_line_window_t* window, lv_obj_t* panel)
{
    uint8_t i;
    size_t offset = 0;

    if (window == NULL || panel == NULL || !window->dirty)
    {
        return;
    }

    if (panel == objects.gnss_dump_panel)
    {
        s_gnss_render_buf[0] = '\0';
        for (i = 0; i < window->count; ++i)
        {
            uint8_t idx = (uint8_t)((window->start + i) % UART_MONITOR_LINES);
            int wrote = snprintf(s_gnss_render_buf + offset,
                sizeof(s_gnss_render_buf) - offset,
                "%s%s",
                window->lines[idx],
                (i + 1 < window->count) ? "\n" : "");
            if (wrote <= 0 || (size_t)wrote >= (sizeof(s_gnss_render_buf) - offset))
            {
                break;
            }
            offset += (size_t)wrote;
        }
        lv_textarea_set_text(panel, s_gnss_render_buf);
    }
#if CONFIG_RALLYBOX_RACEBOX_ENABLED
    else if (panel == s_bt_dump_panel)
    {
        s_bt_render_buf[0] = '\0';
        for (i = 0; i < window->count; ++i)
        {
            uint8_t idx = (uint8_t)((window->start + i) % UART_MONITOR_LINES);
            int wrote = snprintf(s_bt_render_buf + offset,
                sizeof(s_bt_render_buf) - offset,
                "%s%s",
                window->lines[idx],
                (i + 1 < window->count) ? "\n" : "");
            if (wrote <= 0 || (size_t)wrote >= (sizeof(s_bt_render_buf) - offset))
            {
                break;
            }
            offset += (size_t)wrote;
        }
        lv_textarea_set_text(panel, s_bt_render_buf);
    }
#endif

    lv_textarea_set_cursor_pos(panel, LV_TEXTAREA_CURSOR_LAST);
    window->dirty = false;
}

static void ui_refresh_gnss_dump(void)
{
    if (!ui_is_gnss_tab_active())
    {
        return;
    }

    char chunk[UART_STREAM_CHUNK_MAX];
    bool had_overflow = false;
    size_t n;

    n = ui_stream_pop_chunk(&s_gnss_stream, chunk, sizeof(chunk), &had_overflow);
    if (had_overflow)
    {
        ui_window_append_line(&s_gnss_window, "[buffer rollover]");
    }
    if (n > 0)
    {
        ui_window_consume_chunk(&s_gnss_stream, &s_gnss_window, chunk, n);
    }

    if (objects.gnss_dump_panel)
    {
        ui_window_render_if_dirty(&s_gnss_window, objects.gnss_dump_panel);
    }
}

static void ui_refresh_bluetooth_dump(void)
{
#if CONFIG_RALLYBOX_RACEBOX_ENABLED
    if (!ui_is_bluetooth_tab_active())
    {
        return;
    }

    char chunk[UART_STREAM_CHUNK_MAX];
    bool had_overflow = false;
    size_t n;

    n = ui_stream_pop_chunk(&s_bt_stream, chunk, sizeof(chunk), &had_overflow);
    if (had_overflow)
    {
        ui_window_append_line(&s_bt_window, "[buffer rollover]");
    }
    if (n > 0)
    {
        ui_window_consume_chunk(&s_bt_stream, &s_bt_window, chunk, n);
    }

    if (s_bt_dump_panel)
    {
        ui_window_render_if_dirty(&s_bt_window, s_bt_dump_panel);
    }
#endif
}

static void action_gnss_start_stop(lv_event_t* e)
{
    int baud_rate;
    int tx_gpio;
    int rx_gpio;
    lv_obj_t* label;
    char line[96];

    LV_UNUSED(e);

    if (s_gnss_listening)
    {
        gps_points_set_feed_active(GPS_POINTS_FEED_GNSS, false);

        if (objects.gnss_start_stop_button)
        {
            label = ui_get_first_child(objects.gnss_start_stop_button);
            if (label) lv_label_set_text(label, "START");
        }

        gnss_monitor_stop();
        s_gnss_listening = false;
        gnss_dump_append_line("Stopped by user");
        ui_refresh_gnss_dump();
        ui_stream_stop(&s_gnss_stream);
        snprintf(s_gnss_status_text, sizeof(s_gnss_status_text), "Stopped. Press START to listen again");
        if (objects.gnss_status_label) lv_label_set_text(objects.gnss_status_label, s_gnss_status_text);
        return;
    }

    if (objects.gnss_baud_input == NULL || objects.gnss_tx_input == NULL || objects.gnss_rx_input == NULL)
    {
        return;
    }

    baud_rate = atoi(lv_textarea_get_text(objects.gnss_baud_input));
    tx_gpio = atoi(lv_textarea_get_text(objects.gnss_tx_input));
    rx_gpio = atoi(lv_textarea_get_text(objects.gnss_rx_input));

    if (baud_rate < 1200 || baud_rate > 921600)
    {
        ui_show_message("GNSS", "Invalid baud rate. Use 1200..921600", lv_color_hex(UI_COLOR_DANGER));
        return;
    }

    if (tx_gpio < 0 || tx_gpio > 54 || rx_gpio < 0 || rx_gpio > 54 || tx_gpio == rx_gpio)
    {
        ui_show_message("GNSS", "Invalid TX/RX GPIO values.", lv_color_hex(UI_COLOR_DANGER));
        return;
    }

    if (objects.gnss_start_stop_button)
    {
        label = ui_get_first_child(objects.gnss_start_stop_button);
        if (label) lv_label_set_text(label, "STOP");
    }

    if (objects.gnss_dump_panel)
    {
        ui_window_reset(&s_gnss_window, objects.gnss_dump_panel);
    }
    s_gnss_rx_count = 0;
    ui_stream_start(&s_gnss_stream);

    esp_err_t ret = gnss_monitor_start(baud_rate, tx_gpio, rx_gpio);
    if (ret != ESP_OK)
    {
        ui_stream_stop(&s_gnss_stream);
        if (objects.gnss_start_stop_button)
        {
            label = ui_get_first_child(objects.gnss_start_stop_button);
            if (label) lv_label_set_text(label, "START");
        }
        ui_handle_error("Failed to start GNSS listener", ret);
        return;
    }

    s_gnss_listening = true;
    gps_points_set_feed_active(GPS_POINTS_FEED_GNSS, true);
    snprintf(line, sizeof(line), "Started by user: Baud=%d TX=%d RX=%d", baud_rate, tx_gpio, rx_gpio);
    gnss_dump_append_line(line);
    ui_refresh_gnss_dump();

    snprintf(s_gnss_status_text, sizeof(s_gnss_status_text),
        "Listening @ %d baud (TX=%d RX=%d)",
        baud_rate, tx_gpio, rx_gpio);
    if (objects.gnss_status_label) lv_label_set_text(objects.gnss_status_label, s_gnss_status_text);
}

// Forward declarations for hardware functions (to be linked from main/sd_card.c, etc.)
extern void system_sdcard_test(uint8_t card_number);
extern uint8_t system_sdcard1_init(void);
extern bool sd_card1_is_mounted(void);
extern bool sd_card2_is_mounted(void);

// ─────────────────────────────────────────────────────────────────────────────
// FILE MANAGER AUTO-REFRESH SYSTEM
// ─────────────────────────────────────────────────────────────────────────────

typedef struct
{
    char mount_point[32];
    uint32_t file_count;
    time_t last_check;
    lv_obj_t* ui_container;  // Reference to UI element to update
    bool enabled;
} file_manager_state_t;

static file_manager_state_t file_manager_sd1 = { 0 };
static file_manager_state_t file_manager_sd2 = { 0 };

/**
 * @brief Scan directory and count files (detects changes)
 */
static uint32_t count_directory_files(const char* path)
{
    DIR* dir = opendir(path);
    if (!dir) return 0;

    uint32_t count = 0;
    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL)
    {
        // Skip . and .. entries
        if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
        {
            count++;
        }
    }
    closedir(dir);
    return count;
}

/**
 * @brief Auto-refresh timer callback for file manager
 * Detects new files and triggers UI updates
 */
static void file_manager_auto_refresh_cb(lv_timer_t* t)
{
    file_manager_state_t* fm = (file_manager_state_t*)lv_timer_get_user_data(t);

    if (!fm || !fm->enabled || fm->mount_point[0] == '\0')
    {
        return;
    }
    if (s_ota_update_in_progress || !ui_is_files_tab_active())
    {
        return;
    }

    // Count current files
    uint32_t current_count = count_directory_files(fm->mount_point);

    // Detect changes
    if (current_count != fm->file_count)
    {
        ESP_LOGI(TAG, "File manager detected change: %u -> %u files in %s",
            fm->file_count, current_count, fm->mount_point);

        fm->file_count = current_count;
        fm->last_check = time(NULL);

        if (fm->ui_container)
        {
            if (strcmp(g_file_manager_view.active_mount, fm->mount_point) == 0)
            {
                file_manager_load_files(fm->mount_point, fm->ui_container);
                ESP_LOGI(TAG, "File manager UI refreshed for %s", fm->mount_point);
            }
        }
    }
}

static void file_manager_initial_load_async(void* user_data)
{
    system_status_t status;

    LV_UNUSED(user_data);

    if (!g_file_manager_view.list_container)
    {
        return;
    }

    status = system_monitor_get_status();
    ui_update_storage_dependent_controls(&status);

    if (status.sdcard1_initialized)
    {
        file_manager_load_files("/sdcard", g_file_manager_view.list_container);
    }
    else if (status.sdcard2_initialized)
    {
        file_manager_load_files("/sdcard2", g_file_manager_view.list_container);
    }
    else
    {
        file_manager_show_placeholder("No active SD cards");
    }
}

/**
 * @brief Initialize file manager auto-refresh for a mount point
 */
void file_manager_enable_auto_refresh(const char* mount_point, lv_obj_t* ui_container)
{
    if (!mount_point || strlen(mount_point) == 0) return;

    file_manager_state_t* fm = NULL;

    if (strcmp(mount_point, "/sdcard") == 0)
    {
        fm = &file_manager_sd1;
    }
    else if (strcmp(mount_point, "/sdcard2") == 0)
    {
        fm = &file_manager_sd2;
    }
    else
    {
        return;
    }

    // Initialize or reset
    strncpy(fm->mount_point, mount_point, sizeof(fm->mount_point) - 1);
    fm->file_count = count_directory_files(mount_point);
    fm->last_check = time(NULL);
    fm->ui_container = ui_container;
    fm->enabled = true;

    ESP_LOGI(TAG, "File manager auto-refresh enabled for %s (initial: %u files)",
        mount_point, fm->file_count);

    // Create timer for periodic checks (every 2 seconds)
    lv_timer_t* timer = lv_timer_create(file_manager_auto_refresh_cb, 2000, fm);
    lv_timer_set_repeat_count(timer, -1);  // Infinite repeats
}

/**
 * @brief Disable file manager auto-refresh
 */
void file_manager_disable_auto_refresh(const char* mount_point)
{
    if (!mount_point) return;

    file_manager_state_t* fm = NULL;

    if (strcmp(mount_point, "/sdcard") == 0)
    {
        fm = &file_manager_sd1;
    }
    else if (strcmp(mount_point, "/sdcard2") == 0)
    {
        fm = &file_manager_sd2;
    }
    else
    {
        return;
    }

    fm->enabled = false;
    fm->mount_point[0] = '\0';

    ESP_LOGI(TAG, "File manager auto-refresh disabled for %s", mount_point);
}



static lv_obj_t* find_msgbox_ancestor(lv_obj_t* obj)
{
    while (obj)
    {
        if (lv_obj_check_type(obj, &lv_msgbox_class))
        {
            return obj;
        }
        obj = lv_obj_get_parent(obj);
    }
    return NULL;
}

static void ui_style_msgbox(lv_obj_t* mbox, lv_color_t bg_color)
{
    lv_obj_set_style_bg_color(mbox, bg_color, 0);
    lv_obj_set_style_bg_opa(mbox, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(mbox, lv_color_hex(UI_COLOR_BORDER), 0);
    lv_obj_set_style_border_width(mbox, 1, 0);
    lv_obj_set_style_radius(mbox, 16, 0);
    lv_obj_set_style_pad_all(mbox, 16, 0);
    lv_obj_set_style_text_color(mbox, lv_color_hex(0xff000000), 0);

    lv_obj_t* title = lv_msgbox_get_title(mbox);
    if (title)
    {
        lv_obj_set_style_text_color(title, lv_color_hex(0xff000000), 0);
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    }

    lv_obj_t* content = lv_msgbox_get_content(mbox);
    if (content)
    {
        lv_obj_set_style_bg_opa(content, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(content, 0, 0);
        lv_obj_set_style_text_color(content, lv_color_hex(0xff000000), 0);
    }

    lv_obj_t* footer = lv_msgbox_get_footer(mbox);
    if (footer)
    {
        // Footer should be transparent but arrange buttons nicely
        lv_obj_set_style_bg_opa(footer, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(footer, 0, 0);
        lv_obj_set_flex_flow(footer, LV_FLEX_FLOW_ROW);
        /* LVGL variant in this build doesn't provide lv_obj_set_style_pad_inner()
         * Use column padding to space items horizontally in a row flex container.
         */
        lv_obj_set_style_pad_column(footer, 12, 0);
        lv_obj_set_style_pad_top(footer, 8, 0);
        lv_obj_set_style_pad_bottom(footer, 8, 0);
        lv_obj_set_style_align(footer, LV_FLEX_ALIGN_CENTER, 0);
        uint32_t child_count = lv_obj_get_child_count(footer);
        for (uint32_t index = 0; index < child_count; index++)
        {
            lv_obj_t* child = lv_obj_get_child(footer, index);
            lv_obj_set_style_bg_color(child, lv_color_hex(UI_COLOR_SURFACE_ALT), 0);
            lv_obj_set_style_bg_opa(child, LV_OPA_COVER, 0);
            lv_obj_set_style_text_color(child, lv_color_hex(0xff000000), 0);
            lv_obj_set_style_border_color(child, lv_color_hex(UI_COLOR_BORDER), 0);
            lv_obj_set_style_border_width(child, 1, 0);
            lv_obj_set_style_radius(child, 12, 0);
            lv_obj_set_style_pad_hor(child, 18, 0);
            lv_obj_set_style_pad_ver(child, 10, 0);
            lv_obj_set_width(child, LV_PCT(0));
        }
    }
    // Limit width so msgboxes are not full-screen on large displays
    lv_obj_set_width(mbox, 640);
}

static void mbox_close_cb(lv_event_t* e)
{
    lv_obj_t* mbox = lv_event_get_current_target(e);
    lv_msgbox_close_async(mbox);
}

static void mbox_deleted_cb(lv_event_t* e)
{
    // When the messagebox is deleted, also delete its overlay parent (if any)
    lv_obj_t* mbox = lv_event_get_current_target(e);
    lv_obj_t* parent = lv_obj_get_parent(mbox);
    if (parent)
    {
        lv_obj_del_async(parent);
    }
}

/**
 * @brief Show a professional popup message/modal
 */
void ui_show_message(const char* title, const char* msg, lv_color_t color)
{
    // Create a semi-opaque overlay to dim background and capture taps
    lv_obj_t* overlay = lv_obj_create(lv_scr_act());
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_clear_flag(overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(overlay, lv_color_hex(0x000000), 0);
    /* Use an opaque overlay to avoid underlying header/title showing through
     * when viewing a file; a semi-transparent dim can reveal duplicate titles
     * and cause visual clutter. */
    lv_obj_set_style_bg_opa(overlay, LV_OPA_COVER, 0);

    lv_obj_t* mbox = lv_msgbox_create(overlay);
    lv_msgbox_add_title(mbox, title);
    lv_msgbox_add_text(mbox, msg);

    lv_obj_t* btn = lv_msgbox_add_close_button(mbox);
    lv_obj_add_event_cb(btn, mbox_close_cb, LV_EVENT_CLICKED, NULL);

    // Ensure overlay is removed when msgbox is deleted
    lv_obj_add_event_cb(mbox, mbox_deleted_cb, LV_EVENT_DELETE, NULL);

    ui_style_msgbox(mbox, color);
    lv_obj_center(mbox);
}

/**
 * @brief Create a modal (overlay + msgbox) and return the msgbox object.
 * The overlay parent will be deleted when the msgbox is deleted.
 */
static lv_obj_t* ui_create_modal(const char* title, const char* msg, lv_color_t color)
{
    lv_obj_t* overlay = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(overlay, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_60, 0);

    lv_obj_t* mbox = lv_msgbox_create(overlay);
    lv_msgbox_add_title(mbox, title);
    lv_msgbox_add_text(mbox, msg);
    ui_style_msgbox(mbox, color);
    lv_obj_add_event_cb(mbox, mbox_deleted_cb, LV_EVENT_DELETE, NULL);
    lv_obj_center(mbox);
    return mbox;
}

// ─────────────────────────────────────────────────────────────────────────────
// FILE MANAGER UI - FILE LISTING & NAVIGATION
// ─────────────────────────────────────────────────────────────────────────────

static void ui_style_surface(lv_obj_t* obj, lv_color_t bg_color, lv_coord_t radius)
{
    if (!obj)
    {
        return;
    }

    lv_obj_set_style_bg_color(obj, bg_color, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(obj, lv_color_hex(UI_COLOR_BORDER), 0);
    lv_obj_set_style_border_width(obj, 1, 0);
    lv_obj_set_style_radius(obj, radius, 0);
}

static void ui_style_button(lv_obj_t* button, lv_color_t bg_color, lv_color_t text_color)
{
    if (!button)
    {
        return;
    }

    lv_obj_set_style_bg_color(button, bg_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(button, lv_color_hex(UI_COLOR_SURFACE_ALT), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_border_color(button, lv_color_hex(UI_COLOR_BORDER), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(button, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(button, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_hor(button, 18, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_ver(button, 12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(button, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t* label = ui_get_first_child(button);
    if (label)
    {
        lv_obj_set_style_text_color(label, text_color, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_letter_space(label, 1, 0);
    }
}

static void ui_style_input(lv_obj_t* input)
{
    if (!input)
    {
        return;
    }

    ui_style_surface(input, lv_color_hex(UI_COLOR_SURFACE), 14);
    lv_obj_set_style_pad_left(input, 16, LV_PART_MAIN);
    lv_obj_set_style_pad_right(input, 16, LV_PART_MAIN);
    lv_obj_set_style_pad_top(input, 14, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(input, 14, LV_PART_MAIN);
    lv_obj_set_style_text_font(input, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_set_style_text_color(input, lv_color_hex(UI_COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_color(input, lv_color_hex(UI_COLOR_MUTED), LV_PART_TEXTAREA_PLACEHOLDER);
    lv_obj_set_style_border_color(input, lv_color_hex(UI_COLOR_ACCENT), LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_border_width(input, 2, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_shadow_width(input, 0, LV_PART_MAIN);
}

static void ui_style_keyboard(lv_obj_t* keyboard)
{
    if (!keyboard)
    {
        return;
    }

    ui_style_surface(keyboard, lv_color_hex(UI_COLOR_SURFACE), 18);
    lv_obj_set_style_pad_all(keyboard, 10, 0);
    lv_obj_set_style_pad_row(keyboard, 8, 0);
    lv_obj_set_style_pad_column(keyboard, 8, 0);
    lv_obj_set_style_bg_color(keyboard, lv_color_hex(UI_COLOR_SURFACE), LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(keyboard, lv_color_hex(UI_COLOR_ACCENT_ALT), LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_bg_color(keyboard, lv_color_hex(UI_COLOR_ACCENT), LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(keyboard, lv_color_hex(UI_COLOR_TEXT), LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(keyboard, lv_color_hex(UI_COLOR_BORDER), LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(keyboard, 1, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(keyboard, 12, LV_PART_ITEMS | LV_STATE_DEFAULT);
}

static void ui_update_file_tab_buttons(bool sd1_active)
{
    if (g_file_manager_view.sd1_button)
    {
        ui_style_button(g_file_manager_view.sd1_button,
            lv_color_hex(sd1_active ? UI_COLOR_ACCENT : UI_COLOR_SURFACE),
            lv_color_hex(UI_COLOR_TEXT));
    }
    if (g_file_manager_view.sd2_button)
    {
        ui_style_button(g_file_manager_view.sd2_button,
            lv_color_hex(sd1_active ? UI_COLOR_SURFACE : UI_COLOR_ACCENT),
            lv_color_hex(UI_COLOR_TEXT));
    }
}
static void ui_set_button_enabled(lv_obj_t* button, bool enabled)
{
    if (!button)
    {
        return;
    }

    if (enabled)
    {
        lv_obj_clear_state(button, LV_STATE_DISABLED);
        lv_obj_set_style_opa(button, LV_OPA_COVER, 0);
    }
    else
    {
        lv_obj_add_state(button, LV_STATE_DISABLED);
        lv_obj_set_style_opa(button, LV_OPA_50, 0);
    }
}

static void file_manager_show_placeholder(const char* text)
{
    uint32_t child_count;
    lv_obj_t* label;

    if (!g_file_manager_view.list_container)
    {
        return;
    }

    child_count = lv_obj_get_child_count(g_file_manager_view.list_container);
    while (child_count > 0)
    {
        lv_obj_del(lv_obj_get_child(g_file_manager_view.list_container, 0));
        child_count--;
    }

    label = lv_label_create(g_file_manager_view.list_container);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(UI_COLOR_MUTED), 0);
}

static void ui_update_storage_dependent_controls(const system_status_t* status)
{
    bool sd1_available;
    bool sd2_available;
    bool prefer_sd1;

    if (status == NULL)
    {
        return;
    }

    sd1_available = (status->sdcard1_initialized != 0);
    sd2_available = (status->sdcard2_initialized != 0);

    if (s_panel_sd1)
    {
        if (sd1_available)
        {
            lv_obj_clear_flag(s_panel_sd1, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(s_panel_sd1, LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (s_panel_sd2)
    {
        if (sd2_available)
        {
            lv_obj_clear_flag(s_panel_sd2, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(s_panel_sd2, LV_OBJ_FLAG_HIDDEN);
        }
    }

    ui_set_button_enabled(s_btn_ble_sd1, sd1_available);
    ui_set_button_enabled(s_btn_gnss_sd1, sd1_available);
    ui_set_button_enabled(s_btn_ble_sd2, sd2_available);
    ui_set_button_enabled(s_btn_gnss_sd2, sd2_available);

    if (g_file_manager_view.sd1_button)
    {
        if (sd1_available)
        {
            lv_obj_clear_flag(g_file_manager_view.sd1_button, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(g_file_manager_view.sd1_button, LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (g_file_manager_view.sd2_button)
    {
        if (sd2_available)
        {
            lv_obj_clear_flag(g_file_manager_view.sd2_button, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(g_file_manager_view.sd2_button, LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (!g_file_manager_view.list_container)
    {
        s_files_sd1_available = sd1_available;
        s_files_sd2_available = sd2_available;
        return;
    }

    prefer_sd1 = strcmp(g_file_manager_view.active_mount, "/sdcard2") != 0;

    if (!sd1_available && !sd2_available)
    {
        if (g_file_manager_view.status_label)
        {
            lv_label_set_text(g_file_manager_view.status_label, "No active storage");
        }
        g_file_manager_view.active_mount[0] = '\0';
    }
    else if (!sd1_available)
    {
        if (g_file_manager_view.status_label && strcmp(g_file_manager_view.active_mount, "/sdcard2") != 0)
        {
            lv_label_set_text(g_file_manager_view.status_label, "SD Card 1 inactive - use SD Card 2");
        }
        snprintf(g_file_manager_view.active_mount, sizeof(g_file_manager_view.active_mount), "%s", "/sdcard2");
        prefer_sd1 = false;
    }
    else if (!sd2_available)
    {
        if (g_file_manager_view.status_label && strcmp(g_file_manager_view.active_mount, "/sdcard") != 0)
        {
            lv_label_set_text(g_file_manager_view.status_label, "SD Card 2 inactive - use SD Card 1");
        }
        snprintf(g_file_manager_view.active_mount, sizeof(g_file_manager_view.active_mount), "%s", "/sdcard");
        prefer_sd1 = true;
    }
    else if (!s_files_sd1_available || !s_files_sd2_available)
    {
        if (g_file_manager_view.active_mount[0] == '\0')
        {
            snprintf(g_file_manager_view.active_mount, sizeof(g_file_manager_view.active_mount), "%s", "/sdcard");
            prefer_sd1 = true;
        }
    }

    if (g_file_manager_view.active_mount[0] != '\0')
    {
        if (strcmp(g_file_manager_view.active_mount, "/sdcard2") == 0)
        {
            prefer_sd1 = false;
        }
    }

    ui_update_file_tab_buttons(prefer_sd1);

    s_files_sd1_available = sd1_available;
    s_files_sd2_available = sd2_available;
}

static bool ui_is_text_file(const char* filename)
{
    const char* extension = filename ? strrchr(filename, '.') : NULL;

    if (!extension)
    {
        return false;
    }

    return strcasecmp(extension, ".txt") == 0 ||
        strcasecmp(extension, ".log") == 0 ||
        strcasecmp(extension, ".csv") == 0 ||
        strcasecmp(extension, ".json") == 0;
}

static void file_manager_open_text_modal(const file_entry_t* entry)
{
    if (!entry || entry->path[0] == '\0')
    {
        return;
    }

    lv_obj_t* overlay = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(overlay, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_60, 0);

    lv_obj_t* panel = lv_obj_create(overlay);
    lv_obj_set_size(panel, 920, 600);
    lv_obj_center(panel);
    ui_style_surface(panel, lv_color_hex(UI_COLOR_BG), 22);
    lv_obj_set_style_pad_all(panel, 18, 0);
    lv_obj_set_style_pad_row(panel, 14, 0);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);

    lv_obj_t* header = lv_obj_create(panel);
    lv_obj_set_width(header, LV_PCT(100));
    lv_obj_set_height(header, LV_SIZE_CONTENT);
    ui_style_surface(header, lv_color_hex(UI_COLOR_SURFACE), 16);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_style_pad_all(header, 14, 0);

    lv_obj_t* title = lv_label_create(header);
    lv_label_set_text(title, entry->filename);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(UI_COLOR_TEXT), 0);

    lv_obj_t* subtitle = lv_label_create(header);
    lv_label_set_text(subtitle, entry->path);
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(subtitle, lv_color_hex(UI_COLOR_MUTED), 0);
    lv_obj_align(subtitle, LV_ALIGN_BOTTOM_LEFT, 0, 0);

    lv_obj_t* preview_container = lv_obj_create(panel);
    lv_obj_set_width(preview_container, LV_PCT(100));
    lv_obj_set_flex_grow(preview_container, 1);
    ui_style_surface(preview_container, lv_color_hex(UI_COLOR_SURFACE), 16);
    lv_obj_set_style_pad_all(preview_container, 14, 0);
    lv_obj_set_style_pad_row(preview_container, 6, 0);
    lv_obj_set_scrollbar_mode(preview_container, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_set_flex_flow(preview_container, LV_FLEX_FLOW_COLUMN);
    notepad_viewer_load_file(entry->path, preview_container);

    lv_obj_t* actions = lv_obj_create(panel);
    lv_obj_set_width(actions, LV_PCT(100));
    lv_obj_set_height(actions, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(actions, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(actions, 0, 0);
    lv_obj_set_style_pad_all(actions, 0, 0);
    lv_obj_set_style_pad_column(actions, 12, 0);
    lv_obj_set_flex_flow(actions, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(actions, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t* delete_btn = lv_button_create(actions);
    lv_label_set_text(lv_label_create(delete_btn), "Delete File");
    ui_style_button(delete_btn, lv_color_hex(UI_COLOR_DANGER), lv_color_hex(UI_COLOR_TEXT));
    lv_obj_add_event_cb(delete_btn, file_modal_delete_cb, LV_EVENT_CLICKED, overlay);

    lv_obj_t* close_btn = lv_button_create(actions);
    lv_label_set_text(lv_label_create(close_btn), "Close");
    ui_style_button(close_btn, lv_color_hex(UI_COLOR_ACCENT), lv_color_hex(UI_COLOR_TEXT));
    lv_obj_add_event_cb(close_btn, file_modal_close_cb, LV_EVENT_CLICKED, overlay);
}

static void file_modal_close_cb(lv_event_t* e)
{
    lv_obj_t* overlay = (lv_obj_t*)lv_event_get_user_data(e);

    if (overlay && lv_obj_is_valid(overlay))
    {
        lv_obj_delete_async(overlay);
    }
}

static void file_modal_delete_cb(lv_event_t* e)
{
    lv_obj_t* overlay = (lv_obj_t*)lv_event_get_user_data(e);

    if (selected_file.path[0] == '\0' || selected_file.is_dir)
    {
        ui_show_message("Delete Failed", "Only text files can be deleted from this viewer.", lv_color_hex(UI_COLOR_DANGER));
        return;
    }

    if (remove(selected_file.path) != 0)
    {
        ui_show_message("Delete Failed", "Unable to delete the selected file.", lv_color_hex(UI_COLOR_DANGER));
        return;
    }

    if (overlay && lv_obj_is_valid(overlay))
    {
        lv_obj_delete_async(overlay);
    }

    if (g_file_manager_view.list_container && g_file_manager_view.active_mount[0] != '\0')
    {
        file_manager_load_files(g_file_manager_view.active_mount, g_file_manager_view.list_container);
    }

    memset(&selected_file, 0, sizeof(selected_file));
    ui_show_message("File Deleted", "The selected text file was removed successfully.", lv_color_hex(0xff000000));
}

/**
 * @brief Tab switching callback for SD Card 1
 */
static void file_manager_tab_sd1_cb(lv_event_t* e)
{
    if (!g_file_manager_view.list_container) return;
    if (!s_files_sd1_available) return;

    ui_update_file_tab_buttons(true);
    file_manager_load_files("/sdcard", g_file_manager_view.list_container);
    ESP_LOGI(TAG, "Switched to SD Card 1");
    (void)e;
}

/**
 * @brief Tab switching callback for SD Card 2
 */
static void file_manager_tab_sd2_cb(lv_event_t* e)
{
    if (!g_file_manager_view.list_container) return;
    if (!s_files_sd2_available) return;

    ui_update_file_tab_buttons(false);
    file_manager_load_files("/sdcard2", g_file_manager_view.list_container);
    ESP_LOGI(TAG, "Switched to SD Card 2");
    (void)e;
}

/**
 * @brief Load and display files from directory (LVGL 9.x compatible)
 */
void file_manager_load_files(const char* mount_point, lv_obj_t* container)
{
    if (!container || !mount_point) return;

    if ((strcmp(mount_point, "/sdcard") == 0 && !s_files_sd1_available) ||
        (strcmp(mount_point, "/sdcard2") == 0 && !s_files_sd2_available))
    {
        if (g_file_manager_view.status_label)
        {
            lv_label_set_text(g_file_manager_view.status_label, "Selected storage is inactive");
        }
        g_file_manager_view.active_mount[0] = '\0';
        file_manager_show_placeholder("Selected SD card is inactive");
        return;
    }

    strncpy(g_file_manager_view.active_mount, mount_point, sizeof(g_file_manager_view.active_mount) - 1);
    g_file_manager_view.active_mount[sizeof(g_file_manager_view.active_mount) - 1] = '\0';

    if (g_file_manager_view.status_label)
    {
        char status_text[96];
        snprintf(status_text, sizeof(status_text), "Showing %s", mount_point);
        lv_label_set_text(g_file_manager_view.status_label, status_text);
    }

    uint32_t child_count = lv_obj_get_child_count(container);
    for (int i = 0; i < child_count; i++)
    {
        lv_obj_del(lv_obj_get_child(container, 0));
    }

    memset(g_file_items, 0, sizeof(g_file_items));

    DIR* dir = opendir(mount_point);
    if (!dir)
    {
        lv_obj_t* err = lv_label_create(container);
        lv_label_set_text(err, "Unable to open storage");
        lv_obj_set_style_text_color(err, lv_color_hex(UI_COLOR_DANGER), 0);
        lv_obj_set_style_text_font(err, &lv_font_montserrat_16, 0);
        return;
    }

    struct dirent* entry;
    struct stat st;
    char full_path[512];
    int file_count = 0;

    while ((entry = readdir(dir)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
        {
            continue;
        }

        snprintf(full_path, sizeof(full_path), "%s/%s", mount_point, entry->d_name);

        if (stat(full_path, &st) == 0 && file_count < FILE_MANAGER_MAX_ITEMS)
        {
            file_item_context_t* item = &g_file_items[file_count];
            snprintf(item->full_path, sizeof(item->full_path), "%s", full_path);
            snprintf(item->filename, sizeof(item->filename), "%s", entry->d_name);
            snprintf(item->mount_point, sizeof(item->mount_point), "%s", mount_point);
            item->is_dir = S_ISDIR(st.st_mode);
            item->size = (uint32_t)st.st_size;

            lv_obj_t* btn = lv_button_create(container);
            lv_obj_set_width(btn, LV_PCT(100));
            lv_obj_set_height(btn, 58);
            ui_style_button(btn, lv_color_hex(UI_COLOR_SURFACE), lv_color_hex(UI_COLOR_TEXT));
            lv_obj_set_style_pad_left(btn, 14, 0);
            lv_obj_set_style_pad_right(btn, 14, 0);

            char text[160];
            snprintf(text, sizeof(text), "%s %s",
                item->is_dir ? LV_SYMBOL_DIRECTORY : LV_SYMBOL_FILE,
                item->filename);

            lv_obj_t* label = lv_label_create(btn);
            lv_label_set_text(label, text);
            lv_obj_set_width(label, LV_PCT(100));
            lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
            lv_obj_set_style_text_color(label, lv_color_hex(UI_COLOR_TEXT), 0);
            lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);
            lv_obj_center(label);

            lv_obj_add_event_cb(btn, file_manager_file_selected, LV_EVENT_CLICKED, item);
            file_count++;
        }
    }
    closedir(dir);

    if (file_count == 0)
    {
        lv_obj_t* empty = lv_label_create(container);
        lv_label_set_text(empty, "No files found");
        lv_obj_set_style_text_font(empty, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(empty, lv_color_hex(UI_COLOR_MUTED), 0);
    }
}

/**
 * @brief File selection callback (LVGL 9.x compatible)
 */
void file_manager_file_selected(lv_event_t* e)
{
    file_item_context_t* item = (file_item_context_t*)lv_event_get_user_data(e);

    if (!item) return;

    snprintf(selected_file.filename, sizeof(selected_file.filename), "%s", item->filename);
    snprintf(selected_file.path, sizeof(selected_file.path), "%s", item->full_path);
    snprintf(selected_file.mount_point, sizeof(selected_file.mount_point), "%s", item->mount_point);
    selected_file.is_dir = item->is_dir;
    selected_file.size = item->size;

    if (item->is_dir)
    {
        ui_show_message("Folder Selected", "Folder navigation is not enabled yet. Select a text file instead.", lv_color_hex(UI_COLOR_SURFACE));
        return;
    }

    if (!ui_is_text_file(item->filename))
    {
        ui_show_message("Preview Unavailable", "Only text-based files can be opened in the notepad viewer.", lv_color_hex(UI_COLOR_WARNING));
        return;
    }

    ESP_LOGI(TAG, "File selected: %s", selected_file.filename);
    file_manager_open_text_modal(&selected_file);
}

/**
 * @brief Load and display txt file content in notepad
 */
void notepad_viewer_load_file(const char* filepath, lv_obj_t* preview_container)
{
    if (!preview_container) return;

    uint32_t child_count = lv_obj_get_child_count(preview_container);
    for (uint32_t index = 0; index < child_count; index++)
    {
        lv_obj_del(lv_obj_get_child(preview_container, 0));
    }

    FILE* f = fopen(filepath, "r");
    if (!f)
    {
        lv_obj_t* error_label = lv_label_create(preview_container);
        lv_label_set_text(error_label, "Error: Could not open file");
        lv_obj_set_style_text_font(error_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(error_label, lv_color_hex(UI_COLOR_DANGER), 0);
        return;
    }

    char line_buffer[FILE_PREVIEW_MAX_LINE_LENGTH + 1] = { 0 };
    int ch;
    int line_length = 0;
    int line_count = 0;
    size_t total_bytes = 0;
    bool truncated = false;

    while ((ch = fgetc(f)) != EOF)
    {
        total_bytes++;

        if (total_bytes > FILE_PREVIEW_MAX_BYTES || line_count >= FILE_PREVIEW_MAX_LINES)
        {
            truncated = true;
            break;
        }

        if (ch == '\r')
        {
            continue;
        }

        if (ch == '\n' || line_length >= FILE_PREVIEW_MAX_LINE_LENGTH)
        {
            line_buffer[line_length] = '\0';

            lv_obj_t* line_label = lv_label_create(preview_container);
            lv_label_set_text(line_label, line_length > 0 ? line_buffer : " ");
            lv_obj_set_width(line_label, LV_PCT(100));
            lv_obj_set_style_text_font(line_label, &lv_font_montserrat_14, 0);
            lv_obj_set_style_text_color(line_label, lv_color_hex(UI_COLOR_TEXT), 0);
            lv_obj_set_style_text_align(line_label, LV_TEXT_ALIGN_LEFT, 0);
            /* Wrap long lines instead of clipping or replacing with dots. */
            lv_label_set_long_mode(line_label, LV_LABEL_LONG_WRAP);

            line_count++;
            line_length = 0;

            if (ch == '\n')
            {
                continue;
            }
        }

        if (!isprint((unsigned char)ch) && ch != '\t')
        {
            /* Replace non-printable characters with a space to avoid long
             * runs of dots or control glyphs in the preview. */
            ch = ' ';
        }

        if (ch == '\t')
        {
            ch = ' ';
        }

        if (line_length < FILE_PREVIEW_MAX_LINE_LENGTH)
        {
            line_buffer[line_length++] = (char)ch;
        }
    }

    if (line_length > 0 && line_count < FILE_PREVIEW_MAX_LINES)
    {
        line_buffer[line_length] = '\0';
        lv_obj_t* line_label = lv_label_create(preview_container);
        lv_label_set_text(line_label, line_buffer);
        lv_obj_set_width(line_label, LV_PCT(100));
        lv_obj_set_style_text_font(line_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(line_label, lv_color_hex(UI_COLOR_TEXT), 0);
        lv_obj_set_style_text_align(line_label, LV_TEXT_ALIGN_LEFT, 0);
        lv_label_set_long_mode(line_label, LV_LABEL_LONG_CLIP);
        line_count++;
    }

    fclose(f);

    if (line_count == 0)
    {
        lv_obj_t* empty_label = lv_label_create(preview_container);
        lv_label_set_text(empty_label, "File is empty");
        lv_obj_set_style_text_font(empty_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(empty_label, lv_color_hex(UI_COLOR_MUTED), 0);
    }

    if (truncated)
    {
        lv_obj_t* truncated_label = lv_label_create(preview_container);
        lv_label_set_text(truncated_label, "[Preview truncated for performance]");
        lv_obj_set_style_text_font(truncated_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(truncated_label, lv_color_hex(UI_COLOR_WARNING), 0);
    }
}

/**
 * @brief Create file manager screen with SD1 and SD2 tabs
 */
lv_obj_t* create_file_manager_screen(lv_obj_t* parent)
{
    lv_obj_t* cont = lv_obj_create(parent);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    ui_style_surface(cont, lv_color_hex(UI_COLOR_BG), 0);
    lv_obj_set_style_pad_all(cont, 0, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_radius(cont, 0, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);

    lv_obj_t* header = lv_obj_create(cont);
    lv_obj_set_size(header, LV_PCT(100), 68);
    ui_style_surface(header, lv_color_hex(UI_COLOR_SURFACE), 0);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_style_pad_left(header, 18, 0);
    lv_obj_set_style_pad_right(header, 18, 0);
    lv_obj_set_style_pad_top(header, 12, 0);
    lv_obj_set_style_pad_bottom(header, 8, 0);

    lv_obj_t* title = lv_label_create(header);
    lv_label_set_text(title, "File Manager");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(UI_COLOR_TEXT), 0);

    lv_obj_t* subtitle = lv_label_create(header);
    lv_label_set_text(subtitle, "Tap a text file to open it in a clean notepad viewer.");
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(subtitle, lv_color_hex(UI_COLOR_MUTED), 0);
    lv_obj_align(subtitle, LV_ALIGN_BOTTOM_LEFT, 0, 0);

    lv_obj_t* tab_container = lv_obj_create(cont);
    lv_obj_set_size(tab_container, LV_PCT(100), 64);
    ui_style_surface(tab_container, lv_color_hex(UI_COLOR_BG), 0);
    lv_obj_set_style_border_width(tab_container, 0, 0);
    lv_obj_set_style_pad_left(tab_container, 18, 0);
    lv_obj_set_style_pad_right(tab_container, 18, 0);
    lv_obj_set_style_pad_top(tab_container, 8, 0);
    lv_obj_set_style_pad_bottom(tab_container, 8, 0);
    lv_obj_set_style_pad_column(tab_container, 10, 0);
    lv_obj_set_flex_flow(tab_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(tab_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t* btn_sd1 = lv_button_create(tab_container);
    lv_obj_set_size(btn_sd1, 160, 46);
    lv_label_set_text(lv_label_create(btn_sd1), "SD Card 1");

    lv_obj_t* btn_sd2 = lv_button_create(tab_container);
    lv_obj_set_size(btn_sd2, 160, 46);
    lv_label_set_text(lv_label_create(btn_sd2), "SD Card 2");

    lv_obj_t* status_label = lv_label_create(tab_container);
    lv_label_set_text(status_label, "Showing /sdcard");
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(status_label, lv_color_hex(UI_COLOR_MUTED), 0);

    lv_obj_t* content = lv_obj_create(cont);
    lv_obj_set_size(content, LV_PCT(100), LV_PCT(100));
    ui_style_surface(content, lv_color_hex(UI_COLOR_BG), 0);
    lv_obj_set_style_border_width(content, 0, 0);
    lv_obj_set_style_pad_left(content, 18, 0);
    lv_obj_set_style_pad_right(content, 18, 0);
    lv_obj_set_style_pad_top(content, 6, 0);
    lv_obj_set_style_pad_bottom(content, 18, 0);

    lv_obj_t* file_list = lv_obj_create(content);
    lv_obj_set_size(file_list, LV_PCT(100), LV_PCT(100));
    ui_style_surface(file_list, lv_color_hex(UI_COLOR_SURFACE), 18);
    lv_obj_set_style_pad_all(file_list, 10, 0);
    lv_obj_set_style_pad_row(file_list, 8, 0);
    lv_obj_set_flex_flow(file_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scrollbar_mode(file_list, LV_SCROLLBAR_MODE_AUTO);

    g_file_manager_view.list_container = file_list;
    g_file_manager_view.status_label = status_label;
    g_file_manager_view.sd1_button = btn_sd1;
    g_file_manager_view.sd2_button = btn_sd2;

    lv_obj_add_event_cb(btn_sd1, file_manager_tab_sd1_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(btn_sd2, file_manager_tab_sd2_cb, LV_EVENT_CLICKED, NULL);
    file_manager_enable_auto_refresh("/sdcard", file_list);
    file_manager_enable_auto_refresh("/sdcard2", file_list);
    {
        system_status_t status = system_monitor_get_status();
        ui_update_storage_dependent_controls(&status);
    }
    file_manager_show_placeholder("Loading files...");
    lv_async_call(file_manager_initial_load_async, NULL);

    return cont;
}

static void ui_get_rallybox_id(char* out_text, size_t out_text_len)
{
    uint8_t base_mac[6] = { 0 };

    if (out_text == NULL || out_text_len == 0)
    {
        return;
    }

    if (esp_read_mac(base_mac, ESP_MAC_BASE) != ESP_OK)
    {
        snprintf(out_text, out_text_len, "%s", "Unavailable");
        return;
    }

    snprintf(out_text,
        out_text_len,
        "RBX-P4-%02X%02X%02X%02X%02X%02X",
        base_mac[0],
        base_mac[1],
        base_mac[2],
        base_mac[3],
        base_mac[4],
        base_mac[5]);
}

static const char* ui_get_firmware_version(void)
{
    return RALLYBOX_FW_VERSION;
}

static const char* ui_get_build_timestamp(void)
{
    return __DATE__ " " __TIME__;
}

static void ui_refresh_ota_button_state(void)
{
    lv_obj_t* label;
    bool should_disable;

    if (s_settings_ota_button == NULL)
    {
        return;
    }

    should_disable = s_ota_update_in_progress;
    if (s_ota_button_disabled_cached == (int8_t)(should_disable ? 1 : 0))
    {
        return;
    }

    label = ui_get_first_child(s_settings_ota_button);
    if (should_disable)
    {
        lv_obj_add_state(s_settings_ota_button, LV_STATE_DISABLED);
        if (label)
        {
            lv_label_set_text(label, "Updating...");
        }
    }
    else
    {
        lv_obj_clear_state(s_settings_ota_button, LV_STATE_DISABLED);
        if (label)
        {
            lv_label_set_text(label, "Update OTA");
        }
    }

    s_ota_button_disabled_cached = should_disable ? 1 : 0;
}

static void ui_set_ota_status(const char* text, uint32_t color)
{
    int percent = 0;
    bool status_changed;

    if (text == NULL)
    {
        text = "";
    }

    if (strcmp(s_settings_ota_status_text, text) == 0 && s_settings_ota_status_color == color)
    {
        return;
    }

    if (sscanf(text, "Downloading firmware... %d%%", &percent) == 1)
    {
        if (percent < 0)
        {
            percent = 0;
        }
        else if (percent > 100)
        {
            percent = 100;
        }

        s_settings_ota_progress_visible = true;
        s_settings_ota_progress_percent = percent;
        snprintf(s_settings_ota_progress_value_text, sizeof(s_settings_ota_progress_value_text), "%d%%", percent);
        ui_refresh_ota_progress_widgets();

        snprintf(s_settings_ota_status_text,
            sizeof(s_settings_ota_status_text),
            "%s",
            "Downloading firmware");
        s_settings_ota_status_color = UI_COLOR_WARNING;
        status_changed = strcmp(s_settings_ota_status_text_cached, s_settings_ota_status_text) != 0 ||
            s_settings_ota_status_color_cached != s_settings_ota_status_color;

        if (s_settings_ota_status_value && status_changed)
        {
            lv_label_set_text(s_settings_ota_status_value, s_settings_ota_status_text);
            lv_obj_set_style_text_color(s_settings_ota_status_value, lv_color_hex(s_settings_ota_status_color), 0);
            snprintf(s_settings_ota_status_text_cached,
                sizeof(s_settings_ota_status_text_cached),
                "%s",
                s_settings_ota_status_text);
            s_settings_ota_status_color_cached = s_settings_ota_status_color;
        }

        s_ota_progress_last_ui_update_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        return;
    }

    if (ui_is_ota_progress_message(text))
    {
        s_settings_ota_progress_visible = true;
        snprintf(s_settings_ota_progress_value_text,
            sizeof(s_settings_ota_progress_value_text),
            "%s",
            "Working");
        ui_refresh_ota_progress_widgets();

        snprintf(s_settings_ota_status_text,
            sizeof(s_settings_ota_status_text),
            "%s",
            "Downloading firmware");
        s_settings_ota_status_color = UI_COLOR_WARNING;
        status_changed = strcmp(s_settings_ota_status_text_cached, s_settings_ota_status_text) != 0 ||
            s_settings_ota_status_color_cached != s_settings_ota_status_color;

        if (s_settings_ota_status_value && status_changed)
        {
            lv_label_set_text(s_settings_ota_status_value, s_settings_ota_status_text);
            lv_obj_set_style_text_color(s_settings_ota_status_value, lv_color_hex(s_settings_ota_status_color), 0);
            snprintf(s_settings_ota_status_text_cached,
                sizeof(s_settings_ota_status_text_cached),
                "%s",
                s_settings_ota_status_text);
            s_settings_ota_status_color_cached = s_settings_ota_status_color;
        }

        s_ota_progress_last_ui_update_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        return;
    }

    snprintf(s_settings_ota_status_text,
        sizeof(s_settings_ota_status_text),
        "%s",
        text);
    s_settings_ota_status_color = color;

    s_settings_ota_progress_visible = false;
    ui_refresh_ota_progress_widgets();

    if (s_settings_ota_status_value)
    {
        status_changed = strcmp(s_settings_ota_status_text_cached, s_settings_ota_status_text) != 0 ||
            s_settings_ota_status_color_cached != color;
        if (status_changed)
        {
            lv_label_set_text(s_settings_ota_status_value, s_settings_ota_status_text);
            lv_obj_set_style_text_color(s_settings_ota_status_value, lv_color_hex(color), 0);
            snprintf(s_settings_ota_status_text_cached,
                sizeof(s_settings_ota_status_text_cached),
                "%s",
                s_settings_ota_status_text);
            s_settings_ota_status_color_cached = color;
        }
    }
}

static bool ui_is_ota_progress_message(const char* msg)
{
    return msg != NULL && strncmp(msg, "Downloading firmware...", strlen("Downloading firmware...")) == 0;
}

static void ui_refresh_ota_progress_widgets(void)
{
    if (s_settings_ota_progress_row == NULL)
    {
        return;
    }

    if (s_settings_ota_progress_visible != s_settings_ota_progress_visible_cached)
    {
        if (s_settings_ota_progress_visible)
        {
            lv_obj_clear_flag(s_settings_ota_progress_row, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(s_settings_ota_progress_row, LV_OBJ_FLAG_HIDDEN);
        }
        s_settings_ota_progress_visible_cached = s_settings_ota_progress_visible;
    }

    if (s_settings_ota_progress_bar && s_settings_ota_progress_percent != s_settings_ota_progress_percent_cached)
    {
        lv_bar_set_value(s_settings_ota_progress_bar, s_settings_ota_progress_percent, LV_ANIM_OFF);
        s_settings_ota_progress_percent_cached = s_settings_ota_progress_percent;
    }

    if (s_settings_ota_progress_value && strcmp(s_settings_ota_progress_value_text, s_settings_ota_progress_value_text_cached) != 0)
    {
        lv_label_set_text(s_settings_ota_progress_value, s_settings_ota_progress_value_text);
        snprintf(s_settings_ota_progress_value_text_cached,
            sizeof(s_settings_ota_progress_value_text_cached),
            "%s",
            s_settings_ota_progress_value_text);
    }
}

static void ui_queue_deferred_message(const char* title, const char* msg, uint32_t color, bool restart_requested)
{
    if (s_ui_deferred_message_mutex == NULL)
    {
        s_ui_deferred_message_mutex = xSemaphoreCreateMutex();
        if (s_ui_deferred_message_mutex == NULL)
        {
            ESP_LOGE(TAG, "Failed to create deferred message mutex");
            return;
        }
    }

    if (xSemaphoreTake(s_ui_deferred_message_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Deferred message queue busy");
        return;
    }

    s_ui_deferred_message.pending = true;
    s_ui_deferred_message.restart_requested = restart_requested;
    s_ui_deferred_message.color = color;
    snprintf(s_ui_deferred_message.title, sizeof(s_ui_deferred_message.title), "%s", title ? title : "Update");
    snprintf(s_ui_deferred_message.message, sizeof(s_ui_deferred_message.message), "%s", msg ? msg : "");

    xSemaphoreGive(s_ui_deferred_message_mutex);
}

static void ui_restart_timer_cb(lv_timer_t* timer)
{
    if (timer)
    {
        lv_timer_delete(timer);
    }
    esp_restart();
}

static void ui_process_deferred_message(void)
{
    ui_deferred_message_t pending = { 0 };
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    if (s_ui_deferred_message_mutex == NULL)
    {
        ui_refresh_ota_button_state();
        return;
    }

    if (xSemaphoreTake(s_ui_deferred_message_mutex, 0) != pdTRUE)
    {
        return;
    }

    if (s_ui_deferred_message.pending)
    {
        if (ui_is_ota_progress_message(s_ui_deferred_message.message) &&
            s_ota_progress_last_ui_update_ms != 0 &&
            (now_ms - s_ota_progress_last_ui_update_ms) < UI_OTA_PROGRESS_UI_INTERVAL_MS)
        {
            xSemaphoreGive(s_ui_deferred_message_mutex);
            ui_refresh_ota_button_state();
            return;
        }

        pending = s_ui_deferred_message;
        memset(&s_ui_deferred_message, 0, sizeof(s_ui_deferred_message));
    }

    xSemaphoreGive(s_ui_deferred_message_mutex);

    ui_refresh_ota_button_state();

    if (!pending.pending)
    {
        return;
    }

    ui_set_ota_status(pending.message, pending.color);
    if (pending.restart_requested)
    {
        lv_timer_t* restart_timer = lv_timer_create(ui_restart_timer_cb, UI_OTA_RESTART_DELAY_MS, NULL);
        if (restart_timer)
        {
            lv_timer_set_repeat_count(restart_timer, 1);
        }
    }
}

static bool ui_build_ota_object_request(
    const char* object_name,
    char* out_host,
    size_t out_host_len,
    char* out_path,
    size_t out_path_len,
    char* out_url,
    size_t out_url_len)
{
    char prefix[160];
    size_t prefix_len;
    const char* bucket = CONFIG_RALLYBOX_OTA_S3_BUCKET[0] != '\0' ? CONFIG_RALLYBOX_OTA_S3_BUCKET : CONFIG_RALLYBOX_GPX_S3_BUCKET;
    const char* region = CONFIG_RALLYBOX_OTA_S3_REGION[0] != '\0' ? CONFIG_RALLYBOX_OTA_S3_REGION : CONFIG_RALLYBOX_GPX_S3_REGION;
    const char* raw_prefix = CONFIG_RALLYBOX_OTA_S3_OBJECT_PREFIX;
    const char* filename = object_name;

    if (bucket[0] == '\0' || filename[0] == '\0')
    {
        return false;
    }

    while (*raw_prefix == '/')
    {
        raw_prefix++;
    }

    snprintf(prefix, sizeof(prefix), "%s", raw_prefix);
    prefix_len = strlen(prefix);
    if (prefix_len > 0 && prefix[prefix_len - 1] != '/')
    {
        if (prefix_len + 1 >= sizeof(prefix))
        {
            return false;
        }
        prefix[prefix_len] = '/';
        prefix[prefix_len + 1] = '\0';
    }

    if (region[0] == '\0')
    {
        region = "us-east-1";
    }

#if CONFIG_RALLYBOX_GPX_S3_VIRTUAL_HOSTED_STYLE
    if (out_host != NULL && out_host_len > 0)
    {
        snprintf(out_host, out_host_len, "%s.s3.%s.amazonaws.com", bucket, region);
}
    if (out_path != NULL && out_path_len > 0)
    {
        snprintf(out_path, out_path_len, "/%s%s", prefix, filename);
    }
    if (out_url != NULL && out_url_len > 0)
    {
        snprintf(out_url,
            out_url_len,
            "%s://%s.s3.%s.amazonaws.com/%s%s",
#if CONFIG_RALLYBOX_GPX_S3_USE_HTTPS
            "https",
#else
            "http",
#endif
            bucket,
            region,
            prefix,
            filename);
    }
#else
    if (out_host != NULL && out_host_len > 0)
    {
        snprintf(out_host, out_host_len, "s3.%s.amazonaws.com", region);
    }
    if (out_path != NULL && out_path_len > 0)
    {
        snprintf(out_path, out_path_len, "/%s/%s%s", bucket, prefix, filename);
    }
    if (out_url != NULL && out_url_len > 0)
    {
        snprintf(out_url,
            out_url_len,
            "%s://s3.%s.amazonaws.com/%s/%s%s",
#if CONFIG_RALLYBOX_GPX_S3_USE_HTTPS
            "https",
#else
            "http",
#endif
            region,
            bucket,
            prefix,
            filename);
    }
#endif

    return true;
}

static bool ui_build_ota_request(
    char* out_host,
    size_t out_host_len,
    char* out_path,
    size_t out_path_len,
    char* out_url,
    size_t out_url_len)
{
    return ui_build_ota_object_request(CONFIG_RALLYBOX_OTA_FILENAME,
        out_host,
        out_host_len,
        out_path,
        out_path_len,
        out_url,
        out_url_len);
}

static bool ui_build_ota_url(char* out_url, size_t out_url_len)
{
    return ui_build_ota_request(NULL, 0, NULL, 0, out_url, out_url_len);
}

static bool ui_parse_json_int_field_range(const char* start, const char* end, const char* field_name, int* out_value)
{
    char pattern[48];
    const char* field_pos;
    const char* colon;
    char* parse_end = NULL;
    long parsed;

    if (start == NULL || end == NULL || field_name == NULL || out_value == NULL || start >= end)
    {
        return false;
    }

    snprintf(pattern, sizeof(pattern), "\"%s\"", field_name);
    field_pos = strstr(start, pattern);
    if (field_pos == NULL || field_pos >= end)
    {
        return false;
    }

    colon = strchr(field_pos, ':');
    if (colon == NULL || colon >= end)
    {
        return false;
    }

    colon++;
    while (colon < end && isspace((unsigned char)*colon))
    {
        colon++;
    }

    errno = 0;
    parsed = strtol(colon, &parse_end, 10);
    if (errno != 0 || parse_end == colon || parse_end > end)
    {
        return false;
    }

    *out_value = (int)parsed;
    return true;
}

static bool ui_parse_json_string_field_range(const char* start, const char* end, const char* field_name, char* out_value, size_t out_value_len)
{
    char pattern[48];
    const char* field_pos;
    const char* colon;
    const char* value_start;
    const char* value_end;
    size_t copy_len;

    if (start == NULL || end == NULL || field_name == NULL || out_value == NULL || out_value_len == 0 || start >= end)
    {
        return false;
    }

    snprintf(pattern, sizeof(pattern), "\"%s\"", field_name);
    field_pos = strstr(start, pattern);
    if (field_pos == NULL || field_pos >= end)
    {
        return false;
    }

    colon = strchr(field_pos, ':');
    if (colon == NULL || colon >= end)
    {
        return false;
    }

    colon++;
    while (colon < end && isspace((unsigned char)*colon))
    {
        colon++;
    }

    if (colon >= end || *colon != '"')
    {
        return false;
    }

    value_start = colon + 1;
    value_end = strchr(value_start, '"');
    if (value_end == NULL || value_end > end)
    {
        return false;
    }

    copy_len = (size_t)(value_end - value_start);
    if (copy_len >= out_value_len)
    {
        copy_len = out_value_len - 1;
    }

    memcpy(out_value, value_start, copy_len);
    out_value[copy_len] = '\0';
    return true;
}

static bool ui_fetch_ota_source_metadata(ui_ota_source_metadata_t* metadata)
{
    char host[192];
    char path[256];
    char url[512];
    esp_http_client_config_t http_config = { 0 };
    esp_http_client_handle_t client = NULL;
    char* response = NULL;
    int response_capacity = (int)UI_OTA_MANIFEST_MAX_BYTES;
    int response_len = 0;
    int read_len;
    int status_code;
    bool parsed_ok = false;
    const char* stable_start;
    const char* stable_end;

    if (metadata == NULL)
    {
        return false;
    }

    memset(metadata, 0, sizeof(*metadata));

    if (!ui_build_ota_object_request(UI_OTA_MANIFEST_FILENAME,
        host,
        sizeof(host),
        path,
        sizeof(path),
        url,
        sizeof(url)))
    {
        return false;
    }

    http_config.host = host;
    http_config.path = path;
#if CONFIG_RALLYBOX_GPX_S3_USE_HTTPS
    http_config.port = 443;
    http_config.transport_type = HTTP_TRANSPORT_OVER_SSL;
#else
    http_config.port = 80;
    http_config.transport_type = HTTP_TRANSPORT_OVER_TCP;
#endif
    http_config.timeout_ms = 15000;
    http_config.keep_alive_enable = true;
#if UI_HAS_CRT_BUNDLE
    http_config.crt_bundle_attach = esp_crt_bundle_attach;
#endif

    client = esp_http_client_init(&http_config);
    if (client == NULL)
    {
        ESP_LOGW(TAG, "Failed to create HTTP client for OTA manifest");
        return false;
    }

    response = malloc(UI_OTA_MANIFEST_MAX_BYTES + 1U);
    if (response == NULL)
    {
        ESP_LOGW(TAG, "Failed to allocate OTA manifest buffer");
        esp_http_client_cleanup(client);
        return false;
    }

    if (esp_http_client_open(client, 0) != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to open OTA manifest request: %s", url);
        goto cleanup;
    }

    status_code = esp_http_client_fetch_headers(client);
    if (status_code < 0)
    {
        ESP_LOGW(TAG, "Failed to fetch OTA manifest headers");
        goto cleanup;
    }

    status_code = esp_http_client_get_status_code(client);
    if (status_code != 200)
    {
        ESP_LOGW(TAG, "OTA manifest request returned HTTP %d", status_code);
        goto cleanup;
    }

    while (response_len < response_capacity)
    {
        read_len = esp_http_client_read(client,
            response + response_len,
            response_capacity - response_len);
        if (read_len < 0)
        {
            ESP_LOGW(TAG, "Failed while reading OTA manifest response");
            goto cleanup;
        }
        if (read_len == 0)
        {
            break;
        }
        response_len += read_len;
    }

    response[response_len] = '\0';
    stable_start = strstr(response, "\"stable\"");
    if (stable_start == NULL)
    {
        ESP_LOGW(TAG, "OTA manifest missing stable section");
        goto cleanup;
    }

    stable_end = strstr(stable_start, "\"versioned\"");
    if (stable_end == NULL)
    {
        stable_end = response + response_len;
    }

    parsed_ok = ui_parse_json_int_field_range(stable_start, stable_end, "sizeBytes", &metadata->size_bytes) &&
        ui_parse_json_string_field_range(stable_start, stable_end, "sha256", metadata->sha256, sizeof(metadata->sha256));
    if (!parsed_ok)
    {
        ESP_LOGW(TAG, "OTA manifest missing stable sizeBytes/sha256 fields");
        goto cleanup;
    }

    (void)ui_parse_json_string_field_range(stable_start, stable_end, "url", metadata->url, sizeof(metadata->url));
    metadata->available = true;
    ESP_LOGI(TAG,
        "OTA source manifest: size=%d bytes sha256=%s%s%s",
        metadata->size_bytes,
        metadata->sha256,
        metadata->url[0] != '\0' ? " url=" : "",
        metadata->url[0] != '\0' ? metadata->url : "");

cleanup:
    if (client != NULL)
    {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
    }
    free(response);
    return metadata->available;
}

static bool ui_compute_partition_sha256_hex(const esp_partition_t* partition, size_t image_size, char* out_hex, size_t out_hex_len)
{
    uint8_t* chunk = NULL;
    uint8_t digest[32];
    psa_hash_operation_t sha_op = PSA_HASH_OPERATION_INIT;
    size_t offset = 0;
    bool success = false;

    if (partition == NULL || out_hex == NULL || out_hex_len < (UI_OTA_SHA256_HEX_LEN + 1U) || image_size == 0 || image_size > partition->size)
    {
        return false;
    }

    chunk = malloc(UI_OTA_HASH_CHUNK_BYTES);
    if (chunk == NULL)
    {
        ESP_LOGW(TAG, "Failed to allocate OTA hash buffer");
        return false;
    }

    if (psa_hash_setup(&sha_op, PSA_ALG_SHA_256) != PSA_SUCCESS)
    {
        ESP_LOGW(TAG, "Failed to initialise SHA-256 context");
        goto cleanup;
    }

    while (offset < image_size)
    {
        size_t chunk_len = image_size - offset;
        if (chunk_len > UI_OTA_HASH_CHUNK_BYTES)
        {
            chunk_len = UI_OTA_HASH_CHUNK_BYTES;
        }

        if (esp_partition_read(partition, (size_t)offset, chunk, chunk_len) != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to read OTA partition at offset %u for hashing", (unsigned)offset);
            goto cleanup;
        }

        if (psa_hash_update(&sha_op, chunk, chunk_len) != PSA_SUCCESS)
        {
            ESP_LOGW(TAG, "SHA-256 update failed at offset %u", (unsigned)offset);
            goto cleanup;
        }

        offset += chunk_len;
    }

    size_t hash_length;
    if (psa_hash_finish(&sha_op, digest, sizeof(digest), &hash_length) != PSA_SUCCESS)
    {
        ESP_LOGW(TAG, "SHA-256 finish failed");
        goto cleanup;
    }

    for (offset = 0; offset < sizeof(digest); ++offset)
    {
        snprintf(out_hex + (offset * 2U), out_hex_len - (offset * 2U), "%02x", digest[offset]);
    }
    out_hex[UI_OTA_SHA256_HEX_LEN] = '\0';
    success = true;

cleanup:
    psa_hash_abort(&sha_op);
    free(chunk);
    return success;
}

static bool ui_is_wifi_ready_for_ota(void)
{
    system_status_t status = system_monitor_get_status();

    return status.wifi_connected &&
        status.wifi_state == SYSTEM_WIFI_STATE_CONNECTED &&
        status.wifi_ip[0] != '\0';
}

static void ui_ota_update_task(void* param)
{
    char msg[224];
    char host[192];
    char path[256];
    char url[512];
    char progress_msg[224];
    char downloaded_sha256[(UI_OTA_SHA256_HEX_LEN + 1U)] = { 0 };
    esp_err_t ret;
    esp_https_ota_handle_t ota_handle = NULL;
    esp_http_client_config_t http_config = { 0 };
    esp_https_ota_config_t ota_config = { 0 };
    const esp_partition_t* update_partition = NULL;
    ui_ota_source_metadata_t source_metadata = { 0 };
    bool aborted_for_wifi = false;
    bool downloaded_hash_available = false;
    bool logged_invalid_total_size = false;
    bool logged_progress_complete = false;
    int last_reported_percent = 0;
    int last_reported_kb = 0;
    int last_logged_percent = 0;
    int final_image_len_read = 0;
    int final_image_size = 0;
    int bytes_hashed = 0;
    uint32_t ota_started_ms = 0;
    uint32_t last_waiting_status_ms = 0;
    wifi_ps_type_t previous_ps_mode = WIFI_PS_NONE;
    bool ps_mode_changed = false;

    LV_UNUSED(param);

    if (!ui_build_ota_request(host, sizeof(host), path, sizeof(path), url, sizeof(url)))
    {
        s_ota_update_in_progress = false;
        ui_queue_deferred_message("OTA Update",
            "OTA S3 settings are incomplete. Configure the bucket, region, and firmware filename in menuconfig.",
            UI_COLOR_WARNING,
            false);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "OTA request host='%s'", host);
    ESP_LOGI(TAG, "OTA request path='%s'", path);
    ESP_LOGI(TAG, "OTA request url='%s'", url);

    (void)ui_fetch_ota_source_metadata(&source_metadata);

    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition != NULL)
    {
        ESP_LOGI(TAG,
            "OTA target partition: label=%s address=0x%lx size=%lu bytes",
            update_partition->label,
            (unsigned long)update_partition->address,
            (unsigned long)update_partition->size);
    }

    if (!ui_is_wifi_ready_for_ota())
    {
        s_ota_update_in_progress = false;
        ui_queue_deferred_message("OTA Update",
            "Wi-Fi disconnected before the OTA download could start. Reconnect and try again.",
            UI_COLOR_WARNING,
            false);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Starting OTA download from %s", url);
    ota_started_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    last_waiting_status_ms = ota_started_ms;
    ui_queue_deferred_message("OTA Update",
        "Connecting to OTA server...",
        UI_COLOR_WARNING,
        false);

    ret = esp_wifi_get_ps(&previous_ps_mode);
    if (ret == ESP_OK)
    {
        if (previous_ps_mode != WIFI_PS_NONE)
        {
            esp_err_t ps_ret = esp_wifi_set_ps(WIFI_PS_NONE);
            if (ps_ret == ESP_OK)
            {
                ps_mode_changed = true;
                ESP_LOGI(TAG, "Disabled Wi-Fi power save during OTA (previous mode=%d)", (int)previous_ps_mode);
            }
            else
            {
                ESP_LOGW(TAG, "Failed to disable Wi-Fi power save for OTA: %s", esp_err_to_name(ps_ret));
            }
        }
    }
    else
    {
        ESP_LOGW(TAG, "Failed to query Wi-Fi power save mode before OTA: %s", esp_err_to_name(ret));
    }

    http_config.host = host;
    http_config.path = path;
#if CONFIG_RALLYBOX_GPX_S3_USE_HTTPS
    http_config.port = 443;
    http_config.transport_type = HTTP_TRANSPORT_OVER_SSL;
#else
    http_config.port = 80;
    http_config.transport_type = HTTP_TRANSPORT_OVER_TCP;
#endif
    http_config.timeout_ms = 30000;
    http_config.keep_alive_enable = true;
#if UI_HAS_CRT_BUNDLE
    http_config.crt_bundle_attach = esp_crt_bundle_attach;
#endif
    ota_config.http_config = &http_config;

    ret = esp_https_ota_begin(&ota_config, &ota_handle);
    if (ret == ESP_OK)
    {
        ui_queue_deferred_message("OTA Update",
            "Connected. Waiting for first OTA data...",
            UI_COLOR_WARNING,
            false);

        while (true)
        {
            int image_len_read;
            int image_size;
            bool can_report_percent;
            uint32_t now_ms;

            if (!ui_is_wifi_ready_for_ota())
            {
                ESP_LOGW(TAG, "Aborting OTA because Wi-Fi is no longer connected");
                aborted_for_wifi = true;
                ret = ESP_ERR_INVALID_STATE;
                break;
            }

            ret = esp_https_ota_perform(ota_handle);
            now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
            image_len_read = esp_https_ota_get_image_len_read(ota_handle);
            image_size = esp_https_ota_get_image_size(ota_handle);
            final_image_len_read = image_len_read;
            final_image_size = image_size;
            can_report_percent = image_size > 0 && image_len_read >= 0 && image_len_read <= image_size;

            if (image_len_read <= 0 && (now_ms - last_waiting_status_ms) >= UI_OTA_WAITING_STATUS_INTERVAL_MS)
            {
                uint32_t waited_seconds = (now_ms - ota_started_ms) / 1000U;
                snprintf(progress_msg,
                    sizeof(progress_msg),
                    "Connected. Waiting for OTA data... %lus",
                    (unsigned long)waited_seconds);
                ui_queue_deferred_message("OTA Update", progress_msg, UI_COLOR_WARNING, false);
                last_waiting_status_ms = now_ms;
            }

            if (!can_report_percent && image_size > 0 && image_len_read > image_size && !logged_invalid_total_size)
            {
                ESP_LOGW(TAG,
                    "OTA progress size mismatch: read=%d bytes total=%d bytes; falling back to downloaded-bytes status",
                    image_len_read,
                    image_size);
                logged_invalid_total_size = true;
            }

            if (image_len_read > 0)
            {
                if (can_report_percent)
                {
                    int percent = (int)(((int64_t)image_len_read * 100LL) / image_size);
                    if (percent > 100)
                    {
                        percent = 100;
                    }
                    if (!logged_progress_complete &&
                        (percent >= (last_logged_percent + 10) || percent == 100))
                    {
                        ESP_LOGI(TAG,
                            "OTA download progress: %d%% (%d / %d bytes)",
                            percent,
                            image_len_read,
                            image_size);
                        last_logged_percent = percent;
                        if (percent == 100)
                        {
                            logged_progress_complete = true;
                        }
                    }
                    if (percent >= (last_reported_percent + UI_OTA_PROGRESS_PERCENT_STEP) || percent == 100)
                    {
                        snprintf(progress_msg,
                            sizeof(progress_msg),
                            "Downloading firmware... %d%% (%d / %d KB)",
                            percent,
                            image_len_read / 1024,
                            image_size / 1024);
                        ui_queue_deferred_message("OTA Update", progress_msg, UI_COLOR_WARNING, false);
                        last_reported_percent = percent;
                        last_waiting_status_ms = now_ms;
                    }
                }
                else
                {
                    int downloaded_kb = image_len_read / 1024;
                    if (downloaded_kb >= (last_reported_kb + UI_OTA_PROGRESS_KB_STEP))
                    {
                        ESP_LOGI(TAG,
                            "OTA download progress: %d.%d MB received",
                            downloaded_kb / 1024,
                            ((downloaded_kb % 1024) * 10) / 1024);
                        snprintf(progress_msg,
                            sizeof(progress_msg),
                            "Downloading firmware... %d.%d MB received",
                            downloaded_kb / 1024,
                            ((downloaded_kb % 1024) * 10) / 1024);
                        ui_queue_deferred_message("OTA Update", progress_msg, UI_COLOR_WARNING, false);
                        last_reported_kb = downloaded_kb;
                        last_waiting_status_ms = now_ms;
                    }
                }
            }

            if (ret != ESP_ERR_HTTPS_OTA_IN_PROGRESS)
            {
                break;
            }
        }

        if (ret == ESP_OK)
        {
            if (!esp_https_ota_is_complete_data_received(ota_handle))
            {
                ret = ESP_FAIL;
            }
            else
            {
                ret = esp_https_ota_finish(ota_handle);
                ota_handle = NULL;
            }
        }
    }

    if (ota_handle != NULL)
    {
        final_image_len_read = esp_https_ota_get_image_len_read(ota_handle);
        final_image_size = esp_https_ota_get_image_size(ota_handle);
        esp_https_ota_abort(ota_handle);
        ota_handle = NULL;
    }

    if (ps_mode_changed)
    {
        esp_err_t restore_ret = esp_wifi_set_ps(previous_ps_mode);
        if (restore_ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Restored Wi-Fi power save mode after OTA to %d", (int)previous_ps_mode);
        }
        else
        {
            ESP_LOGW(TAG, "Failed to restore Wi-Fi power save mode after OTA: %s", esp_err_to_name(restore_ret));
        }
    }

    if (update_partition != NULL)
    {
        if (source_metadata.available && source_metadata.size_bytes > 0)
        {
            bytes_hashed = source_metadata.size_bytes;
        }
        else if (final_image_len_read > 0)
        {
            bytes_hashed = final_image_len_read;
        }

        if (bytes_hashed > 0 && (size_t)bytes_hashed <= update_partition->size)
        {
            downloaded_hash_available = ui_compute_partition_sha256_hex(update_partition,
                (size_t)bytes_hashed,
                downloaded_sha256,
                sizeof(downloaded_sha256));
            if (downloaded_hash_available)
            {
                ESP_LOGI(TAG,
                    "OTA partition image: size=%d bytes sha256=%s location=%s@0x%lx",
                    bytes_hashed,
                    downloaded_sha256,
                    update_partition->label,
                    (unsigned long)update_partition->address);

                if (source_metadata.available)
                {
                    ESP_LOGI(TAG,
                        "OTA source vs downloaded partition: source_size=%d downloaded_size=%d source_sha256=%s downloaded_sha256=%s result=%s",
                        source_metadata.size_bytes,
                        bytes_hashed,
                        source_metadata.sha256,
                        downloaded_sha256,
                        (source_metadata.size_bytes == bytes_hashed && strcmp(source_metadata.sha256, downloaded_sha256) == 0) ? "MATCH" : "MISMATCH");
                }
            }
        }
    }

    s_ota_update_in_progress = false;
    if (aborted_for_wifi)
    {
        snprintf(msg,
            sizeof(msg),
            "Wi-Fi disconnected during OTA. Downloaded %d bytes%s%s.",
            final_image_len_read,
            final_image_size > 0 ? " of " : "",
            final_image_size > 0 ? "" : "");
        if (final_image_size > 0)
        {
            size_t msg_len = strlen(msg);
            snprintf(msg + msg_len,
                sizeof(msg) - msg_len,
                "%d total bytes",
                final_image_size);
        }
        ui_queue_deferred_message("OTA Update", msg, UI_COLOR_WARNING, false);
    }
    else if (ret == ESP_OK)
    {
        snprintf(msg,
            sizeof(msg),
            "Firmware update installed successfully. Downloaded %d bytes%s%s. RallyBox will reboot now.",
            final_image_len_read,
            final_image_size > 0 ? " of " : "",
            final_image_size > 0 ? "" : "");
        if (final_image_size > 0)
        {
            size_t msg_len = strlen(msg);
            snprintf(msg + msg_len,
                sizeof(msg) - msg_len,
                "%d total bytes",
                final_image_size);
        }
        ui_queue_deferred_message("OTA Update", msg, UI_COLOR_SUCCESS, true);
    }
    else
    {
        if (ret == ESP_ERR_OTA_VALIDATE_FAILED)
        {
            snprintf(msg,
                sizeof(msg),
                "Firmware download completed, but image verification failed. Downloaded %d bytes%s%s. The OTA file on S3 is corrupted, incomplete, or not a valid RallyBox firmware image.",
                final_image_len_read,
                final_image_size > 0 ? " of " : "",
                final_image_size > 0 ? "" : "");
            if (final_image_size > 0)
            {
                size_t msg_len = strlen(msg);
                snprintf(msg + msg_len,
                    sizeof(msg) - msg_len,
                    "%d total bytes",
                    final_image_size);
            }
        }
        else
        {
            snprintf(msg,
                sizeof(msg),
                "Firmware update failed: %s. Downloaded %d bytes%s%s.",
                esp_err_to_name(ret),
                final_image_len_read,
                final_image_size > 0 ? " of " : "",
                final_image_size > 0 ? "" : "");
            if (final_image_size > 0)
            {
                size_t msg_len = strlen(msg);
                snprintf(msg + msg_len,
                    sizeof(msg) - msg_len,
                    "%d total bytes",
                    final_image_size);
            }
        }
        ui_queue_deferred_message("OTA Update", msg, UI_COLOR_DANGER, false);
    }

    vTaskDelete(NULL);
}

static void action_settings_ota_update(lv_event_t* e)
{
    char ota_url[512];
    BaseType_t task_created;
    system_status_t status = system_monitor_get_status();

    LV_UNUSED(e);

    if (s_ota_update_in_progress)
    {
        ui_set_ota_status("OTA already running. Wait for completion or failure.", UI_COLOR_WARNING);
        return;
    }
    if (!ui_is_wifi_ready_for_ota())
    {
        ui_set_ota_status("Connect to Wi-Fi before starting an OTA update.", UI_COLOR_WARNING);
        return;
    }
#if CONFIG_RALLYBOX_RACEBOX_ENABLED
    if (status.racebox_connected)
    {
        ui_set_ota_status(
            "Disconnect RaceBox BLE before starting OTA. The current BLE transport conflicts with OTA memory usage.",
            UI_COLOR_WARNING);
        return;
    }
    if (status.racebox_initialized)
    {
        ui_set_ota_status(
            "OTA currently requires the Bluetooth/RaceBox BLE stack to remain unused. Reboot the device and start OTA before opening the Bluetooth page.",
            UI_COLOR_WARNING);
        return;
    }
#endif
    if (!ui_build_ota_url(ota_url, sizeof(ota_url)))
    {
        ui_set_ota_status(
            "OTA S3 settings are incomplete. Configure the bucket, region, and firmware filename in menuconfig.",
            UI_COLOR_WARNING);
        return;
    }

    s_ota_update_in_progress = true;
    ui_set_ota_status("Starting OTA download...", UI_COLOR_WARNING);
    ui_refresh_ota_button_state();

    task_created = xTaskCreatePinnedToCore(ui_ota_update_task, "ota_update", 16384, NULL, 4, NULL, 1);
    if (task_created != pdPASS)
    {
        s_ota_update_in_progress = false;
        ui_refresh_ota_button_state();
        ui_set_ota_status("Unable to start the OTA task.", UI_COLOR_DANGER);
    }
}

static void ui_style_checkbox(lv_obj_t* checkbox)
{
    if (!checkbox)
    {
        return;
    }

    lv_obj_set_style_text_color(checkbox, lv_color_hex(UI_COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(checkbox, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_set_style_pad_column(checkbox, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(checkbox, 8, LV_PART_MAIN);
    lv_obj_set_style_bg_color(checkbox, lv_color_hex(UI_COLOR_SURFACE), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(checkbox, LV_OPA_COVER, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(checkbox, lv_color_hex(UI_COLOR_BORDER), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(checkbox, 2, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(checkbox, 6, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(checkbox, lv_color_hex(UI_COLOR_ACCENT), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_border_color(checkbox, lv_color_hex(UI_COLOR_ACCENT), LV_PART_INDICATOR | LV_STATE_CHECKED);
}

static void ui_refresh_settings_controls(void)
{
    gps_points_filter_config_t config = { 0 };
    char build_text[96];
    char rallybox_id[32];
    char stationary_hint[160];
    char threshold_text[24];

    ui_get_rallybox_id(rallybox_id, sizeof(rallybox_id));
    if (s_settings_rallybox_id_value)
    {
        lv_label_set_text(s_settings_rallybox_id_value, rallybox_id);
    }
    if (s_settings_firmware_version_value)
    {
        lv_label_set_text(s_settings_firmware_version_value, ui_get_firmware_version());
    }
    if (s_settings_build_value)
    {
        snprintf(build_text, sizeof(build_text), "Build %s", ui_get_build_timestamp());
        lv_label_set_text(s_settings_build_value, build_text);
    }
    if (s_settings_ota_status_value)
    {
        lv_label_set_text(s_settings_ota_status_value, s_settings_ota_status_text);
        lv_obj_set_style_text_color(s_settings_ota_status_value, lv_color_hex(s_settings_ota_status_color), 0);
    }
    ui_refresh_ota_progress_widgets();
    ui_refresh_ota_button_state();

    if (!gps_points_get_filter_config(&config))
    {
        return;
    }

    s_settings_stationary_radius_m = config.stationary_radius_m;
    snprintf(stationary_hint,
        sizeof(stationary_hint),
        "Drops new points that stay within %u m horizontally and 3.0 m vertically of the last stored point.",
        (unsigned)s_settings_stationary_radius_m);
    snprintf(threshold_text, sizeof(threshold_text), "%u m", (unsigned)s_settings_stationary_radius_m);

    s_settings_ui_syncing = true;
    if (s_settings_filter_stationary_checkbox)
    {
        if (config.filter_stationary_points)
        {
            lv_obj_add_state(s_settings_filter_stationary_checkbox, LV_STATE_CHECKED);
        }
        else
        {
            lv_obj_clear_state(s_settings_filter_stationary_checkbox, LV_STATE_CHECKED);
        }
    }
    if (s_settings_filter_impossible_checkbox)
    {
        if (config.filter_impossible_values)
        {
            lv_obj_add_state(s_settings_filter_impossible_checkbox, LV_STATE_CHECKED);
        }
        else
        {
            lv_obj_clear_state(s_settings_filter_impossible_checkbox, LV_STATE_CHECKED);
        }
    }
    if (s_settings_stationary_hint_label)
    {
        lv_label_set_text(s_settings_stationary_hint_label, stationary_hint);
    }
    if (s_settings_impossible_hint_label)
    {
        lv_label_set_text(s_settings_impossible_hint_label,
            "Accepts latitude [-90..90], longitude [-180..180], UTC time [2000-01-01..2100-01-01], elevation [-1000..20000] m.");
    }
    if (s_settings_stationary_radius_value)
    {
        lv_label_set_text(s_settings_stationary_radius_value, threshold_text);
    }
    s_settings_ui_syncing = false;
}

static void ui_settings_filter_checkbox_cb(lv_event_t* e)
{
    gps_points_filter_config_t config;

    LV_UNUSED(e);
    if (s_settings_ui_syncing)
    {
        return;
    }
    if (s_settings_filter_stationary_checkbox == NULL || s_settings_filter_impossible_checkbox == NULL)
    {
        return;
    }

    config.filter_stationary_points = lv_obj_has_state(s_settings_filter_stationary_checkbox, LV_STATE_CHECKED);
    config.filter_impossible_values = lv_obj_has_state(s_settings_filter_impossible_checkbox, LV_STATE_CHECKED);
    config.stationary_radius_m = s_settings_stationary_radius_m;

    if (gps_points_set_filter_config(&config) != ESP_OK)
    {
        ui_refresh_settings_controls();
        ui_show_message("Settings", "Failed to save GPS filter settings.", lv_color_hex(UI_COLOR_DANGER));
    }
    else
    {
        ui_refresh_settings_controls();
    }
}

static void ui_settings_stationary_threshold_cb(lv_event_t* e)
{
    gps_points_filter_config_t config;
    intptr_t delta = (intptr_t)lv_event_get_user_data(e);
    int next_value;

    if (s_settings_ui_syncing)
    {
        return;
    }
    if (!gps_points_get_filter_config(&config))
    {
        ui_show_message("Settings", "Failed to load GPS filter settings.", lv_color_hex(UI_COLOR_DANGER));
        return;
    }

    next_value = (int)config.stationary_radius_m + (int)delta;
    if (next_value < 1)
    {
        next_value = 1;
    }
    else if (next_value > 20)
    {
        next_value = 20;
    }

    config.stationary_radius_m = (uint8_t)next_value;
    if (gps_points_set_filter_config(&config) != ESP_OK)
    {
        ui_refresh_settings_controls();
        ui_show_message("Settings", "Failed to save stationary filter threshold.", lv_color_hex(UI_COLOR_DANGER));
        return;
    }

    ui_refresh_settings_controls();
}

lv_obj_t* create_settings_screen(lv_obj_t* parent)
{
    lv_obj_t* cont = lv_obj_create(parent);
    lv_obj_t* header;
    lv_obj_t* title;
    lv_obj_t* subtitle;
    lv_obj_t* content;
    lv_obj_t* device_card;
    lv_obj_t* filter_card;
    lv_obj_t* fw_info;
    lv_obj_t* fw_row;
    lv_obj_t* label;
    lv_obj_t* hint;
    lv_obj_t* threshold_row;
    lv_obj_t* threshold_minus_btn;
    lv_obj_t* threshold_plus_btn;
    lv_obj_t* threshold_label;

    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    ui_style_surface(cont, lv_color_hex(UI_COLOR_BG), 0);
    lv_obj_set_style_pad_all(cont, 0, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_radius(cont, 0, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);

    header = lv_obj_create(cont);
    lv_obj_set_size(header, LV_PCT(100), 68);
    ui_style_surface(header, lv_color_hex(UI_COLOR_SURFACE), 0);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_style_pad_left(header, 18, 0);
    lv_obj_set_style_pad_right(header, 18, 0);
    lv_obj_set_style_pad_top(header, 12, 0);
    lv_obj_set_style_pad_bottom(header, 8, 0);

    title = lv_label_create(header);
    lv_label_set_text(title, "Settings");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(UI_COLOR_TEXT), 0);

    subtitle = lv_label_create(header);
    lv_label_set_text(subtitle, "Persistent GPS filters and device identity.");
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(subtitle, lv_color_hex(UI_COLOR_MUTED), 0);
    lv_obj_align(subtitle, LV_ALIGN_BOTTOM_LEFT, 0, 0);

    content = lv_obj_create(cont);
    lv_obj_set_size(content, LV_PCT(100), LV_PCT(100));
    ui_style_surface(content, lv_color_hex(UI_COLOR_BG), 0);
    lv_obj_set_style_border_width(content, 0, 0);
    lv_obj_set_style_pad_left(content, 18, 0);
    lv_obj_set_style_pad_right(content, 18, 0);
    lv_obj_set_style_pad_top(content, 12, 0);
    lv_obj_set_style_pad_bottom(content, 18, 0);
    lv_obj_set_style_pad_row(content, 14, 0);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    device_card = lv_obj_create(content);
    lv_obj_set_size(device_card, LV_PCT(100), 272);
    ui_style_surface(device_card, lv_color_hex(UI_COLOR_SURFACE), 18);
    lv_obj_set_style_pad_all(device_card, 18, 0);
    lv_obj_set_style_pad_row(device_card, 8, 0);
    lv_obj_set_flex_flow(device_card, LV_FLEX_FLOW_COLUMN);

    label = lv_label_create(device_card);
    lv_label_set_text(label, "RallyBoxID");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(UI_COLOR_TEXT), 0);

    s_settings_rallybox_id_value = lv_label_create(device_card);
    lv_label_set_text(s_settings_rallybox_id_value, "Loading...");
    lv_obj_set_style_text_font(s_settings_rallybox_id_value, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(s_settings_rallybox_id_value, lv_color_hex(UI_COLOR_ACCENT), 0);

    hint = lv_label_create(device_card);
    lv_obj_set_width(hint, LV_PCT(100));
    lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
    lv_label_set_text(hint, "Derived from the ESP32-P4 base MAC address. Stable across reboots.");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(UI_COLOR_MUTED), 0);

    label = lv_label_create(device_card);
    lv_label_set_text(label, "Firmware Version");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(UI_COLOR_TEXT), 0);

    fw_row = lv_obj_create(device_card);
    lv_obj_set_size(fw_row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(fw_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(fw_row, 0, 0);
    lv_obj_set_style_pad_all(fw_row, 0, 0);
    lv_obj_set_style_pad_column(fw_row, 14, 0);
    lv_obj_set_flex_flow(fw_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(fw_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    fw_info = lv_obj_create(fw_row);
    lv_obj_set_size(fw_info, 0, LV_SIZE_CONTENT);
    lv_obj_set_flex_grow(fw_info, 1);
    lv_obj_set_style_bg_opa(fw_info, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(fw_info, 0, 0);
    lv_obj_set_style_pad_all(fw_info, 0, 0);
    lv_obj_set_style_pad_row(fw_info, 4, 0);
    lv_obj_set_flex_flow(fw_info, LV_FLEX_FLOW_COLUMN);

    s_settings_firmware_version_value = lv_label_create(fw_info);
    lv_label_set_text(s_settings_firmware_version_value, "Loading...");
    lv_obj_set_style_text_font(s_settings_firmware_version_value, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_settings_firmware_version_value, lv_color_hex(UI_COLOR_ACCENT), 0);

    s_settings_build_value = lv_label_create(fw_info);
    lv_label_set_text(s_settings_build_value, "Build --");
    lv_obj_set_style_text_font(s_settings_build_value, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_settings_build_value, lv_color_hex(UI_COLOR_MUTED), 0);

    s_settings_ota_button = lv_button_create(fw_row);
    lv_obj_set_size(s_settings_ota_button, 170, 48);
    lv_label_set_text(lv_label_create(s_settings_ota_button), "Update OTA");
    ui_style_button(s_settings_ota_button, lv_color_hex(UI_COLOR_ACCENT), lv_color_hex(UI_COLOR_TEXT));
    lv_obj_add_event_cb(s_settings_ota_button, action_settings_ota_update, LV_EVENT_CLICKED, NULL);

    s_settings_ota_progress_row = lv_obj_create(device_card);
    lv_obj_set_width(s_settings_ota_progress_row, LV_PCT(100));
    lv_obj_set_height(s_settings_ota_progress_row, 28);
    lv_obj_set_style_bg_opa(s_settings_ota_progress_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_settings_ota_progress_row, 0, 0);
    lv_obj_set_style_pad_all(s_settings_ota_progress_row, 0, 0);
    lv_obj_set_style_pad_column(s_settings_ota_progress_row, 10, 0);
    lv_obj_set_flex_flow(s_settings_ota_progress_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(s_settings_ota_progress_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    s_settings_ota_progress_bar = lv_bar_create(s_settings_ota_progress_row);
    lv_obj_set_size(s_settings_ota_progress_bar, 420, 10);
    lv_bar_set_range(s_settings_ota_progress_bar, 0, 100);
    lv_bar_set_value(s_settings_ota_progress_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_settings_ota_progress_bar, lv_color_hex(UI_COLOR_BORDER), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_settings_ota_progress_bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s_settings_ota_progress_bar, 6, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_settings_ota_progress_bar, lv_color_hex(UI_COLOR_ACCENT), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(s_settings_ota_progress_bar, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_settings_ota_progress_bar, 6, LV_PART_INDICATOR);

    s_settings_ota_progress_value = lv_label_create(s_settings_ota_progress_row);
    lv_obj_set_width(s_settings_ota_progress_value, 72);
    lv_label_set_text(s_settings_ota_progress_value, s_settings_ota_progress_value_text);
    lv_label_set_long_mode(s_settings_ota_progress_value, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(s_settings_ota_progress_value, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(s_settings_ota_progress_value, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_settings_ota_progress_value, lv_color_hex(UI_COLOR_MUTED), 0);

    s_settings_ota_status_value = lv_label_create(device_card);
    lv_obj_set_width(s_settings_ota_status_value, LV_PCT(100));
    lv_obj_set_height(s_settings_ota_status_value, 22);
    lv_label_set_long_mode(s_settings_ota_status_value, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_settings_ota_status_value, s_settings_ota_status_text);
    lv_obj_set_style_text_font(s_settings_ota_status_value, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_settings_ota_status_value, lv_color_hex(s_settings_ota_status_color), 0);

    ui_refresh_ota_progress_widgets();

    hint = lv_label_create(device_card);
    lv_obj_set_width(hint, LV_PCT(100));
    lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
    lv_label_set_text(hint, "OTA strategy: update the root VERSION file, rebuild, upload firmware/RallyBox-Dashboard/RallyBox-Dashboard.bin plus the manifest JSON to S3, then trigger Update OTA on the device.");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(UI_COLOR_MUTED), 0);

    filter_card = lv_obj_create(content);
    lv_obj_set_size(filter_card, LV_PCT(100), 308);
    ui_style_surface(filter_card, lv_color_hex(UI_COLOR_SURFACE), 18);
    lv_obj_set_style_pad_all(filter_card, 18, 0);
    lv_obj_set_style_pad_row(filter_card, 10, 0);
    lv_obj_set_flex_flow(filter_card, LV_FLEX_FLOW_COLUMN);

    label = lv_label_create(filter_card);
    lv_label_set_text(label, "GPS Point Filters");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(UI_COLOR_TEXT), 0);

    hint = lv_label_create(filter_card);
    lv_obj_set_width(hint, LV_PCT(100));
    lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
    lv_label_set_text(hint, "These options apply to both BLE and GNSS feeds and are saved permanently.");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(UI_COLOR_MUTED), 0);

    s_settings_filter_stationary_checkbox = lv_checkbox_create(filter_card);
    lv_checkbox_set_text(s_settings_filter_stationary_checkbox,
        "Filter stationary GPS points");
    ui_style_checkbox(s_settings_filter_stationary_checkbox);
    lv_obj_add_event_cb(s_settings_filter_stationary_checkbox, ui_settings_filter_checkbox_cb, LV_EVENT_VALUE_CHANGED, NULL);

    s_settings_stationary_hint_label = lv_label_create(filter_card);
    lv_obj_set_width(s_settings_stationary_hint_label, LV_PCT(100));
    lv_label_set_long_mode(s_settings_stationary_hint_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(s_settings_stationary_hint_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_settings_stationary_hint_label, lv_color_hex(UI_COLOR_MUTED), 0);

    threshold_row = lv_obj_create(filter_card);
    lv_obj_set_size(threshold_row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(threshold_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(threshold_row, 0, 0);
    lv_obj_set_style_pad_all(threshold_row, 0, 0);
    lv_obj_set_style_pad_column(threshold_row, 10, 0);
    lv_obj_set_flex_flow(threshold_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(threshold_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    threshold_label = lv_label_create(threshold_row);
    lv_label_set_text(threshold_label, "Stationary radius");
    lv_obj_set_style_text_font(threshold_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(threshold_label, lv_color_hex(UI_COLOR_TEXT), 0);

    threshold_minus_btn = lv_button_create(threshold_row);
    lv_obj_set_size(threshold_minus_btn, 52, 42);
    lv_label_set_text(lv_label_create(threshold_minus_btn), "-");
    ui_style_button(threshold_minus_btn, lv_color_hex(UI_COLOR_SURFACE_ALT), lv_color_hex(UI_COLOR_TEXT));
    lv_obj_add_event_cb(threshold_minus_btn, ui_settings_stationary_threshold_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);

    s_settings_stationary_radius_value = lv_label_create(threshold_row);
    lv_label_set_text(s_settings_stationary_radius_value, "3 m");
    lv_obj_set_style_text_font(s_settings_stationary_radius_value, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(s_settings_stationary_radius_value, lv_color_hex(UI_COLOR_ACCENT), 0);

    threshold_plus_btn = lv_button_create(threshold_row);
    lv_obj_set_size(threshold_plus_btn, 52, 42);
    lv_label_set_text(lv_label_create(threshold_plus_btn), "+");
    ui_style_button(threshold_plus_btn, lv_color_hex(UI_COLOR_SURFACE_ALT), lv_color_hex(UI_COLOR_TEXT));
    lv_obj_add_event_cb(threshold_plus_btn, ui_settings_stationary_threshold_cb, LV_EVENT_CLICKED, (void*)(intptr_t)1);

    s_settings_filter_impossible_checkbox = lv_checkbox_create(filter_card);
    lv_checkbox_set_text(s_settings_filter_impossible_checkbox,
        "Filter impossible latitude, longitude, time, or elevation values");
    ui_style_checkbox(s_settings_filter_impossible_checkbox);
    lv_obj_add_event_cb(s_settings_filter_impossible_checkbox, ui_settings_filter_checkbox_cb, LV_EVENT_VALUE_CHANGED, NULL);

    s_settings_impossible_hint_label = lv_label_create(filter_card);
    lv_obj_set_width(s_settings_impossible_hint_label, LV_PCT(100));
    lv_label_set_long_mode(s_settings_impossible_hint_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(s_settings_impossible_hint_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_settings_impossible_hint_label, lv_color_hex(UI_COLOR_MUTED), 0);

    ui_refresh_settings_controls();
    return cont;
}

/**
 * @brief Simple minimal keyboard for file search
 */
lv_obj_t* create_minimal_keyboard(lv_obj_t* parent)
{
    lv_obj_t* kb = lv_keyboard_create(parent);
    lv_obj_set_size(kb, LV_PCT(100), 150);
    ui_style_keyboard(kb);
    lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_TEXT_LOWER);

    return kb;
}

static lv_obj_t* ui_get_first_child(lv_obj_t* obj)
{
    if (obj == NULL || lv_obj_get_child_count(obj) == 0)
    {
        return NULL;
    }
    return lv_obj_get_child(obj, 0);
}

static void ui_set_wifi_button_style(lv_obj_t* button, lv_color_t base, lv_color_t accent, const char* text)
{
    lv_obj_t* label = ui_get_first_child(button);

    if (button == NULL)
    {
        return;
    }

    lv_obj_set_style_bg_color(button, base, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(button, accent, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(button, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(button, lv_color_hex(UI_COLOR_BORDER), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(button, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(button, 14, LV_PART_MAIN | LV_STATE_DEFAULT);

    if (label != NULL)
    {
        lv_label_set_text(label, text);
        lv_obj_set_style_text_color(label, lv_color_hex(UI_COLOR_TEXT), 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_18, 0);
}
    }







static void ui_refresh_wifi_controls(const system_status_t* status)
{
    static int last_wifi_state = -1;
    static char last_connection_text[24] = "";
    static uint32_t last_connection_color = UINT32_MAX;
    static char last_detail_text[128] = "";
    static uint32_t last_detail_color = UINT32_MAX;
    static char last_button_text[16] = "";
    static int last_button_state = -1;
    char detail_text[128];
    const char* connection_text = "Starting...";
    uint32_t connection_color = 0xff80d8ff;
    uint32_t detail_color = 0xff80d8ff;
    lv_color_t button_base = lv_color_hex(0xff546e7a);
    lv_color_t button_accent = lv_color_hex(0xff37474f);
    const char* button_text = "CONNECT";
    int wifi_state;

    if (status == NULL)
    {
        return;
    }

    s_wifi_connected = (status->wifi_state == SYSTEM_WIFI_STATE_CONNECTED);
    snprintf(s_wifi_ip, sizeof(s_wifi_ip), "%s", status->wifi_ip);
    wifi_state = (int)status->wifi_state;

    switch ((system_wifi_state_t)status->wifi_state)
    {
        case SYSTEM_WIFI_STATE_CONNECTED:
            connection_text = "Connected";
            connection_color = 0xff0ad757;
            snprintf(detail_text, sizeof(detail_text), "Connected to %s | IP %s | RSSI %d dBm | Tap DISCONNECT, long-press to FORGET",
                status->wifi_ssid[0] ? status->wifi_ssid : "saved network",
                status->wifi_ip,
                status->wifi_rssi);
            detail_color = 0xff0ad757;
            button_base = lv_color_hex(0xffd84343);
            button_accent = lv_color_hex(0xff8b0000);
            button_text = "DISCONNECT";
            break;
        case SYSTEM_WIFI_STATE_CONNECTING:
            connection_text = "Connecting...";
            connection_color = 0xffffc107;
            snprintf(detail_text, sizeof(detail_text), "Connecting to %s | Searching all channels | Attempt %lu/%lu",
                status->wifi_ssid[0] ? status->wifi_ssid : "saved network",
                (unsigned long)(status->wifi_retry_count + 1),
                (unsigned long)(status->wifi_max_retries + 1));
            detail_color = 0xffffc107;
            button_base = lv_color_hex(0xffffb300);
            button_accent = lv_color_hex(0xffff6f00);
            button_text = "CONNECTING";
            break;
        case SYSTEM_WIFI_STATE_DISCONNECTED:
            connection_text = "Disconnected";
            connection_color = 0xffff6b6b;
            snprintf(detail_text, sizeof(detail_text), "%s | Tap CONNECT to connect, long-press CONNECT to forget saved",
                status->wifi_last_error[0] ? status->wifi_last_error : "Not connected. Enter SSID and password");
            detail_color = 0xffff8a80;
            button_base = lv_color_hex(0xff00acc1);
            button_accent = lv_color_hex(0xff1565c0);
            button_text = status->wifi_connect_attempts > 0 ? "RETRY" : "CONNECT";
            break;
        case SYSTEM_WIFI_STATE_STARTING:
        default:
            snprintf(detail_text, sizeof(detail_text), "Initializing WiFi backend...");
            button_base = lv_color_hex(0xff546e7a);
            button_accent = lv_color_hex(0xff37474f);
            button_text = "CONNECT";
            break;
    }

    if (objects.wifi_connection &&
        (strcmp(last_connection_text, connection_text) != 0 || last_connection_color != connection_color))
    {
        lv_label_set_text(objects.wifi_connection, connection_text);
        lv_obj_set_style_text_color(objects.wifi_connection, lv_color_hex(connection_color), 0);
        snprintf(last_connection_text, sizeof(last_connection_text), "%s", connection_text);
        last_connection_color = connection_color;
    }

    if (objects.obj4 &&
        (strcmp(last_detail_text, detail_text) != 0 || last_detail_color != detail_color))
    {
        lv_label_set_text(objects.obj4, detail_text);
        lv_obj_set_style_text_color(objects.obj4, lv_color_hex(detail_color), 0);
        snprintf(last_detail_text, sizeof(last_detail_text), "%s", detail_text);
        last_detail_color = detail_color;
}

    if (objects.wifi_conenct_button_1 &&
        (last_button_state != wifi_state || strcmp(last_button_text, button_text) != 0))
    {
        ui_set_wifi_button_style(objects.wifi_conenct_button_1, button_base, button_accent, button_text);
        last_button_state = wifi_state;
        snprintf(last_button_text, sizeof(last_button_text), "%s", button_text);
    }

    last_wifi_state = wifi_state;
    LV_UNUSED(last_wifi_state);
}

static bool ui_is_bluetooth_tab_active(void)
{
    if (objects.obj0 == NULL)
    {
        return false;
    }

    return lv_tabview_get_tab_active(objects.obj0) == BLUETOOTH_TAB_INDEX;
}

static bool ui_is_gnss_tab_active(void)
{
    if (objects.obj0 == NULL)
    {
        return false;
    }

    return lv_tabview_get_tab_active(objects.obj0) == GNSS_TAB_INDEX;
}

static bool ui_is_files_tab_active(void)
{
    if (objects.obj0 == NULL || g_files_tab == NULL)
    {
        return false;
    }

    return lv_tabview_get_tab_active(objects.obj0) == lv_obj_get_index(g_files_tab);
}

static bool ui_is_settings_tab_active(void)
{
    if (objects.obj0 == NULL || g_settings_tab == NULL)
    {
        return false;
    }

    return lv_tabview_get_tab_active(objects.obj0) == lv_obj_get_index(g_settings_tab);
}

static void ui_refresh_bluetooth_dropdown(bool force)
{
#if CONFIG_RALLYBOX_RACEBOX_ENABLED
    size_t i;
    size_t offset = 0;
    char options[1536] = { 0 };
    uint64_t now_us = (uint64_t)esp_timer_get_time();

    if (objects.bluetooth_device_dropdown == NULL)
    {
        return;
    }

    if (!force)
    {
        if (!ui_is_bluetooth_tab_active())
        {
            return;
        }

        if ((now_us - s_bt_last_dropdown_refresh_us) < (BLUETOOTH_DROPDOWN_REFRESH_MS * 1000ULL))
        {
            return;
        }
}

    s_bt_last_dropdown_refresh_us = now_us;

    s_bt_visible_count = racebox_get_visible_devices(s_bt_visible_devices, RACEBOX_VISIBLE_DEVICES_MAX);
    if (s_bt_visible_count == 0)
    {
        if (strcmp(s_bt_last_options, "No visible BLE devices") != 0)
        {
            lv_dropdown_set_options(objects.bluetooth_device_dropdown, "No visible BLE devices");
            lv_dropdown_set_selected(objects.bluetooth_device_dropdown, 0);
            snprintf(s_bt_last_options, sizeof(s_bt_last_options), "%s", "No visible BLE devices");
        }
        return;
    }

    for (i = 0; i < s_bt_visible_count; ++i)
    {
        int written = snprintf(options + offset, sizeof(options) - offset,
            "%s | %s | %d dBm%s",
            s_bt_visible_devices[i].name,
            s_bt_visible_devices[i].address,
            (int)s_bt_visible_devices[i].rssi,
            (i + 1 < s_bt_visible_count) ? "\n" : "");
        if (written < 0)
        {
            break;
        }
        if ((size_t)written >= (sizeof(options) - offset))
        {
            break;
        }
        offset += (size_t)written;
    }

    if (strcmp(s_bt_last_options, options) != 0)
    {
        lv_dropdown_set_options(objects.bluetooth_device_dropdown, options);
        lv_dropdown_set_selected(objects.bluetooth_device_dropdown, 0);
        snprintf(s_bt_last_options, sizeof(s_bt_last_options), "%s", options);
    }
#else
    if (objects.bluetooth_device_dropdown)
    {
        lv_dropdown_set_options(objects.bluetooth_device_dropdown, "RaceBox BLE disabled in menuconfig");
        lv_dropdown_set_selected(objects.bluetooth_device_dropdown, 0);
    }
#endif
}

static void ui_refresh_bluetooth_controls(const system_status_t* status)
{
    char text[192];

    if (objects.bluetooth_status_label == NULL)
    {
        return;
    }

#if CONFIG_RALLYBOX_RACEBOX_ENABLED
    if (status == NULL)
    {
        lv_label_set_text(objects.bluetooth_status_label, "Bluetooth status unavailable");
        return;
    }

    if (status->racebox_connected)
    {
        snprintf(text, sizeof(text), "Connected to %s | RSSI %d dBm | %s",
            status->racebox_device_name[0] ? status->racebox_device_name : "RaceBox",
            (int)status->racebox_rssi,
            status->racebox_status_text[0] ? status->racebox_status_text : "Connected");
        lv_obj_set_style_text_color(objects.bluetooth_status_label, lv_color_hex(UI_COLOR_SUCCESS), 0);
        if (objects.bluetooth_connect_button)
        {
            ui_set_wifi_button_style(objects.bluetooth_connect_button,
                lv_color_hex(0xffd84343),
                lv_color_hex(0xff8b0000),
                "DISCONNECT");
        }
    }
    else if (status->racebox_initialized)
    {
        snprintf(text, sizeof(text), "%s",
            status->racebox_status_text[0] ? status->racebox_status_text : "Scanning for BLE devices");
        lv_obj_set_style_text_color(objects.bluetooth_status_label, lv_color_hex(UI_COLOR_WARNING), 0);
        if (objects.bluetooth_connect_button)
        {
            ui_set_wifi_button_style(objects.bluetooth_connect_button,
                lv_color_hex(0xff00acc1),
                lv_color_hex(0xff1565c0),
                "CONNECT");
        }
    }
    else
    {
        snprintf(text, sizeof(text), "%s",
            status->racebox_status_text[0] ? status->racebox_status_text : "RaceBox BLE not initialized");
        lv_obj_set_style_text_color(objects.bluetooth_status_label, lv_color_hex(UI_COLOR_DANGER), 0);
        if (objects.bluetooth_connect_button)
        {
            ui_set_wifi_button_style(objects.bluetooth_connect_button,
                lv_color_hex(0xff00acc1),
                lv_color_hex(0xff1565c0),
                "CONNECT");
        }
    }

    if (status->racebox_connected && !s_bt_prev_connected)
    {
        char line[160];
        gps_points_set_feed_active(GPS_POINTS_FEED_BLE, true);
        ui_stream_start(&s_bt_stream);
        s_bt_rx_count = 0;
        s_last_bt_count = 0;
        snprintf(line, sizeof(line), "Connected: %s | RSSI=%d dBm | %s",
            status->racebox_device_name[0] ? status->racebox_device_name : "RaceBox",
            (int)status->racebox_rssi,
            status->racebox_status_text[0] ? status->racebox_status_text : "Connected");
        ui_window_append_line(&s_bt_window, line);
    }
    else if (!status->racebox_connected && s_bt_prev_connected)
    {
        char line[160];
        if (!s_bt_suspend_preserve_feed)
        {
            gps_points_set_feed_active(GPS_POINTS_FEED_BLE, false);
        }
        snprintf(line, sizeof(line), "Disconnected: %s",
            status->racebox_status_text[0] ? status->racebox_status_text : "RaceBox disconnected");
        ui_window_append_line(&s_bt_window, line);
        ui_stream_stop(&s_bt_stream);
    }
    s_bt_prev_connected = status->racebox_connected;

    lv_label_set_text(objects.bluetooth_status_label, text);
    ui_refresh_bluetooth_dropdown(false);
#else
    lv_label_set_text(objects.bluetooth_status_label, "RaceBox BLE is disabled in menuconfig");
    lv_obj_set_style_text_color(objects.bluetooth_status_label, lv_color_hex(UI_COLOR_DANGER), 0);
    ui_refresh_bluetooth_dropdown(false);
#endif
}

static void action_bluetooth_refresh(lv_event_t* e)
{
    LV_UNUSED(e);

    ui_prepare_bluetooth_panel(true, true);
}

static void action_bluetooth_connect(lv_event_t* e)
{
    LV_UNUSED(e);

#if CONFIG_RALLYBOX_RACEBOX_ENABLED
    system_status_t status = system_monitor_get_status();
    uint32_t selected_index;
    esp_err_t ret;

    if (status.racebox_connected)
    {
        ret = racebox_disconnect();
        if (ret != ESP_OK)
        {
            ui_handle_error("Failed to disconnect BLE device", ret);
            return;
    }

        ui_window_append_line(&s_bt_window, "Disconnect requested by user");
        ui_refresh_bluetooth_dump();
        ui_show_message("Bluetooth", "Disconnecting from RaceBox...", lv_color_hex(UI_COLOR_SURFACE));
        return;
}

    if (objects.bluetooth_device_dropdown == NULL)
    {
        return;
    }

    if (s_bt_visible_count == 0)
    {
        ui_show_message("Bluetooth", "No visible device to connect. Tap Refresh first.", lv_color_hex(UI_COLOR_WARNING));
        return;
    }

    selected_index = lv_dropdown_get_selected(objects.bluetooth_device_dropdown);
    if (selected_index >= s_bt_visible_count)
    {
        ui_show_message("Bluetooth", "Selected device is no longer available.", lv_color_hex(UI_COLOR_WARNING));
        return;
    }

    ret = racebox_connect_visible_device((size_t)selected_index);
    if (ret != ESP_OK)
    {
        ui_handle_error("Failed to start BLE connection", ret);
        return;
    }

    ui_stream_start(&s_bt_stream);
    s_bt_rx_count = 0;
    s_last_bt_count = 0;
    if (s_bt_dump_panel)
    {
        ui_window_reset(&s_bt_window, s_bt_dump_panel);
    }

    {
        char line[160];
        snprintf(line, sizeof(line), "Connect requested: %s | RSSI=%d dBm",
            s_bt_visible_devices[selected_index].name,
            (int)s_bt_visible_devices[selected_index].rssi);
        ui_window_append_line(&s_bt_window, line);
    }
    ui_refresh_bluetooth_dump();

    ui_show_message("Bluetooth", "Connecting to selected BLE device...", lv_color_hex(UI_COLOR_SURFACE));
#else
    ui_show_message("Bluetooth", "RaceBox BLE is disabled in menuconfig.", lv_color_hex(UI_COLOR_DANGER));
#endif
    }

static void ui_load_saved_wifi_credentials(void)
{
    nvs_handle_t handle;
    char ssid[33] = { 0 };
    char password[65] = { 0 };
    size_t ssid_len = sizeof(ssid);
    size_t password_len = sizeof(password);

    if (objects.wifi_ssid_1 == NULL || objects.wifi_password_1 == NULL)
    {
        return;
    }

    if (nvs_open("wifi_cfg", NVS_READONLY, &handle) != ESP_OK)
    {
        lv_textarea_set_text(objects.wifi_ssid_1, "");
        lv_textarea_set_text(objects.wifi_password_1, "");
        return;
    }

    if (nvs_get_str(handle, "ssid", ssid, &ssid_len) == ESP_OK)
    {
        lv_textarea_set_text(objects.wifi_ssid_1, ssid);
    }
    else
    {
        lv_textarea_set_text(objects.wifi_ssid_1, "");
    }

    if (nvs_get_str(handle, "password", password, &password_len) == ESP_OK)
    {
        lv_textarea_set_text(objects.wifi_password_1, password);
    }
    else
    {
        lv_textarea_set_text(objects.wifi_password_1, "");
    }

    nvs_close(handle);
}

static void action_remove_saved_wifi(lv_event_t* e)
{
    esp_err_t ret = system_wifi_disconnect(true);
    if (ret != ESP_OK)
    {
        ui_handle_error("Failed to remove saved WiFi network", ret);
        return;
    }

    if (objects.wifi_ssid_1)
    {
        lv_textarea_set_text(objects.wifi_ssid_1, "");
    }
    if (objects.wifi_password_1)
    {
        lv_textarea_set_text(objects.wifi_password_1, "");
    }

    system_status_t status_copy = system_monitor_get_status();
    ui_refresh_wifi_controls(&status_copy);
    ui_show_message("Saved Network Removed",
        "Saved WiFi credentials were removed from NVS. Enter new SSID/password to connect.",
        lv_color_hex(0xff1565c0));
    (void)e;
}

/**
 * @brief Production Live Logging Logic: Writes system data to SD Card 2 (Slot 1)
 * Requirement: Create new file after 2000 lines, no crashes, real-time stable.
 */
static void ui_logic_log_to_sd(const char* data)
{
    // Prefer SD Card 1 (/sdcard) for production logs
    if (!sd_card1_is_mounted()) return;

    // Create a new timestamped file if needed
    if (current_log_filename[0] == '\0' || s_log_line_count >= 2000)
    {
        s_log_line_count = 0;
        s_log_file_index++;
        time_t now = time(NULL);
        struct tm tm;
        localtime_r(&now, &tm);
        snprintf(current_log_filename, sizeof(current_log_filename), "/sdcard/rallybox_sdcard1_%04d%02d%02d_%02d%02d%02d.txt",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
        ESP_LOGI(TAG, "Creating new log file: %s", current_log_filename);
        // Write header line
        FILE* hf = fopen(current_log_filename, "w");
        if (hf)
        {
            fprintf(hf, "RallyBox log start: %04d-%02d-%02d %02d:%02d:%02d IST\n",
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, tm.tm_sec);
            fprintf(hf, "Columns: sl_no | timestamp_IST | cpu_pct | free_heap_bytes | wifi_connected | wifi_ssid | wifi_ip | sd_slot | sd_capacity_mb | sd_used_mb\n\n");
            fclose(hf);
        }
    }

    FILE* f = fopen(current_log_filename, "a");
    if (!f)
    {
        ESP_LOGE(TAG, "Failed to open log file %s", current_log_filename);
        return;
    }

    system_status_t st = system_monitor_get_status();
    time_t tnow = time(NULL);
    struct tm tlocal;
    localtime_r(&tnow, &tlocal);
    char timestr[64];
    strftime(timestr, sizeof(timestr), "%Y-%m-%d %H:%M:%S %Z", &tlocal);

    uint64_t cap = 0, used = 0;
    sd_card_get_info(1, &cap, &used);

    s_log_line_count++;
    fprintf(f, "%u. %s | CPU: %u%% | FreeHeap: %u | WiFi: %s | SSID: %s | IP: %s | SD1: %.0lluMB total, %.0lluMB used | Msg: %s\n",
        s_log_line_count,
        timestr,
        (unsigned)st.cpu_load_percent,
        (unsigned)st.free_heap_bytes,
        st.wifi_connected ? "Connected" : "Disconnected",
        st.wifi_ssid[0] ? st.wifi_ssid : "",
        st.wifi_ip[0] ? st.wifi_ip : "",
        (unsigned long long)cap,
        (unsigned long long)used,
        data ? data : "");
    fclose(f);
}

/**
 * @brief Callback for WiFi events to update UI
 */
void ui_logic_notify_wifi_status(bool connected, const char* ip)
{
    s_wifi_connected = connected;
    if (ip)
    {
        strncpy(s_wifi_ip, ip, sizeof(s_wifi_ip) - 1);
        s_wifi_ip[sizeof(s_wifi_ip) - 1] = '\0';
    }
}

void ui_logic_update_wifi_live(const system_status_t* status)
{
    ui_process_deferred_message();

    if (status != NULL)
    {
        time_t now_utc = time(NULL);

#if CONFIG_RALLYBOX_RACEBOX_ENABLED
        racebox_nav_snapshot_t rb_nav;
        if (status->racebox_connected && racebox_get_nav_snapshot(&rb_nav) && rb_nav.solution_valid)
        {
            gps_points_append(GPS_POINTS_FEED_BLE, rb_nav.latitude_deg, rb_nav.longitude_deg, rb_nav.altitude_m, now_utc);
        }
#endif

        if (status->gnss_fix_valid)
        {
            gps_points_append(GPS_POINTS_FEED_GNSS,
                status->gnss_latitude_deg,
                status->gnss_longitude_deg,
                status->gnss_altitude_m,
                now_utc);
        }
    }

    if (!(s_ota_update_in_progress && ui_is_settings_tab_active()))
    {
        ui_refresh_wifi_controls(status);
    }
    if (status)
    {
        if (s_ota_update_in_progress && ui_is_settings_tab_active())
        {
            return;
        }

        ui_update_racebox_auto_connect(status);

        bool bt_tab_active = ui_is_bluetooth_tab_active();
        bool bt_state_changed = (status->racebox_connected != s_bt_ui_last_connected) ||
            (status->racebox_initialized != s_bt_ui_last_initialized);

        if (s_bt_auto_connect_pending || bt_tab_active || bt_state_changed)
        {
            ui_refresh_bluetooth_controls(status);
            s_bt_ui_last_connected = status->racebox_connected;
            s_bt_ui_last_initialized = status->racebox_initialized;
        }
    }
    ui_update_main_status_panels(status);

    if (ui_is_gnss_tab_active() && objects.gnss_status_label)
    {
        lv_label_set_text(objects.gnss_status_label, s_gnss_status_text);
    }
    if (ui_is_gnss_tab_active() && objects.gnss_start_stop_button)
    {
        lv_obj_t* label = ui_get_first_child(objects.gnss_start_stop_button);
        if (label)
        {
            lv_label_set_text(label, s_gnss_listening ? "STOP" : "START");
        }
    }
}

typedef struct
{
    lv_obj_t* bar;
    lv_obj_t* label;
    lv_obj_t* overlay;
    int slot;
    volatile bool done;
    esp_err_t result;
} format_progress_t;

typedef struct
{
    lv_obj_t* bar;
    lv_obj_t* label;
    lv_obj_t* overlay;
    volatile bool done;
    esp_err_t result;
} log_export_progress_t;

static void quick_format_task(void* pvParameters)
{
    format_progress_t* progress = (format_progress_t*)pvParameters;
    progress->result = sd_card_quick_format(progress->slot);
    progress->done = true;
    vTaskDelete(NULL);
}

static void log_export_task(void* pvParameters)
{
    log_export_progress_t* progress = (log_export_progress_t*)pvParameters;

    if (progress == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

    progress->result = sd_card_export_logs();
    progress->done = true;
    vTaskDelete(NULL);
}

static void delayed_delete_cb(lv_timer_t* t)
{
    lv_obj_t* obj = (lv_obj_t*)lv_timer_get_user_data(t);
    if (lv_obj_is_valid(obj))
    {
        lv_obj_delete_async(obj);
    }
    lv_timer_delete(t);
}

static void format_timer_cb(lv_timer_t* t)
{
    format_progress_t* p = (format_progress_t*)lv_timer_get_user_data(t);
    int val = lv_bar_get_value(p->bar);

    if (!p->done)
    {
        if (val < 90)
        {
            lv_bar_set_value(p->bar, val + 4, LV_ANIM_ON);
        }
    }
    else
    {
        char result_text[48];
        lv_bar_set_value(p->bar, 100, LV_ANIM_ON);

        if (p->result == ESP_OK)
        {
            snprintf(result_text, sizeof(result_text), "SD Card %d formatted successfully.", p->slot);
            lv_obj_set_style_text_color(p->label, lv_color_hex(0xff0ad757), 0);
        }
        else
        {
            snprintf(result_text, sizeof(result_text), "Failed to format SD Card %d.", p->slot);
            lv_obj_set_style_text_color(p->label, lv_color_hex(0xffff6b6b), 0);
        }
        lv_label_set_text(p->label, result_text);
        /* Log the result to SD now that formatting has completed and FATFS is free. */
        if (p->result == ESP_OK)
        {
            ui_logic_log_to_sd("QUICK_FORMAT_COMPLETED_OK");
        }
        else
        {
            ui_logic_log_to_sd("QUICK_FORMAT_FAILED");
        }

        lv_timer_delete(t);
        // Delete overlay after 2 seconds
        lv_timer_t* close_timer = lv_timer_create(delayed_delete_cb, 2000, p->overlay);
        lv_timer_set_repeat_count(close_timer, 1);
        free(p);
    }
}

static void log_export_timer_cb(lv_timer_t* t)
{
    log_export_progress_t* progress = (log_export_progress_t*)lv_timer_get_user_data(t);
    int value;

    if (progress == NULL)
    {
        lv_timer_delete(t);
        return;
    }

    if (!progress->done)
    {
        value = lv_bar_get_value(progress->bar);
        if (value < 92)
        {
            lv_bar_set_value(progress->bar, value + 4, LV_ANIM_ON);
        }
        return;
    }

    lv_bar_set_value(progress->bar, 100, LV_ANIM_ON);

    if (progress->result == ESP_OK)
    {
        lv_label_set_text(progress->label, "Export Successful!");
        lv_obj_set_style_text_color(progress->label, lv_color_hex(0xff0ad757), 0);
        ui_logic_log_to_sd("LOG_EXPORT_COMPLETED_OK");
    }
    else if (progress->result == ESP_ERR_NOT_FOUND)
    {
        lv_label_set_text(progress->label, "No logs found to export.");
        lv_obj_set_style_text_color(progress->label, lv_color_hex(UI_COLOR_WARNING), 0);
        ui_logic_log_to_sd("LOG_EXPORT_NO_FILES");
    }
    else
    {
        lv_label_set_text(progress->label, "Export Failed!");
        lv_obj_set_style_text_color(progress->label, lv_color_hex(0xffff6b6b), 0);
        ui_logic_log_to_sd("LOG_EXPORT_FAILED");
    }

    lv_timer_delete(t);
    lv_timer_t* close_timer = lv_timer_create(delayed_delete_cb, 2000, progress->overlay);
    lv_timer_set_repeat_count(close_timer, 1);
    free(progress);
}

/**
 * @brief Handle confirmation of SD card formatting
 */
static void action_format_confirmed(lv_event_t* e)
{
    int slot = (int)lv_event_get_user_data(e);
    lv_obj_t* mbox = find_msgbox_ancestor(lv_event_get_current_target(e));

    if (mbox == NULL)
    {
        ESP_LOGE(TAG, "Format action missing message box ancestor");
        return;
    }

    ESP_LOGI(TAG, "Starting quick format for Slot %d", slot);
    char title_text[32];
    char status_text[72];
    snprintf(title_text, sizeof(title_text), "Quick Format SD Card %d", slot);
    snprintf(status_text, sizeof(status_text), "Please wait while SD Card %d is being quick-formatted.", slot);

    // Create a blocking backdrop with a centered progress panel
    lv_obj_t* overlay = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(overlay, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_60, 0);
    lv_obj_center(overlay);

    lv_obj_t* panel = lv_obj_create(overlay);
    lv_obj_set_size(panel, 430, 220);
    lv_obj_center(panel);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x222222), 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(panel, lv_color_hex(0xffffffff), 0);
    lv_obj_set_style_border_width(panel, 2, 0);
    lv_obj_set_style_radius(panel, 16, 0);
    lv_obj_set_style_pad_all(panel, 18, 0);

    lv_obj_t* title_label = lv_label_create(panel);
    lv_label_set_text(title_label, title_text);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title_label, lv_color_hex(0xffffffff), 0);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 18);

    lv_obj_t* msg_label = lv_label_create(panel);
    lv_label_set_text(msg_label, status_text);
    lv_label_set_long_mode(msg_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(msg_label, 320);
    lv_obj_set_style_text_color(msg_label, lv_color_hex(0xffffffff), 0);
    lv_obj_align(msg_label, LV_ALIGN_TOP_MID, 0, 52);

    lv_obj_t* bar = lv_bar_create(panel);
    lv_obj_set_size(bar, 300, 30);
    lv_obj_align(bar, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xff4a4a4a), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xff0ad757), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_bar_set_value(bar, 6, LV_ANIM_OFF);

    // Close modal
    lv_msgbox_close_async(mbox);

    // Setup progress state and background task
    format_progress_t* p = malloc(sizeof(format_progress_t));
    p->bar = bar;
    p->label = msg_label;
    p->overlay = overlay;
    p->slot = slot;
    p->done = false;
    p->result = ESP_FAIL;

    lv_timer_create(format_timer_cb, 120, p);
    xTaskCreatePinnedToCore(quick_format_task, "quick_format", 6144, p, 4, NULL, 1);
    /* Avoid writing to the SD while a background quick-format may hold FATFS locks.
     * We'll emit a completion log from the LVGL timer callback (`format_timer_cb`) once
     * the background task sets `p->done` and `p->result`.
     */
    return;
}

/**
 * @brief Handle confirmation of Log Export from SD2 to SD1
 */
static void action_export_logs_confirmed(lv_event_t* e)
{
    lv_obj_t* btn = lv_event_get_current_target(e);
    lv_obj_t* mbox = find_msgbox_ancestor(btn);
    log_export_progress_t* progress;
    BaseType_t task_created;

    if (mbox == NULL)
    {
        ESP_LOGE(TAG, "Export action missing message box ancestor");
        return;
    }

    ESP_LOGI(TAG, "Starting Log Export from SD2 to SD1");

    // Create an overlay with a progress bar
    lv_obj_t* overlay = lv_obj_create(lv_screen_active());
    lv_obj_set_size(overlay, 400, 200);
    lv_obj_center(overlay);
    lv_obj_set_style_bg_color(overlay, lv_color_hex(0x222222), 0);

    lv_obj_t* msg_label = lv_label_create(overlay);
    lv_label_set_text(msg_label, "Backing up logs to SD1...");
    lv_obj_align(msg_label, LV_ALIGN_TOP_MID, 0, 20);

    lv_obj_t* bar = lv_bar_create(overlay);
    lv_obj_set_size(bar, 300, 30);
    lv_obj_center(bar);
    lv_bar_set_value(bar, 10, LV_ANIM_OFF);

    lv_msgbox_close_async(mbox);

    progress = malloc(sizeof(*progress));
    if (progress == NULL)
    {
        if (lv_obj_is_valid(overlay))
        {
            lv_obj_delete_async(overlay);
        }
        ui_show_message("Export Failed", "Unable to start background log export.", lv_color_hex(UI_COLOR_DANGER));
        return;
    }

    progress->bar = bar;
    progress->label = msg_label;
    progress->overlay = overlay;
    progress->done = false;
    progress->result = ESP_FAIL;

    task_created = xTaskCreatePinnedToCore(log_export_task, "log_export", 6144, progress, 4, NULL, 1);
    if (task_created != pdPASS)
    {
        free(progress);
        if (lv_obj_is_valid(overlay))
        {
            lv_obj_delete_async(overlay);
        }
        ui_show_message("Export Failed", "Failed to create log export task.", lv_color_hex(UI_COLOR_DANGER));
        return;
    }

    ui_logic_log_to_sd("LOG_EXPORT_TRIGGERED");
    lv_timer_create(log_export_timer_cb, 120, progress);
    return;
}

static void action_modal_cancel(lv_event_t* e)
{
    lv_obj_t* btn = lv_event_get_current_target(e);
    lv_obj_t* mbox = find_msgbox_ancestor(btn);
    if (mbox)
    {
        lv_msgbox_close_async(mbox);
    }
}

// SD test progress tracking structure
typedef struct
{
    lv_obj_t* overlay;
    lv_obj_t* bar;
    lv_obj_t* status_label;
    lv_obj_t* progress_label;
    int slot;
    volatile uint32_t write_bytes;
    volatile uint32_t elapsed_seconds;
    volatile bool test_done;
    esp_err_t test_result;
    time_t start_time;
} sd_test_progress_t;

// Background task for SD card write test (instant completion)
static void sd_write_test_task(void* pvParameters)
{
    sd_test_progress_t* progress = (sd_test_progress_t*)pvParameters;
    const char* mount = (progress->slot == 1) ? "/sdcard" : "/sdcard2";
    time_t start = time(NULL);

    ESP_LOGI(TAG, "SD Card %d write test starting (instant)", progress->slot);

    // Create test file with detailed header. Use the rallybox naming convention
    // so test files are easily identifiable and consistent with production logs.
    char test_filename[128];
    struct tm start_tm;
    localtime_r(&start, &start_tm);
    snprintf(test_filename, sizeof(test_filename), "%s/rallybox_sdcard%d_%04d%02d%02d_%02d%02d%02d_sdtest.txt",
        mount, progress->slot,
        start_tm.tm_year + 1900, start_tm.tm_mon + 1, start_tm.tm_mday,
        start_tm.tm_hour, start_tm.tm_min, start_tm.tm_sec);

    FILE* f = fopen(test_filename, "w");
    if (!f)
    {
        progress->test_result = ESP_FAIL;
        progress->test_done = true;
        vTaskDelete(NULL);
        return;
    }

    system_status_t sys = system_monitor_get_status();

    // Write a detailed start line including CPU, RAM, timezone, Wi-Fi and SD info
    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    char timestr[64];
    strftime(timestr, sizeof(timestr), "%Y-%m-%d %H:%M:%S %Z", &timeinfo);

    uint64_t cap = 0, used = 0;
    sd_card_get_info(progress->slot, &cap, &used);

    fprintf(f, "SDTEST_START %s slot=%d uptime=%lu cpu=%u%% free_heap=%u total_heap=%u wifi=%s ssid=%s ip=%s sd_total_mb=%.0llu sd_used_mb=%.0llu\n",
        timestr,
        progress->slot,
        sys.uptime_seconds,
        (unsigned)sys.cpu_load_percent,
        (unsigned)sys.free_heap_bytes,
        (unsigned)sys.total_heap_bytes,
        sys.wifi_connected ? "connected" : "disconnected",
        sys.wifi_ssid[0] ? sys.wifi_ssid : "",
        sys.wifi_ip[0] ? sys.wifi_ip : "",
        (unsigned long long)cap,
        (unsigned long long)used);

    progress->start_time = start;
    uint32_t bytes_written = 0;
    uint32_t write_count = 0;
    const uint32_t chunk_size = 4096;
    char write_buffer[4096];

    // Fill buffer with pattern
    memset(write_buffer, 0xAA, sizeof(write_buffer));

    // Write just one chunk to complete instantly
    size_t written = fwrite(write_buffer, 1, chunk_size, f);
    if (written == chunk_size)
    {
        bytes_written += chunk_size;
        write_count++;
    }
    else
    {
        ESP_LOGE(TAG, "Write failed at offset %lu", bytes_written);
    }

    // Concise end line
    time_t end_time = time(NULL);
    struct tm end_timeinfo;
    localtime_r(&end_time, &end_timeinfo);
    char end_timestr[64];
    strftime(end_timestr, sizeof(end_timestr), "%Y-%m-%d %H:%M:%S %Z", &end_timeinfo);

    uint64_t cap_end = 0, used_end = 0;
    sd_card_get_info(progress->slot, &cap_end, &used_end);

    fprintf(f, "SDTEST_END %s slot=%d bytes=%lu ops=%lu speed=%.2fMB/s cpu=%u%% free_heap=%u total_heap=%u sd_total_mb=%.0llu sd_used_mb=%.0llu\n",
        end_timestr,
        progress->slot,
        bytes_written,
        write_count,
        (float)(bytes_written) / (1024.0 * 1024.0) / 1.0,  // Speed over 1 second
        (unsigned)sys.cpu_load_percent,
        (unsigned)sys.free_heap_bytes,
        (unsigned)sys.total_heap_bytes,
        (unsigned long long)cap_end,
        (unsigned long long)used_end);

    fclose(f);

    progress->elapsed_seconds = 1;
    progress->write_bytes = bytes_written;
    progress->test_result = ESP_OK;
    progress->test_done = true;

    ESP_LOGI(TAG, "SD Card %d test completed: %lu bytes in %lu operations",
        progress->slot, bytes_written, write_count);

    vTaskDelete(NULL);
}

// Timer callback for SD test progress updates
static void sd_test_progress_cb(lv_timer_t* t)
{
    sd_test_progress_t* p = (sd_test_progress_t*)lv_timer_get_user_data(t);

    if (!p->test_done)
    {
        // Update progress bar (0-99)
        int progress = (p->elapsed_seconds >= 1) ? 99 : ((p->elapsed_seconds * 100) / 1);
        lv_bar_set_value(p->bar, progress, LV_ANIM_ON);

        // Update labels
        char status_text[80];
        snprintf(status_text, sizeof(status_text), "Please wait, writing in progress...\n%u / 1 seconds",
            p->elapsed_seconds);
        lv_label_set_text(p->status_label, status_text);

        char progress_text[64];
        snprintf(progress_text, sizeof(progress_text), "%.2f MB written",
            (float)(p->write_bytes) / (1024.0 * 1024.0));
        lv_label_set_text(p->progress_label, progress_text);
    }
    else
    {
        // Test complete
        lv_bar_set_value(p->bar, 100, LV_ANIM_ON);

        char result_text[128];
        if (p->test_result == ESP_OK)
        {
            snprintf(result_text, sizeof(result_text),
                "SD Card %d test completed successfully!\nTotal: %.2f MB written\nDuration: %u seconds",
                p->slot, (float)(p->write_bytes) / (1024.0 * 1024.0), p->elapsed_seconds);
            lv_obj_set_style_text_color(p->status_label, lv_color_hex(0xff0ad757), 0);
        }
        else
        {
            snprintf(result_text, sizeof(result_text),
                "SD Card %d test failed!\nPlease check the card and try again.", p->slot);
            lv_obj_set_style_text_color(p->status_label, lv_color_hex(0xffff6b6b), 0);
        }
        lv_label_set_text(p->status_label, result_text);
        lv_obj_set_style_text_color(p->progress_label, lv_color_hex(0xffffffff), 0);

        lv_timer_delete(t);
        lv_timer_t* close_timer = lv_timer_create(delayed_delete_cb, 3000, p->overlay);
        lv_timer_set_repeat_count(close_timer, 1);
        free(p);
    }
}

/**
 * @brief Action for SD Card Testing (Production Method with Wi-Fi requirement)
 */
void action_test_sdcard(lv_event_t* e)
{
    int slot = (int)(intptr_t)lv_event_get_user_data(e);

    bool mounted = (slot % 100 == 1) ? sd_card1_is_mounted() : sd_card2_is_mounted();
    if (!mounted)
    {
        ui_show_message("Error", "No SD card detected in slot. Insert card first.", lv_color_hex(0xffe57373));
        return;
    }

    // Export Logic (Short press/Click with specific slot code > 200)
    if (slot > 200)
    {
        if (!sd_card1_is_mounted() || !sd_card2_is_mounted())
        {
            ui_show_message("Error", "Both SD cards required for export.", lv_color_hex(0xffe57373));
            return;
        }
        lv_obj_t* mbox = ui_create_modal("Backup Logs", "Export all system logs from Slot 2 to Slot 1?", lv_color_hex(0xff2a2a2a));
        lv_obj_t* export_btn = lv_msgbox_add_footer_button(mbox, "Export");
        lv_obj_t* cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
        lv_obj_add_event_cb(export_btn, action_export_logs_confirmed, LV_EVENT_CLICKED, NULL);
        lv_obj_add_event_cb(cancel_btn, action_modal_cancel, LV_EVENT_CLICKED, NULL);
        return;
    }

    // Format Logic Request
    if (slot > 100)
    {
        int card_num = slot - 100;
        lv_obj_t* mbox = ui_create_modal("Confirm Quick Format", "A quick format will rebuild the FAT filesystem on this card.\nExisting files will no longer be accessible.", lv_color_hex(0xff8b2e2e));
        lv_obj_t* format_btn = lv_msgbox_add_footer_button(mbox, "Quick Format");
        lv_obj_t* cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
        lv_obj_add_event_cb(format_btn, action_format_confirmed, LV_EVENT_CLICKED, (void*)(intptr_t)card_num);
        lv_obj_add_event_cb(cancel_btn, action_modal_cancel, LV_EVENT_CLICKED, NULL);
        return;
    }

    // Wi-Fi CONNECTION CHECK before write test
    system_status_t status = system_monitor_get_status();
    if (status.wifi_state != SYSTEM_WIFI_STATE_CONNECTED)
    {
        ui_show_message("Wi-Fi Required",
            "SD card write testing requires Wi-Fi connection.\n"
            "Please connect to Wi-Fi first and try again.",
            lv_color_hex(0xffe57373));
        ui_logic_log_to_sd("SDCARD_TEST_FAILED_NO_WIFI");
        return;
    }

    // Create progress overlay for 1-minute write test
    lv_obj_t* overlay = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(overlay, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_60, 0);
    lv_obj_center(overlay);

    lv_obj_t* panel = lv_obj_create(overlay);
    lv_obj_set_size(panel, 500, 280);
    lv_obj_center(panel);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x1a1a1a), 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(panel, lv_color_hex(0xffffffff), 0);
    lv_obj_set_style_border_width(panel, 2, 0);
    lv_obj_set_style_radius(panel, 16, 0);
    lv_obj_set_style_pad_all(panel, 20, 0);

    char title_text[64];
    snprintf(title_text, sizeof(title_text), "SD Card %d Write Test", slot);
    lv_obj_t* title_label = lv_label_create(panel);
    lv_label_set_text(title_label, title_text);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title_label, lv_color_hex(0xffffffff), 0);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 0);

    lv_obj_t* status_label = lv_label_create(panel);
    lv_label_set_text(status_label, "Please wait, writing in progress...\n0 / 60 seconds");
    lv_label_set_long_mode(status_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(status_label, 380);
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xffffc107), 0);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_16, 0);
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 40);

    lv_obj_t* bar = lv_bar_create(panel);
    lv_obj_set_size(bar, 380, 35);
    lv_obj_align(bar, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xff3a3a3a), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xff00acc1), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar, 10, 0);
    lv_bar_set_value(bar, 0, LV_ANIM_OFF);

    lv_obj_t* progress_label = lv_label_create(panel);
    lv_label_set_text(progress_label, "0.00 MB written");
    lv_obj_set_style_text_color(progress_label, lv_color_hex(0xffffffff), 0);
    lv_obj_set_style_text_font(progress_label, &lv_font_montserrat_14, 0);
    lv_obj_align(progress_label, LV_ALIGN_BOTTOM_MID, 0, -10);

    // Setup progress tracking state
    sd_test_progress_t* p = malloc(sizeof(sd_test_progress_t));
    p->overlay = overlay;
    p->bar = bar;
    p->status_label = status_label;
    p->progress_label = progress_label;
    p->slot = slot;
    p->write_bytes = 0;
    p->elapsed_seconds = 0;
    p->test_done = false;
    p->test_result = ESP_FAIL;

    // Start background write test task
    char log_buf[128];
    snprintf(log_buf, sizeof(log_buf), "SDCARD_TEST_STARTED_SLOT_%d_WITH_WIFI", slot);
    ui_logic_log_to_sd(log_buf);

    lv_timer_create(sd_test_progress_cb, 500, p);  // Update progress every 500ms
    xTaskCreatePinnedToCore(sd_write_test_task, "sd_write_test", 8192, p, 5, NULL, 1);
}

/**
 * @brief Initialize all event listeners for the professional production UI
 */
void ui_logic_init_events(void);


/**
 * @brief Update Live System Stats on Dashboard
 */
void ui_logic_update_system_stats(uint32_t cpu_load, uint32_t errors)
{
    if (s_ota_update_in_progress)
    {
        return;
    }

    if (objects.system_load_status_1)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%lu%%", cpu_load);
        lv_label_set_text(objects.system_load_status_1, (cpu_load < 80) ? "STABLE" : "OVERLOAD");
        lv_obj_set_style_text_color(objects.system_load_status_1, (cpu_load < 80) ? lv_color_hex(0xff0ad757) : lv_color_hex(0xffe57373), 0);
    }
    if (objects.error_status_1)
    { // Corrected: error_status_1 is the counter label
        char buf[16];
        snprintf(buf, sizeof(buf), "%lu", errors);
        lv_label_set_text(objects.error_status_1, buf);
    }
}

/**
 * @brief Professional error handler for UI operations
 */
static void ui_handle_error(const char* msg, esp_err_t err)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "%s\nError: %s", msg, esp_err_to_name(err));
    ui_show_message("System Error", buf, lv_color_hex(0xffe57373));
}

/**
 * @brief Show SD card mount/unmount event
 */
void ui_show_sd_card_event(int slot, bool mounted)
{
    char title[32];
    char msg[64];
    if (mounted)
    {
        snprintf(title, sizeof(title), "SD Card %d Mounted", slot);
        snprintf(msg, sizeof(msg), "SD Card %d has been mounted successfully.", slot);
        ESP_LOGI(TAG, "UI: Showing SD Card %d mounted notification", slot);
    }
    else
    {
        snprintf(title, sizeof(title), "SD Card %d Disconnected", slot);
        snprintf(msg, sizeof(msg), "SD Card %d has been disconnected/ejected.", slot);
        ESP_LOGW(TAG, "UI: Showing SD Card %d disconnected notification", slot);
    }
    ui_show_message(title, msg, mounted ? lv_color_hex(0xff4caf50) : lv_color_hex(0xffff9800));
}

/**
 * @brief Action for WiFi Connection (Production Level)
 */
void action_connect_wifi(lv_event_t* e)
{
    system_status_t status = system_monitor_get_status();
    lv_obj_t* ssid_obj = objects.wifi_ssid_1;
    lv_obj_t* pass_obj = objects.wifi_password_1;

    const char* ssid = ssid_obj ? lv_textarea_get_text(ssid_obj) : "";
    const char* pass = pass_obj ? lv_textarea_get_text(pass_obj) : "";

    if (status.wifi_state == SYSTEM_WIFI_STATE_CONNECTING)
    {
        ui_show_message("WiFi Busy", "A WiFi connection attempt is already running. Wait for it to finish or fail before retrying.", lv_color_hex(0xffffc107));
        lv_obj_add_flag(objects.ui_keyboard_1, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    if (status.wifi_state == SYSTEM_WIFI_STATE_DISCONNECTED && strlen(ssid) == 0)
    {
        if (status.wifi_ssid[0] != '\0')
        {
            esp_err_t saved_err = system_wifi_connect_saved();
            if (saved_err != ESP_OK)
            {
                ui_handle_error("Failed to reconnect using saved WiFi credentials", saved_err);
                return;
            }
            system_status_t status_copy = system_monitor_get_status();
            ui_refresh_wifi_controls(&status_copy);
            lv_obj_add_flag(objects.ui_keyboard_1, LV_OBJ_FLAG_HIDDEN);
            return;
        }

        ui_show_message("WiFi Required", "Enter an SSID and password, then tap CONNECT. Scan is only for discovering nearby networks.", lv_color_hex(0xff1565c0));
        system_status_t status_copy2 = system_monitor_get_status();
        ui_refresh_wifi_controls(&status_copy2);
        lv_obj_add_flag(objects.ui_keyboard_1, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    if (status.wifi_state == SYSTEM_WIFI_STATE_CONNECTED)
    {
        esp_err_t disconnect_err = system_wifi_disconnect(false);
        if (disconnect_err != ESP_OK)
        {
            ui_handle_error("Failed to disconnect WiFi", disconnect_err);
            return;
        }
        system_status_t status_copy3 = system_monitor_get_status();
        ui_refresh_wifi_controls(&status_copy3);
        lv_obj_add_flag(objects.ui_keyboard_1, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    if (strlen(ssid) < 2)
    {
        if (status.wifi_ssid[0])
        {
            esp_err_t saved_err = system_wifi_connect_saved();
            if (saved_err != ESP_OK)
            {
                ui_handle_error("Failed to reconnect using saved WiFi credentials", saved_err);
                return;
            }
        }
        else
        {
            ui_show_message("Validation Error", "SSID must be at least 2 characters.", lv_color_hex(0xffe57373));
            return;
        }
    }

    if (strlen(pass) > 0 && strlen(pass) < 8)
    {
        ui_show_message("Validation Error", "Password must be at least 8 characters (or empty).", lv_color_hex(0xffe57373));
        return;
    }

    if (strlen(ssid) >= 2)
    {
        ESP_LOGI(TAG, "Connecting to WiFi: %s", ssid);
        esp_err_t connect_err = system_wifi_connect_credentials(ssid, pass);
        if (connect_err != ESP_OK)
        {
            ui_handle_error("Failed to start WiFi connection", connect_err);
            return;
        }
    }

    system_status_t status_copy4 = system_monitor_get_status();
    ui_refresh_wifi_controls(&status_copy4);

    lv_obj_add_flag(objects.ui_keyboard_1, LV_OBJ_FLAG_HIDDEN);
}

void action_show_keyboard(lv_event_t* e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* ta = lv_event_get_current_target(e);

    if (code != LV_EVENT_PRESSED && code != LV_EVENT_CLICKED && code != LV_EVENT_FOCUSED)
    {
        return;
    }

    if (ta == NULL || objects.ui_keyboard_1 == NULL)
    {
        return;
    }

    /* Some events can bubble from non-textarea children/containers.
     * Guard aggressively to avoid LVGL type assertions. */
    if (ta != objects.wifi_ssid_1 &&
        ta != objects.wifi_password_1 &&
        ta != objects.gnss_baud_input &&
        ta != objects.gnss_tx_input &&
        ta != objects.gnss_rx_input)
    {
        return;
    }

    if (!lv_obj_check_type(ta, &lv_textarea_class) || !lv_obj_check_type(objects.ui_keyboard_1, &lv_keyboard_class))
    {
        return;
    }

    /* Ensure the target input is editable and focusable before attaching keyboard. */
    lv_obj_add_flag(ta, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_clear_state(ta, LV_STATE_DISABLED);
    lv_textarea_set_cursor_click_pos(ta, true);

    if (ta == objects.gnss_baud_input || ta == objects.gnss_tx_input || ta == objects.gnss_rx_input)
    {
        lv_keyboard_set_mode(objects.ui_keyboard_1, LV_KEYBOARD_MODE_NUMBER);
    }
    else
    {
        lv_keyboard_set_mode(objects.ui_keyboard_1, LV_KEYBOARD_MODE_TEXT_LOWER);
    }

    lv_keyboard_set_textarea(objects.ui_keyboard_1, ta);
    lv_obj_remove_flag(objects.ui_keyboard_1, LV_OBJ_FLAG_HIDDEN);

    // Move keyboard to a visible position
    lv_obj_set_pos(objects.ui_keyboard_1, 0, 432); // Bottom 40% of 720p screen starts at 432
    lv_obj_set_size(objects.ui_keyboard_1, LV_PCT(100), 288); // 40% of 720 is 288

    // Potential UX improvement: Scroll the input box up so the active textarea is visible
    lv_obj_scroll_to_view(ta, LV_ANIM_ON);
}

void action_password_ssid_keyboad(lv_event_t* e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL)
    {
        lv_obj_add_flag(objects.ui_keyboard_1, LV_OBJ_FLAG_HIDDEN);
        lv_keyboard_set_textarea(objects.ui_keyboard_1, NULL);
    }
}

void action_jumptowifiscreen(lv_event_t* e)
{
    ESP_LOGI(TAG, "Navigating to WiFi Tab");
    if (objects.obj0)
    { // obj0 is the TabView according to screens.c
        lv_tabview_set_active(objects.obj0, 0, LV_ANIM_ON); // 0 = Menu, 1 = Wi-Fi
    }
}

/**
 * @brief Callback for Files tab click - create/show file manager
 */
 /* Production Method: Helper functions to update UI state without touching UI code */

void ui_logic_update_sd_status(int slot, bool active)
{
    lv_obj_t* target = (slot == 1) ? objects.active_checkbox_sd_card_1_1 : objects.active_checkbox_sd_card_2_1;
    if (target)
    {
        if (active) lv_obj_add_state(target, LV_STATE_CHECKED);
        else lv_obj_remove_state(target, LV_STATE_CHECKED);
    }
}

void ui_logic_set_uptime(uint32_t seconds)
{
    char buf[32];

    if (s_ota_update_in_progress)
    {
        return;
    }

    uint32_t h = seconds / 3600;
    uint32_t m = (seconds % 3600) / 60;
    uint32_t s = seconds % 60;
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", h, m, s);
    if (objects.title_4)
    {
        lv_label_set_text(objects.title_4, buf);
    }
}

/**
 * @brief Logic to update SD card visibility and status on dashboard
 */
void ui_logic_update_storage_info(int slot, bool detected, uint64_t capacity_mb, uint64_t used_mb)
{
    lv_obj_t* active_cb = (slot == 1) ? objects.active_checkbox_sd_card_1_1 : objects.active_checkbox_sd_card_2_1;
    lv_obj_t* status_label = (slot == 1) ? objects.read_write_label_1_1 : objects.read_write_label_2_1;
    lv_obj_t* storage_label = (slot == 1) ? objects.sdcard_storage_1_1 : objects.sd_card_title_2_1;

    char storage_buf[128];
    char status_buf[64];

    if (s_ota_update_in_progress)
    {
        return;
    }

    if (detected && capacity_mb > 0)
    {
        lv_obj_add_state(active_cb, LV_STATE_CHECKED);

        // Format storage: "Used / Total" with auto-unit conversion (MB/GB)
        double total_gb = capacity_mb / 1024.0;
        double used_gb = used_mb / 1024.0;

        if (total_gb >= 1.0)
        {
            snprintf(storage_buf, sizeof(storage_buf), "%.1f / %.1f GB", used_gb, total_gb);
            UI_STORAGE_DEBUG_LOGI(TAG, "UI: SD Card %d - %.1f GB / %.1f GB", slot, used_gb, total_gb);
        }
        else
        {
            snprintf(storage_buf, sizeof(storage_buf), "%llu / %llu MB", used_mb, capacity_mb);
            UI_STORAGE_DEBUG_LOGI(TAG, "UI: SD Card %d - %llu MB / %llu MB", slot, used_mb, capacity_mb);
        }

        lv_label_set_text(storage_label, storage_buf);

        // Status string
        snprintf(status_buf, sizeof(status_buf), "SD CARD %d READY", slot);
        lv_label_set_text(status_label, status_buf);
        lv_obj_set_style_text_color(status_label, lv_color_hex(0xff0ad757), 0);
    }
    else
    {
        lv_obj_remove_state(active_cb, LV_STATE_CHECKED);
        lv_label_set_text(storage_label, slot == 1 ? "Storage 1" : "Storage 2");
        lv_label_set_text(status_label, "DISCONNECTED");
        lv_obj_set_style_text_color(status_label, lv_color_hex(0xffe57373), 0);
        UI_STORAGE_DEBUG_LOGW(TAG, "UI: SD Card %d - DISCONNECTED", slot);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MODERN UI COLOR PALETTE & STYLING HELPERS
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Apply consistent modern styling to buttons
 */
static void style_modern_button(lv_obj_t* button, lv_color_t primary_color, lv_color_t secondary_color)
{
    if (!button) return;

    // Button styling
    lv_obj_set_style_bg_color(button, primary_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(button, secondary_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(button, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(button, lv_color_hex(UI_COLOR_BORDER), LV_PART_MAIN);
    lv_obj_set_style_border_width(button, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(button, 12, 0);
    lv_obj_set_style_shadow_width(button, 0, 0);
    lv_obj_set_style_outline_width(button, 0, 0);

    // Pressed state
    lv_obj_set_style_bg_opa(button, 220, LV_PART_MAIN | LV_STATE_PRESSED);

    // Label styling inside button
    lv_obj_t* label = lv_obj_get_child(button, 0);
    if (label)
    {
        lv_obj_set_style_text_color(label, lv_color_hex(UI_COLOR_TEXT), 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_letter_space(label, 1, 0);
    }
}

typedef enum
{
    UI_GPX_OPERATION_SAVE_SD = 0,
    UI_GPX_OPERATION_UPLOAD_WEB
} ui_gpx_operation_kind_t;

typedef struct
{
    lv_obj_t* panel;
    lv_obj_t* bar;
    lv_obj_t* status_label;
    ui_gpx_operation_kind_t kind;
    gps_points_feed_t feed;
    int sd_slot;
    volatile bool done;
    esp_err_t result;
    size_t points;
    time_t first_utc;
    char filename[96];
    char path[160];
    char feed_name[8];
} ui_gpx_operation_t;

static void ui_gpx_operation_task(void* pvParameters)
{
    ui_gpx_operation_t* operation = (ui_gpx_operation_t*)pvParameters;

    if (operation == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

    if (operation->kind == UI_GPX_OPERATION_SAVE_SD)
    {
        operation->result = gps_points_write_gpx(operation->feed,
            operation->path,
            &operation->points,
            &operation->first_utc) ? ESP_OK : ESP_FAIL;
    }
    else
    {
#if CONFIG_RALLYBOX_RACEBOX_ENABLED
        system_status_t status = system_monitor_get_status();
        bool suspend_racebox = status.racebox_initialized != 0;

        if (suspend_racebox)
        {
            s_bt_suspend_preserve_feed = true;
            esp_err_t ret = racebox_shutdown(3000);
            if (ret != ESP_OK)
            {
                s_bt_suspend_preserve_feed = false;
                ESP_LOGE(TAG, "Failed to suspend RaceBox BLE before WEB upload: %s", esp_err_to_name(ret));
                operation->result = ret;
                operation->done = true;
                vTaskDelete(NULL);
                return;
            }

            ESP_LOGI(TAG, "RaceBox BLE suspended for WEB upload");
        }
#endif

        operation->result = gps_points_upload_web(operation->feed,
            operation->filename,
            &operation->points,
            &operation->first_utc);

#if CONFIG_RALLYBOX_RACEBOX_ENABLED
        if (suspend_racebox)
        {
            ESP_LOGI(TAG, "RaceBox BLE remains stopped after WEB upload; restart only on explicit user action");
            s_bt_suspend_preserve_feed = false;
        }
#endif
    }

    operation->done = true;
    vTaskDelete(NULL);
}

static void ui_gpx_operation_timer_cb(lv_timer_t* timer)
{
    ui_gpx_operation_t* operation = (ui_gpx_operation_t*)lv_timer_get_user_data(timer);
    char message[224];
    const char* title;
    uint32_t color = UI_COLOR_SUCCESS;
    int value;

    if (operation == NULL)
    {
        lv_timer_delete(timer);
        s_gpx_operation_in_progress = false;
        return;
    }

    if (!operation->done)
    {
        value = lv_bar_get_value(operation->bar);
        if (value < 92)
        {
            lv_bar_set_value(operation->bar, value + 4, LV_ANIM_ON);
        }
        return;
    }

    lv_timer_delete(timer);
    s_gpx_operation_in_progress = false;

    if (lv_obj_is_valid(operation->panel))
    {
        lv_obj_delete_async(operation->panel);
    }

    if (operation->kind == UI_GPX_OPERATION_SAVE_SD)
    {
        title = "GPX Save";
        if (operation->result == ESP_OK)
        {
            snprintf(message,
                sizeof(message),
                "%s saved %lu points to %s",
                operation->feed_name,
                (unsigned long)operation->points,
                operation->path);
        }
        else
        {
            color = UI_COLOR_DANGER;
            snprintf(message,
                sizeof(message),
                "Failed to save %s GPX on SD%d.",
                operation->feed_name,
                operation->sd_slot);
        }
    }
    else
    {
        title = "GPX Upload";
        if (operation->result == ESP_OK)
        {
            snprintf(message,
                sizeof(message),
                "%s uploaded %lu points",
                operation->feed_name,
                (unsigned long)operation->points);
        }
        else if (operation->result == ESP_ERR_INVALID_STATE)
        {
            color = UI_COLOR_WARNING;
            snprintf(message,
                sizeof(message),
                "System time not synced yet. Wait for SNTP and retry WEB upload.");
        }
        else if (operation->result == ESP_ERR_INVALID_ARG)
        {
            color = UI_COLOR_WARNING;
            snprintf(message,
                sizeof(message),
                "Configure GPX WEB/S3 settings in menuconfig.");
        }
        else
        {
            color = UI_COLOR_DANGER;
            snprintf(message,
                sizeof(message),
                "%s WEB upload failed.",
                operation->feed_name);
        }
    }

    ui_show_message(title, message, lv_color_hex(color));
    free(operation);
}

static void ui_start_gpx_operation(ui_gpx_operation_kind_t kind, gps_points_feed_t feed, int sd_slot)
{
    ui_gpx_operation_t* operation;
    lv_obj_t* panel;
    lv_obj_t* title_label;
    lv_obj_t* detail_label;
    const char* feed_name = (feed == GPS_POINTS_FEED_BLE) ? "BLE" : "GNSS";
    size_t points = 0;
    time_t first_utc = 0;
    char message[192];
    const char* mount_path = NULL;
    bool mounted = false;
    BaseType_t task_created;

    if (s_gpx_operation_in_progress)
    {
        ui_show_message("GPX Busy", "Another GPX save/upload is already running.", lv_color_hex(UI_COLOR_WARNING));
        return;
    }

    if (kind == UI_GPX_OPERATION_SAVE_SD)
    {
        mount_path = (sd_slot == 1) ? "/sdcard" : "/sdcard2";
        mounted = (sd_slot == 1) ? sd_card1_is_mounted() : sd_card2_is_mounted();
        if (!mounted)
        {
            snprintf(message, sizeof(message), "SD%d is not mounted.", sd_slot);
            ui_show_message("GPX Save", message, lv_color_hex(UI_COLOR_WARNING));
            return;
        }
    }

    if (!gps_points_get_summary(feed, &points, &first_utc))
    {
        ui_show_message(kind == UI_GPX_OPERATION_SAVE_SD ? "GPX Save" : "GPX Upload",
            "Track buffer unavailable.",
            lv_color_hex(UI_COLOR_DANGER));
        return;
    }
    if (points == 0)
    {
        snprintf(message, sizeof(message), "%s feed has no points yet.", feed_name);
        ui_show_message(kind == UI_GPX_OPERATION_SAVE_SD ? "GPX Save" : "GPX Upload",
            message,
            lv_color_hex(UI_COLOR_WARNING));
        return;
    }

    operation = calloc(1, sizeof(*operation));
    if (operation == NULL)
    {
        ui_show_message("GPX Error", "Unable to start background GPX operation.", lv_color_hex(UI_COLOR_DANGER));
        return;
    }

    operation->kind = kind;
    operation->feed = feed;
    operation->sd_slot = sd_slot;
    operation->points = points;
    operation->first_utc = first_utc;
    operation->result = ESP_FAIL;
    snprintf(operation->feed_name, sizeof(operation->feed_name), "%s", feed_name);
    gps_points_make_filename(feed, first_utc, operation->filename, sizeof(operation->filename));
    if (kind == UI_GPX_OPERATION_SAVE_SD)
    {
        snprintf(operation->path, sizeof(operation->path), "%s/%s", mount_path, operation->filename);
    }

    panel = lv_obj_create(lv_screen_active());
    lv_obj_set_size(panel, 440, 210);
    lv_obj_center(panel);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(panel, lv_color_hex(UI_COLOR_SURFACE), 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(panel, lv_color_hex(UI_COLOR_BORDER), 0);
    lv_obj_set_style_border_width(panel, 2, 0);
    lv_obj_set_style_radius(panel, 16, 0);
    lv_obj_set_style_pad_all(panel, 18, 0);
    lv_obj_move_foreground(panel);
    operation->panel = panel;

    title_label = lv_label_create(panel);
    lv_label_set_text(title_label, kind == UI_GPX_OPERATION_SAVE_SD ? "Saving GPX" : "Uploading GPX");
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title_label, lv_color_hex(UI_COLOR_TEXT), 0);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 12);

    detail_label = lv_label_create(panel);
    snprintf(message,
        sizeof(message),
        kind == UI_GPX_OPERATION_SAVE_SD ? "%s: %lu points to SD%d" : "%s: %lu points to WEB",
        feed_name,
        (unsigned long)points,
        sd_slot);
    lv_label_set_text(detail_label, message);
    lv_obj_set_style_text_color(detail_label, lv_color_hex(UI_COLOR_MUTED), 0);
    lv_obj_set_style_text_font(detail_label, &lv_font_montserrat_16, 0);
    lv_obj_align(detail_label, LV_ALIGN_TOP_MID, 0, 54);

    operation->status_label = lv_label_create(panel);
    lv_label_set_text(operation->status_label,
        kind == UI_GPX_OPERATION_SAVE_SD ? "Writing GPX in background..." : "Uploading GPX in background...");
    lv_obj_set_style_text_color(operation->status_label, lv_color_hex(UI_COLOR_TEXT), 0);
    lv_obj_set_style_text_font(operation->status_label, &lv_font_montserrat_16, 0);
    lv_obj_align(operation->status_label, LV_ALIGN_CENTER, 0, -4);

    operation->bar = lv_bar_create(panel);
    lv_obj_set_size(operation->bar, 320, 22);
    lv_obj_align(operation->bar, LV_ALIGN_CENTER, 0, 42);
    lv_obj_set_style_bg_color(operation->bar, lv_color_hex(UI_COLOR_SURFACE_ALT), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(operation->bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(operation->bar, lv_color_hex(UI_COLOR_ACCENT), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(operation->bar, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(operation->bar, 8, LV_PART_MAIN);
    lv_obj_set_style_radius(operation->bar, 8, LV_PART_INDICATOR);
    lv_bar_set_value(operation->bar, 8, LV_ANIM_OFF);

    s_gpx_operation_in_progress = true;
    task_created = xTaskCreatePinnedToCore(ui_gpx_operation_task,
        kind == UI_GPX_OPERATION_SAVE_SD ? "gpx_save" : "gpx_upload",
        10240,
        operation,
        4,
        NULL,
        1);
    if (task_created != pdPASS)
    {
        s_gpx_operation_in_progress = false;
        if (lv_obj_is_valid(panel))
        {
            lv_obj_delete(panel);
        }
        free(operation);
        ui_show_message("GPX Error", "Failed to create GPX background task.", lv_color_hex(UI_COLOR_DANGER));
        return;
    }

    lv_timer_create(ui_gpx_operation_timer_cb, 120, operation);
}

static void ui_save_feed_to_sd(gps_points_feed_t feed, int sd_slot)
{
    ui_start_gpx_operation(UI_GPX_OPERATION_SAVE_SD, feed, sd_slot);
}

static void ui_upload_feed_web(gps_points_feed_t feed)
{
    ui_start_gpx_operation(UI_GPX_OPERATION_UPLOAD_WEB, feed, 0);
}

static void action_ble_save_sd1(lv_event_t* e)
{
    LV_UNUSED(e);
    ui_save_feed_to_sd(GPS_POINTS_FEED_BLE, 1);
}

static void action_ble_save_sd2(lv_event_t* e)
{
    LV_UNUSED(e);
    ui_save_feed_to_sd(GPS_POINTS_FEED_BLE, 2);
}

static void action_ble_save_web(lv_event_t* e)
{
    LV_UNUSED(e);
    ui_upload_feed_web(GPS_POINTS_FEED_BLE);
}

static void action_ble_reset(lv_event_t* e)
{
    esp_err_t ret;
    system_status_t status;

    LV_UNUSED(e);
    ret = gps_points_reset_feed(GPS_POINTS_FEED_BLE);
    if (ret != ESP_OK)
    {
        ui_handle_error("Failed to reset BLE track buffer", ret);
        return;
    }

    s_bt_rx_count = 0;
    s_last_bt_count = 0;
    s_bt_last_rx_change_us = 0;
    status = system_monitor_get_status();
    ui_update_main_status_panels(&status);
}

static void action_gnss_save_sd1(lv_event_t* e)
{
    LV_UNUSED(e);
    ui_save_feed_to_sd(GPS_POINTS_FEED_GNSS, 1);
}

static void action_gnss_save_sd2(lv_event_t* e)
{
    LV_UNUSED(e);
    ui_save_feed_to_sd(GPS_POINTS_FEED_GNSS, 2);
}

static void action_gnss_save_web(lv_event_t* e)
{
    LV_UNUSED(e);
    ui_upload_feed_web(GPS_POINTS_FEED_GNSS);
}

static void action_gnss_reset(lv_event_t* e)
{
    esp_err_t ret;
    system_status_t status;

    LV_UNUSED(e);
    ret = gps_points_reset_feed(GPS_POINTS_FEED_GNSS);
    if (ret != ESP_OK)
    {
        ui_handle_error("Failed to reset GNSS track buffer", ret);
        return;
    }

    s_gnss_rx_count = 0;
    s_last_gnss_count = 0;
    s_gnss_last_rx_change_us = 0;
    status = system_monitor_get_status();
    ui_update_main_status_panels(&status);
}

static void ui_create_gpx_buttons(void)
{
    lv_obj_t* label;

    if (objects.menu == NULL)
    {
        return;
    }

    if (s_panel_racebox == NULL || s_panel_gnss == NULL)
    {
        ui_create_main_status_panels();
    }
    if (s_panel_racebox == NULL || s_panel_gnss == NULL)
    {
        return;
    }

    if (s_btn_ble_sd1 == NULL)
    {
        s_btn_ble_sd1 = lv_button_create(s_panel_racebox);
        label = lv_label_create(s_btn_ble_sd1);
        lv_label_set_text(label, "BLE SD1");
        lv_obj_center(label);
        lv_obj_add_event_cb(s_btn_ble_sd1, action_ble_save_sd1, LV_EVENT_CLICKED, NULL);
    }
    ui_setup_dashboard_button(s_btn_ble_sd1,
        s_panel_racebox,
        18,
        184,
        MENU_PANEL_THIRD_BUTTON_WIDTH,
        42,
        "BLE SD1",
        lv_color_hex(0xff2e7d32),
        lv_color_hex(0xff1b5e20));

    if (s_btn_ble_sd2 == NULL)
    {
        s_btn_ble_sd2 = lv_button_create(s_panel_racebox);
        label = lv_label_create(s_btn_ble_sd2);
        lv_label_set_text(label, "BLE SD2");
        lv_obj_center(label);
        lv_obj_add_event_cb(s_btn_ble_sd2, action_ble_save_sd2, LV_EVENT_CLICKED, NULL);
    }
    ui_setup_dashboard_button(s_btn_ble_sd2,
        s_panel_racebox,
        197,
        184,
        MENU_PANEL_THIRD_BUTTON_WIDTH,
        42,
        "BLE SD2",
        lv_color_hex(0xff1565c0),
        lv_color_hex(0xff0d47a1));

    if (s_btn_ble_web == NULL)
    {
        s_btn_ble_web = lv_button_create(s_panel_racebox);
        label = lv_label_create(s_btn_ble_web);
        lv_label_set_text(label, "BLE WEB");
        lv_obj_center(label);
        lv_obj_add_event_cb(s_btn_ble_web, action_ble_save_web, LV_EVENT_CLICKED, NULL);
    }
    ui_setup_dashboard_button(s_btn_ble_web,
        s_panel_racebox,
        MENU_PANEL_ACTION_COLUMN_X,
        MENU_PANEL_ACTION_BOTTOM_Y,
        MENU_PANEL_THIRD_BUTTON_WIDTH,
        MENU_PANEL_ACTION_BUTTON_HEIGHT,
        "BLE WEB",
        lv_color_hex(0xff6a1b9a),
        lv_color_hex(0xff4a148c));

    if (s_btn_ble_reset == NULL)
    {
        s_btn_ble_reset = lv_button_create(s_panel_racebox);
        label = lv_label_create(s_btn_ble_reset);
        lv_label_set_text(label, "BLE RESET");
        lv_obj_center(label);
        lv_obj_add_event_cb(s_btn_ble_reset, action_ble_reset, LV_EVENT_CLICKED, NULL);
    }
    ui_setup_dashboard_button(s_btn_ble_reset,
        s_panel_racebox,
        MENU_PANEL_ACTION_COLUMN_X,
        MENU_PANEL_ACTION_TOP_Y,
        MENU_PANEL_THIRD_BUTTON_WIDTH,
        MENU_PANEL_ACTION_BUTTON_HEIGHT,
        "BLE RESET",
        lv_color_hex(0xff8d6e63),
        lv_color_hex(0xff5d4037));

    if (s_btn_gnss_sd1 == NULL)
    {
        s_btn_gnss_sd1 = lv_button_create(s_panel_gnss);
        label = lv_label_create(s_btn_gnss_sd1);
        lv_label_set_text(label, "GNSS SD1");
        lv_obj_center(label);
        lv_obj_add_event_cb(s_btn_gnss_sd1, action_gnss_save_sd1, LV_EVENT_CLICKED, NULL);
    }
    ui_setup_dashboard_button(s_btn_gnss_sd1,
        s_panel_gnss,
        18,
        184,
        MENU_PANEL_THIRD_BUTTON_WIDTH,
        42,
        "GNSS SD1",
        lv_color_hex(0xff2e7d32),
        lv_color_hex(0xff1b5e20));

    if (s_btn_gnss_sd2 == NULL)
    {
        s_btn_gnss_sd2 = lv_button_create(s_panel_gnss);
        label = lv_label_create(s_btn_gnss_sd2);
        lv_label_set_text(label, "GNSS SD2");
        lv_obj_center(label);
        lv_obj_add_event_cb(s_btn_gnss_sd2, action_gnss_save_sd2, LV_EVENT_CLICKED, NULL);
    }
    ui_setup_dashboard_button(s_btn_gnss_sd2,
        s_panel_gnss,
        197,
        184,
        MENU_PANEL_THIRD_BUTTON_WIDTH,
        42,
        "GNSS SD2",
        lv_color_hex(0xff1565c0),
        lv_color_hex(0xff0d47a1));

    if (s_btn_gnss_web == NULL)
    {
        s_btn_gnss_web = lv_button_create(s_panel_gnss);
        label = lv_label_create(s_btn_gnss_web);
        lv_label_set_text(label, "GNSS WEB");
        lv_obj_center(label);
        lv_obj_add_event_cb(s_btn_gnss_web, action_gnss_save_web, LV_EVENT_CLICKED, NULL);
    }
    ui_setup_dashboard_button(s_btn_gnss_web,
        s_panel_gnss,
        MENU_PANEL_ACTION_COLUMN_X,
        MENU_PANEL_ACTION_BOTTOM_Y,
        MENU_PANEL_THIRD_BUTTON_WIDTH,
        MENU_PANEL_ACTION_BUTTON_HEIGHT,
        "GNSS WEB",
        lv_color_hex(0xff6a1b9a),
        lv_color_hex(0xff4a148c));

    if (s_btn_gnss_reset == NULL)
    {
        s_btn_gnss_reset = lv_button_create(s_panel_gnss);
        label = lv_label_create(s_btn_gnss_reset);
        lv_label_set_text(label, "GNSS RESET");
        lv_obj_center(label);
        lv_obj_add_event_cb(s_btn_gnss_reset, action_gnss_reset, LV_EVENT_CLICKED, NULL);
    }
    ui_setup_dashboard_button(s_btn_gnss_reset,
        s_panel_gnss,
        MENU_PANEL_ACTION_COLUMN_X,
        MENU_PANEL_ACTION_TOP_Y,
        MENU_PANEL_THIRD_BUTTON_WIDTH,
        MENU_PANEL_ACTION_BUTTON_HEIGHT,
        "GNSS RESET",
        lv_color_hex(0xff8d6e63),
        lv_color_hex(0xff5d4037));
    }
/**
 * @brief Initialize all event listeners for the professional production UI
 */
void ui_logic_init_events(void)
{
    char dashboard_subtitle[96];

    // 0. Set default tab and add test button handlers
    if (objects.obj0)
    {
        lv_tabview_set_active(objects.obj0, 0, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(objects.obj0, lv_color_hex(UI_COLOR_BG), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(objects.obj0, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);

        g_settings_tab = lv_tabview_add_tab(objects.obj0, "Settings");
        if (g_settings_tab)
        {
            lv_obj_set_size(g_settings_tab, LV_PCT(100), LV_PCT(100));
            lv_obj_set_style_bg_color(g_settings_tab, lv_color_hex(UI_COLOR_BG), 0);
            lv_obj_set_style_bg_opa(g_settings_tab, LV_OPA_COVER, 0);
            lv_obj_set_style_pad_all(g_settings_tab, 0, 0);
            lv_obj_set_style_border_width(g_settings_tab, 0, 0);

            g_settings_screen = create_settings_screen(g_settings_tab);
            ESP_LOGI(TAG, "Settings tab created");
        }

        // Add Files tab to the tabview
        g_files_tab = lv_tabview_add_tab(objects.obj0, "Files");
        if (g_files_tab)
        {
            // Style the Files tab - fill entire space
            lv_obj_set_size(g_files_tab, LV_PCT(100), LV_PCT(100));
            lv_obj_set_style_bg_color(g_files_tab, lv_color_hex(UI_COLOR_BG), 0);
            lv_obj_set_style_bg_opa(g_files_tab, LV_OPA_COVER, 0);
            lv_obj_set_style_pad_all(g_files_tab, 0, 0);
            lv_obj_set_style_border_width(g_files_tab, 0, 0);

            // Create file manager immediately so it displays
            g_file_manager_screen = create_file_manager_screen(g_files_tab);

            ESP_LOGI(TAG, "Files tab created with file manager");
        }
    }

    if (objects.subtitle_dashbaord_1)
    {
        snprintf(dashboard_subtitle,
            sizeof(dashboard_subtitle),
            "System Dashboard (Version %s)",
            ui_get_firmware_version());
        lv_label_set_text(objects.subtitle_dashbaord_1, dashboard_subtitle);
    }

    // Connect SD Test Buttons
    if (objects.test_sd_card1_mount)
    {
        lv_obj_add_event_cb(objects.test_sd_card1_mount, action_test_sdcard, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    }
    if (objects.test_sd_card1_mount_2)
    { // This is actually Test SD Card 2 according to screens.c
        lv_obj_add_event_cb(objects.test_sd_card1_mount_2, action_test_sdcard, LV_EVENT_CLICKED, (void*)(intptr_t)2);
    }

    // Connect Format Buttons
    if (objects.format_sd_card1)
    {
        lv_obj_add_event_cb(objects.format_sd_card1, action_test_sdcard, LV_EVENT_CLICKED, (void*)(intptr_t)101);
    }
    if (objects.format_sd_card2)
    {
        lv_obj_add_event_cb(objects.format_sd_card2, action_test_sdcard, LV_EVENT_CLICKED, (void*)(intptr_t)102);
    }

    // Connect Export Log Button (Assuming slot > 200 trigger for export)
    // Map it to any secondary "Action" button if available

    // Connect WiFi Textareas to show keyboard
    if (objects.wifi_ssid_1)
    {
        lv_obj_add_event_cb(objects.wifi_ssid_1, action_show_keyboard, LV_EVENT_PRESSED, NULL);
        lv_obj_add_event_cb(objects.wifi_ssid_1, action_show_keyboard, LV_EVENT_FOCUSED, NULL);
        lv_textarea_set_one_line(objects.wifi_ssid_1, true);
        lv_textarea_set_placeholder_text(objects.wifi_ssid_1, "Network name");
        ui_style_input(objects.wifi_ssid_1);
    }
    if (objects.wifi_password_1)
    {
        lv_obj_add_event_cb(objects.wifi_password_1, action_show_keyboard, LV_EVENT_PRESSED, NULL);
        lv_obj_add_event_cb(objects.wifi_password_1, action_show_keyboard, LV_EVENT_FOCUSED, NULL);
        lv_textarea_set_one_line(objects.wifi_password_1, true);
        lv_textarea_set_placeholder_text(objects.wifi_password_1, "Password");
        ui_style_input(objects.wifi_password_1);
    }

    // Overlap and Layout Fixes:
    // 1. Hide redundant static "SD CARD" labels
    if (objects.sdcard_label_1_1) lv_obj_add_flag(objects.sdcard_label_1_1, LV_OBJ_FLAG_HIDDEN);
    if (objects.sdcard_label_2_1) lv_obj_add_flag(objects.sdcard_label_2_1, LV_OBJ_FLAG_HIDDEN);

    // 2. Reposition storage data and reduce text size for better layout
    if (objects.sdcard_storage_1_1)
    {
        lv_obj_set_pos(objects.sdcard_storage_1_1, 95, 126);
        lv_obj_set_width(objects.sdcard_storage_1_1, 180);
        lv_obj_set_style_text_font(objects.sdcard_storage_1_1, &lv_font_montserrat_24, 0);
    }
    if (objects.sd_card_title_2_1)
    {
        lv_obj_set_pos(objects.sd_card_title_2_1, 95, 314);
        lv_obj_set_width(objects.sd_card_title_2_1, 180);
        lv_obj_set_style_text_font(objects.sd_card_title_2_1, &lv_font_montserrat_24, 0);
    }

    // 3. Move checkboxes and reduce their text size
    if (objects.active_checkbox_sd_card_1_1)
    {
        lv_obj_set_pos(objects.active_checkbox_sd_card_1_1, 265, 126);
        lv_obj_set_style_text_font(objects.active_checkbox_sd_card_1_1, &lv_font_montserrat_20, 0);
    }
    if (objects.active_checkbox_sd_card_2_1)
    {
        lv_obj_set_pos(objects.active_checkbox_sd_card_2_1, 265, 314);
        lv_obj_set_style_text_font(objects.active_checkbox_sd_card_2_1, &lv_font_montserrat_20, 0);
    }

    // Enhanced Wi-Fi Screen Labels and Visual Hierarchy
    if (objects.ssid_1)
    {
        lv_obj_set_style_text_font(objects.ssid_1, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_color(objects.ssid_1, lv_color_hex(UI_COLOR_MUTED), 0);
        lv_label_set_text(objects.ssid_1, "NETWORK");
    }
    if (objects.wifi_subtitle_1)
    {
        lv_obj_set_style_text_font(objects.wifi_subtitle_1, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_color(objects.wifi_subtitle_1, lv_color_hex(UI_COLOR_MUTED), 0);
        lv_label_set_text(objects.wifi_subtitle_1, "PASSWORD");
    }
    if (objects.titile_wifi_screen_1)
    {
        lv_obj_set_style_text_font(objects.titile_wifi_screen_1, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(objects.titile_wifi_screen_1, lv_color_hex(UI_COLOR_TEXT), 0);
        lv_label_set_text(objects.titile_wifi_screen_1, "Wi-Fi Setup");
    }
    if (objects.brand_name_wifi_screen_1)
    {
        lv_obj_set_style_text_font(objects.brand_name_wifi_screen_1, &lv_font_montserrat_26, 0);
        lv_obj_set_style_text_color(objects.brand_name_wifi_screen_1, lv_color_hex(UI_COLOR_ACCENT), 0);
    }

    // Improve input_box_1 styling for better visual separation
    if (objects.input_box_1)
    {
        ui_style_surface(objects.input_box_1, lv_color_hex(UI_COLOR_SURFACE), 20);
        lv_obj_set_style_pad_all(objects.input_box_1, 18, 0);
        lv_obj_set_style_pad_row(objects.input_box_1, 20, 0);
        lv_obj_set_style_pad_column(objects.input_box_1, 8, 0);
    }

    // Global Style Fixes for Production Look
    // Remove default "yellow" focused style from main containers/tabs
    if (objects.obj1)
    { // Tab bar
        lv_obj_set_style_bg_color(objects.obj1, lv_color_hex(UI_COLOR_SURFACE), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(objects.obj1, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(objects.obj1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_pad_all(objects.obj1, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(objects.obj1, lv_color_hex(UI_COLOR_SURFACE), LV_PART_ITEMS | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(objects.obj1, LV_OPA_TRANSP, LV_PART_ITEMS | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(objects.obj1, lv_color_hex(UI_COLOR_MUTED), LV_PART_ITEMS | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(objects.obj1, lv_color_hex(UI_COLOR_ACCENT), LV_PART_ITEMS | LV_STATE_CHECKED);
        lv_obj_set_style_bg_opa(objects.obj1, LV_OPA_COVER, LV_PART_ITEMS | LV_STATE_CHECKED);
        lv_obj_set_style_text_color(objects.obj1, lv_color_hex(UI_COLOR_TEXT), LV_PART_ITEMS | LV_STATE_CHECKED);
        lv_obj_set_style_radius(objects.obj1, 16, LV_PART_ITEMS | LV_STATE_DEFAULT);
        lv_obj_add_flag(objects.obj1, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(objects.obj1, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    }
    if (objects.wifi_page)
    {
        lv_obj_set_style_bg_color(objects.wifi_page, lv_color_hex(UI_COLOR_BG), 0);
        lv_obj_set_style_bg_opa(objects.wifi_page, LV_OPA_COVER, 0);
        /* Prevent pressed-state color flash on Wi‑Fi page */
        lv_obj_set_style_bg_color(objects.wifi_page, lv_color_hex(UI_COLOR_BG), LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(objects.wifi_page, lv_color_hex(UI_COLOR_BG), LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
    }
    if (objects.bluetooth_page)
    {
        lv_obj_set_style_bg_color(objects.bluetooth_page, lv_color_hex(UI_COLOR_BG), 0);
        lv_obj_set_style_bg_opa(objects.bluetooth_page, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(objects.bluetooth_page, lv_color_hex(UI_COLOR_BG), LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(objects.bluetooth_page, lv_color_hex(UI_COLOR_BG), LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
    }
    if (objects.gnss_page)
    {
        lv_obj_set_style_bg_color(objects.gnss_page, lv_color_hex(UI_COLOR_BG), 0);
        lv_obj_set_style_bg_opa(objects.gnss_page, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(objects.gnss_page, lv_color_hex(UI_COLOR_BG), LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(objects.gnss_page, lv_color_hex(UI_COLOR_BG), LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
    }
    if (objects.menu)
    {
        lv_obj_set_style_bg_color(objects.menu, lv_color_hex(UI_COLOR_BG), 0);
        lv_obj_set_style_bg_opa(objects.menu, LV_OPA_COVER, 0);
    }

    ui_create_main_status_panels();
    ui_create_gpx_buttons();

    if (objects.ui_keyboard_1)
    {
        lv_obj_add_event_cb(objects.ui_keyboard_1, action_password_ssid_keyboad, LV_EVENT_ALL, NULL);
        ui_style_keyboard(objects.ui_keyboard_1);
    }

    if (s_gnss_dump_timer == NULL)
    {
        /* Keep monitor refresh light to preserve touch responsiveness. */
        s_gnss_dump_timer = lv_timer_create(gnss_dump_timer_cb, 120, NULL);
        if (s_gnss_dump_timer)
        {
            lv_timer_set_repeat_count(s_gnss_dump_timer, -1);
        }
    }
    ui_stream_init(&s_gnss_stream);
    ui_stream_stop(&s_gnss_stream);
    ui_window_reset(&s_gnss_window, objects.gnss_dump_panel);

#if CONFIG_RALLYBOX_RACEBOX_ENABLED
    if (s_bt_dump_timer == NULL)
    {
        s_bt_dump_timer = lv_timer_create(bluetooth_dump_timer_cb, 120, NULL);
        if (s_bt_dump_timer)
        {
            lv_timer_set_repeat_count(s_bt_dump_timer, -1);
        }
    }
    ui_stream_init(&s_bt_stream);
    ui_stream_stop(&s_bt_stream);
    racebox_set_rx_callback(racebox_ui_rx_cb, NULL);
#endif

    if (objects.input_box_1)
    {
        lv_obj_set_size(objects.input_box_1, 640, 264);
    }
    if (objects.wifi_conenct_button_1)
    {
        lv_obj_set_pos(objects.wifi_conenct_button_1, 12, 370);
        lv_obj_set_size(objects.wifi_conenct_button_1, 180, 54);
        lv_obj_set_style_radius(objects.wifi_conenct_button_1, 14, 0);
        lv_obj_set_style_shadow_width(objects.wifi_conenct_button_1, 0, 0);
        lv_obj_add_event_cb(objects.wifi_conenct_button_1, action_remove_saved_wifi, LV_EVENT_LONG_PRESSED, NULL);
    }

    if (objects.bluetooth_status_label)
    {
        lv_obj_set_style_text_font(objects.bluetooth_status_label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(objects.bluetooth_status_label, lv_color_hex(UI_COLOR_MUTED), 0);
    }

    if (objects.bluetooth_device_dropdown)
    {
        lv_obj_set_pos(objects.bluetooth_device_dropdown, 12, 170);
        lv_obj_set_size(objects.bluetooth_device_dropdown, 780, 54);
        lv_obj_set_style_bg_color(objects.bluetooth_device_dropdown, lv_color_hex(UI_COLOR_SURFACE), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(objects.bluetooth_device_dropdown, lv_color_hex(UI_COLOR_TEXT), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(objects.bluetooth_device_dropdown, lv_color_hex(UI_COLOR_BORDER), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(objects.bluetooth_device_dropdown, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_radius(objects.bluetooth_device_dropdown, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_dropdown_set_selected(objects.bluetooth_device_dropdown, 0);
    }

    if (objects.bluetooth_refresh_button)
    {
        lv_obj_set_pos(objects.bluetooth_refresh_button, 808, 170);
        lv_obj_set_size(objects.bluetooth_refresh_button, 160, 54);
        style_modern_button(objects.bluetooth_refresh_button, lv_color_hex(0xff546e7a), lv_color_hex(0xff37474f));
        lv_obj_add_event_cb(objects.bluetooth_refresh_button, action_bluetooth_refresh, LV_EVENT_CLICKED, NULL);
    }

    if (objects.bluetooth_connect_button)
    {
        lv_obj_set_pos(objects.bluetooth_connect_button, 984, 170);
        lv_obj_set_size(objects.bluetooth_connect_button, 160, 54);
        style_modern_button(objects.bluetooth_connect_button, lv_color_hex(0xff00acc1), lv_color_hex(0xff1565c0));
        lv_obj_add_event_cb(objects.bluetooth_connect_button, action_bluetooth_connect, LV_EVENT_CLICKED, NULL);
    }

    if (objects.bluetooth_page && s_bt_close_button == NULL)
    {
        s_bt_close_button = lv_button_create(objects.bluetooth_page);
        lv_obj_add_event_cb(s_bt_close_button, action_bluetooth_close, LV_EVENT_CLICKED, NULL);
        {
            lv_obj_t* label = lv_label_create(s_bt_close_button);
            lv_label_set_text(label, "CLOSE");
            lv_obj_center(label);
        }
    }
    if (s_bt_close_button)
    {
        lv_obj_set_pos(s_bt_close_button, 984, 10);
        lv_obj_set_size(s_bt_close_button, 160, 42);
        style_modern_button(s_bt_close_button, lv_color_hex(0xff546e7a), lv_color_hex(0xff37474f));
    }

#if CONFIG_RALLYBOX_RACEBOX_ENABLED
    if (objects.bluetooth_page)
    {
        if (s_bt_dump_panel == NULL)
        {
            s_bt_dump_panel = lv_textarea_create(objects.bluetooth_page);
            lv_obj_set_pos(s_bt_dump_panel, 12, 240);
            lv_obj_set_size(s_bt_dump_panel, 1132, 446);
            lv_textarea_set_one_line(s_bt_dump_panel, false);
            lv_textarea_set_max_length(s_bt_dump_panel, 16384);
            lv_textarea_set_placeholder_text(s_bt_dump_panel, "RaceBox BLE messages (NMEA/raw) will appear here...");
            ui_style_input(s_bt_dump_panel);
            lv_obj_set_style_text_font(s_bt_dump_panel, &lv_font_montserrat_14, 0);
            lv_obj_set_style_text_color(s_bt_dump_panel, lv_color_hex(UI_COLOR_TEXT), 0);
            lv_obj_clear_flag(s_bt_dump_panel, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_flag(s_bt_dump_panel, LV_OBJ_FLAG_SCROLLABLE);
        }
    }
#endif

    if (objects.gnss_status_label)
    {
        lv_obj_set_style_text_font(objects.gnss_status_label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(objects.gnss_status_label, lv_color_hex(UI_COLOR_MUTED), 0);
        lv_label_set_text(objects.gnss_status_label, s_gnss_status_text);
    }

    if (objects.gnss_baud_input)
    {
#if CONFIG_RALLYBOX_GNSS_ENABLED
        int gnss_baud = 0;
        int gnss_tx = 0;
        int gnss_rx = 0;
        char cfg_text[16];
#endif

        lv_obj_add_event_cb(objects.gnss_baud_input, action_show_keyboard, LV_EVENT_PRESSED, NULL);
        lv_obj_add_event_cb(objects.gnss_baud_input, action_show_keyboard, LV_EVENT_FOCUSED, NULL);
        lv_textarea_set_one_line(objects.gnss_baud_input, true);
        lv_textarea_set_accepted_chars(objects.gnss_baud_input, "0123456789");
        ui_style_input(objects.gnss_baud_input);

#if CONFIG_RALLYBOX_GNSS_ENABLED
        if (gnss_get_config(&gnss_baud, &gnss_tx, &gnss_rx) == ESP_OK)
        {
            snprintf(cfg_text, sizeof(cfg_text), "%d", gnss_baud);
            lv_textarea_set_text(objects.gnss_baud_input, cfg_text);
            if (objects.gnss_tx_input)
            {
                snprintf(cfg_text, sizeof(cfg_text), "%d", gnss_tx);
                lv_textarea_set_text(objects.gnss_tx_input, cfg_text);
            }
            if (objects.gnss_rx_input)
            {
                snprintf(cfg_text, sizeof(cfg_text), "%d", gnss_rx);
                lv_textarea_set_text(objects.gnss_rx_input, cfg_text);
            }
        }
#endif
    }

    if (objects.gnss_tx_input)
    {
        lv_obj_add_event_cb(objects.gnss_tx_input, action_show_keyboard, LV_EVENT_PRESSED, NULL);
        lv_obj_add_event_cb(objects.gnss_tx_input, action_show_keyboard, LV_EVENT_FOCUSED, NULL);
        lv_textarea_set_one_line(objects.gnss_tx_input, true);
        lv_textarea_set_accepted_chars(objects.gnss_tx_input, "0123456789");
        ui_style_input(objects.gnss_tx_input);
    }

    if (objects.gnss_rx_input)
    {
        lv_obj_add_event_cb(objects.gnss_rx_input, action_show_keyboard, LV_EVENT_PRESSED, NULL);
        lv_obj_add_event_cb(objects.gnss_rx_input, action_show_keyboard, LV_EVENT_FOCUSED, NULL);
        lv_textarea_set_one_line(objects.gnss_rx_input, true);
        lv_textarea_set_accepted_chars(objects.gnss_rx_input, "0123456789");
        ui_style_input(objects.gnss_rx_input);
    }

    if (objects.gnss_start_stop_button)
    {
        style_modern_button(objects.gnss_start_stop_button, lv_color_hex(0xff00897b), lv_color_hex(0xff005f73));
        lv_obj_add_event_cb(objects.gnss_start_stop_button, action_gnss_start_stop, LV_EVENT_CLICKED, NULL);
    }

    if (objects.gnss_dump_panel)
    {
        ui_style_input(objects.gnss_dump_panel);
        lv_obj_set_style_text_font(objects.gnss_dump_panel, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(objects.gnss_dump_panel, lv_color_hex(UI_COLOR_TEXT), 0);
        lv_obj_clear_flag(objects.gnss_dump_panel, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(objects.gnss_dump_panel, LV_OBJ_FLAG_SCROLLABLE);
    }

    // Enhanced SD Card Test Buttons
    if (objects.test_sd_card1_mount)
    {
        style_modern_button(objects.test_sd_card1_mount, lv_color_hex(0xff00acc1), lv_color_hex(0xff0077ff));
    }
    if (objects.test_sd_card1_mount_2)
    {
        style_modern_button(objects.test_sd_card1_mount_2, lv_color_hex(0xff00acc1), lv_color_hex(0xff0077ff));
    }

    // Enhanced SD Card Format Buttons
    if (objects.format_sd_card1)
    {
        style_modern_button(objects.format_sd_card1, lv_color_hex(0xffffc107), lv_color_hex(0xffff6f00));
    }
    if (objects.format_sd_card2)
    {
        style_modern_button(objects.format_sd_card2, lv_color_hex(0xffffc107), lv_color_hex(0xffff6f00));
    }

    // Overall dashboard polish
    if (objects.dashboard)
    {
        lv_obj_set_style_bg_color(objects.dashboard, lv_color_hex(UI_COLOR_BG), 0);
        lv_obj_set_style_bg_opa(objects.dashboard, LV_OPA_COVER, 0);
        /* Keep background steady when pressed to avoid flashing colors */
        lv_obj_set_style_bg_color(objects.dashboard, lv_color_hex(UI_COLOR_BG), LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(objects.dashboard, lv_color_hex(UI_COLOR_BG), LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
    }
    if (objects.menu)
    {
        lv_obj_set_style_bg_color(objects.menu, lv_color_hex(UI_COLOR_BG), 0);
        lv_obj_set_style_bg_opa(objects.menu, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(objects.menu, lv_color_hex(UI_COLOR_BG), LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(objects.menu, lv_color_hex(UI_COLOR_BG), LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
    }

    if (objects.obj4)
    {
        lv_obj_set_pos(objects.obj4, 12, 452);
        lv_obj_set_width(objects.obj4, 1120);
        lv_label_set_long_mode(objects.obj4, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_font(objects.obj4, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(objects.obj4, lv_color_hex(UI_COLOR_MUTED), 0);
    }

    system_status_t status = system_monitor_get_status();
    ui_load_saved_wifi_credentials();
    ui_update_storage_dependent_controls(&status);
    ui_refresh_wifi_controls(&status);
    ui_refresh_bluetooth_controls(&status);
    ui_refresh_gnss_dump();

    ESP_LOGI(TAG, "UI Event Handlers Initialized with Modern Styling");
    }
