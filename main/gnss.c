#include "gnss.h"

#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "bsp/esp32_p4_wifi6_touch_lcd_x.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "system_monitor.h"

#define GNSS_UART_PORT ((uart_port_t)CONFIG_RALLYBOX_GNSS_UART_PORT)
#define GNSS_DEFAULT_BAUD_RATE CONFIG_RALLYBOX_GNSS_UART_BAUD_RATE
#define GNSS_DEFAULT_RX_PIN CONFIG_RALLYBOX_GNSS_UART_RX_GPIO
#define GNSS_DEFAULT_TX_PIN CONFIG_RALLYBOX_GNSS_UART_TX_GPIO
#define GNSS_BUFFER_SIZE 256
#define GNSS_LINE_SIZE 128
#define GNSS_MAX_DEBUG_LOG_MESSAGES 10
#define GNSS_NVS_NAMESPACE "gnss"
#define GNSS_NVS_KEY_BAUD "baud"
#define GNSS_NVS_KEY_TX "tx"
#define GNSS_NVS_KEY_RX "rx"

static const char* TAG = "GNSS";

typedef struct
{
  bool initialized;
  bool fix_valid;
  uint8_t fix_quality;
  uint8_t satellites;
  double latitude_deg;
  double longitude_deg;
  float speed_kph;
  float heading_deg;
  float altitude_m;
  char last_sentence[8];
  char status_text[64];
} gnss_state_t;

static gnss_state_t s_gnss_state = {
    .initialized = false,
    .fix_valid = false,
    .fix_quality = 0,
    .satellites = 0,
    .latitude_deg = 0.0,
    .longitude_deg = 0.0,
    .speed_kph = 0.0f,
    .heading_deg = 0.0f,
    .altitude_m = 0.0f,
    .last_sentence = "",
    .status_text = "Idle",
};

static uint32_t s_gnss_sentence_count = 0;
static uint32_t s_gnss_debug_log_count = 0;
static TickType_t s_last_gnss_rx_tick = 0;
static bool s_gnss_debug_logging_suppressed = false;
static int s_gnss_baud_rate = GNSS_DEFAULT_BAUD_RATE;
static int s_gnss_tx_gpio = GNSS_DEFAULT_TX_PIN;
static int s_gnss_rx_gpio = GNSS_DEFAULT_RX_PIN;
static bool s_gnss_running = false;
static bool s_gnss_uart_driver_installed = false;
static volatile TaskHandle_t s_gnss_task_handle = NULL;
static SemaphoreHandle_t s_gnss_lock = NULL;
static gnss_sentence_callback_t s_sentence_cb = NULL;
static void* s_sentence_cb_ctx = NULL;

static bool gnss_pin_is_reserved(gpio_num_t pin)
{
  /* Keep console UART pins reserved; re-routing them can destabilize logging and boot flow. */
  if (pin == GPIO_NUM_37 || pin == GPIO_NUM_38)
  {
    return true;
  }

  /* SD card control pins and external SD SPI pins are reserved by storage subsystem. */
  if (pin == SD1_CONTROL_PIN || pin == SD2_CONTROL_PIN ||
    pin == SD2_PIN_CLK || pin == SD2_PIN_MOSI || pin == SD2_PIN_MISO || pin == SD2_PIN_CS)
  {
    return true;
  }

  /* Avoid low-numbered strapping/boot sensitive pins for runtime GNSS UART routing. */
  if (pin >= GPIO_NUM_0 && pin <= GPIO_NUM_6)
  {
    return true;
  }

  if (pin == BSP_I2C_SCL || pin == BSP_I2C_SDA ||
    pin == BSP_I2S_SCLK || pin == BSP_I2S_MCLK ||
    pin == BSP_I2S_LCLK || pin == BSP_I2S_DOUT || pin == BSP_I2S_DSIN ||
    pin == BSP_POWER_AMP_IO ||
    pin == BSP_LCD_BACKLIGHT || pin == BSP_LCD_RST ||
    pin == BSP_SD_D0 || pin == BSP_SD_D1 || pin == BSP_SD_D2 || pin == BSP_SD_D3 ||
    pin == BSP_SD_CMD || pin == BSP_SD_CLK)
  {
    return true;
  }

  /* ESP-Hosted SDIO transport pins used by this project on P4. */
  if (pin == GPIO_NUM_14 || pin == GPIO_NUM_15 || pin == GPIO_NUM_16 ||
    pin == GPIO_NUM_17 || pin == GPIO_NUM_18 || pin == GPIO_NUM_19 ||
    pin == GPIO_NUM_54)
  {
    return true;
  }

  return false;
}

static void gnss_log_debug_limited(const char* format, ...)
{
  va_list args;

  if (s_gnss_debug_log_count < GNSS_MAX_DEBUG_LOG_MESSAGES)
  {
    ++s_gnss_debug_log_count;
    va_start(args, format);
    esp_log_writev(ESP_LOG_INFO, TAG, format, args);
    va_end(args);
    return;
  }

  if (!s_gnss_debug_logging_suppressed)
  {
    s_gnss_debug_logging_suppressed = true;
    ESP_LOGI(TAG,
      "GNSS debug logging suppressed after %d messages",
      GNSS_MAX_DEBUG_LOG_MESSAGES);
  }
}

static bool gnss_ensure_lock(void)
{
  if (s_gnss_lock == NULL)
  {
    s_gnss_lock = xSemaphoreCreateMutex();
  }

  return s_gnss_lock != NULL;
}

static bool gnss_validate_config(int baud_rate, int tx_gpio, int rx_gpio)
{
  if (baud_rate < 1200 || baud_rate > 921600)
  {
    return false;
  }

  if (tx_gpio < 0 || tx_gpio > 54 || rx_gpio < 0 || rx_gpio > 54 || tx_gpio == rx_gpio)
  {
    return false;
  }

  return true;
}

static esp_err_t gnss_load_config_from_nvs(int* baud_rate, int* tx_gpio, int* rx_gpio)
{
  nvs_handle_t handle;
  esp_err_t err;
  int32_t baud = GNSS_DEFAULT_BAUD_RATE;
  int32_t tx = GNSS_DEFAULT_TX_PIN;
  int32_t rx = GNSS_DEFAULT_RX_PIN;

  err = nvs_open(GNSS_NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK)
  {
    *baud_rate = GNSS_DEFAULT_BAUD_RATE;
    *tx_gpio = GNSS_DEFAULT_TX_PIN;
    *rx_gpio = GNSS_DEFAULT_RX_PIN;
    return ESP_OK;
  }

  if (nvs_get_i32(handle, GNSS_NVS_KEY_BAUD, &baud) != ESP_OK)
  {
    baud = GNSS_DEFAULT_BAUD_RATE;
  }
  if (nvs_get_i32(handle, GNSS_NVS_KEY_TX, &tx) != ESP_OK)
  {
    tx = GNSS_DEFAULT_TX_PIN;
  }
  if (nvs_get_i32(handle, GNSS_NVS_KEY_RX, &rx) != ESP_OK)
  {
    rx = GNSS_DEFAULT_RX_PIN;
  }

  nvs_close(handle);

  if (!gnss_validate_config((int)baud, (int)tx, (int)rx))
  {
    *baud_rate = GNSS_DEFAULT_BAUD_RATE;
    *tx_gpio = GNSS_DEFAULT_TX_PIN;
    *rx_gpio = GNSS_DEFAULT_RX_PIN;
    return ESP_OK;
  }

  *baud_rate = (int)baud;
  *tx_gpio = (int)tx;
  *rx_gpio = (int)rx;
  return ESP_OK;
}

static esp_err_t gnss_save_config_to_nvs(int baud_rate, int tx_gpio, int rx_gpio)
{
  nvs_handle_t handle;
  esp_err_t err = nvs_open(GNSS_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK)
  {
    return err;
  }

  err = nvs_set_i32(handle, GNSS_NVS_KEY_BAUD, (int32_t)baud_rate);
  if (err == ESP_OK)
  {
    err = nvs_set_i32(handle, GNSS_NVS_KEY_TX, (int32_t)tx_gpio);
  }
  if (err == ESP_OK)
  {
    err = nvs_set_i32(handle, GNSS_NVS_KEY_RX, (int32_t)rx_gpio);
  }
  if (err == ESP_OK)
  {
    err = nvs_commit(handle);
  }

  nvs_close(handle);
  return err;
}

static bool gnss_parse_coordinate(const char* value, const char* hemisphere, double* out_deg)
{
  if (!value || !hemisphere || !out_deg || value[0] == '\0' || hemisphere[0] == '\0')
  {
    return false;
  }

  double raw = strtod(value, NULL);
  int degrees = (int)(raw / 100.0);
  double minutes = raw - ((double)degrees * 100.0);
  double decimal = degrees + (minutes / 60.0);

  if (hemisphere[0] == 'S' || hemisphere[0] == 'W')
  {
    decimal = -decimal;
  }

  *out_deg = decimal;
  return true;
}

static void gnss_publish_state(void)
{
  system_monitor_set_gnss_status(s_gnss_state.initialized,
    s_gnss_state.fix_valid,
    s_gnss_state.fix_quality,
    s_gnss_state.satellites,
    s_gnss_state.latitude_deg,
    s_gnss_state.longitude_deg,
    s_gnss_state.speed_kph,
    s_gnss_state.heading_deg,
    s_gnss_state.altitude_m,
    "UART NMEA",
    s_gnss_state.last_sentence,
    s_gnss_state.status_text);
}

static void gnss_parse_sentence(char* line)
{
  char sentence_text[GNSS_LINE_SIZE];
  char* fields[20] = { 0 };
  int field_count = 0;
  char* saveptr = NULL;
  char* checksum = strchr(line, '*');

  snprintf(sentence_text, sizeof(sentence_text), "%s", line);

  if (checksum)
  {
    *checksum = '\0';
  }

  for (char* token = strtok_r(line, ",", &saveptr);
    token && field_count < (int)(sizeof(fields) / sizeof(fields[0]));
    token = strtok_r(NULL, ",", &saveptr))
  {
    fields[field_count++] = token;
  }

  if (field_count == 0 || fields[0] == NULL || strlen(fields[0]) < 6 || fields[0][0] != '$')
  {
    return;
  }

  const char* sentence = fields[0] + 3;
  ++s_gnss_sentence_count;
  gnss_log_debug_limited("NMEA[%lu]: %s", (unsigned long)s_gnss_sentence_count, sentence_text);
  if (s_sentence_cb)
  {
    s_sentence_cb(sentence_text, s_sentence_cb_ctx);
  }
  snprintf(s_gnss_state.last_sentence, sizeof(s_gnss_state.last_sentence), "%s", sentence);

  if (strcmp(sentence, "GGA") == 0 && field_count > 9)
  {
    double latitude = s_gnss_state.latitude_deg;
    double longitude = s_gnss_state.longitude_deg;
    uint8_t fix_quality = (uint8_t)atoi(fields[6]);
    uint8_t satellites = (uint8_t)atoi(fields[7]);

    gnss_parse_coordinate(fields[2], fields[3], &latitude);
    gnss_parse_coordinate(fields[4], fields[5], &longitude);

    s_gnss_state.fix_quality = fix_quality;
    s_gnss_state.fix_valid = (fix_quality > 0);
    s_gnss_state.satellites = satellites;
    s_gnss_state.latitude_deg = latitude;
    s_gnss_state.longitude_deg = longitude;
    s_gnss_state.altitude_m = strtof(fields[9], NULL);
    gnss_log_debug_limited(
      "Parsed GGA fix_quality=%u sats=%u lat=%.6f lon=%.6f alt=%.2f",
      s_gnss_state.fix_quality,
      s_gnss_state.satellites,
      s_gnss_state.latitude_deg,
      s_gnss_state.longitude_deg,
      s_gnss_state.altitude_m);
    snprintf(s_gnss_state.status_text, sizeof(s_gnss_state.status_text), "%s",
      s_gnss_state.fix_valid ? "GNSS fix valid" : "Waiting for GNSS fix");
    gnss_publish_state();
    return;
  }

  if (strcmp(sentence, "RMC") == 0 && field_count > 8)
  {
    double latitude = s_gnss_state.latitude_deg;
    double longitude = s_gnss_state.longitude_deg;
    bool valid = (fields[2][0] == 'A');

    gnss_parse_coordinate(fields[3], fields[4], &latitude);
    gnss_parse_coordinate(fields[5], fields[6], &longitude);

    s_gnss_state.fix_valid = valid;
    s_gnss_state.latitude_deg = latitude;
    s_gnss_state.longitude_deg = longitude;
    s_gnss_state.speed_kph = strtof(fields[7], NULL) * 1.852f;
    s_gnss_state.heading_deg = strtof(fields[8], NULL);
    gnss_log_debug_limited(
      "Parsed RMC valid=%d lat=%.6f lon=%.6f speed=%.2f kph heading=%.2f",
      s_gnss_state.fix_valid,
      s_gnss_state.latitude_deg,
      s_gnss_state.longitude_deg,
      s_gnss_state.speed_kph,
      s_gnss_state.heading_deg);
    snprintf(s_gnss_state.status_text, sizeof(s_gnss_state.status_text), "%s",
      s_gnss_state.fix_valid ? "GNSS motion data valid" : "RMC received without fix");
    gnss_publish_state();
  }
}

static void gnss_task(void* arg)
{
  uint8_t rx_buffer[GNSS_BUFFER_SIZE];
  char line_buffer[GNSS_LINE_SIZE];
  size_t line_length = 0;

  (void)arg;

  s_gnss_state.initialized = true;
  s_last_gnss_rx_tick = xTaskGetTickCount();
  ESP_LOGI(TAG, "GNSS task started, waiting for UART data");
  snprintf(s_gnss_state.status_text, sizeof(s_gnss_state.status_text), "%s", "Listening for NMEA");
  gnss_publish_state();

  while (s_gnss_running)
  {
    int bytes_read = uart_read_bytes(GNSS_UART_PORT, rx_buffer, sizeof(rx_buffer), pdMS_TO_TICKS(30));
    if (bytes_read <= 0)
    {
      TickType_t now = xTaskGetTickCount();
      if ((now - s_last_gnss_rx_tick) >= pdMS_TO_TICKS(5000))
      {
        ESP_LOGW(TAG,
          "No GNSS UART data for %lu ms on RX=%d TX=%d @ %d baud",
          (unsigned long)(pdTICKS_TO_MS(now - s_last_gnss_rx_tick)),
          s_gnss_rx_gpio,
          s_gnss_tx_gpio,
          s_gnss_baud_rate);
        s_last_gnss_rx_tick = now;
      }
      continue;
    }

    s_last_gnss_rx_tick = xTaskGetTickCount();
    gnss_log_debug_limited("Received %d GNSS UART bytes", bytes_read);

    for (int i = 0; i < bytes_read; ++i)
    {
      char ch = (char)rx_buffer[i];
      if (ch == '\r')
      {
        continue;
      }

      if (ch == '\n')
      {
        line_buffer[line_length] = '\0';
        if (line_length > 0)
        {
          gnss_parse_sentence(line_buffer);
        }
        line_length = 0;
        continue;
      }

      if (line_length < (sizeof(line_buffer) - 1))
      {
        line_buffer[line_length++] = ch;
      }
      else
      {
        line_length = 0;
      }
    }
  }

  s_gnss_state.initialized = false;
  snprintf(s_gnss_state.status_text, sizeof(s_gnss_state.status_text), "%s", "Stopped");
  gnss_publish_state();

  s_gnss_task_handle = NULL;
  vTaskDelete(NULL);
}

esp_err_t gnss_start_with_config(int baud_rate, int tx_gpio, int rx_gpio, bool persist)
{
  TaskHandle_t task_handle = NULL;
  const uart_config_t uart_config = {
      .baud_rate = baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  if (!gnss_validate_config(baud_rate, tx_gpio, rx_gpio))
  {
    return ESP_ERR_INVALID_ARG;
  }

  if (gnss_pin_is_reserved((gpio_num_t)rx_gpio) || gnss_pin_is_reserved((gpio_num_t)tx_gpio))
  {
    ESP_LOGE(TAG,
      "GNSS UART pin config conflicts with board-critical pins (RX=%d TX=%d)."
      " Reconfigure GNSS pins to free GPIOs.",
      rx_gpio,
      tx_gpio);
    return ESP_ERR_INVALID_ARG;
  }

  if (!gnss_ensure_lock())
  {
    return ESP_ERR_NO_MEM;
  }

  if (xSemaphoreTake(s_gnss_lock, pdMS_TO_TICKS(200)) != pdTRUE)
  {
    return ESP_ERR_TIMEOUT;
  }

  if (s_gnss_running)
  {
    xSemaphoreGive(s_gnss_lock);
    return ESP_ERR_INVALID_STATE;
  }

  s_gnss_baud_rate = baud_rate;
  s_gnss_tx_gpio = tx_gpio;
  s_gnss_rx_gpio = rx_gpio;

  ESP_LOGI(TAG, "Initializing GNSS UART on RX=%d TX=%d @ %d baud",
    s_gnss_rx_gpio, s_gnss_tx_gpio, s_gnss_baud_rate);

  esp_err_t ret = uart_driver_install(GNSS_UART_PORT, GNSS_BUFFER_SIZE * 2, 0, 0, NULL, 0);
  if (ret != ESP_OK)
  {
    xSemaphoreGive(s_gnss_lock);
    return ret;
  }
  s_gnss_uart_driver_installed = true;

  ret = uart_param_config(GNSS_UART_PORT, &uart_config);
  if (ret != ESP_OK)
  {
    uart_driver_delete(GNSS_UART_PORT);
    s_gnss_uart_driver_installed = false;
    xSemaphoreGive(s_gnss_lock);
    return ret;
  }

  ret = uart_set_pin(GNSS_UART_PORT, s_gnss_tx_gpio, s_gnss_rx_gpio,
    UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (ret != ESP_OK)
  {
    uart_driver_delete(GNSS_UART_PORT);
    s_gnss_uart_driver_installed = false;
    xSemaphoreGive(s_gnss_lock);
    return ret;
  }

  s_gnss_running = true;
  if (xTaskCreate(gnss_task, "gnss_task", 4096, NULL, 4, &task_handle) != pdPASS)
  {
    s_gnss_running = false;
    if (s_gnss_uart_driver_installed)
    {
      uart_driver_delete(GNSS_UART_PORT);
      s_gnss_uart_driver_installed = false;
    }
    xSemaphoreGive(s_gnss_lock);
    return ESP_FAIL;
  }

  s_gnss_task_handle = task_handle;

  if (persist)
  {
    esp_err_t save_ret = gnss_save_config_to_nvs(s_gnss_baud_rate, s_gnss_tx_gpio, s_gnss_rx_gpio);
    if (save_ret != ESP_OK)
    {
      ESP_LOGW(TAG, "Failed to persist GNSS config to NVS: %s", esp_err_to_name(save_ret));
    }
  }

  xSemaphoreGive(s_gnss_lock);

  return ESP_OK;
}

esp_err_t gnss_stop(void)
{
  if (!gnss_ensure_lock())
  {
    return ESP_ERR_NO_MEM;
  }

  if (xSemaphoreTake(s_gnss_lock, pdMS_TO_TICKS(200)) != pdTRUE)
  {
    return ESP_ERR_TIMEOUT;
  }

  if (!s_gnss_running)
  {
    xSemaphoreGive(s_gnss_lock);
    return ESP_OK;
  }

  /* Signal gnss_task() to leave its read loop, then release the lock so it can
   * finish the in-flight uart_read_bytes() and self-delete. */
  s_gnss_running = false;
  xSemaphoreGive(s_gnss_lock);

  /* Wait for the task to actually exit before tearing down the UART driver.
   * Deleting the driver while the task is still blocked in uart_read_bytes()
   * frees structures the read references and reboots the device. The task uses
   * a 30 ms read timeout, so it observes the flag and exits well within 500 ms. */
  for (int waited_ms = 0; s_gnss_task_handle != NULL && waited_ms < 500; waited_ms += 10)
  {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (xSemaphoreTake(s_gnss_lock, pdMS_TO_TICKS(200)) != pdTRUE)
  {
    return ESP_ERR_TIMEOUT;
  }

  if (s_gnss_uart_driver_installed)
  {
    uart_driver_delete(GNSS_UART_PORT);
    s_gnss_uart_driver_installed = false;
  }

  xSemaphoreGive(s_gnss_lock);

  s_gnss_state.initialized = false;
  snprintf(s_gnss_state.status_text, sizeof(s_gnss_state.status_text), "%s", "Stopped");
  gnss_publish_state();

  return ESP_OK;
}

bool gnss_is_running(void)
{
  return s_gnss_running;
}

esp_err_t gnss_get_config(int* baud_rate, int* tx_gpio, int* rx_gpio)
{
  int loaded_baud;
  int loaded_tx;
  int loaded_rx;

  if (!baud_rate || !tx_gpio || !rx_gpio)
  {
    return ESP_ERR_INVALID_ARG;
  }

  loaded_baud = s_gnss_baud_rate;
  loaded_tx = s_gnss_tx_gpio;
  loaded_rx = s_gnss_rx_gpio;

  if (!s_gnss_running)
  {
    gnss_load_config_from_nvs(&loaded_baud, &loaded_tx, &loaded_rx);
    s_gnss_baud_rate = loaded_baud;
    s_gnss_tx_gpio = loaded_tx;
    s_gnss_rx_gpio = loaded_rx;
  }

  *baud_rate = loaded_baud;
  *tx_gpio = loaded_tx;
  *rx_gpio = loaded_rx;
  return ESP_OK;
}

void gnss_set_sentence_callback(gnss_sentence_callback_t callback, void* user_ctx)
{
  s_sentence_cb = callback;
  s_sentence_cb_ctx = user_ctx;
}

esp_err_t gnss_init(void)
{
  int baud_rate = GNSS_DEFAULT_BAUD_RATE;
  int tx_gpio = GNSS_DEFAULT_TX_PIN;
  int rx_gpio = GNSS_DEFAULT_RX_PIN;

  gnss_load_config_from_nvs(&baud_rate, &tx_gpio, &rx_gpio);
  s_gnss_baud_rate = baud_rate;
  s_gnss_tx_gpio = tx_gpio;
  s_gnss_rx_gpio = rx_gpio;

  return gnss_start_with_config(baud_rate, tx_gpio, rx_gpio, false);
}