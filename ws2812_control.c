#include "ws2812_control.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

// Configure these based on your project needs using menuconfig ********
#define LED_RMT_TX_CHANNEL CONFIG_WS2812_LED_RMT_TX_CHANNEL
#define LED_RMT_TX_GPIO    CONFIG_WS2812_LED_RMT_TX_GPIO
// ****************************************************

#if CONFIG_WS2812_LED_TYPE_RGB
#define BITS_PER_LED_CMD  24
#define BYTES_PER_LED_CMD 3
#elif CONFIG_WS2812_LED_TYPE_RGBW
#define BITS_PER_LED_CMD  32
#define BYTES_PER_LED_CMD 4
#endif

#define LED_BUFFER_ITEMS (NUM_LEDS * BYTES_PER_LED_CMD)

// These values are determined by measuring pulse timing with logic analyzer and
// adjusting to match datasheet.
#define T0H CONFIG_WS2812_T0H // 0 bit high time
#define T1H CONFIG_WS2812_T1H // 1 bit high time
#define T0L CONFIG_WS2812_T0L // low time for either bit
#define T1L CONFIG_WS2812_T1L

// Tag for log messages
static const char *TAG = "NeoPixel WS2812 Driver";

// This is the buffer which the hw peripheral will access while pulsing the
// output pin
static uint8_t led_data_buffer[LED_BUFFER_ITEMS];
static rmt_channel_handle_t _txChan;
static rmt_encoder_handle_t _txBytesEncoder;
static const rmt_transmit_config_t _rmtTxConfig = {
  .loop_count = 0,
  .flags =
    {
      .eot_level = 0,
    },
};

static rmt_symbol_word_t _bit0_symbol = {
  .duration0 = T0H,
  .level0    = 1,
  .duration1 = T0L,
  .level1    = 0,
};

static rmt_symbol_word_t _bit1_symbol = {
  .duration0 = T1H,
  .level0    = 1,
  .duration1 = T1L,
  .level1    = 0,
};

static void setup_rmt_data_buffer(struct led_state new_state);

esp_err_t ws2812_control_init(void) {
  rmt_bytes_encoder_config_t bytes_encoder_config = {
    .bit0 = _bit0_symbol,
    .bit1 = _bit1_symbol,
    .flags =
      {
        .msb_first = true,
      },
  };
  ESP_RETURN_ON_ERROR(
    rmt_new_bytes_encoder(&bytes_encoder_config, &_txBytesEncoder),
    TAG,
    "Failed to install bytes encoder");

  rmt_tx_channel_config_t config = {
    .gpio_num          = LED_RMT_TX_GPIO,
    .clk_src           = RMT_CLK_SRC_DEFAULT,
    .resolution_hz     = 40000000, // 40 MHz, 1 tick = 25ns
    .mem_block_symbols = 128,
    .trans_queue_depth = 4,
    .flags =
      {
        .invert_out   = false,
        .with_dma     = false,
        .io_loop_back = false,
        // Disable open-drain mode.
        .io_od_mode = false,
      },
  };

  ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&config, &_txChan),
                      TAG,
                      "Failed to install RMT TX channel");

  // Disable the pull-up resistor to make the edge transitions fast enough.
  gpio_set_pull_mode(LED_RMT_TX_GPIO, GPIO_FLOATING);

  ESP_RETURN_ON_ERROR(
    rmt_enable(_txChan), TAG, "Failed to enable RMT TX Channel");

  return ESP_OK;
}

esp_err_t ws2812_write_leds(struct led_state new_state) {
  setup_rmt_data_buffer(new_state);

  ESP_RETURN_ON_ERROR(rmt_transmit(_txChan,
                                   _txBytesEncoder,
                                   led_data_buffer,
                                   LED_BUFFER_ITEMS,
                                   &_rmtTxConfig),
                      TAG,
                      "Unable to transmit LED State");

  ESP_RETURN_ON_ERROR(rmt_tx_wait_all_done(_txChan, 50),
                      TAG,
                      "Failed while wait form LED state to transmit");

  return ESP_OK;
}

static void setup_rmt_data_buffer(struct led_state new_state) {
  for (uint32_t led = 0; led < NUM_LEDS; led++) {
    uint32_t bytes_to_send = new_state.leds[led];
    if (BYTES_PER_LED_CMD > 3) {
      led_data_buffer[(led * BYTES_PER_LED_CMD)]     = bytes_to_send >> 24;
      led_data_buffer[(led * BYTES_PER_LED_CMD) + 1] = bytes_to_send >> 16;
      led_data_buffer[(led * BYTES_PER_LED_CMD) + 2] = bytes_to_send >> 8;
      led_data_buffer[(led * BYTES_PER_LED_CMD) + 3] = bytes_to_send;
    }
    else {
      led_data_buffer[(led * BYTES_PER_LED_CMD)]     = bytes_to_send >> 16;
      led_data_buffer[(led * BYTES_PER_LED_CMD) + 1] = bytes_to_send >> 8;
      led_data_buffer[(led * BYTES_PER_LED_CMD) + 2] = bytes_to_send;
    }
  }
}
