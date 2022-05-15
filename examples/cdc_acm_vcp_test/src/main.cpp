//************************************************************
// this is a simple example that uses the painlessMesh library
// 
// This example shows how to build a mesh with named nodes
//
//************************************************************
#include <arduino.h>

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "cp210x_usb.hpp"
#include "ftdi_usb.hpp"
#include "usb/usb_host.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


using namespace esp_usb;

// Change these values to match your needs
#define EXAMPLE_BAUDRATE     (256000)
#define EXAMPLE_STOP_BITS    (0)      // 0: 1 stopbit, 1: 1.5 stopbits, 2: 2 stopbits
#define EXAMPLE_PARITY       (0)      // 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space
#define EXAMPLE_DATA_BITS    (8)

static const char *TAG = "VCP example";

static void handle_rx(uint8_t *data, size_t data_len, void *arg)
{
    printf("%.*s", data_len, data);
}

static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
        case CDC_ACM_HOST_ERROR:
            ESP_LOGE(TAG, "CDC-ACM error has occurred, err_no = %d", event->data.error);
            break;
        case CDC_ACM_HOST_DEVICE_DISCONNECTED:
            ESP_LOGI(TAG, "Device suddenly disconnected");
            break;
        case CDC_ACM_HOST_SERIAL_STATE:
            ESP_LOGI(TAG, "serial state notif 0x%04X", event->data.serial_state.val);
            break;
        case CDC_ACM_HOST_NETWORK_CONNECTION:
        default: break;
    }
}

void usb_lib_task(void *arg)
{
    while (1) {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: All devices freed");
            // Continue handling USB events to allow device reconnection
        }
    }
}

CP210x *vcp;
void setup() {
  Serial.begin(115200);

  while(!Serial);

  delay(100);

  Serial.println("program has start.");

  esp_log_level_set(TAG, ESP_LOG_DEBUG);

  //Install USB Host driver. Should only be called once in entire application
  ESP_LOGI(TAG, "Installing USB Host");
  const usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };
  ESP_ERROR_CHECK(usb_host_install(&host_config));

  // Create a task that will handle USB library events
  xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL);

  ESP_LOGI(TAG, "Installing CDC-ACM driver");
  ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

  const cdc_acm_host_device_config_t dev_config = {
      .connection_timeout_ms = 10000,
      .out_buffer_size = 64,
      .event_cb = handle_event,
      .data_cb = handle_rx,
      .user_arg = NULL,
  };

#if defined(CONFIG_EXAMPLE_USE_FTDI)
  FT23x *vcp;
  // try {
      ESP_LOGI(TAG, "Opening FT232 UART device");
      vcp = FT23x::open_ftdi(FTDI_FT232_PID, &dev_config);
  // }
#else
  try {
      ESP_LOGI(TAG, "Opening CP210X device");
      vcp = CP210x::open_cp210x(CP210X_PID, &dev_config);
  }
#endif
  catch (esp_err_t err) {
      ESP_LOGE(TAG, "The required device was not opened.\nExiting...");
      ESP.restart();
  }

  ESP_LOGI(TAG, "Setting up line coding");
  cdc_acm_line_coding_t line_coding = {
      .dwDTERate = EXAMPLE_BAUDRATE,
      .bCharFormat = EXAMPLE_STOP_BITS,
      .bParityType = EXAMPLE_PARITY,
      .bDataBits = EXAMPLE_DATA_BITS,
  };
  ESP_ERROR_CHECK(vcp->line_coding_set(&line_coding));
}

void loop()
{
  while (Serial.available()) {
    String str = Serial.readString();
    ESP_ERROR_CHECK(vcp->tx_blocking((uint8_t *)str.c_str(), str.length()));
  }
  /*
  ESP_ERROR_CHECK(vcp->set_control_line_state(false, true));
  ESP_ERROR_CHECK(vcp->tx_blocking((uint8_t *)"Test string", 12));
  */
}
