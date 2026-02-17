#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

static const char *TAG = "cdc_acm";

// HR50 UART on UART1, TX-only, 19200 8N1
// HR50 serial port is TTL-level — GPIO 17 connects directly
#define HR50_UART      UART_NUM_1
#define HR50_TX_PIN    17
#define HR50_BAUD      19200

// Set these to match your CDC-ACM device, or 0 to match any
#define USB_DEVICE_VID 0
#define USB_DEVICE_PID 0
#define USB_DEVICE_INTERFACE 0

#define SEQ_LEN 5

static SemaphoreHandle_t device_disconnected_sem;

// RX accumulation buffer for 5-byte CAT responses
static uint8_t rx_buf[SEQ_LEN];
static size_t rx_pos = 0;

static const char *ft817_mode_str(uint8_t mode)
{
    switch (mode) {
    case 0x00: return "LSB";
    case 0x01: return "USB";
    case 0x02: return "CW";
    case 0x03: return "CW-R";
    case 0x04: return "AM";
    case 0x06: return "WFM";
    case 0x08: return "FM";
    case 0x0A: return "DIG";
    case 0x0C: return "PKT";
    default:   return "?";
    }
}

static void hr50_send_freq(uint32_t freq_hz)
{
    // HR50 K3 protocol: FAxxxxxxxxxxx; (11-digit Hz, zero-padded)
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "FA%011lu;", (unsigned long)freq_hz);
    uart_write_bytes(HR50_UART, cmd, strlen(cmd));
    ESP_LOGI(TAG, "HR50 TX: %s", cmd);
}

static void decode_freq_mode(const uint8_t *resp)
{
    // Bytes 0-3: BCD frequency (e.g. 0x14 0x19 0x50 0x00 = 14.195.000 MHz)
    // Byte 4: mode
    uint32_t freq_hz = 0;
    for (int i = 0; i < 4; i++) {
        freq_hz = freq_hz * 100 + ((resp[i] >> 4) & 0x0F) * 10 + (resp[i] & 0x0F);
    }
    freq_hz *= 10; // BCD encodes down to 10 Hz steps

    ESP_LOGI(TAG, "Frequency: %lu.%03lu.%03lu MHz  Mode: %s",
             freq_hz / 1000000, (freq_hz / 1000) % 1000, freq_hz % 1000,
             ft817_mode_str(resp[4]));

    hr50_send_freq(freq_hz);
}

static bool handle_rx(const uint8_t *data, size_t data_len, void *arg)
{
    for (size_t i = 0; i < data_len; i++) {
        rx_buf[rx_pos++] = data[i];
        if (rx_pos >= SEQ_LEN) {
            ESP_LOG_BUFFER_HEX(TAG, rx_buf, SEQ_LEN);
            decode_freq_mode(rx_buf);
            rx_pos = 0;
        }
    }
    return true;
}

static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC-ACM error: %d", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "Device disconnected");
        xSemaphoreGive(device_disconnected_sem);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        ESP_LOGI(TAG, "Serial state: 0x%04X", event->data.serial_state.val);
        break;
    default:
        break;
    }
}

static void handle_new_dev(usb_device_handle_t usb_dev)
{
    const usb_device_desc_t *desc;
    if (usb_host_get_device_descriptor(usb_dev, &desc) == ESP_OK) {
        ESP_LOGI(TAG, "New USB device:");
        ESP_LOGI(TAG, "  VID: 0x%04X  PID: 0x%04X", desc->idVendor, desc->idProduct);
        ESP_LOGI(TAG, "  bcdDevice: %d.%02d", desc->bcdDevice >> 8, desc->bcdDevice & 0xFF);
        ESP_LOGI(TAG, "  Class: 0x%02X  Subclass: 0x%02X  Protocol: 0x%02X",
                 desc->bDeviceClass, desc->bDeviceSubClass, desc->bDeviceProtocol);
        ESP_LOGI(TAG, "  Configurations: %d", desc->bNumConfigurations);
    }
}

static void usb_host_lib_task(void *arg)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            break;
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    device_disconnected_sem = xSemaphoreCreateBinary();

    // Init HR50 UART (19200 8N1)
    const uart_config_t hr50_uart_config = {
        .baud_rate = HR50_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(HR50_UART, &hr50_uart_config));
    ESP_ERROR_CHECK(uart_set_pin(HR50_UART, HR50_TX_PIN, UART_PIN_NO_CHANGE,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // Invert TX polarity — HR50 ACC port may expect RS-232-style inverted logic
    //ESP_ERROR_CHECK(uart_set_line_inverse(HR50_UART, UART_SIGNAL_TXD_INV));
    ESP_ERROR_CHECK(uart_driver_install(HR50_UART, 256, 0, 0, NULL, 0));
    ESP_LOGI(TAG, "HR50 UART initialized on TX=%d @ %d baud", HR50_TX_PIN, HR50_BAUD);

    // Install USB Host Library
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    xTaskCreate(usb_host_lib_task, "usb_host", 4096, NULL, 5, NULL);

    // Install CDC-ACM host driver
    const cdc_acm_host_driver_config_t cdc_config = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = 5,
        .xCoreID = 0,
    };
    ESP_ERROR_CHECK(cdc_acm_host_install(&cdc_config));
    ESP_ERROR_CHECK(cdc_acm_host_register_new_dev_callback(handle_new_dev));

    while (1) {
        const cdc_acm_host_device_config_t dev_config = {
            .connection_timeout_ms = 10000,
            .out_buffer_size = 64,
            .in_buffer_size = 64,
            .event_cb = handle_event,
            .data_cb = handle_rx,
        };
        cdc_acm_dev_hdl_t cdc_dev = NULL;

        ESP_LOGI(TAG, "Waiting for CDC-ACM device...");
        esp_err_t err = cdc_acm_host_open(USB_DEVICE_VID, USB_DEVICE_PID,
                                           USB_DEVICE_INTERFACE, &dev_config,
                                           &cdc_dev);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Open failed: %s, retrying...", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Device opened");
        cdc_acm_host_desc_print(cdc_dev);

        // 9600 8N2
        const cdc_acm_line_coding_t line_coding = {
            .dwDTERate = 9600,
            .bCharFormat = 2,  // 2 stop bits
            .bParityType = 0,  // no parity
            .bDataBits = 8,
        };
        cdc_acm_host_line_coding_set(cdc_dev, &line_coding);
        cdc_acm_host_set_control_line_state(cdc_dev, true, true);

        // FT-817 CAT: Read Frequency & Mode (opcode 0x03 is last byte)
        uint8_t tx_buf[SEQ_LEN] = {0x00, 0x00, 0x00, 0x00, 0x03};
        rx_pos = 0;

        while (xSemaphoreTake(device_disconnected_sem, 0) != pdTRUE) {

            err = cdc_acm_host_data_tx_blocking(cdc_dev, tx_buf, SEQ_LEN,
                                                 pdMS_TO_TICKS(1000));
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "TX error: %s", esp_err_to_name(err));
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        cdc_acm_host_close(cdc_dev);
        ESP_LOGI(TAG, "Device closed, waiting for reconnect...");
    }
}
