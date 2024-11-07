/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "stdint.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TAG "USB_UART"

#define EX_UART_NUM UART_NUM_1
#define TXD_GPIO_NUM 2
#define RXD_GPIO_NUM 1


#define BUF_SIZE (4096)
#define RD_BUF_SIZE (BUF_SIZE)


static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];

static QueueHandle_t uart0_queue;



void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;

    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Data from channel %d:", itf);
        ESP_LOG_BUFFER_HEXDUMP(TAG, buf, rx_size, ESP_LOG_INFO);
    }
    else
    {
        ESP_LOGE(TAG, "Read error");
    }

    /* write back */
    // tinyusb_cdcacm_write_queue(itf, buf, rx_size);
    // tinyusb_cdcacm_write_flush(itf, 0);

    uart_write_bytes(EX_UART_NUM, (const char *)buf, rx_size);
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr,rts);
    uint32_t bit_rate  = event->line_coding_changed_data.p_line_coding->bit_rate;
    uint8_t  stop_bits = event->line_coding_changed_data.p_line_coding->stop_bits;
    uint8_t  parity    = event->line_coding_changed_data.p_line_coding->parity;
    uint8_t  data_bits = event->line_coding_changed_data.p_line_coding->data_bits;
    ESP_LOGI(TAG, "bit_rate:%ld: stop_bits:%d, parity:%d, data_bits:%d", bit_rate, stop_bits,parity,data_bits);
    uart_set_word_length(EX_UART_NUM,data_bits);
    uart_set_stop_bits(EX_UART_NUM,stop_bits);
    uart_set_parity(EX_UART_NUM,parity);
    uart_set_baudrate(EX_UART_NUM,bit_rate);
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);
    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch (event.type)
            {
            case UART_DATA:
            {
                ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                ESP_LOGI(TAG, "[DATA EVT]:");
                // uart_write_bytes(EX_UART_NUM, (const char *)dtmp, event.size);
                tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, dtmp, event.size);
                tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
                break;
            }
            case UART_FIFO_OVF:
            {
                ESP_LOGI(TAG, "hw fifo overflow");
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart0_queue);
                break;
            }
            case UART_BUFFER_FULL:
            {
                ESP_LOGI(TAG, "ring buffer full");
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart0_queue);
                break;
            }
            case UART_BREAK:
            {
                ESP_LOGI(TAG, "uart rx break");
                break;
            }
            case UART_PARITY_ERR:
            {
                ESP_LOGI(TAG, "uart parity error");
                break;
            }
            case UART_FRAME_ERR:
            {
                ESP_LOGI(TAG, "uart frame error");
                break;
            }
            // Others
            default:
            {
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void uart_init()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue,
                        0);
    uart_param_config(EX_UART_NUM, &uart_config);
    uart_set_pin(EX_UART_NUM, TXD_GPIO_NUM, RXD_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_pattern_queue_reset(EX_UART_NUM, 20);

    xTaskCreatePinnedToCore(uart_event_task, "uart_event_task", 4096 * 10, NULL, 20, NULL, 1);
}

const char *device0_string_descriptor[] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},       // 0: is supported language is English (0x0409)
    "Techprogeny",            // 1: Manufacturer
    "Esp32 USB to UART",          // 2: Product
    "123456",                   // 3: Serials, should use chip ID
    "Techprogeny CDC Device", // 4: CDC Interface
};

const tusb_desc_device_t device0_descriptor_dev = {
    .bLength = sizeof(device0_descriptor_dev),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,

#if CFG_TUD_CDC
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
#else
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
#endif

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0xB1B0,
    .idProduct = 0x8055,
    .bcdDevice = CONFIG_TINYUSB_DESC_BCD_DEVICE,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01};

void usb_init()
{
    ESP_LOGI(TAG, "USB initialization");

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &device0_descriptor_dev,
        .string_descriptor = device0_string_descriptor,
        .string_descriptor_count = sizeof(device0_string_descriptor) /
                                   sizeof(device0_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = NULL,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL};

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    /* the second way to register a callback */
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
        TINYUSB_CDC_ACM_0, CDC_EVENT_LINE_STATE_CHANGED,
        &tinyusb_cdc_line_state_changed_callback));
    ESP_LOGI(TAG, "USB initialization DONE");
}


void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_NONE);
    usb_init();
    uart_init();
}
