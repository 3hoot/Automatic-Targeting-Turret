#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "framed_serial.h"

static const char *TAG = "framed_serial";

/* -------------------- UART -------------------- */

static uint8_t uart_rx_buffer[FRAMED_SERIAL_UART_RX_BUFFER_LENGTH];
static uint8_t uart_tx_buffer[FRAMED_SERIAL_UART_TX_BUFFER_LENGTH];
static QueueHandle_t uart_event_queue = NULL;

/*
 * Task for handling UART events, such as data reception, errors, and other UART-related events.
 * This task is created when the framed_serial_init function is called with the UART protocol.
 */
static void framed_serial_uart_event_task(void *arg);
static TaskHandle_t framed_serial_uart_event_task_handle = NULL;

/* ------------------- USB CDC ------------------ */

static uint8_t usb_cdc_rx_buffer[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];

/*
 * Callback function for handling USB CDC events, such as data reception, line state changes, and other USB-related events.
 * This callback is registered with the TinyUSB stack when the framed_serial_init function is called with the USB CDC protocol.
 */
static void framed_serial_usb_cdc_rx_callback(int itf, cdcacm_event_t *event);

/* ----------------- Main logic ----------------- */

RingbufHandle_t framed_serial_rx_buffer = NULL;
RingbufHandle_t framed_serial_tx_buffer = NULL;

/*
 * Task for transmitting framed serial data.
 * Waits for data in the TX buffer and sends it through the selected protocol.
 * This task is created when the framed_serial_init function is called with any supported protocol.
 */
static void framed_serial_transmiter_task(void *arg);
static TaskHandle_t framed_serial_transmiter_task_handle = NULL;

/* -------------- Helper functions -------------- */

typedef enum framed_serial_parser_state
{
    STATE_WAIT_START,
    STATE_READ_LENGTH,
    STATE_READ_CHECKSUM,
    STATE_READ_DATA
} framed_serial_parser_state_t;

static framed_serial_frame_t parsed_frame;
static size_t parsed_frame_data_idx = 0;
static uint8_t running_checksum = 0;
static framed_serial_parser_state_t parser_state = STATE_WAIT_START;

/*
 * Function to parse incoming bytes and construct framed serial frames.
 * This function is called by the protocol-specific data reception handlers (UART and USB CDC) to process incoming bytes.
 * The parser maintains a state machine to handle the different stages of frame construction, 
 * including waiting for the start of a frame, reading the length, reading the data, and verifying the checksum.
 * 
 * @return True if a complete frame has been successfully parsed and is ready for further processing, false otherwise.
 */
static bool framed_serial_byte_parser(const uint8_t byte);

esp_err_t framed_serial_init(const framed_serial_protocol_t protocol, void **config)
{
    if (config == NULL || *config == NULL)
    {
        ESP_LOGE(TAG, "Configuration not provided.");
        return ESP_ERR_INVALID_ARG;
    }
    
    framed_serial_rx_buffer = xRingbufferCreate(FRAMED_SERIAL_RX_BUFFER_LENGTH * sizeof(framed_serial_frame_t), RINGBUF_TYPE_BYTEBUF);
    assert(framed_serial_rx_buffer != NULL && "Failed to create RX buffer");
    framed_serial_tx_buffer = xRingbufferCreate(FRAMED_SERIAL_TX_BUFFER_LENGTH * sizeof(framed_serial_frame_t), RINGBUF_TYPE_BYTEBUF);
    assert(framed_serial_tx_buffer != NULL && "Failed to create TX buffer");

    switch (protocol)
    {
        case FRAMED_SERIAL_PROTOCOL_UART:
        {
            const uart_config_t uart_config = *(uart_config_t *)config[0];
            const framed_serial_uart_config_t uart_custom_config = *(framed_serial_uart_config_t *)config[1];
            
            ESP_ERROR_CHECK(uart_param_config(uart_custom_config.port, &uart_config));
            ESP_ERROR_CHECK(uart_set_pin(uart_custom_config.port,
                                         uart_custom_config.tx_pin, uart_custom_config.rx_pin,
                                         uart_custom_config.cts_pin, uart_custom_config.rts_pin));
            ESP_ERROR_CHECK(uart_driver_install(uart_custom_config.port,
                                                FRAMED_SERIAL_UART_RX_BUFFER_LENGTH, FRAMED_SERIAL_UART_TX_BUFFER_LENGTH,
                                                FRAMED_SERIAL_UART_EVENT_QUEUE_LENGTH, &uart_event_queue, 0));

            xTaskCreate(framed_serial_uart_event_task, "uart_event_task", FRAMED_SERIAL_UART_TASK_STACK_SIZE,
                        config[1], FRAMED_SERIAL_UART_TASK_PRIORITY, &framed_serial_uart_event_task_handle);
            assert(framed_serial_uart_event_task_handle != NULL && "Failed to create UART event task");

            break;
        }
        case FRAMED_SERIAL_PROTOCOL_USB_CDC:
        {
            const tinyusb_config_t usb_config = *(tinyusb_config_t *)config[0];
            const tinyusb_config_cdcacm_t cdc_config = *(tinyusb_config_cdcacm_t *)config[1];
            
            ESP_ERROR_CHECK(tinyusb_driver_install(&usb_config));
            ESP_ERROR_CHECK(tinyusb_cdcacm_init(&cdc_config));

            ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                cdc_config.cdc_port,
                CDC_EVENT_RX,
                framed_serial_usb_cdc_rx_callback));

            break;
        }
        default:
            ESP_LOGE(TAG, "Unsupported protocol");
            return ESP_ERR_INVALID_ARG;
    }

    void **protocol_config_pair = malloc(2 * sizeof(void *));
    if (protocol_config_pair == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for protocol configuration pair");
        return ESP_ERR_NO_MEM;
    }
    protocol_config_pair[0] = (void *)protocol;
    protocol_config_pair[1] = config;

    xTaskCreate(framed_serial_transmiter_task, "frame_transmitter_task", FRAMED_SERIAL_FRAME_TRANSMITTER_TASK_STACK_SIZE,
                (void *)protocol_config_pair, FRAMED_SERIAL_FRAME_TRANSMITTER_TASK_PRIORITY, &framed_serial_transmiter_task_handle);
    assert(framed_serial_transmiter_task_handle != NULL && "Failed to create frame transmitter task");

    return ESP_OK;
}

framed_serial_frame_t framed_serial_get_frame_from_buffer(const RingbufHandle_t buffer, const TickType_t timeout)
{
    framed_serial_frame_t frame;
    size_t bytes_received = 0;
    void *frame_prefix = NULL;
    void *frame_data = NULL;

    const size_t frame_prefix_size = sizeof(frame) - FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH;

    frame_prefix = xRingbufferReceiveUpTo(buffer, &bytes_received, timeout, frame_prefix_size);
    if (frame_prefix == NULL || bytes_received != frame_prefix_size)
    {
        ESP_LOGE(TAG, "Failed to receive frame prefix from buffer");
        if (frame_prefix != NULL)
        {
            vRingbufferReturnItem(buffer, frame_prefix);
        }
        return (framed_serial_frame_t){0};
    }
    vRingbufferReturnItem(buffer, frame_prefix);

    memcpy(&frame, frame_prefix, frame_prefix_size);
    size_t frame_data_size = frame.data_length;

    if (frame_data_size > FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH)
    {
        ESP_LOGE(TAG, "Frame data length %zu exceeds maximum allowed length %d", frame_data_size, FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH);
        return (framed_serial_frame_t){0};
    }

    if (frame_data_size > 0)
    {
        frame_data = xRingbufferReceiveUpTo(buffer, &bytes_received, timeout, frame_data_size);
        if (frame_data == NULL || bytes_received != frame_data_size)
        {
            ESP_LOGE(TAG, "Failed to receive frame data from buffer");
            return (framed_serial_frame_t){0};
        }
        memcpy(frame.data, frame_data, frame_data_size);
        vRingbufferReturnItem(buffer, frame_data);
    }

    return frame;
}

static bool framed_serial_byte_parser(const uint8_t byte)
{
    switch (parser_state)
    {
        case STATE_WAIT_START:
            parsed_frame.type = byte;
            running_checksum = byte; 
            parser_state = STATE_READ_LENGTH;
            return false;

        case STATE_READ_LENGTH:
            parsed_frame.data_length = byte;
            running_checksum += byte;

            if (parsed_frame.data_length > FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH)
            {
                ESP_LOGW(TAG, "Received frame length %d exceeds maximum allowed length %d", parsed_frame.data_length, FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH);
                parser_state = STATE_WAIT_START;
            }
            else 
            {
                parser_state = STATE_READ_CHECKSUM;
            }
            return false;

        case STATE_READ_CHECKSUM:
            parsed_frame.checksum = byte;
            parsed_frame_data_idx = 0;

            if (parsed_frame.data_length == 0)
            {
                parser_state = STATE_WAIT_START;
                return (running_checksum == parsed_frame.checksum);
            }
            else
            {
                parser_state = STATE_READ_DATA;
                return false;
            }

        case STATE_READ_DATA:
            parsed_frame.data[parsed_frame_data_idx++] = byte;
            running_checksum += byte;

            if (parsed_frame_data_idx >= parsed_frame.data_length)
            {
                parser_state = STATE_WAIT_START;

                if (running_checksum == parsed_frame.checksum)
                {
                    return true;
                }
                else
                {
                    ESP_LOGW(TAG, "Checksum mismatch: calculated 0x%02X, received 0x%02X", running_checksum, parsed_frame.checksum);
                    return false;
                }
            }
            return false;

        default:
            ESP_LOGE(TAG, "Invalid parser state");
            parser_state = STATE_WAIT_START;
            return false;
    }
}

static void framed_serial_uart_event_task(void *arg)
{
    uart_event_t event;
    uart_port_t uart_num = ((framed_serial_uart_config_t *)arg)->port;
    size_t bytes_read = 0;
    size_t frame_size = 0;

    while (true)
    {
        if (!xQueueReceive(uart_event_queue, (void *)&event, portMAX_DELAY))
        {
            continue;
        }
        
        switch(event.type)
        {
            case UART_DATA:
                ESP_LOGI(TAG, "UART_DATA event, size: %d", event.size);
                bytes_read = uart_read_bytes(uart_num, uart_rx_buffer, event.size, portMAX_DELAY);

                /* Parse each byte of the received data */
                for (size_t i = 0; i < bytes_read; i++)
                {
                    if (framed_serial_byte_parser(uart_rx_buffer[i]))
                    {
                        /* A complete frame has been parsed */
                        frame_size = sizeof(parsed_frame) - FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH + parsed_frame.data_length;

                        if (xRingbufferSend(framed_serial_rx_buffer, &parsed_frame, frame_size, portMAX_DELAY) != pdTRUE)
                        {
                            ESP_LOGE(TAG, "Failed to send parsed frame to RX buffer");
                            parser_state = STATE_WAIT_START;  /* Reset parser state to avoid desynchronization */
                        }
                    }
                }
                break;

            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "UART FIFO overflow or buffer full! Host ignored RTS line.");
                uart_flush_input(uart_num);
                xQueueReset(uart_event_queue);
                parser_state = STATE_WAIT_START;  /* Reset parser state to avoid desynchronization */
                break;
            
            default:
                ESP_LOGI(TAG, "UART event type: %d", event.type);
                break;
        }
    }
}

static void framed_serial_usb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    size_t rx_size = 0;
    
    esp_err_t ret = tinyusb_cdcacm_read(itf, usb_cdc_rx_buffer, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read from USB CDC interface %d: %s", itf, esp_err_to_name(ret));
        return;
    }

    for (size_t i = 0; i < rx_size; i++)
    {
        if (framed_serial_byte_parser(usb_cdc_rx_buffer[i]))
        {
            size_t frame_size = sizeof(parsed_frame) - FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH + parsed_frame.data_length;

            if (xRingbufferSend(framed_serial_rx_buffer, &parsed_frame, frame_size, pdMS_TO_TICKS(50)) != pdTRUE)
            {
                ESP_LOGE(TAG, "Failed to send parsed frame to RX buffer");
                parser_state = STATE_WAIT_START;  /* Reset parser state to avoid desynchronization */
            }
        }
    }
}

static void framed_serial_transmiter_task(void *arg)
{
    framed_serial_frame_t frame;
    
    const framed_serial_protocol_t used_protocol = (framed_serial_protocol_t)((void **)arg)[0];
    const void **config = ((void **)arg)[1];
    const size_t frame_prefix_size = sizeof(frame) - FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH;

    size_t frame_size = 0;
    int bytes_written = 0;

    while (true)
    {
        frame = framed_serial_get_frame_from_buffer(framed_serial_tx_buffer, portMAX_DELAY);
        if (frame.type == 0 && frame.data_length == 0 && frame.checksum == 0)
        {
            ESP_LOGE(TAG, "Failed to retrieve frame from TX buffer");
            continue;
        }
        frame_size = frame_prefix_size + frame.data_length;

        frame.checksum = frame.type + frame.data_length;
        for (size_t i = 0; i < frame.data_length; i++)
        {
            frame.checksum += frame.data[i];
        }

        switch (used_protocol)
        {
            case FRAMED_SERIAL_PROTOCOL_UART:
            {
                const uart_config_t uart_config = *(uart_config_t *)config[0];
                const framed_serial_uart_config_t uart_custom_config = *(framed_serial_uart_config_t *)config[1];

                bytes_written = uart_write_bytes(uart_custom_config.port, (const uint8_t *)&frame, frame_size);
                if (bytes_written < 0)
                {
                    ESP_LOGE(TAG, "Failed to write frame to UART");
                }
                break;
            }
            case FRAMED_SERIAL_PROTOCOL_USB_CDC:
            {
                const tinyusb_config_t usb_config = *(tinyusb_config_t *)config[0];
                const tinyusb_config_cdcacm_t cdc_config = *(tinyusb_config_cdcacm_t *)config[1];

                bytes_written = tinyusb_cdcacm_write_queue(cdc_config.cdc_port, (const uint8_t *)&frame, frame_size);
                if (bytes_written < 0)
                {
                    ESP_LOGE(TAG, "Failed to write frame to USB CDC");
                }
                if (tinyusb_cdcacm_write_flush(cdc_config.cdc_port, pdMS_TO_TICKS(FRAMED_SERIAL_USB_CDC_RETRY_TIMEOUT_MS)) != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to flush USB CDC write queue");
                }

                break;
            }
            default:
                ESP_LOGE(TAG, "Unsupported protocol for transmission");
                break;
        }
    }
}