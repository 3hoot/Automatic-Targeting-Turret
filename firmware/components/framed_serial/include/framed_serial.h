#ifndef FRAMED_SERIAL_H
#define FRAMED_SERIAL_H

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "tinyusb_cdc_acm.h"

/* Preprocessor directive to avoid C++ name mangling */
#ifdef __cplusplus
extern "C" 
{
#endif

#define FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH (128) 

/*
 * The raw frame is structured as follows:
 * [1] - Frame Type
 * [2] - Frame Payload Length
 * [3] - Checksum, calculated as the sum of all bytes in the frame, excluding the checksum byte itself
 * [4] - Frame Payload
 * 
 * The user is responsible for ensuring that the frame type and payload are correctly set, 
 * however the checksum is automatically calculated and verified upon transmission.
 * 
 * Any additions to the frame structure should be done before the declaration of the data member and be reflected in the parser logic.
 */
typedef struct __attribute__((packed)) framed_serial_frame
{
    uint8_t type;
    uint8_t data_length;
    uint8_t checksum;
    uint8_t data[FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH];
} framed_serial_frame_t;

/*
 * Available protocols of communication.
 * Each protocol must have an appropriate handler inside the framed_serial.c file (see the function "framed_serial_init").
 * Each protocol specifies its own configuration arguments, which are passed to the "framed_serial_init" function.
 */
typedef enum framed_serial_protocol
{
    FRAMED_SERIAL_PROTOCOL_UART,    /* Requires to pass in uart_config_t and framed_serial_uart_config_t*/
    FRAMED_SERIAL_PROTOCOL_USB_CDC  /* Requires to pass in tinyusb_config_t*/
} framed_serial_protocol_t;

/*
 * Framed serial configuration structure for UART protocol.
 * This structure is used to pass all UART-specific configuration parameters not covered by the default configuration structure.
 */
typedef struct framed_serial_uart_config
{
    uart_port_t port;
    uint8_t tx_pin;
    uint8_t rx_pin;
    uint8_t cts_pin;
    uint8_t rts_pin;
} framed_serial_uart_config_t;

/* -------------------- UART -------------------- */

#define FRAMED_SERIAL_UART_TASK_PRIORITY        (10)
#define FRAMED_SERIAL_UART_TASK_STACK_SIZE      (4096)
#define FRAMED_SERIAL_UART_EVENT_QUEUE_LENGTH   (16)
#define FRAMED_SERIAL_UART_RX_THRESHOLD         (122)
#define FRAMED_SERIAL_UART_RX_BUFFER_LENGTH     (1024)
#define FRAMED_SERIAL_UART_TX_BUFFER_LENGTH     (1024)

#define FRAMED_SERIAL_UART_DEFAULT_CONFIG()                \
(uart_config_t)                                            \
{                                                          \
    .baud_rate = 115200,                                   \
    .data_bits = UART_DATA_8_BITS,                         \
    .parity = UART_PARITY_DISABLE,                         \
    .stop_bits = UART_STOP_BITS_1,                         \
    .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,                 \
    .source_clk = UART_SCLK_DEFAULT,                       \
    .rx_flow_ctrl_thresh = FRAMED_SERIAL_UART_RX_THRESHOLD \
}

#define FRAMED_SERIAL_UART_DEFAULT_CUSTOM_CONFIG() \
(framed_serial_uart_config_t)                      \
{                                                  \
    .port = UART_NUM_0,                            \
    .tx_pin = GPIO_NUM_43,                         \
    .rx_pin = GPIO_NUM_44,                         \
    .cts_pin = GPIO_NUM_1,                         \
    .rts_pin = GPIO_NUM_2                          \
}

/* ------------------- USB CDC ------------------ */

#define FRAMED_SERIAL_USB_CDC_RETRY_TIMEOUT_MS (1000) /* Retry to send data if it fails */

#define FRAMED_SERIAL_USB_CDC_DEFAULT_CONFIG() \
(tinyusb_config_cdcacm_t)                      \
{                                              \
    .cdc_port = TINYUSB_CDC_ACM_0,             \
    .callback_rx = NULL,                       \
    .callback_rx_wanted_char = NULL,           \
    .callback_line_state_changed = NULL,       \
    .callback_line_coding_changed = NULL       \
}

/* ----------------- Main logic ----------------- */

#define FRAMED_SERIAL_FRAME_PARSER_TASK_PRIORITY        (10)
#define FRAMED_SERIAL_FRAME_TRANSMITTER_TASK_PRIORITY   (10)
#define FRAMED_SERIAL_FRAME_PARSER_TASK_STACK_SIZE      (4096)
#define FRAMED_SERIAL_FRAME_TRANSMITTER_TASK_STACK_SIZE (4096)

#define FRAMED_SERIAL_RX_BUFFER_LENGTH  (1024)
#define FRAMED_SERIAL_TX_BUFFER_LENGTH  (1024)

/* 
 * Ring buffers for RX and TX data, initialized by the framed_serial_init function.
 * Access to sent and received data is done through these buffers.
 * Received data is pushed to the RX buffer by the protocol handler, and can be read from the RX buffer by the user (and disposed of).
 * Data to be sent is meant to be pushed to the TX buffer by the user, and is handled by the internal frame parser.
 */
extern RingbufHandle_t framed_serial_rx_buffer;
extern RingbufHandle_t framed_serial_tx_buffer;

/*
 * Initializes the framed serial communication with the specified protocol and configuration.
 * The configuration structure must live for the entire duration of the communication, as it is used by the protocol handler.
 *
 * @param protocol The communication protocol to use.
 * @param config A pointer to list of configuration parameters specific to the chosen protocol.
 *               The structure of this configuration is defined by the protocol handler.
 * @return ESP_OK on success, non-zero error code on failure (uses esp error codes).
 */
esp_err_t framed_serial_init(const framed_serial_protocol_t protocol, void **config);

/*
 * Helper function to retrieve a complete frame from the specified ring buffer.
 * This function blocks until a complete frame is available or the specified timeout is reached.
 * Upon successful retrieval, the frame is removed from the buffer and returned to the caller.
 * If the function fails for some reason, a framed_serial_frame_t structure with all fields set to zero is returned.
 * 
 * @param buffer The ring buffer to read from (either framed_serial_rx_buffer or framed_serial_tx_buffer).
 * @param timeout The maximum time to wait for a complete frame, in ticks.
 * 
 * @return A framed_serial_frame_t structure containing the retrieved frame.
 */
framed_serial_frame_t framed_serial_get_frame_from_buffer(const RingbufHandle_t buffer, const TickType_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* FRAMED_SERIAL_H */