#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "driver/i2c_master.h"

#include "app_config.h"
#include "framed_serial.h"
#include "stepper_driver.h"
#include "utils.h"
#include "as5600.h"
#include "ina226.h"
#include "motion_controller.h"

static const char *TAG = "app_main";

#define A_ID (0)
#define B_ID (1)

static stepper_drv_handle_t stepper[APP_STEPPER_COUNT] = {0};
static as5600_handle_t position_sensor[APP_AS5600_COUNT] = {0};
static ina226_handle_t power_sensor[APP_INA226_COUNT] = {0};

static motion_ctrl_handle_t motion_controller[APP_MOTION_CONTROLLER_COUNT] = {0};

/* -------------- Command executor -------------- */

/*
 * Task responsible for executing commands received via framed serial communication.
 */
static void app_executor_task(void *arg);
TaskHandle_t app_executor_task_handle = NULL;

/* -------------- Helper functions -------------- */

/*
 * Send an ACK frame in response to a received command.
 * 
 * @param command_type The type of the command being acknowledged.
 * @param status_code The status code to include in the ACK frame.
 * @return ESP_OK on success, non-zero error code on failure.
 */
static esp_err_t app_send_ack(const uint8_t command_type, const esp_err_t status_code);

/*
 * Send a response frame in response to a received command.
 * Preforms an internal memcpy of the data to be sent, so the caller can free or modify the data after return.
 * If the data_length is more than FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH, the function will block until all the data is sent.
 *
 * @param command_type The type of the command for which a response is being sent.
 * @param data Pointer to the data to include in the response frame.
 * @param data_length The length of the data.
 * @return ESP_OK on success, non-zero error code on failure.
 */
static esp_err_t app_send_response(const uint8_t command_type, const uint8_t *data, const size_t data_length);

void app_main(void)
{
    /* ---------------- Memory check ---------------- */

    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "Detected Flash Size: %lu MB", flash_size / (1024 * 1024));

    if (esp_psram_is_initialized())
    {
        ESP_LOGI(TAG, "PSRAM Total Size: %d bytes", esp_psram_get_size());
        ESP_LOGI(TAG, "PSRAM Free Size: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }
    else
    {
        ESP_LOGE(TAG, "PSRAM Failed to initialize!");
    }

    /* ----------- USB communication init ----------- */
    
    static tinyusb_config_t usb_config;
    usb_config = (tinyusb_config_t)TINYUSB_DEFAULT_CONFIG();
    static tinyusb_config_cdcacm_t cdc_config = FRAMED_SERIAL_USB_CDC_DEFAULT_CONFIG();

    void** config = malloc(2 * sizeof(void*));
    assert(config != NULL && "Failed to allocate memory for USB framed serial configuration");
    config[0] = (void *)&usb_config;
    config[1] = (void *)&cdc_config;
    ESP_ERROR_CHECK(framed_serial_init(FRAMED_SERIAL_PROTOCOL_USB_CDC, config));

    /* ------------- Stepper driver init ------------ */

    stepper_drv_config_t stepper_A_config = {
        .pin_step = APP_STEPPER_A_STEP_PIN,
        .pin_dir = APP_STEPPER_A_DIR_PIN,
        .pin_enable = APP_STEPPER_A_EN_PIN
    };
    ESP_ERROR_CHECK(stepper_drv_init(stepper_A_config, &stepper[A_ID]));

    stepper_drv_handle_t stepper_B_handle = {0};
    stepper_drv_config_t stepper_B_config = {
        .pin_step = APP_STEPPER_B_STEP_PIN,
        .pin_dir = APP_STEPPER_B_DIR_PIN,
        .pin_enable = APP_STEPPER_B_EN_PIN
    };
    ESP_ERROR_CHECK(stepper_drv_init(stepper_B_config, &stepper[B_ID]));

    /* ---------------- I2C bus init ---------------- */

    /*
     * Because the AS5600 position sensor has a fixed I2C address, 
     * we need to initialize two separate I2C buses to communicate with two sensors simultaneously.
     * Another thing to note, it is recommended by Espressif to use external pull-up whenever possible
     * (Internal pull-ups are not strong enough for high-speed I2C communication).
     */

    i2c_master_bus_config_t bus_A_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = 0,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_NUM_0,
        .intr_priority = APP_I2C_BUS_A_INTERRUPT_PRIORITY,
        .scl_io_num = APP_I2C_BUS_A_SCL_PIN,
        .sda_io_num = APP_I2C_BUS_A_SDA_PIN
    };
    i2c_master_bus_handle_t bus_A_handle = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_A_config, &bus_A_handle));
    
    i2c_master_bus_config_t bus_B_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = 0,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_NUM_1,
        .intr_priority = APP_I2C_BUS_B_INTERRUPT_PRIORITY,
        .scl_io_num = APP_I2C_BUS_B_SCL_PIN,
        .sda_io_num = APP_I2C_BUS_B_SDA_PIN
    };
    i2c_master_bus_handle_t bus_B_handle = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_B_config, &bus_B_handle));

    uint8_t found_addresses[APP_I2C_SCAN_FOUND_DEVICES_ARRAY_SIZE] = {0};
    size_t found_count_A = 0;
    size_t found_count_B = 0;
    utils_scan_i2c_bus(bus_A_handle, found_addresses, &found_count_A, APP_I2C_SCAN_MAX_FOUND_DEVICES, APP_I2C_SCAN_TIMEOUT_MS);
    utils_scan_i2c_bus(bus_B_handle, (found_addresses + (uint8_t)found_count_A), &found_count_B, (APP_I2C_SCAN_MAX_FOUND_DEVICES - found_count_A), APP_I2C_SCAN_TIMEOUT_MS);

    /* ---------- INA226 power sensors init --------- */

    ina226_config_t power_sensor_A_config = INA226_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(ina226_init(bus_A_handle, &power_sensor_A_config, APP_INA226_A_I2C_ADDRESS, &power_sensor[A_ID]));
    
    ina226_config_t ina226_B_config = INA226_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(ina226_init(bus_B_handle, &ina226_B_config, APP_INA226_B_I2C_ADDRESS, &power_sensor[B_ID]));

    /* -------- AS5600 position sensors init -------- */

    as5600_config_t position_sensor_A_config = AS5600_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(as5600_init(bus_A_handle, &position_sensor_A_config, &position_sensor[B_ID]));

    as5600_config_t position_sensor_B_config = AS5600_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(as5600_init(bus_B_handle, &position_sensor_B_config, &position_sensor[A_ID]));

    /* Swapped idx for sensors due to my poor hardware layout (DON'T HATE ME PLEASE PLEASE PLEASE PLEASE) */

    /* ----------- Motion controller init ----------- */

    motion_ctrl_config_t motion_controller_A_config = {
        .stepper = &stepper[A_ID],
        .position_sensor = &position_sensor[A_ID],
        .power_sensor = &power_sensor[A_ID],
        .limits = {.max_position = APP_MOTION_CONTROLLER_A_MAX_POSITION,
                   .mid_position = APP_MOTION_CONTROLLER_A_MID_POSITION,
                   .min_position = APP_MOTION_CONTROLLER_A_MIN_POSITION},
        .position_threshold = APP_MOTION_CONTROLLER_POSITION_THRESHOLD
    };
    ESP_ERROR_CHECK(motion_ctrl_init(&motion_controller_A_config, &motion_controller[A_ID]));

    motion_ctrl_config_t motion_controller_B_config = {
        .stepper = &stepper[B_ID],
        .position_sensor = &position_sensor[B_ID],
        .power_sensor = &power_sensor[B_ID],
        .limits = {.max_position = APP_MOTION_CONTROLLER_B_MAX_POSITION,
                   .mid_position = APP_MOTION_CONTROLLER_B_MID_POSITION,
                   .min_position = APP_MOTION_CONTROLLER_B_MIN_POSITION},
        .position_threshold = APP_MOTION_CONTROLLER_POSITION_THRESHOLD
    };
    ESP_ERROR_CHECK(motion_ctrl_init(&motion_controller_B_config, &motion_controller[B_ID]));

    /* ------------ Command executor init ----------- */

    xTaskCreate(app_executor_task, "app_executor_task", APP_EXECUTOR_TASK_STACK_SIZE,
                NULL, APP_EXECUTOR_TASK_PRIORITY, &app_executor_task_handle);
    if (app_executor_task_handle == NULL)
    {
        ESP_LOGE(TAG, "Failed to create app executor task");
    }
}

static void app_executor_task(void *arg)
{
    framed_serial_frame_t rx_frame = {0};

    const size_t frame_prefix_size = sizeof(rx_frame) - FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH;
    size_t rx_frame_size = 0;

    while (true)
    {
        memset(&rx_frame, 0, sizeof(rx_frame));
        rx_frame = framed_serial_get_frame_from_buffer(framed_serial_rx_buffer, portMAX_DELAY);
        rx_frame_size = frame_prefix_size + rx_frame.data_length;

        switch (rx_frame.type)
        {
            case APP_CMD_ECHO_CODE:
            {
                ESP_LOGI(TAG, "Received ECHO command with payload length: %d", rx_frame.data_length);
                if (rx_frame.data_length == 0)
                {
                    ESP_LOGW(TAG, "ECHO command received with no payload");
                    break;
                }

                uint8_t *echo_payload = malloc(rx_frame.data_length);
                if (echo_payload == NULL)
                {
                    ESP_LOGE(TAG, "Failed to allocate memory for ECHO payload");
                    break;
                }
                memcpy(echo_payload, rx_frame.data, rx_frame.data_length);
                ESP_ERROR_CHECK(app_send_response(rx_frame.type, echo_payload, rx_frame.data_length));
                free(echo_payload);
                break;
            }

            case APP_CMD_CLEAR_RX:
            {
                ESP_LOGI(TAG, "Received CLEAR_RX command");

                framed_serial_frame_t dummy_frame = framed_serial_get_frame_from_buffer(framed_serial_rx_buffer, 0);
                while (dummy_frame.type != 0)
                {
                    ESP_LOGW(TAG, "Clearing RX buffer, discarded frame of type: 0x%02X", dummy_frame.type);
                    dummy_frame = framed_serial_get_frame_from_buffer(framed_serial_rx_buffer, 0);
                }

                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, ESP_OK));
                break;
            }

            case APP_CMD_STEP_ARM_CODE:
                ESP_LOGI(TAG, "Received STEP_ARM command for stepper ID: %d", rx_frame.data[0]);
                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, stepper_drv_enable(&stepper[rx_frame.data[0]])));
                break;
                
            case APP_CMD_STEP_DISARM_CODE:
                ESP_LOGI(TAG, "Received STEP_DISARM command for stepper ID: %d", rx_frame.data[0]);
                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, stepper_drv_disable(&stepper[rx_frame.data[0]])));
                break;

            case APP_CMD_STEP_GET_POS_CODE:
            {
                uint8_t stepper_id = rx_frame.data[0];
                int32_t position = 0;
                ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_get_position(&stepper[stepper_id], &position));

                ESP_LOGI(TAG, "Received STEP_GET_POS command for stepper ID: %d, current position: %d", stepper_id, position);

                uint8_t position_bytes[4];
                utils_u32_to_bytes_be((uint32_t)position, position_bytes, 0);
                ESP_ERROR_CHECK(app_send_response(rx_frame.type, position_bytes, sizeof(position_bytes)));
                break;
            }

            case APP_CMD_SENSE_GET_POS_CODE:
            {
                uint8_t sensor_id = rx_frame.data[0];
                float angle = 0;
                ESP_ERROR_CHECK_WITHOUT_ABORT(as5600_read_angle(&position_sensor[sensor_id], &angle));

                ESP_LOGI(TAG, "Received SENSE_GET_POS command for sensor ID: %d, current angle: %f", sensor_id, angle);

                uint8_t position_bytes[4];
                utils_float_to_bytes_be(angle, position_bytes, 0);
                ESP_ERROR_CHECK(app_send_response(rx_frame.type, position_bytes, sizeof(position_bytes)));
                break;
            }

            case APP_CMD_SENSE_GET_PWR_CODE:
            {
                uint8_t sensor_id = rx_frame.data[0];
                float power = 0;
                ESP_ERROR_CHECK_WITHOUT_ABORT(ina226_read_power(&power_sensor[sensor_id], &power));

                ESP_LOGI(TAG, "Received SENSE_GET_PWR command for sensor ID: %d, current power: %f", sensor_id, power);

                uint16_t power_raw = (uint16_t)(power / INA226_POWER_FACTOR);
                uint8_t power_bytes[2];
                utils_u16_to_bytes_be(power_raw, power_bytes, 0);
                ESP_ERROR_CHECK(app_send_response(rx_frame.type, power_bytes, sizeof(power_bytes)));
                break;
            }

            case APP_CMD_SENSE_GET_CURR_CODE:
            {
                uint8_t sensor_id = rx_frame.data[0];
                float current = 0;
                ESP_ERROR_CHECK_WITHOUT_ABORT(ina226_read_current(&power_sensor[sensor_id], &current));

                ESP_LOGI(TAG, "Received SENSE_GET_CURR command for sensor ID: %d, current current: %f", sensor_id, current);

                uint16_t current_raw = (uint16_t)(current / INA226_CURRENT_FACTOR);
                uint8_t current_bytes[2];
                utils_u16_to_bytes_be(current_raw, current_bytes, 0);
                ESP_ERROR_CHECK(app_send_response(rx_frame.type, current_bytes, sizeof(current_bytes)));
                break;
            }

            case APP_CMD_SENSE_GET_VOLT_CODE:
            {
                uint8_t sensor_id = rx_frame.data[0];
                float voltage = 0;
                ESP_ERROR_CHECK(ina226_read_bus_voltage(&power_sensor[sensor_id], &voltage));

                ESP_LOGI(TAG, "Received SENSE_GET_VOLT command for sensor ID: %d, current voltage: %f", sensor_id, voltage);

                uint16_t voltage_raw = (uint16_t)(voltage / INA226_BUS_VOLTAGE_FACTOR);
                uint8_t voltage_bytes[2];
                utils_u16_to_bytes_be(voltage_raw, voltage_bytes, 0);
                ESP_ERROR_CHECK(app_send_response(rx_frame.type, voltage_bytes, sizeof(voltage_bytes)));
                break;
            }

            case APP_CMD_MCTRL_MOVE_CODE:
            {
                uint8_t controller_id = rx_frame.data[0];
                motion_ctrl_move_t move = {0};
                move.end_position = utils_bytes_to_float_be(rx_frame.data, 1);
                move.step_frequency_hz = utils_bytes_to_u32_be(rx_frame.data, 5);
                move.block_until_complete = (rx_frame.data[9] != 0);

                ESP_LOGI(TAG, "Received MCTRL_MOVE command for controller ID: %d, end position: %f, step frequency: %u Hz, block until complete: %s",
                         controller_id, move.end_position, move.step_frequency_hz, move.block_until_complete ? "true" : "false");

                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, motion_ctrl_move(&motion_controller[controller_id], &move)));
                break;
            }

            case APP_CMD_MCTRL_GET_POS_CODE:
            {
                uint8_t controller_id = rx_frame.data[0];
                float position = 0;
                ESP_ERROR_CHECK_WITHOUT_ABORT(motion_ctrl_get_position(&motion_controller[controller_id], &position));

                ESP_LOGI(TAG, "Received MCTRL_GET_POS command for controller ID: %d, current position: %f", controller_id, position);

                uint8_t position_bytes[4];
                utils_float_to_bytes_be(position, position_bytes, 0);
                ESP_ERROR_CHECK(app_send_response(rx_frame.type, position_bytes, sizeof(position_bytes)));
                break;
            }

            case APP_CMD_MCTRL_DUMP_LOG_CODE:
            {
                uint8_t controller_id = rx_frame.data[0];
                uint32_t sample_count = utils_bytes_to_u32_be(rx_frame.data, 1);

                ESP_LOGI(TAG, "Received MCTRL_DUMP_LOG command for controller ID: %d, sample count: %u", controller_id, sample_count);

                motion_ctrl_log_sample_t *log_samples = malloc(sample_count * sizeof(motion_ctrl_log_sample_t));
                if (log_samples == NULL)
                {
                    ESP_LOGE(TAG, "Failed to allocate memory for log samples");
                    break;
                }

                size_t actual_sample_count = 0;
                ESP_ERROR_CHECK_WITHOUT_ABORT(motion_ctrl_dump_log(&motion_controller[controller_id], log_samples, sample_count, &actual_sample_count));

                ESP_LOGI(TAG, "Dumped %zu log samples for controller ID: %d", actual_sample_count, controller_id);

                size_t response_size = actual_sample_count * sizeof(motion_ctrl_log_sample_t);
                ESP_ERROR_CHECK(app_send_response(rx_frame.type, (uint8_t *)log_samples, response_size));
                free(log_samples);
                break;
            }

            case APP_CMD_MCTRL_RESET_WATCHDOG_CODE:
            {
                uint8_t controller_id = rx_frame.data[0];
                ESP_LOGI(TAG, "Received MCTRL_RESET_WATCHDOG command for controller ID: %d", controller_id);
                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, motion_ctrl_watchdog_reset(&motion_controller[controller_id])));
                break;
            }

            default:
                ESP_LOGW(TAG, "Received unknown command type: 0x%02X", rx_frame.type);
                break;
        }
    }
}

static esp_err_t app_send_ack(const uint8_t command_type, const esp_err_t status_code)
{
    framed_serial_frame_t tx_frame = {.type = APP_CMD_ACK_CODE(command_type), .data_length = 2, .checksum = 0};
    utils_u16_to_bytes_be((uint16_t)status_code, tx_frame.data, 0);
    const size_t tx_frame_size = sizeof(tx_frame) - FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH + tx_frame.data_length;

    if (xRingbufferSend(framed_serial_tx_buffer, &tx_frame, tx_frame_size, APP_TX_BUFFER_TIMEOUT_MS) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to send ACK for command type: 0x%02X", command_type);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ACK sent for command type: 0x%02X with status code: %u", command_type, (unsigned int)status_code);
    return ESP_OK;
}

static esp_err_t app_send_response(const uint8_t command_type, const uint8_t *data, const size_t data_length)
{
    if (data_length <= FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH)
    {
        framed_serial_frame_t tx_frame = {.type = APP_CMD_ACK_CODE(command_type), .data_length = data_length, .checksum = 0};
        memcpy(tx_frame.data, data, data_length);
        const size_t tx_frame_size = sizeof(tx_frame) - FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH + tx_frame.data_length;

        if (xRingbufferSend(framed_serial_tx_buffer, &tx_frame, tx_frame_size, APP_TX_BUFFER_TIMEOUT_MS) != pdTRUE)
        {
            ESP_LOGE(TAG, "Failed to send response for command type: 0x%02X", command_type);
            return ESP_FAIL;
        }
    }
    else
    {
        const size_t frames_needed = data_length / FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH + ((data_length % FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH) ? 1 : 0);
        for (size_t i = 0; i < frames_needed; ++i)
        {
            size_t chunk_size = FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH;
            if (i == frames_needed - 1 && (data_length % FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH) != 0)
            {
                chunk_size = data_length % FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH;
            }

            framed_serial_frame_t tx_frame = {.type = APP_CMD_ACK_CODE(command_type), .data_length = chunk_size, .checksum = 0};
            memcpy(tx_frame.data, data + i * FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH, chunk_size);
            const size_t tx_frame_size = sizeof(tx_frame) - FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH + tx_frame.data_length;

            if (xRingbufferSend(framed_serial_tx_buffer, &tx_frame, tx_frame_size, APP_TX_BUFFER_TIMEOUT_MS) != pdTRUE)
            {
                ESP_LOGE(TAG, "Failed to send response frame %zu/%zu for command type: 0x%02X", i + 1, frames_needed, command_type);
                return ESP_FAIL;
            }

            /* Allow some breathing room for framed serial communication */
            vTaskDelay(pdMS_TO_TICKS(APP_TX_BUFFER_INTER_FRAME_DELAY_MS));
        }
    }

    ESP_LOGI(TAG, "Response sent for command type: 0x%02X with data length: %zu", command_type, data_length);
    return ESP_OK;
}