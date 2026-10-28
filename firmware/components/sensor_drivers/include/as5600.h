#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>
#include <stdatomic.h>

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Preprocessor directive to avoid C++ name mangling */
#ifdef __cplusplus
extern "C" 
{
#endif

#define AS5600_TELEMETRY_TASK_PRIORITY      (8)
#define AS5600_TELEMETRY_TASK_STACK_SIZE    (4096)
#define AS5600_TELEMETRY_TASK_INTERVAL_MS   (10)

#define AS5600_I2C_ADDRESS              (0x36) 
#define AS5600_I2C_SPEED_HZ             (100000) 
#define AS5600_I2C_TIMEOUT_MS           (1000)

#define AS5600_REG_ANGLE1               (0x0E)
#define AS5600_REG_ANGLE1_MASK          (0x0F)
#define AS5600_REG_ANGLE1_SHIFT         (8)
#define AS5600_REG_ANGLE2               (0x0F)
#define AS5600_REG_ANGLE2_MASK          (0xFF)
#define AS5600_REG_ANGLE2_SHIFT         (0)
#define AS5600_ANGLE_FACTOR             (360.0f / 4096.0f) /* 12-bit resolution, 360 degrees */

#define AS5600_REG_CONF1                (0x07)
#define AS5600_REG_CONF2                (0x08)
#define AS5600_REG_CONF1_WD_MASK        (0x20)
#define AS5600_REG_CONF1_WD_SHIFT       (5)
#define AS5600_REG_CONF1_FTH_MASK       (0x1C)
#define AS5600_REG_CONF1_FTH_SHIFT      (2)
#define AS5600_REG_CONF1_SF_MASK        (0x03)
#define AS5600_REG_CONF1_SF_SHIFT       (0)
#define AS5600_REG_CONF2_HYST_MASK      (0x0C)
#define AS5600_REG_CONF2_HYST_SHIFT     (2)

#define AS5600_REG_STATUS               (0x0B)
#define AS5600_REG_STATUS_MD_MASK       (0x20)
#define AS5600_REG_STATUS_MD_SHIFT      (5)
#define AS5600_REG_STATUS_ML_MASK       (0x10)
#define AS5600_REG_STATUS_ML_SHIFT      (4)
#define AS5600_REG_STATUS_MH_MASK       (0x08)
#define AS5600_REG_STATUS_MH_SHIFT      (3)

typedef struct as5600_config
{
    uint8_t watchdog;               /* Watchdog setting (0-1) */
    uint8_t fast_filter_threshold;  /* Filter threshold (0-7) */
    uint8_t slow_filter;            /* Slow filter setting (0-3) */
    uint8_t hysteresis;             /* Hysteresis setting (0-3) */
} as5600_config_t;

#define AS5600_DEFAULT_CONFIG { \
    .watchdog = 0b0,                /* Disable magnet watchdog for faster response */ \
    .fast_filter_threshold = 0b001, /* Lowest change threshold for the fast filter to kick in */ \
    .slow_filter = 0b11,            /* Fastest step response */ \
    .hysteresis = 0b01,             /* 1 LSB hysteresis, about 0.0879deg, so 1.5cm of error at 10m */ \
}

typedef enum as5600_status
{
    AS5600_MAGNET_DETECTED,
    AS5600_MAGNET_TOO_WEAK,
    AS5600_MAGNET_TOO_STRONG
} as5600_status_t;

/* 
 * AS5600 device handle structure.
 * It is advised to treat this structure as opaque and not access its members directly.
 * Use the provided API functions to interact with the AS5600 sensor.
 */
typedef struct as5600_handle
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t bus_dev_handle;
    TaskHandle_t telemetry_task_handle;
    as5600_config_t config;
    _Atomic uint16_t angle;
    _Atomic uint16_t zero_angle; 
    _Atomic as5600_status_t status;
} as5600_handle_t;

/*
 * Initialize the AS5600 sensor with the provided configurations.
 * Loads the configuration into the sensor and sets up the device handle for further operations.
 *
 * @param bus_handle Handle to the I2C master bus.
 * @param config Pointer to the AS5600 configuration structure.
 * @param dev_handle Pointer to the AS5600 device handle to be initialized.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t as5600_init(i2c_master_bus_handle_t bus_handle, const as5600_config_t *config, as5600_handle_t *dev_handle);

/*
 * Get the status of the AS5600 sensor.
 *
 * @param dev_handle Pointer to the AS5600 device handle.
 * @param status Pointer to the variable where the status will be stored.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t as5600_get_status(as5600_handle_t* dev_handle, as5600_status_t *status);

/*
 * Read the angle from the AS5600 sensor.
 *
 * @param dev_handle Pointer to the AS5600 device handle.
 * @param angle Pointer to the variable where the angle will be stored.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t as5600_read_angle(as5600_handle_t* dev_handle, float *angle);

/*
 * Set the zero angle for the AS5600 sensor.
 *
 * @param dev_handle Pointer to the AS5600 device handle.
 * @param zero_angle The zero angle to set.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t as5600_set_zero_angle(as5600_handle_t* dev_handle, float zero_angle);

/*
 * Get the zero angle for the AS5600 sensor.
 *
 * @param dev_handle Pointer to the AS5600 device handle.
 * @param zero_angle Pointer to the variable where the zero angle will be stored.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t as5600_get_zero_angle(as5600_handle_t* dev_handle, float *zero_angle);

#ifdef __cplusplus
}
#endif

#endif /* AS5600_H */