#ifndef INA226_H
#define INA226_H

#include <stdint.h>
#include <stdatomic.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Preprocessor directive to avoid C++ name mangling */
#ifdef __cplusplus
extern "C" 
{
#endif

#define INA226_TELEMETRY_TASK_PRIORITY      (8)
#define INA226_TELEMETRY_TASK_STACK_SIZE    (4096)
#define INA226_TELEMETRY_TASK_INTERVAL_MS   (15)

/* No static I2C address defined, device supports multiple addresses */
#define INA226_I2C_SPEED_HZ             (100000)
#define INA226_I2C_TIMEOUT_MS           (200) 
#define INA226_INTR_PRIORITY            (ESP_INTR_FLAG_LEVEL3)

#define INA226_REG_CALIBRATION          (0x05)
/*
 * Value for the calibration register is to be calculated in the following manner:
 * Current_LSB = Max_expected_current / 2^15
 * Calibration = 0.00512 / (Current_LSB * R_shunt)
 * It's recommended to round up the Current_LSB number for easier calculations.
 */
#define INA226_CURRENT_LSB_F            (2.5f / 32768.0f)                               /* For 2.5 A peak current */
#define INA226_CURRENT_CALIBRATION_F    (0.00512f / (INA226_CURRENT_LSB_F * 0.025f))    /* For 0.025 Ohm shunt */
#define INA226_CURRENT_CALIBRATION      ((uint16_t)INA226_CURRENT_CALIBRATION_F)

#define INA226_REG_SHUNT_VOLTAGE        (0x01)
#define INA226_SHUNT_VOLTAGE_FACTOR     (2.5f*1e-6f) /* 2.5uV per bit */

#define INA226_REG_BUS_VOLTAGE          (0x02)
#define INA226_BUS_VOLTAGE_FACTOR       (1.25f*1e-3f) /* 1.25mV per bit */

#define INA226_REG_CURRENT              (0x04)
#define INA226_CURRENT_FACTOR           (INA226_CURRENT_LSB_F)

#define INA226_REG_POWER                (0x03)
#define INA226_POWER_FACTOR             (INA226_CURRENT_LSB_F * 25.0f) /* Power LSB = 25 * Current LSB */

#define INA226_REG_ALERT_LIMIT          (0x07)
#define INA226_ALERT_LIMIT_FACTOR       (0.0025f) /* 2.5 mV per bit */
#define INA226_ALERT_LIMIT_DEFAULT      (uint16_t)(0.05f / INA226_ALERT_LIMIT_FACTOR) /* 50 mV (2 A) alert limit for shunt overvoltage */

#define INA226_REG_CONFIG               (0x00)
#define INA226_REG_CONFIG_AVG_MASK      (0x0E00)
#define INA226_REG_CONFIG_AVG_SHIFT     (9)
#define INA226_REG_CONFIG_VBUSCT_MASK   (0x01C0)
#define INA226_REG_CONFIG_VBUSCT_SHIFT  (6)
#define INA226_REG_CONFIG_VSHCT_MASK    (0x0038)
#define INA226_REG_CONFIG_VSHCT_SHIFT   (3)
#define INA226_REG_CONFIG_MODE_MASK     (0x0007)
#define INA226_REG_CONFIG_MODE_SHIFT    (0)

#define INA226_REG_ALERT                (0x06)
#define INA226_REG_ALERT_FUNC_MASK      (0xFC00)
#define INA226_REG_ALERT_FUNC_SHIFT     (10)
#define INA226_REG_ALERT_AFF_MASK       (0X0010)
#define INA226_REG_ALERT_AFF_SHIFT      (4)
#define INA226_REG_ALERT_CVRF_MASK      (0x0008)
#define INA226_REG_ALERT_CVRF_SHIFT     (3)
#define INA226_REG_ALERT_OVF_MASK       (0x0004)
#define INA226_REG_ALERT_OVF_SHIFT      (2)
#define INA226_REG_ALERT_APOL_MASK      (0x0002)
#define INA226_REG_ALERT_APOL_SHIFT     (1)
#define INA226_REG_ALERT_LEN_MASK       (0x0001)
#define INA226_REG_ALERT_LEN_SHIFT      (0)

typedef enum ina226_alert_function
{
    INA226_ALERT_FUNCTION_DISABLED = 0b000000,
    INA226_ALERT_FUNCTION_SHUNT_OVER_LIMIT = 0b100000,
    INA226_ALERT_FUNCTION_SHUNT_UNDER_LIMIT = 0b010000,
    INA226_ALERT_FUNCTION_BUS_OVER_LIMIT = 0b001000,
    INA226_ALERT_FUNCTION_BUS_UNDER_LIMIT = 0b000100,
    INA226_ALERT_FUNCTION_POWER_OVER_LIMIT = 0b000010,
    INA226_ALERT_FUNCTION_CONVERSION_READY = 0b000001
} ina226_alert_function_t;

typedef struct ina226_config
{
    uint8_t averaging;                      /* Averaging mode (0-7) */
    uint8_t vbus_conversion_time;           /* Bus voltage conversion time (0-7) */
    uint8_t vsh_conversion_time;            /* Shunt voltage conversion time (0-7) */
    uint8_t operating_mode;                 /* Operating mode (0-7) */
    uint16_t calibration_value;             /* Calibration value for current measurement */
    uint16_t alert_limit;                   /* Alert limit value for the alert function */
    ina226_alert_function_t alert_function; /* Alert function configuration */
    bool alert_active_high;                 /* Alert polarity configuration */
    bool alert_latch;                       /* Alert latch configuration */
} ina226_config_t;

#define INA226_DEFAULT_CONFIG { \
    .averaging = 0b001,                                 /* 4 samples per read */ \
    .vbus_conversion_time = 0b100,                      /* 1.1 ms conversion time */ \
    .vsh_conversion_time = 0b100,                       /* 1.1 ms conversion time */ \
    .operating_mode = 0b111,                            /* Continuous shunt and bus voltage measurement */ \
    .calibration_value = INA226_CURRENT_CALIBRATION,    /* Calibration value macro */ \
    .alert_limit = INA226_ALERT_LIMIT_DEFAULT,          /* Shunt limit macro */ \
    .alert_function = INA226_ALERT_FUNCTION_SHUNT_OVER_LIMIT, \
    .alert_active_high = false,                         /* Active low alert */ \
    .alert_latch = false,                               /* Non-latching alert */ \
}

/* Forward declaration of the INA226 handle */
typedef struct ina226_handle ina226_handle_t;

/* Alert callback function type */
typedef void (*ina226_alert_callback_t)(ina226_handle_t dev_handle, void *arg);

/*
 * INA226 device handle structure.
 * It is advised to treat this structure as opaque and not access its members directly.
 * Use the provided API functions to interact with the INA226 sensor.
 */
typedef struct ina226_handle
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t bus_dev_handle;
    TaskHandle_t telemetry_task_handle;
    ina226_config_t config;
    _Atomic uint16_t shunt_voltage; 
    _Atomic uint16_t bus_voltage; 
    _Atomic uint16_t power; 
    _Atomic uint16_t current; 
    ina226_alert_callback_t alert_callback;
    void *alert_callback_arg;
} ina226_handle_t;

/*
 * Initialize the INA226 sensor with the provided configurations.
 * Loads the configuration into the sensor and sets up the device handle for further operations.
 * 
 * @param bus_handle Handle to the I2C master bus.
 * @param config Pointer to the INA226 configuration structure.
 * @param i2c_address Physically configured I2C address of the INA226 sensor.
 * @param dev_handle Pointer to the INA226 device handle to be initialized.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t ina226_init(i2c_master_bus_handle_t bus_handle, const ina226_config_t *config, const uint8_t i2c_address, ina226_handle_t *dev_handle); 

/* 
 * Register an alert interrupt callback function for the INA226 sensor.
 * This function will be called when an alert is triggered.
 * 
 * @param dev_handle Pointer to the INA226 device handle.
 * @param alert_gpio_num The GPIO number of the alert pin.
 * @param callback The alert callback function to register.
 * @param arg User-defined argument to pass to the callback function.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t ina226_register_callback(ina226_handle_t *dev_handle, const gpio_num_t alert_gpio_num, ina226_alert_callback_t callback, void *arg);

/*
 * Read the shunt voltage from the INA226 sensor.
 * 
 * @param dev_handle Pointer to the INA226 device handle.
 * @param shunt_voltage Pointer to the variable to store the shunt voltage in V.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t ina226_read_shunt_voltage(ina226_handle_t *dev_handle, float *shunt_voltage);

/*
 * Read the bus voltage from the INA226 sensor.
 * 
 * @param dev_handle Pointer to the INA226 device handle.
 * @param bus_voltage Pointer to the variable to store the bus voltage in V.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t ina226_read_bus_voltage(ina226_handle_t *dev_handle, float *bus_voltage);

/*
 * Read the power from the INA226 sensor.
 * 
 * @param dev_handle Pointer to the INA226 device handle.
 * @param power Pointer to the variable to store the power in W.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t ina226_read_power(ina226_handle_t *dev_handle, float *power);

/*
 * Read the current from the INA226 sensor.
 * 
 * @param dev_handle Pointer to the INA226 device handle.
 * @param current Pointer to the variable to store the current in A.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t ina226_read_current(ina226_handle_t *dev_handle, float *current);

#ifdef __cplusplus
}
#endif

#endif /* INA226_H */