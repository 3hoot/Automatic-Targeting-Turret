#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

/* --------------- Command executor -------------- */

#define APP_EXECUTOR_TASK_PRIORITY   (10)
#define APP_EXECUTOR_TASK_STACK_SIZE (4096)

#define APP_TX_BUFFER_TIMEOUT_MS            (1000)
#define APP_TX_BUFFER_INTER_FRAME_DELAY_MS  (30)

/* All command arguments are assumed to be in big-endian format. */

#define APP_CMD_ECHO_CODE           (0x01u) /* [0 - 127] Anything  */
#define APP_CMD_ACK_CODE(code)      (code)  /* [0 - 1] Status code */
#define APP_CMD_CLEAR_RX            (0x02u) /* No arguments        */

#define APP_CMD_STEP_ARM_CODE       (0x03u) /* [0] Stepper ID                           */
#define APP_CMD_STEP_DISARM_CODE    (0x04u) /* [0] Stepper ID                           */
#define APP_CMD_STEP_GET_POS_CODE   (0x08u) /* [0] Stepper ID                           */

#define APP_CMD_SENSE_GET_POS_CODE  (0x09u) /* [0] Sensor ID                            */
#define APP_CMD_SENSE_GET_PWR_CODE  (0x0Bu) /* [0] Sensor ID                            */
#define APP_CMD_SENSE_GET_CURR_CODE (0x0Cu) /* [0] Sensor ID                            */
#define APP_CMD_SENSE_GET_VOLT_CODE (0x0Du) /* [0] Sensor ID                            */

#define APP_CMD_MCTRL_MOVE_CODE           (0x0Eu) /* [0] Controller ID, [1 - 4] End position, [5 - 8] Step frequency, [9] Block until complete */
#define APP_CMD_MCTRL_GET_POS_CODE        (0x0Fu) /* [0] Controller ID                                                                         */
#define APP_CMD_MCTRL_DUMP_LOG_CODE       (0x10u) /* [0] Controller ID, [1 - 4] Sample count                                                   */
#define APP_CMD_MCTRL_RESET_WATCHDOG_CODE (0x14u) /* [0] Controller ID                                                                         */

/* ------------------- Stepper ------------------ */

#define APP_STEPPER_COUNT                   (2) 
#define APP_STEPPER_DEFAULT_FREQUENCY_HZ    (1000) 

#define APP_STEPPER_A_EN_PIN    (GPIO_NUM_4)
#define APP_STEPPER_A_DIR_PIN   (GPIO_NUM_5)
#define APP_STEPPER_A_STEP_PIN  (GPIO_NUM_6)
#define APP_STEPPER_B_EN_PIN    (GPIO_NUM_7)
#define APP_STEPPER_B_DIR_PIN   (GPIO_NUM_15)
#define APP_STEPPER_B_STEP_PIN  (GPIO_NUM_16)

/* ------------------- I2C bus ------------------ */

#define APP_I2C_BUS_A_INTERRUPT_PRIORITY    (1)
#define APP_I2C_BUS_A_SDA_PIN               (GPIO_NUM_18)
#define APP_I2C_BUS_A_SCL_PIN               (GPIO_NUM_17)

#define APP_I2C_BUS_B_INTERRUPT_PRIORITY    (1)
#define APP_I2C_BUS_B_SDA_PIN               (GPIO_NUM_13)
#define APP_I2C_BUS_B_SCL_PIN               (GPIO_NUM_12)

/* Joined search macros */
#define APP_I2C_SCAN_TIMEOUT_MS                 (100)
#define APP_I2C_SCAN_MAX_FOUND_DEVICES          (10)
#define APP_I2C_SCAN_FOUND_DEVICES_ARRAY_SIZE   (APP_I2C_SCAN_MAX_FOUND_DEVICES)

/* ------------------- AS5600 ------------------- */

#define APP_AS5600_COUNT (2)

/* ------------------- INA226 ------------------- */

#define APP_INA226_COUNT            (2)
#define APP_INA226_A_I2C_ADDRESS    (0x44) 
#define APP_INA226_B_I2C_ADDRESS    (0x44)
#define APP_INA226_A_ALERT_GPIO_PIN (GPIO_NUM_8)
#define APP_INA226_B_ALERT_GPIO_PIN (GPIO_NUM_14)

/* -------------- Motion controller ------------- */

#define APP_MOTION_CONTROLLER_COUNT              (2)
#define APP_MOTION_CONTROLLER_POSITION_THRESHOLD (0.5f) /* Degrees */

#define APP_MOTION_CONTROLLER_A_MAX_POSITION (6.85546875f)
#define APP_MOTION_CONTROLLER_A_MID_POSITION (275.712890625f)
#define APP_MOTION_CONTROLLER_A_MIN_POSITION (185.9765625f)

#define APP_MOTION_CONTROLLER_B_MAX_POSITION (96.591796875f)
#define APP_MOTION_CONTROLLER_B_MID_POSITION (51.240234375f)
#define APP_MOTION_CONTROLLER_B_MIN_POSITION (8.173828125f)

#endif /* CONFIG_H */