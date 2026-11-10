#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

/* --------------- Command executor -------------- */

#define APP_EXECUTOR_TASK_PRIORITY   (10)
#define APP_EXECUTOR_TASK_STACK_SIZE (16384) /* Might be an overkill, but better safe than sorry */

#define APP_TX_BUFFER_TIMEOUT_MS            (1000)
#define APP_TX_BUFFER_INTER_FRAME_DELAY_MS  (20)

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

#define APP_CMD_MCTRL_MOVE_CODE           (0x0Eu) /* [0] Controller ID, [1 - 4] End position     */
#define APP_CMD_MCTRL_GET_POS_CODE        (0x0Fu) /* [0] Controller ID                           */
#define APP_CMD_MCTRL_DUMP_LOG_CODE       (0x10u) /* [0] Controller ID, [1 - 4] Sample count     */
#define APP_CMD_MCTRL_DUMP_LOG_COUNT_RESP (0x11u) /* [0 - 3] Actual sample count dumped, RX only */
#define APP_CMD_MCTRL_RESET_WATCHDOG_CODE (0x14u) /* [0] Controller ID                           */

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

#define APP_MOTION_CONTROLLER_A_MAX_POSITION    (6.85546875f)
#define APP_MOTION_CONTROLLER_A_MID_POSITION    (275.712890625f)
#define APP_MOTION_CONTROLLER_A_MIN_POSITION    (185.9765625f)
#define APP_MOTION_CONTROLLER_A_STEP_SCALE      (85.6549479971f)

/* Values derived from empirical data */
#define APP_MOTION_CONTROLLER_A_KV              (0.010636921289335f)
#define APP_MOTION_CONTROLLER_A_TAU             (0.012944388107410f)
#define APP_MOTION_CONTROLLER_A_A_ESO           {{ 5.91626736e-02,  4.12135741e-04, 2.32340110e-06}, \
                                                 {-6.10155996e+01,  3.67970707e-02, 4.56997652e-03}, \
                                                 {-4.89865657e+03, -3.41247502e+01, 8.07622891e-01}}
#define APP_MOTION_CONTROLLER_A_B_ESO           {1.90923159e-06, 3.75533244e-03, -1.58083962e-01}
#define APP_MOTION_CONTROLLER_A_L               {9.4084e-01, 6.1016e+01, 4.8987e+03}
#define APP_MOTION_CONTROLLER_A_K               {3.5562e+03, 4.3578e+01, 1.2169e+00}

#define APP_MOTION_CONTROLLER_B_MAX_POSITION    (96.591796875f)
#define APP_MOTION_CONTROLLER_B_MID_POSITION    (51.240234375f)
#define APP_MOTION_CONTROLLER_B_MIN_POSITION    (8.173828125f)

/* Values derived from empirical data */
#define APP_MOTION_CONTROLLER_B_STEP_SCALE      (52.9082338537f)
#define APP_MOTION_CONTROLLER_B_KV              (0.017561605479475f)
#define APP_MOTION_CONTROLLER_B_TAU             (0.047899584948521f)
#define APP_MOTION_CONTROLLER_B_A_ESO           {{ 3.36672433e-02,  3.03852015e-04, 1.57208439e-06}, \
                                                 {-8.47753099e+01,  4.64716907e-02, 5.06658869e-03}, \
                                                 {-3.78107174e+03, -3.41247502e+01, 8.23443702e-01}}
#define APP_MOTION_CONTROLLER_B_B_ESO           {5.76379228e-07, 1.85758252e-03, -6.47315015e-02}
#define APP_MOTION_CONTROLLER_B_L               {9.6633e-01, 8.4775e+01, 3.7811e+03}
#define APP_MOTION_CONTROLLER_B_K               {1.8024e+03, 8.1343e+01, 2.7275e+00}


#define APP_MOTION_CONTROLLER_LOG_CHUNK_SIZE    (100)

#endif /* CONFIG_H */