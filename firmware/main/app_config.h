#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"
#include "driver/ledc.h"

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
#define APP_CMD_MCTRL_STOP_CODE           (0x12u) /* [0] Controller ID                           */
#define APP_CMD_MCTRL_RESET_WATCHDOG_CODE (0x14u) /* [0] Controller ID                           */

#define APP_CMD_SERVO_SET_POS   (0x15u) /* [0 - 3] End position */
#define APP_CMD_SERVO_GET_POS   (0x16u) /* No arguments         */
#define APP_CMD_SERVO_TRIGGER   (0x17u) /* No arguments         */

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

/* -------------------- Servo ------------------- */

#define APP_SERVO_PWM_PIN           (GPIO_NUM_38)
#define APP_SERVO_LEDC_TIMER        (LEDC_TIMER_0)
#define APP_SERVO_LEDC_SPEED_MODE   (LEDC_LOW_SPEED_MODE)
#define APP_SERVO_LEDC_CHANNEL      (LEDC_CHANNEL_0)
#define APP_SERVO_MIN_WIDTH_US      (1000)
#define APP_SERVO_MAX_WIDTH_US      (2000)

#define APP_SERVO_INITIAL_POSITION  (110.0f)
#define APP_SERVO_TRIGGER_POSITION  (70.0f)
#define APP_SERVO_TRIGGER_PERIOD_MS (300)

/* -------------- Motion controller ------------- */

#define APP_MOTION_CONTROLLER_COUNT              (2)
#define APP_MOTION_CONTROLLER_POSITION_THRESHOLD (0.1f) /* Degrees */

#define APP_MOTION_CONTROLLER_A_MAX_POSITION    (6.85546875f)
#define APP_MOTION_CONTROLLER_A_MID_POSITION    (275.712890625f)
#define APP_MOTION_CONTROLLER_A_MIN_POSITION    (185.9765625f)

#define APP_MOTION_CONTROLLER_B_MAX_POSITION    (96.591796875f)
#define APP_MOTION_CONTROLLER_B_MID_POSITION    (51.240234375f)
#define APP_MOTION_CONTROLLER_B_MIN_POSITION    (8.173828125f)

/* Values derived from empirical data */

/* 
 * omega_c = 35.0 
 * omega_o = 120.0
 */
#define APP_MOTION_CONTROLLER_A_STEP_SCALE      (-85.6549479971f)
#define APP_MOTION_CONTROLLER_A_KV              (0.013825927156620f)
#define APP_MOTION_CONTROLLER_A_TAU             (0.011705960531035f)
#define APP_MOTION_CONTROLLER_A_A_ESO           {{ 6.42011376e-02,  4.31685567e-04, 2.46206563e-06}, \
                                                 {-5.82377165e+01,  3.40078209e-02, 4.49058221e-03}, \
                                                 {-5.07510084e+03, -3.41247502e+01, 8.05373677e-01}}
#define APP_MOTION_CONTROLLER_A_B_ESO           {2.90794932e-06, 5.30383324e-03, -2.29873435e-01}
#define APP_MOTION_CONTROLLER_A_L               {9.3580e-01, 5.8238e+01, 5.0751e+03}
#define APP_MOTION_CONTROLLER_A_K               {1.0981e+03, -4.2207e+00, 8.4667e-01}

/* 
 * omega_c = 15.0 
 * omega_o = 120.0
 */
#define APP_MOTION_CONTROLLER_B_STEP_SCALE      (-52.9082338537f)
#define APP_MOTION_CONTROLLER_B_KV              (0.018695321366165f)
#define APP_MOTION_CONTROLLER_B_TAU             (0.027931540966981f)
#define APP_MOTION_CONTROLLER_B_A_ESO           {{ 3.90863569e-02,  3.28548507e-04, 1.74055572e-06}, \
                                                 {-7.77782111e+01,  4.52794142e-02, 4.94216426e-03}, \
                                                 {-4.05971154e+03, -3.41247502e+01, 8.19216865e-01}}
#define APP_MOTION_CONTROLLER_B_B_ESO           {1.16500012e-06, 3.30792165e-03, -1.21002948e-01}
#define APP_MOTION_CONTROLLER_B_L               {9.6091e-01, 7.7778e+01, 4.0597e+03}
#define APP_MOTION_CONTROLLER_B_K               {2.2728e+02, -1.4496e+01, 1.4940e+00}

#define APP_MOTION_CONTROLLER_LOG_CHUNK_SIZE    (100)

#endif /* CONFIG_H */