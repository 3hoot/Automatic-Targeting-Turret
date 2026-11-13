#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"

#include "as5600.h"
#include "ina226.h"
#include "stepper_driver.h"

/* Preprocessor directive to avoid C++ name mangling */
#ifdef __cplusplus
extern "C" 
{
#endif

/* -------------- Main loop / init -------------- */

#define MOTION_CTRL_LOOP_TASK_PRIORITY          (12)
#define MOTION_CTRL_LOOP_TASK_STACK_SIZE        (4096)
#define MOTION_CTRL_LOOP_TASK_INTERVAL_MS       (10)
#define MOTION_CTRL_LOOP_TASK_ERROR_RESOLVE_MS  (100)

/* Sample time of the loop in seconds. The ESO matrices in the config must be discretized with this value. */
#define MOTION_CTRL_LOOP_TASK_DT_S              ((float)MOTION_CTRL_LOOP_TASK_INTERVAL_MS / 1000.0f)

#define MOTION_CTRL_FINISH_MOVES            (0)       /* 1: main loop stops after the dest. is reached, 0: loop fires continuously (with hysteresis) */
#define MOTION_CTRL_SETTLE_TIMEOUT_S        (3.0f)
#define MOTION_CTRL_SETTLE_TIMEOUT_FATAL    (0)       /* 1: timeout flags MOTION_CTRL_STATE_ERROR, 0: just stop and go idle */
#define MOTION_CTRL_SENSOR_FAIL_LIMIT       (5)       /* Consecutive position sensor failures before flagging an error */
#define MOTION_CTRL_SENSOR_LAG_S            (0.02f)
#define MOTION_CTRL_VELOCITY_HEADROOM       (0.8f)
#define MOTION_CTRL_ACCEL_HEADROOM          (0.6f)

#define MOTION_CTRL_MISSED_STEPS_REGISTER_THRESHOLD (100)   /* What constitutes to registering a lost step */
#define MOTION_CTRL_MISSED_STEPS_COUNT_THRESHOLD    (10)    /* If stepper loses steps 20 times within set period, we flag an error */
#define MOTION_CTRL_MISSED_STEPS_DECAY_PERIOD_MS    (1000)  /* Period for missed steps detection */

typedef enum motion_ctrl_state
{
    MOTION_CTRL_STATE_IDLE = 0,
    MOTION_CTRL_STATE_CALIBRATING,
    MOTION_CTRL_STATE_MOVING,
    MOTION_CTRL_STATE_ERROR
} motion_ctrl_state_t;

typedef struct motion_ctrl_limits
{
    float max_position;
    float mid_position;
    float min_position;
} motion_ctrl_limits_t;

typedef struct motion_ctrl_config 
{
    /* Hardware */
    stepper_drv_handle_t* stepper;
    as5600_handle_t* position_sensor;
    ina226_handle_t* power_sensor;
    
    /* Positioning */
    motion_ctrl_limits_t limits;
    float position_threshold;
    float step_scale;
    
    /* ADRC controller */
    float Kv;       /* Plant gain, deg/s per Hz */
    float Tau;      /* Plant time constant, s */
    float A_eso[3][3];
    float B_eso[3];
    float L[3];
    float K[3]; 
} motion_ctrl_config_t;

/*
 * Motion controller handle structure.
 * It is advised to treat this structure as opaque and not access its members directly.
 * Use the provided API functions to interact with the motion controller.
 */
typedef struct motion_ctrl_handle
{
    /* Configuration */
    motion_ctrl_config_t config;
    float origin_position;
    float position_range;
    bool flip_sensor_direction;
    _Atomic bool stop_requested;
    _Atomic bool move_requested;
    _Atomic bool configured;

    /* State and control */
    _Atomic motion_ctrl_state_t state;
    _Atomic float target_position;
    _Atomic float current_position;
    _Atomic float current_velocity;

    /* Logging */
    RingbufHandle_t log_buffer;
    QueueHandle_t log_queue;        /* Length 1 mailbox: control loop -> log task */
    
    /* Reference generation limits */
    float v_max;
    float a_max;

    /* ADRC states */
    float x_hat_prev[3];
} motion_ctrl_handle_t;

/*
 * Initializes the motion controller.
 * Creates the control loop, watchdog and log tasks.
 *
 * @param config Pointer to the motion controller configuration.
 * @param ctrl_handle Pointer to the motion controller handle.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t motion_ctrl_init(const motion_ctrl_config_t *config, motion_ctrl_handle_t *ctrl_handle);

/*
 * Gets the current position of the motion controller.
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param position Pointer to the variable that will hold the current position.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t motion_ctrl_get_position(motion_ctrl_handle_t *ctrl_handle, float *position);

/*
 * Gets the current state of the motion controller.
 * State changes requested through the API (e.g. a move) are applied by the control loop,
 * so they can take up to one loop period to show up here.
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param state Pointer to the variable that will hold the current state.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t motion_ctrl_get_state(motion_ctrl_handle_t *ctrl_handle, motion_ctrl_state_t *state);

/*
 * Moves the motion controller to a specific position.
 * Can be called while a move is in progress: the reference generator retargets on the fly,
 * continuing from its current position and velocity (no stop in between).
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param position The target position to move to (clamped to the position range).
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t motion_ctrl_move(motion_ctrl_handle_t *ctrl_handle, float position);

/* 
 * Stops the motion controller.
 * Drops any pending move and soft-stops the stepper.
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t motion_ctrl_stop(motion_ctrl_handle_t *ctrl_handle);

/* ------------------- Watchdog ----------------- */

#define MOTION_CTRL_WATCHDOG_TASK_PRIORITY          (13)
#define MOTION_CTRL_WATCHDOG_TASK_STACK_SIZE        (3072)
#define MOTION_CTRL_WATCHDOG_TASK_INTERVAL_MS       (100)

/*
 * Resets the motion controller's watchdog.
 * Clears an error raised by the watchdog or the control loop (MOTION_CTRL_STATE_ERROR -> MOTION_CTRL_STATE_IDLE)
 * and drops any stale stop/move request. Does nothing in any other state.
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t motion_ctrl_watchdog_reset(motion_ctrl_handle_t *ctrl_handle);

/* ------------------- Logging ------------------ */

#define MOTION_CTRL_LOG_SAMPLE_COUNT            (5000)
#define MOTION_CTRL_LOG_TASK_PRIORITY           (5)
#define MOTION_CTRL_LOG_TASK_STACK_SIZE         (3072)

/*  
 * When the motion controller's state is set to moving, 
 * the controller will log samples and dump them continuously to the log buffer.
 * Each motion_ctrl handle has its own log buffer.
 *
 * The control loop only publishes the control-side values; a separate low priority log task adds the
 * INA226 readings (so the slow I2C transactions never delay the control loop) and pushes the sample.
 * Current and voltage are therefore a few ms newer than the timestamp, and if the log task ever falls
 * behind, the oldest unlogged sample is overwritten.
 */

typedef struct __attribute__((packed)) motion_ctrl_log_sample
{
    int64_t timestamp_us;
    float current;          /* A */
    float voltage;          /* V */
    float angle;            /* Raw sensor angle, deg */
    float position;         /* Controller-frame position, deg (what the control law sees) */
    float target_position;  /* Controller-frame target, deg */
    int32_t step_position;  /* Stepper step counter */
    int32_t commanded_hz;   /* u_k: signed command computed by the control law */
    int32_t applied_hz;     /* Signed frequency actually being generated (after ramp limiting) */
    float x_hat1;           /* ESO state 1: estimated position */
    float x_hat2;           /* ESO state 2: estimated velocity */
    float x_hat3;           /* ESO state 3: estimated disturbance */
    float ref_position;     /* Reference generator position r, deg */
    float ref_velocity;     /* Reference generator velocity dr, deg/s */
} motion_ctrl_log_sample_t;

/*
 * Dumps the log samples from the motion controller's log buffer.
 * Throws a warning if the motion controller is not idle, as the log may be inaccurate.
 * Dumped samples are removed from the buffer.
 *
 * @param handle Pointer to the motion controller handle.
 * @param samples Pointer to the buffer where the log samples will be stored.
 * @param sample_count How many samples to dump. If the log buffer has fewer samples than this, it will dump all available samples.
 * @param actual_sample_count Pointer to the variable that will hold the actual number of samples dumped.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t motion_ctrl_dump_log(motion_ctrl_handle_t *handle, motion_ctrl_log_sample_t *samples, uint32_t sample_count, size_t* actual_sample_count);

/*
 * Gets the number of log samples currently stored in the motion controller's log buffer.
 *
 * @param handle Pointer to the motion controller handle.
 * @param sample_count Pointer to the variable that will hold the number of samples in the log buffer.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t motion_ctrl_dump_log_count(motion_ctrl_handle_t *handle, size_t* sample_count);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROLLER_H */