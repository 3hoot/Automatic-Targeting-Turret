#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <math.h>   

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "motion_controller.h"
#include "motion_controller_utils.h"
#include "as5600.h"
#include "ina226.h"
#include "stepper_driver.h"

static const char *TAG = "motion_controller";

/* -------------- Helper functions -------------- */

/*
 * Update the motion controller's origin position and range based on the current limits.
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 */
static void motion_ctrl_update_range(motion_ctrl_handle_t *ctrl_handle);

/*
 * Compare two motion_ctrl_move_t structures for equality.
 *
 * @param move1 Pointer to the first move structure.
 * @param move2 Pointer to the second move structure.
 * @return true if the moves are equal, false otherwise.
 */
static bool motion_ctrl_same_moves(const motion_ctrl_move_t *move1, const motion_ctrl_move_t *move2);

/* ------------------- Logging ------------------ */

/*
 * Push a sample into the log ring buffer, evicting the oldest sample if the buffer is full.
 * Called from the control loop only.
 * 
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param sample Pointer to the sample to be pushed into the log buffer.
 */
static void motion_ctrl_log_push(motion_ctrl_handle_t *ctrl_handle, const motion_ctrl_log_sample_t *sample);

/* ---------------- Control law ----------------- */
 
/*
 * THE controller. Pure function of (target, measurement) -> signed step frequency command [Hz].
 * Positive = towards larger controller-frame position. Magnitude is clamped by the driver to its max frequency;
 * the driver additionally rate-limits the change (acceleration), which is an actuator constraint to model/anti-windup for.
 *
 * @param move Active move (target, requested cruise frequency).
 * @param position Current controller-frame position in degrees.
 * @param position_error move->end_position - position.
 */
static int32_t motion_ctrl_compute_command(const motion_ctrl_move_t *move, float position, float position_error);
 
/*
 * Turns the signed command into stepper driver calls (enable, frequency, direction, safe reversal, stop on zero).
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param command_hz The signed command in Hz.
 */
static void motion_ctrl_apply_command(motion_ctrl_handle_t *ctrl_handle, int32_t command_hz);

/* -------------- Main loop / init -------------- */

/*
 * Motion controller main loop task.
 * This task will continuously monitor the motion controller's state and perform the necessary actions.
 *
 * @param arg Pointer to the motion controller handle.
 */
static void motion_ctrl_loop_task(void *arg);

/*
 * Motion controller timeout callback.
 * This function will be called when the motion controller times out.
 *
 * @param arg Pointer to the motion controller handle.
 */
static void motor_ctrl_timeout_callback(void *arg);

/*
 * Motion controller watchdog task.
 * This task will continuously monitor the motion controller's state.
 * If the motor is overloaded or the position is invalid, the controller will disarm the motor
 * and set the controller state to MOTION_CTRL_STATE_ERROR until reset.
 *
 * @param arg Pointer to the motion controller handle.
 */
static void motor_ctrl_watchdog_task(void *arg);

esp_err_t motion_ctrl_init(const motion_ctrl_config_t *config, motion_ctrl_handle_t *ctrl_handle)
{
    ESP_LOGI(TAG, "Initializing motion controller");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid motion controller configuration");
    ESP_RETURN_ON_FALSE(ctrl_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid motion controller handle");
    ESP_RETURN_ON_FALSE(config->stepper != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid stepper driver handle");
    ESP_RETURN_ON_FALSE(config->position_sensor != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid position sensor handle");
    ESP_RETURN_ON_FALSE(config->power_sensor != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid power sensor handle");

    ctrl_handle->config = *config;
    ctrl_handle->flip_sensor_direction = false;
    atomic_init(&ctrl_handle->state, MOTION_CTRL_STATE_IDLE);
    
    bool limits_set = true;
    if (config->limits.mid_position == 0.0f || config->limits.max_position == 0.0f || config->limits.min_position == 0.0f)
    {
        ESP_LOGW(TAG, "Motion controller limits are not set. Please configure the limits.");
        limits_set = false;
    }
    atomic_init(&ctrl_handle->limits_set, limits_set);

    if (limits_set)
    {
        motion_ctrl_update_range(ctrl_handle);
    }

    ESP_LOGI(TAG, "Creating motion controller log buffer");
    ctrl_handle->log_buffer = xRingbufferCreateNoSplit(sizeof(motion_ctrl_log_sample_t), MOTION_CTRL_LOG_SAMPLE_COUNT);
    ESP_RETURN_ON_FALSE(ctrl_handle->log_buffer != NULL, ESP_ERR_NO_MEM, TAG, "Failed to create motion controller log buffer");

    ESP_LOGI(TAG, "Creating motion controller move loop queue");
    ctrl_handle->move_queue = xQueueCreate(MOTION_CTRL_MOVE_QUEUE_LENGTH, sizeof(motion_ctrl_move_t));
    ESP_RETURN_ON_FALSE(ctrl_handle->move_queue != NULL, ESP_ERR_NO_MEM, TAG, "Failed to create motion controller move queue");

    ESP_LOGI(TAG, "Creating motion controller loop task");
    BaseType_t result = xTaskCreate(motion_ctrl_loop_task, "motion_ctrl_loop", MOTION_CTRL_LOOP_TASK_STACK_SIZE,
                                ctrl_handle, MOTION_CTRL_LOOP_TASK_PRIORITY, NULL);
    ESP_RETURN_ON_FALSE(result == pdPASS, ESP_ERR_NO_MEM, TAG, "Failed to create motion controller loop task");

    ESP_LOGI(TAG, "Creating motion controller watchdog task");
    result = xTaskCreate(motor_ctrl_watchdog_task, "motion_ctrl_watchdog", MOTION_CTRL_WATCHDOG_TASK_STACK_SIZE,
                                ctrl_handle, MOTION_CTRL_WATCHDOG_TASK_PRIORITY, NULL);
    ESP_RETURN_ON_FALSE(result == pdPASS, ESP_ERR_NO_MEM, TAG, "Failed to create motion controller watchdog task");

    const esp_timer_create_args_t timeout_args = {
        .callback = motor_ctrl_timeout_callback,
        .arg = (void*)ctrl_handle,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "motion_ctrl_timeout"
    };
    esp_timer_handle_t timeout_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timeout_args, &timeout_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timeout_timer, MOTION_CTRL_TIMEOUT_US));

    ESP_LOGI(TAG, "Motion controller initialized successfully");
    return ESP_OK;
}

esp_err_t motion_ctrl_get_position(motion_ctrl_handle_t *ctrl_handle, float *position)
{
    ESP_RETURN_ON_FALSE(ctrl_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid motion controller handle");
    ESP_RETURN_ON_FALSE(position != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid position pointer");

    float current_angle = 0;
    ESP_ERROR_CHECK_WITHOUT_ABORT(as5600_read_angle(ctrl_handle->config.position_sensor, &current_angle));

    const bool flip = ctrl_handle->flip_sensor_direction;
    *position = motion_ctrl_utils_norm_180(current_angle - ctrl_handle->origin_position) * (flip ? -1.0f : 1.0f);
    return ESP_OK;
}

esp_err_t motion_ctrl_move(motion_ctrl_handle_t *ctrl_handle, motion_ctrl_move_t *move)
{
    ESP_RETURN_ON_FALSE(ctrl_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid motion controller handle");
    ESP_RETURN_ON_FALSE(move != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid move pointer");

    if (atomic_load(&ctrl_handle->state) == MOTION_CTRL_STATE_ERROR)
    {
        ESP_LOGE(TAG, "Motion controller is in error state. Cannot move.");
        return ESP_ERR_INVALID_STATE;
    }

    motion_ctrl_utils_clamp(&move->end_position, ctrl_handle->position_range * -1.0f, ctrl_handle->position_range);
    const BaseType_t result = xQueueSend(ctrl_handle->move_queue, move, pdMS_TO_TICKS(MOTION_CTRL_MOVE_QUEUE_WAIT_MS));
    if (result != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to enqueue move command");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t motion_ctrl_watchdog_reset(motion_ctrl_handle_t *ctrl_handle)
{
    ESP_RETURN_ON_FALSE(ctrl_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid motion controller handle");
    atomic_store(&ctrl_handle->state, MOTION_CTRL_STATE_IDLE);
    return ESP_OK;
}

esp_err_t motion_ctrl_dump_log(motion_ctrl_handle_t *handle, motion_ctrl_log_sample_t *samples, int32_t sample_count, size_t *actual_sample_count)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid motion controller handle");
    ESP_RETURN_ON_FALSE(samples != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid samples buffer");
    ESP_RETURN_ON_FALSE(sample_count >= -1, ESP_ERR_INVALID_ARG, TAG, "Invalid sample count");
    ESP_RETURN_ON_FALSE(actual_sample_count != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid actual sample count pointer");

    /* BEWARE, YE FOOLISH MORTALS: TO PEEK UPON THE MOTION ENGINE WHILE IT STILL THRASHES IS TO INVOKE THE ANCIENT CURSE OF MALFORMED DATA! */
    if (atomic_load(&handle->state) != MOTION_CTRL_STATE_IDLE)
    {
        ESP_LOGW(TAG, "Motion controller is not idle. Dumping log may be inaccurate.");
    }

    /* Samples *stored* in the buffer (not free space) */
    UBaseType_t items_waiting = 0;
    vRingbufferGetInfo(handle->log_buffer, NULL, NULL, NULL, NULL, &items_waiting);
 
    const size_t available_samples = (size_t)items_waiting;
    *actual_sample_count = (sample_count == -1 || (size_t)sample_count > available_samples) ? available_samples : (size_t)sample_count;
 
    for (size_t i = 0; i < *actual_sample_count; ++i)
    {
        motion_ctrl_log_sample_t *sample = (motion_ctrl_log_sample_t *)xRingbufferReceive(handle->log_buffer, NULL, 0);
        if (sample == NULL)
        {
            ESP_LOGW(TAG, "Failed to receive log sample no. %zu from buffer", i);
            break;
        }
 
        samples[i] = *sample;
        vRingbufferReturnItem(handle->log_buffer, (void *)sample);
    }

    return ESP_OK;
}

static void motion_ctrl_update_range(motion_ctrl_handle_t *ctrl_handle)
{
    const motion_ctrl_limits_t *limits = &ctrl_handle->config.limits;

    const float lower_limit = fminf(limits->max_position, limits->min_position);
    const float upper_limit = fmaxf(limits->max_position, limits->min_position);
    const bool is_interior = (limits->mid_position > lower_limit && limits->mid_position < upper_limit);

    float span = 0;
    if (is_interior)
    {
        span = upper_limit - lower_limit;
        if (limits->min_position > limits->max_position)
        {
            ctrl_handle->flip_sensor_direction = true;
        }
    }
    else
    {
        span = 360.0f - (upper_limit - lower_limit); /* BOO! Spooky magic number! (go figure...) */
        if (limits->max_position != lower_limit)
        {
            ctrl_handle->flip_sensor_direction = true;
        }
    }

    ctrl_handle->position_range = span * 0.5f;
    ctrl_handle->origin_position = limits->mid_position;
}

static bool motion_ctrl_same_moves(const motion_ctrl_move_t *move1, const motion_ctrl_move_t *move2)
{
    if (move1 == NULL || move2 == NULL)
    {
        return false;
    }

    return (move1->end_position == move2->end_position) &&
           (move1->step_frequency_hz == move2->step_frequency_hz) &&
           (move1->block_until_complete == move2->block_until_complete);
}

static void motion_ctrl_log_push(motion_ctrl_handle_t *ctrl_handle, const motion_ctrl_log_sample_t *sample)
{
    void *slot = NULL;
 
    while (xRingbufferSendAcquire(ctrl_handle->log_buffer, &slot, sizeof(motion_ctrl_log_sample_t), 0) != pdTRUE)
    {
        size_t dummy_size = 0;
        void *oldest_sample = xRingbufferReceive(ctrl_handle->log_buffer, &dummy_size, 0);
        if (oldest_sample == NULL)
        {
            ESP_LOGW(TAG, "Failed to acquire space in log buffer, dropping sample");
            return;
        }
        vRingbufferReturnItem(ctrl_handle->log_buffer, oldest_sample);
    }
 
    memcpy(slot, sample, sizeof(motion_ctrl_log_sample_t));
    if (xRingbufferSendComplete(ctrl_handle->log_buffer, slot) != pdTRUE)
    {
        ESP_LOGW(TAG, "Failed to commit log sample to buffer");
    }
}

static int32_t motion_ctrl_compute_command(const motion_ctrl_move_t *move, float position, float position_error)
{
    /* Placeholder: Trapezoidal frequency control around target, floored at the pull-in rate */
    const uint32_t cruise_hz = (move->step_frequency_hz != 0) ? move->step_frequency_hz : MOTION_CTRL_ADAPTIVE_MAX_FREQUENCY_HZ;
    float taper = fabsf(position_error) / MOTION_CTRL_SLOWDOWN_DISTANCE_DEG;
    if (taper > 1.0f)
    {
        taper = 1.0f;
    }
 
    uint32_t magnitude_hz = (uint32_t)(taper * (float)cruise_hz);
    if (magnitude_hz < STEPPER_DRV_MIN_STEP_FREQUENCY_HZ)
    {
        magnitude_hz = (cruise_hz < STEPPER_DRV_MIN_STEP_FREQUENCY_HZ) ? cruise_hz : STEPPER_DRV_MIN_STEP_FREQUENCY_HZ;
    }
 
    return (position_error > 0.0f) ? (int32_t)magnitude_hz : -(int32_t)magnitude_hz;
}
 
static void motion_ctrl_apply_command(motion_ctrl_handle_t *ctrl_handle, int32_t command_hz)
{
    stepper_drv_handle_t *stepper = ctrl_handle->config.stepper;
 
    if (command_hz == 0)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_raw_step_stop(stepper));
        return;
    }
 
    /* Logic inversion because the stepper driver expects a different direction (either my messy wiring or flipped dir pin) */
    const stepper_drv_direction_t direction = (command_hz > 0) ? STEPPER_DRV_DIRECTION_REV : STEPPER_DRV_DIRECTION_FWD;
    const uint32_t magnitude_hz = (uint32_t)abs(command_hz);
 
    if (!stepper_drv_is_enabled(stepper))
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_enable(stepper));
    }
 
    ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_raw_step_set_frequency(stepper, magnitude_hz));
 
    /*
     * ESP_ERR_INVALID_STATE means the command is opposite to the current motion: the driver is decelerating
     * to a stop and we retry on the next iteration (no instant reversal).
     */
    const esp_err_t start_result = stepper_drv_raw_step_start(stepper, direction);
    if (start_result != ESP_OK && start_result != ESP_ERR_INVALID_STATE)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(start_result);
    }
}

static void motion_ctrl_loop_task(void *arg)
{
    motion_ctrl_handle_t *ctrl_handle = (motion_ctrl_handle_t *)arg;
    if (ctrl_handle == NULL)
    {
        ESP_LOGE(TAG, "Motion controller loop task received NULL handle");
        vTaskDelete(NULL);
        return;
    }

    const TickType_t period_ticks = pdMS_TO_TICKS(MOTION_CTRL_LOOP_TASK_INTERVAL_MS);
    TickType_t move_wait_ticks = portMAX_DELAY;
    TickType_t last_wake = xTaskGetTickCount();

    motion_ctrl_move_t next_move = {0};
    motion_ctrl_move_t move = {.end_position = 0.0f, .step_frequency_hz = 0, .block_until_complete = false};

    float current_position = 0;
    float position_error = 0;

    while (true)
    {
        if (!atomic_load(&ctrl_handle->limits_set))
        {
            vTaskDelay(period_ticks);
            continue;
        }   

        /* Computed every iteration so limits that get configured after task start are honoured */
        const float min_position = ctrl_handle->position_range * -1.0f;
        const float max_position = ctrl_handle->position_range;
        
        /* --------- Queue management & Blocking -------- */
        
        /* 
         * When idle, the task blocks indefinitely until a new move is received.
         * When moving, the queue is polled without blocking and the loop period is set by vTaskDelayUntil below,
         * so the control law runs at a fixed sample time Ts = MOTION_CTRL_LOOP_TASK_INTERVAL_MS.
         */
        const bool was_idle = (move_wait_ticks == portMAX_DELAY);
        if (xQueueReceive(ctrl_handle->move_queue, &next_move, move_wait_ticks) == pdTRUE)
        {
            /* Only overwrite current move if it's not a blocking command or if current target is reached */
            if (!move.block_until_complete || atomic_load(&ctrl_handle->state) == MOTION_CTRL_STATE_IDLE)
            {
                if (!motion_ctrl_same_moves(&next_move, &move))
                {
                    move = next_move;
                }
            }
        }
        if (was_idle)
        {
            last_wake = xTaskGetTickCount();
        }
        
        /* ------------ Valid position check ------------ */

        ESP_ERROR_CHECK_WITHOUT_ABORT(motion_ctrl_get_position(ctrl_handle, &current_position));
        if (current_position < min_position || current_position > max_position)
        {
            ESP_LOGE(TAG, "Motion controller position out of bounds: %f (min: %f, max: %f)", current_position, min_position, max_position);

            if (stepper_drv_is_enabled(ctrl_handle->config.stepper))
            {
                ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_disable(ctrl_handle->config.stepper));
                atomic_store(&ctrl_handle->state, MOTION_CTRL_STATE_ERROR);
            }
        
            /* Wait for the user to resolve the issue i.e. manually move the controller back into the valid range */
            vTaskDelay(pdMS_TO_TICKS(MOTION_CTRL_LOOP_TASK_ERROR_RESOLVE_MS));
            last_wake = xTaskGetTickCount();
            continue;
        }

        /* --------------- Move execution --------------- */

        position_error = move.end_position - current_position;
        if (fabsf(position_error) <= ctrl_handle->config.position_threshold)
        {
            /* Target reached -> ramp down, and set task to sleep on queue */    
            move_wait_ticks = portMAX_DELAY; 
            move.block_until_complete = false;
            atomic_store(&ctrl_handle->state, MOTION_CTRL_STATE_IDLE);
            ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_raw_step_stop(ctrl_handle->config.stepper));
        }
        else
        {
            /* Motion active -> non-blocking queue checks */
            move_wait_ticks = 0;
            atomic_store(&ctrl_handle->state, MOTION_CTRL_STATE_MOVING);

            const int32_t command_hz = motion_ctrl_compute_command(&move, current_position, position_error);
            motion_ctrl_apply_command(ctrl_handle, command_hz);

           /* ------------------ Logging ------------------ */
 
            motion_ctrl_log_sample_t sample = {0};
            sample.timestamp_us = esp_timer_get_time();
            ESP_ERROR_CHECK_WITHOUT_ABORT(ina226_read_current(ctrl_handle->config.power_sensor, &sample.current));
            ESP_ERROR_CHECK_WITHOUT_ABORT(ina226_read_bus_voltage(ctrl_handle->config.power_sensor, &sample.voltage));
            ESP_ERROR_CHECK_WITHOUT_ABORT(as5600_read_angle(ctrl_handle->config.position_sensor, &sample.angle));
            sample.position = current_position;
            sample.target_position = move.end_position;
            ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_get_position(ctrl_handle->config.stepper, &sample.step_position));
            sample.commanded_hz = command_hz;
 
            uint32_t applied_hz = 0;
            stepper_drv_direction_t applied_direction = STEPPER_DRV_DIRECTION_FWD;
            ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_raw_step_get_frequency(ctrl_handle->config.stepper, &applied_hz, &applied_direction));
            
            /* Same sign flippage as in the command */
            sample.applied_hz = (applied_direction == STEPPER_DRV_DIRECTION_REV) ? (int32_t)applied_hz : -(int32_t)applied_hz;
 
            motion_ctrl_log_push(ctrl_handle, &sample);
 
            vTaskDelayUntil(&last_wake, period_ticks);
        }
    }
}

static void motor_ctrl_timeout_callback(void *arg)
{
    motion_ctrl_handle_t *ctrl_handle = (motion_ctrl_handle_t *)arg;
    if (ctrl_handle == NULL)
    {
        ESP_LOGE(TAG, "Motion controller timeout callback received NULL handle");
        return;
    }

    if (atomic_load(&ctrl_handle->state) == MOTION_CTRL_STATE_IDLE &&
        stepper_drv_is_enabled(ctrl_handle->config.stepper) &&
        !stepper_drv_is_moving(ctrl_handle->config.stepper)) /* Don't cut power mid-deceleration, shouldn't happen but hey */
    {
        ESP_LOGW(TAG, "Motion controller timeout: disabling stepper driver");
        ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_disable(ctrl_handle->config.stepper));    
    }
}

static void motor_ctrl_watchdog_task(void *arg)
{
    motion_ctrl_handle_t *ctrl_handle = (motion_ctrl_handle_t *)arg;
    if (ctrl_handle == NULL)
    {
        ESP_LOGE(TAG, "Motion controller watchdog task received NULL handle");
        vTaskDelete(NULL);
        return;
    }

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(MOTION_CTRL_WATCHDOG_TASK_INTERVAL_MS));

        if (!atomic_load(&ctrl_handle->limits_set))
        {
            continue;
        }

        float current_position = 0;
        ESP_ERROR_CHECK_WITHOUT_ABORT(motion_ctrl_get_position(ctrl_handle, &current_position));

        const float min_position = ctrl_handle->position_range * -1.0f;
        const float max_position = ctrl_handle->position_range;

        const bool stepper_enabled = stepper_drv_is_enabled(ctrl_handle->config.stepper);
        if (current_position < min_position || current_position > max_position)
        {
            ESP_LOGE(TAG, "Watchdog: Position out of bounds: %d [mdeg] (min: %d, max: %d)",
                     (int)(current_position * 1000), (int)(min_position * 1000), (int)(max_position * 1000));

            if (stepper_drv_is_enabled(ctrl_handle->config.stepper))
            {
                ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_disable(ctrl_handle->config.stepper));
                atomic_store(&ctrl_handle->state, MOTION_CTRL_STATE_ERROR);
            }
        }
    }
}