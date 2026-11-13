#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"

#include "motion_controller.h"
#include "motion_controller_utils.h"
#include "as5600.h"
#include "ina226.h"
#include "stepper_driver.h"

static const char *TAG = "motion_controller";

#define MOTION_CTRL_FULL_TURN_DEG   (360.0f)

/*
 * Positive command <-> stepper direction mapping.
 * Inverted because the stepper driver expects a different direction (either my messy wiring or flipped dir pin).
 * This is the only place where that inversion lives, flip it here if the wiring ever changes.
 */
#define MOTION_CTRL_DIRECTION_POSITIVE  (STEPPER_DRV_DIRECTION_REV)
#define MOTION_CTRL_DIRECTION_NEGATIVE  (STEPPER_DRV_DIRECTION_FWD)

/* The ESO matrices are discretized for a fixed sample time, so the RTOS tick must not round the loop period */
_Static_assert((pdMS_TO_TICKS(MOTION_CTRL_LOOP_TASK_INTERVAL_MS) * portTICK_PERIOD_MS) == MOTION_CTRL_LOOP_TASK_INTERVAL_MS,
               "Loop interval must be a multiple of the RTOS tick period");

/*
 * State of the reference generator. This is the signal the ADRC controller tracks.
 * It lives in the loop task and is deliberately independent of the measured position.
 */
typedef struct motion_ctrl_reference
{
    float position;     /* r,  deg */
    float velocity;     /* dr, deg/s */
} motion_ctrl_reference_t;

/* -------------- Helper functions -------------- */

/*
 * Converts a raw sensor angle into the controller-frame position (origin and direction applied).
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param angle Raw sensor angle in degrees.
 * @return Controller-frame position in degrees, in [-180, 180).
 */
static float motion_ctrl_angle_to_position(const motion_ctrl_handle_t *ctrl_handle, const float angle);

/*
 * Reads the stepper's current frequency as a signed value in the controller's sign convention
 * (positive = towards positive position).
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param applied_hz Pointer to the variable that will hold the signed frequency in Hz.
 * @return ESP_OK if successful, otherwise an error code.
 */
static esp_err_t motion_ctrl_get_applied_hz(const motion_ctrl_handle_t *ctrl_handle, int32_t *applied_hz);

/*
 * Moves the controller into the error state and disables the stepper.
 * Safe to call from any task and any number of times.
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @return true if this call caused the transition (so the caller should log the details), false if it was already in error.
 */
static bool motion_ctrl_flag_error(motion_ctrl_handle_t *ctrl_handle);

/*
 * Checks the limits, position threshold and step scale of the configuration.
 *
 * @param config Pointer to the motion controller configuration.
 * @return true if valid, otherwise false (a warning is logged).
 */
static bool motion_ctrl_positioning_valid(const motion_ctrl_config_t *config);

/*
 * Checks the parameters needed for reference generation.
 *
 * @param config Pointer to the motion controller configuration.
 * @return true if valid, otherwise false (a warning is logged).
 */
static bool motion_ctrl_feedforward_valid(const motion_ctrl_config_t *config);

/*
 * Checks the ADRC / ESO parameters.
 *
 * @param config Pointer to the motion controller configuration.
 * @return true if valid, otherwise false (a warning is logged).
 */
static bool motion_ctrl_adrc_valid(const motion_ctrl_config_t *config);

/*
 * Update the motion controller's origin position and range based on the current limits.
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 */
static void motion_ctrl_update_range(motion_ctrl_handle_t *ctrl_handle);

/*
 * Update motion controller's limits and values for feedforward trajectory generation.
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 */
static void motion_ctrl_update_feedforward(motion_ctrl_handle_t *ctrl_handle);

/* ------------------- Logging ------------------ */

/*
 * Push a sample into the log ring buffer, evicting the oldest sample if the buffer is full.
 * Called from the log task only.
 * 
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param sample Pointer to the sample to be pushed into the log buffer.
 */
static void motion_ctrl_log_push(motion_ctrl_handle_t *ctrl_handle, const motion_ctrl_log_sample_t *sample);

/*
 * Motion controller log task.
 * Waits for a sample from the control loop, adds the power sensor readings and pushes it into the log buffer.
 * Runs at a low priority so the I2C transactions never get in the way of the control loop.
 *
 * @param arg Pointer to the motion controller handle.
 */
static void motion_ctrl_log_task(void *arg);

/* ---------------- Control law ----------------- */

/*
 * Resets the ADRC ESO observer states.
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param position The position to reset the observer state to, in degrees.
 * @param velocity The velocity to reset the observer state to, in deg/s.
 */
static void motion_ctrl_observer_reset(motion_ctrl_handle_t *ctrl_handle, const float position, const float velocity);

/*
 * ADRC controller implementation.
 * Computes the signed command to be sent to the stepper driver based on the current position, the reference, and the controller parameters.
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param reference Pointer to the current reference (r, dr).
 * @param position Current controller-frame position in degrees.
 * @param prev_hz Previous applied frequency in Hz (the ESO input).
 * @return Signed command in Hz to be sent to the stepper driver.
 */
static int32_t motion_ctrl_compute_command(motion_ctrl_handle_t *ctrl_handle, const motion_ctrl_reference_t *reference, const float position, const int32_t prev_hz);

/*
 * Starts a move from idle.
 * Seeds the reference generator and the observer with the position and the velocity the stepper is
 * *actually* running at, so a move that starts while the stepper is still ramping down is continuous too.
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param reference Pointer to the reference to be seeded.
 * @param position Current controller-frame position in degrees.
 * @param applied_hz Pointer to a variable that will hold the signed applied frequency at the start of the move.
 * @return true if the controller entered the moving state, false if it was no longer idle.
 */
static bool motion_ctrl_begin_move(motion_ctrl_handle_t *ctrl_handle, motion_ctrl_reference_t *reference, const float position, int32_t *applied_hz);

/*
 * Advances the reference generator by one loop period (online, state based trajectory generation).
 *
 * Every step the generator picks the highest speed towards the target from which it can still brake to rest on the target
 * with a_max (discrete-time exact, so it lands on the target without overshooting), and slews its velocity towards that speed at a_max. This gives the usual trapezoid / triangle profile from rest,
 * but because it only depends on its current (position, velocity) it also works when the target changes mid-move:
 * it keeps its velocity, decelerates / reverses at a_max as needed and carries on. 
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param reference Pointer to the reference to be advanced.
 * @param target The target position in degrees.
 * @return true if the reference is at rest on the target, otherwise false.
 */
static bool motion_ctrl_reference_step(const motion_ctrl_handle_t *ctrl_handle, motion_ctrl_reference_t *reference, const float target);

/*
 * Turns the signed command into stepper driver calls (enable, frequency, direction, safe reversal, stop on zero).
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param command_hz The signed command in Hz.
 */
static void motion_ctrl_apply_command(motion_ctrl_handle_t *ctrl_handle, const int32_t command_hz);

/* -------------- Main loop / init -------------- */

/*
 * Motion controller main loop task.
 * This task will continuously monitor the motion controller's state and perform the necessary actions.
 *
 * @param arg Pointer to the motion controller handle.
 */
static void motion_ctrl_loop_task(void *arg);

/*
 * Difference between the stepper's step counter and the step count the position sensor says it should have.
 * It stays constant as long as no steps are missed.
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param position Current controller-frame position in degrees.
 * @return The discrepancy in steps.
 */
static int32_t motion_ctrl_steps_discrepancy(const motion_ctrl_handle_t *ctrl_handle, const float position);

/*
 * Motion controller watchdog task.
 * This task will continuously monitor the motion controller's state.
 * If the motor is overloaded or the position is invalid, the controller will disarm the motor
 * and set the controller state to MOTION_CTRL_STATE_ERROR until reset.
 *
 * @param arg Pointer to the motion controller handle.
 */
static void motion_ctrl_watchdog_task(void *arg);

esp_err_t motion_ctrl_init(const motion_ctrl_config_t *config, motion_ctrl_handle_t *ctrl_handle)
{
    ESP_LOGI(TAG, "Initializing motion controller");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid motion controller configuration");
    ESP_RETURN_ON_FALSE(ctrl_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid motion controller handle");
    ESP_RETURN_ON_FALSE(config->stepper != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid stepper driver handle");
    ESP_RETURN_ON_FALSE(config->position_sensor != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid position sensor handle");
    ESP_RETURN_ON_FALSE(config->power_sensor != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid power sensor handle");

    /* The handle may come from the stack or the heap, so don't trust whatever is in it */
    memset(ctrl_handle, 0, sizeof(*ctrl_handle));
    ctrl_handle->config = *config;
    ctrl_handle->flip_sensor_direction = false;
    ctrl_handle->position_range = 0.0f;
    ctrl_handle->origin_position = 0.0f;

    atomic_init(&ctrl_handle->state, MOTION_CTRL_STATE_IDLE);
    atomic_init(&ctrl_handle->stop_requested, false);
    atomic_init(&ctrl_handle->move_requested, false);
    atomic_init(&ctrl_handle->configured, false);
    atomic_init(&ctrl_handle->target_position, 0.0f);
    atomic_init(&ctrl_handle->current_position, 0.0f);
    atomic_init(&ctrl_handle->current_velocity, 0.0f);

    const bool positioning_set = motion_ctrl_positioning_valid(config);
    const bool feedforward_set = motion_ctrl_feedforward_valid(config);
    const bool adrc_eso_set = motion_ctrl_adrc_valid(config);

    if (positioning_set)
    {
        motion_ctrl_update_range(ctrl_handle);
    }
    if (feedforward_set)
    {
        motion_ctrl_update_feedforward(ctrl_handle);
    }

    if (!positioning_set || !feedforward_set || !adrc_eso_set)
    {
        ESP_LOGW(TAG, "Motion controller is not fully configured. Loop will not start.");
        atomic_store(&ctrl_handle->configured, false);
    }
    else
    {
        atomic_store(&ctrl_handle->configured, true);
    }

    motion_ctrl_observer_reset(ctrl_handle, 0.0f, 0.0f);

    ESP_LOGI(TAG, "Creating motion controller log buffer");
    ctrl_handle->log_buffer = xRingbufferCreateNoSplit(sizeof(motion_ctrl_log_sample_t), MOTION_CTRL_LOG_SAMPLE_COUNT);
    ESP_RETURN_ON_FALSE(ctrl_handle->log_buffer != NULL, ESP_ERR_NO_MEM, TAG, "Failed to create motion controller log buffer");

    ctrl_handle->log_queue = xQueueCreate(1, sizeof(motion_ctrl_log_sample_t));
    ESP_RETURN_ON_FALSE(ctrl_handle->log_queue != NULL, ESP_ERR_NO_MEM, TAG, "Failed to create motion controller log queue");

    ESP_LOGI(TAG, "Creating motion controller log task");
    BaseType_t result = xTaskCreate(motion_ctrl_log_task, "motion_ctrl_log", MOTION_CTRL_LOG_TASK_STACK_SIZE,
                                ctrl_handle, MOTION_CTRL_LOG_TASK_PRIORITY, NULL);
    ESP_RETURN_ON_FALSE(result == pdPASS, ESP_ERR_NO_MEM, TAG, "Failed to create motion controller log task");

    ESP_LOGI(TAG, "Creating motion controller loop task");
    result = xTaskCreate(motion_ctrl_loop_task, "motion_ctrl_loop", MOTION_CTRL_LOOP_TASK_STACK_SIZE,
                                ctrl_handle, MOTION_CTRL_LOOP_TASK_PRIORITY, NULL);
    ESP_RETURN_ON_FALSE(result == pdPASS, ESP_ERR_NO_MEM, TAG, "Failed to create motion controller loop task");

    ESP_LOGI(TAG, "Creating motion controller watchdog task");
    result = xTaskCreate(motion_ctrl_watchdog_task, "motion_ctrl_watchdog", MOTION_CTRL_WATCHDOG_TASK_STACK_SIZE,
                                ctrl_handle, MOTION_CTRL_WATCHDOG_TASK_PRIORITY, NULL);
    ESP_RETURN_ON_FALSE(result == pdPASS, ESP_ERR_NO_MEM, TAG, "Failed to create motion controller watchdog task");

    ESP_LOGI(TAG, "Motion controller initialized successfully");
    return ESP_OK;
}

esp_err_t motion_ctrl_get_position(motion_ctrl_handle_t *ctrl_handle, float *position)
{
    ESP_RETURN_ON_FALSE(ctrl_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid motion controller handle");
    ESP_RETURN_ON_FALSE(position != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid position pointer");

    float current_angle = 0.0f;
    ESP_RETURN_ON_ERROR(as5600_read_angle(ctrl_handle->config.position_sensor, &current_angle), TAG, "Failed to read position sensor");

    *position = motion_ctrl_angle_to_position(ctrl_handle, current_angle);
    return ESP_OK;
}

esp_err_t motion_ctrl_get_state(motion_ctrl_handle_t *ctrl_handle, motion_ctrl_state_t *state)
{
    ESP_RETURN_ON_FALSE(ctrl_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid motion controller handle");
    ESP_RETURN_ON_FALSE(state != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid state pointer");

    *state = atomic_load(&ctrl_handle->state);
    return ESP_OK;
}

esp_err_t motion_ctrl_move(motion_ctrl_handle_t *ctrl_handle, float position)
{
    ESP_RETURN_ON_FALSE(ctrl_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid motion controller handle");
    ESP_RETURN_ON_FALSE(isfinite(position), ESP_ERR_INVALID_ARG, TAG, "Invalid target position");
    ESP_RETURN_ON_FALSE(atomic_load(&ctrl_handle->configured), ESP_ERR_INVALID_STATE, TAG, "Controller not configured");

    const motion_ctrl_state_t state = atomic_load(&ctrl_handle->state);
    ESP_RETURN_ON_FALSE(state != MOTION_CTRL_STATE_ERROR, ESP_ERR_INVALID_STATE, TAG, "Controller in error state");
    ESP_RETURN_ON_FALSE(state != MOTION_CTRL_STATE_CALIBRATING, ESP_ERR_INVALID_STATE, TAG, "Controller is calibrating");

    /*
     * Target first, request second: the loop consumes the request flag and only then reads the target,
     * so it can never see the request without the new target. The loop owns the IDLE -> MOVING transition,
     * which is what keeps a move requested right as the previous one finishes from getting lost.
     * If already moving, the new target is simply picked up on the next iteration (retargeting).
     */
    motion_ctrl_utils_clamp_f(&position, -ctrl_handle->position_range, ctrl_handle->position_range);
    atomic_store(&ctrl_handle->target_position, position);
    atomic_store(&ctrl_handle->move_requested, true);
    return ESP_OK;
}

esp_err_t motion_ctrl_stop(motion_ctrl_handle_t *ctrl_handle)
{
    ESP_RETURN_ON_FALSE(ctrl_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid motion controller handle");

    /* Just request a stop */
    atomic_store(&ctrl_handle->stop_requested, true);

    return ESP_OK;
}

esp_err_t motion_ctrl_watchdog_reset(motion_ctrl_handle_t *ctrl_handle)
{
    ESP_RETURN_ON_FALSE(ctrl_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid motion controller handle");

    /* Whatever was requested before the error is stale by now */
    atomic_store(&ctrl_handle->stop_requested, false);
    atomic_store(&ctrl_handle->move_requested, false);

    /* Only an error gets cleared, resetting must never interrupt a move in progress */
    motion_ctrl_state_t expected = MOTION_CTRL_STATE_ERROR;
    atomic_compare_exchange_strong(&ctrl_handle->state, &expected, MOTION_CTRL_STATE_IDLE);
    return ESP_OK;
}

esp_err_t motion_ctrl_dump_log(motion_ctrl_handle_t *handle, motion_ctrl_log_sample_t *samples, uint32_t sample_count, size_t *actual_sample_count)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid motion controller handle");
    ESP_RETURN_ON_FALSE(samples != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid samples buffer");
    ESP_RETURN_ON_FALSE(sample_count != 0, ESP_ERR_INVALID_ARG, TAG, "Invalid sample count: %u", (unsigned)sample_count);
    ESP_RETURN_ON_FALSE(actual_sample_count != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid actual sample count pointer");

    /* BEWARE, YE FOOLISH MORTALS: TO PEEK UPON THE MOTION ENGINE WHILE IT STILL THRASHES IS TO INVOKE THE ANCIENT CURSE OF MALFORMED DATA! */
    if (atomic_load(&handle->state) != MOTION_CTRL_STATE_IDLE)
    {
        ESP_LOGW(TAG, "Motion controller is not idle. Dumping log may be inaccurate.");
    }
 
    size_t available_samples = 0;
    ESP_ERROR_CHECK_WITHOUT_ABORT(motion_ctrl_dump_log_count(handle, &available_samples));
    const size_t samples_to_dump = ((size_t)sample_count > available_samples) ? available_samples : (size_t)sample_count;
 
    size_t dumped = 0;
    for (size_t i = 0; i < samples_to_dump; ++i)
    {
        size_t sample_size = 0;
        motion_ctrl_log_sample_t *sample = (motion_ctrl_log_sample_t *)xRingbufferReceive(handle->log_buffer, &sample_size, 0);
        if (sample == NULL || sample_size != sizeof(motion_ctrl_log_sample_t))
        {
            ESP_LOGW(TAG, "Failed to receive log sample no. %zu from buffer", i);
            if (sample != NULL)
            {
                vRingbufferReturnItem(handle->log_buffer, (void *)sample);
            }
            break;
        }
 
        samples[i] = *sample;
        vRingbufferReturnItem(handle->log_buffer, (void *)sample);
        dumped++;
    }

    *actual_sample_count = dumped;
    return ESP_OK;
}

esp_err_t motion_ctrl_dump_log_count(motion_ctrl_handle_t *handle, size_t* sample_count)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid motion controller handle");
    ESP_RETURN_ON_FALSE(sample_count != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid sample count pointer");

    /* Samples *stored* in the buffer (not free space) */
    UBaseType_t items_waiting = 0;
    vRingbufferGetInfo(handle->log_buffer, NULL, NULL, NULL, NULL, &items_waiting);
 
    *sample_count = (size_t)items_waiting;

    return ESP_OK;
}

/* -------------- Helper functions -------------- */

static float motion_ctrl_angle_to_position(const motion_ctrl_handle_t *ctrl_handle, const float angle)
{
    const float direction = ctrl_handle->flip_sensor_direction ? -1.0f : 1.0f;
    return motion_ctrl_utils_norm_180(angle - ctrl_handle->origin_position) * direction;
}

static esp_err_t motion_ctrl_get_applied_hz(const motion_ctrl_handle_t *ctrl_handle, int32_t *applied_hz)
{
    uint32_t magnitude_hz = 0;
    stepper_drv_direction_t direction = STEPPER_DRV_DIRECTION_FWD;
    ESP_RETURN_ON_ERROR(stepper_drv_raw_step_get_frequency(ctrl_handle->config.stepper, &magnitude_hz, &direction), TAG, "Failed to read stepper frequency");

    *applied_hz = (direction == MOTION_CTRL_DIRECTION_POSITIVE) ? (int32_t)magnitude_hz : -(int32_t)magnitude_hz;
    return ESP_OK;
}

static bool motion_ctrl_flag_error(motion_ctrl_handle_t *ctrl_handle)
{
    /* Atomically flag the error; only the first detection reports and disables */
    const motion_ctrl_state_t previous = atomic_exchange(&ctrl_handle->state, MOTION_CTRL_STATE_ERROR);
    if (previous == MOTION_CTRL_STATE_ERROR)
    {
        return false;
    }

    if (stepper_drv_is_enabled(ctrl_handle->config.stepper))
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_disable(ctrl_handle->config.stepper));
    }
    return true;
}

static bool motion_ctrl_positioning_valid(const motion_ctrl_config_t *config)
{
    /* A limit of exactly 0 deg is perfectly legal, so "not set" means "no span at all" */
    if (config->limits.max_position == config->limits.min_position)
    {
        ESP_LOGW(TAG, "Motion controller limits are not set. Please configure the limits.");
        return false;
    }
    if (!(config->position_threshold > 0.0f))
    {
        ESP_LOGW(TAG, "Motion controller position threshold must be positive.");
        return false;
    }
    if (config->step_scale == 0.0f)
    {
        ESP_LOGW(TAG, "Motion controller step scale is not set.");
        return false;
    }
    return true;
}

static bool motion_ctrl_feedforward_valid(const motion_ctrl_config_t *config)
{
    if (config->stepper->acceleration_hz_per_s == 0 || !(config->Kv > 0.0f) || !(config->Tau > 0.0f))
    {
        ESP_LOGW(TAG, "Motion controller feedforward parameters (stepper acceleration, Kv, Tau) are not set.");
        return false;
    }
    return true;
}

static bool motion_ctrl_adrc_valid(const motion_ctrl_config_t *config)
{
    bool any_input_gain = false;
    for (size_t i = 0; i < 3; ++i)
    {
        if (config->A_eso[i][0] == 0.0f && config->A_eso[i][1] == 0.0f && config->A_eso[i][2] == 0.0f)
        {
            ESP_LOGW(TAG, "Motion controller ESO matrix A has an empty row.");
            return false;
        }
        if (config->L[i] == 0.0f)
        {
            ESP_LOGW(TAG, "Motion controller ESO gain L[%zu] is not set.", i);
            return false;
        }
        any_input_gain |= (config->B_eso[i] != 0.0f);
    }

    /* The disturbance row of B is commonly zero (the disturbance is not driven by the input), so only require *some* input gain. K[2] may be zero too (no disturbance rejection) */
    if (!any_input_gain || config->K[0] == 0.0f || config->K[1] == 0.0f)
    {
        ESP_LOGW(TAG, "Motion controller ESO input gain B or controller gains K are not set.");
        return false;
    }
    return true;
}

static void motion_ctrl_update_range(motion_ctrl_handle_t *ctrl_handle)
{
    const motion_ctrl_limits_t *limits = &ctrl_handle->config.limits;

    const float lower_limit = fminf(limits->max_position, limits->min_position);
    const float upper_limit = fmaxf(limits->max_position, limits->min_position);
    const bool is_interior = (limits->mid_position > lower_limit && limits->mid_position < upper_limit);

    float span = 0.0f;
    bool flip = false;
    if (is_interior)
    {
        span = upper_limit - lower_limit;
        flip = (limits->min_position > limits->max_position);
    }
    else
    {
        /* The allowed range wraps around 0/360, so it is the part of the circle *outside* the limits */
        span = MOTION_CTRL_FULL_TURN_DEG - (upper_limit - lower_limit);
        flip = (limits->max_position > limits->min_position);
    }

    ctrl_handle->flip_sensor_direction = flip;
    ctrl_handle->position_range = span * 0.5f;
    ctrl_handle->origin_position = limits->mid_position;
}

static void motion_ctrl_update_feedforward(motion_ctrl_handle_t *ctrl_handle)
{
    ctrl_handle->v_max = MOTION_CTRL_VELOCITY_HEADROOM * ctrl_handle->config.Kv * STEPPER_DRV_MAX_STEP_FREQUENCY_HZ;
    
    const float a_actuator_max = ctrl_handle->config.Kv * ctrl_handle->config.stepper->acceleration_hz_per_s;
    const float a_magnitude_max = ctrl_handle->config.Kv / ctrl_handle->config.Tau * STEPPER_DRV_MAX_STEP_FREQUENCY_HZ;
    ctrl_handle->a_max = MOTION_CTRL_ACCEL_HEADROOM * fminf(a_actuator_max, a_magnitude_max);
}

/* ------------------- Logging ------------------ */

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

static void motion_ctrl_log_task(void *arg)
{
    motion_ctrl_handle_t *ctrl_handle = (motion_ctrl_handle_t *)arg;
    if (ctrl_handle == NULL)
    {
        ESP_LOGE(TAG, "Motion controller log task received NULL handle");
        vTaskDelete(NULL);
        return;
    }

    motion_ctrl_log_sample_t sample = {0};

    while (true)
    {
        /* Sleep until the control loop publishes a sample */
        if (xQueueReceive(ctrl_handle->log_queue, &sample, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        /* Local variables because taking the address of a packed struct member is asking for trouble */
        float current = 0.0f;
        float voltage = 0.0f;
        ESP_ERROR_CHECK_WITHOUT_ABORT(ina226_read_current(ctrl_handle->config.power_sensor, &current));
        ESP_ERROR_CHECK_WITHOUT_ABORT(ina226_read_bus_voltage(ctrl_handle->config.power_sensor, &voltage));
        sample.current = current;
        sample.voltage = voltage;

        motion_ctrl_log_push(ctrl_handle, &sample);
    }
}

/* ---------------- Control law ----------------- */

static void motion_ctrl_observer_reset(motion_ctrl_handle_t *ctrl_handle, const float position, const float velocity)
{
    ctrl_handle->x_hat_prev[0] = position;
    ctrl_handle->x_hat_prev[1] = velocity;
    ctrl_handle->x_hat_prev[2] = 0.0f;
}

static int32_t motion_ctrl_compute_command(motion_ctrl_handle_t *ctrl_handle, const motion_ctrl_reference_t *reference, const float position, const int32_t prev_hz)
{
    /* We don't check for NULL here as it should be handled by the caller */

    const float (*A_eso)[3] = ctrl_handle->config.A_eso;
    const float *B_eso = ctrl_handle->config.B_eso;
    const float *L = ctrl_handle->config.L;
    const float *K = ctrl_handle->config.K;

    float *x_hat_prev = ctrl_handle->x_hat_prev;
    float x_hat[3] = {0};

    /* Pray to god we don't have any pointer error here */
    for (size_t i = 0; i < 3; i++)
    {
        x_hat[i] = A_eso[i][0] * x_hat_prev[0] + A_eso[i][1] * x_hat_prev[1] + A_eso[i][2] * x_hat_prev[2] 
                 + B_eso[i] * (float)prev_hz + L[i] * position;
    }
    memcpy(x_hat_prev, x_hat, sizeof(x_hat));

    float u = K[0] * (reference->position - x_hat[0]) + K[1] * (reference->velocity - x_hat[1]) - K[2] * x_hat[2];
    motion_ctrl_utils_clamp_f(&u, -(float)STEPPER_DRV_MAX_STEP_FREQUENCY_HZ, (float)STEPPER_DRV_MAX_STEP_FREQUENCY_HZ);
    return (int32_t)u;
}

static bool motion_ctrl_begin_move(motion_ctrl_handle_t *ctrl_handle, motion_ctrl_reference_t *reference, const float position, int32_t *applied_hz)
{
    /* If reading fails we fall back to starting from rest */
    *applied_hz = 0;
    ESP_ERROR_CHECK_WITHOUT_ABORT(motion_ctrl_get_applied_hz(ctrl_handle, applied_hz));

    const float velocity = ctrl_handle->config.Kv * (float)(*applied_hz);
    reference->position = position;
    reference->velocity = velocity;
    motion_ctrl_observer_reset(ctrl_handle, position, velocity);

    /* Not a plain store: the watchdog may have flagged an error in the meantime and that must win */
    motion_ctrl_state_t expected = MOTION_CTRL_STATE_IDLE;
    return atomic_compare_exchange_strong(&ctrl_handle->state, &expected, MOTION_CTRL_STATE_MOVING);
}

static bool motion_ctrl_reference_step(const motion_ctrl_handle_t *ctrl_handle, motion_ctrl_reference_t *reference, const float target)
{
    const float dt = MOTION_CTRL_LOOP_TASK_DT_S;
    const float a_max = ctrl_handle->a_max;
    const float v_max = ctrl_handle->v_max;
    const float dv_limit = a_max * dt;
    const float error = target - reference->position;

    /*
     * Close enough and slow enough that another step could only overshoot: land on the target and rest.
     */
    if (fabsf(error) <= a_max * dt * dt && fabsf(reference->velocity) <= dv_limit)
    {
        reference->position = target;
        reference->velocity = 0.0f;
        return true;
    }

    /* Work in the frame where the target is ahead (distance > 0), so only one case has to be handled */
    const float direction = (error >= 0.0f) ? 1.0f : -1.0f;
    const float distance = fabsf(error);
    const float velocity = direction * reference->velocity;

    /*
     * Highest next velocity v' from which we can still brake to rest on the target at a_max, *including the distance
     * covered during this step* (trapezoidal integration: (v + v') / 2 * dt). Braking from v' needs v'^2 / (2 a_max), so
     *      v'^2 / (2 a_max) + (v + v') * dt / 2 <= distance
     * Solving for v' gives the line below. Looking at the distance only (v = sqrt(2 a d)) is a step late, which makes
     * the reference overshoot by ~1 deg and ring around the target.
     */
    const float half_dv = 0.5f * dv_limit;
    const float radicand = half_dv * half_dv + 2.0f * a_max * distance - dv_limit * velocity;
    const float v_brake = sqrtf(fmaxf(radicand, 0.0f)) - half_dv;

    /* Slew towards it, no faster than a_max allows (if it is already too late, this is simply full braking) */
    float velocity_next = fminf(v_max, v_brake);
    motion_ctrl_utils_clamp_f(&velocity_next, velocity - dv_limit, velocity + dv_limit);

    reference->position += direction * 0.5f * (velocity + velocity_next) * dt;     /* exact for constant acceleration */
    reference->velocity = direction * velocity_next;
    return false;
}
 
static void motion_ctrl_apply_command(motion_ctrl_handle_t *ctrl_handle, const int32_t command_hz)
{
    stepper_drv_handle_t *stepper = ctrl_handle->config.stepper;
 
    if (command_hz == 0)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_raw_step_stop(stepper));
        return;
    }
 
    const stepper_drv_direction_t direction = (command_hz > 0) ? MOTION_CTRL_DIRECTION_POSITIVE : MOTION_CTRL_DIRECTION_NEGATIVE;
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

/* -------------- Main loop / init -------------- */

static void motion_ctrl_loop_task(void *arg)
{
    motion_ctrl_handle_t *ctrl_handle = (motion_ctrl_handle_t *)arg;
    if (ctrl_handle == NULL)
    {
        ESP_LOGE(TAG, "Motion controller loop task received NULL handle");
        vTaskDelete(NULL);
        return;
    }

    /* Error reporting variables */
    bool error_reported = false;
    uint32_t sensor_failures = 0;

    /* Missed step tracking variables */
    bool discrepancy_baseline_set = false;
    int32_t baseline_discrepancy = 0;
    uint32_t missed_steps_count = 0;  

    /* Loop timing variables */
    uint64_t loop_counter = 0;
    const TickType_t period_ticks = pdMS_TO_TICKS(MOTION_CTRL_LOOP_TASK_INTERVAL_MS);
    TickType_t last_wake = xTaskGetTickCount();

    /* Measurement variables */
    bool have_previous = false;
    int64_t previous_time_us = 0;
    float previous_position = 0.0f;
    float current_position = 0.0f;
    float current_velocity = 0.0f;

    /* Move variables */
    motion_ctrl_reference_t reference = {0};
    int32_t previous_applied_hz = 0;
    float settle_time_s = 0.0f;

    while (true)
    {
        /* Single place where the loop sleeps, so every `continue` below stays on the period */
        vTaskDelayUntil(&last_wake, period_ticks);
        
        /* Loop execution counter with guard against overflow */
        loop_counter++;
        if (loop_counter == UINT64_MAX)
        {
            loop_counter = 0;
        }

        if (!atomic_load(&ctrl_handle->configured))
        {
            continue;
        }

        /* ----------------- State check ---------------- */

        if (atomic_load(&ctrl_handle->state) == MOTION_CTRL_STATE_ERROR)
        {
            if (!error_reported)
            {
                ESP_LOGE(TAG, "Motion controller is in error state. Halting loop.");
                error_reported = true;
            }

            /* Doing that here for redundancy's sake */
            if (stepper_drv_is_enabled(ctrl_handle->config.stepper))
            {
                ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_disable(ctrl_handle->config.stepper));
            }

            /* Nothing to do until somebody resets us, so no need to spin at full rate. Previous samples are stale after this. */
            have_previous = false;
            vTaskDelay(pdMS_TO_TICKS(MOTION_CTRL_LOOP_TASK_ERROR_RESOLVE_MS));
            last_wake = xTaskGetTickCount();
            continue;
        }
        error_reported = false;

        /* --------- Updating internal variables -------- */

        float raw_angle = 0.0f;
        if (as5600_read_angle(ctrl_handle->config.position_sensor, &raw_angle) != ESP_OK)
        {
            sensor_failures++;
            /* Using PRIu32 here cause ESP's log may freakout with %d and ints over 16 */
            ESP_LOGW(TAG, "Position sensor read failed (%" PRIu32 "/%d)", sensor_failures, MOTION_CTRL_SENSOR_FAIL_LIMIT);
            if (sensor_failures >= MOTION_CTRL_SENSOR_FAIL_LIMIT && motion_ctrl_flag_error(ctrl_handle))
            {
                ESP_LOGE(TAG, "Position sensor lost, stepper disabled");
            }

            /* 
             * If successive reads fail, this continue will loop back to read again,
             * so we're free to clear sensor failures after reading.
             */
            continue;
        }
        sensor_failures = 0;

        const int64_t now_us = esp_timer_get_time();
        current_position = motion_ctrl_angle_to_position(ctrl_handle, raw_angle);
        if (have_previous)
        {
            /* Velocity integration */
            const float dt_s = (float)(now_us - previous_time_us) * 1e-6f;
            if (dt_s > 0.0f)
            {
                /* Wrapped difference, so crossing the +-180 seam doesn't look like a 360 deg jump */
                current_velocity = motion_ctrl_utils_norm_180(current_position - previous_position) / dt_s; /* degrees per second */
            }
        }
        previous_position = current_position;
        previous_time_us = now_us;
        have_previous = true;

        atomic_store(&ctrl_handle->current_position, current_position);
        atomic_store(&ctrl_handle->current_velocity, current_velocity);

        /* ------------------ Requests ------------------ */

        if (atomic_exchange(&ctrl_handle->stop_requested, false))
        {
            /* Stop wins over a move requested at the same time */
            atomic_store(&ctrl_handle->move_requested, false);
            ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_raw_step_stop(ctrl_handle->config.stepper));
            motion_ctrl_state_t expected = MOTION_CTRL_STATE_MOVING;
            atomic_compare_exchange_strong(&ctrl_handle->state, &expected, MOTION_CTRL_STATE_IDLE);
            discrepancy_baseline_set = false;
            missed_steps_count = 0;
            continue;
        }

        /*
         * The request flag is consumed *before* the target is read (see motion_ctrl_move), and a request
         * that arrives while moving is already covered because the target is re-read every iteration.
         */
        const bool move_requested = atomic_exchange(&ctrl_handle->move_requested, false);
        if (move_requested && atomic_load(&ctrl_handle->state) == MOTION_CTRL_STATE_IDLE)
        {
            if (motion_ctrl_begin_move(ctrl_handle, &reference, current_position, &previous_applied_hz))
            {
                settle_time_s = 0.0f;
            }
        }

        /* ------------ Missed steps checking ----------- */

        if (atomic_load(&ctrl_handle->state) == MOTION_CTRL_STATE_MOVING)
        {
            const int32_t current_discrepancy = motion_ctrl_steps_discrepancy(ctrl_handle, current_position);
            if (!discrepancy_baseline_set)
            {
                baseline_discrepancy = current_discrepancy;
                discrepancy_baseline_set = true;
            }

            const int32_t allowed = MOTION_CTRL_MISSED_STEPS_REGISTER_THRESHOLD + (int32_t)((float)abs(previous_applied_hz) * MOTION_CTRL_SENSOR_LAG_S);
            if (abs(current_discrepancy - baseline_discrepancy) > allowed)
            {            
                missed_steps_count++;
                
                /* Motor lost synchronization so we're safe to assume the velocity fell to 0 */
                reference.velocity = 0.0f;
                motion_ctrl_observer_reset(ctrl_handle, reference.position, reference.velocity);

                /* Revert stepping rate to 0 so the motor doesn't try stepping at full power when stalled */
                previous_applied_hz = 0; 
                ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_raw_step_force_stop(ctrl_handle->config.stepper)); /* Only time this is allowed */
                
                ESP_LOGW(TAG, "Stepper has lost steps, baseline v current: %d v %d", baseline_discrepancy, current_discrepancy);
                baseline_discrepancy = current_discrepancy;
                
                if (missed_steps_count >= MOTION_CTRL_MISSED_STEPS_COUNT_THRESHOLD && motion_ctrl_flag_error(ctrl_handle))
                {
                    ESP_LOGE(TAG, "Stepper stalled, disabling.");
                }
            }
        }

        if (missed_steps_count != 0 && ((loop_counter * MOTION_CTRL_LOOP_TASK_INTERVAL_MS) % MOTION_CTRL_MISSED_STEPS_DECAY_PERIOD_MS) == 0)
        {
            missed_steps_count--;
        }

        /* --------------- Move execution --------------- */

        /* If we're not moving, we're not moving */
        if (atomic_load(&ctrl_handle->state) != MOTION_CTRL_STATE_MOVING)
        {
            continue;
        }

        const float target_position = atomic_load(&ctrl_handle->target_position);
        const bool reference_done = motion_ctrl_reference_step(ctrl_handle, &reference, target_position);
        const bool within_target_threshold = fabsf(target_position - current_position) <= ctrl_handle->config.position_threshold;

        if (reference_done && within_target_threshold)
        {
            /* Target reached: ramp down and park until the next move */
            ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_raw_step_stop(ctrl_handle->config.stepper));
#if MOTION_CTRL_FINISH_MOVES
            motion_ctrl_state_t expected = MOTION_CTRL_STATE_MOVING;
            atomic_compare_exchange_strong(&ctrl_handle->state, &expected, MOTION_CTRL_STATE_IDLE);
            discrepancy_baseline_set = false;
            missed_steps_count = 0;
#endif
            continue;
        }

        /* The reference has stopped and we still aren't there: start the settle clock, any progress or retarget resets it */
        settle_time_s = (reference_done && !within_target_threshold) ? settle_time_s + MOTION_CTRL_LOOP_TASK_DT_S : 0.0f;
        if (settle_time_s >= MOTION_CTRL_SETTLE_TIMEOUT_S)
        {
            settle_time_s = 0.0f;
#if MOTION_CTRL_SETTLE_TIMEOUT_FATAL
            if (motion_ctrl_flag_error(ctrl_handle))
            {
                ESP_LOGE(TAG, "Failed to settle on target (error: %.3f deg), flagging error", target_position - current_position);
            }
#else
            ESP_LOGW(TAG, "Failed to settle on target (error: %.3f deg), giving up", target_position - current_position);
            ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_raw_step_stop(ctrl_handle->config.stepper));
#if MOTION_CTRL_FINISH_MOVES
            motion_ctrl_state_t expected = MOTION_CTRL_STATE_MOVING;
            atomic_compare_exchange_strong(&ctrl_handle->state, &expected, MOTION_CTRL_STATE_IDLE);
            discrepancy_baseline_set = false;
            missed_steps_count = 0;
#endif
#endif
            continue;
        }

        /* Watchdog may have tripped since the check above; never re-enable the stepper behind its back */
        if (atomic_load(&ctrl_handle->state) != MOTION_CTRL_STATE_MOVING)
        {
            continue;
        }

        const int32_t command_hz = motion_ctrl_compute_command(ctrl_handle, &reference, current_position, previous_applied_hz);
        motion_ctrl_apply_command(ctrl_handle, command_hz);

        /* Saving applied frequency for the next ESO update and the log */
        int32_t applied_hz = 0;
        ESP_ERROR_CHECK_WITHOUT_ABORT(motion_ctrl_get_applied_hz(ctrl_handle, &applied_hz));
        previous_applied_hz = applied_hz;

        /* ------------------ Logging ------------------ */

        /* Only the cheap, control-side values here, the log task adds the power sensor readings */
        int32_t step_position = 0;
        ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_get_position(ctrl_handle->config.stepper, &step_position));

        motion_ctrl_log_sample_t sample = {0};
        sample.timestamp_us = now_us;
        sample.angle = raw_angle;
        sample.position = current_position;
        sample.target_position = target_position;
        sample.step_position = step_position;
        sample.commanded_hz = command_hz;
        sample.applied_hz = applied_hz;
        sample.x_hat1 = ctrl_handle->x_hat_prev[0];
        sample.x_hat2 = ctrl_handle->x_hat_prev[1];
        sample.x_hat3 = ctrl_handle->x_hat_prev[2];
        sample.ref_position = reference.position;
        sample.ref_velocity = reference.velocity;

        /* Never blocks, overwrites the previous sample if the log task hasn't picked it up yet */
        xQueueOverwrite(ctrl_handle->log_queue, &sample);
    }
}

static int32_t motion_ctrl_steps_discrepancy(const motion_ctrl_handle_t *ctrl_handle, const float position)
{
    int32_t step_position = 0;
    ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_get_position(ctrl_handle->config.stepper, &step_position));
    return step_position - (int32_t)(position * ctrl_handle->config.step_scale);
}

static void motion_ctrl_watchdog_task(void *arg)
{
    motion_ctrl_handle_t *ctrl_handle = (motion_ctrl_handle_t *)arg;
    if (ctrl_handle == NULL)
    {
        ESP_LOGE(TAG, "Motion controller watchdog task received NULL handle");
        vTaskDelete(NULL);
        return;
    }

    /* Let the position sensor produce a few samples before the first baseline is read */
    vTaskDelay(pdMS_TO_TICKS(2 * AS5600_TELEMETRY_TASK_INTERVAL_MS + 50));

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(MOTION_CTRL_WATCHDOG_TASK_INTERVAL_MS));

        if (!atomic_load(&ctrl_handle->configured) || atomic_load(&ctrl_handle->state) == MOTION_CTRL_STATE_ERROR)
        {
            continue;
        }

        float current_position = 0.0f;
        if (motion_ctrl_get_position(ctrl_handle, &current_position) != ESP_OK)
        {
            continue;
        }

        /* -------------- Bounds checking -------------- */

        const float min_position = ctrl_handle->position_range * -1.0f;
        const float max_position = ctrl_handle->position_range;
        const bool out_of_bounds = (current_position < min_position) || (current_position > max_position);

        if (out_of_bounds && motion_ctrl_flag_error(ctrl_handle))
        {
            ESP_LOGE(TAG, "Position out of bounds: %d [mdeg] (min: %d, max: %d)",
                     (int)(current_position * 1000), (int)(min_position * 1000), (int)(max_position * 1000));
        }
    }
}