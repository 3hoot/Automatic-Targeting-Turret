#ifndef MOTION_CONTROLLER_UTILS_H
#define MOTION_CONTROLLER_UTILS_H

/* Preprocessor directive to avoid C++ name mangling */
#ifdef __cplusplus
extern "C" 
{
#endif

#include <stdint.h>
#include <math.h>

#include "motion_controller.h"

/*
 * Normalize an angle to the range [-180, 180) degrees.
 *
 * @param angle The angle to normalize.
 * @return The normalized angle in the range [-180, 180) degrees.
 */
static float motion_ctrl_utils_norm_180(float angle)
{
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0.0f) {
        angle += 360.0f;
    }
    return angle - 180.0f;
}

/*
 * Clamp a value between a minimum and maximum.
 *
 * @param value The value to clamp.
 * @param min The minimum value.
 * @param max The maximum value.
 */
static void motion_ctrl_utils_clamp(float* value, const float min, const float max)
{
    *value = fmaxf(min, fminf(*value, max));
}

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROLLER_UTILS_H */