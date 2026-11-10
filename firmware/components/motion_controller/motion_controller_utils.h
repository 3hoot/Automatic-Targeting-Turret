#ifndef MOTION_CONTROLLER_UTILS_H
#define MOTION_CONTROLLER_UTILS_H

#include <stdint.h>
#include <math.h>

/* Preprocessor directive to avoid C++ name mangling */
#ifdef __cplusplus
extern "C" 
{
#endif

/*
 * Normalize an angle to the range [-180, 180) degrees.
 *
 * @param angle The angle to normalize.
 * @return The normalized angle in the range [-180, 180) degrees.
 */
static inline float motion_ctrl_utils_norm_180(float angle)
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
static inline void motion_ctrl_utils_clamp_f(float* value, const float min, const float max)
{
    *value = fmaxf(min, fminf(*value, max));
}

/* Integer compare on purpose: going through float loses precision above 2^24 */
static inline void motion_ctrl_utils_clamp_u32(uint32_t* value, const uint32_t min, const uint32_t max)
{
    if (*value < min)
    {
        *value = min;
    }
    else if (*value > max)
    {
        *value = max;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROLLER_UTILS_H */