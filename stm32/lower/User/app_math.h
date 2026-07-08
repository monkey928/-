#ifndef APP_MATH_H
#define APP_MATH_H

static inline float App_ClampFloat(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

#endif
