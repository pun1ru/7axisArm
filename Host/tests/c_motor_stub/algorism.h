#ifndef ALGORISM_H
#define ALGORISM_H

static inline float AngleLimit(float value, float min_value, float max_value)
{
    while (value > max_value)
    {
        value -= max_value - min_value;
    }
    while (value < min_value)
    {
        value += max_value - min_value;
    }
    return value;
}

#endif
