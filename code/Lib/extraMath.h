#ifndef EXTRA_MATH_H
#define EXTRA_MATH_H

#ifdef EXTRA_MATH_DISABLE_INLINE
    #define EXTRA_MATH_FUNC_HEAD
#else
    #define EXTRA_MATH_FUNC_HEAD static inline
#endif

#include <math.h> // Base math library

EXTRA_MATH_FUNC_HEAD int mod_i(int x, int a);
EXTRA_MATH_FUNC_HEAD double mod(double x, double a);

EXTRA_MATH_FUNC_HEAD int sign_i(int x);
EXTRA_MATH_FUNC_HEAD double sign(double x);

EXTRA_MATH_FUNC_HEAD int min_i(int x1, int x2);
EXTRA_MATH_FUNC_HEAD double min(double x1, double x2);

EXTRA_MATH_FUNC_HEAD int max_i(int x1, int x2);
EXTRA_MATH_FUNC_HEAD double max(double x1, double x2);

EXTRA_MATH_FUNC_HEAD int clamp_i(int x, int min, int max);
EXTRA_MATH_FUNC_HEAD double clamp(double x, double min, double max);

EXTRA_MATH_FUNC_HEAD double avg_i(int* array, int size);
EXTRA_MATH_FUNC_HEAD double avg(double* array, int size);

EXTRA_MATH_FUNC_HEAD double wrapToPi(double theta);
EXTRA_MATH_FUNC_HEAD double wrapTo180(double theta);

// Modulo for Intagers
// @param x: First operand value of modulo operator
// @param a: Second operand value of modulo operator
EXTRA_MATH_FUNC_HEAD int mod_i(int x, int a) {
    return (x < 0) ? ((x+1)%a)+(a-1) : (x%a);
}

// Modulo for Doubles
// @param x: First operand value of modulo operator
// @param a: Second operand value of modulo operator
EXTRA_MATH_FUNC_HEAD double mod(double x, double a) {
    return x - (a*floor(x/a));
}

// Sign of input Integer
// @param x: Value to get sign of
EXTRA_MATH_FUNC_HEAD int sign_i(int x) {
    return (x < 0) ? -1 : ((x > 0) ? 1 : 0);
}

// Sign of input Double
// @param x: Value to get sign of
EXTRA_MATH_FUNC_HEAD double sign(double x) {
    return (x < 0) ? -1 : ((x > 0) ? 1 : 0);
}

// Returns min of two integers
// @param x1: first number
// @param x2: Second number
EXTRA_MATH_FUNC_HEAD int min_i(int x1, int x2) {
    return (x1 < x2) ? x1 : x2;
}

// Returns min of two doubles
// @param x1: first number
// @param x2: Second number
EXTRA_MATH_FUNC_HEAD double min(double x1, double x2) {
    return (x1 < x2) ? x1 : x2;
}

// Returns max of two integers
// @param x1: first number
// @param x2: Second number
EXTRA_MATH_FUNC_HEAD int max_i(int x1, int x2) {
    return (x1 > x2) ? x1 : x2;
}

// Returns max of two doubles
// @param x1: first number
// @param x2: Second number
EXTRA_MATH_FUNC_HEAD double max(double x1, double x2) {
    return (x1 > x2) ? x1 : x2;
}

// Clamp a ingeger value to a range
// @param x: Value to clamp
// @param min: Minimum value of range
// @param max: Maximum value of range
EXTRA_MATH_FUNC_HEAD int clamp_i(int x, int min, int max) {
    return (x < min) ? min : ((x > max) ? max : x);
}

// Clamp a double value to a range
// @param x: Value to clamp
// @param min: Minimum value of range
// @param max: Maximum value of range
EXTRA_MATH_FUNC_HEAD double clamp(double x, double min, double max) {
    return (x < min) ? min : ((x > max) ? max : x);
}

// Returns average of an array of integers
// @param x1: Array of integers
// @param x2: Size of array
EXTRA_MATH_FUNC_HEAD double avg_i(int* array, int size) {
    int sum = 0;
    for (int i = 0; i < size; ++i)
    {
        sum += array[i];
    }
    return (double)sum/size;
}

// Returns average of an array of doubles
// @param x1: Array of doubles
// @param x2: Size of array
EXTRA_MATH_FUNC_HEAD double avg(double* array, int size) {
    double sum = 0;
    for (int i = 0; i < size; ++i)
    {
        sum += array[i];
    }
    return sum/size;
}

// Wraps theta between -pi and pi
// @param theta: Angle to wrap (in radians)
EXTRA_MATH_FUNC_HEAD double wrapToPi(double theta)
{
    while(theta > M_PI)
    {
        theta -= 2*M_PI;
    }
    while(theta < -M_PI)
    {
        theta += 2*M_PI;
    }
    return theta;
}

// Wraps theta between -180 and 180
// @param theta: Angle to wrap (in degrees)
EXTRA_MATH_FUNC_HEAD double wrapTo180(double theta)
{
    while(theta > 180)
    {
        theta -= 360;
    }
    while(theta < -180)
    {
        theta += 360;
    }
    return theta;
}

#undef EXTRA_MATH_FUNC_HEAD
#endif