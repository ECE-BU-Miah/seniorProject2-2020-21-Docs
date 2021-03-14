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
// @peram x: Value to get sign of
EXTRA_MATH_FUNC_HEAD int sign_i(int x) {
    return (x < 0) ? -1 : ((x > 0) ? 1 : 0);
}

// Sign of input Double
// @peram x: Value to get sign of
EXTRA_MATH_FUNC_HEAD double sign(double x) {
    return (x < 0) ? -1 : ((x > 0) ? 1 : 0);
}

// Returns min of two integers
// @peram x1: first number
// @peram x2: Second number
EXTRA_MATH_FUNC_HEAD int min_i(int x1, int x2) {
    return (x1 < x2) ? x1 : x2;
}

// Returns min of two doubles
// @peram x1: first number
// @peram x2: Second number
EXTRA_MATH_FUNC_HEAD double min(double x1, double x2) {
    return (x1 < x2) ? x1 : x2;
}

// Returns max of two integers
// @peram x1: first number
// @peram x2: Second number
EXTRA_MATH_FUNC_HEAD int max_i(int x1, int x2) {
    return (x1 > x2) ? x1 : x2;
}

// Returns max of two doubles
// @peram x1: first number
// @peram x2: Second number
EXTRA_MATH_FUNC_HEAD double max(double x1, double x2) {
    return (x1 > x2) ? x1 : x2;
}

// Clamp a ingeger value to a range
// @peram x: Value to clamp
// @peram min: Minimum value of range
// @peram max: Maximum value of range
EXTRA_MATH_FUNC_HEAD int clamp_i(int x, int min, int max) {
    return (x < min) ? min : ((x > max) ? max : x);
}

// Clamp a double value to a range
// @peram x: Value to clamp
// @peram min: Minimum value of range
// @peram max: Maximum value of range
EXTRA_MATH_FUNC_HEAD double clamp(double x, double min, double max) {
    return (x < min) ? min : ((x > max) ? max : x);
}

#undef EXTRA_MATH_FUNC_HEAD
#endif