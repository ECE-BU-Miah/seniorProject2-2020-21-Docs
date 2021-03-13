#ifndef EXTRA_MATH_H
#define EXTRA_MATH_H

#include <math.h> // Base math library

int mod_i(int x, int a);
double mod_d(double x, double a);

int sign_i(int x);
double sign_d(double x);

// Modulo for Intagers
// @param x: First operand value of modulo operator
// @param a: Second operand value of modulo operator
int mod_i(int x, int a){
    return (x%a) + ((x < 0) ? a : 0);
}

// Modulo for Doubles
// @param x: First operand value of modulo operator
// @param a: Second operand value of modulo operator
double mod_d(double x, double a){
    return fmod(x, a) + ((x<0) ? a : 0);
}

// Sign of input Integer
// @peram x: Value to get sign of
int sign_i(int x){
    return (x == 0) ? 0 : ((x < 0) ? -1 : 1);
}

// Sign of double Double
// @peram x: Value to get sign of
double sign_d(double x){
    return (x == 0) ? 0 : ((x < 0) ? -1 : 1);
}

#endif