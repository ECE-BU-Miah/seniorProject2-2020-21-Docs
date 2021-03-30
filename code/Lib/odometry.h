#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <rc/encoder_eqep.h>

#include "core.h"

// Encoder defines
#define ODOMETRY_ENCODER_LEFT 1
#define ODOMETRY_ENCODER_RIGHT 2
#define ODOMETRY_COUNTS_PER_REV 1200

double odometry_GetAngle();
double odometry_GetDistance();
int odometry_SetZeroRef();

// @param R: Radius of the wheels
// @param L: Distance between wheels
double odometry_GetAngle(double R, double L)
{
    int leftEncVal = -rc_encoder_eqep_read(ODOMETRY_ENCODER_LEFT);
    int rightEncVal = rc_encoder_eqep_read(ODOMETRY_ENCODER_RIGHT);
    double theta_rad = (leftEncVal - rightEncVal)*2*M_PI*R/(1200*L);
    double theta = theta_rad*180/M_PI;

    return theta;
}

// @param R: Radius of the wheels
double odometry_GetDistance(double R)
{
    int leftEncVal = -rc_encoder_eqep_read(ODOMETRY_ENCODER_LEFT);
    int rightEncVal = rc_encoder_eqep_read(ODOMETRY_ENCODER_RIGHT);
    double distance = (leftEncVal + rightEncVal)*2*M_PI*R/(1200*2);

    return distance;
}

int odometry_SetZeroRef()
{
    ASSERT(rc_encoder_eqep_write(ODOMETRY_ENCODER_LEFT, 0) == 0, "\tERROR: Failed to reset left encoder\n");
    ASSERT(rc_encoder_eqep_write(ODOMETRY_ENCODER_RIGHT, 0) == 0, "\tERROR: Failed to reset right encoder\n")

    return 0;
}

#endif