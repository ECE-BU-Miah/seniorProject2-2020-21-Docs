#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <rc/encoder_eqep.h>

#include "core.h"
#include "robot.h"

// Encoder defines
#define ENCODER_LEFT 1
#define ENCODER_RIGHT 2
#define COUNTS_PER_REV 1200

double odometry_getAngle();
double odometry_getDistance();
int odometry_setZeroRef();

double odometry_getAngle()
{
    int leftEncVal = -rc_encoder_eqep_read(ENCODER_LEFT);
    int rightEncVal = rc_encoder_eqep_read(ENCODER_RIGHT);
    double theta_rad = (leftEncVal - rightEncVal)*2*M_PI*R/(1200*L);
    double theta = theta_rad*180/M_PI;

    return theta;
}

double odometry_getDistance()
{
    int leftEncVal = -rc_encoder_eqep_read(ENCODER_LEFT);
    int rightEncVal = rc_encoder_eqep_read(ENCODER_RIGHT);
    double distance = (leftEncVal + rightEncVal)*2*M_PI*R/(1200*2);

    return distance;
}

int odometry_setZeroRef()
{
    ASSERT(rc_encoder_eqep_write(ENCODER_LEFT, 0) == 0, "\tERROR: Failed to reset left encoder\n");
    ASSERT(rc_encoder_eqep_write(ENCODER_RIGHT, 0) == 0, "\tERROR: Failed to reset right encoder\n")

    return 0;
}

#endif