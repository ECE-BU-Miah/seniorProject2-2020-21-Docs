#ifndef ROBOT_H
#define ROBOT_H

// C Library headers
// #include <stdio.h>
// #include <string.h>
// #include <stdbool.h>
// #include <stdint.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <signal.h>
// #include <math.h>

// Robot Control Library headers
// #include <rc/uart.h>
// #include <rc/gpio.h>
#include <rc/motor.h>
#include <rc/encoder_eqep.h>
// #include <rc/button.h>

// Custom headers
#include "CoreLib.h"
// #include "ATCom.h"
#include "XBeeArray.h"
#include "step.h"
// #include "extraMath.h"
// #include "odometry.h"

typedef struct
{
    // Robot Geometry
    double L; // Distance between wheels [m]
    double R; // Radius of wheels [m]

    // Speed limits
    double vMax; // Maximum linear speed [m/s]
    double omegaMax; // Maximum angular speed [rad/s]

    int left_motor;
    int right_motor;

    // Array of XBees
    XBeeArray_Settings array;

    // Stepper motor
    StepperMotor sm;

} Robot_t;

int robot_init(Robot_t* robot);
int robot_close(Robot_t* robot);
int robot_setSpeeds(Robot_t* robot, double v, double omega);

// Initializes the robot
// @param robot: The robot object to initialize
int robot_init(Robot_t* robot)
{
    // Assign robot variables
    robot->L = 0.21668;
    robot->R = 0.0492125;
    robot->vMax = 0.12;
    robot->omegaMax = M_PI/4;
    robot->left_motor = 1;
    robot->right_motor = 2;
    robot->array = (XBeeArray_Settings){
        5,1,   // Uart buses Top(5) and Side(1) 
        3,1,   // GPIO 0 (Chip 3 Pin 1)
        3,2,   // GPIO 1 (Chip 3 Pin 2)
        0x1111 // Target Remote XBee's 16-bit address
    };

    // Initalize XBee Reflector Array
    printf("\tInitializing XBee Reflector Array...\n");
    ASSERT(XBeeArray_Init(&(robot->array)) == 0, "\tERROR: Failed to Initialize XBee Array.\n");

    // Initialize Stepper Motor
    printf("\tInitializing Stepper Motor...\n");
    ASSERT(stepper_init(&(robot->sm), 3, 4) != -1, "\tERROR: Failed to initialize Stepper Motor.\n");

    // Initalize Encoders
    printf("\tInitalizing Quadrature Encoders...\n");
    ASSERT(rc_encoder_eqep_init() == 0, "\tERROR: Failed to Initialize  Quadrature Encoders.\n");

    // Zero the stepper motor by rotating 90 degrees counterclockwise
    // The frame will stop the motor from moving past 0 degrees
    step(&(robot->sm), -1, 50);

    return 0;
}

// Closes out the robot
// @param robot: the robot object to close out
int robot_close(Robot_t* robot)
{
    // Close XBee Reflector Array
    printf("\tClosing XBee Reflector Array...\n");
    MAIN_ASSERT(XBeeArray_Close(&(robot->array)) == 0, "\tERROR: Failed to close XBee Array.\n");

    // Close the stepper motor
    printf("\tClosing Stepper Motor...\n");
    MAIN_ASSERT(stepper_cleanup() != -1, "\tERROR: Failed to close Stepper Motor\n");

    // Close the encoders
    printf("\tClosing Encoders...\n");
    MAIN_ASSERT(rc_encoder_eqep_cleanup() != -1, "\tERROR: Failed to close Encoders\n");

    return 0;
}

// Sets the linear and angular speed of the robot
// @param robot: Robot to set the speed of
// @param v: Linear velocity
// @param omega: Angular velocity
int robot_setSpeeds(Robot_t* robot, double v, double omega)
{
    double omegaR = (v - omega*robot->L/2)/robot->R; // right wheel angular speed [rad/s]
    double omegaL = (v + omega*robot->L/2)/robot->R; // left wheel angular speed [rad/s]

    // Convert angular wheel speeds to duty cycles
    double dutyR = omegaR*0.05/robot->omegaMax; // Right wheel duty cycle
    double dutyL = -omegaL*0.05/robot->omegaMax; // Left wheel duty cycle

#if !MOTORS_OFF
    ASSERT(rc_motor_set(robot->right_motor, dutyR) != -1, "\tERROR: Failed to set right motor\n");
    ASSERT(rc_motor_set(robot->left_motor, dutyL) != -1, "\tERROR: Failed to set left motor\n");
#endif

    return 0;
}

#endif