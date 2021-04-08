#ifndef ROBOT_H
#define ROBOT_H

// Robot Control Library headers
#include <rc/motor.h>
#include <rc/encoder_eqep.h>

// Custom headers
#include "core.h"
#include "extraMath.h"
#include "xbeeArray.h"
#include "stepMotor.h"

static double vLMax = 1.291988; // Maximum linear velocity of left wheel in m/s
static double vRMax = 1.265705; // Maximum linear velocity of right wheel in m/s

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
    xbeeArray_settings array;

    // Stepper motor
    stepMotor_motor sm;

} Robot_t;

int robot_init(Robot_t* robot);
int robot_close(Robot_t* robot);
int robot_setSpeeds(Robot_t* robot, double v, double omega);

// Initializes the robot
// @param robot: The robot object to initialize
int robot_init(Robot_t* robot)
{
    // Assign robot variables
    robot->L = 0.199;
    robot->R = 0.0492125;
    robot->vMax = 0.13;
    robot->omegaMax = M_PI/4;
    robot->left_motor = 1;
    robot->right_motor = 2;
    robot->array = (xbeeArray_settings){
        5,1,   // Uart buses Top(5) and Side(1) 
        3,1,   // GPIO 0 (Chip 3 Pin 1)
        3,2,   // GPIO 1 (Chip 3 Pin 2)
        0x1111 // Target Remote XBee's 16-bit address
    };

    // Initalize XBee Reflector Array
    printf("\tInitializing XBee Reflector Array...\n");
    ASSERT(xbeeArray_Init(&(robot->array)) == 0, "\tERROR: Failed to Initialize XBee Array.\n");

    // Initialize Stepper Motor
    printf("\tInitializing Stepper Motor...\n");
    ASSERT(stepMotor_Init(&(robot->sm), 3, 4) != -1, "\tERROR: Failed to initialize Stepper Motor.\n");

    // Initalize Encoders
    printf("\tInitalizing Quadrature Encoders...\n");
    ASSERT(rc_encoder_eqep_init() == 0, "\tERROR: Failed to Initialize  Quadrature Encoders.\n");

    // Zero the stepper motor by rotating 90 degrees counterclockwise
    // The frame will stop the motor from moving past 0 degrees
    stepMotor_Step(&(robot->sm), -1, 50);

    return 0;
}

// Closes out the robot
// @param robot: the robot object to close out
int robot_close(Robot_t* robot)
{
    // Close XBee Reflector Array
    printf("\tClosing XBee Reflector Array...\n");
    MAIN_ASSERT(xbeeArray_Close(&(robot->array)) == 0, "\tERROR: Failed to close XBee Array.\n");

    // Close the stepper motor
    printf("\tClosing Stepper Motor...\n");
    MAIN_ASSERT(stepMotor_Cleanup() != -1, "\tERROR: Failed to close Stepper Motor\n");

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
    // Minimum duty cycle to apply to make sure the robot moves
    // (does not apply if velocity is 0)
    static const double minDuty = 0.25;
    
    // Velocities below this threshold will be treated as 0
    static const double threshold = 1E-3;

    double vR = (v - omega*robot->L/2); // right wheel linear speed [rad/s]
    double vL = (v + omega*robot->L/2); // left wheel linear speed [rad/s]
    
    double dutyR = 0;
    double dutyL = 0;

    // Only compute duty cycle if the velocity is above the threshold. Otherwise, leave it at 0
    if (fabs(vR) > threshold)
    {
        // Convert linear wheel speed to duty cycle by dividing by the maximum speed
        dutyR = sign(vR)*clamp(vR/vRMax, minDuty, 1.0); // Right wheel duty cycle
    }

    // Only compute duty cycle if the velocity is above the threshold. Otherwise, leave it at 0
    if (fabs(vL) > threshold)
    {
        // Convert angular wheel speeds to duty cycles by dividing by the maximum speed of each wheel
        dutyL = sign(vL)*clamp(vL/vLMax, minDuty, 1.0); // Left wheel duty cycle
    }

#if !MOTORS_OFF
    ASSERT(rc_motor_set(robot->right_motor, dutyR) != -1, "\tERROR: Failed to set right motor\n");
    ASSERT(rc_motor_set(robot->left_motor, dutyL) != -1, "\tERROR: Failed to set left motor\n");
#endif

    return 0;
}

#endif