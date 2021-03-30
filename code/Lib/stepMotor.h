#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <rc/motor.h>
#include <rc/time.h>
#include "core.h"

// Number of degrees per step on the stepper motor
#define STEP_MOTOR_DEG_PER_STEP 1.8

typedef struct
{
    // Motor driver ports used for the stepper motor
    int m1;
    int m2;

    // Direction of the motor
    int direction;

    // State of the windings
    int state;
} stepMotor_motor;

// Function Prototypes
int stepMotor_Init(stepMotor_motor* stepper, int m1, int m2);
int stepMotor_Step(stepMotor_motor* stepper, int direction, int numSteps);
int stepMotor_SetState(stepMotor_motor* stepper);
int stepMotor_Cleanup();

// Function to initialize a stepper motor
int stepMotor_Init(stepMotor_motor* stepper, int m1, int m2)
{
    // Initialize the motor drivers
    ASSERT(rc_motor_init() != -1, "\tERROR: Failed to initialize motors\n");

    // Set the motor ports
    stepper->m1 = m1;
    stepper->m2 = m2;

    // Set the current state
    stepper->state = 0;
    stepMotor_SetState(stepper);

    return 0;
}

// Function to step the motor a specified number of steps
int stepMotor_Step(stepMotor_motor* stepper, int direction, int numSteps)
{
    for (int i = 0; i < numSteps; ++i)
    {
        // Handle directional control
        if(direction > 0)
        {
            stepper->state = (stepper->state + 1)%4;
        }
        else if (direction < 0)
        {
            // Note: (state + 3)%4 is the same as decrementing state by 1 and wrapping at 0
            stepper->state = (stepper->state + 3)%4;
        }

        // Set the state of the motor windings
        stepMotor_SetState(stepper);

        // Sleep to allow motor to turn
        rc_usleep(20000);
    }

    return 0;
}

int stepMotor_SetState(stepMotor_motor* stepper)
{
    // Winding states
    switch(stepper->state)
    {
        case 0:
        default:
            ASSERT(rc_motor_set(stepper->m1, 1) != -1, "\tERROR: Failed to set motor %d\n", stepper->m1);
            ASSERT(rc_motor_set(stepper->m2, 1) != -1, "\tERROR: Failed to set motor %d\n", stepper->m2);
            break;
        case 1:
            ASSERT(rc_motor_set(stepper->m1, -1) != -1, "\tERROR: Failed to set motor %d\n", stepper->m1);
            ASSERT(rc_motor_set(stepper->m2, 1) != -1, "\tERROR: Failed to set motor %d\n", stepper->m2);
            break;
        case 2:
            ASSERT(rc_motor_set(stepper->m1, -1) != -1, "\tERROR: Failed to set motor %d\n", stepper->m1);
            ASSERT(rc_motor_set(stepper->m2, -1) != -1, "\tERROR: Failed to set motor %d\n", stepper->m2);
            break;
        case 3:
            ASSERT(rc_motor_set(stepper->m1, 1) != -1, "\tERROR: Failed to set motor %d\n", stepper->m1);
            ASSERT(rc_motor_set(stepper->m2, -1) != -1, "\tERROR: Failed to set motor %d\n", stepper->m2);
            break;
    }

    return 0;
}

int stepMotor_Cleanup()
{
    return rc_motor_cleanup();
}

#endif