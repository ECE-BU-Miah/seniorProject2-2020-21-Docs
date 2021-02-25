#ifndef _STEP_H_
#define _STEP_H_

#include <rc/motor.h>
#include <rc/time.h>

// Number of degrees per step on the stepper motor
#define DEG_PER_STEP 1.8

typedef struct
{
    // Motor driver ports used for the stepper motor
    int m1;
    int m2;

    // Direction of the motor
    int direction;

    // State of the windings
    int state;
} StepperMotor;

// Function to initialize a stepper motor
void stepper_init(StepperMotor* stepper, int m1, int m2)
{
    // Set the motor ports
    stepper->m1 = m1;
    stepper->m2 = m2;

    // Set the current state
    stepper->state = 0;
}

// Function to step the motor a specified number of steps
void step(StepperMotor* stepper, int direction, int numSteps)
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
            stepper->state = (stepper->state - 1)%4;
        }

        // Winding states
        switch(stepper->state)
        {
            case 0:
            default:
                rc_motor_set(stepper->m1, 1);
                rc_motor_set(stepper->m2, 1);
                break;
            case 1:
                rc_motor_set(stepper->m1,-1);
                rc_motor_set(stepper->m2,1);
                break;
            case 2:
                rc_motor_set(stepper->m1,-1);
                rc_motor_set(stepper->m2,-1);
                break;
            case 3:
                rc_motor_set(stepper->m1,1);
                rc_motor_set(stepper->m2,-1);
                break;
        }

        // Sleep to allow motor to turn
        rc_usleep(12000);
    }
}

#endif