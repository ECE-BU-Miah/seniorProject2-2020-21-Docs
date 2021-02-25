#include "step.h"
#include <rc/motor.h>
#include <rc/time.h>

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
            stepper->state++;
        }
        else if (direction < 0)
        {
            stepper->state--;
        }

        // Handle state rollover
        if(stepper->state >= 4)
        {
            stepper->state = 0;
        }
        else if(stepper->state < 0)
        {
            stepper->state = 3;
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