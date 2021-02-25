#ifndef _STEP_H_
#define _STEP_H_

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
void stepper_init(StepperMotor* stepper, int m1, int m2);

// Function to step the motor a specified number of steps
void step(StepperMotor* stepper, int direction, int numSteps);

#endif