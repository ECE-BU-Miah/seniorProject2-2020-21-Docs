/* 
*    Main Program
*/

// Development tools
#define DEBUG_XBEECOM 0
#define XBEE_ARRAY_DEBUG 1
#define MOTORS_OFF 0

// C Library headers
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>

// Robot Control Library headers
#include <rc/uart.h>
#include <rc/gpio.h>
#include <rc/start_stop.h>
#include <rc/motor.h>
#include <rc/encoder_eqep.h>
#include <rc/button.h>

// Custom headers
#include "CoreLib.h"
#include "robot.h"
#include "ATCom.h"
#include "XBeeArray.h"
#include "step.h"
#include "extraMath.h"
#include "odometry.h"

#define STEPS_PER_MEASUREMENT 5
#define DEG_PER_MEASUREMENT (int)(STEPS_PER_MEASUREMENT*DEG_PER_STEP)
#define NUM_MEASUREMENTS 90/DEG_PER_MEASUREMENT
#define MOVING_AVG_SIZE 2 // Size of the moving average window

#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        rc_set_state(EXITING);
        return;
}

// Function to stop program on pause press
static void __on_pause_press(void)
{
        rc_set_state(EXITING);
        return;
}

// Local functions
int getMeasurement(struct XBeeArray_Settings array, unsigned int curStep, unsigned char* strengths, unsigned char* distance_strengths, int* directional_strengths);
double getDistance(unsigned char* distance_strengths, int num_strengths);
int getAngle(int* directional_strengths, int num_strengths);
int set_robot_speeds(double v, double omega);
double avg_dbl(double* array, int size);
double avg_int(int* array, int size);
double absVal(double x);
double wrapToPi(double theta);
double wrapTo180(double theta);

int main(){
    printf("\tStarting Main Program...\n");

    // set signal handler so the loop can exit cleanly
    signal(SIGINT, __signal_handler);

    // make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

    // make PID file to indicate your project is running
    // due to the check made on the call to rc_kill_existing_process() above
    // we can be fairly confident there is no PID file already and we can
    // make our own safely.
    rc_make_pid_file();

    // Initialize pause button
    if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH, RC_BTN_DEBOUNCE_DEFAULT_US))
    {
        fprintf(stderr,"ERROR: failed to init buttons\n");
        return -1;
    }
    // Set the callback function
     rc_button_set_callbacks(RC_BTN_PIN_PAUSE, __on_pause_press, NULL);

    // Initialize mode button
    if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH, RC_BTN_DEBOUNCE_DEFAULT_US))
    {
        fprintf(stderr,"ERROR: failed to init buttons\n");
        return -1;
    }

    printf("\tPress Mode button to start\n");
    while (rc_button_get_state(RC_BTN_PIN_MODE) == RC_BTN_STATE_RELEASED)
    {}

    // Keep looping until state changes to EXITING
    rc_set_state(RUNNING);

    // Initalize state variables
    int position = 0; // Angular position of reflector array (degrees)
    int direction = 1; // Direction of stepper motor
    unsigned int curStep = 0; // Current step number
    struct XBeeArray_Settings array = {
        5,1,   // Uart buses Top(5) and Side(1) 
        3,1,   // GPIO 0 (Chip 3 Pin 1)
        3,2,   // GPIO 1 (Chip 3 Pin 2)
        0x1111 // Target Remote XBee's 16-bit address
    };

    // Initalize storage variables
    int result;
    unsigned char strengths[5]; // Temporary storage during measurement
    unsigned char distance_strengths[NUM_MEASUREMENTS]; // Strengths from transmitter
    int directional_strengths[4*NUM_MEASUREMENTS]; // Strengths from side XBees
    double distance[MOVING_AVG_SIZE] = {0, }; // Distances to remote in m
    int angle[MOVING_AVG_SIZE] = {0, }; // Angles to remote in radians
    unsigned int curMsmt = 0; // Next location to store the distance and angle in the moving average arrays
    double targetDistance = 0; // Average of distance array in meters
    double targetAngle = 0; // Average of angle array in degrees
    double targetAngle_rad = 0; // Average of angle array in radians
    double targetX = 0; // Estimated X coordinate of remote w.r.t. robot's local frame
    // double targetY = 0; // Estimated Y coordinate of remote w.r.t. robot's local frame
    double v = 0; // Desired linear velocity
    double omega = 0; // Desired angular velocity
    double theta = 0; // Angle (in rad) of robot with respect to where it was when the last target point was computed

    // Initalize XBee Reflector Array
    printf("\tInitializing XBee Reflector Array...\n");
    result = XBeeArray_Init(array);
    MAIN_ASSERT(result == 0, "\tERROR: Failed to Initialize XBee Array.\n");

    // Initialize Stepper Motor
    StepperMotor sm;
    printf("\tInitializing Stepper Motor...\n");
    MAIN_ASSERT(stepper_init(&sm, 3, 4) != -1, "\tERROR: Failed to initialize Stepper Motor.\n");

    // Initalize Encoders
    printf("\tInitalizing Quadrature Encoders...\n");
    MAIN_ASSERT(rc_encoder_eqep_init() == 0, "\tERROR: Failed to Initialize  Quadrature Encoders.\n");

    // Zero the stepper motor by rotating 90 degrees counterclockwise
    // The frame will stop the motor from moving past 0 degrees
    step(&sm, -1, 50);

    // Fill out the measurement arrays
    result = getMeasurement(array, curStep, strengths, distance_strengths, directional_strengths);
    MAIN_ASSERT(result == 0, "\tERROR: Failed to get measurement.\n");
    int num_sweeps = 0;
    while (num_sweeps < MOVING_AVG_SIZE)
    {
        // Move the stepper motor
        step(&sm, direction, STEPS_PER_MEASUREMENT);

        // Update current step number
        curStep += direction;

        // Update position and direction
        position += direction*DEG_PER_MEASUREMENT;

        // Check if a full sweep has been completed
        if ((position >= (90 - DEG_PER_MEASUREMENT)) || (position <= 0))
        {
            // Calculate the distance to the remote
            distance[curMsmt] = getDistance(distance_strengths, sizeof(distance_strengths));
            angle[curMsmt] = getAngle(directional_strengths, sizeof(directional_strengths));
            
            // Update the current measurement pointer
            curMsmt = (curMsmt + 1) % MOVING_AVG_SIZE;

            // Change direction
            direction *= -1;
            
            ++num_sweeps;
        }

        // Get Strength Values from the XBee Reflector Array
        result = getMeasurement(array, curStep, strengths, distance_strengths, directional_strengths);
        MAIN_ASSERT(result == 0, "\tERROR: Failed to get measurement.\n");
        if(rc_get_state() == EXITING) break;
    }


    /**************************************************************************
                                    MAIN LOOP
    **************************************************************************/
    while(rc_get_state() != EXITING)
    {
        // Move the stepper motor
        step(&sm, direction, STEPS_PER_MEASUREMENT);

        // Update current step number
        curStep += direction;

        // Update position and direction
        position += direction*DEG_PER_MEASUREMENT;

        // Check if a full sweep has been completed
        if ((position >= (90 - DEG_PER_MEASUREMENT)) || (position <= 0))
        {
            // Calculate the distance to the remote
            distance[curMsmt] = getDistance(distance_strengths, sizeof(distance_strengths));
            angle[curMsmt] = getAngle(directional_strengths, sizeof(directional_strengths));
            
            // Update the current measurement pointer
            curMsmt = (curMsmt + 1) % MOVING_AVG_SIZE;

            // Find the target distance to the remote
            targetDistance = avg_dbl(distance, MOVING_AVG_SIZE);
            // printf("Distance to remote: %.4f\n", targetDistance);
            
            // Find the target angle
            targetAngle = avg_int(angle, MOVING_AVG_SIZE);
            // printf("Angle to remote: %.0f\n", targetAngle);
            //Saturate the target angle to 15 degrees to avoid turning too fast
            targetAngle = clamp(targetAngle, -15, 15);
            
            // Convert target angle to radians
            targetAngle_rad = targetAngle*M_PI/180;
            targetX = targetDistance*cos(targetAngle_rad);
            // targetY = targetDistance*sin(targetAngle);

            // Calculate motor control signals
            v = sign(targetX)*Kp*targetDistance;
            v = (absVal(v) > vMax ? sign(v)*vMax : v); // Saturate velocity
            omega = Kw*targetAngle_rad;
            omega = (absVal(omega) > omegaMax ? sign(omega)*omegaMax : omega); // Saturate velocity

            // Reset the encoder positions to 0
            odometry_setZeroRef();

            // Pause measurements until robot is turned to target angle
            MAIN_ASSERT(set_robot_speeds(v, omega) == 0, "\tERROR: Failed to set motor speeds\n");
            while (absVal(theta - targetAngle) > 4 && rc_get_state() != EXITING)
            {
                // Determine the angle of the robot relative to where it was when the last measurement was completed
                theta = odometry_getAngle();
                printf("Target angle: %f\n Robot angle: %f\n", targetAngle, theta*180/M_PI);
            }

            //Just go straight now
            MAIN_ASSERT(set_robot_speeds(v, 0) == 0, "\tERROR: Failed to set motor speeds\n");

            // Change direction
            direction *= -1;
        }

        // Get Strength Values from the XBee Reflector Array
        result = getMeasurement(array, curStep, strengths, distance_strengths, directional_strengths);
        MAIN_ASSERT(result == 0, "\tERROR: Failed to get measurement.\n");
    }

    // Close XBee Reflector Array
    printf("\tClosing XBee Reflector Array...\n");
    result = XBeeArray_Close(array);
    MAIN_ASSERT(result == 0, "\tERROR: Failed to close XBee Array.\n");

    // Close the stepper motor
    printf("\tClosing Stepper Motor...\n");
    MAIN_ASSERT(stepper_cleanup() != -1, "\tERROR: Failed to close Stepper Motor\n");

    // Close the encoders
    printf("\tClosing Encoders...\n");
    MAIN_ASSERT(rc_encoder_eqep_cleanup() != -1, "\tERROR: Failed to close Encoders\n");

    rc_remove_pid_file();    // remove pid file LAST

    printf("\tExiting...\n");
    return result;
}

int getMeasurement(struct XBeeArray_Settings array, unsigned int curStep, unsigned char* strengths, unsigned char* distance_strengths, int* directional_strengths)
{
    // Offset values for the side XBees
    static int offset[4] = {29, 31, 34, 32};

    // Get Strength Values from the XBee Reflector Array
    int result = XBeeArray_GetStrengths(array, strengths);
    ASSERT(result == 0, "\tERROR: Failed to get signal strength values from the XBee Array.\n");

    // Store the strengths in the proper locations
    distance_strengths[curStep] = strengths[0]; // Transmitter XBee
    for (int i = 1; i <= 4; ++i) // Side XBees
    {
        directional_strengths[curStep + (i - 1)*10] = strengths[i] - offset[i];
    }

    return result;
}

double getDistance(unsigned char* distance_strengths, int num_strengths)
{
    // Find the average path loss in dBm
    int sum = 0;
    for (int i = 0; i < num_strengths; ++i)
    {
        sum += distance_strengths[i];
    }
    double avg_loss = (double)sum/num_strengths;

    // Calculate the distance in meters
    double distance = 2.9979*pow(10, avg_loss/20)/(4*M_PI*24);
    return distance;
}

int getAngle(int* directional_strengths, int num_strengths)
{
    // Calculate a value based on the surrounding strengths as well as the current one
    double val = 0.5*directional_strengths[num_strengths-1] + directional_strengths[0] + 0.5*directional_strengths[1];
    // Find the minimum value
    double min = val;
    int min_idx = 0;
    for (int i = 1; i < num_strengths-1; ++i)
    {
        val = 0.5*directional_strengths[i-1] + directional_strengths[i] + 0.5*directional_strengths[i + 1];
        if (val < min)
        {
            min = val;
            min_idx = i;
        }
    }

    // Convert the index to an angle in radians
    int angle = min_idx*9;
    return wrapTo180(angle);
}

int set_robot_speeds(double v, double omega)
{
    double omegaR = (v - omega*L/2)/R; // right wheel angular speed [rad/s]
    double omegaL = (v + omega*L/2)/R; // left wheel angular speed [rad/s]

    // Convert angular wheel speeds to duty cycles
    double dutyR = omegaR*0.05/omegaMax; // Right wheel duty cycle
    double dutyL = -omegaL*0.05/omegaMax; // Left wheel duty cycle

#if !MOTORS_OFF
    ASSERT(rc_motor_set(MOTOR_RIGHT, dutyR) != -1, "\tERROR: Failed to set right motor\n");
    ASSERT(rc_motor_set(MOTOR_LEFT, dutyL) != -1, "\tERROR: Failed to set left motor\n");
#endif

    return 0;
}

// Returns average of a double array
double avg_dbl(double* array, int size)
{
    double sum = 0;
    for (int i = 0; i < size; ++i)
    {
        sum += array[i];
    }
    return sum/size;
}

// Returns average of an integer array
double avg_int(int* array, int size)
{
    int sum = 0;
    for (int i = 0; i < size; ++i)
    {
        sum += array[i];
    }
    return (double)sum/size;
}

// Absolute value function (I was getting warnings with abs() for some reason)
double absVal(double x)
{
    return sign(x)*x;
}

// Wraps theta between -pi and pi
double wrapToPi(double theta)
{
    while(theta > M_PI)
    {
        theta -= 2*M_PI;
    }
    while(theta < -M_PI)
    {
        theta += 2*M_PI;
    }
    return theta;
}

// Wraps theta between -180 and 180
double wrapTo180(double theta)
{
    while(theta > 180)
    {
        theta -= 360;
    }
    while(theta < -180)
    {
        theta += 360;
    }
    return theta;
}