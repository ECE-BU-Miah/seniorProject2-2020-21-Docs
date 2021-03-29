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
#include <stdlib.h>
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
#include "core.h"
#include "../constants/robotSettings.h"
#include "atCom.h"
#include "xbeeArray.h"
#include "setpMotor.h"
#include "extraMath.h"
#include "odometry.h"

#define STEPS_PER_MEASUREMENT 5
#define DEG_PER_MEASUREMENT (int)(STEPS_PER_MEASUREMENT*STEP_MOTOR_DEG_PER_STEP)
#define NUM_MEASUREMENTS 90/DEG_PER_MEASUREMENT
#define MOVING_AVG_SIZE 2 // Size of the moving average window

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
int getMeasurement(xbeeArray_settings* arrayP, unsigned int curStep, ubyte* strengths, ubyte* distance_strengths, int* directional_strengths);
double getDistance(ubyte* distance_strengths, int num_strengths);
int getAngle(int* directional_strengths, int num_strengths);

// Global control parameters
const double Kp = 1; // Linear speed proportional gain
const double Kw = 4; // Angular speed proportional gain
const int maxTargetAngle = 30; // Maximum angle change for a single measurement

// Global Constants
const double Kp = 1; // Linear speed proportional gain
const double Kw = 4; // Angular speed proportional gain
const double vMax = 0.12; // Maximum linear speed [m/s]
const double omegaMax = M_PI/6; // Maximum angular speed [rad/s]

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

    Robot_t robot;

    // Initalize storage variables
    int result;
    ubyte strengths[5]; // Temporary storage during measurement
    ubyte distance_strengths[NUM_MEASUREMENTS]; // Strengths from transmitter
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
    double theta = 0; // Angle (in degrees) of robot with respect to where it was when the last target point was computed

    // Initialize Robot
    printf("\tInitializing Robot...\n");
    robot_init(&robot);

    // Fill out the measurement arrays
    result = getMeasurement(&(robot.array), curStep, strengths, distance_strengths, directional_strengths);
    MAIN_ASSERT(result == 0, "\tERROR: Failed to get measurement.\n");
    int num_sweeps = 0;
    while (num_sweeps < MOVING_AVG_SIZE)
    {
        // Move the stepper motor
        step(&robot.sm, direction, STEPS_PER_MEASUREMENT);

        // Update current step number
        curStep += direction;

        // Update position and direction
        position += direction*DEG_PER_MEASUREMENT;

        // Check if a full sweep has been completed
        if ((position >= (90 - DEG_PER_MEASUREMENT)) || (position <= 0))
        {
            // Calculate the distance to the remote
            distance[curMsmt] = getDistance(distance_strengths, sizeof(distance_strengths)/sizeof(distance_strengths[0]));
            angle[curMsmt] = getAngle(directional_strengths, sizeof(directional_strengths)/sizeof(directional_strengths[0]));
            
            // Update the current measurement pointer
            curMsmt = (curMsmt + 1) % MOVING_AVG_SIZE;

            // Change direction
            direction *= -1;
            
            ++num_sweeps;
        }

        // Get Strength Values from the XBee Reflector Array
        result = getMeasurement(&(robot.array), curStep, strengths, distance_strengths, directional_strengths);
        MAIN_ASSERT(result == 0, "\tERROR: Failed to get measurement.\n");
        if(rc_get_state() == EXITING) break;
    }


    /**************************************************************************
                                    MAIN LOOP
    **************************************************************************/
    while(rc_get_state() != EXITING)
    {
        // Move the stepper motor
        step(&robot.sm, direction, STEPS_PER_MEASUREMENT);

        // Update current step number
        curStep += direction;

        // Update position and direction
        position += direction*DEG_PER_MEASUREMENT;

        // Check if a full sweep has been completed
        if ((position >= (90 - DEG_PER_MEASUREMENT)) || (position <= 0))
        {
            // Calculate the distance to the remote
            distance[curMsmt] = getDistance(distance_strengths, sizeof(distance_strengths)/sizeof(distance_strengths[0]));
            angle[curMsmt] = getAngle(directional_strengths, sizeof(directional_strengths)/sizeof(directional_strengths[0]));
            // printf("%d, %d\n", angle[0], angle[1]);
            
            // Update the current measurement pointer
            curMsmt = (curMsmt + 1) % MOVING_AVG_SIZE;

            // Find the target distance to the remote
            targetDistance = avg(distance, MOVING_AVG_SIZE);
            // printf("Distance to remote: %.4f\n", targetDistance);
            
            // Find the target angle
            targetAngle = avg_i(angle, MOVING_AVG_SIZE);
            // printf("Angle to remote: %.0f\n", targetAngle);
            //Saturate the target angle to avoid turning too fast
            targetAngle = clamp(targetAngle, -maxTargetAngle, maxTargetAngle);
            
            // Convert target angle to radians
            targetAngle_rad = targetAngle*M_PI/180;
            targetX = targetDistance*cos(targetAngle_rad);
            // targetY = targetDistance*sin(targetAngle);

            // Calculate motor control signals
            v = sign(targetX)*Kp*targetDistance;
            v = clamp(v, -robot.vMax, robot.vMax); // Saturate velocity
            omega = Kw*targetAngle_rad;
            omega = clamp(omega, -robot.omegaMax, robot.omegaMax); // Saturate velocity

            // Reset the encoder positions to 0
            odometry_SetZeroRef();
            theta = 0;

            // Pause measurements until robot is turned to target angle
            MAIN_ASSERT(robot_setSpeeds(&robot, v, omega) == 0, "\tERROR: Failed to set motor speeds\n");
#if !MOTORS_OFF // Odometry only works if the motors move
            while (abs(theta - targetAngle) > 2 && rc_get_state() != EXITING)
            {
                // Determine the angle of the robot relative to where it was when the last measurement was completed
                theta = odometry_getAngle(robot.R, robot.L);
                // printf("Target angle: %f\n Robot angle: %f\n", targetAngle, theta);
            }
#endif

            //Just go straight now
            MAIN_ASSERT(robot_setSpeeds(&robot, v, 0) == 0, "\tERROR: Failed to set motor speeds\n");

            // Change direction
            direction *= -1;
        }

        // Get Strength Values from the XBee Reflector Array
        result = getMeasurement(&(robot.array), curStep, strengths, distance_strengths, directional_strengths);
        MAIN_ASSERT(result == 0, "\tERROR: Failed to get measurement.\n");
    }

    rc_remove_pid_file();    // remove pid file LAST

    printf("\tExiting...\n");
    return result;
}

int getMeasurement(xbeeArray_settings* arrayP, unsigned int curStep, ubyte* strengths, ubyte* distance_strengths, int* directional_strengths)
{
    // Offset values for the side XBees
    static int offset[4] = {29, 31, 34, 32};

    // Get Strength Values from the XBee Reflector Array
    int result = xbeeArray_GetStrengths(arrayP, strengths);
    ASSERT(result == 0, "\tERROR: Failed to get signal strength values from the XBee Array.\n");

    // Store the strengths in the proper locations
    distance_strengths[curStep] = strengths[0]; // Transmitter XBee
    for (int i = 1; i <= 4; ++i) // Side XBees
    {
        directional_strengths[curStep + (i - 1)*10] = strengths[i] - offset[i];
    }

    return result;
}

double getDistance(ubyte* distance_strengths, int num_strengths)
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