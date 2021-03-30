/* 
*    
*/

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

// Custom headers
#define DEBUG_XBEECOM 0
#define XBEE_ARRAY_DEBUG 1
#include "core.h"
#include "atCom.h"
#include "xbeeArray.h"
#include "stepMotor.h"

#define STEPS_PER_MEASUREMENT 5
#define DEG_PER_MEASUREMENT (int)(STEPS_PER_MEASUREMENT*STEP_MOTOR_DEG_PER_STEP)
#define NUM_MEASUREMENTS 90/DEG_PER_MEASUREMENT
#define MOVING_AVG_SIZE 1 // Size of the moving average window

#define DUTY_MAX 0.25

#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        rc_set_state(EXITING);
        return;
}

// Local functions
int getMeasurement(xbeeArray_settings* arrayP, unsigned int curStep, ubyte* strengths, ubyte* distance_strengths, ubyte* directional_strengths);
double getDistance(ubyte* distance_strengths, int num_strengths);
int getAngle(ubyte* directional_strengths, int num_strengths);
double avg_dbl(double* array, int size);
double avg_int(int* array, int size);
int sign(double x);
double wrapToPi(double theta);
double wrapTo180(double theta);

// Control parameters
double Kp = 1; // Linear speed proportional gain
double Kw = 4; // Angular speed proportional gain
double L = 0.21668; // Distance between wheels [m]
double R = 0.0492125; // Radius of wheels [m]
double vMax = 0.1; // Maximum linear speed [m/s]
double omegaMax = M_PI/6; // Maximum angular speed [rad/s]

int main(){
    printf("\tStarting Location Test...\n");

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

    // Initalize state variables
    int position = 0; // Angular position of reflector array (degrees)
    int direction = 1; // Direction of stepper motor
    unsigned int curStep = 0; // Current step number
    xbeeArray_settings array = {
        5,1,   // Uart buses Top(5) and Side(1) 
        3,1,   // GPIO 0 (Chip 3 Pin 1)
        3,2,   // GPIO 1 (Chip 3 Pin 2)
        0x1111 // Target Remote XBee's 16-bit address
    };

    // Initalize storage variables
    int result;
    ubyte strengths[5]; // Temporary storage during measurement
    ubyte distance_strengths[NUM_MEASUREMENTS]; // Strengths from transmitter
    ubyte directional_strengths[4*NUM_MEASUREMENTS]; // Strengths from side XBees
    double distance[MOVING_AVG_SIZE] = {0, }; // Distances to remote in m
    int angle[MOVING_AVG_SIZE] = {0, }; // Angles to remote in radians
    unsigned int curMsmt = 0; // Next location to store the distance and angle in the moving average arrays
    double avg_dist = 0; // Average of distance array in meters
    double avg_ang = 0; // Average of angle array in degrees
    double avg_ang_rad = 0; // Average of angle array in radians
    double targetX = 0; // Estimated X coordinate of remote w.r.t. robot's local frame
    // double targetY = 0; // Estimated Y coordinate of remote w.r.t. robot's local frame
    double v = 0; // Desired linear velocity
    double omega = 0; // Desired angular velocity
    double omegaR = 0; // Right wheel angular velocity
    double omegaL = 0; // Left wheel angular velocity
    double dutyR = 0; // Right wheel duty cycle
    double dutyL = 0; // Left wheel duty cycle

    // Initalize XBee Reflector Array
    printf("\tInitializing XBee Reflector Array...\n");
    result = xbeeArray_Init(&array);
    MAIN_ASSERT(result == 0, "\tERROR: Failed to Initialize XBee Array.\n");

    // Initialize Stepper Motor
    stepMotor_motor sm;
    printf("\tInitializing Stepper Motor...\n");
    MAIN_ASSERT(stepMotor_Init(&sm, 3, 4) != -1, "\tERROR: Failed to initialize Stepper Motor.\n");

    // Keep looping until state changes to EXITING
    rc_set_state(RUNNING);

    // Fill out the measurement arrays
    result = getMeasurement(&array, curStep, strengths, distance_strengths, directional_strengths);
    MAIN_ASSERT(result == 0, "\tERROR: Failed to get measurement.\n");
    int num_sweeps = 0;
    while (num_sweeps < MOVING_AVG_SIZE)
    {
        // Move the stepper motor
        stepMotor_Step(&sm, direction, STEPS_PER_MEASUREMENT);

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
        result = getMeasurement(&array, curStep, strengths, distance_strengths, directional_strengths);
        MAIN_ASSERT(result == 0, "\tERROR: Failed to get measurement.\n");
    }


    /**************************************************************************
                                    MAIN LOOP
    **************************************************************************/
    while(rc_get_state() != EXITING)
    {
        // Move the stepper motor
        stepMotor_Step(&sm, direction, STEPS_PER_MEASUREMENT);

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

            // Calculate the coordinates of the remote
            avg_dist = avg_dbl(distance, MOVING_AVG_SIZE);
            // printf("Distance to remote: %.4f\n", avg_dist);
            avg_ang = avg_int(angle, MOVING_AVG_SIZE);
            printf("Angle to remote: %.0f\n", avg_ang);
            avg_ang_rad = avg_ang*M_PI/180;
            targetX = avg_dist*cos(avg_ang_rad);
            // targetY = avg_dist*sin(avg_ang);

            // Calculate motor control signals
            v = sign(targetX)*Kp*avg_dist;
            v = (abs(v) > vMax ? sign(v)*vMax : v); // Saturate velocity
            omega = Kw*avg_ang_rad;
            omega = (abs(omega) > omegaMax ? sign(omega)*omegaMax : omega); // Saturate velocity

            omegaR = (v - omega*L/2)/R; // right wheel angular speed [rad/s]
            omegaL = (v + omega*L/2)/R; // left wheel angular speed [rad/s]

            // Convert angular wheel speeds to duty cycles
            dutyR = omegaR*0.05/omegaMax;
            dutyL = -omegaL*0.05/omegaMax;
            // dutyR = (abs(dutyR) > DUTY_MAX ? sign(dutyR)*DUTY_MAX : dutyR);
            // dutyL = (abs(dutyL) > DUTY_MAX ? sign(dutyL)*DUTY_MAX : dutyL);

            // printf("dutyR: %f\ndutyL: %f\n", dutyR, dutyL);

            // Set the motor speeds
            rc_motor_set(MOTOR_LEFT, dutyL);
            rc_motor_set(MOTOR_RIGHT, dutyR);

            // Change direction
            direction *= -1;
        }

        // Get Strength Values from the XBee Reflector Array
        result = getMeasurement(&array, curStep, strengths, distance_strengths, directional_strengths);
        MAIN_ASSERT(result == 0, "\tERROR: Failed to get measurement.\n");
    }

    // Close XBee Reflector Array
    printf("\tClosing XBee Reflector Array...\n");
    result = xbeeArray_Close(&array);
    MAIN_ASSERT(result == 0, "\tERROR: Failed to close XBee Array.\n");

    // Close the stepper motor
    printf("\tClosing Stepper Motor...\n");
    MAIN_ASSERT(stepMotor_Cleanup() != -1, "\tERROR: Failed to close Stepper Motor\n");

    rc_remove_pid_file();    // remove pid file LAST

    printf("\tExiting...\n");
    return result;
}

int getMeasurement(xbeeArray_settings* arrayP, unsigned int curStep, ubyte* strengths, ubyte* distance_strengths, ubyte* directional_strengths)
{
    // Get Strength Values from the XBee Reflector Array
    int result = xbeeArray_GetStrengths(arrayP, strengths);
    ASSERT(result == 0, "\tERROR: Failed to get signal strength values from the XBee Array.\n");

    // Store the strengths in the proper locations
    distance_strengths[curStep] = strengths[0]; // Transmitter XBee
    for (int i = 1; i <= 4; ++i) // Side XBees
    {
        directional_strengths[curStep + (i - 1)*10] = strengths[i];
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

int getAngle(ubyte* directional_strengths, int num_strengths)
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

double avg_dbl(double* array, int size)
{
    double sum = 0;
    for (int i = 0; i < size; ++i)
    {
        sum += array[i];
    }
    return sum/size;
}

double avg_int(int* array, int size)
{
    int sum = 0;
    for (int i = 0; i < size; ++i)
    {
        sum += array[i];
    }
    return (double)sum/size;
}

int sign(double x)
{
    return (x > 0) - (x < 0);
}

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