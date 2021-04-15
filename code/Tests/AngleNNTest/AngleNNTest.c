// C Library headers
#include <stdio.h>
#include <signal.h>

// Robot Control Library headers
#include <rc/time.h>
#include <rc/start_stop.h>
#include <rc/button.h>

#include "extraMath.h"
#include "core.h"
#include "robot.h"
#include "angleNN.h"

#define NUM_STRENGTHS 5
#define AVG_SIZE 10

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        rc_set_state(EXITING);
        return;
}

int main()
{
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

    // Create and initialize a robot object
    Robot_t robot;
    printf("\tInitializing Robot...\n");
    MAIN_ASSERT(robot_init(&robot) == 0, "\tERROR: Failed to initialize robot.\n");

    // Storage variables
    ubyte strengths[NUM_STRENGTHS]; // The extra 1 is for the top strength
    double nn_input[ANGLE_NN_NUM_INPUTS][1];

    // Keep looping until state changes to EXITING
    rc_set_state(RUNNING);

    // Array to keep the angles to average
    double angles[AVG_SIZE] = { 0 };
    unsigned int index = 0;

    while(rc_get_state() != EXITING)
    {
        // Get the signal strength readings
        MAIN_ASSERT(xbeeArray_GetStrengths(&(robot.array), strengths) == 0, "\tERROR: Failed to get signal strength values from the XBee Array.\n");
        
        // Put the strengths into the neural network input
        for (unsigned int i = 0; i < NUM_STRENGTHS; ++i)
        {
            nn_input[i][0] = (double)(strengths[i]);
        }

        // Find the max and min strengths
        double maxStrength = nn_input[0][0];
        double minStrength = nn_input[0][0];
        for (unsigned int i = 1; i < NUM_STRENGTHS; ++i)
        {
            maxStrength = max(maxStrength, nn_input[i][0]);
            minStrength = min(minStrength, nn_input[i][0]);
        }

        // Finish out the nn_input array
        for (unsigned int i = NUM_STRENGTHS; i < ANGLE_NN_NUM_INPUTS; ++i)
        {
            nn_input[i][0] = (nn_input[i - 5][0] - minStrength)/(maxStrength - minStrength);
        }

        // Predict the angle
        angles[index] = angleNN_predict(nn_input);
        
        // Increment the index
        index = (index + 1)%AVG_SIZE;

        // Calculate the average of the angles
        double angle = avg(angles, AVG_SIZE);

        // Print the angle
        printf("Angle: %4.2f\n", angle);
        usleep(500000);
    }

    // Close out the robot
    printf("\tClosing the Robot...\n");
    robot_close(&robot);

    rc_remove_pid_file();    // remove pid file LAST

    return 0;
}