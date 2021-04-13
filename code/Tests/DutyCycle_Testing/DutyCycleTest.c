// C Library headers
#include <stdio.h>
#include <signal.h>

// Robot Control Library headers
#include <rc/time.h>
#include <rc/start_stop.h>
#include <rc/button.h>

#include "extraMath.h"
#include "core.h"
#include "odometry.h"
#include "robot.h"

double duty = 0.17;

//double L = 0.199;
double R = 0.0492125;

int main()
{
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

    // Initalize Encoders
    printf("\tInitalizing Quadrature Encoders...\n");
    MAIN_ASSERT(rc_encoder_eqep_init() == 0, "\tERROR: Failed to Initialize  Quadrature Encoders.\n");
    
    printf("\tInitalizing Motors...\n");
    ASSERT(rc_motor_init() != -1, "\tERROR: Failed to initialize motors\n");

    odometry_SetZeroRef();

    // Create the robot object
    Robot_t robot;
    robot.left_motor = 1;
    robot.right_motor = 2;

    // Record starting time
    uint64_t startTime = rc_nanos_since_boot();
    
    // Drive the robot at the desired duty cycle
    rc_motor_set(robot.right_motor, duty);
    rc_motor_set(robot.left_motor, -duty);

    // Wait for 2 seconds
    while (rc_nanos_since_boot() - startTime < 2E9){}

    //Get the encoder readings
    int leftEncVal = -rc_encoder_eqep_read(ODOMETRY_ENCODER_LEFT);
    int rightEncVal = rc_encoder_eqep_read(ODOMETRY_ENCODER_RIGHT);

    // Turn off the motors
    rc_motor_set(robot.right_motor, 0);
    rc_motor_set(robot.left_motor, 0);

    // Calculate the distances traveled by each wheel
    double distanceL = (double)(leftEncVal)/ODOMETRY_COUNTS_PER_REV*2*M_PI*R;
    double distanceR = (double)(rightEncVal)/ODOMETRY_COUNTS_PER_REV*2*M_PI*R;

    // Calculate the linear velocities of each wheel
    double vL = distanceL/2;
    double vR = distanceR/2;

    printf("==================================================\n");
    printf("Linear velocities for a duty cycle of %f:\nLeft: %f, Right: %f\n", duty, vL, vR);
    printf("==================================================\n");

    // Close the encoders
    printf("\tClosing Encoders...\n");
    MAIN_ASSERT(rc_encoder_eqep_cleanup() != -1, "\tERROR: Failed to close Encoders\n");

    rc_remove_pid_file();    // remove pid file LAST
}