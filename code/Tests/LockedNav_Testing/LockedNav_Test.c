/* 
*    
*/

// C Library headers
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>

// Robot Control Library headers
#include <rc/start_stop.h>
#include <rc/uart.h>
#include <rc/gpio.h>
#include <rc/motor.h>

// Custom headers
#define DEBUG_XBEECOM 0
#define XBEE_ARRAY_DEBUG 1
#include "CoreLib.h"
#include "extraMath.h"
#include "XBeeArray.h"
#include "step.h"

// Uhhh some base stuff that probably should be moved to CoreLib.h
#define ARRSIZE(x) (sizeof(x)/sizeof(x[0]))
typedef unsigned char ubyte;

// Change to Main Assert functionality -- Probably should also be changed in CoreLib.h
#undef MAIN_ASSERT
void emergencyStop();
#define MAIN_ASSERT_LAMBDA emergencyStop() // Lambda def is for includer script to set
#define MAIN_ASSERT(condition,failTxt...) if(!(condition)){ printf(failTxt); MAIN_ASSERT_LAMBDA; return -1;}

// Set up Data Storage Structes
#define DATA_BUFFER_SIZE 5

typedef struct DistanceData {
    int curIndex;
    ubyte strengths[DATA_BUFFER_SIZE];
    double distances[DATA_BUFFER_SIZE];
} DistanceData;

typedef struct AngleData {
    int curIndex;
    ubyte strengths[DATA_BUFFER_SIZE*4];
    double angles[DATA_BUFFER_SIZE];
} AngleData;

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy) {
        rc_set_state(EXITING);
        return;
}

// function prototypes
int getMeasurement(struct XBeeArray_Settings xbeeArray, DistanceData* dData, AngleData* aData);
double updateDistance(DistanceData* dData);
double updateAngle(AngleData* aData);
int setWheelVelocitys(double v, double w);

static inline double wrapToPi(double theta) { return mod(theta, M_PI); }
static inline double wrapTo180(double theta) { return mod(theta, 180.0); }

int main() {
    printf("\tStarting Locked Navigation Test...\n");

    // set signal handler so the loop can exit cleanly
    signal(SIGINT, __signal_handler);

    // make sure another instance isn't running
    if(rc_kill_existing_process(2.0)<-2) return -1;
    rc_make_pid_file();

    // Start Pid state system
    rc_set_state(RUNNING);

    // Initalize property variables
    static const double Kv = 2; // Linear speed proportional gain
    static const double Kw = 2; // Angular speed proportional gain
    static const double vMax = 0.3; // Maximum linear speed [m/s]
    static const double wMax = M_PI/4; // Maximum angular speed [rad/s]
    struct XBeeArray_Settings array = {
        5,1,   // Uart buses Top(5) and Side(1) 
        3,1,   // GPIO 0 (Chip 3 Pin 1)
        3,2,   // GPIO 1 (Chip 3 Pin 2)
        0x1111 // Target Remote XBee's 16-bit address
    };

    // Initalize storage variables
    int result;
    DistanceData distanceData = {0}; // Distance data structure
    AngleData angleData = {0};       // Angle data structure

    // Initalize XBee Reflector Array
    printf("\tInitializing XBee Reflector Array...\n");
    result = XBeeArray_Init(array);
    MAIN_ASSERT(result == 0, "\tERROR: Failed to Initialize XBee Array.\n");

    // Initialize Stepper Motor
    StepperMotor sm;
    printf("\tInitializing Stepper Motor...\n");
    MAIN_ASSERT(stepper_init(&sm, 3, 4) != -1, "\tERROR: Failed to initialize Stepper Motor.\n");

    // Populate Inital Distance and Angle data
    printf("\tPopulating the initial distances and angles...\n");
    do {
        // Get Mesurement data
        result = getMeasurement(array, &distanceData, &angleData);
        MAIN_ASSERT(result == 0, "\tERROR: Failed to get measurement.\n");

        // Process Mesurement data
        updateDistance(&distanceData);
        updateAngle(&angleData);

        // Check if full and looped back to begining
    } while(distanceData.curIndex != 0);

    // Main Program loop
    while(rc_get_state() != EXITING) {
        // Update Distance and Angle Data
        double distance = updateDistance(&distanceData);
        double angle = updateAngle(&angleData);
        
        //printf("Angle to remote: %.0f\n", angle);

        // Calculate velocities
        double v = (Kv * distance) * cos(angle);
        v = clamp(v, -vMax/2, vMax);
        double w = (Kw * angle);
        w = clamp(w, -wMax, wMax);

        // Set Wheel Velocitys
        result = setWheelVelocitys(v, w);
        MAIN_ASSERT(result == 0, "\tERROR: Failed to set Wheel velocitys.\n");

        // Get Strength Values from the XBee Reflector Array
        result = getMeasurement(array, &distanceData, &angleData);
        MAIN_ASSERT(result == 0, "\tERROR: Failed to get measurement.\n");
    }

    // Close XBee Reflector Array
    printf("\tClosing XBee Reflector Array...\n");
    result = XBeeArray_Close(array);
    MAIN_ASSERT(result == 0, "\tERROR: Failed to close XBee Array.\n");

    // Close the stepper motor
    printf("\tClosing Stepper Motor...\n");
    MAIN_ASSERT(stepper_cleanup() != -1, "\tERROR: Failed to close Stepper Motor\n");

    // remove pid file LAST
    rc_remove_pid_file();

    printf("\tExiting...\n");
    return result;
}

int getMeasurement(struct XBeeArray_Settings xbeeArray, DistanceData* dData, AngleData* aData) {
    // Initalize storage variables
    unsigned char strengths[5];

    // Get Strength Values from the XBee Reflector Array
    int result = XBeeArray_GetStrengths(xbeeArray, strengths);
    ASSERT(result == 0, "\tERROR: Failed to get signal strength values from the XBee Array.\n");

    // Update Current Indexes
    dData->curIndex = (dData->curIndex + 1) % DATA_BUFFER_SIZE;
    aData->curIndex = (aData->curIndex + 1) % DATA_BUFFER_SIZE;
    int aStrIndex= (aData->curIndex) * 4; // Angle strength index

    // Store the strengths in the proper locations
    dData->strengths[dData->curIndex] = strengths[0];
    for (int i = 1; i <= 4; ++i) {
        aData->strengths[aStrIndex + i] = strengths[i];
    }

    // Return 0 for success
    return 0;
}

double updateDistance(DistanceData* dData) {
    // Define constant propertys
    const int length = ARRSIZE((*dData).strengths);

    // Find the average path loss in dBm
    int sum = 0;
    for (int i = 0; i < length; ++i) {
        sum += (*dData).strengths[i];
    }
    double avg_loss = sum/(double)length;

    // Calculate the distance in meters using free space path loss equation
    double distance = 2.9979*pow(10, avg_loss/20)/(4*M_PI*24);

    // Update distance log
    dData->distances[dData->curIndex] = distance;

    // Return distance value
    return distance;
}

double updateAngle(AngleData* aData) {
    // Define constant propertys
    static const int length = ARRSIZE((*aData).strengths);

    // Find min signal loss index
    int strIOffset = (aData->curIndex * 4);
    int minIndex = strIOffset;
    for (int i = strIOffset+1; i < (strIOffset + 4); i++){
        if(aData->strengths[i] < aData->strengths[minIndex])
            minIndex = i;
    }

    // Calculate estimate angle
    int sensorNum = minIndex-strIOffset;
    double angle = (((sensorNum+1)%4) - 1) * 90; // map to [0,90,180,-90]
    if (sensorNum == 2 && aData->angles[aData->curIndex-1] < 0) { // Handle -180
        angle = -angle;
    }
    // printf("Sensor: %d, angle: %.0f\n", sensorNum, angle);

    // Update Detected Angle Log
    aData->angles[aData->curIndex] = angle;

    // Determine target angle based on Log
    double mulFactor = 1;
    double sum = 0;
    double sumDiv = 0;
    for(int i = 0; i < length; ++i){
        int index = mod_i(aData->curIndex - i, length);
        sum += aData->angles[index] * mulFactor;
        sumDiv += mulFactor;
        mulFactor *= 0.5;
    }
    angle = sum/sumDiv;

    return angle;
}

int setWheelVelocitys(double v, double w) {
    // Define Constant propertys
    static const double dutyMax = 0.25; // Max duty cap
    static const double L = 0.21668;    // Distance between wheels [m]
    //static const double R = 0.0492125;  // Radius of wheels [m]
    static const double invMaxDutyVel = 1/1.39f; // Inverse Velocity at at Max Duty(1) [s/m]
    static const int motorL = 1, motorR = 2;     // Motor port numbers

    // Calculate liniar wheel velocitys
    double vR = (v - w*L/2); // right wheel liniar speed [m/s]
    double vL = (v + w*L/2); // left wheel liniar speed [m/s]

    // Calculate wheel duty cycles
    double dutyR = vR*invMaxDutyVel;
    double dutyL = vL*invMaxDutyVel;
    dutyR = clamp(dutyR, -dutyMax, dutyMax);
    dutyL = clamp(dutyL, -dutyMax, dutyMax);

    printf("dutyR: %f, dutyL: %f\n", dutyR, dutyL);

    // Set the motor speeds
    ASSERT(rc_motor_set(motorL, -dutyL) != -1, "\tERROR: Failed to set motor duty.\n");
    ASSERT(rc_motor_set(motorR,  dutyR) != -1, "\tERROR: Failed to set motor duty.\n");

    // Return 0 for success
    return 0;
}

void emergencyStop() {
    printf("\tEmergency Stop Cleanup Triggered..."); 
    fflush(stdout);

    // Set state to exiting for rc library
    rc_set_state(EXITING);

    // Close the stepper motor explicetley to prevent data leak
    stepper_cleanup();
    
    // remove pid file LAST
    rc_remove_pid_file();

    printf("Done\n");
}
