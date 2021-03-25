#ifndef ROBOT_H
#define ROBOT_H

// Robot Geometry
double L = 0.21668; // Distance between wheels [m]
double R = 0.0492125; // Radius of wheels [m]

// Control parameters
double Kp = 1; // Linear speed proportional gain
double Kw = 4; // Angular speed proportional gain
double vMax = 0.12; // Maximum linear speed [m/s]
double omegaMax = M_PI/6; // Maximum angular speed [rad/s]

#endif