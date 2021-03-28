#ifndef ROBOT_H
#define ROBOT_H

// Robot Geometry
const double L = 0.21668; // Distance between wheels [m]
const double R = 0.0492125; // Radius of wheels [m]

// Control parameters
const double Kp = 1; // Linear speed proportional gain
const double Kw = 4; // Angular speed proportional gain
const double vMax = 0.12; // Maximum linear speed [m/s]
const double omegaMax = M_PI/6; // Maximum angular speed [rad/s]

#endif