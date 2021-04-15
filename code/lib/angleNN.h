#ifndef ANGLENN_H
#define ANGLENN_H
/* Neural network to predict angle
 * Architecture:
 *  - 10 inputs (top, side1, side2, side3, side4, normTop, normSide1, normSide2, normSide3, normSide4)
 *  - 50 nodes (plus 1 bias node) in the hidden layer
 *  - 1 node in the output layer
 *  - All activations are the relu function
*/

#define ANGLE_NN_NUM_INPUTS 10

double angleNN_predict(double inputs[ANGLE_NN_NUM_INPUTS][1]);

#endif