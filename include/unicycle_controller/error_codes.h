#ifndef ERROR_CODES_H
#define ERROR_CODES_H

#include <iostream>

enum Error {
    TRAJECTORY_CTRL_INPUTS_DIFFERENT_SIZE,
    TRAJECTORY_POSE_DIFFERENT_SIZE,
    NO_POSE_FEEDBACK_4_INIT,
    DESIRED_TRAJECTORY_INCOMPLETE,
};

void printErrorCode(Error code);
#endif