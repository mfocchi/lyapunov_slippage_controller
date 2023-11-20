#include "error_codes.h"

void printErrorCode(Error code)
{
    switch (code)
    {
    case TRAJECTORY_CTRL_INPUTS_DIFFERENT_SIZE:
        std::cout << "Control inputs vectors have different size CODE:["<< code << "]" << std::endl;
        break;
    case TRAJECTORY_POSE_DIFFERENT_SIZE:
        std::cout << "Desired poses vectors have different size CODE:["<< code << "]" << std::endl;
        break;
    case NO_POSE_FEEDBACK_4_INIT:
        std::cout << "No pose data was recieved during initialization CODE:["<< code << "]" << std::endl;
        break;
    case DESIRED_TRAJECTORY_INCOMPLETE:
        std::cout << "No desired values have been set! Remember to use setControlInputsDesired(...)" << std::endl;
        break;
    default:
        std::cout << "Code not found CODE:["<< code << "]" << std::endl;
        break;
    }
}