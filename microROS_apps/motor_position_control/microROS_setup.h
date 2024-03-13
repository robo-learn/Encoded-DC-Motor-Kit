/*
    * microROS_setup.h
    * 
    * created on 2024-02-15
    * 
    * Description: 
    *   Header file for setting up the microROS agent 
    *   functions and inclusion of uselful libraries
*/


#ifndef MICROROS_SETUP_H
#define MICROROS_SETUP_H

#include <unistd.h>
// Include libraries for setting up the microROS
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Include libraries for defining the message type
// TODO: message type to be changed to float32
#include <std_msgs/msg/char.h>

// Macro functions
/*
    Macro - RCCHECK(fn)

    Description:
    This macro function checks the return status of a function and prints an error message if the status is not RCL_RET_OK.
    If the status is not RCL_RET_OK, the function will be aborted.

    Parameters:
    fn - The function to be checked for a return status of RCL_RET_OK.

    Returns:
    None

*/
#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }

// Constants
#define FRAME_TIME 100 // 1000 / FRAME_TIME = FPS (frames per second)
#define SLEEP_TIME 10

// Variables Declarations
std_msgs__msg__Char msg;

// Function declarations
/*
    Function - microROS_setup()

    Description:
    This function initializes the microROS middleware
    and sets up the communication between the microcontroller and the host computer.

    */

void microROS_setup(void);

/*
    Function - commandCallback(void):

    Description:
    This function is the callback function for the microROS agent.
    None
*/
void commandCallback(void);

#endif