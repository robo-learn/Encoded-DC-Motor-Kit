/*
    * app.c
    *
    *  Created on: 2024-03-10
    * 
    * Description:
    * This file contains the application code for the motor position control app.
*/

#include "app.h"

// Variable definitions
uint16_t dutyCycle = 2048;

// Start bit
bool start = false;
bool clockwise = true;

/*
    Function - appMain(void)

    Description:
    This function is the main function for the motor position control app.
    It sets up the GPIO pins and the ROS middleware.
*/
void appMain(void *args)
{
    setup_pins();
    microROS_setup();
}

/**
 * @brief This function is a timer callback function that is triggered every 100ms.
 * 
 * @param timer The timer that triggered the callback.
 * @param last_call_time The time when the last call to this function was made.
 */
void timerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    if (timer == NULL)
    {
        return;
    }

    // Read message
    unsigned char data = msg.data;

    // If keyboard press is '.' rotate motor in clockwise direction
    if (data == '.')
    {
        start = true;
        clockwise = true;
    }

    // If keyboard press is ',' rotate motor in counterclockwise direction
    if (data == ',')
    {
        start = true;
        clockwise = false;
    }

    // // If keyboard press is w increase the speed by 10%
    // if (data == 'w')
    // {
    //     increaseSpeed(&dutyCycle);
    // }

    // // If keyboard press is x decrease the speed by 10%
    // if (data == 'x')
    // {
    //     decreaseSpeed(&dutyCycle);
    // }

    /* Run the motor */
    if (start)
    {
        if (clockwise)
        {
            runMotorClockwise(&dutyCycle);
        }
        else
        {
            runMotorCounterClockwise(&dutyCycle);
        }
    }

    /* Reset motor speed to default speed */
    if (data == 'r')
    {
        dutyCycle = 1048;
    }

    /* Stop the motor */
    // If keyboard press is k stop the motor
    if (data == 'k')
    {
        start = false;
        stopMotor();
    }
}