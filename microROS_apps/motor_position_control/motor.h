#ifndef MOROT_SETUP_H
#define MOROT_SETUP_H

#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include "driver/gpio.h"
#include "driver/ledc.h"


/*
    Pins connected to the motor driver and the LEDs
*/

#define LED_BUILTIN 2
#define PIN_IN1 32
#define PIN_IN2 13

// PWM Channels
#define PWM_FORWARD LEDC_CHANNEL_0
#define PWM_BACKWARD LEDC_CHANNEL_1

// Other PWM settings
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_12_BIT
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_HIGH_SPEED_MODE

// These values are determined by experiment and are unique to every robot
#define PWM_MOTOR_MIN 750    // The value where the motor starts moving
#define PWM_MOTOR_MAX 3500   // Full speed (2^12 - 1)

/*
    Function prototypes
*/
/*
    Function - setup_pins(void)

    Description:
    Setup the pins for the motor driver and the LEDs
*/
void setup_pins(void);

/*
    Function - runMotorClockwise(uint16_t *)

    Description:
    Run the motor in the clockwise direction

    Parameters:
    uint16_t * - pointer to the speed of the motor
*/
void runMotorClockwise(uint16_t *);

/*
    Function - runMotorCounterClockwise(uint16_t *)

    Description:
    Run the motor in the counter clockwise direction

    Parameters:
    uint16_t * - pointer to the speed of the motor
*/
void runMotorCounterClockwise(uint16_t *);

/*
    Function - stopMotor(void)
    
    Description:
    Stop the motor

    Parameters:
    None
    */
void stopMotor(void);


#endif