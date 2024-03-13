/*
    File: motor.h

    Created on 2024-02-13

    Description:
     Declares functions for seeting up the motor so that it can be run using PWM
     Also defines the min and max speeds of the motor
*/

#ifndef MOROT_SETUP_H
#define MOROT_SETUP_H

/*===============================MIN AND MAX MOTOR SPEEDS=========================*/
// These values are determined by experiment and are unique to every motor
// TODO: change these values to the actual values of the motor
#define PWM_MOTOR_MIN 750    // The value where the motor starts moving
#define PWM_MOTOR_MAX 3500   // Full speed (2^12 - 1)

/*===============================FUNCTION PROTOTYPES==============================*/

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