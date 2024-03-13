/*
 * encoder.h
 * created on 2024-03-13
 * Description: Header file for setting up encoder pins and functions
*/
#ifndef ENCODER_H
#define ENCODER_H

// Contant defining the number of encoder counts per revolution
#define ENCODER_COUNTS_PER_REVOLUTION 500 // TODO: this should be changed to the actual value

/* Shaft direction */
bool clockwise_direction;

/* Varialbles to store the encoder counts for encoders A and B */
extern int encoder_a_count;
extern int encoder_b_count;

/* Variables to store angular position and velocity of the shaft calculated from the encoder counts */
float angular_position;
float angular_velocity;


/* Encoder Setup functions */

/*
    Function - setup_encoder_pins(void)

    Description:
    Setup the pins for the encoder
*/
void setup_encoder_pins(void);

/*
    Function - reset_encoder(void)

    Description:
    Reset the encoder value to 0
*/
void reset_encoder(void);

/*
    Function - float get_angular_position(extern int*)

    Description:
     Calculates the angular positions from the encoder counts
*/
float get_angular_position(extern int*);

/*
    Function - float get_angular_velocity(extern int*)

    Description:
     Calculates the angular velocity from the encoder counts
*/
float get_angular_velocity(extern int*);

#endif