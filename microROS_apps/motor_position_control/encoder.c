/*
    * encoder.c
    *
    *  Created on: 2024-03-13
    * 
    * Description:
    * Contains definitions for the encoder functions.
*/
#include "pin_setup.h"
#include "encoder.h"

void setup_encoder() {
    // Set up encoder pins
    gpio_pad_select_gpio(PIN_ENCODER_A);
    gpio_set_direction(PIN_ENCODER_A, GPIO_MODE_INPUT);
    gpio_pad_select_gpio(PIN_ENCODER_B);
    gpio_set_direction(PIN_ENCODER_B, GPIO_MODE_INPUT);

    // Enable interrupt on both rising edges
    gpio_set_intr_type(PIN_ENCODER_A, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(PIN_ENCODER_B, GPIO_INTR_POSEDGE);
    gpio_intr_enable(PIN_ENCODER_A);
    gpio_intr_enable(PIN_ENCODER_B);
}

void reset_encoder() {
    encoder_a_count = 0;
    encoder_b_count = 0;
}

