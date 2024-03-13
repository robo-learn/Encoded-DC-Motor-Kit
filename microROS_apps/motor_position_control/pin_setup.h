/*
 * pin_setup.h
 *
 *  Created on: 2024-03-13
 * Description: 
 *  Header file for setting up pins and defining
 *  and setting up the PWM channels
 */

#ifndef PIN_SETUP_H
#define PIN_SETUP_H

#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include "driver/gpio.h"
#include "driver/ledc.h"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif


/* Encoder Pins */
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3

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

#endif