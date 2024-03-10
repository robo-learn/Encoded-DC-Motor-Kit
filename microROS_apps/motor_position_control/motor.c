#include "motor.h"

// TODO: revise pin configuration for setting up pwm with the motor driver useed
void setup_pins() {

    // Set up motor pins
    gpio_pad_select_gpio(PIN_IN1);
    gpio_set_direction(PIN_IN1, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(PIN_IN2);
    gpio_set_direction(PIN_IN2, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(PIN_IN1);
    gpio_set_direction(PIN_IN1, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(PIN_IN2);
    gpio_set_direction(PIN_IN2, GPIO_MODE_OUTPUT);

    // Set up PWM
    ledc_timer_config_t timer_conf;
    timer_conf.duty_resolution = PWM_RESOLUTION;
    timer_conf.freq_hz = PWM_FREQUENCY;
    timer_conf.speed_mode = PWM_MODE;
    timer_conf.timer_num = PWM_TIMER;
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf[2] = {
        {
            .channel = PWM_FORWARD,
            .duty = 0,
            .gpio_num = PIN_IN1,
            .speed_mode = PWM_MODE,
            .hpoint = 0,
            .timer_sel = PWM_TIMER
        },
        {
            .channel = PWM_BACKWARD,
            .duty = 0,
            .gpio_num = PIN_IN2,
            .speed_mode = PWM_MODE,
            .hpoint = 0,
            .timer_sel = PWM_TIMER
        }
    };

    for(int i = 0; i < 2; i++) {
        ledc_channel_config(&channel_conf[i]);
    }
}

void runMotorClockwise(uint16_t *dutyCycle){
    ledc_set_duty(PWM_MODE, PWM_FORWARD, *dutyCycle);
    ledc_set_duty(PWM_MODE, PWM_BACKWARD, 0);
    ledc_update_duty(PWM_MODE, PWM_FORWARD);
    ledc_update_duty(PWM_MODE, PWM_BACKWARD);
}

void runMotorCounterClockwise(uint16_t *dutyCycle){
    ledc_set_duty(PWM_MODE, PWM_BACKWARD, *dutyCycle);
    ledc_set_duty(PWM_MODE, PWM_FORWARD, 0);
    ledc_update_duty(PWM_MODE, PWM_BACKWARD);
    ledc_update_duty(PWM_MODE, PWM_FORWARD);
}

void stopMotor(void){
    ledc_set_duty(PWM_MODE, PWM_FORWARD, 0);
    ledc_set_duty(PWM_MODE, PWM_BACKWARD, 0);
    ledc_update_duty(PWM_MODE, PWM_FORWARD);
    ledc_update_duty(PWM_MODE, PWM_BACKWARD);
}

