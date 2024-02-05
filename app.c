#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/char.h>


#include "driver/gpio.h"
#include "driver/ledc.h"


#ifdef ESP_PLATFORM
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
#endif

#include <unistd.h>

// Macros functions
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define RCLCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc!= RCL_RET_OK)) {printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define FRAME_TIMEOUT 100
#define SLEEP_TIME 10

// Pin Definition
#define PIN_IN1 34
#define PIN_IN2 35

#define ENCODER_A_PIN 36
#define ENCODER_B_PIN 37

#define START_LED_PIN 32
#define STOP_LED_PIN 0

// PWM Channels
#define PIN_IN1_PWM LEDC_CHANNEL_0
#define PIN_IN2_PWM LEDC_CHANNEL_1

// Other PWM Settings
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_12_BIT
#define PWM_TIMER LEDC_TIMER_1
#define PWM_MODE LEDC_HIGH_SPEED_MODE

// 
#define PWM_MOTOR_MIN 750
#define PWM_MOTOR_MAX 4095

// Function Prototypes
void setupPins(void);
void setupROS(void);

void subscriptionCallback(const void *);

void rotateClockwise(unsigned int);
void rotateCounterClockwise(unsigned int);

// Variable declarations and initializations
std_msgs__msg__Char msg;
unsigned int dutyCycle = 0;

// Main Function
void appMain(void *args){
    setupPins();
    setupROS();
}

/***************************************************/
// Function Definition

void setupPins(){
    //Setup indicator LED pins
    // GREEN LED to indicate the motor is active
    gpio_pad_select_gpio(START_LED_PIN);
    gpio_set_direction(START_LED_PIN, GPIO_MODE_OUTPUT);
    // RED LED to indicate the motor is not moving
    gpio_pad_select_gpio(STOP_LED_PIN);
    gpio_set_direction(STOP_LED_PIN, GPIO_MODE_OUTPUT);
    
    // Setup Motor Pins
    gpio_pad_select_gpio(PIN_IN1);
    gpio_set_direction(PIN_IN1, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(PIN_IN2);
    gpio_set_direction(PIN_IN2, GPIO_MODE_OUTPUT);

    // Setup Encoder pins as input
    gpio_pad_select_gpio(ENCODER_A_PIN);
    gpio_set_direction(ENCODER_A_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(ENCODER_A_PIN);

    gpio_pad_select_gpio(ENCODER_B_PIN);
    gpio_set_direction(ENCODER_B_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(ENCODER_B_PIN);

    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    // PWM Channels configuration
    ledc_channel_config_t ledc_channel[2] = {
        {
            .channel = PIN_IN1_PWM,
            .duty = 0,
            .gpio_num = PIN_IN1,
            .speed_mode = PWM_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER_1
        },

        {
            .channel = PIN_IN2_PWM,
            .duty = 0,
            .gpio_num = PIN_IN2,
            .speed_mode = PWM_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER_1
        }
    };

    for (int i = 0; i < 2; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }

}


void setupROS(void){
    rcl_allocator_t rcl_allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Create init options
    RCLCHECK(rclc_support_init(&support, 0, NULL, &rcl_allocator));

    // Create a node
    rcl_node_t node;
    RCLCHECK(rclc_node_init_default(&node, "dc_motor_speed_control", "", &support));

    // Create a subscriber
    rcl_subscription_t subscriber;
    RCLCHECK(
        rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Char),
            "/microROS/SpeedControlKeys"
        )
    );

    // Create an executor
    rclc_executor_t executor;
    RCLCHECK(
        rclc_executor_init(
            &executor,
            &support.context,
            2,
            &rcl_allocator));

    RCLCHECK(
        rclc_executor_add_subscription(
            &executor,
            &subscriber,
            &msg, 
            &subscriptionCallback,
            ON_NEW_DATA));
    
    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SLEEP_TIME));
        usleep(SLEEP_TIME*1000);
    }

    // Free resources allocated
    RCLCHECK(rcl_subscription_fini(&subscriber, &node));
    RCLCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);

}


void subscriptionCallback(const void *msgin){
    const std_msgs__msg__Char *msg = (const std_msgs__msg__Char *)msgin;

    if (msg->data == 's'){
        dutyCycle = 2048;
        gpio_set_level(START_LED_PIN, 1);
        gpio_set_direction(STOP_LED_PIN, 0);
        rotateClockwise(dutyCycle);
    }

    if (msg->data == 'p'){
        dutyCycle = 0;
        gpio_set_level(STOP_LED_PIN, 1);
        gpio_set_direction(START_LED_PIN, 0);
        rotateClockwise(dutyCycle);

    }

    if (msg->data == '>'){
        rotateClockwise(dutyCycle);
    }

    if (msg->data == '<'){
        rotateCounterClockwise(dutyCycle);
    }

}

void rotateClockwise(unsigned int duty_cycle){
    ledc_set_duty(PWM_MODE, PIN_IN1_PWM, dutyCycle);
    ledc_update_duty(PWM_MODE, PIN_IN1_PWM);
    gpio_set_level(PIN_IN1, 1);
    gpio_set_level(PIN_IN2, 0);
}

void rotateCounterClockwise(unsigned int dutyCycle){
    ledc_set_duty(PWM_MODE, PIN_IN2_PWM, dutyCycle);
    ledc_update_duty(PWM_MODE, PIN_IN2_PWM);
    gpio_set_level(PIN_IN2, 1);
    gpio_set_level(PIN_IN1, 0);
}
