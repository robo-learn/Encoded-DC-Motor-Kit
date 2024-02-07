#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>

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


// Macro functions
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}

// Constants
#define FRAME_TIME 100 // 1000 / FRAME_TIME = FPS (frames per second)
#define SLEEP_TIME 10

// PINS
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

// Function declarations
void setupPins();
void setupROS();
void commandCallback(const void *msgin);
void timerCallback(rcl_timer_t *timer, int64_t last_call_time);
float fmap(float val, float in_min, float in_max, float out_min, float out_max);


void runMotorClockwise(uint16_t *);
void runMotorCounterClockwise(uint16_t *);
void increaseSpeed(uint16_t *);
void decreaseSpeed(uint16_t *);
void stopMotor();


// Variable definitions
std_msgs__msg__Char msg;
uint16_t dutyCycle = 2048;

// Start bit
bool start = false;
bool clockwise = true;


// Main function
void appMain(void *arg) {
    setupPins();
    setupROS();
}

// Set up GPIO pins for use with the DC motors.
void setupPins() {

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


// Set up ROS2 node and publisher
void setupROS(){
    // Micro ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Create init options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "motor_control_app", "", &support));
    // Create subcriber
    rcl_subscription_t subscriber;
    RCCHECK(
        rclc_subscription_init_default(
            &subscriber,
            &node, 
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Char),
            "/microROS/Control_keys"
        )
    );

    // Create timer
    rcl_timer_t timer;
    RCCHECK(
        rclc_timer_init_default(
            &timer, 
            &support,
            RCL_MS_TO_NS(FRAME_TIME),
            timerCallback
        )
    );

    // Create executor
    rclc_executor_t executor;
    RCCHECK(
        rclc_executor_init(&executor, &support.context, 2, &allocator)
    );

    RCCHECK(
        rclc_executor_add_subscription(&executor, &subscriber, &msg, commandCallback, ON_NEW_DATA)
    );
    
    RCCHECK(
        rclc_executor_add_timer(&executor, &timer)
    );

    while (1)
    {
        rclc_executor_spin_some(
            &executor, 
            RCL_MS_TO_NS(SLEEP_TIME)
        );
        usleep(SLEEP_TIME * 1000);
    }

    // Clean up
    RCCHECK(
        rcl_subscription_fini(&subscriber, &node)
    );
    RCCHECK(rcl_node_fini(&node));
    
    vTaskDelete(NULL);
    
}


void commandCallback(const void *msgin){
    const std_msgs__msg__Char *msg = (const std_msgs__msg__Char *) msgin;
}

void timerCallback(rcl_timer_t *timer, int64_t last_call_time){

    if (timer == NULL) return;

    // Read message
    unsigned char data = msg.data;


    // If keyboard press is '.' rotate motor in clockwise direction
    if (data == '.'){
        start = true;
        clockwise = true;
    }

    // If keyboard press is ',' rotate motor in counterclockwise direction
    if (data == ','){
        start = true;
        clockwise = false;
    }


    // If keyboard press is w increase the speed by 10%
    if (data == 'w'){
        increaseSpeed(&dutyCycle);
    }

    // If keyboard press is x decrease the speed by 10%
    if (data == 'x'){
        decreaseSpeed(&dutyCycle);
    }


    /* Run the motor */
    if (start){
        if (clockwise){
            runMotorClockwise(&dutyCycle);
        }else{
            runMotorCounterClockwise(&dutyCycle);
            }
        }

    /* Reset motor speed to default speed */
    if (data == 'r'){
        dutyCycle = 1048;
        }

    /* Stop the motor */
    // If keyboard press is k stop the motor
    if (data == 'k'){
        start = false;
        stopMotor();
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


void increaseSpeed(uint16_t *dutyCycle) {
    if (*dutyCycle < PWM_MOTOR_MAX){
        *dutyCycle = (uint16_t) ((float)(*dutyCycle) * 1.1);
    }
}

void decreaseSpeed(uint16_t *dutyCycle) {
    if (*dutyCycle > PWM_MOTOR_MIN){
        *dutyCycle = (uint16_t) ((float)(*dutyCycle) * 0.9);
    }
}