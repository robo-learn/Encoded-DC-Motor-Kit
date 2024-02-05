#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/char.h>

#include "driver/gpio.h"


#ifdef ESP_PLATFORM
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
#endif

#include <unistd.h>

// Macro Functions
#define constrain(amt, low, high) ((amt)<low)?(low):((amt)>(high)?(high):(amt)))
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc!= RCL_RET_OK)) {printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {printf("Failed status on line %d: %d. COntinuing.\n", __LINE__, int(temp_rc));}}


// Constants
#define FRAME_TIME 100 // 1000 / FRAME_TIME = FPS (frames per second)
#define SLEEP_TIME 10
#define BLINK_DELAY 1000


// PINS
#define RED_LED 0
#define GREEN_LED 32
#define BLUE_LED 13

// Function Declaration
void setupPins(void);
void setupROS(void);
void blinkRedLED();
void blinkGreenLED();
void blinkBlueLED();

void subscriptionCallback(const void *);


// Variable Declaration
std_msgs__msg__Char msg;



// Main Function
void appMain(void *arg){
    setupPins();
    setupROS();
}

// Setup GPIO pins
void setupPins(void){
    // Set up LED pins
    gpio_pad_select_gpio(RED_LED);
    gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(GREEN_LED);
    gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(BLUE_LED);
    gpio_set_direction(BLUE_LED, GPIO_MODE_OUTPUT);

}

// Setup ROS2 
void setupROS(void){
    // Setup microROS
    rcl_allocator_t rcl_allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Create init options
    RCCHECK(rclc_support_init(&support, 0, NULL, &rcl_allocator));

    // Create a node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "light_led_node", "", &support));
    // Create a subscriber
    rcl_subscription_t subscriber;
    RCCHECK(
        rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Char),
            "/microROS/led_switch"
        )
    );

    // Create Executor
    rclc_executor_t executor;
    RCCHECK(
        rclc_executor_init(&executor, &support.context, 2, &rcl_allocator)
    );

    RCCHECK(
        rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscriptionCallback, ON_NEW_DATA)
    );

    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SLEEP_TIME));
        usleep(SLEEP_TIME*1000);
    }

    // Free Resources
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void subscriptionCallback(const void *msgin){
    const std_msgs__msg__Char *msg = (const std_msgs__msg__Char *)msgin;
    printf("Received: %c\n", msg->data);

    if (msg->data == 'R') {
        blinkRedLED();
    }else if (msg->data == 'B') {
        blinkBlueLED();
    }else if (msg->data == 'G') {
        blinkGreenLED();
    }else{
        blinkBlueLED();
        blinkGreenLED();
        blinkRedLED();
    }
}

// Blink LED functions
void blinkRedLED(void){
    // Turn on RED LED
    gpio_set_level(RED_LED, 1);
    vTaskDelay(BLINK_DELAY / portTICK_PERIOD_MS);
    // Turn off RED LED
    gpio_set_level(RED_LED, 0);
}

// Blink Green LED
void blinkGreenLED(void){
    // Turn on GREEN LED
    gpio_set_level(GREEN_LED, 1);
    vTaskDelay(BLINK_DELAY / portTICK_PERIOD_MS);
    // Turn off GREEN LED
    gpio_set_level(GREEN_LED, 0);
}

void blinkBlueLED(void){
    // Turn on BLUE LED
    gpio_set_level(BLUE_LED, 1);
    vTaskDelay(BLINK_DELAY / portTICK_PERIOD_MS);
    // Turn off BLUE LED
    gpio_set_level(BLUE_LED, 0);
}