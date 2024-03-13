/*
    * microROS_setup.c
    *
    *  Created on: 2024-03-13
    * 
    * Description:
    * Contains the microROS configuration and implementations of microROS agent objects such as
    * nodes, subscribers etc 
    * Also implements the main loop for the motor position control app
    * communication with ROS2
*/
#include "pin_setup.h"
#include "microROS_setup.h"


void microROS_setup(){
    // Micro ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Create init options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "motor_position_control_app", "", &support));
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


    // Create executor
    rclc_executor_t executor;
    RCCHECK(
        rclc_executor_init(&executor, &support.context, 2, &allocator)
    );

    RCCHECK(
        rclc_executor_add_subscription(&executor, &subscriber, &msg, commandCallback, ON_NEW_DATA)
    )
    

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
