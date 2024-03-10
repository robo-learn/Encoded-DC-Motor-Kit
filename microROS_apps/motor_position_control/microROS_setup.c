#include "microROS_setup.h"

void microROS_setup(){
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

    // TODO: Execute node and subscriber
    
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
