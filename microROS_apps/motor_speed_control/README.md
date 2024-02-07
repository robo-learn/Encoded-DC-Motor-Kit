**README.md**

# ESP32 MicroROS DC Motor Control

This file contains code for controlling a DC motor using the ESP32 microcontroller with Micro-ROS. The setup allows you to teleoperate the DC motor using the `teleop_twist_keyboard` ROS2 node.

## Overview

The provided code is a Micro-ROS agent running on an ESP32 (ESP-Wroom-32) microcontroller. The agent controls a single DC motor based on Twist messages received from the `teleop_twist_keyboard` node.

## Prerequisites

Before using this code, make sure you have the following installed:

- [Micro-ROS](https://micro.ros.org/) on ESP32
- [ROS2](https://docs.ros.org/en/humble/Installation.html) installed on your development machine
- `teleop_twist_keyboard` ROS2 package
    ```bash
    sudo apt-get install ros-humble-teleop-twist-keyboard
    ```

## Usage

1. **Run the Micro-ROS Agent on ESP32:**

   Flash the ESP32 with the provided code. Ensure the motor is connected to the specified GPIO pins (`PIN_IN1`, `PIN_IN2`) and the PWM channels (`PWM_FORWARD`, `PWM_BACKWARD`).

2. **Run the `teleop_twist_keyboard` Node:**

   Open a new terminal and run the `teleop_twist_keyboard` node:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard_node
   ```

3. **Teleoperate the DC Motor:**

   Use the keyboard to send Twist messages to control the DC motor. The keyboard bindings are provided in the `msg` variable in the script. For example:

   - `i`: Move forward
   - `j`: Turn left
   - `l`: Turn right
   - ...

   Refer to the provided message for a complete list of keyboard commands.

   Hold down the shift key for holonomic mode and use keys like `U`, `I`, `O` for strafing.

   Adjust the speeds using keys `q`, `z`, `w`, `x`, `e`, `c`.

   Press `CTRL-C` to exit the teleop node.

## Code Explanation

The code consists of two parts:

1. **Micro-ROS Agent (`motor_kit_simple_app`):**
   - Sets up GPIO pins for the DC motor.
   - Initializes PWM channels for motor control.
   - Subscribes to `/cmd_vel` topic for Twist messages.
   - Converts Twist messages to PWM values for motor control.
   - Runs a timer callback to update motor PWM based on Twist messages.

2. **`teleop_twist_keyboard` Node:**
   - Reads keyboard inputs and publishes Twist messages.
   - Allows teleoperation of a robot using keyboard commands.

**Note:** Ensure you have the necessary hardware connections and configurations before running the code.
