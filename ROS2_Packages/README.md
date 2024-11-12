---

# Encoded DC Motor Kit ROS2 Project Documentation

---

## Table of Contents
- [1. Introduction](#1-introduction)
- [2. System Overview](#2-system-overview)
- [3. Creating the URDF from CAD](#3-creating-the-urdf-from-cad)
- [4. ROS2 Package Setup](#4-ros2-package-setup)
  - [4.1. encoded_dc_motor_kit_description Package](#41-encoded_dc_motor_kit_description-package)
  - [4.2. encoded_dc_motor_kit_control Package](#42-encoded_dc_motor_kit_control-package)
  - [4.3. encoded_dc_motor_kit_interfaces Package](#43-encoded_dc_motor_kit_interfaces-package)
  - [4.4. encoded_dc_motor_kit_bringup Package](#44-encoded_dc_motor_kit_bringup-package)
- [5. Hardware Interface Plugin Development](#5-hardware-interface-plugin-development)
  - [5.1. Communication Protocol](#51-communication-protocol)
  - [5.2. Implementation Steps](#52-implementation-steps)
- [6. Summary](#6-summary)

---

# 1. Introduction

## Project Overview
The **Encoded DC Motor Kit** is an educational and practical system designed for use in robotics and automation projects. It features a geared DC motor equipped with a magnetic encoder for precise motion control and position feedback. This project utilizes an ESP32-Wroom microcontroller for real-time motor control and communication.

## Objectives
The primary goal of this project is to develop a comprehensive ROS2-based framework that includes:
- A digital twin representation of the motor system for simulation in ROS2.
- Integration of a custom hardware interface to bridge the control logic implemented on the ESP32 with the ROS2 control system.
- A modular and reproducible controller design that follows control theory principles, enabling system identification and controller tuning.

## Key Features
- **Custom Controller Implementation**: The project incorporates a tailor-made controller implemented on the ESP32. This control system includes system identification and tuning methodologies that are educational and scalable for use in higher education settings.
- **Digital Twin in ROS2**: The hardware control logic in the ESP32 is mirrored in the ROS2 environment through the hardware interface, creating a synchronized digital twin that allows for testing and validation of control strategies.
- **ROS2 Integration**: The project involves setting up essential ROS2 packages for robot description, control, and simulation, providing an end-to-end solution for motor control training and development.

## Target Audience
This project is intended for:
- Higher education institutions that teach robotics, control systems, and automation.
- Researchers and developers interested in precise motor control and digital twin technology.

With this setup, users can simulate, test, and deploy control algorithms on the Encoded DC Motor Kit in a manner that bridges the gap between theoretical learning and real-world applications.

---
# 2. System Overview

## Hardware Components
The **Encoded DC Motor Kit** comprises the following main components:
- **Geared DC Motor**: A motor equipped with a gearbox to provide high torque and reduced speed for precision control.
- **Magnetic Encoder**: Integrated with the motor shaft, the encoder provides real-time feedback on position and speed, essential for closed-loop control.
- **ESP32-Wroom Microcontroller**: Serves as the main controller for executing the custom control algorithms and communicating with the ROS2 system.

### Suggested Image Placement:
- **Diagram of the complete motor kit**, showing the motor, encoder, and ESP32 with labels for each component.
- **Close-up of the magnetic encoder** to highlight its placement and connection with the motor.

## Control Architecture
### 2.1. Hardware Control Logic
The control logic is implemented in the firmware running on the ESP32. This includes:
- **System Identification**: The process of analyzing motor behavior to develop an accurate model. This enables the controller to predict how the motor responds to different inputs.
- **Controller Tuning**: Ensures that the control system can adjust motor output effectively and maintain stability and accuracy. This is achieved using PID (Proportional-Integral-Derivative) control or more advanced techniques, depending on project requirements.

### 2.2. Digital Twin Integration
A key feature of the project is the synchronization between the physical motor system and its digital counterpart in ROS2:
- **ROS2 Control Framework**: Utilizes a custom hardware interface to replicate the control logic from the ESP32 in the ROS2 environment. This enables a true digital twin, allowing users to simulate motor behavior and control responses in real-time.
- **Feedback Loop**: The motor's encoder provides feedback to the ESP32, which is processed and sent to ROS2. This closed-loop setup ensures data consistency between the physical and simulated environments.

### Suggested Image Placement:
- **Diagram of the control architecture**, illustrating the connection between the ESP32, motor, encoder, and ROS2 digital twin.
- **Flowchart** showing the feedback loop between ROS2 and the ESP32 controller.

## Software Stack
The project uses the following software technologies:
- **ROS2 (Robot Operating System 2)**: Provides tools and libraries for real-time robot control and simulation. Key components include `ros2_control`, `gazebo_ros`, and MoveIt (optional).
- **ESP-IDF Framework/Arduino**: Used for developing and deploying firmware to the ESP32 microcontroller.
- **Gazebo**: A simulation tool for creating and running the digital twin of the motor kit.
- **MoveIt** (Optional): Employed if motion planning and manipulator integration are required.

### 2.3. Communication Protocol
The ROS2 environment and the ESP32 communicate through a serial protocol. The protocol includes:
- **Custom message structure**: For efficient data transmission between ROS2 and the microcontroller.
- **Error handling and synchronization mechanisms**: To ensure reliable communication and data integrity.

### Suggested Image Placement:
- **Annotated screenshot of the ROS2 simulation**, showing the motor's digital twin in Gazebo.
- **Code snippet image** of the serial communication logic for visualization of data structure.

## System Workflow
### Step-by-step Process:
1. **Initialization**:
   - The ESP32 is powered on, and the firmware initializes the control logic.
   - ROS2 nodes are launched, establishing a connection with the hardware interface.

2. **Data Exchange**:
   - The encoder sends position and speed data to the ESP32.
   - The ESP32 processes the data and applies the control logic, adjusting motor output as necessary.
   - The processed data is transmitted to ROS2 through the serial interface, updating the digital twin.

3. **Simulation and Monitoring**:
   - ROS2 simulates the system in Gazebo, providing users with a real-time visualization and data logging.
   - Users can modify control parameters and observe their effects both on the digital twin and the actual motor.

### Suggested Image Placement:
- **Flow diagram** depicting the data flow from initialization to simulation and monitoring.
- **Photo of the physical setup** of the motor kit connected to the ESP32 and a computer running ROS2.

---

# 3. Creating the URDF from CAD

## 3.1. CAD Model Creation in Fusion 360
To generate the URDF for the Encoded DC Motor Kit, the CAD model must first be created in **Fusion 360**. The process involves modeling or importing the components of the motor, encoder, and any associated mechanical structures. Ensure that the model is well-organized into separate components to facilitate joint definitions and correct hierarchy.

### Recommended Steps:
1. **Model the motor, acrylic support and encoder**: Ensure that each part is defined as a separate component within Fusion 360 for easy identification during the URDF conversion process.
2. **Assign properties**: Apply material properties for accurate mass and inertia calculations.
3. **Name components consistently**: Use logical and descriptive names to avoid confusion when mapping them to URDF links.

### Suggested Image Placement:
- **Screenshot of the CAD model in Fusion 360**, showing labeled parts of the motor and encoder.

## 3.2. Converting CAD to URDF Using the Fusion 360 Plugin
To convert the Fusion 360 model to a URDF file, use the `fusion2urdf` plugin available on GitHub: [fusion2urdf Plugin](https://github.com/syuntoku14/fusion2urdf). This tool automates the process by exporting a `.urdf` file along with any required mesh files.

### Conversion Steps:
1. **Install the Plugin**:
   - Clone the repository or download it as a ZIP.
   - Follow the installation guide provided in the repository to integrate the plugin with Fusion 360.

2. **Prepare the Model**:
   - Ensure the first link in the assembly is named `base_link`.
   - Define joints in Fusion 360 by selecting the child link first, followed by the parent link. This rule is crucial for the plugin to correctly convert joint hierarchies into URDF format.
   - Confirm that all necessary constraints and relationships between components are defined.

3. **Run the Plugin**:
   - Launch the plugin from the Fusion 360 environment.
   - Choose the export options, including the format for the mesh files (e.g., STL or Collada).
   - Review and save the exported URDF and mesh files into your ROS2 workspace.

### Plugin Rules and Best Practices:
- **Link Naming**: The root link must be named `base_link` to ensure proper structure.
- **Joint Definition Order**: Select the child link first and then the parent link when defining joints to prevent conversion errors.
- **Verification**: Review the exported URDF for potential adjustments, such as refining origin placements and ensuring mesh references are correct.

### Suggested Image Placement:
- **Screenshot of the `fusion2urdf` plugin interface** within Fusion 360.
- **Annotated diagram** showing how to select child and parent links when defining joints.

## 3.3. Manual Creation of the URDF
For those who prefer or need more control over the URDF structure, manual creation is also possible. This method requires gathering essential data from Fusion 360 or other CAD tools and writing the URDF file by hand.

### Required Data from CAD:
- **Meshes**: Export the 3D model as mesh files in formats like STL or Collada.
- **Mass and Inertia**: Obtain these parameters directly from Fusion 360’s component properties or calculations.
- **Origin and Orientation**: Use the coordinate data to define the position and orientation of each link and joint.


---

# 4. ROS2 Package Setup

This section explains the purpose of each ROS2 package in the **encoded_dc_motor_kit** project. These packages collectively facilitate the simulation, control, interface communication, and launch management for the motor system.

## 4.1. `encoded_dc_motor_kit_description` Package
### Purpose:
The **`encoded_dc_motor_kit_description`** package provides all the necessary files for defining and visualizing the motor model. It includes URDF (Unified Robot Description Format) and XACRO (XML Macros) files to describe the physical properties of the motor, such as its links, joints, and mesh representations. This package is crucial for visualizing the motor in tools like RViz2 and Gazebo, allowing users to observe the motor's structure and simulate its behavior.

## 4.2. `encoded_dc_motor_kit_control` Package
### Purpose:
The **`encoded_dc_motor_kit_control`** package is responsible for setting up and managing ROS2 controllers required for motor operation. It defines how the motor interacts with the ROS2 control system, configuring controllers for position, velocity, or effort control. This package also facilitates the integration with the Gazebo simulator to enable realistic control simulations. Additionally, the hardware interface is implemented here, allowing communication between the ROS2 control framework and the physical motor via the ESP32 microcontroller.

## 4.3. `encoded_dc_motor_kit_interfaces` Package
### Purpose:
The **`encoded_dc_motor_kit_interfaces`** package defines custom ROS2 message types, services, and actions that are specific to the motor system. These custom messages facilitate specialized communication between ROS2 nodes, supporting more detailed interaction with the motor, such as sending control commands and receiving status updates. This package is essential for creating tailored message structures that meet the project's unique requirements.

## 4.4. `encoded_dc_motor_kit_bringup` Package
### Purpose:
The **`encoded_dc_motor_kit_bringup`** package handles the launch and orchestration of the project, whether running in simulation or interfacing with actual hardware. It includes launch files that manage the startup processes for the system, ensuring all necessary nodes and configurations are correctly initiated. Users can choose to launch the motor in a simulated Gazebo environment or run it with real hardware, making it versatile for both development and deployment.


Each package is designed to support specific aspects of the project, from model visualization and simulation to real-world control and communication.

---

# 5. Hardware Interface Plugin Development

Developing a hardware interface plugin is critical for bridging the control logic in ROS2 with the physical motor system. This section describes the protocol used for communication between ROS2 and the ESP32 microcontroller and outlines the steps for implementing the hardware interface plugin.

## 5.1. Communication Protocol
### Overview:
The communication protocol defines how data is transmitted between the ROS2 system and the ESP32 microcontroller, which controls the geared DC motor with the magnetic encoder. For this project, serial communication is employed due to its simplicity and reliability for real-time control.

### Key Details:
- **Protocol Type**: Serial communication (UART)
- **Data Format**: Structured byte streams for commands and responses
- **Command Types**:
  - **Control Commands**: Commands sent from ROS2 to the ESP32 to adjust motor position, velocity, or effort.
  - **Feedback Data**: Responses from the ESP32 that provide current motor status, including position, speed, and encoder readings.
- **Error Handling**: Basic checks to handle communication errors and ensure data integrity.

## 5.2. Implementation Steps
### Overview:
This subsection outlines the procedure to implement the hardware interface plugin, ensuring that the ESP32's control logic integrates seamlessly with ROS2 control frameworks.

### Steps Involved:
1. **Define a Hardware Interface Class**:
   - Implement the `hardware_interface::SystemInterface` base class in C++.
   - Include functions to configure the hardware, start and stop the controller, and read/write data.

2. **Integrate Serial Communication**:
   - Establish a serial communication link between the plugin and the ESP32.
   - Implement data parsing to send commands and receive motor feedback in real-time.

3. **Register the Plugin**:
   - Ensure the plugin is registered in the `pluginlib` to be recognized by the ROS2 control framework.
   - Define the plugin's entry in `package.xml` and create an XML configuration for plugin declaration.

4. **Test and Validate**:
   - Test the hardware interface with simulated and physical motor setups to validate performance and data integrity.
   - Adjust control parameters to achieve stable and responsive control.

---

This section guides developers in understanding the foundational elements of creating a hardware interface plugin for real-world motor control, covering communication protocols and practical implementation steps.

---
# 6. Summary

The **encoded_dc_motor_kit** project provides a comprehensive framework for the integration and control of a geared DC motor with a magnetic encoder, managed by an ESP32 microcontroller. This documentation outlines the complete workflow from CAD model creation to real-world hardware integration through ROS2.

Key takeaways include:
- **CAD to URDF Conversion**: A step-by-step process for converting CAD models from Fusion 360 to URDF using the `fusion2urdf` plugin or manual methods, enabling accurate simulation and visualization in RViz2 and Gazebo.
- **ROS2 Package Setup**: Detailed explanations of the primary ROS2 packages (`_description`, `_control`, `_interfaces`, and `_bringup`), each serving specific roles such as visualization, controller management, custom communication interfaces, and system launch configuration.
- **Hardware Interface Plugin Development**: A breakdown of how to create a plugin to connect ROS2 control frameworks with the ESP32 microcontroller using serial communication. This section covers the implementation of the plugin, ensuring data integrity and responsiveness in real-time control scenarios.

This documentation serves as a guide to building a robust digital twin of the motor system for simulation and physical operation. By following the outlined steps, developers and educators can create an effective platform for training, research, and prototyping in robotics and automation.
