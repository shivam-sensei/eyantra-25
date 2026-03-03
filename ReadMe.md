# Holonomic Robot Control and Perception System  
## e-Yantra Robotics Competition (eYRC) Project  

---

## Overview

This project was developed as part of the **e-Yantra Robotics Competition (eYRC)** conducted by IIT Bombay.

The system implements a complete perception–control pipeline for a holonomic mobile robot operating in a simulated **ROS 2** environment. It integrates real-time pose estimation, closed-loop motion control, and structured task execution.

The objective was to design a robust, modular, and scalable robotic software architecture capable of handling dynamic task requirements in simulation.

---

## System Architecture

The project follows a modular **ROS 2 node-based architecture** consisting of:

### 1. Perception Module

- Detects environmental features  
- Estimates robot pose  
- Processes sensor/image data  
- Publishes pose information for control layer  

### 2. Holonomic Controller

- Implements inverse kinematics for omnidirectional motion  
- Generates velocity commands  
- Uses feedback-based correction  
- Ensures smooth and accurate navigation  

### 3. Simulation Control Layer

- Manages task logic  
- Coordinates perception and control modules  
- Handles execution flow and state transitions  

---

## Tech Stack

- ROS 2  
- Python  
- OpenCV  
- Gazebo Simulation  
- Colcon Build System  
- Custom Control Algorithms  

---

## Key Features

- Real-time pose estimation  
- Closed-loop holonomic motion control  
- Modular ROS 2 node structure  
- Integrated perception–control pipeline  
- Simulation-based validation  
- Structured and scalable architecture  


## Demonstration

### Theme Run Video

[Watch the Theme Run](https://www.youtube.com/watch?v=1uYSSPXg_bE)

---

## Authors

Shivam Gupta  
Tanishk Singhal  
Sanidhya Jain  
Aditya Singh  

**Maharaja Agrasen Institute of Technology**

---

## License

This project was developed for academic and competition purposes under e-Yantra guidelines.
