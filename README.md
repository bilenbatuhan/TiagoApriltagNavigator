# **Tiago Apriltag Navigator**

## **Project Overview**
This project implements a robotic navigation and object detection system using the Tiago robot and ROS (Robot Operating System). The main task involves navigating through an environment to locate red cubes tagged with AprilTags, identifying their unique IDs, and determining their positions.

---

## **Features**
- **AprilTag Detection**: Identifies specific tags on red cubes using the robot's camera.
- **Navigation in Narrow Spaces**: Implements custom motion control for precise movement in tight areas.
- **Feedback System**: Displays the robot's current status in real-time.
- **Data Transformation**: Converts detected tag positions from the camera frame to the map frame using TF transformations.

---

## **Project Structure**
- **Node A**:
  - Requests AprilTag IDs from the `ids_generator_node`.
  - Sends IDs to Node B and provides feedback on task status.
- **Node B**:
  - Navigates through the environment.
  - Detects AprilTags, processes their positions, and transforms them to the map frame.
  - Sends the final list of cube positions back to Node A.

---

## **Technologies Used**
- **Robot Operating System (ROS Noetic)**:
  - Navigation Stack
  - TF Transformations
  - Sensor Data Processing
- **Programming Language**: C++
- **Simulation Tools**: Gazebo
- **Libraries**:
  - [AprilTag](https://github.com/AprilRobotics/apriltag)
  - [AprilTag_ROS](https://github.com/AprilRobotics/apriltag_ros)

---

## **Video Demonstration**
[Link to Video Demonstration]([https://github.com/AprilRobotics/apriltag_ros](https://github.com/bilenbatuhan/TiagoApriltagNavigator/blob/main/Video/Assignment.mp4))
