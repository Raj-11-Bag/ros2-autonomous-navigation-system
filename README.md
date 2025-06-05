
# 🧭 ROS 2 Action-Based Navigation System

A high-level, modular, and extensible implementation of a **ROS 2 Action server-client framework** that enables robot navigation to 3D coordinate targets with real-time feedback. This project demonstrates advanced concepts in ROS 2 such as custom actions, asynchronous communication, and real-time telemetry tracking.

---

## 📌 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [Installation](#installation)
- [Usage](#usage)
- [Custom Action Definition](#custom-action-definition)
- [Code Walkthrough](#code-walkthrough)
- [Build and Run](#build-and-run)
- [License](#license)
- [Acknowledgments](#acknowledgments)

---

## 🧠 Overview

This package provides a complete simulation pipeline for robot navigation using ROS 2 Actions. It consists of an **Action Server** that listens for target coordinates and provides real-time feedback, and an **Action Client** that sends the goal and handles the server’s response. The system is designed with extensibility and clarity in mind, perfect for robotics education or foundational development.

---

## ✨ Features

- 🔧 **Custom ROS 2 Action** for navigation (`Navigate.action`)
- 📡 **Real-time Feedback** on distance to goal
- ⏱️ **Elapsed Time Measurement** for complete navigation cycle
- 🧩 **Seamless Integration** with `geometry_msgs` and ROS 2 tooling
- 🧵 **Multithreaded Event Handling** via asynchronous callbacks
- 🧪 **Simple CLI Interface** for testing client functionality

---

## 🧱 Architecture

```plaintext
+----------------+        Send Goal         +------------------+
| Action Client  |  --------------------->  |  Action Server   |
| (User Input)   |                         | (Goal Processing) |
+----------------+                         +------------------+
        ^                                           |
        |                                           v
        |   Result / Feedback Loop (distance)   +--------+
        +-------------------------------------- | Robot  |
                                                +--------+
```

- The **Action Client** collects goal input and communicates with the server.
- The **Action Server** processes goals and listens for the robot’s current position.
- Robot position is subscribed via `geometry_msgs/Point` topic.

---

## 🔧 Installation

### Requirements

- ROS 2 (Foxy, Galactic, Humble or later)
- Python 3.8+
- Dependencies: `rclpy`, `geometry_msgs`, `action_msgs`, `rosidl_default_generators`

### Build the Package

```bash
cd <your_ros2_ws>/src
git clone <this-repo>
cd ..
colcon build
source install/setup.bash
```

---

## 🚀 Usage

### 1. Start the Action Server

```bash
ros2 run ros2_pkg action_server
```

### 2. Start the Action Client

```bash
ros2 run ros2_pkg action_client
```

### 3. Simulate Robot Position Publisher

Ensure another node publishes to `/robot_position` as `geometry_msgs/Point`.

```bash
ros2 topic pub /robot_position geometry_msgs/Point "{x: 1.0, y: 2.0, z: 0.0}"
```

---

## 📝 Custom Action Definition

Located in `Navigate.action`:

```action
# Goal
geometry_msgs/Point goal_point
---
# Result
float32 elapsed_time
---
# Feedback
float32 distance_to_point
```

---

## 🔍 Code Walkthrough

### 🧠 Action Server (`action_server.py`)

- Initializes an `ActionServer` node on topic `/navigate`
- Subscribes to `/robot_position`
- Continuously checks distance to goal until threshold met
- Publishes feedback and returns elapsed time as result

### 📤 Action Client (`action_client.py`)

- Collects goal coordinates from user input
- Sends goal to server asynchronously
- Handles feedback and prints final result

### 🧪 Example Output

```plaintext
Action Client is Running...
Enter a X Coordinate: 3.0
Enter a Y Coordinate: 2.5
Enter a Z Coordinate: 0.0
Received Feedback: 1.23
Received Feedback: 0.45
Result: 5.42 seconds
```

---

## 🛠️ Build and Run

```bash
colcon build
source install/setup.bash

# In separate terminals
ros2 run ros2_pkg action_server
ros2 run ros2_pkg action_client
```

---

## 📄 License

This project is currently under placeholder license. Update `package.xml` with your license of choice (e.g., MIT, Apache 2.0).

---

## 🙌 Acknowledgments

Built as part of an educational project on ROS 2 Action communication systems. Inspired by ROS 2 best practices and modular robotics design.

---
