# ROS2-Python_Car
# 🚗 ROS2 Autonomous Car – Navigation & Obstacle Avoidance

## 📝 Overview
This project is a **ROS2-based autonomous car** implemented in **Python**.  
The car is capable of **navigating to a given target coordinate** while dynamically **avoiding obstacles** detected on its path.  

It combines the fundamentals of **robotics, control systems, and ROS2 middleware** to achieve autonomous navigation in a structured environment.

---

## 🔧 Key Features
- 🛰️ **Goal Navigation**: Move to a specific (x, y) coordinate.  
- 🚧 **Obstacle Avoidance**: Detect and bypass obstacles in real-time.  
- ⚙️ **ROS2 Nodes**: Modular architecture with publishers, subscribers, and services.  
- 📡 **Sensor Integration**: Uses simulated or real sensors (e.g., LiDAR, ultrasonic, camera) for perception.  
- 🔁 **Control Loop**: Real-time decision making and path correction.  

---

## 📂 Project Structure
- `navigation_node.py` → Handles path planning and coordinate following  
- `obstacle_avoidance.py` → Processes sensor data to detect and avoid obstacles  
- `car_controller.py` → Low-level motor commands and velocity control  
- `launch/` → ROS2 launch files for starting the system  

---

## ▶️ How to Run
1. **Source your ROS2 workspace**:
   ```bash
   source install/setup.bash
