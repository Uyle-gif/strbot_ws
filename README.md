# STR Robot: Setup and Build Guide

[Project Website](https://ntdathp.github.io/outdoor-robot-web/)   
[arXiv Paper](https://arxiv.org/abs/2605.28110)

## 1. Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Livox ROS Driver 2
- Micro-ROS Agent
- FAST-LIO / FAST-LIVO
- Nav2-based navigation stack

Install basic dependencies:

```bash
sudo apt update
sudo apt install -y \
    libpcl-dev \
    libeigen3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    git

```

Source ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash

```

---

## 2. USB Device Setup

Create udev rule for STM32:

```bash
sudo nano /etc/udev/rules.d/99-ttbot.rules

```

Paste the following content:

```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttbot_stm32", MODE="0666"

```

Apply udev rules:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger

```

Check device:

```bash
ls /dev/ttbot_stm32

```

---

## 3. Build Livox ROS Driver 2

```bash
cd ~/strbot_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
source install/setup.bash

```

---

## 4. Build STR Robot Workspace

```bash
cd ~/strbot_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

```

---

## 5. Micro-ROS Agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttbot_stm32

```

---

## 6. Simulation

Run each command in a separate terminal.

### Terminal 1: Robot Bringup

```bash
cd ~/strbot_ws
source install/setup.bash
ros2 launch ttbot_bringup sim.launch.py

```

### Terminal 2: FAST-LIO Simulation

```bash
cd ~/strbot_ws
source install/setup.bash
ros2 launch fast_lio fast_lio_sim.launch.py

```

### Terminal 3: Navigation Simulation

```bash
cd ~/strbot_ws
source install/setup.bash
ros2 launch ttbot_navigation navigation_sim.launch.py

```

---

## 7. Real-World Deployment

Run the real robot system in the following order:

1. Launch Micro-ROS Agent.
2. Launch real-robot bringup.
3. Launch FAST-LIO / FAST-LIVO in real mode.
4. Launch the navigation stack with:

```bash
use_sim_time:=false

```

Before running navigation, check the map used by the Navigation map server.

---

# Experimental Results

Physical experiments were conducted using a custom Ackermann-steered mobile robot.

Physical Robot
<p align="center">
  <img src="https://github.com/user-attachments/assets/1e197d75-f146-46c9-97ed-553a42c45d5f" width="60%" />
</p>

## Real-World Path-Tracking Performance
The proposed architecture was deployed on the physical robot to verify sim-to-real consistency. A-GMPC consistently achieved lower RMSE than the standard MPC for all tested trajectories and speeds.

* **At 0.6 m/s Average Speed:** A-GMPC RMSE was 0.2349 m (Lemniscate) and 0.2588 m (Square); Standard MPC RMSE was 0.3194 m (Lemniscate) and 0.4260 m (Square).
* **At 1.1 m/s Average Speed:** A-GMPC RMSE was 0.3048 m (Lemniscate) and 0.2777 m (Square); Standard MPC RMSE was 0.3281 m (Lemniscate) and 0.4413 m (Square).
* **At 1.5 m/s Average Speed:** A-GMPC RMSE was 0.3582 m (Lemniscate) and 0.6182 m (Square); Standard MPC RMSE was 0.5149 m (Lemniscate) and 1.1475 m (Square).

Both controllers executed well within the 30 Hz (33.3 ms) control loop.

Runtime Statistics
<p align="center">
  <img src="https://github.com/user-attachments/assets/d3463c61-63e3-45b9-9ad2-0bcfde8e1eca" width="500">
</p>

Real-World Path Tracking
<p align="center">
  <img src="https://github.com/user-attachments/assets/9205a880-3b4e-4405-b985-ba4c91043992" width="40%" />
      &nbsp;&nbsp;
  <img src="https://github.com/user-attachments/assets/558b6e6d-5d3c-4ef8-aa3d-c47cd8750556" width="40%" />
</p>


**Video Demonstration:** https://youtu.be/eXZOD7MUVX8.

## Autonomous Navigation Demonstration
An end-to-end real-world navigation experiment validated the complete autonomy pipeline, successfully integrating onboard perception, mapping, path planning, and trajectory tracking. The system achieved smooth tracking behavior with a low tracking error of RMSE = 0.2624 m over a reference path length of 114.0290 m.

Environmental Mapping
<p align="center">
  <img src="https://github.com/user-attachments/assets/24dcb81f-80fd-4c67-9c73-f95225365dfe" width="40%" />
  &nbsp;&nbsp;
  <img src="https://github.com/user-attachments/assets/ad11d657-691d-48af-a0a0-e659c9392087" width="40%" />
</p>


Trajectory Tracking Performance
<p align="center">
  <img src="https://github.com/user-attachments/assets/53598319-b269-405a-83ea-6d74d662c2d0" width="50%" />
</p>

**Video Demonstration:** https://youtu.be/2TXuBDscRR4.

---

# Sub-repositories

The core development stack of the STR Robot is modularized into the following specialized sub-repositories:

* **Hardware (PCB Design):** https://github.com/Nvinh5148/STR_PCB.git
* **Firmware (micro-ROS Workspace):** https://github.com/Nvinh5148/microros_ws.git

