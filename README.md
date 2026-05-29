
# STR Robot: Setup and Build Guide

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
cd ~/STR_Robot/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
source install/setup.bash

```

---

## 4. Build STR Robot Workspace

```bash
cd ~/STR_Robot
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
cd ~/STR_Robot
source install/setup.bash
ros2 launch ttbot_bringup sim.launch.py

```

### Terminal 2: FAST-LIO Simulation

```bash
cd ~/STR_Robot
source install/setup.bash
ros2 launch fast_lio fast_lio_sim.launch.py

```

### Terminal 3: Navigation Simulation

```bash
cd ~/STR_Robot
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

![Physical Robot](path/to/fig7.png)
<img width="1448" height="1086" alt="c4_car" src="https://github.com/user-attachments/assets/1e197d75-f146-46c9-97ed-553a42c45d5f" />


## Real-World Path-Tracking Performance
The proposed architecture was deployed on the physical robot to verify sim-to-real consistency. A-GMPC consistently achieved lower RMSE than the standard MPC for all tested trajectories and speeds.

* **At 0.6 m/s Average Speed:** A-GMPC RMSE was 0.2349 m (Lemniscate) and 0.2588 m (Square); Standard MPC RMSE was 0.3194 m (Lemniscate) and 0.4260 m (Square).
* **At 1.1 m/s Average Speed:** A-GMPC RMSE was 0.3048 m (Lemniscate) and 0.2777 m (Square); Standard MPC RMSE was 0.3281 m (Lemniscate) and 0.4413 m (Square).
* **At 1.5 m/s Average Speed:** A-GMPC RMSE was 0.3582 m (Lemniscate) and 0.6182 m (Square); Standard MPC RMSE was 0.5149 m (Lemniscate) and 1.1475 m (Square).

Both controllers executed well within the 30 Hz (33.3 ms) control loop.

![Runtime Statistics](path/to/fig8.png)
[solvetime.pdf](https://github.com/user-attachments/files/28376093/solvetime.pdf)


![Real-World Path Tracking](path/to/fig5_cd.png)
[square_path_tracking.pdf](https://github.com/user-attachments/files/28376101/square_path_tracking.pdf)
[8_path_tracking.pdf](https://github.com/user-attachments/files/28376100/8_path_tracking.pdf)


**Video Demonstration:** https://youtu.be/eXZOD7MUVX8.

## Autonomous Navigation Demonstration
An end-to-end real-world navigation experiment validated the complete autonomy pipeline, successfully integrating onboard perception, mapping, path planning, and trajectory tracking. The system achieved smooth tracking behavior with a low tracking error of RMSE = 0.2624 m over a reference path length of 114.0290 m.

![Environmental Mapping](path/to/fig9.png)
<img width="1475" height="1066" alt="c4_ggmap" src="https://github.com/user-attachments/assets/24dcb81f-80fd-4c67-9c73-f95225365dfe" />
<img width="1411" height="1114" alt="c4_map_3d_path" src="https://github.com/user-attachments/assets/ad11d657-691d-48af-a0a0-e659c9392087" />


![Trajectory Tracking Performance](path/to/fig10.png)
<img width="7196" height="7128" alt="c4_end2end" src="https://github.com/user-attachments/assets/53598319-b269-405a-83ea-6d74d662c2d0" />


**Video Demonstration:** https://youtu.be/2TXuBDscRR4.

---

# Sub-repositories

The core development stack of the STR Robot is modularized into the following specialized sub-repositories:

* **Hardware (PCB Design):** https://github.com/Nvinh5148/STR_PCB.git
* **Firmware (micro-ROS Workspace):** https://github.com/Nvinh5148/microros_ws.git

