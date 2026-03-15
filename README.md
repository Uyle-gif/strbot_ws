### 2. OSQP
```bash
cd ~
git clone https://github.com/osqp/osqp.git
cd osqp
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
### 3. Eigen
```bash
cd ~
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
mkdir build && cd build
cmake ..
sudo make install
```


### 4. USB Device Setup (Udev Rules)
```bash
sudo nano /etc/udev/rules.d/99-ttbot.rules
```
Copy and paste the following content into the file:
```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="ttbot_imu", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttbot_stm32", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", SYMLINK+="ttbot_gps", MODE="0666"
```
Apply and verify
``` bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

``` bash
sudo apt update
sudo apt install ros-humble-nmea-navsat-driver -y
sudo apt install ros-humble-ublox-serialization -y

sudo apt install python3-pip -y
pip3 install transforms3d pyserial
pip3 install "numpy==1.23.5"
```

### 5. IMU config (1 on 2 on)
```bash
stty -F /dev/ttbot_imu 460800 cs8 -cstopb -parenb -echo
echo -e '$DATOP 0000\r' > /dev/ttbot_imu
echo -e '$DATOP 1111\r' > /dev/ttbot_imu
echo -e '$BRATE 3\r' > /dev/ttbot_imu
```


## ⚙️ Build Instructions (ROS2 Humble)

### 1. Source ROS2 
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
source /opt/ros/humble/setup.bash
```

### 2. Build & source the workspace 
```bash
cd ~/Desktop/ttbot_ws
colcon build 
source install/setup.bash
```
### 3. Run the full system (no path_pub, no rviz, no joy, no qgc and run mpc) or full option
```bash
ros2 launch ttbot_bringup sim.launch.py
```
```bash
ros2 launch ttbot_bringup sim.launch.py run_path:=true run_rviz:=true run_qgc:=true controller_type:=stanley run_joy:=true 
```

### 6. Pub path for MPC
```bash
ros2 run ttbot_controller path_publisher --ros-args -p path_file:=path_u_to_S.csv
```
### 7. Run imu node 
```bash
sudo usermod -aG dialout $USER

logout then login again 
ros2 run adis16488_driver adis16488_node --ros-args -p port:=/dev/ttbot_imu -p baudrate:=460800
```

source /home/vinh/microros_ws/install/local_setup.bash

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0




## REAL
### B1. Check connection 
```bash
ls -l /dev/ttbot_* 
```
must have ttbot_gps, ttbot_imu, ttbot_stm32
### B2. Sensors check
no run_path, joy
```bash
ros2 launch ttbot_bringup real.launch.py
```

```bash
ros2 topic echo /gps/fix
```
status.status > 0 


```bash
ros2 topic echo /imu/data_filtered
```
orientation change


```bash
ros2 topic echo /ackermann_controller/odom
```
pose.position change
twist.linear.x change 

### B3. Manual Control 
```bash
ros2 launch ttbot_bringup real.launch.py run_joy:=true
```
### B4. Localization 
```bash
ros2 run rviz2 rviz2
```
Lower the robot onto the ground and manually guide it in a straight line for about 5–10 meters.
In RViz, the red arrow (Robot) should move smoothly and follow exactly the direction you are guiding it.
If the robot on the screen drifts or jumps around erratically, then the EKF needs to be retuned (ekf_global.yaml).

```bash
	ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttbot_stm32
```


### B4. Auto

(no path_pub, no rviz, no joy, no qgc and run mpc)
```bash
ros2 launch ttbot_bringup real.launch.py
```
full option 
```bash
ros2 launch ttbot_bringup real.launch.py run_path:=true run_joy:=true run_qgc:=true controller_type:=stanley
```

# Data Logging & Analysis
## 1. Rosbag
Chạy lệnh này trên terminal của Robot trước khi cho xe chạy
```bash
mkdir -p ~/bag_files
cd ~/bag_files

# Ghi lại các topic quan trọng
ros2 bag record -a -o full_data_test

Cho xe chạy hết đường (Path). Sau khi xe dừng, quay lại terminal bấm Ctrl+C để lưu file.
## 2. updating....




# LOCALIZATION CHECK
## 1. CHECK SENSOR (sensor nguon)
hz > 0
```bash
ros2 topic hz /gps/fix
ros2 topic hz /imu/data_filtered
ros2 topic hz /ackermann_controller/odom
```

## 2. CHECK TANG 1 (ekf local)
```bash
ros2 topic hz /odometry/filtered
```
## 3. CHECK TANG 2
```bash
ros2 topic echo --qos-reliability best_effort /odometry/gps
```
## 4. CHECK TF
```bash
ros2 run tf2_ros tf2_echo map base_link
```
(Nếu hiện Translation/Rotation -> Hệ thống định vị đã thành công)


ros2 launch fast_lio mapping.launch.py config_file:=velodyne_sim.yaml use_sim_time:=true

ros2 launch slam_toolbox online_async_launch.py \
slam_params_file:=/home/uylegia/ttbot_ws/src/ttbot_mapping/config/mapper_params.yaml \
use_sim_time:=True


nano ~/.bashrc

pcl_viewer /home/vinh/ttbot_ws/src/FAST-LIVO2/Log/PCD/all_raw_points.pcd


powerprofilesctl set performance

python3 record_tracking.py --prefix mpc