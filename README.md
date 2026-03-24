### OSQP, EIGEN
### USB Device Setup (Udev Rules)
```bash
sudo nano /etc/udev/rules.d/99-ttbot.rules
```
Copy and paste the following content into the file:
```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttbot_stm32", MODE="0666"
```
Apply and verify
``` bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

<!-- ### IMU config (1 on 2 on)
```bash
stty -F /dev/ttbot_imu 460800 cs8 -cstopb -parenb -echo
echo -e '$DATOP 0000\r' > /dev/ttbot_imu
echo -e '$DATOP 1111\r' > /dev/ttbot_imu
echo -e '$BRATE 3\r' > /dev/ttbot_imu
``` -->


### Micro ROS-Agent
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttbot_stm32
```

### Simulation
1. Launch the simulation bringup.
2. Launch FAST-LIO/FAST-LIVO in sim mode.
3. Launch the navigation stack.

### Real-World
1. Launch the real-robot bringup.
2. Launch FAST-LIO/FAST-LIVO on real mode
3. Launch the navigation stack. (use_sim_time:=false) 
    NOTE: Check the map used by the Navigation map server
