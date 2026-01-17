# ROS2 Workspace - Dual LiDAR Robot

## Cài đặt Dependencies

```bash
# ROS2 packages cần thiết
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-robot-localization ros-humble-tf2-ros ros-humble-tf2-tools ros-humble-rviz2 ros-humble-serial-driver

# Clone RPLidar driver
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Cấu hình serial port
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
sudo chmod 666 /dev/ttyUSB2
```

## Launch Commands

```bash
# Source workspace (chạy trước mỗi lần dùng)
source ~/ros2_ws/install/setup.bash

# Launch dual LiDAR + merge
ros2 launch my_robot_config dual_lidar_with_merge.launch.py

# Launch odometry (encoder + IMU)
ros2 launch my_robot_config encoder_imu_odom.launch.py

# Launch SLAM với 2 LiDAR
ros2 launch my_robot_config slam_2lidar.launch.py

# Launch localization (AMCL)
ros2 launch my_robot_config bringup_localization.launch.py
# chạy lại amcl
ros2 set params /amcl_deactive
ros2 set params /amcl_active
# Launch navigation
ros2 launch my_robot_config bringup_navigation.launch.py
```

## Lệnh Debug

```bash
# Kiểm tra topics
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /imu/data

# Kiểm tra TF
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link laser_front

# Mở RViz
rviz2

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/my_map

# Build lại package
colcon build --packages-select my_robot_config --symlink-install
```

## Troubleshooting

```bash
# Lỗi serial port
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0

# Clean build
rm -rf build/ install/ log/
colcon build --symlink-install
```
