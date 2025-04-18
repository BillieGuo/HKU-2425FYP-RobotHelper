# ROS2 Navigation Module

Python version: 3.10.12
```
$ python --version
Python 3.10.12
```

## Install and Config Livox SDK2

[Official Repo](https://github.com/Livox-SDK/Livox-SDK2)

1. install cmake
    ```bash
        sudo apt install cmake
    ```
2. install and build Livox SDK2
    ```bash
        cd ~/fyp_ws
        git clone https://github.com/Livox-SDK/Livox-SDK2.git
        cd ./Livox-SDK2/
        mkdir build
        cd build
        cmake .. && make -j
        sudo make install
    ```

## Install livox_ros_driver2

[Official Repo](https://github.com/Livox-SDK/livox_ros2_driver)

Note: is **livox_ros_driver2** not livox_ros2_driver

1. **Get the repo source code**
    ```bash
        cd ~/fyp_ws/src
        git clone https://github.com/Livox-SDK/livox_ros_driver2.git
    ```

2. **Build**
    ```bash
        source /opt/ros/{ROS2_VERSION}/setup.sh
        cd ~/fyp_ws/src/livox_ros_driver2
        ./build.sh {ROS2_VERSION}
    ```
    For Humble, also our project ROS2 version:
    ```bash
        source /opt/ros/humble/setup.sh
        cd ~/fyp_ws/src/livox_ros_driver2
        ./build.sh humble
    ```

3. **Config json file**

- To start the corresponding(correct) LiDAR, go to ```MID360_config``` under ```livox_ros_driver2/config/```

- Change the ```host_net_info: *ip:``` and ```lidar_configs: ip: ``` with the S/N pin on the your LiDAR and your computer's net ip

- Go to ```msg_MID360_launch.py``` or ```rviz_MID360_launch.py``` under ```livox_ros_driver2/launch_ROS2/```

- Change the ```cmdline_bd_code``` with the S/N pin of the LiDAR

4. **Run Livox ROS Driver 2**
    ```bash
        ros2 launch livox_ros_driver2 [launch file]
    ```
In this project would be:
    ```bash
        ros2 launch livox_ros_driver2 msg_MID360.launch
    ```

## Install Fast_LIO2

[Official Repo](https://github.com/hku-mars/FAST_LIO/tree/ROS2)

1. **[Prerequisites](https://github.com/hku-mars/FAST_LIO/tree/ROS2)**

    - Ubuntu and ROS

        ROS >= Foxy (Recommend to use ROS-Humble). [ROS Installation](https://docs.ros.org/en/humble/Installation.html)

    - PCL && Eigen

        PCL    >= 1.8,   Follow [PCL Installation](https://pointclouds.org/downloads/#linux).

        Eigen  >= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

    - livox_ros_driver2

        Follow [livox_ros_driver2 Installation](https://github.com/Livox-SDK/livox_ros_driver2).

2. **Build**

    ```bash
    cd <ros2_ws>/src # cd into a ros2 workspace folder
    git clone https://github.com/Ericsii/FAST_LIO.git --recursive
    cd ..
    rosdep install --from-paths src --ignore-src -y
    colcon build --symlink-install
    . ./install/setup.bash # use setup.zsh if use zsh
    ```

3. Run
    ```bash
    cd <ros2_ws>
    . install/setup.bash # use setup.zsh if use zsh
    ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml
    ```

## Nav2
[Official website](https://docs.nav2.org/getting_started/index.html)

[Official Repo](https://github.com/ros-navigation/navigation2)

In this project, I follow the website installation. 

- Run (General)
```bash
ros2 launch nav2_bringup navigation_launch.py
```

## SLAM_Toolbox
[Nav2 Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)

[Official Repo](https://github.com/SteveMacenski/slam_toolbox)

In this project, I follow the website installation.

- Run (General)
```bash
ros2 launch slam_toolbox online_async_launch.py
```

# To Run the Navigation Module
- Follow the ```README.md``` under the ```fyp_ws/src```
```bash
navigation_control
```
- OR start individuals
```bash
# In fyp_ws
colcon build --packages-select nav_control_hub
colcon build --packages-select livox_to_laserscan

source install/setup.bash # or .zsh

ros2 launch livox_ros_driver2 msg_MID360_launch.py # livox mid360
ros2 run livox_to_laserscan livox_to_laserscan # 3d pl to 2d scan
ros2 run livox_to_laserscan custom_tf_pub # static tf

ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml rviz:=false # Fast-lio2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false # nav2 Navigation
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false # nav2 SLAM
ros2 run nav_control_hub serial # serial node to chassis 
ros2 run nav_control_hub control # 
ros2 run nav_control_hub explore
```

# To-Do

- [x] livox dependencies
- [ ] fast-lio submodule
- [ ] serial task explanation
