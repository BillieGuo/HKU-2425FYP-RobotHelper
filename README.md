## HKU 2425 FYP Project 
This is the code repository of a final year project, whose authors are Guo Bao, Zeng Shaoyu, and Zhang Shengce.

This repository contains four main modules for a language-instructed robot helper: grasp, semantic map, navigation and tranlator. Each module contains one or more ROS2 packages to facilitate the functions.

# Start Sequence
The project requires a specific launch sequence. For the alias command and functions, refer to [bashrc_scripts.sh](./bashrc_scripts.sh)
## Start Remote Servers
1. Semantic Map Server
    ```bash
    ```
2. Robotic Arm Server
    ```bash
    server_image
    server_grasp
    ```
## The on-board computer
1. Realsense camera rgbd and pointcloud
    ```bash
    realsense_up
    ```
2. Interbotix Robotic Arm Control SDK
    ```bash
    arm_control
    ```
3. Arm Manipulator to manipulate robotic arm and broadcast important transforms
    ```bash
    arm_manipulator
    ```
4. Robotic Arm communication with servers to request and get response of the grasp pose
    ```bash
    grasp_request
    grasp_response
    ```
5. Livox MID360 3D LiDAR ROS2 SDK Pointcloud, convert Pointcloud to 2D Laserscan, and broadcast transforms
    ```bash
    livox_up
    ```
6. Navigation 
    ```bash
    navigation_control
    ```
7. Translator(Language Model) with socket server for receiving commands
    ```bash
    translator_ready
    ```
7. Semantic Map socket on robot

All programs needed is now started, to query the robot please conduct the following:

0. LLM client socket on any computer connect to translator socket server with: ```host='robot-helper', port=7000```. Please check the ```socket_client.py``` for deatils/example.