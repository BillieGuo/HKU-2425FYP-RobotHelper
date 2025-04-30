## HKU 2425 FYP Project 
This is the code repository of a final year project, whose authors are Guo Bao, Zeng Shaoyu, and Zhang Shengce.

This repository contains four main modules for a language-instructed robot helper: grasp, semantic map, navigation and tranlator. Each module contains one or more ROS2 packages to facilitate the functions.

# Start Sequence
The project requires a specific launch sequence. For the alias command and functions, refer to [bashrc_scripts.sh](./bashrc_scripts.sh)
## Robotic Arm Server
1. Robotic Arm Server
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
8. Semantic Map socket on robot
   ```bash
    ros2 run semantic_map image_transform_listener
    ```
## Semantic Map Server
1. Semantic Map Server
    ```bash
    cd llmbot2_ws
    ./start_semantic_map.sh
    # Wait until the tmux sessions are listed
    # in another terminal, to enable gdino explore
    ros2 run sem_map gdino_query_client
    ```
## The on-board computer
9. Semantic Map Query
    ```bash
    ros2 run semantic_map query_semantic_map
    ```

All programs needed is now started, to query the robot please conduct the following:

0. LLM client socket on any computer connect to translator socket server with: ```host='robot-helper', port=7000```. Please check the ```socket_client.py``` for deatils/example.
