## HKU 2425 FYP Project 
This is the code repository of a final year project, whose authors are Guo Bao, Zeng Shaoyu, and Zhang Shengce.

This repository contains four main modules for a language-instructed robot helper: grasp, semantic map, navigation and tranlator. Each module contains one or more ROS2 packages to facilitate the functions.

# Start Sequence
The project requires a specific launch sequence:
1. Semantic Map on remote server
2. Navigation and Arm control on robot (TF to be exact)
3. Semantic Map socket on robot

All programs needed is now started, to query the robot please conduct the following:
0. LLM client socket on any computer