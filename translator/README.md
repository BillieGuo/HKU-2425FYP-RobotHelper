# ROS2 Translator Module

## Environment
Python version: 3.10.12
```
$ python --version
Python 3.10.12
```
other packages: 
- openai, dotenv, torch, re, ROS2 (peft, transformer, if using Llama from hf)

- create your own ```.env``` file in the ```configs``` folder
```bash
    # if in fyp_ws
    cd ./src/HKU-2425FYP-RobotHelper/translator/configs
    touch .env
```

Inside the ```.env``` file, includes your ```OPENAI_API_KEY， OPENAI_API_ENDPOINT， OPENAI_API_VERSION， HF_TOKEN```

## Folder for LLM 

Put the LLM you want to use under the folder ./translator/models for your program.

LLM will be setup if the correct path/name is provided in "config.yaml".


## Folder for LLM adapters

Put your LLM corresonding adapter under the folder ./translator/adapters for your program.

Adapter will be merged into LLM if the correct path/name is provided in "config.yaml".


__Note__: Better to create and named the folder exactly the same as the correspongding name of LLM you want to adapt to. The program will automatically generate the path to the name of folder you assign in "config.yaml".

# To run the Translator Module
- Follow the ```README.md``` under the ```fyp_ws/src```
```bash
translator_ready
```
- OR start individuals
```bash
# In fyp_ws
colcon build --packages-select translator
colcon build --packages-select livox_to_laserscan

source install/setup.bash # or .zsh

ros2 run translator master
ros2 run translator socket
```
The socket client can be on any computer that can connect to socket server on the Robot's computer with with: ```host='robot-helper', port=7000```. Please check the ```socket_client.py``` for deatils/example.

# TODO
- [x] env requirement 
    - openai, dotenv, torch, re, ROS2 (peft, transformer, if using Llama from hf)
    - LLM strucutre organization
