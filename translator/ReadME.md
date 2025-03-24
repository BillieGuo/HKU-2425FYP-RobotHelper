
# Folder for LLM 

Put the LLM you want to use under the folder ./translator/models for your program.

LLM will be setup if the correct path/name is provided in "config.yaml".


# Folder for LLM adapters

Put your LLM corresonding adapter under the folder ./translator/adapters for your program.

Adapter will be merged into LLM if the correct path/name is provided in "config.yaml".


__Note__: Better to create and named the folder exactly the same as the correspongding name of LLM you want to adapt to. The program will automatically generate the path to the name of folder you assign in "config.yaml".