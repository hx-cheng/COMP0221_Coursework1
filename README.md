# _COMP0221_Coursework1_

## Folder contents

The project contains two source file in C language [main.c](main/main.c) and [attack.c](main/attack.c). The files are located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). Switch between normal ditributed flocking control system and attack 
system by manually change the path in CMakeLists.txt. 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── README.md                This is the file you are currently reading
├── dependencies.lock
├── .gitignore
├── .clangd
├── core/
│   ├── EspHal.h
│   ├── compute_cmac.c
│   ├── compute_cmac.h
│   ├── flocking_control.c
│   ├── flocking_control.h
│   ├── monitor_task.c
│   ├── monitor_task.h
│   ├── neighbour_table.cpp
│   ├── neighbour_table.h
│   ├── physics_integration.c
│   ├── physics_integration.h
│   ├── telemetry.c
│   └── telemetry.h
├── main/
│   ├── CMakeLists.txt
|   ├── idf_component.yml
|   ├── config.h
│   ├── main.c
│   └── attack.c
├── mac_dict.py
└── visualiser.py
```
