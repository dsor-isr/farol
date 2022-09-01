```

Tinkers: DSOR Group
         malaclemys  -> jquintas@gmail.com

Description: Scripts for creating new packages according the farol stack best practices
Date: 20200226

@~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~@
@         ?                                               @
@          __                                             @
@      ____||_                  ___                       @
@     (_______)            ,,  // \\                      @  
@      ____||_            (_,\/ \_/ \                     @
@     (_______)             \ \_/_\_/>                    @
@                            /_/  /_/         ______      @
@                                            |START |     @
@                                            |______|     @
@                                            |            @
@____________________________________________|____________@
```

## Scripts

| File | Usage via Alias | Purpose|
|:----------:|:----------|:----:|:----:|
| **farol_create_ros_pkg_cpp.sh** |  farol_pkg_cpp my_awesome_package  | c++ package |
| **farol_create_ros_pkg_cpp.sh** |  farol_pkg_py my_awesome_package  | python package |
| **farol_create_ros_pkg_cpp.sh** |  farol_pkg_meta my_awesome_package  | meta package |

## Example fiic_awesome package (meta, c++, python)

### What you get

After running the following commands in terminal (please follow the same order):
```
farol_pkg_meta fiic_awesome 
farol_pkg_cpp fiic_awesome_cpp 
cd ..
farol_pkg_py fiic_awesome_py
cd ../../
tree
```

After running the commands successfully you will get the following tree structure:
```
.
├── fiic_awesome
│   ├── CMakeLists.txt
│   ├── doc
│   │   └── README.md
│   └── package.xml
│
│
├── fiic_awesome_cpp
│   ├── CMakeLists.txt
│   ├── config
│   │   └── config_fiic_awesome_cpp.yaml
│   ├── doc
│   │   └── README.md
│   ├── include
│   │   ├── fiic_awesome_cpp_algorithms
│   │   │   └── FiicAwesomeCppAlgorithm.h
│   │   └── fiic_awesome_cpp_ros
│   │       └── FiicAwesomeCppNode.h
│   ├── launch
│   │   └── fiic_awesome_cpp.launch
│   ├── package.xml
│   ├── scripts
│   │   └── fiicAwesomeCppScript
│   ├── src
│   │   ├── fiic_awesome_cpp_algorithms
│   │   │   └── FiicAwesomeCppAlgorithm.cpp
│   │   └── fiic_awesome_cpp_ros
│   │       └── FiicAwesomeCppNode.cpp
│   └── test
│       └── fiic_awesome_cpp_test.cpp
│
│
└── fiic_awesome_py
    ├── CMakeLists.txt
    ├── config
    │   └── config_fiic_awesome_py.yaml
    ├── doc
    │   └── README.md
    ├── launch
    │   └── fiic_awesome_py.launch
    ├── package.xml
    ├── scripts
    │   └── fiic_awesome_py_node
    ├── setup.py
    ├── src
    │   ├── fiic_awesome_py_algorithms
    │   │   ├── FiicAwesomePyAlgorithm.py
    │   │   ├── FiicAwesomePyAlgorithm.pyc
    │   │   ├── __init__.py
    │   │   └── __init__.pyc
    │   └── fiic_awesome_py_ros
    │       ├── FiicAwesomePyNode.py
    │       ├── FiicAwesomePyNode.pyc
    │       └── __init__.py
    └── test
        └── fiic_awesome_py_test.py

23 directories, 29 files
```

## Tiny Details

### In all scripts
| Feature | Description  
|:----------:|:----------:|
| Package folders names | should always be lower case and separated by underscore. Even if you introduce capital letters the script will make them lowercase. The underscore is up to you to follow it.| 
| Ros Code part | In dedicated folders ending with **_ros** and files ending with **Node**.|
| Algorithm Code part | **YES** we want to make the algorithm part **standalone**. The idea is to have cleaner code and also make possible to use the algorithms in another middleware. In dedicated folders ending with **_algorithms** and files ending with **Algorithm**|
| Ros Nodes | Are implemented as **classes** and use **Timer calbacks** instead of the classical ros::Rate/while loop/ros_sleep/ rosspin|
| Class names | Are like **MyClassName** first letter uppercase and the rest camelcase|
| Function Names | Regular camelcase: **myFunctionName** |
| variables | lowercase and separated by underscores: **my_variable_example**|
| Parameters from parameter server | start with **p_** followed by parameter name: **p_my_super_parameter**|
| Files that have classes | Will have the same name as the class|
|...| Probably more will come| 

### farol_pkg_cpp fiic_awesome
| Creates | Description | Details | 
|:----------:|:----------|:----:|
| **fiic_awesome_cpp** | Folder with cpp node | Everything ready.|
| **CMakeLists.txt** | CMake for ros | c++11 ready: **add_compile_options(-std=c++11)**; executable: **fiic_awesome_cpp_node** -> package_name+**_node** |
|**doc/Readme.md** | Folder with a Readme file for documentation | Empty, please use it|
|  **package.xml** | Simple package manifest | Already format 2. As dependencies has std_msgs, rospy and farol_msgs by default |
| **launch** | launch file for the node | Has xml header, thank god. Also it has an example with what you should had to the file and alredy configured to be launched. |
| **scripts** | Useful for python, maybe handy for the future | Empty at this stage for c++ code. |
| **src** | Where the code lives | ROS and Algorithms code separated in two different folders. Configure **CMakeLists.txt** properly, but in doubt see what is done and follow the recipe.
| **test** | Isolated code to test the node | Empty now, maybe in a future|
| **config** |  Has yaml file where you can add parameters to be loaded into the parameter server | just node_frequency, feel free to add more params|
| **include** |  Declarations (.h files) of your code. | ROS and Algorithms declarations separated in two different folders.|

### farol_pkg_py fiic_awesome
| Creates | Description | Details | 
|:----------:|:----------|:----:|
| **fiic_awesome_py** | Folder with python node | Everything ready.|
| **CMakeLists.txt** | CMake for ros | has setup configuration: **catkin_python_setup()**;  |
|**doc/Readme.md** | Folder with a Readme file for documentation | Empty, please use it|
|  **package.xml** | simple package manifest | Already format 2. As dependencies has std_msgs, rospy and farol_msgs by default |
| **launch** | launch file for the node | Has xml header, thank god. Also it has an example with what you should had to the file and alredy configured to be launched. |
| **scripts** | It will be your node executable | Executable: **fiic_awesome_py_node**, this mimics the executable of c++. Without this you should run the .py file (**FiicAwesomePyNode.py**). You should look the file as a recipe for future implementation, without using the create package script.   |
|**src** | where the code lives | ROS and Algorithms code separated in two different folders. 
| **test** | Isolated code to test the node | Empty now, maybe in a future|
|**setup.py**| Tells in what folders the code is | For this to work every folder with python code should have an empty *__init__.py* file. Again look the file as recipe if you decide to create packages without using the script to create packages.|
| **config** |  Has yaml file where you can add parameters to be loaded by the parameter server | just node_frequency|

## Default Code behavior

By default you have one *Publisher* and one *Subscriber* in the node (py and cpp). If the *Subscriber* receives a "4" it will **activate** the **timer callback** and publish true in the **/wp_status** topic. Any other value will **deactivate** the **timer callback** and nothing will be publish, but in **main** you have a **spin** that will keep the other callback active. This means if you receive again a "4" it will **reactivate** the **timer callback**. So in an nutshell, the **timer callback** works like a iteration loop (while loop with ros rate). 

Note: this is only an example, please feel free to change this default behavior.

```
@.@: Publish when you receive a "4" in the Flag topic
Publishers: ['/wp_status']
msgs types: ['std_msgs/Bool']

@.@: Always active
Subscribers: ['/Flag']
msgs types: ['/Int8']
```