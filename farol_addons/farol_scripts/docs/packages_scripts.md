# FAROL New Packages Scripts

## Description
Bash scripts for making new packages and nodes in a more consice manner. Using these scripts allows the stack to have a uniform and common structure, improving code readability and consequently easier to understand.

## Scripts

| File                             | Usage via Alias                    | Output         |
| ----------                       | ----------                         | ----           |
| **farol_create_ros_pkg_cpp.sh**  | farol_pkg_cpp <package_name>       | c++ package    |
| **farol_create_ros_pkg_py.sh**   | farol_pkg_py  <package_name>       | python package |
| **farol_create_ros_pkg_meta.sh** | farol_pkg_meta <meta_package_name> | meta package   |

**Note:** The scripts automatically compile the package. After execution you will be inside the package folder.

## Example fiic_awesome package (meta, c++, python)

### What you get

After running the following commands in terminal (please follow the same order):
```
medusa_pkg_meta fiic_awesome 
medusa_pkg_cpp fiic_awesome_cpp 
cd ..
medusa_pkg_py fiic_awesome_py
cd ../../
tree
```

After running the commands successfully you will get the following tree structure:
```
.
├── fiic_awesome
│   ├── CMakeLists.txt
│   ├── docs
│   │   └── README.md
│   └── package.xml
│
│
├── fiic_awesome_cpp
│   ├── CMakeLists.txt
│   ├── config
│   │   └── config_fiic_awesome_cpp.yaml
│   ├── docs
│   │   └── README.md
│   ├── include
│   │   ├── fiic_awesome_cpp_algorithms
│   │   │   └── FiicAwesomeCppAlgorithm.h
│   │   └── fiic_awesome_cpp_ros
│   │       └── FiicAwesomeCppNode.h
│   ├── launch
│   │   └── fiic_awesome_cpp.launch
│   ├── package.xml
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
    ├── docs
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

## Details

### In all scripts
|             Feature              |                                                                                                                     Description                                                                                                                      |
|:--------------------------------:|:----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
|      Package folders names       |                                    should always be lower case and separated by underscore. Even if you introduce capital letters the script will make them lowercase. The underscore is up to you to follow it.                                     |
|          Ros Code part           |                                                                                      In dedicated folders ending with **_ros** and files ending with **Node**.                                                                                       |
|       Algorithm Code part        | **YES** we want to make the algorithm part **standalone**. The idea is to have cleaner code and also make possible to use the algorithms in another middleware. In dedicated folders ending with **_algorithms** and files ending with **Algorithm** |
|            Ros Nodes             |                                                              Are implemented as **classes** and use **Timer calbacks** instead of the classical ros::Rate/while loop/ros_sleep/ rosspin                                                              |
|           Class names            |                                                                                        Are like **MyClassName** first letter uppercase and the rest camelcase                                                                                        |
|          Function Names          |                                                                                                        Regular camelcase: **myFunctionName**                                                                                                         |
|            variables             |                                                                                           lowercase and separated by underscores: **my_variable_example**                                                                                            |
| Parameters from parameter server |                                                                                        start with **p_** followed by parameter name: **p_my_super_parameter**                                                                                        |
|     Files that have classes      |                                                                                                         Will have the same name as the class                                                                                                         |

### medusa_pkg_cpp fiic_awesome
|       Creates        | Description                                                                       |                                                                         Details                                                                         |
|:--------------------:|-----------------------------------------------------------------------------------|:-------------------------------------------------------------------------------------------------------------------------------------------------------:|
| **fiic_awesome_cpp** | Folder with cpp node                                                              |                                                                    Everything ready.                                                                    |
|  **CMakeLists.txt**  | CMake for ros                                                                     |                    c++17 ready: **add_compile_options(-std=c++17)**; executable: **fiic_awesome_cpp_node** -> package_name+**_node**                    |
|  **docs/Readme.md**  | Folder with a Readme file for documentation                                       |                                                                  Empty, please use it                                                                   |
|   **package.xml**    | Simple package manifest                                                           |                                    Already format 2. As dependencies has std_msgs, rospy and medusa_msgs by default                                     |
|      **launch**      | launch file for the node                                                          |              Has xml header, thank god. Also it has an example with what you should had to the file and alredy configured to be launched.               |
|       **src**        | Where the code lives                                                              | ROS and Algorithms code separated in two different folders. Configure **CMakeLists.txt** properly, but in doubt see what is done and follow the recipe. |
|       **test**       | Isolated code to test the node                                                    |                                                              Empty now, maybe in a future                                                               |
|      **config**      | Has yaml file where you can add parameters to be loaded into the parameter server |                                                    just node_frequency, feel free to add more params                                                    |
|     **include**      | Declarations (.h files) of your code.                                             |                                           ROS and Algorithms declarations separated in two different folders.                                           |

### medusa_pkg_py fiic_awesome
|       Creates       | Description                                 |                                                                                                                        Details                                                                                                                         |
|:-------------------:|---------------------------------------------|:------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| **fiic_awesome_py** | Folder with python node                     |                                                                                                                   Everything ready.                                                                                                                    |
| **CMakeLists.txt**  | CMake for ros                               |                                                                                                  has setup configuration: **catkin_python_setup()**;                                                                                                   |
|  **docs/Readme.md**  | Folder with a Readme file for documentation |                                                                                                                  Empty, please use it                                                                                                                  |
|   **package.xml**   | simple package manifest                     |                                                                                    Already format 2. As dependencies has std_msgs, rospy and medusa_msgs by default                                                                                    |
|     **launch**      | launch file for the node                    |                                                              Has xml header, thank god. Also it has an example with what you should had to the file and alredy configured to be launched.                                                              |
|     **scripts**     | It will be your node executable             | Executable: **fiic_awesome_py_node**, this mimics the executable of c++. Without this you should run the .py file (**FiicAwesomePyNode.py**). You should look the file as a recipe for future implementation, without using the create package script. |
|       **src**       | where the code lives                        |                                                                                              ROS and Algorithms code separated in two different folders.                                                                                               |
| **test**     | Isolated code to test the node                                                  | Empty now, maybe in a future                                                                                                                                                                        |
| **setup.py** | Tells in what folders the code is                                               | For this to work every folder with python code should have an empty *__init__.py* file. Again look the file as recipe if you decide to create packages without using the script to create packages. |
| **config**   | Has yaml file where you can add parameters to be loaded by the parameter server | just node_frequency                                                                                                                                                                                 |
