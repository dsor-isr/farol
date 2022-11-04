# Farol Gimmicks Library

## Description

Collection of methods to be used by several nodes in the Farol stack.

---

## Important Sources

The library was built following the steps in this [link](https://roboticsbackend.com/ros-include-cpp-header-from-another-package/)
Also an important note about implementing template methods [here](https://stackoverflow.com/questions/1353973/c-template-linking-error). Basically this avoids linking errors with the library.

---

## How to use the library in a new package/node

#### CmakeLists.txt
Add library to find package 
```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  farol_gimmicks_lirabry
)
```

#### package.xml
Add the following
```
<depend>farol_gimmicks_lirabry</depend>
```

#### Include the library in your new node

```
#include <farol_gimmicks_library/FarolGimmicks.h>
```

#### Use in your code

```
FarolGimmicks::method_to_use(...)
```

List of methods available [here](https://dsor-isr.github.io/farol/api/markdown/farol_addons/libraries/farol_gimmicks_library/Namespaces/namespaceFarolGimmicks/#functions).

---


