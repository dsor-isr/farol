# cpf_control Node

## In a nutshell
This node adds a cooperative to an already set PF algorithm.

## Diagram
![cpf\_control Diagram](img/cpf_control.png)

## Subscribers
| Subscribers | msg type | Purpose |
| --- | --- | --- |
| /#vehicle#/External/Gamma | [std\_msgs/Float64]() | |
| /#vehicle#/PathData | [dsor\_paths/PathData]() | |

## Publishers
| Publishers | msg type | Purpose |
| --- | --- | --- |
| /#vehicle#/PF/vc | [std\_msgs/Float64]() | |
| /#vehicle#/Internal/Gamma | [std\_msgs/Float64]() | |

## Services
| Services | msg type | Purpose |
| --- | --- | --- |
| /#vehicle#/CPFStart | [cpf\_control/StartStop](StartStop.md) | |
| /#vehicle#/CPFStop | [cpf\_control/StartStop](StartStop.md) | |
| /#vehicle#/CPFChangeTopology | [cpf\_control/ChangeTopology](ChangeTopology.md) | |

## Parameters

### General Parameters
| Parameters | type | Default | Purpose |
| --- | --- | --- | --- |
| node\_frequency | float | 10.0 | Working frequency of the node |
| ID | int | 2 | |
| adjacency\_matrix | array | [0, 1, 1, 1, <br /> 1, 0, 1, 1, <br /> 1, 1, 0, 1, <br /> 1, 1, 1, 0] | |

### Event-Triggered Communications Gains
| Parameters | type | Default | Purpose |
| --- | --- | --- | --- |
| c0 | float | 0.001 | |
| c1 | float | 5.0 | |
| alpha | float | 1.0 | |
| k\_epsilon | float | 1.0 | |