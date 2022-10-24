# static_thruster_allocation node

## In a nutshell
Transforms the forces and torques of the inner loops controllers into percentage of RPM(revolution per minute).

## Diagram
![static_thruster_allocation Diagram](img/static_thruster_allocation.png)

## Subscribers

## Publishers

## Services

## Parameters

## Rationale

![thrust_allocation_overview](img/thrust_allocation_overview.png)

### Steps

1. **Forces and torques desired:** Given by the inner loops.
2. **Thrust allocation:**  Transforms the forces and torques that we want apply to the vehicle into the necessary forces to give to each thruster.
3.  **Thruster's Force Desired:** Force applied to each thruster.
4. **Forces (N) to %RPM:** Converts the forces into a percentage of RPMs (Revolution per minute). NOTE: can be modified.
5. **Driver:** Computes the signal to send to the thruster ESC (Electronic speed controller).
6. **Thruster:** The thruster ESC reads the signal and converts to to PWMs (Pulse with modulation).

### Thruster Allocation Matrix (TAM)

![tam](img/TAM_single_thruster.png)

$r$ - distance to the thruster in $x$, $y$ and $z$ axis. 

### Normal Medusa thruster installation

![normal_medusa](img/tam_simple_medusa.png)

### Medusa Vector thruster installation

![medusa_vector](img/tam_vector_medusa.png)

### Computation of force allocation

#### Explicit solution using Lagrange multipliers

![lagrange_multipliers](img/con
