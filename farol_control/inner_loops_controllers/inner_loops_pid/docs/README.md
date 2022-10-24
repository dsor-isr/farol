# Inner Loops

## Sources

From

* [dos Santos Ribeiro, Jorge Miguel. "Motion control of single and multiple autonomous marine vehicles." PhD diss., MS thesis, Dept. Elect. Comput. Eng., Inst. Superior Técnico, Lisbon, Portugal, 2011.](https://fenix.tecnico.ulisboa.pt/downloadFile/395143403496/Tese_JorgeRibeiro.pdf)

* [Resendes Maia, Ana Cristina. "Sensor-Based Formation Control of Autonomous Robotic Vehicles." PhD diss., MS thesis, Dept. Elect. Comput. Eng., Inst. Superior Técnico, Lisbon, Portugal, 2013.](https://fenix.tecnico.ulisboa.pt/downloadFile/395146003786/disserta%C3%A7%C3%A3o.pdf)


* [P. C. Abreu et al., "The FAROL class of autonomous marine vehicles and their role in EU projects," OCEANS 2016 - Shanghai, Shanghai, 2016, pp. 1-10.](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7485620)

## Description

### Speed Controller

**Theory** 
The speed controller is a **PI** controller responsible to compute a common mode command for the two longitudinal thrusters as

$$
CM = -k_p \tilde{u} - k_i \int_0^t \tilde{u} (\tau) d\tau,
$$

$$
\tilde{u} = \hat{u}-u_d
$$

where $\hat{u}$ is the estimate and $u_d$ is the reference to be tracked.

**In code** 
* Reference $u_d$ is limited between [-0.5; 1.2] (m/s)
* Aceleration of the reference is limited to 0.04 (m/s^2).
* Common Mode is saturated between [-40; 60].

### Heading Controller 
**Theory**
The Heading controller is a **PID** responsible to compute a differential mode for the two longitudinal thrusters.

$$
DM = k_p\tilde{\psi} - k_d\dot{\tilde{\psi}} +k_i\int_0^t\tilde{\psi}(\tau)d\tau,
$$

$$
\tilde{\psi} = \hat{\psi}-\psi_d
$$

where $\hat{\psi}$ is the estimate and $\psi_d$ is the reference to be tracked.

**In code** 
* *Saturating the Reference Derivative:* Since the vehicle can not do more that 25 degrees per sec, the yaw rate given by the previous reference and the new reference is limited by 5 (ts=0.2). In mathematical terms,
  
$$
\dot{\tilde{\psi}} = \dot{\hat{\psi}} - \dot{\psi}_d
$$

$$
\dot{\psi}_d = (\psi_{d|k}-\psi_{d|k-1})/t_s,\quad \dot{\psi} \in [-5;5]
$$

* *Not integrating if the error is too big:* If $|\tilde{\psi}|$ > 20 then there is no integration.

* *Reducing the surge speed if the yaw err is big:* If $|\tilde{\psi}| \in [20;90]$, u_ref_gain is given by:

$$
u\_ref\_gain = (\sin(-|\frac{\tilde{\psi}}{90-20}| + \frac{90}{90-20} - 90) + 1) *0.5/2.0 + 0.5
$$

This component will be multiplied by the reference going into the speed controller. 
Note: If $\tilde{\psi}$ is below 20 the gain is 1 and if it is above 90 the gain is 0.5.

* The diffential mode is saturated between [-(80\*80);(80\*80)], and in case a saturation occurs the integration component is *reverted* to the old value (it is not updated).

### Heading Rate Controller
**Theory**
The heading rate controller is a **PI** with **feedforward** responsible to compute a differential mode for the longitudinal thrusters.

$$
DM = k_{f}\dot{\psi_d} + k_p\dot{\tilde{\psi}}-k_i\int_0^t\dot{\tilde{\psi}}(\tau)d\tau,
$$
**In code**
* The error $\tilde{\psi}$ is saturated between [-20;20].
* The diffential mode is saturated between [-(80\*80);(80\*80)], and in case a saturation occurs the integration is is *reverted* to the old value (it is not updated).

### Z Controller
**Theory**
The Z controller is a **PID** with **feedforward** acceleration controller responsible to compute a common mode for the vertical thrusters.

$$
CM_{vertical} = k_p\tilde{z} + k_d\dot{\tilde{z}} +k_i\int_0^t\tilde{z}(\tau)d\tau + \frac{\ddot{\tilde{z}}_d - \alpha\dot{z} - \beta\dot{z}|\dot{z}|}{\gamma},
$$

**In code**
* If depth references exist then it will work as a depth controller, otherwise if altitude references exist it will work as a altitude controller.
* *Safety features: Depth*
   If the water column minus the depth reference is smaller than our minimum altitude reference then it automatically goes to altitude control with a reference of our minimum altitude.
   It goes back to depth control when the water column minus the reference is bigger than 2m.
* *Safety features: Altitude*
   Limitates to our minimum altitude reference
   If the water column minus the altitude reference is bigger than our maximum depth then it is limited by that value.
 * *Pre filter:* 
  
   - Reference Rate saturation: the reference is saturated taking into account the maximum reference rate (0.2*Ts).
   - Calculation of:
  
$$
\ddot{\tilde{z}}_d = \omega	_n^2*(\tilde{z}_d - \tilde{z}_{d|k-1}) - 2\xi\omega_n*\ddot{\tilde{z}}_{d|k-1}
$$

$$
\dot{\tilde{z}}_d = \ddot{\tilde{z}}_d + \ddot{\tilde{z}}_d t_s
$$

$$
\tilde{z}_{d} = \tilde{z}_{d|k-1} + \dot{\tilde{z}}_d t_s
$$

 * CM is saturated between [-60,60].

### Roll Controller - NOT BEING USED

The roll controller is a **PD** responsible to return a differential mode for the vertical thrusters.

If roll is bigger than 60 degrees then the controller will stop.
### Note:

The CM and DM are defined as a percentage of the maximum RPM:
$$
CM = \frac{Left\ RPM (\%) + right\ RPM (\%)}{2}
$$

$$
DM = \frac{Left\ RPM (\%) - right\ RPM (\%)}{2}
$$
## Code description

### Controller_interface

Every controller inherits a ControllerInterface Class (controller_interface.c/h) composed by the following atributes:

* kp_ - proporcional gain
* ki_ - integral gain
* kd_ - integral gain
* ref_old_ - old reference
* qsi_old_ - saves the integration value

and the following methods:

* int execute([arguments]); - executes the pid controll
* void reset(); - resets the filter

In this interface, a Saturation struct was created to ease the operation of saturation.
Basically, it is only necessary to create a Saturation object in the controller constructor defining the minimum and maximum value. To saturate a value we just need to call the function {clip} from the Saturation object.

### Innerloops main loop

In the innerloops::timerCallback we find the logic to apply these controllers;

1 - If a yaw reference exists, the heading controller will compute a DM for the horizontal thrusters.
otherwise if a yaw rate ref exists the heading controller will be reset and the yaw rate controller will compute the DM. If none of the above cases happen it will check if we have an open loop value for DM otherwise it will reset the heading controller and the heading rate controller.

2 - If a speed reference exists, the speed controller will multiply it by the u_ref_gain (see heading controller) to compute a Common Mode for the longitudinal thrusters. Otherwise it will check if we have an open loop value for CM. If none of the above occur, the speed controller is reset.

3 - If in full power up, the vertical Common Mode will be equal to -60. Else it will check if we have a depth reference to compute de CM. After that it will check if we have a open loop CM and only then will it check if we have a altitude reference. If none of the above occur, the filter is reset.

4 - Roll is actuated in open loop.

5 - Saturating common mode: the horizontal common mode is saturated between [-(60\*60);(60\*60)].

