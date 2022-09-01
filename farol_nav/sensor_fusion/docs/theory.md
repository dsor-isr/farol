# Theory

## Kalman filter

A Kalman filter, also known as Linear Quadratic Estimator (LQE) uses a series of noisy measurements to produce estimates of unknown state variables. A Kalman filter provides an optimum solution to many tracking and data prediction tasks and finds excellent use in control, navigation and guidance.

The algorithm works in a two-step process. In the "predict" step, the Kalman filter looks one-step forward in time to predict the estimates of the current state vector and state covariance using an inner dynamic model. In the second step, known as "update", a "kalman gain matrix" is calculated using the state covariances and measurement covariance. This matrix is then used to update the state of the vehicle as shown below. One key advantage of Kalman filter is that it can run efficiently in real-time.

For a more comprehensive literature on the subject, the reader is recommended to consult [1]. Consult [2] for an excellent introductory article on KFs.

**Predict**

Let xkxk be the state vector and PkPk be the state covariance. The state equation is written as


$$
\hat{x}_{k|k-1} = A_k \hat{x}_{k-1|k-1} + B_k  u_k + w_k \\ \\
P_{k|k-1} = A_k P_{k-1|k-1} A_k^T + Q_k
$$


where,

- $A_k$ = State Transition Matrix
- $B_k$ = Control-Input Model
- $u_k$ = Input Vector
- $w_k$ = Zero-mean Gaussian Process Noise
- $Q_k$ = Covariance of the Process Noise

**Update**

Let $\tilde{y_k}$ be the innovation factor vector, i.e. the difference between the predicted state output and the measured state output.

$$
\tilde{y} = z_k - C_k\hat{x}_{k|k-1} \\ \\
S_k = C_k P_{k|k-1} C_k^T + R_k \\ \\
K_k = P_{k|k-1} C_k^T S_k^{-1} \\ \\
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k \tilde{y}_k \\ \\
P_{k|k-1} = (1 - k_k C_k)P_{k|k-1}
$$


where,

- $z_k$ = Measurement Vector
- $C_k$ = Observation Matrix
- $R_k$ = Measurement Covariance
- $S_k$ = Innovation Covariance
- $K_l$ = Kalman Gain Matrix

The variables $z_k$, $C_k$, $R_k$ are derived from the incoming measurement.

**Filter Model**

*Horizontal filter*

$$
x = \begin{bmatrix} x \\ y \\ \dot{x} \\ \dot{y} \\ \ddot{x} \\ \ddot{y} \\ \dot{x_c} \\ \dot{x_y} \end{bmatrix}, 
$$

$$
A_k =
\begin{bmatrix}
1 & 0 & \delta t & 0 & 0.5 \delta t^2 & 0 & 0 & 0\\
0 & 1 & 0 & \delta t & 0 & 0.5 \delta t^2 & 0 & 0\\
0 & 0 & 1 & 0 & \delta t & 0 & 0 & 0\\
0 & 0 & 0 & 1 & 0 & \delta t & 0 & 0\\
0 & 0 & 0 & 0 & 1  & 0 & 0 & 0\\
0 & 0 & 0 & 0 & 0  & 1 & 0 & 0\\
0 & 0 & 0 & 0 & 0  & 0 & 1 & 0\\
0 & 0 & 0 & 0 & 0  & 0 & 0 & 1\\
\end{bmatrix}, 
$$

$$
B_k = \begin{bmatrix} 0 \end{bmatrix}, w_k = \begin{bmatrix} 0 \end{bmatrix}
$$

where

- $\delta_t$ = predict_period

*Vertical Filter*

$$
x = \begin{bmatrix} z \\ \dot{z} \\ A \\ B \end{bmatrix}, 
$$

$$
A_k = \begin{bmatrix}
1 & \delta t & 0 & 0 \\
0 & 1 + \alpha\delta{t} & 0 & \delta{t} \\
0 & -\delta{t} & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}, 
$$

$$
B_k = \begin{bmatrix} 0 \end{bmatrix}, 
$$

$$
w_k = \begin{bmatrix} 0 \end{bmatrix}
$$

where,

- $\delta_t$ = predict period
- $A$ = altitude
- $B$ = buoyancy

*Rotation Filter*

$$
x = \begin{bmatrix} \psi \\ \theta \\ \gamma \\ \dot{\psi} \\ \dot{\theta} \\ \dot{\gamma} \end{bmatrix}, 
$$

$$
A_k = \begin{bmatrix}
1 & 0 & 0 & \delta{t} & 0 & 0 \\
0 & 1 & 0 & 0 & \delta{t} & 0 \\
0 & 0 & 1 & 0 & 0  & \delta{t} \\
0 & 0 & 0 & 1 & 0  & 0  \\
0 & 0 & 0 & 0 & 1  & 0 \\
0 & 0 & 0 & 0 & 0  & 1 \\
\end{bmatrix}, 
$$

$$
B_k = \begin{bmatrix} 0 \end{bmatrix}, 
$$

$$
w_k = \begin{bmatrix} 0 \end{bmatrix}
$$

where

- $\delta_t$ = predict_period

**Outlier Detection**

$$
e_n = \tilde{y}_k^T S_k^{-1} \tilde{y}_k 
$$

$$
e_n < th_{outlier}
$$

where

- $e_n$ = normalized error
- $th_{outlier}$ = threshold outlier rejection parameter

## References

1. Rhudy, Matthew & Salguero, Roger & Holappa, Keaton. (2017). A Kalman Filtering Tutorial for Undergraduate Students. International Journal of Computer Science & Engineering Survey. 08. 01-18. 10.5121/ijcses.2017.8101.
2. https://towardsdatascience.com/kalman-filter-an-algorithm-for-making-sense-from-the-insights-of-various-sensors-fused-together-ddf67597f35e