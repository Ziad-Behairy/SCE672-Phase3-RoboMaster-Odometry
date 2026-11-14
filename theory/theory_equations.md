# RoboMaster Odometry Project (Phase 3): Theory & Equations Reference
**Course:** SCE-672 Advanced Robotics  
**Extracted from:** Lectures 11A, 11B, 12A, and 12B (Prof. Aamer Iqbal Bhatti)

---

## 1. Project Overview & System Architecture
For **Project Phase 3**, the goal is to drive the RoboMaster EP Core straight toward a fixed QR tag at a known starting distance (1–2 m) and stop at a fixed stopping distance before reaching it, using **odometry-based prediction** and **sensor-based correction** to estimate the robot's pose.

### Sensor Architecture & Double-Counting Rule (Lec 11B)
* **Inward-looking sensors (Proprioceptive):** Wheel encoders and IMU measure internal motion (velocities $v, \omega$), *not* global position $(p_x, p_y)$. Over time, pure odometry drifts (especially along-track distance due to wheel slip or inner-loop lag).
* **Outward-looking sensor (Exteroceptive):** The camera detects the fixed QR tag at known coordinates $(m_x, m_y)$ to correct position drift.
* **CRITICAL WARNING (Lec 11B, Slide 24):** The RoboMaster ROS driver (`/odom` at 10 Hz) **already fuses the IMU and wheel encoders**. 
  * Use `/odom` alone with its own noise covariance, **OR** fuse `/joint_states_p` (raw wheels) and `/imu` separately yourself. 
  * **Never fuse both `/odom` and `/imu` together**, or the IMU evidence is counted twice, causing the covariance matrix $P$ to collapse and making the filter overconfident.

---

## 2. State Vector & Process Model (Prediction Step — Lec 11A)

### State Representation
Because the RoboMaster's inner motion controller takes time to track `/cmd_vel` commands, linear velocity $v$ and angular velocity $\omega$ are included in the state vector alongside the robot's 2D pose:
$$x = \begin{bmatrix} p_x & p_y & \theta & v & \omega \end{bmatrix}^T$$
* $p_x, p_y$: Robot position in the room frame ($\text{m}$)
* $\theta$: Robot heading angle measured anticlockwise from the room's $x$-axis ($\text{rad}$)
* $v$: Linear forward velocity ($\text{m/s}$)
* $\omega$: Angular velocity / turn rate ($\text{rad/s}$)
* Control input: $u = \begin{bmatrix} a & \alpha \end{bmatrix}^T$ (linear and angular accelerations).

*(Note: For a known fixed tag $(m_x, m_y)$, the tag coordinates are constants rather than estimated states, keeping the state size at 5.)*

### Discrete-Time Motion Model ($x_{k+1} = f(x_k, u_k)$)
Running at sample period $dt$ (e.g., $dt = 0.1\text{ s}$ for $10\text{ Hz}$):
$$p_{x,k+1} = p_{x,k} + v_k \cos(\theta_k)\,dt$$
$$p_{y,k+1} = p_{y,k} + v_k \sin(\theta_k)\,dt$$
$$\theta_{k+1} = \theta_k + \omega_k\,dt$$
$$v_{k+1} = v_k + a_k\,dt$$
$$\omega_{k+1} = \omega_k + \alpha_k\,dt$$

### State & Input Jacobians (Lec 11A, Slide 15)
Used to propagate the state error covariance matrix $P$ forward ($P^- = F P F^T + Q$):

$$F = \frac{\partial f}{\partial x_k} = \begin{bmatrix} 
1 & 0 & -v_k \sin(\theta_k)\,dt & \cos(\theta_k)\,dt & 0 \\ 
0 & 1 & v_k \cos(\theta_k)\,dt & \sin(\theta_k)\,dt & 0 \\ 
0 & 0 & 1 & 0 & dt \\ 
0 & 0 & 0 & 1 & 0 \\ 
0 & 0 & 0 & 0 & 1 
\end{bmatrix}, \quad
G = \frac{\partial f}{\partial u_k} = \begin{bmatrix} 
0 & 0 \\ 
0 & 0 \\ 
0 & 0 \\ 
dt & 0 \\ 
0 & dt 
\end{bmatrix}$$

* **Verification Check (Lec 11A, Slide 18):** Verify analytic Jacobians numerically via central differences ($h = 10^{-6}$):
  $$\frac{\partial f_i}{\partial x_j} \approx \frac{f_i(x + h e_j) - f_i(x - h e_j)}{2h}$$

---

## 3. Measurement Models & Jacobians (Correction Step)

### A. Wheel Odometry Measurement Model (Lec 11B, Slides 7 & 11)
For the 4 Mecanum wheels with radius $r_w$, half-width $l_x$, half-length $l_y$, and wheel angular speeds $\dot{\phi}_1, \dots, \dot{\phi}_4$:
$$v = \frac{r_w}{4}(\dot{\phi}_1 + \dot{\phi}_2 + \dot{\phi}_3 + \dot{\phi}_4)$$
$$v_y = \frac{r_w}{4}(-\dot{\phi}_1 + \dot{\phi}_2 + \dot{\phi}_3 - \dot{\phi}_4) \quad (= 0 \text{ for straight-line motion})$$
$$\omega = \frac{r_w}{4(l_x + l_y)}(-\dot{\phi}_1 + \dot{\phi}_2 - \dot{\phi}_3 + \dot{\phi}_4)$$

The odometry measurement $z_{\text{odom}} = \begin{bmatrix} v & \omega \end{bmatrix}^T$ directly observes $v$ and $\omega$:
$$h_{\text{odom}}(x) = \begin{bmatrix} v \\ \omega \end{bmatrix}, \quad H_{\text{odom}} = \begin{bmatrix} 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 1 \end{bmatrix}$$
*(Note: Odometry corrects position $p_x, p_y$ indirectly through the off-diagonal covariance terms $p_{xv}, p_{yv}$ in $P$, which are populated by $F$ during prediction.)*

---

### B. QR Tag Camera Measurement Model (Lec 12A, Slides 10–22)
Let the fixed QR tag be at known coordinates $(m_x, m_y)$ and the robot's predicted pose be $(p_x, p_y, \theta)$. Define the relative displacement vector and squared distance:
$$\Delta x = m_x - p_x, \quad \Delta y = m_y - p_y$$
$$q = \Delta x^2 + \Delta y^2, \quad d = \sqrt{q}$$

The nonlinear range-and-bearing measurement model is:
$$z = h(x) + \delta = \begin{bmatrix} d \\ \varphi \end{bmatrix} + \delta = \begin{bmatrix} \sqrt{(m_x - p_x)^2 + (m_y - p_y)^2} \\ \text{atan2}(m_y - p_y,\, m_x - p_x) - \theta \end{bmatrix} + \delta, \quad \delta \sim \mathcal{N}(0, R)$$

* **Range ($d$):** Euclidean distance from robot to tag (independent of robot heading $\theta$).
* **Bearing ($\varphi$):** Angle to the tag in the robot's body frame, measured anticlockwise from the robot's current heading $\theta$.
* **Sensor Noise Covariance ($R$):** Diagonal matrix of measurement variances, e.g., $R = \text{diag}(r_{dd}, r_{\varphi\varphi})$.

#### Measurement Jacobian ($H_{\text{tag}}$)
Differentiating $h(x)$ with respect to $x = \begin{bmatrix} p_x & p_y & \theta & v & \omega \end{bmatrix}^T$:
$$H_{\text{tag}} = \frac{\partial h}{\partial x} = \begin{bmatrix} 
-\frac{\Delta x}{d} & -\frac{\Delta y}{d} & 0 & 0 & 0 \\ 
\frac{\Delta y}{q} & -\frac{\Delta x}{q} & -1 & 0 & 0 
\end{bmatrix}$$

* If your camera pipeline extracts **both range and bearing** (e.g., from tag size/homography + horizontal pixel offset), use both rows of $H_{\text{tag}}$.
* If your camera extracts **bearing only**, use only the second row $H_\varphi = \begin{bmatrix} \frac{\Delta y}{q} & -\frac{\Delta x}{q} & -1 & 0 & 0 \end{bmatrix}$.

---

## 4. Kalman Filter Update & Angle Wrapping (Lec 11B, 12A, 12B)

### Scalar / Row-by-Row Intuition (Lec 12A)
For a single scalar measurement (such as bearing $\varphi$ with variance $r_{\varphi\varphi}$), the innovation covariance is $S = H_\varphi P H_\varphi^T + r_{\varphi\varphi} = p_{\varphi\varphi} + r_{\varphi\varphi}$, and the Kalman gain for state $i$ is:
$$K_i = \frac{p_{i\varphi}}{p_{\varphi\varphi} + r_{\varphi\varphi}}$$
where $p_{i\varphi}$ is the cross-covariance between state $i$ and the predicted bearing $\varphi^-$ (computed via $P H_\varphi^T$).

### Standard EKF Correction Equations
1. **Innovation (Disagreement):**
   $$e = z - h(x^-)$$
2. **Innovation Covariance:**
   $$S = H P^- H^T + R$$
3. **Kalman Gain:**
   $$K = P^- H^T S^{-1}$$
4. **State Update:**
   $$x^+ = x^- + K e$$
5. **Covariance Update:**
   $$P^+ = (I - K H) P^-$$

### Mandatory Angle Wrapping Rules (Lec 12B, Slides 15–17 & 27)
Subtracting two angles directly fails across the $\pm \pi$ boundary (producing a $\approx 2\pi$ jump). Define the branchless wrap function into $(-\pi, \pi]$:
$$\text{wrap}(\alpha) = \text{atan2}(\sin(\alpha),\, \cos(\alpha))$$

* **Where to apply `wrap()`:**
  1. **Bearing innovation:** $e_\varphi = \text{wrap}(z_\varphi - \varphi^-)$
  2. **Heading state after update:** $\theta^+ = \text{wrap}(\theta^+)$ at every time step so it never walks out of $(-\pi, \pi]$.
  3. **Averaging angles (if used):** Average $\sin$ and $\cos$ separately, then take $\text{atan2}$.
* **Where NEVER to apply `wrap()`:**
  * **Never wrap distance innovations** ($e_d = z_d - d^-$ is a length, not an angle).
  * **Never wrap the covariance matrix $P$ or Kalman gain $K$** (entries of $P$ are second moments/variances, not angles).

---

## 5. Crucial Experimental Insight for Phase 3 (Lec 12A, Slide 26)
* **Observability Geometry:** When driving straight toward a single tag ahead ($\varphi \approx 0$), **bearing alone** strongly corrects **cross-track (lateral) error** ($p_y$) and heading ($\theta$), but provides **almost zero information about along-track distance** ($p_x$ / distance to tag).
* **Why this matters for your report & implementation:**
  * Along-track distance is the exact axis where wheel odometry drifts!
  * Therefore, to accurately estimate the **final distance to the tag** in Phase 3, your camera tag measurement $z$ **must include the measured distance/range $d$ to the tag** (e.g., computed from the known physical width/height of the QR tag and camera focal length via pinhole camera geometry / `solvePnP`), using the first row of $H_{\text{tag}}$ ($\begin{bmatrix} -\frac{\Delta x}{d} & -\frac{\Delta y}{d} & 0 & 0 & 0 \end{bmatrix}$) alongside bearing.
  * Make sure to discuss this exact geometric observability property (bearing fixing cross-track vs. range fixing along-track odometry drift) in **Section 4 of your project report**!