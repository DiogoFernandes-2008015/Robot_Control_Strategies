# Computed Torque, Adaptive and Learning Control for Robotic Manipulators

This repository contains implementations of model-based, **adaptive** and **learning** control techniques for trajectory tracking in rigid robotic manipulators, developed in **MATLAB/Simulink** and **Python**.

It covers three families of controllers:

1. **Computed Torque Control (CTC)** for a 2-DOF serial manipulator (feedback linearization with a PD outer loop).
2. **Adaptive control** for systems with unknown parameters: an *error-based* adaptation law and the **Slotine–Li** algorithm (*filtered-error* adaptation).
3. **Iterative Learning Control (ILC)** with the **Arimoto D-type** law for repetitive tasks.

The repository also includes the post-processing script `anima_simulink_2.m` and the generic graphical routine `dinanima_ref.m` for animating the robot while it executes the reference trajectory.

## 📌 Repository Contents

| File | Description |
| :--- | :--- |
| `controle_ctc_2dof.slx` | Simulink model of the CTC loop: inverse dynamics, trajectory generator and PD loop in joint coordinates. |
| `Controle_Adaptativo_1.slx` | Simulink model of the adaptive controller with **error-based adaptation** (adaptation law driven by $\dot{\tilde q}$). |
| `Controle_Adaptativo_2.slx` | Simulink model of the **Slotine–Li** adaptive controller (adaptation law driven by the **filtered error** $\sigma$). |
| `Controle_Adaptativo_PPR.slx` | Simulink model of the **Slotine–Li** adaptive controller applied to a Prismatic-Prismatic-Rotational Manipulator. |
| `Arimoto.py` | Python script implementing **Arimoto's D-type ILC** on a mass–spring–damper system with an unknown periodic disturbance. |
| `anima_simulink_2.m` | MATLAB script that extracts data from `out.logsout` after a simulation and calls the animation function. |
| `dinanima_ref.m` | Function that computes forward kinematics at each time step and renders the robot animation over the reference trajectory. |
| `README.md` | This document: theory, code description and usage. |

---

## 🧠 Part 1 — Computed Torque Control (CTC)

**Computed Torque Control** is a classical technique based on **feedback linearization**. It cancels the nonlinear forces and moments of the manipulator by injecting the inverse dynamic model into the closed loop, turning a coupled nonlinear system into a set of independent double integrators.

### 1.1 Dynamic Model in Joint Space

The nonlinear ODE describing a rigid manipulator with $n$ DOF is:

$$
\tau = H(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q)
$$

where:
* $q(t) \in \mathbb{R}^n$: joint positions (angular/prismatic).
* $\dot{q}(t), \ddot{q}(t) \in \mathbb{R}^n$: joint velocities and accelerations.
* $\tau(t) \in \mathbb{R}^n$: actuator torques/forces.
* $H(q) \in \mathbb{R}^{n \times n}$: inertia matrix (symmetric, positive definite).
* $C(q, \dot{q})\dot{q} \in \mathbb{R}^n$: centrifugal and Coriolis forces.
* $g(q) \in \mathbb{R}^n$: gravitational torques.

### 1.2 Control Law and Feedback Linearization

$$
\tau := H(q)u + C(q, \dot{q})\dot{q} + g(q)
$$

With perfect parameter knowledge ($\hat{H} = H$, $\hat{C} = C$, $\hat{g} = g$), substituting into the dynamics gives

$$
H(q)u + C\dot{q} + g = H(q)\ddot{q} + C\dot{q} + g \implies u = \ddot{q}
$$

so $u(t)$ acts as a **commanded virtual acceleration** on $n$ decoupled linear double integrators.

### 1.3 Auxiliary PD Loop

With tracking error $e(t) = q_d(t) - q(t)$:

$$
u = \ddot{q}_d + K_v \dot{e} + K_p e \quad\Longrightarrow\quad \ddot{e} + K_v \dot{e} + K_p e = 0
$$

where $K_p, K_v$ are positive-definite diagonal matrices. Choosing critical damping ($\zeta_i = 1$) per joint:

* $K_{pi} = \omega_{ni}^2$
* $K_{vi} = 2\omega_{ni}$

places a double real pole at $s = -\omega_{ni}$, ensuring exponential convergence $e(t) \to 0$.

> **Limitation:** CTC requires accurate knowledge of $H$, $C$ and $g$. When the parameters are uncertain, the cancellation is imperfect — which motivates the adaptive schemes of Part 2.

---

## 🧩 Part 2 — Adaptive Control

### 2.1 Linearity in the Parameters

A fundamental structural property of robot dynamics is that the equations of motion can always be written **linearly in a set of constant dynamic parameters** $\pi$ (masses, moments of inertia, center-of-mass locations):

$$
\tau = Y(q, \dot{q}, \ddot{q})\,\pi
$$

where $Y \in \mathbb{R}^{n \times p}$ is the **regressor matrix**, built only from measurable kinematic quantities.

#### Example: planar 2-DOF RR robot (horizontal plane, no gravity)

Only **lumped parameters** need to be estimated, $\pi = [\pi_1\ \pi_2\ \pi_3]^\top$:

$$
\pi_1 = m_1 l_{c1}^2 + I_1 + m_2 l_1^2 + m_2 l_{c2}^2 + I_2, \qquad
\pi_2 = m_2 l_{c2}^2 + I_2, \qquad
\pi_3 = m_2 l_1 l_{c2}
$$

and the regressor is

$$
Y = \begin{bmatrix}
\ddot q_1 & \ddot q_2 & 2\cos q_2\,\ddot q_1 + \cos q_2\,\ddot q_2 - \sin q_2\,\dot q_2^2 - 2\sin q_2\,\dot q_1\dot q_2 \\
0 & \ddot q_1 + \ddot q_2 & \cos q_2\,\ddot q_1 + \sin q_2\,\dot q_1^2
\end{bmatrix}
$$

### 2.2 Tracking Errors and Notation

$$
\tilde q = q - q_d, \qquad \dot{\tilde q} = \dot q - \dot q_d
$$

$$
\dot q_r = \dot q_d - \Lambda \tilde q, \qquad \ddot q_r = \ddot q_d - \Lambda \dot{\tilde q}, \qquad
\sigma = \dot q - \dot q_r = \dot{\tilde q} + \Lambda \tilde q
$$

with $\Lambda$ diagonal positive definite. $\sigma$ is the **filtered error**. $\hat\pi$ denotes the parameter estimate and $\tilde\pi = \hat\pi - \pi$ the parametric error. $K_p$, $K_d$ and the adaptation gain $\Gamma$ are symmetric positive definite.

> **Convention:** in the code and in the scalar example below, $\Gamma$ is used as the **adaptation gain** (multiplying the update law), so the Lyapunov function carries $\Gamma^{-1}$.

### 2.3 Algorithm 1 — Error-Based Adaptation (`Controle_Adaptativo_1.slx`)

PD action on the tracking error, with the adaptation law driven by $\dot{\tilde q}$:

$$
u = Y(q, \dot q, \dot q_d, \ddot q_d)\,\hat\pi - K_p \tilde q - K_d \dot{\tilde q}
$$

$$
\dot{\hat\pi} = -\Gamma\, Y^\top \dot{\tilde q}
$$

**Scalar example implemented in the model.** Unknown mass $m$, plant $m\ddot q = F$, reference $q_d(t) = 5t^2$ (so $\dot q_d = 10t$, $\ddot q_d = 10$), regressor $Y = \ddot q_d = 10$, error $e = q - q_d$ and $e_m = \hat m - m$:

$$
F = 10\,\hat m - K_p e - K_d \dot e, \qquad \dot{\hat m} = -10\,\Gamma\,\dot e
$$

Closed loop in $(e, \dot e, e_m)$:

$$
m\ddot e + K_d \dot e + K_p e = 10\,e_m, \qquad \dot e_m = -10\,\Gamma\,\dot e
$$

**Lyapunov analysis.** With $V = \tfrac12 m\dot e^2 + \tfrac12 K_p e^2 + \tfrac12 \Gamma^{-1} e_m^2$ one gets

$$
\dot V = -K_d \dot e^2 \le 0
$$

so $e, \dot e, \hat m$ are bounded, $\dot e \in L_2$ and, by Barbalat's lemma, $\dot e \to 0$. However, $\dot V$ is only negative **semi**-definite in $(e, \dot e, e_m)$, so **$e \to 0$ does not follow**.

**Adaptation invariant.** Since $\dot e_m + 10\Gamma \dot e = 0$,

$$
e_m(t) = c - 10\,\Gamma\, e(t), \qquad c = \hat m(0) - m + 10\,\Gamma\, e(0)
$$

and the closed loop reduces to a linear 2nd-order ODE with constant forcing, giving, **exponentially**:

$$
e_\infty = \frac{10\,c}{K_p + 100\,\Gamma}, \qquad
\dot e \to 0, \qquad
e_{m,\infty} = \frac{K_p}{K_p + 100\,\Gamma}\,c
$$

**Non-convergence result.** $e_\infty = 0$ if and only if $c = 0$, a non-generic condition. In particular, for $e(0) = 0$ and $\hat m(0) \ne m$, both the tracking error and the parameter error remain nonzero.

*Numerical example:* $m = 2$ kg, $\hat m(0) = 0.5$ kg, $e(0) = 0.5$ m, $K_p = 100$, $K_d = 20$, $\Gamma = 5$ gives $c = 23.5$, $e_\infty \approx 0.392$ m and $\hat m_\infty \approx 5.92$ kg $\neq m$.

> This model is deliberately included to show **why adaptation based only on $\dot{\tilde q}$ is not enough** — it motivates the filtered-error formulation below.

**Model parameters:** `m`, `Kp`, `Kd`, `Gamma`. **Logged signals:** `q`, `qd`, `qtil`, `qtipl` ($\dot{\tilde q}$), `pih` ($\hat\pi$), `tau`.

### 2.4 Algorithm 2 — Slotine–Li (`Controle_Adaptativo_2.slx`)

The Slotine–Li algorithm replaces the measured acceleration by the filtered reference signals $\dot q_r, \ddot q_r$ (avoiding noise amplification from differentiating measurements) and drives the adaptation with the **filtered error** $\sigma$:

$$
\tau = Y(q, \dot q, \dot q_r, \ddot q_r)\,\hat\pi - K_d\,\sigma
$$

$$
\dot{\hat\pi} = -\Gamma\, Y^\top(q, \dot q, \dot q_r, \ddot q_r)\,\sigma
$$

**Stability proof.** Take

$$
V(\sigma, \tilde\pi) = \tfrac12 \sigma^\top H(q)\sigma + \tfrac12 \tilde\pi^\top \Gamma^{-1}\tilde\pi
$$

The error dynamics are $H\dot\sigma + C\sigma + K_d\sigma = -Y\tilde\pi$. Using the skew-symmetry of $\dot H - 2C$:

$$
\dot V = -\sigma^\top K_d\,\sigma \le 0
$$

Hence $\sigma, \tilde q, \tilde\pi$ are bounded and, by Barbalat's lemma, $\tilde q \to 0$ and $\dot{\tilde q} \to 0$.

**Why it works for the scalar example.** The term $\Lambda\tilde q$ in $\sigma$ introduces an **integral action** on the position error. Defining $z = \int_0^t \tilde q\,d\tau$, the closed loop becomes a third-order system,

$$
m\,\dddot z + K_d\,\ddot z + (K_p + 100\,\Gamma)\,\dot z + 100\,\Gamma\lambda\, z = 10\,c'
$$

($c'$ constant), which by the Routh–Hurwitz criterion is stable if $K_d\,(K_p + 100\,\Gamma) > 100\,\Gamma\lambda\, m$, with $z \to c'/(10\Gamma\lambda)$ and therefore $\tilde q = \dot z \to 0$. The steady-state error of Algorithm 1 is eliminated.

**Model parameters:** `m`, `Kd`, `Lambda`, `Gamma`. **Logged signals:** `q`, `qd`, `qtil`, `qtipl`, `pih`, `tau`.

### 2.5 Parameter Convergence and Persistent Excitation (PE)

Tracking-error convergence ($\tilde q \to 0$) **does not imply** parameter convergence ($\hat\pi \to \pi$). Since $\dot V$ is only negative semi-definite, parameter convergence requires a sufficiently rich trajectory.

**Definition (PE).** The regressor $Y(t)$ is persistently exciting if there exist $\alpha_1, \alpha_2, T > 0$ such that, for all $t \ge 0$,

$$
\alpha_1 I_p \;\le\; \int_t^{t+T} Y^\top(\tau)\,Y(\tau)\,d\tau \;\le\; \alpha_2 I_p
$$

* **Upper bound:** the regressor is bounded (signals do not blow up).
* **Lower bound (crucial):** the autocorrelation matrix is strictly positive definite on every window, i.e. the signal injects energy into **all** $p$ directions of parameter space.

**Frequency content.** To identify $p$ parameters uniquely, the reference must be rich of order $m \ge \lceil p/2 \rceil$:

| Signal | Distinct frequencies | Parameters it can excite |
| :--- | :---: | :--- |
| Constant $r(t)=c$ | 1 (0 Hz) | at most 1 |
| Single sinusoid $A\sin(\omega t)$ | 1 | at most 2 |
| Sum of $N$ sinusoids | $N$ | up to $2N$ |
| PRBS (pseudo-random) | broad spectrum | high orders |

**Theorem (UGES under PE).** If the reference regressor $Y_d(t) = Y(q_d, \dot q_d, \dot q_d, \ddot q_d)$ is PE, the equilibrium $(\sigma, \tilde\pi) = (0, 0)$ is uniformly globally exponentially stable, and $\hat\pi(t) \to \pi$.

**Didactic example — 1 revolute joint:** $J\ddot q + b\dot q + mgl\cos q = \tau$, $\theta = [J\ b\ mgl]^\top$, $Y = [\ddot q_r\ \ \dot q_r\ \ \cos q]$.

* **Scenario A — constant reference** ($q_d = \pi/4$): $Y \to [0\ 0\ 0.707]$, so $Y^\top Y$ has eigenvalues $\{0, 0, 0.5\}$ and PE fails. Only $mgl$ is estimated; $J$ and $b$ stay undetermined.
* **Scenario B — two sinusoids** ($q_d = A_1\sin\omega_1 t + A_2\sin\omega_2 t$, $\omega_1 \ne \omega_2$): the three regressor terms are linearly independent, $\alpha_1 > 0$, and $\hat J \to J$, $\hat b \to b$, $\widehat{mgl} \to mgl$.

| Metric | Without PE | With PE |
| :--- | :--- | :--- |
| Position tracking error ($e \to 0$) | Yes | Yes |
| Parameter error ($\tilde\theta \to 0$) | No (may stall at a nonzero value) | Yes |
| Stability type | Barbalat ($\sigma \to 0$) | Uniform exponential |
| Robustness to noise/disturbances | Low (drift risk) | High |
| Trajectory richness | Poor (constant / low frequency) | Rich in frequencies |

### 2.6 Adaptive vs. Sliding-Mode Robust Control (reference)

| Feature | Adaptive (Slotine–Li) | Robust (SMC) |
| :--- | :--- | :--- |
| Control action | Smooth, continuously adjusted through integration of the error | High-frequency switching around the surface $\sigma = 0$ |
| Model uncertainty | Estimates and compensates constant parametric uncertainty | Rejects bounded uncertainty without explicit identification |
| Chattering | Absent; preserves actuators and joints | Present; requires a boundary layer |
| Parameter convergence | Requires PE | No parameter estimation; focuses on state tracking |

---

## 🔁 Part 3 — Iterative Learning Control (Arimoto D-type, `Arimoto.py`)

For tasks executed **repeatedly** over a finite interval $t \in [0, T]$, Arimoto's algorithm updates the command signal **from one iteration (trial) to the next**, using the error recorded in the previous trial.

### 3.1 First-Order System and D-Type Law

For $\dot y = -a y + b v$ with trial error $e_k(t) = y_d(t) - y_k(t)$:

$$
v_{k+1}(t) = v_k(t) + \frac{1}{b}\,\dot e_k(t)
$$

Assuming identical initial conditions at every trial, the error derivative satisfies

$$
\dot e_k(t) = a\int_0^t e^{-a(t-\tau)}\,\dot e_{k-1}(\tau)\,d\tau
\quad\Longrightarrow\quad
|\dot e_k(t)| \le D\,\frac{(at)^k}{k!} \xrightarrow[k\to\infty]{} 0
$$

with $D = \max_{t\in[0,T]}|\dot e_0(t)|$, which gives uniform convergence on $[0, T]$. For 2nd-order systems the law is extended to $v_{k+1} = v_k + \alpha\ddot e_k + \beta\dot e_k + \gamma e_k$.

### 3.2 Worked Example: Mass–Spring–Damper

$$
m\ddot x_k + b\dot x_k + k_s x_k = u_k(t) + d(t), \qquad t \in [0, 2\ \text{s}]
$$

| Parameter | Value |
| :--- | :--- |
| Mass $m$ | 1.0 kg |
| Damping $b$ | 2.0 N·s/m |
| Stiffness $k_s$ | 10.0 N/m |
| Unknown periodic disturbance $d(t)$ | $5\sin(2\pi t)$ |
| Desired trajectory $x_d(t)$ | $\sin(\pi t)$ |
| Initial condition (perfect reset each trial) | $x_k(0) = 0,\ \dot x_k(0) = \pi$ m/s |
| Learning gain $\Gamma$ | 0.8 |
| Number of iterations | 100 |
| Time step (explicit Euler) | $10^{-3}$ s |

**D-type learning law:**

$$
u_{k+1}(t) = u_k(t) + \Gamma\,\dot e_k(t)
$$

**Convergence condition (contraction):**

$$
\|1 - CB\,\Gamma\| < 1 \;\Longrightarrow\; \left|1 - \frac{\Gamma}{m}\right| < 1 \;\Longrightarrow\; 0 < \Gamma < 2m
$$

With $m = 1$ kg, $\Gamma = 0.8$ satisfies the criterion.

**Learning cycle:**

1. **Trial 1 ($k=1$):** $u_1(t) = 0$. The disturbance causes large deviations; $e_1(t)$ and $\dot e_1(t)$ are stored.
2. **Off-line update:** $u_2(t) = u_1(t) + \Gamma\,\dot e_1(t)$, applied sample-by-sample at the same time instants.
3. **Trial 2 ($k=2$):** the block is reset and $u_2(t)$ acts as **feedforward**, pushing exactly where the disturbance deviates the system. The error drops significantly.
4. **Asymptotic convergence ($k\to\infty$):** $e_k \to 0$ and $u_k(t) \to u^*(t)$, with

$$
u^*(t) = m\ddot x_d + b\dot x_d + k_s x_d - d(t)
$$

The learned signal therefore synthesizes the complete inverse dynamics **and cancels the disturbance** without measuring it or identifying $m$, $b$, $k_s$.

**Script outputs:** (i) trajectory of the block for selected iterations (1, 3, 6 and 15) against $x_d$; (ii) RMS error versus iteration number.

---

## 🎬 Post-Processing and Animation

### 1. Main Animation Script (`anima_simulink_2.m`)

Connects the results stored in the Simulink output structure (`out`) to the rendering tool:

```matlab
syms th1 th2 'real'

% Denavit-Hartenberg parameters and joint type (1 = revolute)
tdh_n = [1 0 0 th1; 1 0 0 th2];
info = [1; 1];

% Extract data logged by Simulink
qout = out.logsout{1}.Values.Data;
tout = out.tout;
xd = out.logsout{2}.Values.Data;
yd = out.logsout{3}.Values.Data;

ref = [xd yd];

% Call the animation function
dinanima_ref(tdh_n, info, qout, tout, 1, ref');
```

### 2. Function `dinanima_ref.m`

Animates any generic serial manipulator with $n$ DOF from the joint trajectories $q_{out}(t)$, recomputing the numerical forward kinematics frame by frame.

```matlab
dinanima_ref(tdh0, info, qout, tout, rastro, ref)
```

| Parameter | Type / Size | Description |
| :--- | :--- | :--- |
| `tdh0` | $n \times 4$ matrix | Standard Denavit–Hartenberg table $[a, \alpha, d, \theta]$ with the fixed geometric parameters. |
| `info` | $n \times 1$ vector | Joint type: `1` = **revolute** ($\theta_i$ variable), `2` = **prismatic** ($d_i$ variable). |
| `qout` | $N_t \times n$ matrix | Executed joint positions from the simulation. |
| `tout` | $N_t \times 1$ vector | Simulation time instants. |
| `rastro` | scalar *(optional)* | Enables the end-effector trail: `1` = on (default), `0` = off. |
| `ref` | matrix *(optional)* | Reference trajectory plotted in green. |

**Formats accepted by `ref`:**
1. **Joint space ($N_t \times n$):** forward kinematics is applied point by point to obtain the Cartesian curve.
2. **3D Cartesian ($3 \times N_t$ or $N_t \times 3$):** desired path as $[x, y, z]^\top$.
3. **2D Cartesian ($2 \times N_t$ or $N_t \times 2$):** desired path as $[x, y]^\top$; $z = 0$ is filled in automatically.

**Rendered elements:**
* 🟢 **Desired reference:** green dashed curve (`g--`) with a green circular marker (`go`) at the target position at the current time $t$.
* 🔴 **Actual trail:** continuous red curve (`r-`) traced by the end-effector up to time $t$.
* 🔵 **Robot structure:** blue lines with black joint markers showing the links and configuration at each frame.

---

## 🚀 How to Run

### Prerequisites
* **MATLAB** (R2020b or later), **Simulink** and the **Symbolic Math Toolbox** (CTC and adaptive models, animation)
* **Python 3** with `numpy` and `matplotlib` (Arimoto ILC)

### Steps

1. Clone the repository:
   ```bash
   git clone https://github.com/your-user/controle-ctc-simulink.git
   cd controle-ctc-simulink
   ```

2. **CTC:** open MATLAB, add the project folder to the Path and run `controle_ctc_2dof.slx`.

3. **Adaptive control:** define the model parameters in the workspace, then run the desired model.
   * `Controle_Adaptativo_1.slx` (error-based) needs `m`, `Kp`, `Kd`, `Gamma`.
   * `Controle_Adaptativo_2.slx` (Slotine–Li) needs `m`, `Kd`, `Lambda`, `Gamma`.

   Example (the values from the numerical example above):
   ```matlab
   m = 2; Kp = 100; Kd = 20; Gamma = 5; Lambda = 5;
   ```
   Compare the logged signals `qtil` and `pih` between the two models: Algorithm 1 settles at a nonzero steady-state error with a biased estimate, while Slotine–Li drives $\tilde q \to 0$.

4. **Animation (CTC):** after the simulation ends, run in the Command Window:
   ```matlab
   run('anima_simulink_2.m')
   ```

5. **Arimoto ILC:**
   ```bash
   pip install numpy matplotlib
   python Arimoto.py
   ```

---

## 📚 References

* FERNANDES, Diogo Lopes. *Controle de Torque Computado em Manipuladores Robóticos - Controle de Trajetória*. Didactic note, 2026.
* FERNANDES, Diogo Lopes. *Controle Adaptativo e por Aprendizado em Sistemas Robóticos*. Didactic note, October 2026.
* SPONG, Mark W.; HUTCHINSON, Seth; VIDYASAGAR, M. *Robot Modeling and Control*. John Wiley & Sons, 2005.
* SLOTINE, J.-J. E.; LI, W. On the adaptive control of robot manipulators. *The International Journal of Robotics Research*, 6(3), 49–59, 1987.
* ARIMOTO, S.; KAWAMURA, S.; MIYAZAKI, F. Bettering operation of robots by learning. *Journal of Robotic Systems*, 1(2), 123–140, 1984.
* LUH, J. Y.; WALKER, M. W.; PAUL, R. P. (1980). On-line computational scheme for mechanical manipulators. *ASME Journal of Dynamic Systems, Measurement, and Control*, 102(2), 69–76.
