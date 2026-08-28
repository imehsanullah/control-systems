# control systems: PID, LQR, and MPC

Both drones follow a path shaped like a sideways number eight: **∞**. This is
often called a *figure-eight trajectory*. It is simply two connected loops—the
drone flies the right loop, crosses the center, flies the left loop, and returns
to the center.

## Run

```bash

conda env create -f environment.yml
conda activate mujoco-drones

# macOS (use `python` instead of `mjpython` on Linux/Windows)
mjpython main.py --drone skydio --controller pid
mjpython main.py --drone skydio --controller lqr
mjpython main.py --drone skydio --controller mpc
```

Use `--drone crazyflie` for the other drone. Rebuild all GIFs with:

```bash
python assets/record_gifs.py
```

Yellow dots are the reference path. The green dot is the current target.

## Shared model

All controllers receive the state

$$x=[p_x,p_y,p_z,v_x,v_y,v_z]^T$$

where:

- $x$ is the six-value drone state.
- $p_x,p_y,p_z$ are positions in metres.
- $v_x,v_y,v_z$ are velocities in metres per second.
- $T$ means transpose, turning the listed values into a column vector.

The controllers return acceleration

$$u=[a_x,a_y,a_z]^T,$$

where $u$ is the control command and $a_x,a_y,a_z$ are desired accelerations
in metres per second squared. With control period
$\Delta t=0.05\text{ s}$, the prediction model is

$$p_{k+1}=p_k+\Delta t\,v_k+\frac{1}{2}\Delta t^2u_k$$

$$v_{k+1}=v_k+\Delta t\,u_k$$

where:

- $k$ is the current discrete control step and $k+1$ is the next step.
- $p_k$ and $v_k$ are the current 3D position and velocity.
- $u_k$ is the 3D acceleration applied during step $k$.
- $\Delta t$ is the 0.05 s controller update period.

or $x_{k+1}=Ax_k+Bu_k$, with

$$
A=
\begin{bmatrix}
I_3 & \Delta t\,I_3 \\
0 & I_3
\end{bmatrix},
\qquad
B=
\begin{bmatrix}
\frac{1}{2}\Delta t^2 I_3 \\
\Delta t\,I_3
\end{bmatrix}.
$$

where:

- $A$ is the $6\times6$ state-transition matrix.
- $B$ is the $6\times3$ control-input matrix.
- $I_3$ is the $3\times3$ identity matrix and $0$ is a zero matrix.
- $x_k$ and $x_{k+1}$ are the current and predicted states.

Every acceleration component is limited to
$[-6,6]\text{ m/s}^2$. MuJoCo runs at $0.01\text{ s}$ and receives force

$$F=m\left(u+[0,0,9.81]^T\right).$$

where $F$ is the 3D actuator force in newtons, $m$ is drone mass in kilograms,
$u$ is controller acceleration, and $[0,0,9.81]^T$ compensates for downward
gravity.

The complete feedback loop is:

```text
sideways-8 reference -> controller -> desired acceleration
         ^                                 |
         |                                 v
measured position/velocity <- MuJoCo <- F = m(u + g)
```

In this diagram, $g=[0,0,9.81]^T\text{ m/s}^2$ is upward gravity
compensation.

Thus, PID, LQR, and MPC produce the same kind of command. They differ only in
how they calculate that command from the measured state and reference.

Only the mass and reference size differ between drones:

| Parameter | Skydio X2 | Crazyflie 2 |
|---|---:|---:|
| Mass used | 1.325 kg | 0.028 kg |
| Visual envelope | 0.660 × 0.560 × 0.203 m | 0.115 × 0.115 × 0.0295 m |
| Opposite motor spacing | 0.584 m | 0.0919 m |
| Joint ranges: x / y / z | ±2.3 / ±1.3 / 0–1.8 m | ±0.9 / ±0.5 / 0–1.05 m |
| Path radius $r$ | 2.0 m | 0.8 m |
| Nominal path height $h$ | 1.4 m | 0.75 m |

Dimensions and inertia are not part of this simplified controller model because
rotation is disabled. A full quadrotor controller would need attitude, inertia,
motor thrust, and torque.

## Reference path: a sideways eight (∞)

Viewed from above, the movement is:

```text
center -> right loop -> center -> left loop -> center
```

This path is useful for learning because the controller must change x and y
together, reverse direction at the center, and handle curved motion. A small
height change makes it a 3D path rather than a flat one.

Takeoff lasts 1.5 s, the sideways-8 path lasts 7 s, landing lasts 1.5 s, and
the drone pauses for 1 s. Smoothstep $s(\tau)=3\tau^2-2\tau^3$ prevents an
abrupt start and stop. During the two loops, $\theta=2\pi s(\tau)$ and

$$p_r(\theta)=
\begin{bmatrix}
r\sin\theta\\
\frac{r}{2}\sin(2\theta)\\
h\left(1+0.18\sin(2\theta)\right)
\end{bmatrix}.$$

where:

- $p_r=[x_r,y_r,z_r]^T$ is the desired 3D position.
- $\tau\in[0,1]$ is normalized time through the curved path.
- $s(\tau)$ smoothly changes path progress from 0 to 1.
- $\theta=2\pi s(\tau)$ is progress around the loops in radians.
- $\pi\approx3.14159$ is the circle constant.
- $r$ controls horizontal path size and $h$ is normal flight height.
- $\sin(\theta)$ and $\sin(2\theta)$ create the two connected loops.
- $0.18$ makes altitude vary by 18% around $h$.

## PID

PID is a model-free feedback controller: it does not predict the drone or know
its mass. It only observes tracking error and combines three reactions:

- **Proportional (P):** responds to the current error. Larger error produces a
  larger correction.
- **Integral (I):** remembers past error. It removes persistent offset, but too
  much can cause overshoot.
- **Derivative (D):** reacts to how quickly the error changes. It adds damping
  and reduces oscillation.

### General equation

For reference $r(t)$, measured output $y(t)$, and error $e(t)=r(t)-y(t)$,

$$u(t)=K_Pe(t)+K_I\int_0^t e(\tau)\,d\tau+K_D\frac{de(t)}{dt}.$$

where:

- $u(t)$ is the controller output at time $t$.
- $r(t)$ is the desired reference and $y(t)$ is the measured output.
- $e(t)=r(t)-y(t)$ is reference minus measured output.
- $K_P$, $K_I$, and $K_D$ are proportional, integral, and derivative gains.
- $t$ is current time and $\tau$ is the integration-time variable.
- $\int_0^t e(\tau)d\tau$ is accumulated past error.
- $de(t)/dt$ is the current rate of change of error.

### Applied to our drone

The measured output is position $y=p$, so $e_p=p_r-p$. Because velocity is
available from MuJoCo, the derivative error is taken directly as
$e_v=v_r-v$. The discrete implementation is

$$u=K_Pe_p+K_I\int e_p\,dt+K_De_v.$$

where:

- $u=[a_x,a_y,a_z]^T$ is desired drone acceleration.
- $e_p=p_r-p$ is desired minus measured position.
- $e_v=v_r-v$ is desired minus measured velocity and supplies the derivative term.
- $p_r,v_r$ are reference position and velocity; $p,v$ are MuJoCo measurements.
- $dt$ indicates integration over time.

Parameters: $K_P=7$, $K_I=0.15$, $K_D=4.5$. The integral is limited to
$[-2,2]$ per axis to reduce windup, and $u$ is clipped to $\pm6\text{ m/s}^2$.
That acceleration is converted using the shared force equation above. PID is
simple and cheap, but it cannot see future turns and its gains must be tuned
manually.

| Skydio X2 | Crazyflie 2 |
|---|---|
| ![Skydio PID](assets/gifs/skydio_pid.gif) | ![Crazyflie PID](assets/gifs/crazyflie_pid.gif) |

## LQR

LQR is a model-based state-feedback controller. Unlike three independent PID
terms, it uses the complete linear state model $x_{k+1}=Ax_k+Bu_k$. It balances
tracking error against control effort and calculates one constant, optimal gain
matrix $K$ before simulation starts.

### General equations

For a discrete linear system

$$x_{k+1}=Ax_k+Bu_k,$$

where $x_k$ is the current state, $x_{k+1}$ is the next state, $u_k$ is the
control input, and $A,B$ describe how the system responds.

LQR minimizes

$$J=\sum_{k=0}^{\infty}\left(x_k^TQx_k+u_k^TRu_k\right),$$

where:

- $J$ is total cost; LQR chooses control inputs that minimize it.
- $Q$ weights state deviations and $R$ weights control effort.
- $x_k^TQx_k$ is state cost and $u_k^TRu_k$ is control cost at step $k$.
- Superscript $T$ means matrix transpose.
- $\sum_{k=0}^{\infty}$ means the cost is considered over all future steps.

The discrete algebraic Riccati equation is

$$P=A^TPA-A^TPB(R+B^TPB)^{-1}B^TPA+Q,$$

where $P$ is the Riccati solution containing future-cost information,
$A,B$ are system matrices, $Q,R$ are cost weights, and $^{-1}$ means matrix
inverse.

The optimal feedback is

$$K=(R+B^TPB)^{-1}B^TPA,\qquad u_k=-Kx_k.$$

where $K$ is the optimal feedback-gain matrix and $u_k=-Kx_k$ turns the current
state into a correcting control command. The minus sign drives the state toward
zero.

### Applied to our drone

For trajectory tracking, the controller uses error $e=x-x_r$ instead of $x$:

$$u=-Ke.$$

where $e=x-x_r$ is actual state minus desired state, $x_r$ is the reference
state, and $u$ is desired acceleration. Driving $e$ toward zero makes the drone
follow the path.

Here $x=[p_x,p_y,p_z,v_x,v_y,v_z]^T$.

The diagonal entries of $Q$ are $[35,35,45,3,3,4]$ (all other entries are
zero), and $R=0.25I_3$.

The first three $Q$ values weight x/y/z position error; the last three weight
x/y/z velocity error. $I_3$ is the $3\times3$ identity matrix, so $R$ applies
the same control-effort penalty to $a_x,a_y,a_z$.

The gain used by the code is

$$K=\begin{bmatrix}
10.1931&0&0&5.4122&0&0\\
0&10.1931&0&0&5.4122&0\\
0&0&11.3944&0&0&5.8591
\end{bmatrix}.$$

The three rows produce $a_x,a_y,a_z$. The six columns multiply
$p_x,p_y,p_z,v_x,v_y,v_z$, respectively. Z gains differ because Z error has a
higher weight in $Q$.

At runtime LQR only performs the matrix multiplication $u=-Ke$, so it is very
fast. Its output is treated as desired x/y/z acceleration, clipped to
$\pm6\text{ m/s}^2$, and converted to force. However, LQR reacts to the current
reference rather than previewing the future path, and clipping is not part of
the original unconstrained LQR solution.

| Skydio X2 | Crazyflie 2 |
|---|---|
| ![Skydio LQR](assets/gifs/skydio_lqr.gif) | ![Crazyflie LQR](assets/gifs/crazyflie_lqr.gif) |

## MPC

MPC is also model-based, but it repeatedly plans ahead instead of using one
fixed gain. At every control update it:

1. Measures the current state.
2. Predicts the drone and reference over $N=20$ steps, or 1 second.
3. Finds the best bounded sequence of 20 acceleration commands.
4. Applies only the first command and solves again from the next measured state.

### General equations

For a model $x_{k+1}=f(x_k,u_k)$, where $f$ predicts the next state from the
current state and input, MPC solves a finite-horizon problem:

$$\min_{u_0,\ldots,u_{N-1}}
\sum_{k=1}^{N}\lVert x_k-x_{r,k}\rVert_Q^2
+\sum_{k=0}^{N-1}\lVert u_k\rVert_R^2$$

where:

- $\min$ means MPC chooses the control sequence with the smallest cost.
- $N$ is the number of future steps in the prediction horizon.
- $u_0,\ldots,u_{N-1}$ is the candidate future control sequence.
- $x_k$ is a predicted state and $x_{r,k}$ is its desired reference.
- $\lVert z\rVert_Q^2=z^TQz$ is a weighted squared error for any vector $z$.
- $Q$ penalizes tracking error and $R$ penalizes control effort.

The optimization is subject to the model and state/input constraints. Only
$u_0$ is applied; this repeated replanning is called a **receding horizon**.

### Applied to our drone

Our model is the linear double integrator $x_{k+1}=Ax_k+Bu_k$. At every update,
MPC predicts $N=20$ steps (1 second) and solves

$$\min_{u_0,\ldots,u_{N-1}}
\sum_{k=1}^{N}(x_k-x_{r,k})^TQ(x_k-x_{r,k})
+\sum_{k=0}^{N-1}u_k^TRu_k$$

where $N=20$, $(x_k-x_{r,k})$ is predicted tracking error, $Q$ weights state
error, $u_k$ is predicted acceleration, and $R$ discourages excessive
acceleration.

It is subject to $x_{k+1}=Ax_k+Bu_k$ and $-6\le u_k\le6$. The first equation
is the drone prediction model; the second limits every acceleration component to
$\pm6\text{ m/s}^2$. MPC uses the same $Q$ and $R$ as LQR. The first optimized
x/y/z acceleration is converted to force and applied for 0.05 s. MPC can
directly respect acceleration limits and anticipate turns, but solving an
optimization problem each update costs more computation.

| Skydio X2 | Crazyflie 2 |
|---|---|
| ![Skydio MPC](assets/gifs/skydio_mpc.gif) | ![Crazyflie MPC](assets/gifs/crazyflie_mpc.gif) |

## One-cycle tracking error

RMSE means root-mean-square position error; lower is better.

| Controller | Skydio RMSE | Crazyflie RMSE |
|---|---:|---:|
| PID | 0.361 m | 0.119 m |
| LQR | 0.324 m | 0.091 m |
| MPC | 0.072 m | 0.030 m |

PID and LQR react to the current reference. MPC performs better on this fast
path because it also sees the upcoming reference.

## Files

- `controllers.py`: trajectory, PID, LQR, and MPC.
- `main.py`: viewer and shared control loop.
- `models/`: the two MuJoCo models.
- `assets/record_gifs.py`: deterministic offscreen GIF generation.
