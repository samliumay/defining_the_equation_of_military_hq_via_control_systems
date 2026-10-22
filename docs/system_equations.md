# System equations

The full model, built from the bottom up. The vehicle is a **planar drone** (moves sideways $x$, up/down $z$, tilts $\theta$). The 3D version has the same structure with more equations.

## 1. Plant: the drone's physics

State and inputs:

$$
s = [x,\ z,\ \theta,\ \dot x,\ \dot z,\ \dot\theta]^\top, \qquad u = [T,\ \tau]^\top
$$

$T$ is total thrust, $\tau$ is torque, $w$ is a wind force, $J$ is rotational inertia.

$$
\begin{aligned}
m\,\ddot x &= T\sin\theta + w \\
m\,\ddot z &= T\cos\theta - m g \\
J\,\ddot\theta &= \tau
\end{aligned}
$$

The rotors produce the inputs ($\Omega_i$ rotor speed, $L$ arm length, $k$ rotor constant):

$$
T = k(\Omega_1^2 + \Omega_2^2), \qquad \tau = k L(\Omega_1^2 - \Omega_2^2)
$$

## 2. Partial derivatives at hover

Take the sideways acceleration as a function of the two inputs, $\ddot x = f(T,\theta) = \frac{T}{m}\sin\theta$:

$$
\frac{\partial f}{\partial \theta} = \frac{T}{m}\cos\theta, \qquad \frac{\partial f}{\partial T} = \frac{\sin\theta}{m}
$$

At hover ($\theta = 0$, $T = mg$):

$$
\frac{\partial \ddot x}{\partial \theta} = g \approx 9.81, \qquad \frac{\partial \ddot x}{\partial T} = 0
$$

- Near hover $\ddot x \approx g\,\theta$: tilting 1° (0.0175 rad) gives about 0.17 m/s² of sideways acceleration.
- When level, extra thrust does not push sideways at all.

Doing the same for $\ddot z$ shows that **thrust controls height and tilt controls sideways motion**, which is why autopilots use separate loops for altitude and position.

## 3. Inner loop: autopilot (~400 Hz)

$$
\begin{aligned}
\text{Altitude:} \quad & a_z = K_{pz}(z_{ref} - z) - K_{dz}\,\dot z, \qquad T = \frac{m(g + a_z)}{\cos\theta} \\
\text{Position:} \quad & \theta_{des} = \frac{K_{px}(x_{ref} - x) - K_{dx}\,\dot x}{g} \\
\text{Attitude:} \quad & \tau = J\left[K_{p\theta}(\theta_{des} - \theta) - K_{d\theta}\,\dot\theta\right]
\end{aligned}
$$

- Dividing by $\cos\theta$ cancels the tilt, so the vertical motion becomes exactly $\ddot z = a_z$. The tilt is capped ($|\theta| \le 30°$) so this never blows up.
- The $\theta_{des}$ line inverts $\ddot x \approx g\,\theta$ from section 2.

## 4. Closed inner loop

Substituting the controllers into the physics:

$$
\begin{aligned}
\ddot z + K_{dz}\,\dot z + K_{pz}\,z &= K_{pz}\,z_{ref} \\
\ddot \theta + K_{d\theta}\,\dot \theta + K_{p\theta}\,\theta &= K_{p\theta}\,\theta_{des} \\
\ddot x + K_{dx}\,\dot x + K_{px}\,x &\approx K_{px}\,x_{ref} + \frac{w}{m}
\end{aligned}
$$

Each line has the standard second-order form

$$
\ddot y + 2\zeta\omega\,\dot y + \omega^2 y = \omega^2 y_{ref}, \qquad \omega = \sqrt{K_p}, \quad \zeta = \frac{K_d}{2\sqrt{K_p}}
$$

| Loop | $K_p$ | $K_d$ | $\omega$ (rad/s) | $\zeta$ |
|------|-------|-------|------------------|---------|
| Position $x$ | 4 | 4 | 2 | 1 (no overshoot) |
| Attitude $\theta$ | 100 | 20 | 10 | 1 |

Attitude is 5× faster than position, which satisfies the timescale-separation rule (the $x$ line assumes $\theta \approx \theta_{des}$).

A constant wind leaves a steady offset $x - x_{ref} = \frac{w}{m K_{px}}$. An integral term (PD → PID) removes it.

## 5. Middle loop: node agent (~10 Hz, step $k$)

$$
\begin{aligned}
o[k] &= h(s) + v && \text{sensor reading + noise} \\
p_{cand}[k] &= \pi_{node}(o[k],\ I_{held}) && \text{model proposes a waypoint} \\
p_{ref}[k] &= \mathrm{Proj}_S(p_{cand}[k]) = \arg\min_{p \in S} \lVert p - p_{cand}[k] \rVert && \text{safety filter}
\end{aligned}
$$

$\mathrm{Proj}_S$ moves the proposal to the nearest point inside the safe set $S$ (geofence, altitude limits, battery range). $p_{ref} = (x_{ref}, z_{ref})$ feeds the inner loop.

## 6. Outer loop: HQ (~1 Hz, step $n$)

$$
\begin{aligned}
y[n] &= \delta[n]\, s(t - d), \qquad \delta[n] = 0 \text{ with probability } p \\
I[n] &= \pi_{HQ}(y[n]) \\
I_{held} &= \begin{cases} I_{last} & \text{if age} < T_{out} \\ I_{safe} & \text{otherwise (loiter / return home)} \end{cases}
\end{aligned}
$$

Security checks on the node:

- accept $I$ only if its signature verifies;
- raise an alarm if the residual $\lVert o - \hat o \rVert > \varepsilon$ (measured vs. predicted observation).

## 7. The whole system

$$
\begin{aligned}
\dot s &= f\big(s,\ \kappa(s, p_{ref}),\ w\big) && \text{continuous physics + autopilot } \kappa \\
p_{ref} &= \mathrm{Proj}_S\big(\pi_{node}(h(s) + v,\ I_{held})\big) && \text{every } 0.1\text{ s} \\
I &= \pi_{HQ}\big(\delta\, s(t - d)\big) && \text{every } 1\text{ s}
\end{aligned}
$$

This is a **hybrid system**: continuous physics with controllers sampled at three different rates.

## 8. Linearisation at hover

With $u = [\delta T,\ \tau]^\top$ the deviation from hover thrust $mg$, $f$ linearises to $\dot s = A s + B u$:

$$
A = \begin{bmatrix}
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & g & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix},
\qquad
B = \begin{bmatrix}
0 & 0 \\ 0 & 0 \\ 0 & 0 \\ 0 & 0 \\ 1/m & 0 \\ 0 & 1/J
\end{bmatrix}
$$

The non-trivial entries are exactly the partial derivatives (the Jacobian): $\partial\ddot x/\partial\theta = g$, $\partial\ddot z/\partial T = 1/m$, $\partial\ddot\theta/\partial\tau = 1/J$. This linear form is what LQR and MPC are designed on.

## Where else derivatives appear

- **PID tuning:** define a cost $J$ (e.g. total tracking error) and adjust gains with $\partial J/\partial K_p$, $\partial J/\partial K_d$.
- **State estimation:** the Extended Kalman Filter fusing GPS and inertial data uses these same Jacobians.
- **The HQ and node models:** training them is gradient descent, i.e. backpropagated partial derivatives.

> Exercise: what is $\partial\ddot x/\partial T$ when the drone is tilted 30° instead of level?
