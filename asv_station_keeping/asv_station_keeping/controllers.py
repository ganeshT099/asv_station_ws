"""Pluggable station-keeping control laws (pure Python + numpy, no ROS).

Every controller maps the body-frame pose error and body velocities to a
body-frame wrench ``(Fx [N], Fy [N], Mz [N m])``.  The ROS node and the
offline simulator share this module, so a law tuned offline is the very code
that flies in Gazebo.  Actuation limits, thrust allocation, the actuator
model and the disturbance observer are shared downstream and identical for
every law, which keeps the comparison fair.

Axis model used by the model-based laws (per axis, near station)::

    m dv/dt = F - b v - c |v| v + d        dp/dt = v      p = -error

Laws: ``pid`` (baseline), ``lqr`` (LQR with integral action), ``smc``
(sliding mode with boundary layer) and ``mpc`` (constrained receding-horizon
control with integral action).
"""

from dataclasses import dataclass
import math

import numpy as np

# Thruster mixer constants (kept in step with twist_to_thruster.py).
_R = 2.0 ** -0.5
FORCE_PER_UNIT = 4.0 * _R
MOMENT_PER_UNIT = 4.0 * _R * (0.25 - 0.175)

CONTROLLER_NAMES = ('pid', 'lqr', 'smc', 'mpc')


@dataclass(frozen=True)
class VehicleSpec:
    """Nominal 3-DOF plant used by the model-based controllers.

    Mass is the sum of the URDF link masses; yaw inertia is a parallel-axis
    estimate; damping follows the Hydrodynamics block of the URDF.  They are
    deliberately *nominal*: the experiments perturb the true plant to test
    robustness to model error.
    """

    mass: tuple = (41.84, 41.84)
    inertia_z: float = 2.3
    linear_drag: tuple = (0.60, 0.80, 0.50)
    quadratic_drag: tuple = (2.0, 3.0, 5.0)

    def axis(self, index):
        """Return (inertia, linear drag, quadratic drag) of axis 0/1/2."""
        inertia = (self.mass[0], self.mass[1], self.inertia_z)[index]
        return (inertia, self.linear_drag[index],
                self.quadratic_drag[index])


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    """Clamp ``value`` into ``[low, high]``."""
    return max(low, min(high, value))


# ---------------------------------------------------------------------------
# Linear-algebra helpers (numpy only)
# ---------------------------------------------------------------------------
def matrix_exponential(matrix, order=14):
    """Return ``expm(matrix)`` by scaling-and-squaring with a Taylor series."""
    matrix = np.asarray(matrix, dtype=float)
    norm = np.linalg.norm(matrix, np.inf)
    squarings = max(0, int(math.ceil(math.log2(max(norm, 1e-12)))) + 1)
    scaled = matrix / (2.0 ** squarings)
    result = np.eye(matrix.shape[0])
    term = np.eye(matrix.shape[0])
    for k in range(1, order + 1):
        term = term @ scaled / k
        result = result + term
    for _ in range(squarings):
        result = result @ result
    return result


def discretise(a_matrix, b_matrix, dt):
    """Exact zero-order-hold discretisation of ``x' = A x + B u``."""
    n = a_matrix.shape[0]
    m = b_matrix.shape[1]
    block = np.zeros((n + m, n + m))
    block[:n, :n] = a_matrix * dt
    block[:n, n:] = b_matrix * dt
    exp = matrix_exponential(block)
    return exp[:n, :n], exp[:n, n:]


def solve_dare(a_d, b_d, q_matrix, r_matrix, iterations=4000, tol=1e-11):
    """Solve the discrete algebraic Riccati equation by fixed-point iteration.

    Returns ``(P, K)`` with ``u = -K x`` the optimal state feedback.
    """
    p_matrix = q_matrix.copy()
    gain = np.zeros((b_d.shape[1], a_d.shape[0]))
    for _ in range(iterations):
        s_matrix = r_matrix + b_d.T @ p_matrix @ b_d
        gain = np.linalg.solve(s_matrix, b_d.T @ p_matrix @ a_d)
        updated = (q_matrix + a_d.T @ p_matrix @ (a_d - b_d @ gain))
        if np.max(np.abs(updated - p_matrix)) < tol:
            p_matrix = updated
            break
        p_matrix = updated
    s_matrix = r_matrix + b_d.T @ p_matrix @ b_d
    gain = np.linalg.solve(s_matrix, b_d.T @ p_matrix @ a_d)
    return p_matrix, gain


def axis_model(inertia, linear_drag, dt, lag=0.35):
    """Return discrete ``(A, B)`` of the integral-augmented axis model.

    State is ``[p, v, i, f]``: position relative to the target, velocity,
    integral of ``p`` and the force actually delivered, which follows the
    commanded force through a first-order lag ``lag`` (motor + ESC + the
    command slew limit).  Designing with the lag in the model is what keeps
    the optimal gains from exciting the actuators.
    """
    a_matrix = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, -linear_drag / inertia, 0.0, 1.0 / inertia],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, -1.0 / lag],
    ])
    b_matrix = np.array([[0.0], [0.0], [0.0], [1.0 / lag]])
    return discretise(a_matrix, b_matrix, dt)


class ForceLag:
    """Open-loop estimate of the delivered force (first-order lag)."""

    def __init__(self, lag):
        self.lag = lag
        self.value = 0.0

    def update(self, command, dt):
        self.value += dt / (self.lag + dt) * (command - self.value)

    def reset(self):
        self.value = 0.0


# ---------------------------------------------------------------------------
# Controller interface
# ---------------------------------------------------------------------------
class Controller:
    """Base class: stateful law producing a body-frame wrench."""

    name = 'base'

    def reset(self):
        """Forget accumulated state (integrators, plans)."""

    def compute(self, errors, rates, dt, saturated=False, limits=None):
        """Return ``(Fx, Fy, Mz)`` for body errors and body velocities.

        ``errors`` is target minus actual ``(ex, ey, eyaw)`` in the body
        frame, ``rates`` is ``(u, v, r)``, ``saturated`` freezes integrators
        (anti-windup) and ``limits`` is ``(Fx_max, Fy_max, Mz_max)``.
        """
        raise NotImplementedError


class PidController(Controller):
    """Baseline PID matching the node's inline control law, in SI units.

    Gains are given in the node's mixer units (the ``kp_x`` family from the
    YAML file) and converted to N and N m with the thruster geometry.
    """

    name = 'pid'

    def __init__(self, gains=None, integral_limit=1.5, integral_leak=0.05,
                 yaw_enabled=True):
        defaults = {
            'kp': (5.0, 5.0, 8.0), 'ki': (0.6, 0.6, 0.4),
            'kd': (5.0, 5.0, 12.0)}
        self.gains = dict(defaults)
        self.gains.update(gains or {})
        self.integral_limit = integral_limit
        self.integral_leak = integral_leak
        self.yaw_enabled = yaw_enabled
        self.scale = (FORCE_PER_UNIT, FORCE_PER_UNIT, MOMENT_PER_UNIT)
        self.integral = [0.0, 0.0, 0.0]

    def reset(self):
        self.integral = [0.0, 0.0, 0.0]

    def compute(self, errors, rates, dt, saturated=False, limits=None):
        wrench = []
        for i in range(3):
            if not saturated:
                decayed = self.integral[i] * max(
                    0.0, 1.0 - self.integral_leak * dt)
                self.integral[i] = clamp(
                    decayed + errors[i] * dt,
                    -self.integral_limit, self.integral_limit)
            demand = (self.gains['kp'][i] * errors[i]
                      + self.gains['ki'][i] * self.integral[i]
                      - self.gains['kd'][i] * rates[i])
            if i == 2 and not self.yaw_enabled:
                demand = 0.0
            wrench.append(demand * self.scale[i])
        return tuple(wrench)


class LqrController(Controller):
    """LQR with integral action, one decoupled loop per axis.

    ``q_pos``/``q_vel``/``q_int`` weight position, velocity and integrated
    error; ``r_weight`` penalises force.  Gains come from the discrete
    Riccati equation of the nominal model, so tuning is by cost weights.
    """

    name = 'lqr'

    def __init__(self, vehicle=None, design_dt=0.05,
                 q_pos=(40.0, 40.0, 60.0), q_vel=(90.0, 90.0, 40.0),
                 q_int=(10.0, 10.0, 4.0), r_weight=(1.0, 1.0, 1.0),
                 actuator_lag=0.35, integral_limit=1.5, integral_leak=0.05,
                 yaw_enabled=True):
        self.vehicle = vehicle or VehicleSpec()
        self.lag = [ForceLag(actuator_lag) for _ in range(3)]
        self.integral_limit = integral_limit
        self.integral_leak = integral_leak
        self.yaw_enabled = yaw_enabled
        self.gains = []
        for i in range(3):
            inertia, linear, _quad = self.vehicle.axis(i)
            a_d, b_d = axis_model(inertia, linear, design_dt, actuator_lag)
            q_matrix = np.diag([q_pos[i], q_vel[i], q_int[i], 0.0])
            _p, gain = solve_dare(
                a_d, b_d, q_matrix, np.array([[r_weight[i]]]))
            self.gains.append(gain[0])
        self.integral = [0.0, 0.0, 0.0]

    def reset(self):
        self.integral = [0.0, 0.0, 0.0]
        for lag in self.lag:
            lag.reset()

    def compute(self, errors, rates, dt, saturated=False, limits=None):
        wrench = []
        for i in range(3):
            # p = -error, so integral of p is minus the integral of error.
            if not saturated:
                decayed = self.integral[i] * max(
                    0.0, 1.0 - self.integral_leak * dt)
                self.integral[i] = clamp(
                    decayed - errors[i] * dt,
                    -self.integral_limit, self.integral_limit)
            state = np.array(
                [-errors[i], rates[i], self.integral[i], self.lag[i].value])
            force = -float(self.gains[i] @ state)
            if limits is not None:
                force = clamp(force, -limits[i], limits[i])
            if i == 2 and not self.yaw_enabled:
                force = 0.0
            self.lag[i].update(force, dt)
            wrench.append(force)
        return tuple(wrench)


class SlidingModeController(Controller):
    """Sliding-mode law with equivalent control and a boundary layer.

    Surface ``s = v + lam p + lam_i integral(p)`` makes the sliding
    dynamics a stable second-order system; the switching term
    ``-k sat(s / phi)`` dominates bounded disturbances and model error.
    """

    name = 'smc'

    def __init__(self, vehicle=None, lam=(0.9, 0.9, 1.2),
                 lam_i=(0.12, 0.12, 0.10), switch_gain=(4.5, 4.5, 0.25),
                 boundary=(0.12, 0.12, 0.10), integral_limit=1.0,
                 integral_leak=0.05, yaw_enabled=True):
        self.vehicle = vehicle or VehicleSpec()
        self.lam = lam
        self.lam_i = lam_i
        self.switch_gain = switch_gain
        self.boundary = boundary
        self.integral_limit = integral_limit
        self.integral_leak = integral_leak
        self.yaw_enabled = yaw_enabled
        self.integral = [0.0, 0.0, 0.0]

    def reset(self):
        self.integral = [0.0, 0.0, 0.0]

    def compute(self, errors, rates, dt, saturated=False, limits=None):
        wrench = []
        for i in range(3):
            inertia, linear, quad = self.vehicle.axis(i)
            p = -errors[i]
            v = rates[i]
            if not saturated:
                decayed = self.integral[i] * max(
                    0.0, 1.0 - self.integral_leak * dt)
                self.integral[i] = clamp(
                    decayed + p * dt,
                    -self.integral_limit, self.integral_limit)
            surface = (v + self.lam[i] * p
                       + self.lam_i[i] * self.integral[i])
            # Equivalent control: cancel nominal drag and impose the
            # surface dynamics s' = 0.
            equivalent = (linear * v + quad * abs(v) * v
                          - inertia * (self.lam[i] * v
                                       + self.lam_i[i] * p))
            switching = -self.switch_gain[i] * clamp(
                surface / self.boundary[i], -1.0, 1.0)
            force = equivalent + switching
            if i == 2 and not self.yaw_enabled:
                force = 0.0
            wrench.append(force)
        return tuple(wrench)


class MpcController(Controller):
    """Constrained receding-horizon control with integral action.

    Each axis solves a small box-constrained QP over ``horizon`` steps of
    the integral-augmented linear model, so the force bound is respected
    *inside* the optimisation instead of being clipped afterwards.  The QP
    is solved by accelerated projected gradient (fixed iteration count, so
    its run time is deterministic).
    """

    name = 'mpc'

    def __init__(self, vehicle=None, horizon=16, step=0.25,
                 q_pos=(60.0, 60.0, 80.0), q_vel=(40.0, 40.0, 20.0),
                 q_int=(12.0, 12.0, 5.0), r_weight=(0.6, 0.6, 0.6),
                 r_delta=(0.05, 0.05, 0.05), iterations=80,
                 default_limits=(9.0, 9.0, 0.45), actuator_lag=0.35,
                 integral_limit=1.5, integral_leak=0.05, yaw_enabled=True):
        self.vehicle = vehicle or VehicleSpec()
        self.actuator_lag = actuator_lag
        self.lag = [ForceLag(actuator_lag) for _ in range(3)]
        self.horizon = horizon
        self.step = step
        self.iterations = iterations
        self.default_limits = default_limits
        self.integral_limit = integral_limit
        self.integral_leak = integral_leak
        self.yaw_enabled = yaw_enabled
        self.integral = [0.0, 0.0, 0.0]
        self.previous = [0.0, 0.0, 0.0]
        self.axes = [self._build_axis(i, q_pos[i], q_vel[i], q_int[i],
                                      r_weight[i], r_delta[i])
                     for i in range(3)]

    def _build_axis(self, index, q_pos, q_vel, q_int, r_weight, r_delta):
        """Condense the horizon problem: states X = Phi x0 + Gamma U."""
        inertia, linear, _quad = self.vehicle.axis(index)
        a_d, b_d = axis_model(inertia, linear, self.step, self.actuator_lag)
        n = self.horizon
        ns = a_d.shape[0]
        phi = np.zeros((ns * n, ns))
        gamma = np.zeros((ns * n, n))
        power = np.eye(ns)
        powers = []
        for k in range(n):
            power = a_d @ power
            powers.append(power.copy())
            phi[ns * k:ns * k + ns, :] = power
        for k in range(n):
            for j in range(k + 1):
                block = (powers[k - j - 1] if k - j >= 1 else np.eye(ns)) @ b_d
                gamma[ns * k:ns * k + ns, j] = block[:, 0]
        q_matrix = np.diag([q_pos, q_vel, q_int, 0.0])
        q_big = np.kron(np.eye(n), q_matrix)
        diff = np.eye(n) - np.eye(n, k=-1)
        hessian = (gamma.T @ q_big @ gamma
                   + r_weight * np.eye(n)
                   + r_delta * diff.T @ diff)
        lipschitz = float(np.max(np.linalg.eigvalsh(hessian)))
        return {'phi': phi, 'gamma': gamma, 'q_big': q_big,
                'hessian': hessian, 'lipschitz': lipschitz,
                'r_delta': r_delta, 'warm': np.zeros(n)}

    def reset(self):
        self.integral = [0.0, 0.0, 0.0]
        self.previous = [0.0, 0.0, 0.0]
        for lag in self.lag:
            lag.reset()
        for axis in self.axes:
            axis['warm'] = np.zeros(self.horizon)

    def _solve(self, axis, state, previous, bound):
        linear = (axis['gamma'].T @ axis['q_big'] @ (axis['phi'] @ state))
        # Move penalty couples the first step to the last applied force.
        linear[0] -= axis['r_delta'] * previous
        step = 1.0 / axis['lipschitz']
        # Warm start from the previous plan shifted by one step.
        u = np.concatenate([axis['warm'][1:], axis['warm'][-1:]])
        u = np.clip(u, -bound, bound)
        y = u.copy()
        t = 1.0
        for _ in range(self.iterations):
            grad = axis['hessian'] @ y + linear
            u_next = np.clip(y - step * grad, -bound, bound)
            t_next = 0.5 * (1.0 + math.sqrt(1.0 + 4.0 * t * t))
            y = u_next + ((t - 1.0) / t_next) * (u_next - u)
            u, t = u_next, t_next
        axis['warm'] = u
        return float(u[0])

    def compute(self, errors, rates, dt, saturated=False, limits=None):
        limits = limits or self.default_limits
        wrench = []
        for i in range(3):
            if not saturated:
                decayed = self.integral[i] * max(
                    0.0, 1.0 - self.integral_leak * dt)
                self.integral[i] = clamp(
                    decayed - errors[i] * dt,
                    -self.integral_limit, self.integral_limit)
            state = np.array(
                [-errors[i], rates[i], self.integral[i], self.lag[i].value])
            bound = max(float(limits[i]), 1e-6)
            force = self._solve(self.axes[i], state, self.previous[i], bound)
            if i == 2 and not self.yaw_enabled:
                force = 0.0
            self.lag[i].update(force, dt)
            self.previous[i] = force
            wrench.append(force)
        return tuple(wrench)


def make_controller(name, vehicle=None, **kwargs):
    """Build a controller by name (``pid``, ``lqr``, ``smc``, ``mpc``)."""
    name = str(name).lower()
    if name == 'pid':
        return PidController(**kwargs)
    if name == 'lqr':
        return LqrController(vehicle, **kwargs)
    if name == 'smc':
        return SlidingModeController(vehicle, **kwargs)
    if name == 'mpc':
        return MpcController(vehicle, **kwargs)
    raise ValueError(
        f'unknown controller "{name}"; choose from {CONTROLLER_NAMES}')
