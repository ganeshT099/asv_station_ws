"""Fast 3-DOF surrogate of the Gazebo vessel for controller studies.

The surrogate integrates the same planar rigid-body model the Gazebo
Hydrodynamics plugin implements (mass, linear + quadratic drag relative to
the water, no added mass), driven by the same sea state (current with
turbulence, gusty wind as a force/moment, wave orbital velocity).  The
control stack above the plant is the *production* code: the controller
library, prioritised saturation, thrust allocation and actuator model.

Purpose: run hundreds of seeded trials in minutes to tune controllers and
get confidence intervals, then confirm the headline results in Gazebo
(``tools/run_experiments.py``).  It is a surrogate, not a replacement: the
plant constants are nominal and the perturbation arguments exist to test how
results change when the model is wrong.
"""

from dataclasses import dataclass, field
import math
import random

from asv_station_keeping.actuation import (
    actuator_step, prioritised_wrench, wrench_limits)
from asv_station_keeping.controllers import (
    FORCE_PER_UNIT, MOMENT_PER_UNIT, VehicleSpec, make_controller,
    wrap_angle)
from asv_station_keeping.twist_to_thruster import (
    THRUSTER_AXES, THRUSTER_POSITIONS, allocate, allocate_optimal)

AIR_DENSITY = 1.225
CONTROL_DT = 0.05
SUBSTEPS = 5


@dataclass(frozen=True)
class SeaState:
    """Environmental disturbance level (same fields as the Gazebo sea)."""

    wind_speed: float = 3.0
    wind_direction_deg: float = 135.0
    wind_gust_std: float = 0.30
    wind_direction_wander_deg: float = 12.0
    current_x: float = 0.10
    current_y: float = 0.0
    current_turbulence_std: float = 0.02
    wave_height: float = 0.10
    wave_period: float = 2.5
    wave_direction_deg: float = 135.0


SEAS = {
    'calm': SeaState(0.0, 135.0, 0.0, 0.0, 0.10, 0.0, 0.0, 0.0),
    'moderate': SeaState(),
    'strong': SeaState(4.5, 135.0, 0.30, 12.0, 0.15, 0.05, 0.03, 0.15),
    'limit': SeaState(5.5, 135.0, 0.40, 12.0, 0.15, 0.05, 0.03, 0.15),
}


@dataclass
class PlantTruth:
    """True plant constants; scale them to model error."""

    mass: tuple = (41.84, 41.84)
    inertia_z: float = 2.3
    linear_drag: tuple = (0.60, 0.80, 0.50)
    quadratic_drag: tuple = (2.0, 3.0, 5.0)
    windage_area: float = 0.15
    wind_cd: float = 1.0
    wind_cp: tuple = (0.08, 0.02)

    def perturbed(self, mass=1.0, drag=1.0, inertia=1.0):
        """Return a copy with scaled mass, drag and yaw inertia."""
        return PlantTruth(
            mass=tuple(m * mass for m in self.mass),
            inertia_z=self.inertia_z * inertia,
            linear_drag=tuple(b * drag for b in self.linear_drag),
            quadratic_drag=tuple(c * drag for c in self.quadratic_drag),
            windage_area=self.windage_area, wind_cd=self.wind_cd,
            wind_cp=self.wind_cp)


@dataclass
class RunConfig:
    """Everything that defines one trial except the random seed."""

    controller: str = 'pid'
    sea: SeaState = field(default_factory=SeaState)
    duration: float = 120.0
    settle: float = 20.0
    use_dob: bool = False
    allocation: str = 'lstsq'           # 'lstsq' or 'optimal'
    events: tuple = ()
    plant: PlantTruth = field(default_factory=PlantTruth)
    model_error: dict = field(default_factory=dict)
    controller_kwargs: dict = field(default_factory=dict)
    max_thrust: float = 5.5
    yaw_reserve: float = 2.2
    slew_rate: float = 1.5
    pos_noise: float = 0.02
    yaw_noise: float = 0.01
    vel_noise: float = 0.015
    velocity_filter_tau: float = 0.15
    hold: bool = True


class DisturbanceObserver:
    """Lumped-force observer identical to the one in the ROS node."""

    def __init__(self, mass=45.0, tau=6.0):
        self.mass = mass
        self.gain = 1.0 / max(tau, 0.05)
        self.z = None
        self.estimate = (0.0, 0.0)

    def update(self, yaw, u, v, fx_body, fy_body, dt):
        """Advance the observer; return the world-frame force estimate."""
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        vx = cos_yaw * u - sin_yaw * v
        vy = sin_yaw * u + cos_yaw * v
        fx = cos_yaw * fx_body - sin_yaw * fy_body
        fy = sin_yaw * fx_body + cos_yaw * fy_body
        if self.z is None:
            self.z = [-self.gain * self.mass * vx,
                      -self.gain * self.mass * vy]
        drag_x = 0.7 * vx + 2.5 * abs(vx) * vx
        drag_y = 0.7 * vy + 2.5 * abs(vy) * vy
        est_x = self.z[0] + self.gain * self.mass * vx
        est_y = self.z[1] + self.gain * self.mass * vy
        self.z[0] += dt * self.gain * (drag_x - fx - est_x)
        self.z[1] += dt * self.gain * (drag_y - fy - est_y)
        self.estimate = (est_x, est_y)
        return self.estimate


class Ou:
    """Ornstein-Uhlenbeck process with stationary std ``sigma``."""

    def __init__(self, rng, sigma, tau):
        self.rng, self.sigma, self.tau = rng, sigma, max(tau, 1e-3)
        self.x = rng.gauss(0.0, sigma) if sigma > 0.0 else 0.0

    def step(self, dt):
        if self.sigma <= 0.0:
            return 0.0
        self.x += (-self.x / self.tau) * dt + self.sigma * math.sqrt(
            2.0 * dt / self.tau) * self.rng.gauss(0.0, 1.0)
        return self.x


def simulate(config, seed=1, record=False):
    """Run one trial and return ``(metrics, trace_or_None)``."""
    rng = random.Random(seed)
    sea, truth = config.sea, config.plant
    if config.model_error:
        truth = truth.perturbed(**config.model_error)
    nominal = VehicleSpec()
    controller = make_controller(
        config.controller, nominal, **config.controller_kwargs)
    observer = DisturbanceObserver(mass=45.0)

    # Gazebo scales the gust std by the configured mean wind speed.
    wind_gust = Ou(rng, sea.wind_gust_std * sea.wind_speed, 6.0)
    wind_dir = Ou(rng, sea.wind_direction_wander_deg, 6.0)
    turb_x = Ou(rng, sea.current_turbulence_std, 20.0)
    turb_y = Ou(rng, sea.current_turbulence_std, 20.0)
    pos_noise = [Ou(rng, config.pos_noise, 2.0) for _ in range(2)]
    wave_phase = rng.uniform(0.0, 2.0 * math.pi)

    state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # x y psi u v r
    target = [0.0, 0.0, 0.0]
    target_goal = [0.0, 0.0, 0.0]
    health = [1.0] * 4
    gains = [1.0] * 4
    wind_speed = sea.wind_speed
    gps_ok = True
    gps_bias = [0.0, 0.0]
    gps_drift = [0.0, 0.0]
    gps_jump = [0.0, 0.0]
    actuator_state = (0.0, 0.0, 0.0, 0.0)
    output = (0.0, 0.0, 0.0, 0.0)
    delivered = (0.0, 0.0, 0.0, 0.0)
    filtered = None
    saturated = False
    events = sorted(config.events, key=lambda e: e[0])
    event_index = 0
    last_event_time = None

    steps = int(config.duration / CONTROL_DT)
    times, err_pos, err_yaw, power, sat_flags = [], [], [], [], []
    trace = {k: [] for k in ('t', 'x', 'y', 'yaw', 'err', 'power')} \
        if record else None
    energy = 0.0
    dt = CONTROL_DT
    sub_dt = dt / SUBSTEPS

    for step in range(steps):
        t = step * dt
        while event_index < len(events) and events[event_index][0] <= t:
            _t, kind, value = events[event_index]
            last_event_time = _t
            if kind == 'thruster_gain':
                gains[int(value[0])] = float(value[1])
            elif kind == 'allocator_health':
                health[int(value[0])] = float(value[1])
            elif kind == 'wind_speed':
                wind_speed = float(value)
            elif kind == 'gps':
                gps_ok = bool(value)
                if gps_ok:
                    gps_drift = [0.0, 0.0]
                else:
                    gps_drift = [rng.gauss(0.0, 0.02), rng.gauss(0.0, 0.02)]
            elif kind == 'gps_jump':
                gps_jump = [float(value[0]), float(value[1])]
            elif kind == 'target':
                target_goal = [float(v) for v in value]
            event_index += 1

        # ---- environment (world frame)
        gust = wind_gust.step(dt)
        speed = max(0.0, wind_speed + gust)
        direction = math.radians(sea.wind_direction_deg + wind_dir.step(dt))
        wind_force = 0.5 * AIR_DENSITY * truth.wind_cd * truth.windage_area \
            * speed * speed
        fw = (wind_force * math.cos(direction),
              wind_force * math.sin(direction))
        omega = 2.0 * math.pi / max(sea.wave_period, 0.1)
        orbital = math.pi * sea.wave_height / max(sea.wave_period, 0.1) \
            * math.cos(omega * t + wave_phase)
        wave_dir = math.radians(sea.wave_direction_deg)
        water = (sea.current_x + turb_x.step(dt) + orbital * math.cos(wave_dir),
                 sea.current_y + turb_y.step(dt) + orbital * math.sin(wave_dir))

        # ---- sensing: GPS/IMU estimator surrogate
        for axis in range(2):
            gps_bias[axis] += gps_drift[axis] * dt if not gps_ok else 0.0
            if gps_ok:
                gps_bias[axis] *= max(0.0, 1.0 - dt / 1.0)
        meas_x = state[0] + pos_noise[0].step(dt) + gps_bias[0] + gps_jump[0]
        meas_y = state[1] + pos_noise[1].step(dt) + gps_bias[1] + gps_jump[1]
        meas_yaw = state[2] + rng.gauss(0.0, config.yaw_noise)
        u_m = state[3] + rng.gauss(0.0, config.vel_noise)
        v_m = state[4] + rng.gauss(0.0, config.vel_noise)
        r_m = state[5] + rng.gauss(0.0, config.vel_noise)
        if filtered is None:
            filtered = [u_m, v_m, r_m]
        else:
            alpha = dt / (config.velocity_filter_tau + dt)
            filtered[0] += alpha * (u_m - filtered[0])
            filtered[1] += alpha * (v_m - filtered[1])
            filtered[2] += alpha * (r_m - filtered[2])

        # ---- reference slewing (matches the node)
        for i, rate in enumerate((0.25, 0.25, 0.35)):
            delta = target_goal[i] - target[i]
            if i == 2:
                delta = wrap_angle(delta)
            step_size = rate * dt
            target[i] += max(-step_size, min(step_size, delta))

        # ---- control
        dx, dy = target[0] - meas_x, target[1] - meas_y
        cos_y, sin_y = math.cos(meas_yaw), math.sin(meas_yaw)
        errors = (cos_y * dx + sin_y * dy, -sin_y * dx + cos_y * dy,
                  wrap_angle(target[2] - meas_yaw))
        translation_limit, yaw_limit = wrench_limits(
            config.max_thrust, config.yaw_reserve, config.allocation)
        limits = (translation_limit * FORCE_PER_UNIT,
                  translation_limit * FORCE_PER_UNIT,
                  yaw_limit * MOMENT_PER_UNIT)
        wrench = controller.compute(
            errors, tuple(filtered), dt, saturated, limits)
        ux = wrench[0] / FORCE_PER_UNIT
        uy = wrench[1] / FORCE_PER_UNIT
        uz = wrench[2] / MOMENT_PER_UNIT

        fx_b = sum(c * a[0] for c, a in zip(delivered, THRUSTER_AXES))
        fy_b = sum(c * a[1] for c, a in zip(delivered, THRUSTER_AXES))
        dob = observer.update(meas_yaw, filtered[0], filtered[1],
                              fx_b, fy_b, dt)
        if config.use_dob:
            ux -= (cos_y * dob[0] + sin_y * dob[1]) / FORCE_PER_UNIT
            uy -= (-sin_y * dob[0] + cos_y * dob[1]) / FORCE_PER_UNIT

        ux, uy, uz, clipped = prioritised_wrench(
            ux, uy, uz, config.max_thrust, config.yaw_reserve,
            config.allocation)
        if config.allocation == 'optimal':
            commands, scale_factor = allocate_optimal(
                ux, uy, uz, health, limit=config.max_thrust)
            saturated = clipped or scale_factor < 0.999999
        else:
            commands = allocate(ux, uy, uz, health)
            scale = max(1.0, max(abs(c) for c in commands))
            saturated = clipped or scale > config.max_thrust
            if scale > config.max_thrust:
                commands = tuple(
                    c * config.max_thrust / scale for c in commands)
        max_delta = config.slew_rate * dt
        output = tuple(
            p + max(-max_delta, min(max_delta, c - p))
            for p, c in zip(output, commands))
        if not config.hold:
            output = (0.0, 0.0, 0.0, 0.0)
        actuator_state, delivered = actuator_step(
            actuator_state, output, dt, 0.05, 0.15, 0.80, gains)

        # ---- plant
        fx_thr = sum(c * a[0] for c, a in zip(delivered, THRUSTER_AXES))
        fy_thr = sum(c * a[1] for c, a in zip(delivered, THRUSTER_AXES))
        mz_thr = sum(
            c * (p[0] * a[1] - p[1] * a[0])
            for c, p, a in zip(delivered, THRUSTER_POSITIONS, THRUSTER_AXES))
        for _ in range(SUBSTEPS):
            x, y, psi, u, v, r = state
            cp, sp = math.cos(psi), math.sin(psi)
            wu = cp * water[0] + sp * water[1]
            wv = -sp * water[0] + cp * water[1]
            ur, vr = u - wu, v - wv
            fwx = cp * fw[0] + sp * fw[1]
            fwy = -sp * fw[0] + cp * fw[1]
            mw = truth.wind_cp[0] * fwy - truth.wind_cp[1] * fwx
            fx = fx_thr - truth.linear_drag[0] * ur \
                - truth.quadratic_drag[0] * abs(ur) * ur + fwx
            fy = fy_thr - truth.linear_drag[1] * vr \
                - truth.quadratic_drag[1] * abs(vr) * vr + fwy
            mz = mz_thr - truth.linear_drag[2] * r \
                - truth.quadratic_drag[2] * abs(r) * r + mw
            u += sub_dt * fx / truth.mass[0]
            v += sub_dt * fy / truth.mass[1]
            r += sub_dt * mz / truth.inertia_z
            x += sub_dt * (u * cp - v * sp)
            y += sub_dt * (u * sp + v * cp)
            psi += sub_dt * r
            state = [x, y, psi, u, v, r]

        # ---- scoring on ground truth
        pos_err = math.hypot(target[0] - state[0], target[1] - state[1])
        yaw_err = abs(wrap_angle(target[2] - state[2]))
        pwr = sum(abs(c) ** 1.5 for c in delivered)
        energy += pwr * dt
        times.append(t)
        err_pos.append(pos_err)
        err_yaw.append(yaw_err)
        power.append(pwr)
        sat_flags.append(1.0 if saturated else 0.0)
        if record:
            trace['t'].append(t)
            trace['x'].append(state[0])
            trace['y'].append(state[1])
            trace['yaw'].append(state[2])
            trace['err'].append(pos_err)
            trace['power'].append(pwr)
        if not (abs(state[0]) < 50.0 and abs(state[1]) < 50.0):
            break  # diverged

    return summarise(times, err_pos, err_yaw, power, sat_flags, energy,
                     config, last_event_time), trace


def summarise(times, err_pos, err_yaw, power, sat_flags, energy, config,
              last_event_time):
    """Reduce a trial to the scalar metrics used in the study."""
    keep = [i for i, t in enumerate(times) if t >= config.settle]
    if len(keep) < 20:
        return {'valid': False}
    pe = [err_pos[i] for i in keep]
    ye = [err_yaw[i] for i in keep]
    ordered = sorted(pe)
    metrics = {
        'valid': True,
        'diverged': len(times) < int(config.duration / CONTROL_DT),
        'rms': math.sqrt(sum(e * e for e in pe) / len(pe)),
        'p95': ordered[int(0.95 * (len(ordered) - 1))],
        'max': ordered[-1],
        'yaw_rms_deg': math.degrees(
            math.sqrt(sum(e * e for e in ye) / len(ye))),
        'power': sum(power[i] for i in keep) / len(keep),
        'sat': sum(sat_flags[i] for i in keep) / len(keep),
        'within_5cm': sum(1 for e in pe if e < 0.05) / len(pe),
    }
    if last_event_time is not None:
        after = [i for i, t in enumerate(times) if t >= last_event_time]
        if after:
            peak = max(err_pos[i] for i in after)
            recovery = float('nan')
            for i in after:
                if all(err_pos[j] < 0.10 for j in range(i, len(times))):
                    recovery = times[i] - last_event_time
                    break
            metrics['event_peak'] = peak
            metrics['recovery_s'] = recovery
    return metrics
