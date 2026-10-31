"""Actuation chain shared by the ROS node and the offline simulator.

Keeping these as plain functions guarantees that a controller evaluated
offline sees exactly the saturation priority and thruster dynamics it meets
in Gazebo.
"""


def wrench_limits(max_thrust, yaw_reserve, allocation_mode):
    """Return the ``(translation, yaw)`` demand caps in mixer units.

    With plain least-squares allocation every thruster carries
    ``|ux| + |uy| + |uz|`` in the worst case, so translation is capped at
    ``max_thrust - yaw_reserve`` to leave room for yaw.  The saturation-aware
    allocator checks exact feasibility itself (shrinking translation before
    yaw), so the conservative split is relaxed: translation may use the
    thrust that yaw does not need.  The controllers are told a bound of
    ``max_thrust - yaw_reserve / 2``; the allocator enforces the rest.
    """
    yaw_limit = min(max(yaw_reserve, 0.0), max_thrust)
    if allocation_mode == 'optimal':
        return max(max_thrust - 0.5 * yaw_limit, 1e-6), yaw_limit
    return max(max_thrust - yaw_limit, 1e-6), yaw_limit


def prioritised_wrench(ux, uy, uz, max_thrust, yaw_reserve,
                       allocation_mode='lstsq'):
    """Cap translation and yaw separately so heading keeps authority.

    The caps come from :func:`wrench_limits`.  Returns
    ``(ux, uy, uz, clipped)``.
    """
    translation_limit, yaw_limit = wrench_limits(
        max_thrust, yaw_reserve, allocation_mode)
    clipped = False
    translation = abs(ux) + abs(uy)
    if translation > translation_limit:
        ux *= translation_limit / translation
        uy *= translation_limit / translation
        clipped = True
    if abs(uz) > yaw_limit:
        uz = yaw_limit if uz > 0.0 else -yaw_limit
        clipped = True
    return ux, uy, uz, clipped


def actuator_step(state, commands, dt, deadband, lag_tau, reverse_scale,
                  gains):
    """Advance first-order motor lag, ESC deadband and weaker reverse thrust.

    Returns ``(new_state, delivered)``; ``gains`` are the true per-thruster
    efficiencies (0 = failed), unknown to the allocator.
    """
    tau = max(lag_tau, 0.0)
    alpha = 1.0 if tau <= 1e-9 else dt / (tau + dt)
    lagged = tuple(
        s + alpha * (c - s) for s, c in zip(state, commands))
    delivered = []
    for value in lagged:
        if abs(value) < deadband:
            value = 0.0
        elif value < 0.0:
            value *= reverse_scale
        delivered.append(value)
    return lagged, tuple(c * g for c, g in zip(delivered, gains))
