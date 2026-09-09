"""Pure actuator math shared by ROS nodes and tests."""

import math

import numpy as np


def validate_named_values(names, values, expected_names, low=None, high=None):
    """Validate and reorder a named numeric vector into canonical order."""
    if len(names) != len(values):
        raise ValueError('names and values must have equal lengths')
    if len(names) != len(set(names)):
        raise ValueError('duplicate thruster name')
    if set(names) != set(expected_names):
        unknown = sorted(set(names) - set(expected_names))
        missing = sorted(set(expected_names) - set(names))
        raise ValueError(f'unknown={unknown}; missing={missing}')
    by_name = dict(zip(names, values))
    ordered = [float(by_name[name]) for name in expected_names]
    if not all(math.isfinite(value) for value in ordered):
        raise ValueError('non-finite actuator value')
    if low is not None and any(value < low for value in ordered):
        raise ValueError(f'actuator value below {low}')
    if high is not None and any(value > high for value in ordered):
        raise ValueError(f'actuator value above {high}')
    return ordered


def command_to_force(command, thruster, voltage_scale=1.0):
    """Quadratic deadband/asymmetric static thrust model."""
    command = max(-1.0, min(1.0, float(command)))
    magnitude = abs(command)
    deadband = float(thruster['deadband'])
    if magnitude <= deadband:
        return 0.0
    effective = (magnitude - deadband) / (1.0 - deadband)
    limit = float(thruster[
        'max_forward_n' if command >= 0.0 else 'max_reverse_n'])
    return math.copysign(limit * effective * effective * voltage_scale,
                         command)


def force_to_command(force, thruster, voltage_scale=1.0):
    """Inverse of command_to_force with saturation."""
    force = float(force)
    if not math.isfinite(force):
        raise ValueError('force must be finite')
    if abs(force) < 1e-9:
        return 0.0
    voltage_scale = max(float(voltage_scale), 1e-6)
    limit = float(thruster[
        'max_forward_n' if force >= 0.0 else 'max_reverse_n'])
    normalized = math.sqrt(min(abs(force) / (limit * voltage_scale), 1.0))
    deadband = float(thruster['deadband'])
    return math.copysign(deadband + (1.0 - deadband) * normalized, force)


def allocate_wrench(matrix, wrench, lower_limits, upper_limits):
    """Bounded least-squares allocation using deterministic active sets."""
    allocation = np.asarray(matrix, dtype=float)
    requested = np.asarray(wrench, dtype=float)
    lower = np.asarray(lower_limits, dtype=float)
    upper = np.asarray(upper_limits, dtype=float)
    count = allocation.shape[1]
    if allocation.shape[0] != 6 or requested.shape != (6,):
        raise ValueError('allocation matrix and wrench must be 6-DOF')
    if lower.shape != (count,) or upper.shape != (count,):
        raise ValueError('limit vector length does not match thruster count')
    if (not np.all(np.isfinite(allocation)) or
            not np.all(np.isfinite(requested))):
        raise ValueError('allocation inputs must be finite')

    forces = np.zeros(count, dtype=float)
    free = list(range(count))
    fixed = []
    for _ in range(count + 1):
        residual = requested.copy()
        if fixed:
            residual -= allocation[:, fixed] @ forces[fixed]
        if free:
            solution, _, _, _ = np.linalg.lstsq(
                allocation[:, free], residual, rcond=None)
            forces[free] = solution
        violations = [index for index in free
                      if forces[index] < lower[index] or
                      forces[index] > upper[index]]
        if not violations:
            break
        # Lock the worst normalized violation first. This produces stable,
        # predictable degradation when the requested wrench is infeasible.
        index = max(
            violations,
            key=lambda item: max(
                lower[item] - forces[item], forces[item] - upper[item]),
        )
        forces[index] = np.clip(forces[index], lower[index], upper[index])
        free.remove(index)
        fixed.append(index)
    return forces
