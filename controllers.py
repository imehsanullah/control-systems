"""Reference trajectory plus three small position controllers."""

from __future__ import annotations

import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.optimize import lsq_linear


MAX_ACCEL = 6.0


def _smoothstep(value: float) -> float:
    return value * value * (3.0 - 2.0 * value)


def _dynamics(dt: float) -> tuple[np.ndarray, np.ndarray]:
    identity = np.eye(3)
    a = np.block([[identity, dt * identity], [np.zeros((3, 3)), identity]])
    b = np.vstack((0.5 * dt**2 * identity, dt * identity))
    return a, b


class FigureEightTrajectory:
    cycle_time = 11.0

    def __init__(self, radius: float, height: float) -> None:
        self.radius = radius
        self.height = height

    def position(self, time_s: float) -> np.ndarray:
        phase = time_s % self.cycle_time
        if phase < 1.5:
            return np.array([0.0, 0.0, self.height * _smoothstep(phase / 1.5)])
        if phase < 8.5:
            angle = 2.0 * np.pi * _smoothstep((phase - 1.5) / 7.0)
            return np.array(
                [
                    self.radius * np.sin(angle),
                    0.5 * self.radius * np.sin(2.0 * angle),
                    self.height * (1.0 + 0.18 * np.sin(2.0 * angle)),
                ]
            )
        if phase < 10.0:
            z = self.height * (1.0 - _smoothstep((phase - 8.5) / 1.5))
            return np.array([0.0, 0.0, z])
        return np.zeros(3)

    def state(self, time_s: float) -> np.ndarray:
        epsilon = 1e-3
        position = self.position(time_s)
        velocity = (
            self.position(time_s + epsilon) - self.position(time_s - epsilon)
        ) / (2.0 * epsilon)
        return np.concatenate((position, velocity))

    def display_points(self, count: int = 180) -> np.ndarray:
        points = np.array(
            [self.position(t) for t in np.linspace(0.0, 10.0, count)]
        )
        keep = np.concatenate(
            ([True], np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-5)
        )
        return points[keep]


class PIDController:
    """Acceleration from proportional, integral, and derivative error."""

    dt = 0.05

    def __init__(self) -> None:
        self.integral = np.zeros(3)

    def control(
        self, state: np.ndarray, time_s: float, trajectory: FigureEightTrajectory
    ) -> np.ndarray:
        reference = trajectory.state(time_s)
        position_error = reference[:3] - state[:3]
        velocity_error = reference[3:] - state[3:]
        self.integral = np.clip(self.integral + position_error * self.dt, -2.0, 2.0)
        acceleration = 7.0 * position_error + 0.15 * self.integral + 4.5 * velocity_error
        return np.clip(acceleration, -MAX_ACCEL, MAX_ACCEL)


class LQRController:
    """Optimal constant state-feedback gain for the linear model."""

    dt = 0.05

    def __init__(self) -> None:
        a, b = _dynamics(self.dt)
        q = np.diag([35.0, 35.0, 45.0, 3.0, 3.0, 4.0])
        r = 0.25 * np.eye(3)
        p = solve_discrete_are(a, b, q, r)
        self.gain = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)

    def control(
        self, state: np.ndarray, time_s: float, trajectory: FigureEightTrajectory
    ) -> np.ndarray:
        error = state - trajectory.state(time_s)
        return np.clip(-self.gain @ error, -MAX_ACCEL, MAX_ACCEL)


class MPCController:
    """Bounded optimization over a one-second prediction horizon."""

    def __init__(self, dt: float = 0.05, horizon: int = 20) -> None:
        self.dt = dt
        self.horizon = horizon
        self.a, self.b = _dynamics(dt)
        self.sx, self.su = self._prediction_matrices()

        weights = np.sqrt(
            np.tile([35.0, 35.0, 45.0, 3.0, 3.0, 4.0], horizon)
        )
        self.weights = weights
        self.objective = np.vstack(
            (weights[:, None] * self.su, np.sqrt(0.25) * np.eye(3 * horizon))
        )

    def _prediction_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        sx = np.zeros((6 * self.horizon, 6))
        su = np.zeros((6 * self.horizon, 3 * self.horizon))
        for step in range(self.horizon):
            sx[6 * step : 6 * (step + 1)] = np.linalg.matrix_power(
                self.a, step + 1
            )
            for control_step in range(step + 1):
                su[
                    6 * step : 6 * (step + 1),
                    3 * control_step : 3 * (control_step + 1),
                ] = np.linalg.matrix_power(self.a, step - control_step) @ self.b
        return sx, su

    def control(
        self, state: np.ndarray, time_s: float, trajectory: FigureEightTrajectory
    ) -> np.ndarray:
        references = np.concatenate(
            [trajectory.state(time_s + (i + 1) * self.dt) for i in range(self.horizon)]
        )
        target = np.concatenate(
            (self.weights * (references - self.sx @ state), np.zeros(3 * self.horizon))
        )
        solution = lsq_linear(
            self.objective,
            target,
            bounds=(-MAX_ACCEL, MAX_ACCEL),
            tol=1e-5,
            max_iter=30,
        )
        return solution.x[:3]


CONTROLLERS = {
    "pid": PIDController,
    "lqr": LQRController,
    "mpc": MPCController,
}
