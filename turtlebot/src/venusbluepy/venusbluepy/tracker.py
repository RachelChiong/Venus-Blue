import numpy as np
import time

from dataclasses import dataclass

@dataclass
class Beacon:
    name: str
    address: str
    major: int
    minor: int
    x: float
    y: float
    timestamp: int = None
    range: float = None
    error: float = None

class Tracker:

    def __init__(
            self,
            beacons: list[Beacon],
            process_error: float = 0.1,
            measurement_error: float = 0.2
        ) -> None:
        """Create a position tracker of beacon range estimates.

        Arguments:
            beacons: The list of beacons to track.
            process_error: The error of model prediction.
            measurement_error: The error of range measurements.
        """

        self._kalman_observation_matrix = np.array([
            (1, 0, 0, 0),
            (0, 1, 0, 0)
        ])

        self._kalman_covariance = np.eye(4)
        self._kalman_process_error = np.eye(4) * process_error
        self._kalman_observation_error = np.eye(2) * measurement_error

        self.update_beacons(beacons)

        # Set the initial state to the current position with zero velocity.
        self._last_time = time.time_ns()
        self._last_state = np.vstack((
            self._observe_position(beacons),
            np.zeros((2, 1))
        ))

    def update_beacons(self, beacons: list[Beacon]):
        """Update the list of beacons used for tracking.

        Arguments:
            beacons: The list of beacons being tracked for position.
        """
        self._kth_beacon = beacons[-1]

        A = 2 * np.vstack([
            (self._kth_beacon.x - beacon.x, self._kth_beacon.y - beacon.y)
            for beacon in beacons
        ])

        V_inv = np.linalg.inv(np.diag([b.error for b in beacons]))

        self._multilaterate_coef = (
            np.linalg.inv(A.T @ V_inv @ A) @ A.T @ V_inv
        )

    def _observe_position(self, beacons: list[Beacon]) -> np.array:
        """Observe a multilaterated

        Arguments:
            beacons: The beacons to observe a position with.

        Returns:
            The least squares estimate of the position.
        """
        return self._multilaterate_coef @ np.vstack([
            b.range ** 2 - self._kth_beacon.range ** 2
            - b.x ** 2 - b.y ** 2
            + self._kth_beacon.x ** 2 + self._kth_beacon.y ** 2
            for b in beacons
        ])

    def _observe_state(self, beacons, dt: float) -> np.array:
        """Observe a state from the beacons.

        Arguments:
            beacons: The beacons to observe position from.
            dt: The change in time from the last observation.
        """
        position = self._observe_position(beacons)
        velocity = (position - self._last_state[0:2, 0, np.newaxis]) / dt

        return np.vstack((position, velocity))

    def _filter_state(self, state, dt: float) -> np.array:
        """Use the kalman filter to estimate the state based on previous
        measurements and predicted state.

        Arguments:
            state: An observed state.

        Returns:
            The filtered state.
        """

        transition_matrix = np.array([
            (1, 0, dt, 0),
            (0, 1, 0, dt),
            (0, 0, 1, 0),
            (0, 0, 0, 1)
        ])

        # Predict how the estimated position will transition.
        predicted_state = transition_matrix @ self._last_state

        # Covariance of prediction.
        self.predicted_covariance = (
            transition_matrix @
            self._kalman_covariance @
            transition_matrix.T
        ) + self._kalman_process_error

        # The error between the observed position and the predicted position.
        position_error = state[0:2, 0, np.newaxis] - (
            self._kalman_observation_matrix @ predicted_state
        )

        error_covariance = (
            self._kalman_observation_matrix @
            self.predicted_covariance @ 
            self._kalman_observation_matrix.T
        ) + self._kalman_observation_error

        K = (
            self.predicted_covariance @
            self._kalman_observation_matrix.T @
            np.linalg.inv(error_covariance)
        )

        self._last_state = predicted_state + K @ position_error

        self._kalman_covariance = (
            (np.eye(4) - K @self._kalman_observation_matrix) @
            self.predicted_covariance
        )

    def update(self, beacons: list[Beacon]) -> np.array:
        """Update the estimated position from the beacons.

        Arguments:
            beacons: The list of beacons to update position with.
            dt: The change in time since the last update.
        """
        # Get the change in time since the last update.
        now = time.time_ns()
        dt = (now - self._last_time) * 10 ** -9
        if dt < 0.1:
            return self._last_state[0:2, 0, np.newaxis]

        # Get the observed state and filter it.
        self._filter_state(self._observe_state(beacons, dt), dt)

        # Set the last time updated to now.
        self._last_time = now

    def get_position(self):
        """Get the most recent tracked position."""
        return self._last_state[0:2, 0, np.newaxis]
