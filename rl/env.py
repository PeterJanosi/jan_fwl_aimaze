"""Minimal RL environment wrapper for jan_fwl_aimaze.

This file implements a lightweight environment class with two modes:
- dry-run (use_ros=False): simulates sensor data and robot motion for testing.
- ROS mode (use_ros=True): placeholder for future ROS2 integration (not initialized
  automatically in the skeleton to keep imports light-weight).

The goal here is a clear contract for reset() and step(action) so an agent can
be attached later.
"""
from typing import Dict, Tuple, List
import random
import numpy as np


class RLEnv:
    """Simple RL environment wrapper.

    Methods:
    - reset() -> state
    - step(action) -> (state, reward, done, info)

    The environment supports a dry-run simulation when `use_ros=False`.
    """

    # Discrete action mapping to (v, omega)
    ACTIONS: List[Tuple[float, float]] = [
        (0.15, 0.0),    # 0: forward
        (0.0, 0.6),     # 1: turn left
        (0.0, -0.6),    # 2: turn right
        (-0.05, 0.0),   # 3: backward
    ]

    def __init__(self, use_ros: bool = False, num_beams: int = 24, max_range: float = 3.5):
        self.use_ros = use_ros
        self.num_beams = num_beams
        self.max_range = max_range

        # dry-run state variables
        self._scan = np.ones(self.num_beams) * self.max_range
        self._distance_to_goal = None
        self._step_count = 0
        self._max_steps = 200

        if self.use_ros:
            # Placeholder: in a full implementation we'd initialize rclpy,
            # a node, subscriptions and publishers here.
            try:
                import rclpy  # type: ignore

                # In this skeleton we don't spin ROS automatically.
                # Users should integrate this class into their ROS node.
                self._ros_available = True
            except Exception:
                self._ros_available = False
        else:
            self._ros_available = False

    def _aggregate_scan(self, raw_scan: np.ndarray) -> np.ndarray:
        """Aggregate raw scan to `self.num_beams` values.

        raw_scan is expected to be a 1D array-like; if it's shorter/longer we
        resample/aggregate uniformly.
        """
        raw = np.asarray(raw_scan, dtype=float)
        if raw.size == 0:
            return np.ones(self.num_beams) * self.max_range

        # simple re-binning by averaging
        bins = np.array_split(raw, self.num_beams)
        agg = np.array([np.nanmean(b) if b.size > 0 else self.max_range for b in bins])
        # replace nan with max_range
        agg = np.where(np.isfinite(agg), agg, self.max_range)
        # clip
        agg = np.clip(agg, 0.0, self.max_range)
        return agg

    def reset(self) -> np.ndarray:
        """Reset the environment and return the initial state vector.

        For dry-run mode we randomize an initial distance to goal and return the
        aggregated scan (currently full-range readings).
        """
        self._step_count = 0
        # initial simulated distance to goal (meters)
        self._distance_to_goal = random.uniform(1.0, 5.0)
        # simulated scan: all clear initially
        self._scan = np.ones(self.num_beams) * self.max_range
        return self._get_state()

    def _get_state(self) -> np.ndarray:
        """Return current observation/state (aggregated scan + optional extras).

        For the skeleton we return only the aggregated LIDAR vector. Later we can
        append odometry or relative goal information.
        """
        return self._aggregate_scan(self._scan)

    def step(self, action: int) -> Tuple[np.ndarray, float, bool, Dict]:
        """Apply the discrete action and return (state, reward, done, info).

        In dry-run mode the environment simulates a small change in distance to
        the goal depending on the action. Collisions are simulated with a low
        probability.
        """
        assert 0 <= action < len(self.ACTIONS), f"Invalid action: {action}"
        self._step_count += 1

        # simulate effect on distance to goal
        v, omega = self.ACTIONS[action]
        # forward reduces distance, turning has small effect
        if v > 0:
            delta = v * random.uniform(0.8, 1.2)
        elif v < 0:
            delta = v * random.uniform(0.5, 1.0)
        else:
            # turning in place: small random change
            delta = -0.01 * random.random()

        prev_dist = float(self._distance_to_goal)
        self._distance_to_goal = max(0.0, prev_dist - delta)

        # simulate a random collision event with low probability
        collision = False
        if random.random() < 0.01:
            collision = True

        # compute reward
        reward = self._compute_reward(prev_dist, self._distance_to_goal, collision)

        done = False
        if collision:
            done = True
        elif self._distance_to_goal <= 0.15:
            done = True
        elif self._step_count >= self._max_steps:
            done = True

        info: Dict = {"collision": collision, "distance": float(self._distance_to_goal)}

        state = self._get_state()
        return state, float(reward), done, info

    def _compute_reward(self, prev_dist: float, cur_dist: float, collision: bool) -> float:
        """Compute reward using a simple distance-based formula.

        r = k*(prev - cur) + R_goal*I_goal - R_collision*I_collision - eps
        """
        k = 1.0
        eps = 0.01
        R_goal = 10.0
        R_collision = 20.0
        r = k * (prev_dist - cur_dist) - eps
        if collision:
            r -= R_collision
        if cur_dist <= 0.15:
            r += R_goal
        return r


if __name__ == "__main__":
    # quick manual test in dry-run mode
    env = RLEnv(use_ros=False)
    s = env.reset()
    print("Initial state shape:", s.shape)
    for i in range(5):
        a = random.randrange(len(env.ACTIONS))
        s, r, done, info = env.step(a)
        print(f"step={i} action={a} reward={r:.3f} done={done} info={info}")
        if done:
            break
