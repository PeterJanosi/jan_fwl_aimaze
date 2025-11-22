"""ROS2-backed RL environment wrapper.

This class provides a thin environment wrapper that integrates with ROS 2 topics:
- subscribes to `/scan` (sensor_msgs/LaserScan) and optional `/odom` (nav_msgs/Odometry)
- publishes to `cmd_vel` (geometry_msgs/Twist)

Notes:
- rclpy is imported lazily; if ROS 2 is not installed importing this module will fail only when
  attempting to create a ROS-backed env. The dry-run `rl.env.RLEnv` remains available without ROS.
- This skeleton intentionally avoids spinning in a background thread. `step()` uses
  `rclpy.spin_once` with short timeouts to process incoming messages synchronously.
"""
from typing import Optional, Tuple, Dict
import math
import threading
import time


class RLEnvROS:
    """ROS 2 environment wrapper. Requires a working ROS 2 Python environment.

    Usage (example):

    env = RLEnvROS()
    env.reset()
    s, r, done, info = env.step(0)
    env.close()
    """

    ACTIONS = [(0.15, 0.0), (0.0, 0.6), (0.0, -0.6), (-0.05, 0.0)]

    def __init__(
        self,
        node_name: str = "rl_env_node",
        scan_topic: str = "/scan",
        odom_topic: Optional[str] = "/odom",
        cmd_vel_topic: str = "cmd_vel",
        num_beams: int = 24,
        max_range: float = 3.5,
        spin_timeout: float = 0.1,
    ):
        # lazy import of rclpy and message types
        try:
            import rclpy  # type: ignore
            from rclpy.node import Node  # type: ignore
            from sensor_msgs.msg import LaserScan  # type: ignore
            from nav_msgs.msg import Odometry  # type: ignore
            from geometry_msgs.msg import Twist  # type: ignore
        except Exception as e:
            raise RuntimeError(
                "ROS 2 Python packages not available. Install ROS 2 or use the dry-run env." + str(e)
            )

        self.rclpy = rclpy
        self.Node = Node
        self.LaserScan = LaserScan
        self.Odometry = Odometry
        self.Twist = Twist

        # initialize rclpy if necessary
        if not rclpy.ok():
            try:
                rclpy.init()
            except Exception:
                # if already initialized in the current process this may fail harmlessly
                pass

        self.node = rclpy.create_node(node_name)
        self.scan_topic = scan_topic
        self.odom_topic = odom_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.num_beams = num_beams
        self.max_range = max_range
        self.spin_timeout = spin_timeout

        # internal state
        self._last_scan = None
        self._last_scan_lock = threading.Lock()
        self._last_odom = None
        self._scan_event = threading.Event()

        # subscriptions and publisher
        self._scan_sub = self.node.create_subscription(
            self.LaserScan, self.scan_topic, self._scan_cb, 10
        )
        if self.odom_topic:
            self._odom_sub = self.node.create_subscription(
                self.Odometry, self.odom_topic, self._odom_cb, 10
            )
        else:
            self._odom_sub = None

        self._cmd_pub = self.node.create_publisher(self.Twist, self.cmd_vel_topic, 10)

        self._step_count = 0
        self._max_steps = 200
        self._distance_to_goal = None

    def _scan_cb(self, msg):
        # store the ranges array (list of floats)
        with self._last_scan_lock:
            self._last_scan = list(msg.ranges)
            self._scan_event.set()

    def _odom_cb(self, msg):
        # store pose information (x,y) from odometry
        try:
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            self._last_odom = (float(px), float(py))
        except Exception:
            self._last_odom = None

    def _aggregate_scan(self, raw_scan):
        import numpy as np

        raw = np.asarray(raw_scan, dtype=float)
        if raw.size == 0:
            return np.ones(self.num_beams) * self.max_range
        bins = np.array_split(raw, self.num_beams)
        agg = np.array([np.nanmean(b) if b.size > 0 else self.max_range for b in bins])
        agg = np.where(np.isfinite(agg), agg, self.max_range)
        agg = np.clip(agg, 0.0, self.max_range)
        return agg

    def reset(self, goal_pose: Optional[Tuple[float, float]] = None):
        """Reset environment. Optionally pass a goal_pose (x,y) in world frame.

        If goal_pose is None and odometry is available, distance is initialized from current odom.
        """
        # clear state and wait for initial scan
        self._step_count = 0
        self._distance_to_goal = None
        self._scan_event.clear()

        # wait for a scan to arrive (spin_once until event or timeout)
        start = time.time()
        while not self._scan_event.is_set() and (time.time() - start) < 5.0:
            self.rclpy.spin_once(self.node, timeout_sec=self.spin_timeout)

        with self._last_scan_lock:
            scan = self._last_scan if self._last_scan is not None else [self.max_range] * self.num_beams

        # initialize distance to goal
        if goal_pose and self._last_odom:
            ox, oy = self._last_odom
            gx, gy = goal_pose
            self._distance_to_goal = math.hypot(gx - ox, gy - oy)
        elif goal_pose:
            # no odom: set distance as hypot of goal
            self._distance_to_goal = math.hypot(goal_pose[0], goal_pose[1])
        elif self._last_odom and self._last_odom is not None:
            # no goal provided: set a default goal offset (forward 3m)
            ox, oy = self._last_odom
            self._distance_to_goal = 3.0
        else:
            self._distance_to_goal = 3.0

        return self._aggregate_scan(scan)

    def step(self, action: int, wait_for_scan: bool = True) -> Tuple:
        assert 0 <= action < len(self.ACTIONS)
        self._step_count += 1

        # publish cmd_vel
        v, omega = self.ACTIONS[action]
        twist = self.Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(omega)
        self._cmd_pub.publish(twist)

        # after publishing, spin until next scan message (or timeout)
        self._scan_event.clear()
        start = time.time()
        while wait_for_scan and (time.time() - start) < 1.0:
            self.rclpy.spin_once(self.node, timeout_sec=self.spin_timeout)
            if self._scan_event.is_set():
                break

        with self._last_scan_lock:
            scan = self._last_scan if self._last_scan is not None else [self.max_range] * self.num_beams

        # compute distance change using odom if available
        prev = float(self._distance_to_goal) if self._distance_to_goal is not None else 3.0
        if self._last_odom is not None:
            # naive update: if v>0 reduce distance by v*dt (approx spin_timeout)
            dt = max(self.spin_timeout, 0.05)
            approx_delta = v * dt
            cur = max(0.0, prev - approx_delta)
        else:
            # fallback: small random-ish change
            cur = max(0.0, prev - (v * 0.5 if v > 0 else v * 0.2))

        self._distance_to_goal = cur

        # simulate collision if any scan reading very small
        collision = any((r is not None and r < 0.08) for r in scan)

        reward = self._compute_reward(prev, cur, collision)

        done = False
        if collision or cur <= 0.15 or self._step_count >= self._max_steps:
            done = True

        info: Dict = {"collision": collision, "distance": float(cur)}
        state = self._aggregate_scan(scan)
        return state, float(reward), done, info

    def _compute_reward(self, prev_dist: float, cur_dist: float, collision: bool) -> float:
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

    def close(self):
        try:
            # stop the robot
            twist = self.Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            self.rclpy.shutdown()
        except Exception:
            pass
