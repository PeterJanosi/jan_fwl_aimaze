"""Run a ROS-based demo that connects to a running simulator and takes random actions.

Usage:
  python3 -m rl.run_ros_demo --episodes 5 --max-steps 200

This script requires a running ROS 2 environment and a publisher on `/scan` (e.g. a
Gazebo or Ignition simulation). It uses `RLEnvROS` to interact with topics.
"""
import argparse
import logging
import random
import time
from typing import Optional

try:
    from rl.ros_env import RLEnvROS
except Exception as e:  # pragma: no cover - runtime environment dependent
    RLEnvROS = None


def run_demo(episodes: int = 5, max_steps: int = 200, goal: Optional[tuple] = None, seed: Optional[int] = None):
    if seed is not None:
        random.seed(seed)

    if RLEnvROS is None:
        raise RuntimeError("RLEnvROS is not available. Ensure ROS2 and package dependencies are installed and sourced.")

    env = RLEnvROS()
    try:
        for ep in range(episodes):
            logging.info("Starting episode %d/%d", ep + 1, episodes)
            state = env.reset(goal_pose=goal) if goal is not None else env.reset()
            total_reward = 0.0
            start_time = time.time()
            for step in range(max_steps):
                action = random.randrange(len(env.ACTIONS))
                state, reward, done, info = env.step(action)
                total_reward += reward
                logging.info("ep=%d step=%d action=%d reward=%.3f done=%s info=%s", ep + 1, step + 1, action, reward, done, info)
                if done:
                    break
            duration = time.time() - start_time
            logging.info("Episode %d finished: total_reward=%.3f steps=%d duration=%.2fs", ep + 1, total_reward, step + 1, duration)
    finally:
        env.close()


def parse_goal(s: str):
    try:
        x_str, y_str = s.split(",")
        return float(x_str), float(y_str)
    except Exception:
        raise argparse.ArgumentTypeError("Goal must be formatted as 'x,y' with numeric values")


def main():
    parser = argparse.ArgumentParser(description="ROS demo: connect to simulator and take random actions")
    parser.add_argument("--episodes", type=int, default=5, help="Number of episodes to run")
    parser.add_argument("--max-steps", type=int, default=200, help="Max steps per episode")
    parser.add_argument("--goal", type=parse_goal, default=None, help="Optional goal pose as 'x,y'")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for reproducibility")
    parser.add_argument("--log-level", type=str, default="INFO", help="Logging level")
    args = parser.parse_args()

    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO), format="%(asctime)s %(levelname)s %(message)s")

    try:
        run_demo(episodes=args.episodes, max_steps=args.max_steps, goal=args.goal, seed=args.seed)
    except Exception as e:
        logging.exception("Demo failed: %s", e)


if __name__ == "__main__":
    main()
