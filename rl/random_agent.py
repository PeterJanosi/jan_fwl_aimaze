"""Random agent demo for the RL environment skeleton.

Run `python -m rl.random_agent` to execute a few dry-run episodes.
"""
import argparse
import random
from rl.env import RLEnv


def run_episodes(episodes: int = 5, max_steps: int = 200) -> None:
    env = RLEnv(use_ros=False)
    for ep in range(episodes):
        state = env.reset()
        total_reward = 0.0
        for t in range(max_steps):
            action = random.randrange(len(env.ACTIONS))
            state, reward, done, info = env.step(action)
            total_reward += reward
            if done:
                print(f"Episode {ep+1} finished after {t+1} steps, reward={total_reward:.2f}")
                break
        else:
            print(f"Episode {ep+1} reached max steps, reward={total_reward:.2f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=5)
    args = parser.parse_args()
    run_episodes(episodes=args.episodes)


if __name__ == "__main__":
    main()
