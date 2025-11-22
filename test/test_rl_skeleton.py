import pytest


def test_rl_env_dry_run_imports_and_steps():
    from rl.env import RLEnv
    import random

    env = RLEnv(use_ros=False)
    state = env.reset()
    assert hasattr(state, "shape"), "state should be array-like"
    total = 0.0
    for _ in range(5):
        a = random.randrange(len(env.ACTIONS))
        s, r, done, info = env.step(a)
        total += r
    assert isinstance(total, float)
