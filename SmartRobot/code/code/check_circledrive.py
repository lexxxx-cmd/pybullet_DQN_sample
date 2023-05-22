if __name__ == "__main__":
    env = CircleDrive(render=False)
    from stable_baselines3.common.env_checker import check_env
    check_env(env)