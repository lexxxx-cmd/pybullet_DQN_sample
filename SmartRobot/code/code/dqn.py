if __name__ == "__main__":
    from stable_baselines import PPO2
    from stable_baselines import deepq
    env = MySim()
    
    model = deepq.DQN(policy="MlpPolicy", env=env)
    model.learn(total_timesteps=10000)

    obs = env.reset()
    # 验证十次
    for _ in range(10):
        action, state = model.predict(observation=obs)
        print(action)
        obs, reward, done, info = env.step(action)
        env.render()