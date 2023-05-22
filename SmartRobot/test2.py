if __name__ == "__main__":
    from stable_baselines3 import PPO
    from env2 import *
    import time

    env = MyEnv(True)
    #加载模型
    model_1 = PPO.load("model_trained_3",env=env)
    
    obs = env.reset()

    # 验证十次
    while True:
        
        action, state = model_1.predict(observation=obs)
        obs, reward, done, info = env.step(action)
        env.render('human')
    
        if done:
            ob = env.reset()
            print("done")
            print(reward)
            time.sleep(1/300)