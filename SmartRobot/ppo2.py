if __name__ == "__main__":
    from stable_baselines3 import PPO
    from env2 import *
    #初始环境
    env = MyEnv()
    #选择策略并训练
    model = PPO(policy="MlpPolicy", env=env,verbose=1)
    print('x')
    model_trained = model.learn(total_timesteps=4000)
    print('x')

    #保存模型
    model_trained.save("model_trained_3")
    print('x')
    #删除模型（没有必要
    del model_trained