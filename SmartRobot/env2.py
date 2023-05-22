import gym
from goals import Goal
from obstacle import block
import pybullet as p
import pybullet_data
import numpy as np
import math
import random
from gym import spaces

class MyEnv(gym.Env):
    def __init__(self,re : bool = False):
        #动作空间，表示前后左右的选择
        self.action_space = spaces.Discrete(7)
        #状态空间，距离
        self.observation_space = spaces.Box(
            #机器人xy位置，,障碍位置，目标位置
            low=np.array([-10, -10, -10, -10,-10, -10],dtype=np.float32),
            high=np.array([10, 10, 10, 10,10, 10],dtype=np.float32)
        )
        self.np_random, _ = gym.utils.seeding.np_random()
        self.renders = re
        #连接引擎
        self._physics_client_id = p.connect(p.GUI if self.renders else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        #最大速度与力
        self.maxV = 10
        self.maxF = 10
        #初始目标位置
        self.goal = None
        self.block = None
        #上一步的距离
        self.prev_dist_to_goal = 0
        self.prev_dist_to_block = 0
        #速度差倍数
        self.t = 2
        #奖励
        self.reward = 0
        #完成指标
        self.done = False
        #每一次训练执行步数，用于奖励函数中的时间惩罚
        self.step_num = 0
        #初始完成后重置环境
        self.reset()

    
    def __apply_action(self, action):
        #表示按键
        act = action
        maxV = self.maxV
        maxF = self.maxF
        t = self.t
        if act==1: # 左前
            p.setJointMotorControlArray(   # 2,3为右 6,7为左
                bodyUniqueId=self.robot,
                jointIndices=[2, 3],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV, maxV],
                forces=[maxF, maxF]
            )
            p.setJointMotorControlArray(
                bodyUniqueId=self.robot,
                jointIndices=[6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV / t, maxV / t],
                forces=[maxF / t, maxF / t]
            )
        elif act==2:  # 右前
            p.setJointMotorControlArray(   # 2,3为右 6,7为左
                bodyUniqueId=self.robot,
                jointIndices=[6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV, maxV],
                forces=[maxF, maxF]
            )
            p.setJointMotorControlArray(
                bodyUniqueId=self.robot,
                jointIndices=[2, 3],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV / t, maxV / t],
                forces=[maxF / t, maxF / t]
           )
        if act==3:        # 向前
            p.setJointMotorControlArray(   
                bodyUniqueId=self.robot,
                jointIndices=[2, 3, 6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV, maxV, maxV, maxV],
                forces=[maxF / t, maxF / t, maxF / t, maxF / t]
            )

        elif act==4:        # 向后
            p.setJointMotorControlArray(   
                bodyUniqueId=self.robot,
                jointIndices=[2, 3, 6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[-maxV / t, -maxV / t, -maxV / t, -maxV / t],
                forces=[maxF / t, maxF / t, maxF / t, maxF / t]
            )


        elif act==5:        # 原地左转
            p.setJointMotorControlArray(   
                bodyUniqueId=self.robot,
                jointIndices=[2, 3, 6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV / t, maxV / t, -maxV / t, -maxV / t],
                forces=[maxF / t, maxF / t, maxF / t, maxF / t]
            )

        elif act==6:        # 原地右转
            p.setJointMotorControlArray(   
                bodyUniqueId=self.robot,
                jointIndices=[2, 3, 6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[-maxV / t, -maxV / t, maxV / t, maxV / t],
                forces=[maxF / t, maxF / t, maxF / t, maxF / t]
           )

        elif act==0:           # 没有按键，则停下
            p.setJointMotorControlArray(   
                bodyUniqueId=self.robot,
                jointIndices=[2, 3, 6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[0, 0, 0, 0],
                forces=[0, 0, 0, 0]
           )

    def __get_observation(self):
        #错误情况排除
        if not hasattr(self, "robot"):
            assert Exception("robot hasn't been loaded in!")
        basePos, baseOri = p.getBasePositionAndOrientation(self.robot, physicsClientId=self._physics_client_id)
        # 先获取位置
        pos = basePos[:2]
        #保证与状态空间的输出一致
        ob = np.array([pos,self.block,self.goal],dtype=np.float32)
        ob = ob.flatten()
        return ob

    def step(self, action):

        #执行动作后返回姿态
        self.__apply_action(action)
        _, baseOri = p.getBasePositionAndOrientation(self.robot, physicsClientId=self._physics_client_id)
        p.stepSimulation(physicsClientId=self._physics_client_id)
        self.step_num += 1
        state = self.__get_observation()

        #判断目标距离
        dist_goal = math.sqrt((state[0]-self.goal[0])**2+(state[1]-self.goal[1])**2)
        #判断障碍距离
        dist_block = math.sqrt((state[0]-self.block[0])**2+(state[1]-self.block[1])**2)

        #调整并确定奖励函数

        #reward = math.tanh(self.prev_dist_to_goal - dist_goal)*100-((((self.step_num)**2)*10)/(self.step_num)**2)
        # #防止摔倒，把俯仰角也考虑,用tanh加大奖惩
        # reward =max(math.tanh(self.prev_dist_to_goal - dist_goal)*100,0)
        reward = math.tanh(self.prev_dist_to_goal - dist_goal - self.prev_dist_to_block + dist_block)*100

        #更新上一步距离
        self.prev_dist_to_goal = dist_goal
        self.prev_dist_to_block = dist_block

        #用旋转角和偏航角判断机器人是否摔倒
        roll,pitch,_ = p.getEulerFromQuaternion(baseOri)

        #判断结束条件
        if (
            self.step_num >30000 or
            abs(roll) > 0.5 or
            abs(pitch) >0.5 or
            dist_block) < 1:
            self.done = True
            reward = reward-10
        elif dist_goal < 1:
             self.done = True
             reward = reward + 50 
        

        info = {}
        state = np.array(state)
        return state, reward, self.done, info
    
    def reset(self):
        p.resetSimulation(physicsClientId=self._physics_client_id)
        p.setGravity(0, 0, -9.8)
        # 不展示GUI的套件
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        # 设置相机位姿
        camera_distance = 5.0
        camera_yaw = 110.0
        camera_pitch = -30.0
        camera_target_position = [10, 10, 7]

        # 放置相机
        p.resetDebugVisualizerCamera(camera_distance,
                                    camera_yaw, camera_pitch,
                                    camera_target_position)

        self.robot = p.loadURDF("r2d2.urdf", basePosition=[0., 0., 0.1], physicsClientId=self._physics_client_id)
        self.plane = p.loadURDF("plane.urdf", physicsClientId=self._physics_client_id)

        #目标位置
        # x = 10*random.random()
        # y = 10*random.random()
        x = -8.
        y = -9.
        block_x = -4.
        block_y = -4.5
        self.goal = (x, y)
        self.block = (block_x,block_y)
        #设定初始距离
        self.prev_dist_to_goal = (math.sqrt((0.-self.goal[0])**2+(0.-self.goal[1])**2))
        #放置鸭子作为目标位置
        Goal(self._physics_client_id, self.goal)
        block(self._physics_client_id, self.block)
        self.step_num = 0

        self.done = False

        return self.__get_observation()
    
    def render(self, mode='human'):
        pass

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
    def close(self):
        if self._physics_client_id >= 0:
            p.disconnect()
        self._physics_client_id = -1
    

if __name__ == "__main__":
    env = MyEnv(re=True)
    from stable_baselines3.common.env_checker import check_env
    check_env(env)
    print('yes')