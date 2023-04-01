#!/home/eduardo/miniconda3/envs/gazgym/bin/python

import random
import numpy as np

import gym
import gym_gazebo
import eboat_gym_gaz
from gym import wrappers

import os
import time

from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout
from tensorflow.keras.optimizers import Adam

from collections import deque

class DQN:
    def __init__(self, env):
        self.env = env
        self.memory = deque(maxlen=2000)

        self.gamma         = 0.65
        self.epsilon       = 1.0
        self.epsilon_min   = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.005
        self.tau           = 0.125

        self.model         = self.create_model()
        self.target_model  = self.create_model()

    def create_model(self):
        model = Sequential()
        # state_shape = self.env.observation_space.shape
        # model.add(Dense(24, input_dim=state_shape[0], activation="relu"))
        # model.add(Dense(48, activation="relu"))
        # model.add(Dense(24, activation="relu"))
        # model.add(Dense(self.env.action_space.n))



if __name__ == '__main__':
    #os.system("source /home/eduardo/USVSim/eboat_ws/devel/setup.bash")
    env = gym.make('GazeboOceanEboatEnv-v0')
    
    total_episodes = 1
    max_steps      = 1

    start_time = time.time()

    for i in range(total_episodes):
        done = False

        step = 0

        observations, info = env.reset()

        state = observations[:5]

        while (not done) & (step < max_steps):
            action = env.action_space.sample() #-->Explore action space

            observations, reward, done, _, info = env.step(action)
            if observations[0] >= env.DMAX:
                observations[0] = env.DMAX
            next_state = observations[:5]

            print("---------------------------------------------------")
            print("Episode {}".format(i))
            print("    action    : [{}, {}, {}]".format(action[0]-5, action[1], action[2]-60))
            print("    transition: {} --> {}".format(state, next_state))
            print(env.observation_space.shape)
            print(env.action_space.shape)
            print(env.action_space.nvec)

            state = next_state

            step += 1

    env.close()
    os.system('/home/eduardo/USVSim/eboat_ws/kill_gaz.sh')



