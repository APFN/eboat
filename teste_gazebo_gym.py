#!/home/eduardo/miniconda3/envs/esailor/bin/python

import os

import gym

import gym
import gym_gazebo
import eboat_gym_gaz

from gym import wrappers
from stable_baselines3 import A2C, PPO, DQN, SAC
from stable_baselines3.common.env_checker import check_env

import tensorboard

env = gym.make('GazeboOceanEboatEnvCC-v1')

print("\n\n-----------------------------\nAMBIENTE CARREGADO COM SUCESSO\n-----------------------------\n")

model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./logs")
print("\n\n-----------------------------\nMODEL CRIADO COM SUCESSO COM SUCESSO\n-----------------------------\n")
model.learn(total_timesteps = (5*2048),
            log_interval    = 1,             #-->NUMBER OF BATCHS BETWEEN TWO CONSECUTIVE LOGS
            tb_log_name     = "PPO_teste",
            progress_bar    = True
            )
print("\n\n-----------------------------\nTREINAMENTO CHEGOU AO FIM\n-----------------------------\n")

env.close()
os.system('/home/eduardo/USVSim/eboat_ws/kill_gaz.sh')