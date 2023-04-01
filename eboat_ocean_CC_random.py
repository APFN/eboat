#!/home/eduardo/miniconda3/envs/gazgym/bin/python

import gym
import gym_gazebo
import eboat_gym_gaz
import os
import time
import random
import numpy as np

from gym import wrappers

def truncate(value):
    ival = int(value)
    if ((value - ival) < 0.5):
        return ival
    else:
        return (ival + 1)



def main():
    env = gym.make('GazeboOceanEboatEnvCC-v0')

    num_of_episodes      = 5
    max_steps_in_episode = 5 #600

    start_time        = time.time()
    actions_in_episod = []

    accu_reward = 0
    count = 0
    for episode in range(num_of_episodes):
        done = False

        observations        = env.reset()
        accu_reward_episode = 0

        for step in range(max_steps_in_episode):
            action    = env.action_space.sample()  # -->Explore action space
            action[0] = truncate(action[0])

            observations, reward, done, info = env.step(action)
            accu_reward         += reward
            accu_reward_episode += reward
            count               += 1
            # print(episode,step, action, reward)

            if done:
                break
        print("Mean reward in episode {}: {}".format(episode, accu_reward_episode/max_steps_in_episode))

    print("Mean reward : {}".format(accu_reward / count))

    env.close()
    os.system('/home/eduardo/USVSim/eboat_ws/kill_gaz.sh')

if __name__ == '__main__':
    main()
    # env = gym.make('GazeboOceanEboatEnvCC-v0')
    # print(env.action_space.sample())
    # env.close()
    # os.system('/home/eduardo/USVSim/eboat_ws/kill_gaz.sh')