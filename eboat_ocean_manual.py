#!/home/eduardo/miniconda3/envs/esailor/bin/python

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

def actionRescale(action):
    raction = np.zeros(3, dtype = np.float32)
    #--> Eletric propulsion [-5, 5]
    raction[0] = action[0] / 5.0
    #--> Boom angle [0, 90]
    raction[1] = (action[1] / 45.0) - 1
    #--> Rudder angle [-60, 60]
    raction[2] = action[2] / 60.0
    return raction

def observationRescale(observations):
    lo   = len(observations)
    robs = np.zeros(lo, dtype = np.float32)
    #--> Distance from the waypoint (m) [0   , 200];
    robs[0] = (observations[0] + 1) * 125 * 0.5
    #--> Trajectory angle               [-180, 180];
    robs[1] = observations[1] * 180.0
    #--> Boat linear velocity (m/s)     [0   , 10 ];
    robs[2] = (observations[2] + 1) * 5
    #--> Aparent wind speed (m/s)       [0   , 30];
    robs[3] = (observations[3] + 1) * 15
    #--> Apparent wind angle            [-180, 180]
    robs[4] = observations[4] * 180.0
    if lo > 5:
        # --> Boom angle                     [0   , 90]
        robs[5] = (observations[5] + 1) * 45.0
        # --> Rudder angle                   [-60 , 60 ]
        robs[6] = observations[6] * 60.0
        # --> Electric propulsion speed      [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]
        robs[7] = observations[7] * 5.0
        # --> Roll angle                     [-180, 180]
        robs[8] = observations[8] * 180.0

    return robs

def main():
    env = gym.make('GazeboOceanEboatEnvCC-v1')

    num_of_episodes      = 2
    max_steps_in_episode = 24 #60 #600

    accu_reward = 0
    count = 0
    for episode in range(num_of_episodes):
        done = False

        observations        = env.reset()
        accu_reward_episode = 0

        action = np.zeros(3, dtype=np.float32)
        for step in range(max_steps_in_episode):
            observations = observationRescale(observations)

            print("--------------------------------------------------------")
            print(f"Episode/step: {episode}/{step}")
            print(f"Observations: Distance from goal   {observations[0]} m")
            print(f"              Trajectory angle     {observations[1]}")
            print(f"              Boat linear velocity {observations[2]} m/s")
            print(f"              Aparent wind speed   {observations[3]} m/s")
            print(f"              Aparent wind angle   {observations[4]}")
            if len(observations) > 5:
                print(f"              Boom angle           {observations[5]}")
                print(f"              Rudder angle         {observations[6]}")
                print(f"              Electric prop speed  {observations[7]}")
                print(f"              Roll angle           {observations[8]}")
                print(f"              BATTERY              {env.BATTERY}")

            input_action = input("Enter Action: ").split(" ")

            if len(input_action) > 1:
                action = np.array(input_action, dtype=np.float32)

            observations, reward, done, info = env.step(actionRescale(action))
            accu_reward         += reward
            accu_reward_episode += reward
            count               += 1
            print(f"Reward            : {reward}")
            print(f"Accumulated reward: {accu_reward_episode}")

            if done:
                break
        print("Mean reward in episode {}: {}".format(episode, accu_reward_episode/max_steps_in_episode))

    print("Mean reward : {}".format(accu_reward / count))

    env.close()
    os.system('/home/eduardo/USVSim/eboat_ws/kill_gaz.sh')

if __name__ == '__main__':
    main()