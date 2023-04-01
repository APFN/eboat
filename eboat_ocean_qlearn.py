#!/home/eduardo/miniconda3/envs/gazgym/bin/python

import gym
from gym import wrappers
import gym_gazebo
import eboat_gym_gaz
import time
import numpy as np
import random
import os

def render():
    render_skip = 0 #_>Skip first X episodes.
    render_interval = 50 #->Show render every Y episodes.
    render_episodes = 10 #->Show Z episodes every rendering.

    if (x % render == 0) and (x != 0) and (x > render_skip):
        env.render()
    elif ((x - render_episodes) % render_interval == 0) and (x != 0) and (x > render_skip) and (render_episodes < x):
        env.render(close=True)

def trunc(val):
    output = int(val)
    if (val > 0) & ((val - output) >= 0.5):
        output += 1
    elif (val < 0) & ((val - output) <= -0.5):
        output -= 1
    return output


def stateFromAngle(observation, delta):
    if observation > 0:
        state = (observation + 179) // delta + 1
    else:
        state = (observation + 180) // delta

    #print(observation, observation + 180, state, (observation + 180) // delta)
    return state


def stateFromObservations(observations, delta):
    #--> Angle between the forward vector and the vector connecting the boat and the waypoint.
    #    interval = [-180, 180] / unit = degrees
    state = [stateFromAngle(trunc(observations[1]), delta)]

    #--> Boat linear velocity
    #    unit = m/s
    state.append(trunc(observations[2]))

    # --> Angle between the apparent wind and the sailing point (forward/trajectory direction)
    #    interval = [-180, 180] / unit = degrees
    state.append(stateFromAngle(trunc(observations[4]), delta))

    return state

def main():
    env = gym.make('GazeboOceanEboatEnvD-v0')

    num_of_episodes     = 10000
    max_steps_in_episode = 600

    #--> Hyperparamters
    alpha         = 0.1
    gamma         = 0.65
    epsilon       = 1.0
    epsilon_min   = 0.1
    epsilon_decay = 0.995

    #--> Initialize q-table
    qtable_shape = []
    for i in range(env.observation_space.shape[0]):
        qtable_shape.append(env.observation_space.nvec[i])
    qtable_shape.append(env.action_space.n)
    qtable = np.zeros(qtable_shape, dtype=float)

    #--> actions that do not use the eletric propultion starts with positive q-value
    # A = np.array(env.action_vec)
    # idx = np.where(A[:,0] == 0)[0]
    # qtable[:,:,:,idx] += 1


    print("\n\n------------------------------------------------")
    print(qtable.shape)
    print("------------------------------------------------\n")

    start_time = time.time()
    actions_in_episod = []
    for ep in range(num_of_episodes):
        done = False

        max_reward, max_penalty, cumulated_reward = 0, 0, 0

        observations, info = env.reset()
        state = stateFromObservations(observations, 10)

        for step in range(max_steps_in_episode):
            if random.uniform(0, 1) < epsilon:
                action = env.action_space.sample() #-->Explore action space
            else:
                action = np.argmax(qtable[state[0],state[1],state[2],:])  #-->Exploit learned values

            if epsilon > epsilon_min:
                epsilon *= epsilon_decay
            elif epsilon < epsilon_min:
                epsilon = epsilon_min

            observations, reward, done, _, info = env.step(action)
            next_state = stateFromObservations(observations, 10)
            if next_state == state:
                reward = -5

            old_value = qtable[state[0], state[1], state[2], action]
            next_max  = np.max(qtable[next_state[0], next_state[1], next_state[2],:])

            new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)
            qtable[state[0], state[1], state[2], action] = new_value

            cumulated_reward += reward
            if reward > max_reward:
                max_reward = reward
            if reward < max_penalty:
                max_penalty = reward

            actions_in_episod.append([ep + 1, max_reward, action])

            # print("------------------------------------------------------------")
            # print("Step {:d}:".format(step))
            # print("       Action     : {}".format(env.action_vec[action]))
            # print("       Observation: {}".format(observations))
            # print("       reward     : {}".format(reward))
            # print("       q-value    : {} --> {}".format(old_value, new_value))

            if done:
                break
            else:
                state = next_state

        #-->Save QTABLE
        #if (ep > 0) & (ep % 20 == 0):
        #    np.save("./qtable.npy", qtable)
        #    np.save("./action_list.npy", np.array(actions_in_episod, dtype = int))

        print("------------------------------------------------")
        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        print("EP {:d}: number of steps     = {:d} ({:02d}:{:02d}:{:02d})".format(ep+1, step, h, m, s))
        print("       max reward earned   = {}".format(max_reward))
        print("       max penalty earned  = {}".format(max_penalty))
        print("       total reward earned = {}".format(cumulated_reward))

    print("End training!")
    print("Time: {:02d}:{:02d}  :{:02d}".format(h, m, s))
    print("Number of episodes   : {}".format(i))
    print("Total number of steps: {}".format(len(actions_in_episod)))
    print("Saving the aquired knoledge...")
    #np.save("./qtable.npy", qtable)
    #np.save("./action_list.npy", np.array(actions_in_episod, dtype=int))
    print("knoledge saved!")
    del(qtable)

    env.close()
    os.system('/home/eduardo/USVSim/eboat_ws/kill_gaz.sh')

if __name__ == '__main__':
    main()
    # #os.system("source /home/eduardo/USVSim/eboat_ws/devel/setup.bash")
    # env = gym.make('GazeboOceanEboatEnv-v0')
    #
    # total_episodes = 3 #1000
    # max_steps      = 30
    # highest_reward = 0
    # seed           = [42]
    #
    # # -->Hyperparamters
    # alpha = 0.1
    # gamma = 0.6
    # epsilon = 0.15
    #
    # # -->For plotting metrics
    # all_epochs = []
    # all_penalties = []
    #
    # actions_in_episod = []
    #
    # #-->Initial q-table
    # qtable_shape = []
    # for i in range(env.observation_space.shape[0]):
    #     qtable_shape.append(env.observation_space.nvec[i])
    # qtable_shape.append(env.action_space.n)
    # qtable = np.zeros(qtable_shape, dtype=float)
    # first_run = False
    # if first_run:
    #     #-->Initial Knoledge
    #     # print(qtable[0,0,0,0].shape)
    #     action_vec = np.array(env.action_vec)
    #     #-->If wind speed not equal to zero, actions that do not use eletric propulsion starts with small positive q-value
    #     idx = np.where(action_vec[:,0] == 0)
    #     qtable[:, :, 1, :, idx] = 1
    #     # -->If wind speed equal to zero, actions that do use eletric propulsion forward starts with small positive q-value
    #     idx = np.where(action_vec[:, 0] > 0)
    #     qtable[:, :, 1, :, idx] = 0.5
    #     #-->Depending on the port/starboard position of the destination (waypoint) rudder angles that makes the boat turn in that direction starts with positive q-values
    #     #-> If the trajectory angle is positive, the destination is on the left, the rudder angle should be negative
    #     idx = np.where(action_vec[:, 2] < 0)[0]
    #     for i in range(10, 19): #->positive trajectory angles
    #         qtable[:,i,:,:,idx] += 1
    #     #-> If the trajectory angle is negative, the destination is on the right, the rudder angle should be positive
    #     idx = np.where(action_vec[:, 2] > 0)[0]
    #     for i in range(0,9): #->negative trajectory angles
    #         qtable[:,i,:,:,idx] += 1
    #     #--> If the trajectory angle is in the interval [-40,40], keep the rudder in the interval [-10,10] is recommended
    #     idx = np.where((action_vec[:, 2] <= 10) & (action_vec[:, 2] >= -10))[0]
    #     for i in range(8, 11): #->trajectory angles in [-40,40[
    #         qtable[:, i, :, :, idx] += 1
    #     #--> If wind angle >= 160 and wind speed > 0, then sail angle >= 60
    #     idx = np.where(action_vec[:,1] >= 60)[0]
    #     for i in [0, 18]:
    #         qtable[:, :, 1, i, idx] += 1
    #     # -->For all states, actions that do use the eletric propulsion starts with small negative q-value
    #     # idx = np.where(action_vec[:, 0] != 0)[0]
    #     # qtable[:, :, :, :, idx] = -2
    #
    #
    #     # idx = np.where(qtable > 0)[4]
    #     # print(action_vec[idx, :].shape)
    #     #
    #     # idx = np.where(qtable < 0)[4]
    #     # print(action_vec[idx,:].shape)
    #     del(action_vec)
    # else:
    #     qtable = np.load('qtable.npy')
    #
    # print("\n\n-------------------------------------\n", qtable.shape, "\n-----------------------------------------\n")
    #
    # start_time = time.time()
    #
    # for i in range(total_episodes):
    #     done = False
    #
    #     max_reward, max_penalty, cumulated_reward = 0, 0, 0
    #
    #     #-->Reset the wind speed
    #     #env.setWindSpeed()
    #
    #     observations, info = env.reset()
    #
    #     state = stateFromObservations(observations)
    #
    #     step, penalties, reward = 0, 0, 0
    #
    #     while (not done) & (step < max_steps):
    #         if random.uniform(0, 1) < epsilon:
    #             action = env.action_space.sample() #-->Explore action space
    #             # print("*Step {}: action = {}/{}".format(step, action, env.action_vec[action]))
    #         else:
    #             action = np.argmax(qtable[state[0],state[1],state[2],state[3]])  #-->Exploit learned values
    #             # print("Step {}: action = {}/{}".format(step, action, env.action_vec[action]))
    #
    #         observations, reward, done, _, info = env.step(action)
    #         next_state = stateFromObservations(observations)
    #         if next_state == state:
    #             reward = -5
    #         elif next_state[0] >= env.observation_space.nvec[0]:
    #             next_state[0] = env.observation_space.nvec[0] - 1
    #
    #         # while next_state == state:
    #         #     observations, reward, done = env.waitForIt()
    #         #     next_state = stateFromObservations(observations)
    #
    #         actions_in_episod.append([i+1, env.getWindSpeed(), action])
    #
    #         old_value = qtable[state[0], state[1], state[2], state[3], action]
    #         next_max  = np.max(qtable[next_state[0], next_state[1], next_state[2], next_state[3]])
    #
    #         new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)
    #         qtable[state[0], state[1], state[2], state[3], action] = new_value
    #
    #         cumulated_reward += reward
    #         if reward > max_reward:
    #             max_reward = reward
    #         if reward < max_penalty:
    #             max_penalty = reward
    #
    #
    #         # print("------------------------------------------------------------")
    #         # print("Step {:d}:".format(step))
    #         # print("       Action     : {}".format(action_vec[action]))
    #         # print("       Observation: {}".format(observations))
    #         # print("       reward     : {}".format(reward))
    #         # print("       q-value    : {} --> {}".format(old_value, new_value))
    #
    #         state = next_state
    #
    #         step += 1
    #
    #     #-->Save QTABLE
    #     # if (i > 0) & (i % 20 == 0):
    #     #     np.save("./qtable.npy", qtable)
    #     #     np.save("./action_list.npy", np.array(actions_in_episod, dtype = int))
    #
    #     print("------------------------------------------------")
    #     m, s = divmod(int(time.time() - start_time), 60)
    #     h, m = divmod(m, 60)
    #     print("EP {:d}: number of steps     = {:d} ({:02d}:{:02d}:{:02d})".format(i+1, step, h, m, s))
    #     print("       max reward earned   = {}".format(max_reward))
    #     print("       max penalty earned  = {}".format(max_penalty))
    #     print("       total reward earned = {}".format(cumulated_reward))
    #
    # print("End training!")
    # print("Time: {:02d}:{:02d}  :{:02d}".format(h, m, s))
    # print("Number of episodes   : {}".format(i))
    # print("Total number of steps: {}".format(len(actions_in_episod)))
    # print("Saving the aquired knoledge...")
    # # np.save("./qtable.npy", qtable)
    # # np.save("./action_list.npy", np.array(actions_in_episod, dtype=int))
    # print("knoledge saved!")
    # del(qtable)
    # env.close()
    # os.system('/home/eduardo/USVSim/eboat_ws/kill_gaz.sh')



