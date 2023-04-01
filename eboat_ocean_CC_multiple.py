#!/home/alvaro/miniconda3/envs/esailor/bin/python

#-->PYTHON UTIL

import os
import time
import random
import numpy as np
from datetime import datetime

#-->GYM
import gym

#-->GAZEBO GYM
import gym_gazebo
import eboat_gym_gaz

#-->STABLE-BASELINES3
from gym import wrappers
from stable_baselines3 import A2C, PPO, DQN, SAC

#-->PYTORCH
import torch as th

#fechar tudo corretamente
import signal
import psutil
import os

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel, GetWorldProperties
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler

#função para fechar tudo corretamente      
def signal_handler(sig, frame):
    # Esperar todos processos do gazebo e ros fecharem
    while True:
        ros_processes = []
        gazebo_processes = []
        
        # Get all running processes
        for proc in psutil.process_iter(['pid', 'name']):
            if 'ros' in proc.info['name']:
                ros_processes.append(proc.info['pid'])
            elif 'gazebo' in proc.info['name']:
                gazebo_processes.append(proc.info['pid'])
        
        # If there are no ROS or Gazebo processes, exit the loop
        if not ros_processes and not gazebo_processes:
            break
        
        # Wait for 1 second before checking again
        time.sleep(1)
    
    
    print('Closing all Python processes...')
    for proc in psutil.process_iter(['pid', 'name']):
        if proc.name() == 'python':
            print(proc.info['name'], proc.info['pid'])
            proc.kill()
            os.kill(proc.info['pid'], signal.SIGKILL)
    
#recebe o sinal ctrl c chama função para fechar tudo          
signal.signal(signal.SIGINT, signal_handler)


def truncate(value):
    ival = int(value)
    if ((value - ival) < 0.5):
        return ival
    else:
        return (ival + 1)

def runA2C(policy, env,learning_rate=0.0007, n_steps=5, gamma=0.99, gae_lambda=1.0, ent_coef=0.001, vf_coef=0.5,
           max_grad_norm=0.5, rms_prop_eps=1e-05, use_rms_prop=True, use_sde=False, sde_sample_freq=-1,
           normalize_advantage=False, tensorboard_log=None, policy_kwargs=None, verbose=0, seed=None, device='auto',
           init_setup_model=True, sufix = ""):

    sufix      = sufix + "_" + datetime.now().strftime("%d%m%Y_%H_%M_%S")
    models_dir = f"models/A2C/{sufix}"

    if not os.path.exists(models_dir):
        os.makedirs(models_dir)

    model = A2C(policy              = policy,
                env                 = env,
                learning_rate       = learning_rate,
                n_steps             = n_steps,
                gamma               = gamma,
                gae_lambda          = gae_lambda,
                ent_coef            = ent_coef,
                vf_coef             = vf_coef,
                max_grad_norm       = max_grad_norm,
                rms_prop_eps        = rms_prop_eps,
                use_rms_prop        = use_rms_prop,
                use_sde             = use_sde,
                sde_sample_freq     = sde_sample_freq,
                normalize_advantage = normalize_advantage,
                tensorboard_log     = tensorboard_log,
                policy_kwargs       = policy_kwargs,
                verbose             = verbose,
                seed                = seed,
                device              = device,
                _init_setup_model   = init_setup_model
                )

    tb_log_name = f"A2C_{sufix}"

    return model, models_dir, tb_log_name

def runPPO(policy, env, learning_rate=0.0003, n_steps=2048, batch_size=64, n_epochs=10, gamma=0.99, gae_lambda=0.95,
           clip_range=0.2, clip_range_vf=None, normalize_advantage=True, ent_coef=0.0, vf_coef=0.5, max_grad_norm=0.5,
           use_sde=False, sde_sample_freq=-1, target_kl=None, tensorboard_log=None, policy_kwargs=None, verbose=0,
           seed=None, device='auto', init_setup_model=True, sufix = ""):

    sufix      = sufix + "_" + datetime.now().strftime("%d%m%Y_%H_%M_%S")
    models_dir = f"models/PPO/{sufix}"

    if not os.path.exists(models_dir):
        os.makedirs(models_dir)
    print("##### chamou ppo ######")
    model = PPO(policy              = policy,
                env                 = env,
                learning_rate       = learning_rate,
                n_steps             = n_steps,
                batch_size          = batch_size,
                n_epochs            = n_epochs,
                gamma               = gamma,
                gae_lambda          = gae_lambda,
                clip_range          = clip_range,
                clip_range_vf       = clip_range_vf,
                # normalize_advantage = normalize_advantage,
                ent_coef            = ent_coef,
                vf_coef             = vf_coef,
                max_grad_norm       = max_grad_norm,
                use_sde             = use_sde,
                sde_sample_freq     = sde_sample_freq,
                target_kl           = target_kl,
                tensorboard_log     = tensorboard_log,
                policy_kwargs       = policy_kwargs,
                verbose             = verbose,
                seed                = seed,
                device              = device,
                _init_setup_model   = init_setup_model
                )
    print("##### saiu do ppo ######")
    tb_log_name = f"PPO_{sufix}"

    return model, models_dir, tb_log_name

def htime(input):
    if input >= 3600:
        h = int(input // 3600)
        t = input % 3600
        m = int(t // 60)
        s = t % 60
        return "{:02d}h {:02d}m {:4.2f}s".format(h, m, s)
    elif input >= 60:
        m = int(input // 60)
        s = input % 60
        return "{:02d}m {:4.2f}s".format(m, s)
    else:
        return "{:4.2f}s".format(input)

def actionRescale(action):
    raction = np.zeros(3, dtype = np.float32)
    #--> Eletric propulsion [-5, 5]
    raction[0] = action[0] * 5.0
    #--> Boom angle [0, 90]
    raction[1] = (action[1] + 1) * 45.0
    #--> Rudder angle [-60, 60]
    raction[2] = action[2] * 60.0
    return raction


def model_exists(model_name):
    # Initialize the ROS node and the service client
    get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)

    # Get the list of model names in the world
    world_properties = get_world_properties()
    model_names = world_properties.model_names

    # Check if the model exists in the world
    return model_name in model_names

def runTrainingv0(env, logdir, sufix="model1"):

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    rospy.wait_for_service('/gazebo/delete_model')
    spawn_model   = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model  = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    policy_kwargs = dict(activation_fn=th.nn.ReLU,
                         net_arch=[dict(pi=[32, 32], vf=[32, 32])]
                         )
    print("##### entrou no runtraining0 ######")
    model, models_dir, TB_LOG_NAME = runPPO(policy          = "MlpPolicy",
                                            env             = env,
                                            tensorboard_log = logdir,
                                            ent_coef        = 0.001,
                                            verbose         = 0,
                                            policy_kwargs   = policy_kwargs,
                                            sufix           = sufix)
    print("##### saiu do runPPo ######")
    
    SAVESTEPS = 100+1
    TIMESTEPS = 2048*5
    
    # SAVESTEPS = 1+1
    # TIMESTEPS = 50*1
     #-->DEFINE NAVIGATION PATH
    # navpath = [[-100, 0, 0.5],
    #            [0, 100, 0.5],
    #            [100, 0, 0.5],
    #            [0, -100, 0.5],
    #            [75, 75, 0.5],
    #            [75, -75, 0.5],
    #            [-75, -75, 0.5],
    #            [-75, 75, 0.5]]

    navpath = [[-100, 0, 0.5],               
               [100, 0, 0.5],
               [0, 100, 0.5]]
    
    start     = time.time()
    model.save(f"{models_dir}/eboat_ocean_0")
    
        
    
    waypoint_count=0
    for waypoint in navpath:
        try:
            delete_model("wayPointMarker")
        except rospy.ServiceException:
            print("/gazebo/get_model_state service call failed")
        ipose = Pose()
        ipose.position.x = waypoint[0]
        ipose.position.y = waypoint[1]
        ipose.position.z = waypoint[2]
        model_path='/home/alvaro/eboat_ws/src/eboat_gz_1/eboat_description/models/wayPointMarker/model.sdf'
        with open(model_path, 'r') as model_file:
            model_xml = model_file.read()
            time.sleep(2)
            print("Abriu modelo do waypoint")
        try:                    
            spawn_model("wayPointMarker",
                        model_xml,
                        'robotos_name_space',
                        ipose, 
                        "world")
            print("spawn_model")
            time.sleep(2)
        except rospy.ServiceException:
            print("/gazebo/SpawnModel service call failed")
        for i in range(1, SAVESTEPS):            
            print("\n\n---------------------------------------------------------")
            print(f"iteration                   : {i}")
            print(f"waypoint_count              : {waypoint_count}")            
            model.learn(total_timesteps     = TIMESTEPS  ,
                        log_interval        = 1          ,
                        tb_log_name         = TB_LOG_NAME,
                        reset_num_timesteps = False     
                        )        
            print("#### saiu do learn ###")
            model.save(f"{models_dir}/eboat_ocean_W_{waypoint_count}_i_{i}")
            timeA = time.time()
            timeB  = time.time()
            avtime = (timeB - start) / i
            print(f"Time spent in this iteration: {htime(timeB - timeA)}")
            print(f"Average time per iteration  : {htime(avtime)}")
            print(f"Elapsed time                : {htime(timeB - start)}")
            print(f"Remaining time              : {htime((SAVESTEPS - i)*avtime)}")
        
        waypoint_count+=1
       

def setWayPoint(model_name="wayPointMarker", Pos = None):
    state = ModelState()
    state.model_name = model_name
    state.reference_frame = "world"
    # pose
    if Pos != None:
        state.pose.position.x = Pos[0]
        state.pose.position.y = Pos[1]
        state.pose.position.z = Pos[2]
    else:
        state.pose.position.x = 0
        state.pose.position.y = 0
        state.pose.position.z = 0
    quaternion = quaternion_from_euler(0, 0, 0)
    state.pose.orientation.x = quaternion[0]
    state.pose.orientation.y = quaternion[1]
    state.pose.orientation.z = quaternion[2]
    state.pose.orientation.w = quaternion[3]
    # twist
    state.twist.linear.x = 0
    state.twist.linear.y = 0
    state.twist.linear.z = 0
    state.twist.angular.x = 0
    state.twist.angular.y = 0
    state.twist.angular.z = 0

    return state

def runTrainingv2(env, logdir, sufix="model2"):
    policy_kwargs = dict(activation_fn=th.nn.ReLU,
                         net_arch=(dict(pi=[32, 32], vf=[32, 32]))
                         )

    model, models_dir, TB_LOG_NAME = runPPO(policy          = "MlpPolicy",
                                            env             = env,
                                            tensorboard_log = logdir,
                                            ent_coef        = 0.001,
                                            verbose         = 0,
                                            policy_kwargs   = policy_kwargs,
                                            sufix           = sufix)

    SAVESTEPS = 50+1
    TIMESTEPS = 2048*5
    start     = time.time()
    model.save(f"{models_dir}/eboat_ocean_0")
    for i in range(1, SAVESTEPS):
        print("\n\n---------------------------------------------------------")
        print(f"iteration                   : {i}")
        timeA = time.time()

        model.learn(total_timesteps     = TIMESTEPS  ,
                    log_interval        = 1          ,
                    tb_log_name         = TB_LOG_NAME,
                    reset_num_timesteps = False      ,
                    progress_bar        = True
                    )
        model.save(f"{models_dir}/eboat_ocean_{i}")

        timeB  = time.time()
        avtime = (timeB - start) / i
        print(f"Time spent in this iteration: {htime(timeB - timeA)}")
        print(f"Average time per iteration  : {htime(avtime)}")
        print(f"Elapsed time                : {htime(timeB - start)}")
        print(f"Remaining time              : {htime((SAVESTEPS - i)*avtime)}")

def main():
    logdir = "logs"
    if not os.path.exists(logdir):
        os.makedirs(logdir)

    training_version = 0

    if training_version == 0:
        env = gym.make('GazeboOceanEboatEnvCC-v0')
        runTrainingv0(env, logdir)
    elif training_version == 1:
        env = gym.make('GazeboOceanEboatEnvCC-v1')
        runTrainingv0(env, logdir)
    else:
        env = gym.make('GazeboOceanEboatEnvCC-v2')
        runTrainingv2(env, logdir)

    print("---------------------------------------------------------\n")

    env.close()
    os.system('/eboat_ws/kill_gaz.sh')

def runModel():
    apwindstr = ["from stern (vento de popa)",
                 "from stern 45 degree port to starboard (de popa, 45 graus bombordo->estibordo)",
                 "from stern 45 degrees starboard to port (de popa, 45 graus estibordo->bombordo",
                 "from bow 45 degree port to starboard (de popa, 45 graus bombordo->estibordo)",
                 "from bow 45 degrees starboard to port (de popa, 45 graus estibordo->bombordo",
                 "from bow (vento de proa)"]
    apwind = np.array([[0    , 9    , 0],
                       [6.36 , 6.36 , 0],
                       [-6.36, 6.36 , 0],
                       [6.36 , -6.36, 0],
                       [-6.36, -6.36, 0],
                       [0    , -9   , 0]])

    training_version = 0

    if training_version == 0:
        env = gym.make('GazeboOceanEboatEnvCC-v0')
        model = PPO.load(f"/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/models/PPO/model1_14022023_21_31_47/eboat_ocean_50")
    elif training_version == 1:
        env = gym.make('GazeboOceanEboatEnvCC-v1')
        model = PPO.load(f"/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/models/PPO/model1_14022023_21_31_47/eboat_ocean_50")
    else:
        env = gym.make('GazeboOceanEboatEnvCC-v2')
        model = PPO.load(f"/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/models/PPO/model2_06022023_21_06_41/eboat_ocean_50")

    # model.set_env(env)

    for episode in range(apwind.shape[0]):
        env.setWindSpeed(apwind[episode])
        obs = env.reset()
        print(f"--------------------------------\nEPISODE {episode} ({apwindstr[episode]})")
        for step in range(60):
            action, _state = model.predict(obs)
            action[0] = np.floor(action[0])
            obsstr = "["
            for val in obs: obsstr += "{:6.3f}, ".format(val)
            obsstr +="]"
            obsstr.replace(", ]", "]")
            actstr = "["
            for val in actionRescale(action): actstr += "{:6.3f}, ".format(val)
            actstr += "]"
            actstr.replace(", ]", "]")
            print("{:2d}  {:s} --> {:s}".format(step, obsstr, actstr), end="")
            obs, reward, done, info = env.step(action)
            print(" (reward = {:5.2f})".format(reward))
            if done:
                break
        # break
    env.close()
    os.system('/home/eduardo/USVSim/eboat_ws/kill_gaz.sh')

if __name__ == '__main__':
    main()
    # runModel()
    # test()

    # env = gym.make('GazeboOceanEboatEnvCC-v0')
    # check_env(env)
    # env.close()
    # os.system('/home/eduardo/USVSim/eboat_ws/kill_gaz.sh')