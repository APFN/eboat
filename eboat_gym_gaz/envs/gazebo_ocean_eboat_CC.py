import gym
import rospy
import roslaunch
import numpy as np
import os
import math

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from std_srvs.srv import Empty

from std_msgs.msg import Float32, Int16, Float32MultiArray, Bool
from geometry_msgs.msg import Point
from gym.utils import seeding

from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

from rosgraph_msgs.msg import Clock
from rospy.rostime import Time

from filterpy.kalman import KalmanFilter

def vet2str(vet):
    vetstr = "["
    for val in vet:
        vetstr += "{:5.3f}, ".format(val)
    vetstr += "]"
    vetstr.replace(",]", "]")
    return vetstr

global actions_buffer_leme, actions_buffer_vela, buffer
actions_buffer_leme = []
actions_buffer_vela = []
# Cria um buffer vazio para armazenar os valores antigos
buffer = []



class GazeboOceanEboatEnvCC(gazebo_env.GazeboEnv):
        
    def __init__(self):   
        print("#################### entrou no CC0 ###################")

        self.reward_global = 0

        self.EBOAT_HOME = "/home/alvaro/eboat_ws/src/eboat_gz_1"
        gazebo_env.GazeboEnv.__init__(self, os.path.join(self.EBOAT_HOME,"eboat_gazebo/launch/ocean.launch"))

        self.boomAng_pub   = rospy.Publisher("/eboat/control_interface/sail"       , Float32, queue_size=5)
        self.rudderAng_pub = rospy.Publisher("/eboat/control_interface/rudder"     , Float32, queue_size=5)
        self.propVel_pub   = rospy.Publisher("/eboat/control_interface/propulsion", Int16  , queue_size=5)
        self.wind_pub      = rospy.Publisher("/eboat/atmosferic_control/wind"      , Point  , queue_size=5)
        self.unpause       = rospy.ServiceProxy('/gazebo/unpause_physics' , Empty)
        self.pause         = rospy.ServiceProxy('/gazebo/pause_physics'   , Empty)
        self.reset_proxy   = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.set_state     = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        #--> GLOBAL VARIABLES
        self.DTOL  = 25.0  #--> Threshold for distance. If the boat goes far than INITIAL POSITION + DMAX, a done signal is trigged.
        self.D0    = None  #--> The intial distance from the waypoint
        self.DMAX  = None
        self.DPREV = None

        #--> We will use a rescaled action space
        self.action_space = spaces.Box(low   = -1 ,
                                       high  = 1  ,
                                       shape = (2,),
                                       dtype = np.float32)

        # --> We will use a rescaled action space
        #--> Actual ranges: Distance from the waypoint (m) [0   , 500];
        #                   Trajectory angle               [-180, 180];
        #                   Boat linear velocity (m/s)     [0   , 10 ];
        #                   Aparent wind speed (m/s)       [0   , 30];
        #                   Apparent wind angle            [-180, 180]
        self.observation_space = spaces.Box(low   = -1 ,
                                            high  = 1  ,
                                            shape = (5,),
                                            dtype = np.float32)
        self.reward_range = (-1, 1)

        self._seed()

        #--> SET WIND SPEED INITIAL VECTOR
        self.windSpeed = np.array([5, 0, 0.0], dtype=np.float32)

        #--> GET INITIAL DISTANCE FROM THE WAYPOINT
        while self.D0 is None:
            try:
                self.D0    = rospy.wait_for_message("/eboat/mission_control/observations",
                                                    Float32MultiArray, timeout = 20).data[0]
                self.DMAX  = self.D0 + self.DTOL  #--> IT TAKE THE INITIAL DISTANCE IN CONSIDERATION
                self.DPREV = self.D0
            except:
                pass

        #--> SUPPORT FOR RANDOM INITIALIZATION OF WIND SPEED AND DIRECTION
        np.random.seed(20)

    def setWindSpeed(self, vector):
        self.windSpeed = vector

    def getWindSpeed(self):
        return self.windSpeed

    def sampleWindSpeed(self, max_wind_speed = 12):
        angle = np.random.uniform(low=-180, high=180) * math.pi / 90
        magnitude = np.random.uniform(low=3, high=12)
        x = magnitude * math.cos(angle)
        y = magnitude * math.sin(angle)
        self.windSpeed[:2] = [x,y]
        
        if np.random.uniform() < 0.1: # 30% das vezes é vento contra. "na cara"
            self.windSpeed[:2] = [(magnitude*-1), 0]
        #print(self.windSpeed)

    def rewardFunction(self, obs, ract):
        #--> Reward;Penalty by decresing/increasing the distance from the goal.
        progre = (self.DPREV - obs[0]) / self.DMAX
        reward = progre

        if obs[2] < 1:
            #--> Have a velocity slower than 0.4 m/s or negative generate a penalty.
            reward -= 0.2
        elif (abs(obs[1]) < 60):
            #--> Have a velocity greater than 0.4 m/s toward the objective generates a reward.
            reward += 0.01 * (1.0 - abs(obs[1]) / 60.0)

            if ((obs[2] > 2) & (obs[7] == 0)):
                #--> Reward for boat speedy greater than 1.0 m/s if the electric engine is offline.
                reward += 2

        # Define o limite máximo de oscilação aceitável
        max_oscilacao = 10
        # Define o tamanho do intervalo de tempo
        time_window = 4
        # Adiciona a ação atual ao buffer de ações
        actions_buffer_leme.append(ract[0])
        actions_buffer_vela.append(ract[1])
        
        # Remove a ação mais antiga do buffer se o seu tamanho exceder o intervalo de tempo
        if len(actions_buffer_leme) > time_window:
            actions_buffer_leme.pop(0)
            actions_buffer_vela.pop(0)
        
        # Calcula o desvio padrão das ações no buffer
        std_dev_leme = np.std(actions_buffer_leme)
        std_dev_vela = np.std(actions_buffer_vela)
        #print("#####oscilou muito o leme", std_dev_leme, actions_buffer_leme)
        #print("#####oscilou muito a vela", std_dev_vela, actions_buffer_vela)
        #print(std_dev_vela, std_dev_leme)
        
        # Se o desvio padrão exceder o limite máximo, penalize o modelo
        if std_dev_leme > max_oscilacao:
            reward += -0.0001
            #print("#####oscilou muito o leme", std_dev_leme, actions_buffer_leme)
        else:
            reward += 1.0
        
        if std_dev_vela > max_oscilacao:
            reward = -0.0001
            #print("#####oscilou muito a vela", std_dev_vela, actions_buffer_vela)
        else:
            reward = 1.0
        
        if obs[7] != 0:
                 reward -= reward + abs(progre)

        #================================Edu==========================
        # progre = (self.DPREV - obs[0]) / self.DMAX
        # reward = progre
        # if obs[2] < 2:
        #     reward = np.min([-2.0*reward, -0.3])
        # else:
        #     if obs[1] < 60:
        #         reward *= 2.0

        #     if obs[7] != 0:
        #         reward -= reward + abs(progre)

        return reward

    def setInitialState(self, model_name, theta):
        state = ModelState()
        state.model_name      = model_name
        state.reference_frame = "world"
        # pose
        state.pose.position.x = 0
        state.pose.position.y = 0
        state.pose.position.z = 0
        quaternion = quaternion_from_euler(0, 0, theta*np.pi/180.0)
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

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = self.set_state
            result = set_state(state)
            assert result.success is True
        except rospy.ServiceException:
            print("/gazebo/get_model_state service call failed")

    def sampleInitialState(self, model_name):
        state = ModelState()
        state.model_name      = model_name
        state.reference_frame = "world"
        theta = np.random.randint(low   = -179,
                                  high  = 180)
        # pose
        self.setInitialState(model_name, theta)

    def getObservations(self):
        obsData = None
        while obsData is None:
            try:
                obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray, timeout=20).data
            except:
                pass
            # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
            #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]
            
        return np.array(obsData, dtype=float)

    def actionRescale(self, action):
        raction = np.zeros(3, dtype = np.float32)
        # #--> Eletric propulsion [-5, 5]
        raction[0] = action[0] * 5.0
        #--> Boom angle [0, 90]
        raction[1] = (action[1] + 1) * 45.0
        #--> Rudder angle [-60, 60]
        raction[2] = action[2] * 60.0
        return raction

    def rescale(self, m, rmin, rmax, tmin, tmax):
        # rmin denote the minimum of the range of your measurement
        # rmax denote the maximum of the range of your measurement
        # tmin denote the minimum of the range of your desired target scaling
        # tmax denote the maximum of the range of your desired target scaling
        # m in [rmin,rmax] denote your measurement to be scaled
        # Then
        # m --> ((m−rmin)/(rmax−rmin))*(tmax-tmin)+tmin
        # will scale m linearly into [tmin,tmax] as desired.
        # To go step by step,
        # m --> m−rmin maps m to [0,rmax−rmin].
        # Next,
        # m --> (m−rmin)/(rmax−rmin)
        # maps m to the interval [0,1], with m=rmin mapped to 0 and m=rmax mapped to 1.
        # Multiplying this by (tmax−tmin) maps m to [0,tmax−tmin].
        # Finally, adding tmin shifts everything and maps m to [tmin,tmax] as desired.
        return (((m - rmin) / (rmax- rmin)) * (tmax - tmin) + tmin)

    def observationRescale(self, observations):
        robs = np.zeros(5, dtype = np.float32)
        #--> Distance from the waypoint (m) [0   , 200];
        robs[0] = observations[0]/100 - 1
        #--> Trajectory angle               [-180, 180];
        robs[1] = observations[1] / 180.0
        #--> Boat linear velocity (m/s)     [0   , 10 ];
        robs[2] = observations[2]/5 - 1
        #--> Aparent wind speed (m/s)       [0   , 30];
        robs[3] = observations[3]/15 - 1
        #--> Apparent wind angle            [-180, 180]
        robs[4] = observations[4] / 180.0

        return robs

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    def simple_moving_average_filter(self, x, window_size):
        
        # Adiciona o valor atual no final do buffer
        buffer.append(x)
        
        # Remove o valor mais antigo do início do buffer
        if len(buffer) > window_size:
            buffer.pop(0)
        
        # Calcula a média móvel do buffer
        filtered_x = sum(buffer) / len(buffer)
        
        return filtered_x


    def step(self, action):
        #print("#################### step no CC0 ###################")
        #--> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except( rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        #-->SEND ACTION TO THE BOAT CONTROL INTERFACE
        ract = self.actionRescale(action)
        # print("sem filtro",ract[0],ract[1])
        # # print(f"---> action = {action}")
        # # print(f"---> ract   = {ract}")
        # ract[0]=self.simple_moving_average_filter(ract[0],3)
        # ract[1]=self.simple_moving_average_filter(ract[1],3)
        # print("com filtro",ract[0],ract[1])        

        self.propVel_pub.publish(int(ract[0]))
        self.boomAng_pub.publish(ract[1])
        self.rudderAng_pub.publish(ract[2])

        
        #-->GET OBSERVATIONS (NEXT STATE)
        observations = self.getObservations()

        #print(f"{vet2str(observations)} --> {vet2str(action)}")

        #-->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except( rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        #-->CHECK FOR A TERMINAL STATE
        #   observation[0] = distance from goal
        #   observation[8] = roll angle
           
        dist = observations[0]
        # print("\n\n-------------------------------------")
        # print("dist <= 5",dist <= 5)
        # print("dist > self.DMAX",dist > self.DMAX)
        # print("dist ",dist )
        # print("self.DMAX ", self.DMAX )
        # print("(observations[8] > 60.0)",(observations[8] > 60.0))
        # print("np.isnan(observations).any())",np.isnan(observations).any())
        done = bool((dist <= 10) | (dist > self.DMAX) | (observations[8] > 60.0) | (np.isnan(observations).any())) #--> Considering that the distance is measured in meters
        #print("done: ", done)
        # print("\n\n-------------------------------------")
        #print("################################################ D0:", self.D0)
        
        if np.isnan(observations).any():
            print("\n\n-------------------------------------")
            print(f"distance: {observations[0]}")
            print(f"traj ang: {observations[1]}")
            print(f"boat vel: {observations[2]}")
            print(f"wind vel: {observations[3]}")
            print(f"wind ang: {observations[4]}")
            print(f"boom ang: {observations[5]}")
            print(f"rud ang : {observations[6]}")
            print(f"prop    : {observations[7]}")
            print(f"roll ang: {observations[8]}")
            print("-------------------------------------\n")
            #--> WAIT FOR ACKNOWLEDGEMENT FROM USER
            # _ = input("Unpause: ")

        
        #-->COMPUTES THE REWARD
        if not done:
            #print("COMPUTES THE REWARD: not done")
            reward  = self.rewardFunction(observations, ract)
            #print(reward)
            self.DPREV = dist                               #-->UPDATE CURRENT DISTANCE
        elif (dist > self.DMAX):
            #print("COMPUTES THE REWARD: dist > self.DMAX")
            reward = -1
        else:
            print("COMPUTES THE REWARD: DONE")
            reward  = 10
            self.reset()
        
        self.reward_global = self.reward_global + reward
        #print("Reward global:", self.reward_global)

        return self.observationRescale(observations), reward, done, {}

    def reset(self):
        print("#################### reset no CC0 ###################")
        #-->RESETS THE STATE OF THE ENVIRONMENT.
        rospy.wait_for_service('/gazebo/reset_simulation')

        #print("##### resetou ######")
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print(("/gazebo/reset_simulation service call failed!"))

        #-->SET RANDOM INITIAL STATE
        self.setInitialState("eboat", 90)
        # self.sampleInitialState("eboat")

        #-->SET RANDOM WIND SPEED AND DIRECTION
        # self.sampleWindSpeed()
        self.wind_pub.publish(Point(self.windSpeed[0], self.windSpeed[1], self.windSpeed[2]))

        #-->SET THE ACTUATORS BACK TO THE DEFAULT SETTING
        self.propVel_pub.publish(0)
        self.boomAng_pub.publish(0.0)
        self.rudderAng_pub.publish(0.0)

        #-->UNPAUSE SIMULATION TO MAKE OBSERVATION
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except( rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        #-->COLLECT OBSERVATIONS
        observations = self.getObservations()

        # -->RESET INITIAL DISTANCE
        self.DPREV = observations[0]
        # self.DMAX  = observations[0] + self.DTOL
        # self.D0 = observations[0]

        #-->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except( rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))
        # print("##### saiu de resetou ######")

        try:
            self.D0 = observations[0]
            #print("################################################ D0:", self.D0)
            self.DMAX  = self.D0 + self.DTOL  #--> IT TAKE THE INITIAL DISTANCE IN CONSIDERATION
            self.DPREV = self.D0
        except:
            pass
        
        print("Reward global:", self.reward_global)
        return self.observationRescale(observations)


class GazeboOceanEboatEnvCC1(GazeboOceanEboatEnvCC):
    
    def clock_callback(self, msg: Clock):
        # Extrai o tempo atual de simulação da mensagem
        self.current_iteration_time = msg.clock

    def __init__(self):
        #print("#################### entrou no CC1 ###################")

        self.reward_global = 0
        self.turn_time = rospy.Time.from_sec(0)
        self.start_time = rospy.Time.from_sec(0)
        self.turning = False
        self.min_dist_goal = 5
        self.lowsSpeedCount = 0

        self.step_count = 0

        self.EBOAT_HOME = "/home/alvaro/eboat_ws/src/eboat_gz_1"
        gazebo_env.GazeboEnv.__init__(self, os.path.join(self.EBOAT_HOME, "eboat_gazebo/launch/ocean.launch"))

        self.boomAng_pub = rospy.Publisher("/eboat/control_interface/sail", Float32, queue_size=5)
        self.rudderAng_pub = rospy.Publisher("/eboat/control_interface/rudder", Float32, queue_size=5)
        self.propVel_pub = rospy.Publisher("/eboat/control_interface/propulsion", Int16, queue_size=5)
        self.wind_pub = rospy.Publisher("/eboat/atmosferic_control/wind", Point, queue_size=5)

        self.flappy_boat_pub = rospy.Publisher('/eboat/control_interface/flappy_boat', Bool, queue_size=10)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        rospy.Subscriber('/clock', Clock, self.clock_callback)
        self.min_iteration_time = [rospy.Time.from_sec(10*60*60)] * 20


        # --> GLOBAL VARIABLES
        self.DTOL = 25.0  # --> Threshold for distance. If the boat goes far than INITIAL POSITION + DMAX, a done signal is trigged.
        self.D0 = None  # --> The intial distance from the waypoint
        self.DMAX = None
        self.DPREV = None

        # --> We will use a rescaled action space
        self.action_space = spaces.Box(low=-1,
                                       high=1,
                                       shape=(2,),
                                       dtype=np.float32)

        # --> We will use a rescaled action space
        self.observation_space = spaces.Box(low=-1,
                                            high=1,
                                            shape=(9,),
                                            dtype=np.float32)
        self.reward_range = (-1, 1)

        self._seed()

        # --> SET WIND SPEED INITIAL VECTOR
        self.windSpeed = np.array([9, 0, 0.0], dtype=np.float32)

        # --> GET INITIAL DISTANCE FROM THE WAYPOINT
        while self.D0 is None:
            try:
                self.D0 = rospy.wait_for_message("/eboat/mission_control/observations",
                                                 Float32MultiArray, timeout=20).data[0]
                self.DMAX = self.D0 + self.DTOL  # --> IT TAKE THE INITIAL DISTANCE IN CONSIDERATION
                self.DPREV = self.D0
            except:
                pass

        # --> SUPPORT FOR RANDOM INITIALIZATION OF WIND SPEED AND DIRECTION
        np.random.seed(20)
        self.max_wind_speed = 6
        self.P = np.arange(-180, 181, 20)
        self.P[0] += 1
        self.A = np.arange(-180, 181, 5)
        self.A[0] += 1
        self.angb = [np.random.choice(self.P, size=self.P.shape[0], replace=False) for i in range(self.max_wind_speed)]
        self.angw = [[np.random.choice(self.A, size=self.A.shape[0], replace=False) for _ in range(self.P.shape[0])] for _ in
               range(self.max_wind_speed)]
        self.count = np.zeros([self.max_wind_speed, 2], dtype=int)

    def rot(self, modulus, theta):
        R = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]], dtype=float)

        return np.dot(np.array([1, 0], dtype=float) * modulus, R)
    
    def actionRescale(self, action):
        raction = np.zeros(3, dtype = np.float32)
        # # #--> Eletric propulsion [-5, 5]
        # raction[0] = action[0] * 5.0
        #--> Boom angle [0, 90]
        raction[0] = (action[0] + 1) * 45.0
        #--> Rudder angle [-60, 60]
        raction[1] = action[1] * 60.0
        return raction
    
    def observationRescale(self, observations):
        lobs = len(observations)
        robs = np.zeros(lobs, dtype=np.float32)
        # --> Distance from the waypoint (m) [0   , DMAX];
        robs[0] = 2 * (observations[0] / self.DMAX) - 1
        # --> Trajectory angle               [-180, 180]
        robs[1] = observations[1] / 180.0
        # --> Boat linear velocity (m/s)     [0   , 10 ]
        robs[2] = observations[2] / 5 - 1
        # --> Aparent wind speed (m/s)       [0   , 30]
        robs[3] = observations[3] / 15 - 1
        # --> Apparent wind angle            [-180, 180]
        robs[4] = observations[4] / 180.0
        if lobs > 5:
            # --> Boom angle                     [0   , 90]
            robs[5] = (observations[5] / 45.0) - 1
            # --> Rudder angle                   [-60 , 60 ]
            robs[6] = observations[6] / 60.0
            # --> Electric propulsion speed      [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]
            robs[7] = observations[7] / 5.0
            # --> Roll angle                     [-180, 180]
            robs[8] = observations[8] / 180.0

        return robs

    def sampleWindSpeed(self, max_wind_speed = 12):
        angle = np.random.uniform(low=-180, high=180) * math.pi / 90
        magnitude = np.random.uniform(low=3, high=12)
        x = magnitude * math.cos(angle)
        y = magnitude * math.sin(angle)
        self.windSpeed[:2] = [x,y]
        
        # if np.random.uniform() < 0.1: # 30% das vezes é vento contra. "na cara"
        #     self.windSpeed[:2] = [(magnitude*-1), 0]
        #print(self.windSpeed)

    def setWindSpeed(self, vector):
        self.windSpeed = vector

    def getWindSpeed(self):
        return self.windSpeed

    def sampleInitialState(self, model_name, max_wind_speed=6):
        #print("############entrou no sampleInitialState CC1")
        speed = max_wind_speed * np.random.sample(size=1)
        s     = int(speed)
        k0    = self.count[s, 0]
        k1    = self.count[s, 1]
        j     = np.where(self.P == self.angb[s][k0])[0][0]
        pos   = self.angb[s][k0]
        theta = self.angw[s][j][k1]
        if k1 < self.A.shape[0] - 1:
            k1 += 1
        elif k0 < self.P.shape[0] - 1:
            k0 += 1
            k1  = 0
        else:
            print(f"\nAconteceu para s = {s}")
            self.count[s, 0] = 0
            self.count[s, 1] = 0
            self.angb[s]     = np.random.choice(self.P, size=self.P.shape[0], replace=False)
            self.angw[s]     = [np.random.choice(self.A, size=self.A.shape[0], replace=False) for _ in range(self.P.shape[0])]

        #-->Set the true wind vector
        ws = self.rot(speed, theta)
        self.windSpeed[:2] = ws

        self.sampleWindSpeed() # !!!!!! reecreve em cima dos ventos sorteados antes

        #--> Set the boat's initial pose
        self.setInitialState(model_name, theta)

    def rewardFunction(self, obs, ract):
        
        # reward = ((self.DPREV - obs[0]) / self.DMAX)        
        
        ## ## usei no modelo que treinei para motor medio e motor livre
        # # if reward > 0:
        # #     reward *= (1.0 - 0.9 * abs(obs[7]) / 5.0)
        # # else:
        # #     reward -= 0.01 * abs(obs[7])

        reward = ((self.DPREV - obs[0]) / self.DMAX)         
        min_speed =  obs[3] / 4 # barco tem que andar a 1/4 da velocidade do vento         
        if obs[2] > min_speed:  # rapido
            reward *=2    


        # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
        #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]     
        return reward

    def step(self, action):

        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except( rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        ract = self.actionRescale(action) #-->SEND ACTION TO THE BOAT CONTROL INTERFACE

        # self.propVel_pub.publish(int(ract[0])) #comentado se motor ligadi      
        self.propVel_pub.publish(int(0)) #comentado se motor desligado    
        self.boomAng_pub.publish(ract[0])
        self.rudderAng_pub.publish(ract[1])
        #self.flappy_boat_pub.publish(True) #Modosem vento

        #-->GET OBSERVATIONS (NEXT STATE)
        observations = self.getObservations()

        #-->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except( rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))
           
        dist = observations[0]        
        
        if np.isnan(observations).any(): #np.isnan(observations).any()
            print("\n\n-------------------------------------")
            print(f"distance: {observations[0]}")
            print(f"traj ang: {observations[1]}")
            print(f"boat vel: {observations[2]}")
            print(f"wind vel: {observations[3]}")
            print(f"wind ang: {observations[4]}")
            print(f"boom ang: {observations[5]}")
            print(f"rud ang : {observations[6]}")
            print(f"prop    : {observations[7]}")
            print(f"roll ang: {observations[8]}")
            print("-------------------------------------\n")
            #--> WAIT FOR ACKNOWLEDGEMENT FROM USER
            # _ = input("Unpause: ")

        reward  = self.rewardFunction(observations, ract)
        self.DPREV = dist #atualiza distancia do objetivo

         #-->CHECK FOR A TERMINAL STATE
        done = bool((self.DPREV <= self.min_dist_goal) | # chegou no objetivo
                    (self.DPREV > self.DMAX) |  # esta muito longe do objetivo
                    (np.isnan(observations).any()) # erro nas observaçoes
                    )                  

        
        wind_slot = math.floor(observations[3])
        ########## COMPUTES THE REWARD  #############
        if done :
            if (self.DPREV <= self.min_dist_goal): #chegou no objetivo                
                if  self.current_iteration_time <= self.min_iteration_time[wind_slot]: #chegou mais rapido
                    self.min_iteration_time[wind_slot] =  self.current_iteration_time
                    reward = 10 #super recompensa  
                    print("=======!!!! DONE !!!!  Super Reward: ", self.reward_global)
                    print("=======Windslot: min_iteration_time: ", wind_slot, self.min_iteration_time[wind_slot])
                else: 
                    reward = 1       
                    print("!!!! DONE !!!!  Reward: ", self.reward_global)         
            else: 
                reward = -1

        
        self.reward_global = self.reward_global + reward

        self.step_count += 1
        
        return self.observationRescale(observations), reward, done, {}
   
    def reset(self):
        #print("#################### reset no CC1 ###################")
        self.step_count = 0
        #-->RESETS THE STATE OF THE ENVIRONMENT.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print(("/gazebo/reset_simulation service call failed!"))

        #-->SET RANDOM INITIAL STATE
        self.sampleInitialState("eboat", self.max_wind_speed)
        self.wind_pub.publish(Point(self.windSpeed[0], self.windSpeed[1], self.windSpeed[2]))


        #-->SET THE ACTUATORS BACK TO THE DEFAULT SETTING
        self.propVel_pub.publish(0)
        self.boomAng_pub.publish(0.0)
        self.rudderAng_pub.publish(0.0)
        #self.flappy_boat_pub.publish(True)

        #-->UNPAUSE SIMULATION TO MAKE OBSERVATION
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except( rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        #-->COLLECT OBSERVATIONS
        observations = self.getObservations()

        # -->RESET INITIAL DISTANCE
        self.DPREV = observations[0]

        #-->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except( rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))
               
        
        if np.random.uniform() < 0.1: 
            print("Global Reward :", self.reward_global)

        return self.observationRescale(observations)


class GazeboOceanEboatEnvCC2(gazebo_env.GazeboEnv):
    def __init__(self):
        #print("#################### entrou no CC2 ###################")
        self.EBOAT_HOME = "/home/alvaro/eboat_ws/src/eboat_gz_1"

        #-->GAZEBO ENVIRONMENT LAUNCH FILE
        gazebo_env.GazeboEnv.__init__(self, os.path.join(self.EBOAT_HOME, "eboat_gazebo/launch/ocean.launch"))

        #-->ROS TOPIS AND SERVICES
        self.boomAng_pub = rospy.Publisher("/eboat/control_interface/sail", Float32, queue_size=5)
        self.rudderAng_pub = rospy.Publisher("/eboat/control_interface/rudder", Float32, queue_size=5)
        self.propVel_pub = rospy.Publisher("/eboat/control_interface/propulsion", Int16, queue_size=5)
        self.wind_pub = rospy.Publisher("/eboat/atmosferic_control/wind", Point, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # --> GLOBAL VARIABLES
        self.DTOL       = 25.0  #--> Threshold for distance. If the boat goes far than INITIAL POSITION + DMAX, a done signal is trigged.
        self.D0         = None  #--> The initial distance from the waypoint.
        self.DMAX       = None  #--> Maximum allowed distance (distanec grater than it will generate a done signal.
        self.DPREV      = None  #--> Distance in the previous state
        self.THETA0     = None  #--> The initial trajectory angle.
        self.BOOM0      = 0.0   #--> The initial boom position.
        self.RUDDER0    = 0.0   #--> The initial rudder position.
        self.BATTERY    = 100   #--> The energy available to perform actions.
        self.ENERGY     = 0.0   #--> The energy consumed in a single time step.
        self.STEP_COUNT = 0

        # --> We will use a rescaled action space
        self.action_space = spaces.Box(low=-1,
                                       high=1,
                                       shape=(3,),
                                       dtype=np.float32)

        # --> We will use a rescaled action space
        self.observation_space = spaces.Box(low=-1,
                                            high=1,
                                            shape=(9,),
                                            dtype=np.float32)
        self.reward_range = (-1, 1)

        self._seed()

        #--> SET THE WIND SPEED INITIAL VECTOR
        self.windSpeed = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        #--> GET INITIAL DISTANCE FROM THE WAYPOINT
        while self.D0 is None:
            try:
                self.D0    = rospy.wait_for_message("/eboat/mission_control/observations",
                                                    Float32MultiArray, timeout = 20).data[0]
                self.DMAX  = self.D0 + self.DTOL  #--> IT TAKE THE INITIAL DISTANCE IN CONSIDERATION
                self.DPREV = self.D0
            except:
                pass

        #--> SET THE RANDOM SEED FOR THE POSITION AND WIND GENERATOR
        np.random.seed(20)

    def setWindSpeed(self, value):
        self.windSpeed = value

    def getWindSpeed(self):
        return self.windSpeed

    def sampleWindSpeed(self, max_wind_speed=12):
        mws2 = max_wind_speed * max_wind_speed
        ws   = (2 * np.random.rand(2) - 1) * max_wind_speed
        while (ws[0]*ws[0] + ws[1]*ws[1]) > mws2:
            ws = (2 * np.random.rand(2) - 1) * max_wind_speed
        self.windSpeed[:2] = ws

    def setBoatState(self, state):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = self.set_state
            result = set_state(state)
            assert result.success is True
        except rospy.ServiceException:
            print("/gazebo/get_model_state service call failed")

    def setInitialState(self, model_name, Pos = None, Rot = None):
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
        if Rot != None:
            rad = np.array(Rot, dtype=np.float32) * np.pi / 180.0
            quaternion = quaternion_from_euler(rad[0], rad[1], rad[2])
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

        self.setBoatState(state)

        return state

    def sampleInitialState(self):
        theta = np.random.randint(low = -179, high = 180)
        #-->SET THE BOAT POSE
        _ = self.setInitialState("eboat", Rot = [0, 0, theta])

        #-->SET THE WAY POINT POSE
        # D = np.random.randint(low = 100, high = 500)
        # x = (2 * np.random.rand() - 1) * D
        # y = np.sqrt(D*D - x*x) * np.random.choice([-1,1])
        # _ = self.setInitialState("wayPointMarker", Pos=[x, y, 0])
        # return [x, y, np.sqrt(x*x + y*y)]

    def getObservations(self):
        obsData = None
        while obsData is None:
            try:
                obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray, timeout=20).data
            except:
                pass
            # --> obsData = [distance, trajectory angle, linear velocity, apparent wind speed, apparent wind angle, boom angle, rudder angle, electric propulsion speed, roll angle]
            #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]

        return np.array(obsData, dtype=float)

    def actionRescale(self, action):
        raction = np.zeros(3, dtype=np.float32)
        # --> Eletric propulsion [-5, 5]
        raction[0] = action[0] * 5.0
        # --> Boom angle [0, 90]
        raction[1] = (action[1] + 1) * 45.0
        # --> Rudder angle [-60, 60]
        raction[2] = action[2] * 60.0
        return raction

    def rescale(self, m, rmin, rmax, tmin, tmax):
        # rmin denote the minimum of the range of your measurement
        # rmax denote the maximum of the range of your measurement
        # tmin denote the minimum of the range of your desired target scaling
        # tmax denote the maximum of the range of your desired target scaling
        # m in [rmin,rmax] denote your measurement to be scaled
        # Then
        # m --> ((m−rmin)/(rmax−rmin))*(tmax-tmin)+tmin
        # will scale m linearly into [tmin,tmax] as desired.
        # To go step by step,
        # m --> m−rmin maps m to [0,rmax−rmin].
        # Next,
        # m --> (m−rmin)/(rmax−rmin)
        # maps m to the interval [0,1], with m=rmin mapped to 0 and m=rmax mapped to 1.
        # Multiplying this by (tmax−tmin) maps m to [0,tmax−tmin].
        # Finally, adding tmin shifts everything and maps m to [tmin,tmax] as desired.
        return (((m - rmin) / (rmax - rmin)) * (tmax - tmin) + tmin)

    def observationRescale(self, observations):
        robs = np.zeros(len(observations), dtype=np.float32)
        # --> Distance from the waypoint (m) [0   , 500];
        robs[0] = 2 * observations[0] / (100 + self.DTOL) - 1
        # --> Trajectory angle               [-180, 180]
        robs[1] = observations[1] / 180.0
        # --> Boat linear velocity (m/s)     [0   , 10 ]
        robs[2] = observations[2] / 5 - 1
        # --> Aparent wind speed (m/s)       [0   , 30]
        robs[3] = observations[3] / 15 - 1
        # --> Apparent wind angle            [-180, 180]
        robs[4] = observations[4] / 180.0
        # --> Boom angle                     [0   , 90]
        robs[5] = (observations[5] / 45.0) - 1
        # --> Rudder angle                   [-60 , 60 ]
        robs[6] = observations[6] / 60.0
        # --> Electric propulsion speed      [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]
        robs[7] = observations[7] / 5.0
        # --> Roll angle                     [-180, 180]
        robs[8] = observations[8] / 180.0

        return robs

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def rewardFunctionV0(self, obs):
        delta_dist  = self.DPREV - obs[0]
        delta_theta = self.THETA0 - abs(obs[1])
        prop        = obs[7]
        roll        = abs(obs[8])
        reward      = 0
        B           = 1 - self.ENERGY * 0.05

        if prop > 0:
            C = 2.0 * (0.2 - 0.04 * (prop - 1.0))
        elif prop < 0:
            C = 2.0 * (0.2 + 0.05 * (prop + 1.0))
        else:
            C = 2.0

        if delta_dist > 0:
            reward += C * B * delta_dist
        else:
            reward += delta_dist

        if delta_theta > 0:
            reward += C * B * 0.1 * delta_theta

        if roll > 30:
            reward -= 0.2 * roll

        return reward

    def rewardFunction(self, obs):
        #--> Reward;Penalty by decresing/increasing the distance from the goal.
        reward = (self.DPREV - obs[0]) / self.D0

        if obs[2] < 0.4:
            #--> Have a velocity slower than 0.4 m/s or negative generate a penalty.
            reward -= 0.2
        elif (abs(obs[1]) < 60):
            #--> Have a velocity greater than 0.4 m/s toward the objective generates a reward.
            reward += 0.01 * (1.0 - abs(obs[1]) / 60.0)

            #--> Save energy increses the reward
            reward *= 1.0 + (1.0 - self.ENERGY/5.8)

            if ((obs[2] > 1.0) & (obs[7] == 0)):
                #--> Reward for boat speedy greater than 1.0 m/s if the electric engine is offline.
                reward += 0.1

        return reward

    def updateEnergy(self, obs):
        self.ENERGY   = 0.01 * abs(self.BOOM0 - obs[5])              #--> Changes in the boom angle consume 0.01% of the battery per degree.
        self.ENERGY  += 0.02 * abs(self.RUDDER0 - obs[6])            #--> Changes in the rudder angle consume 0.02% of the battery per degree.
        self.ENERGY  += 0.5 * abs(obs[7])                            #--> The energy consumption of the electric propulsion is proportional to its rotation speed.
        self.BATTERY -= self.ENERGY                                  #--> Update battery charge
        print(f"ENERGY = 0.01 * {abs(self.BOOM0 - obs[5])} + 0.02 * {abs(self.RUDDER0 - obs[6])} + 0.5 * {abs(obs[7])} = {self.ENERGY}")

    def step(self, action):
        #print("#################### step no CC2 ###################")
        # --> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        #-->SEND ACTION TO THE BOAT CONTROL INTERFACE
        ract = self.actionRescale(action)
        self.propVel_pub.publish(int(ract[0]))
        self.boomAng_pub.publish(ract[1])
        self.rudderAng_pub.publish(ract[2])

        #-->GET OBSERVATIONS
        observations = self.getObservations()

        #-->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        #-->CALCULATE ENERGY CONSUMED DURING THE TIME STEP
        self.updateEnergy(observations)

        #-->CALCULATES THE REWARD
        reward = self.rewardFunction(observations)

        #-->UPDATE PREVIOUS STATE VARIABLES
        self.DPREV   = observations[0]
        self.THETA0  = observations[1]
        self.BOOM0   = observations[5]
        self.RUDDER0 = observations[6]

        #-->CHECK FOR A TERMINAL STATE
        done = bool((self.DPREV <= 5) |
                    (self.DPREV > self.DMAX) |
                    (observations[8] > 50.0) |
                    (np.isnan(observations).any()) |
                    (self.BATTERY <= 0) |
                    (self.STEP_COUNT > 120) #--> with the 5 sec interval between observations, 120 steps = 10 minutes
                    )

        if np.isnan(observations).any():
            print("\n\n-------------------------------------")
            print(f"distance: {observations[0]}")
            print(f"traj ang: {observations[1]}")
            print(f"boat vel: {observations[2]}")
            print(f"wind vel: {observations[3]}")
            print(f"wind ang: {observations[4]}")
            print(f"boom ang: {observations[5]}")
            print(f"rud ang : {observations[6]}")
            print(f"prop    : {observations[7]}")
            print(f"roll ang: {observations[8]}")
            print("-------------------------------------\n")
            # --> WAIT FOR ACKNOWLEDGEMENT FROM USER
            # _ = input("Unpause: ")

        #-->PROCESS DONE SIGNAL
        if done:
            if (self.DPREV < 4):
                reward = 1
            elif (self.BATTERY <= 0):
                reward = -1
            else:
                reward = -1
        else:
            self.STEP_COUNT += 1

        return self.observationRescale(observations), reward, done, {}

    def reset(self):
        #print("#################### reset no CC2 ###################")
        # -->RESETS THE STATE OF THE ENVIRONMENT.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print(("/gazebo/reset_simulation service call failed!"))

        # -->SET RANDOM INITIAL STATE
        # self.setInitialState("eboat", Rot=[0,0,90])
        self.sampleInitialState()

        # -->SET RANDOM WIND SPEED AND DIRECTION
        self.sampleWindSpeed()
        self.wind_pub.publish(Point(self.windSpeed[0], self.windSpeed[1], self.windSpeed[2]))

        # -->SET THE ACTUATORS BACK TO THE DEFAULT SETTING
        self.propVel_pub.publish(0)
        self.boomAng_pub.publish(0.0)
        self.rudderAng_pub.publish(0.0)

        # -->UNPAUSE SIMULATION TO MAKE OBSERVATION
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # -->COLLECT OBSERVATIONS
        observations = self.getObservations()

        #-->RESET INITIAL STATE VALUES
        self.D0         = observations[0]
        self.DMAX       = observations[0] + self.DTOL
        self.DPREV      = observations[0]
        self.THETA0     = observations[1]
        self.BOOM0      = observations[5]
        self.RUDDER0    = observations[6]
        self.BATTERY    = 100
        self.ENERGY     = 0
        self.STEP_COUNT = 0

        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        return self.observationRescale(observations)

