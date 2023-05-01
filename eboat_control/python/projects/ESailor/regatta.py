import numpy as np
import rospy
import time

from std_msgs.msg import Float32, Int16, Float32MultiArray, Bool
from geometry_msgs.msg import Point
from std_srvs.srv import Empty

from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

from plot_coordinates import PlotCoordinates
from geometry_msgs.msg import Vector3
from transforms3d.quaternions import axangle2quat

from stable_baselines3 import PPO

import csv


set_state     = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

def wind_callback(data):
    wind_x, wind_y = map(float, data.data[:2])  
    model_state = ModelState()
    model_state.model_name = "wind_arrow" 
    model_state.pose.position.x = 0
    model_state.pose.position.y = 0
    model_state.pose.position.z = 20

    theta = np.arctan2(wind_x, wind_y)
    axis = np.array([1, 0, 0])
    # criação do quaternion
    wind_quaternion = axangle2quat(axis, -theta)
    model_state.pose.orientation.x = wind_quaternion[0]
    model_state.pose.orientation.y = wind_quaternion[1]
    model_state.pose.orientation.z = wind_quaternion[2]
    model_state.pose.orientation.w = wind_quaternion[3]

    #atulaiza o estado do vetor do vento
    set_state(model_state)

def main():

    global model_flappyBoat, model_onlyMotor, model_sail_freeMotor, model_sail_mediumMotor, model_onlySail, flappy_boat_pub

    #-->INITIALIZE ROS NODE
    rospy.init_node('ESailor', anonymous=True)

    #-->SUBSCRIBE TO PUBLISH ON ROS TOPICS
    boomAng_pub     = rospy.Publisher("/eboat/control_interface/sail"       , Float32, queue_size=5)
    rudderAng_pub   = rospy.Publisher("/eboat/control_interface/rudder"     , Float32, queue_size=5)
    propVel_pub     = rospy.Publisher("/eboat/control_interface/propulsion" , Int16  , queue_size=5)
    wind_pub        = rospy.Publisher("/eboat/atmosferic_control/wind"      , Point  , queue_size=5)
    flappy_boat_pub = rospy.Publisher('/eboat/control_interface/flappy_boat', Bool   , queue_size=10)

    
    #-->ROS SERVICES
    unpause       = rospy.ServiceProxy('/gazebo/unpause_physics' , Empty)
    pause         = rospy.ServiceProxy('/gazebo/pause_physics'   , Empty)
    reset_proxy   = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    
    spawn_model   = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model  = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    rospy.Subscriber('/eboat/atmosferic_control/wind', Float32MultiArray, wind_callback)  



    #-->LOAD AGENT USING STABLE-BASELINES3
    
    model_flappyBoat = PPO.load(f"/home/alvaro/eboat_ws/src/eboat_gz_1/models/PPO/0_flappyBoat_23042023_18_35_47/eboat_ocean_7.zip")
    model_onlyMotor = PPO.load(f"/home/alvaro/eboat_ws/src/eboat_gz_1/models/PPO/0_onlyMotor_27042023_13_44_04/eboat_ocean_8.zip")
    model_onlySail = PPO.load(f"/home/alvaro/eboat_ws/src/eboat_gz_1/models/PPO/0_onlySail_26042023_05_37_21/eboat_ocean_9.zip")
    model_sail_freeMotor = PPO.load(f"/home/alvaro/eboat_ws/src/eboat_gz_1/models/PPO/0_sail_freeMotor_25042023_20_22_07/eboat_ocean_9.zip")
    model_sail_mediumMotor = PPO.load(f"/home/alvaro/eboat_ws/src/eboat_gz_1/models/PPO/0_sail_mediumMotor_25042023_13_05_59/eboat_ocean_9.zip")
    
    navpath = []
    wind = []
    battery = []
    ecoMode = []

    with open('/home/alvaro/eboat_ws/src/eboat_gz_1/eboat_control/python/projects/ESailor/missionPlanner.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        header = next(csv_reader) # discard the header    
        for row in csv_reader:
            # salvar os valores das colunas nas variáveis correspondentes
            navpath.append([float(row[0]), float(row[1]), float(row[2])])
            wind.append([float(row[3]), float(row[4])])
            battery.append(int(row[5]))
            ecoMode.append(bool(int(row[6])))

    for i in range(len(navpath)):
        print('navpath[{}]:'.format(i), navpath[i], 'wind[{}]:'.format(i), wind[i],'battery[{}]:'.format(i), battery[i], 'ecoMode[{}]:'.format(i), ecoMode[i] )


    #########################################################################
    time.sleep(0.1)
    wpose = Pose()
    wpose.position.x = 0
    wpose.position.y = 0
    wpose.position.z = 20
    
    with open("/home/alvaro/eboat_ws/src/eboat_gz_1/eboat_description/models/wind_arrow/model.sdf") as f:
        sdffile2 = f.read()
        try:
            result = spawn_model("wind_arrow",
                                    sdffile2,
                                    "wind_arrow",
                                    wpose, "world")
        except rospy.ServiceException:
            print("/gazebo/SpawnModel service call failed")
        time.sleep(0.1)

    #########################################################################

    #-->RESET SIMULATION
    rospy.wait_for_service('/gazebo/reset_simulation')
    try:
        reset_proxy()
    except (rospy.ServiceException) as e:
        print(("/gazebo/reset_simulation service call failed!"))
    propVel_pub.publish(0)
    boomAng_pub.publish(90.0)
    rudderAng_pub.publish(0.0)

    pc = PlotCoordinates() # Cria uma instância da classe PlotCoordinates para plotar ao final


    for i in range(len(navpath)):

        
        print('navpath[{}]:'.format(i), navpath[i], 'wind[{}]:'.format(i), wind[i],'battery[{}]:'.format(i), battery[i], 'ecoMode[{}]:'.format(i), ecoMode[i] )

        windConfig = np.array([0, 0, 0.0], dtype=np.float32)
        windConfig[:3] = [wind[i][0], wind[i][1], 0]
        wind_pub.publish(Point(windConfig[0],windConfig[1],windConfig[2]))

        

        waypoint = navpath[i]
        delete_model("wayPointMarker")
        time.sleep(1)
        ipose = Pose()
        ipose.position.x = waypoint[0]
        ipose.position.y = waypoint[1]
        ipose.position.z = waypoint[2]
        with open(
                "/home/alvaro/eboat_ws/src/eboat_gz_1/eboat_description/models/wayPointMarker/model.sdf") as f:
            sdffile = f.read()
            try:
                result = spawn_model("wayPointMarker",
                                     sdffile,
                                     "wayPointMarker",
                                     ipose, "world")
            except rospy.ServiceException:
                print("/gazebo/SpawnModel service call failed")
            time.sleep(1)

        #########################################################################


        # -->UNPAUSE SIMULATION
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        #-->COLLECT OBSERVATIONS
        observations = getObservations()[:9]

        

        print("--------------------------------------------------")
        while observations[0] > 15: #waipoint alcançado há 10m de distancia do barco

            pc.save_eboat_coordinates() # Chama o método save_coordinates da instância pc

            obs = observationRescale(observations)
            anemometer = observations[3]
            actions = sailor(obs, anemometer, battery[i], ecoMode[i])

            #-->SEND ACTIONS TO THE CONTROL INTERFACE
            propVel_pub.publish(int(actions[0]))
            boomAng_pub.publish(actions[1])
            rudderAng_pub.publish(actions[2])

            # imprimi opbservações e achoes do modelo
            print(f"{vet2str(observations)} --> {vet2str(actions)}")

            # -->COLLECT OBSERVATIONS
            observations = getObservations()[:9] 
            
        
        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            pause()
        except( rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))
        pc.save_waypoint_coordinates() #salva o x e y do waypoint, precisa ser no final do loop para salvaro tempo




    propVel_pub.publish(0)
    boomAng_pub.publish(90.0)
    rudderAng_pub.publish(0.0)
    
    pc.plot_coordinates() # Chama o método save_coordinates da instância pc

def sailor(observations, wind, battery, ecoMode):
    # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
    #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]     

    print("Battery[{}] || Wind[{}] || ecoMode[{}]".format(battery, wind, ecoMode))  

    if(battery <= 10):
        print("battery <= 10 -> critical battery  = wait battery charger")
        actions = [0]* 3

    elif( 10 < battery <= 30 and wind <= 0.3 ):
        print("10 < battery <= 30 & wind == 0  -> low battery, no wind = model_flappyBoat")
        predict = model_flappyBoat.predict(observations)
        actions = actionRescale_flappy(predict[0])
        flappy_boat_pub.publish(True) #flap the sail

    elif( 10 < battery <= 30 and 0.3 < wind and ecoMode == True):
        print("10<battery<=30 & 0.3 < wind -> low battery, wind low or ok, ecomode:True =  model_onlySail")
        predict =  model_onlySail.predict(observations)
        actions = actionRescale_onlySail(predict[0])
   
    elif( 10 < battery <= 30 and 0.3 < wind  and ecoMode == False):
        print("10<battery<=30 & 3 <wind & ecoMode==True -> low battery, wind ok, ecomode:False = model_sail_mediumMotor")
        predict =  model_sail_mediumMotor.predict(observations)
        actions = actionRescale_SailAndMotor(predict[0])

    elif( 30 < battery and  wind <= 0.3  and ecoMode == True):
        print("30<battery & wind <= 0.3 & ecoMode==True -> battery ok, no wind, ecomode:True = model_flappyBoat")
        predict = model_flappyBoat.predict(observations)
        actions = actionRescale_flappy(predict[0])
        flappy_boat_pub.publish(True) #flap the sail

    elif( 30 < battery and  wind <= 3  and ecoMode == False):
        print("30<battery & wind<=3 & ecoMode==False -> battery ok, no wind or low , ecomode:False = model_onlyMotor")
        predict = model_onlyMotor.predict(observations)
        actions = actionRescale_onlyMotor(predict[0])

    elif( 30 < battery and  0.3 < wind <= 3  and ecoMode == True):
        print("30<battery & 0.3 <wind<=3 & ecoMode==True -> battery ok, low wind, ecomode:True = model_sail_freeMotor")
        predict = model_sail_freeMotor.predict(observations)
        actions = actionRescale_SailAndMotor(predict[0])

    elif( 30 < battery and 3 < wind):
        print("30<battery & 3 <wind -> battery ok, wind ok = model_onlySail")
        predict = model_onlySail.predict(observations)
        actions = actionRescale_onlySail(predict[0])

    else:
        print("No behavior detected")
        actions = [0]* 3


    

    return actions

def spawn_waypoint(waypoint):
    vetstr = "["
    for val in vet:
        vetstr += "{:5.3f}, ".format(val)
    vetstr += "]"
    vetstr.replace(",]", "]")
   
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

def getObservations():
    count = 0
    obsData = None
    while obsData is None:
        try:
            obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray, timeout=20).data
        except:
            pass
        count += 1
        if count > 1000:
            break
        # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
        #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]

    return np.array(obsData, dtype=float)

def observationRescale(observations):
    lobs = len(observations)
    robs = np.zeros(lobs, dtype=np.float32)
    # --> Distance from the waypoint (m) [0   , DMAX];
    robs[0] = observations[0]/100 - 1
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

def actionRescale_onlySail(action): #only have 2 actions: sail and rudder
    raction = np.zeros(3, dtype = np.float32)
    #--> Eletric propulsion [-5, 5]
    raction[0] = 0
    #--> Boom angle [0, 90]
    raction[1] = (action[0] + 1) * 45.0
    #--> Rudder angle [-60, 60]
    raction[2] = action[1] * 60.0   

    return raction

def actionRescale_onlyMotor(action):  #only have 2 actions: motor and rudder
    raction = np.zeros(3, dtype = np.float32)
    # #--> Eletric propulsion [-5, 5]
    raction[0] = int(np.floor(action[0] * 5.0))
    #--> Boom angle [0, 90]
    raction[1] = 90
    #--> Rudder angle [-60, 60]
    raction[2] = action[1] * 60.0  

    return raction

def actionRescale_flappy(action): #only have 1 actions: rudder
    raction = np.zeros(3, dtype = np.float32)
    #--> Eletric propulsion [-5, 5]
    raction[0] = 0
    #--> Boom angle [0, 90]
    raction[1] = 90
    #--> Rudder angle [-60, 60]
    raction[2] = action[0] * 60.0  

    return raction

def actionRescale_SailAndMotor(action): # have 3 actions: motor, sail and rudder
    raction = np.zeros(3, dtype = np.float32)
    # #--> Eletric propulsion [-5, 5]
    raction[0] = int(np.floor(action[0] * 5.0))
    #--> Boom angle [0, 90]
    raction[1] = (action[1] + 1) * 45.0
    #--> Rudder angle [-60, 60]
    raction[2] = action[2] * 60.0

    return raction

def vet2str(vet):
    vetstr = "["
    for val in vet:
        vetstr += "{:5.3f}, ".format(val)
    vetstr += "]"
    vetstr.replace(",]", "]")
    return vetstr

if __name__ == '__main__':
    main()
