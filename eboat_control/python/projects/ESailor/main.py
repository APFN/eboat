import numpy as np
import rospy
import time

from std_msgs.msg import Float32, Int16, Float32MultiArray
from geometry_msgs.msg import Point
from std_srvs.srv import Empty

from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

from plot_coordinates import PlotCoordinates

import threading

from stable_baselines3 import PPO


def main():

    #-->INITIALIZE ROS NODE
    rospy.init_node('ESailor', anonymous=True)

    #-->SUBSCRIBE TO PUBLISH ON ROS TOPICS
    boomAng_pub   = rospy.Publisher("/eboat/control_interface/sail"       , Float32, queue_size=5)
    rudderAng_pub = rospy.Publisher("/eboat/control_interface/rudder"     , Float32, queue_size=5)
    propVel_pub   = rospy.Publisher("/eboat/control_interface/propulsion", Int16  , queue_size=5)
    wind_pub      = rospy.Publisher("/eboat/atmosferic_control/wind"      , Point  , queue_size=5)
    
    #-->ROS SERVICES
    unpause       = rospy.ServiceProxy('/gazebo/unpause_physics' , Empty)
    pause         = rospy.ServiceProxy('/gazebo/pause_physics'   , Empty)
    reset_proxy   = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    set_state     = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    spawn_model   = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model  = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)



    #-->LOAD AGENT USING STABLE-BASELINES3
    model = PPO.load(f"/home/alvaro/eboat_ws/src/eboat_gz_1/models/PPO/modelo-edu/eboat_ocean_100.zip")
    
    #-->DEFINE NAVIGATION PATH
    # navpath = [[0.0     , 100.0, 0.5],
    #            [83.5165 , 155.0, 0.5],
    #            [181.4961, 175.0, 0.5],
    #            [281.4961, 175.0, 0.5],
    #            [354.8173, 243.0, 0.5]]
    
    #  #-->DEFINE NAVIGATION PATH
    # navpath = [[0, 0, 0.5],
    #            [0, 100, 0.5],
    #            [0, 0, 0.5],
    #            [50, 50, 0.5],
    #            [0, 100, 0.5],
    #            [0, 0, 0.5]]


    # #-->DEFINE NAVIGATION PATH
    # navpath = [[-100, 0, 0.5],
    #            [0, 100, 0.5],
    #            [100, 0, 0.5],
    #            [0, -100, 0.5],
    #            [75, 75, 0.5],
    #            [75, -75, 0.5],
    #            [-75, -75, 0.5],
    #            [-75, 75, 0.5]]

    # #->DEFINE NAVIGATION PATH
    # navpath = [[0.0, 100.0, 0.5],
    #            [-100, 0, 0.5],
    #            [100, 0, 0.5],
    #            [-100, 0, 0.5],
    #            [0, 100, 0.5],
    #            [100, 0, 0.5],
    #            [0, -100, 0.5],
    #            [-100, 0, 0.5]]

    navpath = [[0.0, 100.0, 0.5]]

    #-->RESET SIMULATION
    rospy.wait_for_service('/gazebo/reset_simulation')
    try:
        reset_proxy()
    except (rospy.ServiceException) as e:
        print(("/gazebo/reset_simulation service call failed!"))
    propVel_pub.publish(0)
    boomAng_pub.publish(0.0)
    rudderAng_pub.publish(0.0)

    pc = PlotCoordinates() # Cria uma instância da classe PlotCoordinates


    for waypoint in navpath:
        #########################################################################
        try:
            result = delete_model("wayPointMarker")
        except rospy.ServiceException:
            print("/gazebo/get_model_state service call failed")
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
        observations = getObservations()[:5]

        print("--------------------------------------------------")
        while observations[0] > 10: #waipoint alcançado há 10m de distancia do barco
            pc.save_eboat_coordinates() # Chama o método save_coordinates da instância pc

            obs = observationRescale(observations)

            #-->PREDICT ACTIONS
            actions = actionRescale(model.predict(obs)[0])
            actions[0] = np.floor(actions[0])
            # imprimi opbservações e achoes do modelo
            print(f"{vet2str(observations)} --> {vet2str(actions)}")

            #-->SEND ACTIONS TO THE CONTROL INTERFACE
            #propVel_pub.publish(int(actions[0])) #com motor
            # propVel_pub.publish(int(0)) #sem motor
            boomAng_pub.publish(actions[0])
            rudderAng_pub.publish(actions[1])

            # -->COLLECT OBSERVATIONS
            observations = getObservations()[:5] 
        
        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            pause()
        except( rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))
        pc.save_waypoint_coordinates() #salva o x e y do waypoint, precisa ser no final do loop para salvaro tempo




    propVel_pub.publish(0)
    boomAng_pub.publish(0.0)
    rudderAng_pub.publish(0.0)
    pc.plot_coordinates() # Chama o método save_coordinates da instância pc


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

def actionRescale(action):
    raction = np.zeros(2, dtype = np.float32)
    # #--> Eletric propulsion [-5, 5]
    # raction[0] = action[0] * 5.0
    #--> Boom angle [0, 90]
    raction[0] = (action[0] + 1) * 45.0
    #--> Rudder angle [-60, 60]
    raction[1] = action[1] * 60.0
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
