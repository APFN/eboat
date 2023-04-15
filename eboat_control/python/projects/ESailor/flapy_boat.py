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
from geometry_msgs.msg import Vector3
from transforms3d.quaternions import axangle2quat

from stable_baselines3 import PPO

import select


import sys


def main():

    #-->INITIALIZE ROS NODE
    rospy.init_node('ESailor', anonymous=True)

    #-->SUBSCRIBE TO PUBLISH ON ROS TOPICS
    boomAng_pub   = rospy.Publisher("/eboat/control_interface/sail"       , Float32, queue_size=5)
    rudderAng_pub = rospy.Publisher("/eboat/control_interface/rudder"     , Float32, queue_size=5)
    propVel_pub   = rospy.Publisher("/eboat/control_interface/propulsion", Int16  , queue_size=5)
    
    #-->ROS SERVICES
    unpause       = rospy.ServiceProxy('/gazebo/unpause_physics' , Empty)
    pause         = rospy.ServiceProxy('/gazebo/pause_physics'   , Empty)
    reset_proxy   = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    
    spawn_model   = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model  = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    #-->RESET SIMULATION
    rospy.wait_for_service('/gazebo/reset_simulation')

    # -->UNPAUSE SIMULATION
    rospy.wait_for_service('/gazebo/unpause_physics')

    while True:
        propVel_pub.publish(0)
        rudderAng_pub.publish(15.0)

        for i in range(90):
            boomAng_pub.publish(i)
            time.sleep(0.01)
        for i in range(90, -1, -1):
            boomAng_pub.publish(i)
            time.sleep(0.0001)

        if select.select([sys.stdin], [], [], 1)[0]:
            # Lê a entrada do usuário e faz alguma coisa com ela
            user_input = input()
            print("Você digitou:", user_input)
            break
        
        

  

if __name__ == '__main__':
    main()
