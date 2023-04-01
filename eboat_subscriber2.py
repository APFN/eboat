#!/home/eduardo/miniconda3/envs/gazgym/bin/python
  
import rospy
from std_msgs.msg import Float32, Float32MultiArray

if __name__ == '__main__':
      
    rospy.init_node('test_node', anonymous=True)
    dist = rospy.wait_for_message('/eboat/mission_control/distance', Float32, timeout=5).data
    print(dist)
    obs = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray, timeout=5).data
    print(obs)
    print(obs[0])