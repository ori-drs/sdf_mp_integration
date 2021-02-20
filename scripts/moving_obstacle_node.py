#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String, Float32

from sdf_mp_integration.env_manager import EnvManager

def main():
    rospy.init_node('moving_obstacle_node', anonymous=True)
    freq = 30
    duration = 10
    rate = rospy.Rate(freq)

    # Instantiate the environment manager
    environment_manager = EnvManager()

    # pick and place params
    start_pos = np.array([0.4,2.0,0])
    ori = [0,0,0,0]

    def callback(data):    
        velocity = [0, -data.data, 0]

        rate.sleep()

        for i in range(duration*freq):
            print('moving')
            fin_pos = start_pos + (i+1)*np.array(velocity)/freq
            rate.sleep()
            environment_manager.move_object('cylinder', fin_pos, ori)
        

        rospy.sleep(1)
        environment_manager.move_object('cylinder', start_pos, ori)

    rospy.sleep(1)
    environment_manager.move_object('cylinder', start_pos, ori)
    
    rospy.Subscriber("start_moving_obstacle", Float32, callback)

    
    rospy.spin()

if __name__ == '__main__':
    main()
