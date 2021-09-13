#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String, Int8

from sdf_mp_integration.env_manager import EnvManager
    
class DynamicObstacleManager:
    def __init__(self, ind_waypoints_dict):
        self.freq = 20
        self.rate = rospy.Rate(self.freq)
        self.ori = [0,0,0,0]
        self.task_sub = rospy.Subscriber("obstacle_task", Int8, self.callback)

        # Instantiate the environment manager
        self.environment_manager = EnvManager()
        
        # Precalcuate all of our trajectories, ready for execution
        self.exps = {}
        for key, value in ind_waypoints_dict.items():
            trajectories = []
            for i in range(len(value)):
                trajectories.append(self.calculate_trajectory(value[i]))
            self.exps[key] = trajectories

        rospy.loginfo("DynamicObstacleManager ready.")


    def calculate_trajectory(self, waypoints):
        trajectory = []
        pos = waypoints[0, 0:3]
        
        for i in range(len(waypoints) - 1):
            delta_t = waypoints[i + 1, 3] - waypoints[i, 3]
            num_steps  = int(delta_t * self.freq)
            delta_x = (waypoints[i + 1, 0:3] - waypoints[i, 0:3])/num_steps
            for j in range(num_steps):
                pos = pos + delta_x
                trajectory.append(pos)
                
#         print np.array(trajectory)
        return np.array(trajectory)

    def move_to_init(self, exp_id):
        trajectories = self.exps[exp_id]
        self.environment_manager.move_object('cylinder', trajectories[0][0, :], self.ori)
        if(len(trajectories) == 2):
            self.environment_manager.move_object('cylinder2', trajectories[1][0, :], self.ori)
        if(len(trajectories) == 3):
                self.environment_manager.move_object('cylinder3', trajectories[2][0, :], [ 0.7068252, 0, 0, 0.7073883 ])
                
    def execute_trajectories(self, trajectories):
        num_traj_points = trajectories[0].shape[0]
        for i in range(num_traj_points):
            self.environment_manager.move_object('cylinder', trajectories[0][i, :], self.ori)
            if(len(trajectories) == 2):
                self.environment_manager.move_object('cylinder2', trajectories[1][i, :], self.ori)
            if(len(trajectories) == 3):
                self.environment_manager.move_object('cylinder3', trajectories[2][i, :], [ 0.7068252, 0, 0, 0.7073883 ])
            self.rate.sleep()

    def execute_trajectory_id(self, exp_id):
        self.execute_trajectories(self.exps[exp_id])
        
    def callback(self, msg):    
        rospy.loginfo("Dynamic obstaccle command received")
        # pick and place params
        self.move_to_init(msg.data)
        rospy.sleep(0.2)
        self.execute_trajectory_id(msg.data)
        # self.reset(msg.data)




if __name__ == '__main__':
    rospy.init_node('nbv_moving_obstacle_node', anonymous=True)

    ind_waypoints_dict = {0: [np.array([ [100, 20.0, 0.0, 0.0], 
                                    [100, 20.0, 0.0, 1.0]], dtype=float), 
                            np.array([ [100, 21.0, 0.0, 0.0],
                                    [100, 21.0, 0.0, 1.0]], dtype=float),
                            np.array([ [100, 22.0, 0.0, 0.0],
                                    [100, 22.0, 0.0, 1.0]], dtype=float)],

                        1: [np.array([[-1.0, -1.0, 0.0, 0.0], 
                                        [1.0, -1.0, 0.0, 27.0], 
                                        [3.0, -1.0, 0.0, 47.0]], dtype=float)], # This is the original dynamic experiment
                        
                        2: [np.array([[-1.0, -1.0, 0.0, 0.0], 
                                        [0.0, -1.0, 0.0, 15.0], 
                                        [3.0, 2.0, 0.0, 55.0]], dtype=float)],                    
                        }


    obstacle_manager = DynamicObstacleManager(ind_waypoints_dict)
    rospy.spin()