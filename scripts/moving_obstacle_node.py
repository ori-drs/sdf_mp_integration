#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String, Float32

from sdf_mp_integration.env_manager import EnvManager


ind_waypoints_dict = {1: np.array([  [1.0, 0.0, 0.0, 0.0], 
                                    [1.0, 1.0, 0.0, 3.0], 
                                    [2.0, 1.0, 0.0, 7.0]], dtype=float),
                     2: np.array([  [3.5, 1.0, 0.0, 0.0], 
                                    [0.0, -1.0, 0.0, 15.0]], dtype=float),
                     3: np.array([  [4.5, 0.0, 0.0, 0.0], 
                                    [1.0, 0.0, 0.0, 10.0]], dtype=float)}

    
class DynamicObstacleManager:
    def __init__(self, ind_waypoints_dict):
        self.freq = 30
        self.duration = 10
        self.rate = rospy.Rate(self.freq)
        self.ori = [0,0,0,0]

        # Instantiate the environment manager
        self.environment_manager = EnvManager()
        
        # Precalcuate all of our trajectories, ready for execution
        self.exps = {}
        for key, value in ind_waypoints_dict.items():
            self.exps[key] = self.calculate_trajectory(value)
        rospy.Subscriber("start_moving_obstacle", Float32, self.callback)
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

        return np.array(trajectory)

    def move_to_init(self, exp_id):
        traj = self.exps[exp_id]
        self.environment_manager.move_object('cylinder', traj[0, :], self.ori)
        
    def execute_trajectory(self, traj):
        num_traj_points = traj.shape[0]
        for i in range(num_traj_points):
            self.environment_manager.move_object('cylinder', traj[i, :], self.ori)
            self.rate.sleep()

    def execute_trajectory_id(self, exp_id):
        self.execute_trajectory(self.exps[exp_id])
        
    def callback(self, data):    
        # pick and place params
        start_pos = np.array([0.4,2.0,0])
        ori = [0,0,0,0]
        velocity = [0, -data.data, 0]

        self.rate.sleep()

        for i in range(duration*self.freq):
            print('moving')
            fin_pos = start_pos + (i+1)*np.array(velocity)/self.freq
            self.rate.sleep()
            self.environment_manager.move_object('cylinder', fin_pos, ori)
        
        rospy.sleep(1)
        self.environment_manager.move_object('cylinder', start_pos, ori)


if __name__ == '__main__':
    rospy.init_node('moving_obstacle_node', anonymous=True)
    obstacle_manager = DynamicObstacleManager(ind_waypoints_dict)
    # obstacle_manager.move_to_init(3)
    # obstacle_manager.execute_trajectory_id(3)
    rospy.spin()