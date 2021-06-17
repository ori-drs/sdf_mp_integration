#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from hsrb_interface import Robot, ItemTypes, geometry
from sdf_mp_integration.msg import HeadDirection

if __name__ == '__main__':
    rospy.init_node('hsr_python_controller', anonymous=True)
    robot = Robot()
    whole_body = robot.get("whole_body")
    omni_base = robot.get("omni_base")
    gripper = robot.get("gripper")
    rospy.sleep(2.0)
    gripper.set_distance(0.12)
    whole_body.move_end_effector_pose(geometry.pose(z=0.03), whole_body.end_effector_frame)
    gripper.apply_force(0.5)
    rospy.sleep(1.0)
    whole_body.move_to_joint_positions({"arm_lift_joint": 0.1})
    whole_body.move_to_go()
    