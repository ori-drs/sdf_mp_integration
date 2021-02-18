#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from hsrb_interface import Robot, ItemTypes, geometry
from sdf_mp_integration.msg import HeadDirection


class HSRPythonController():
    def __init__(self):
        self.robot = Robot()
        self.whole_body = self.robot.get("whole_body")
        self.gaze_sub = rospy.Subscriber("hsr_gaze_update", HeadDirection, self.look)
        self.move_sub = rospy.Subscriber("hsr_move_to_go", String, self.move_to_go)
        rospy.loginfo("hsr_python_controller ready...")

    def look(self, msg):
        try:
            self.whole_body.gaze_point(geometry.vector3(msg.pt.x, msg.pt.y, msg.pt.z), msg.frame)
        except Exception:
            rospy.loginfo("Error encountered changing the gaze")

    def move_to_go(self, msg):   
        rospy.loginfo("Moving hsr to go position...")
        try:
            self.whole_body.move_to_go()
        except Exception:
            rospy.loginfo("Error encountered in moving to go")


if __name__ == '__main__':
    rospy.init_node('hsr_python_controller', anonymous=True)
    controller = HSRPythonController()
    rospy.spin()