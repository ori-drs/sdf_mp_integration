import rospy
import tf
import math
import numpy as np
import decimal
import time

from xml.dom import minidom
from tf import TransformListener
from copy import deepcopy

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetWorldProperties, SetModelState
from geometry_msgs.msg import *
from std_msgs.msg import String

import tf2_geometry_msgs  # import the packages first

class EnvManager:
    def __init__(self):

        print("Starting the environment manager.")

        # Define all the services needed
        self.delete_service = '/gazebo/delete_model'
        self.get_object_state_service = '/gazebo/get_model_state'
        self.spawn_object_service = '/gazebo/spawn_sdf_model'
        self.move_object_service = '/gazebo/set_model_state'

        rospy.wait_for_service(self.move_object_service)
        print('Initialised EnvManager.')


    def remove_all_objects(self):
        objects = self.get_all_spawned_objects()
        for name in objects:
            if 'block' in name:
                self.delete_object(name)

        self.current_spawn_names = []
        print('All objects removed.')


    def delete_object(self, object_name):
        print('Waiting for service: {0}'.format(self.delete_service))
        rospy.wait_for_service(self.delete_service)

        try:
            print('Calling service: {0}'.format(self.delete_service))
            delete_service = rospy.ServiceProxy(self.delete_service, DeleteModel)
            delete_service(object_name)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def get_object_state(self, object_name):
        rospy.wait_for_service(self.get_object_state_service)
        try:
            get_object_state_service = rospy.ServiceProxy(self.get_object_state_service, String)
            get_object_state_service(object_name)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def spawn_object(self, object_name, pos=[0.0, 0.0, 0.0]):
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
        item_pose = Pose(Point(x=pos[0], y=pos[1], z=pos[2]), orient)

        rospy.wait_for_service(self.spawn_object_service)
        try:
            spawn_object_service = rospy.ServiceProxy(self.spawn_object_service, SpawnModel)
            with open("/home/mark/.gazebo/models/wood_cube_panda/model.sdf", "r") as f:
                product_xml = f.read()

            spawn_object_service(object_name, product_xml, '', item_pose, 'world')

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def get_all_spawned_objects(self):

        rospy.wait_for_service('gazebo/get_world_properties')
        try:
            move_object_service = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)
            props = move_object_service()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        return props.model_names

    def move_object(self, object_name, move_dist=[0, 0, 0], ori= [0,0,0,1]):

        state_msg = ModelState()

        state_msg.model_name = object_name
        state_msg.pose.position.x = move_dist[0]
        state_msg.pose.position.y = move_dist[1]
        state_msg.pose.position.z = move_dist[2]
        state_msg.pose.orientation.x = ori[0]
        state_msg.pose.orientation.y = ori[1]
        state_msg.pose.orientation.z = ori[2]
        state_msg.pose.orientation.w = ori[3]
        state_msg.twist.linear.x = 0
        state_msg.twist.linear.y = 0
        state_msg.twist.linear.z = 0
        state_msg.twist.angular.x = 0
        state_msg.twist.angular.y = 0
        state_msg.twist.angular.z = 0
        state_msg.reference_frame = 'world'

        
        try:
            move_object_service = rospy.ServiceProxy(self.move_object_service, SetModelState)
            resp = move_object_service(state_msg)
            print(resp)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

