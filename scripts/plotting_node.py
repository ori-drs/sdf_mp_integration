#! /usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np

from std_msgs.msg import String
from sdf_mp_integration.msg import GtsamValues
from sdf_mp_integration.msg import GtsamValue
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryActionGoal

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge, CvBridgeError
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure

class listener():
    def __init__(self):
        rospy.init_node('plotting_node', anonymous=True)

        self.sub = rospy.Subscriber("gpmp2_results", GtsamValues, self.callback)
        self.img_pos_pub = rospy.Publisher("gpmp2_plots", Image, queue_size=1)
        self.img_vel_pub = rospy.Publisher("gpmp2_vel_plots", Image, queue_size=1)
        self.bridge = CvBridge()
        self.msg = None
        self.delta_t = None
        self.actual_base_sub = None
        # self.goal_sub = rospy.Subscriber("/hsrb/omni_base_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, self.goalCallback)
        
        
        self.base_time_buffer = []
        self.base_x_buffer = []
        self.base_y_buffer = []
        self.base_t_buffer = []
        
        self.base_vx_buffer = []
        self.base_vy_buffer = []
        self.base_vt_buffer = []
        self.record_time = 10

    def plotPositions(self):
        fig = Figure()
        fig.set_dpi(300.0)
        canvas = FigureCanvas(fig)
        ax = fig.gca()
        
        # Plot the desired
        num_points = len(self.msg.values)
        x = np.zeros(num_points)
        y = np.zeros(num_points)
        theta = np.zeros(num_points)
        vx = np.zeros(num_points)
        vy = np.zeros(num_points)
        vtheta = np.zeros(num_points)            
        
        for i in self.msg.values:
            seq = i.seq
            x[seq] = i.x[0]
            y[seq] = i.x[1]
            theta[seq] = i.x[2]/(2 * 3.141)

            vx[seq] = i.v[0]
            vy[seq] = i.v[1]
            vtheta[seq] = i.v[2]/(2 * 3.141)

        time = np.array(range(num_points))*self.delta_t

        # ax.plot(time,(x-min(x))/(max(x)-min(x)), 'r--', label='desired x')
        # ax.plot(time,(y-min(y))/(max(y)-min(y)), 'b--', label='desired y')
        ax.plot(time, x, 'r--', label='desired x')
        ax.plot(time, y, 'b--', label='desired y')
        ax.plot(time, theta, 'g--', label='desired yaw')
        
        # # Plot the actual
        # num_points = len(self.base_time_buffer)
        # x = np.zeros(num_points)
        # y = np.zeros(num_points)
        # theta = np.zeros(num_points)
        # time = np.zeros(num_points)
        # for i in range(num_points):
        #     x[i] = self.base_x_buffer[i]
        #     y[i] = self.base_y_buffer[i]
        #     theta[i] = self.base_t_buffer[i]/(2 * 3.141)
        #     # theta[i] = (self.base_t_buffer[i] + 3.141)/(2 * 3.141)
        #     time[i] = self.base_time_buffer[i].to_sec()
        

        # # ax.plot(time - self.msg.header.stamp.to_sec(),(x-min(x))/(max(x)-min(x)), 'r', label='actual x')
        # # ax.plot(time - self.msg.header.stamp.to_sec(),(y-min(y))/(max(y)-min(y)), 'b', label='actual y')            
        # ax.plot(time - self.msg.header.stamp.to_sec(), x, 'r', label='actual x')
        # ax.plot(time - self.msg.header.stamp.to_sec(), y, 'b', label='actual y')
        # # ax.plot(time - self.msg.header.stamp.to_sec(), theta, 'g', label='actual yaw')
        # ax.plot(time - self.msg.header.stamp.to_sec(), theta, 'g', label='actual yaw')


        # Finish up the graph
        ax.axes.set_xlabel('Timestep')
        ax.axes.set_ylabel('Position')
        ax.legend(ncol=3, bbox_to_anchor=(0.9, 1.15))
        width, height = fig.get_size_inches() * fig.get_dpi() 
        canvas.draw()       # draw the canvas, cache the renderer
        img = np.frombuffer(canvas.tostring_rgb(), dtype='uint8').reshape(int(height), int(width), 3)

        self.img_pos_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))         
    
    def plotVelocities(self):
            fig = Figure()
            fig.set_dpi(300.0)
            canvas = FigureCanvas(fig)
            ax = fig.gca()
            # delta_t = 0.5
            
            # Plot the desired
            num_points = len(self.msg.values)
            vx = np.zeros(num_points)
            vy = np.zeros(num_points)
            vtheta = np.zeros(num_points)            
            
            for i in self.msg.values:
                seq = i.seq
                vx[seq] = i.v[0]
                vy[seq] = i.v[1]
                vtheta[seq] = i.v[2]/(2 * 3.141)

            time = np.array(range(num_points))*self.delta_t

            # ax.plot(time,(x-min(x))/(max(x)-min(x)), 'r--', label='desired x')
            # ax.plot(time,(y-min(y))/(max(y)-min(y)), 'b--', label='desired y')
            ax.plot(time, vx, 'r--', label='desired vx')
            ax.plot(time, vy, 'b--', label='desired vy')
            ax.plot(time, vtheta, 'g--', label='desired vyaw')
            
            # Plot the actual
            num_points = len(self.base_time_buffer)
            vx = np.zeros(num_points)
            vy = np.zeros(num_points)
            vtheta = np.zeros(num_points)
            time = np.zeros(num_points)
            for i in range(num_points):
                vx[i] = self.base_vx_buffer[i]
                vy[i] = self.base_vy_buffer[i]
                vtheta[i] = self.base_vt_buffer[i]/(2 * 3.141)
                # theta[i] = (self.base_t_buffer[i] + 3.141)/(2 * 3.141)
                time[i] = self.base_time_buffer[i].to_sec()
            

            # ax.plot(time - self.msg.header.stamp.to_sec(),(x-min(x))/(max(x)-min(x)), 'r', label='actual x')
            # ax.plot(time - self.msg.header.stamp.to_sec(),(y-min(y))/(max(y)-min(y)), 'b', label='actual y')            
            ax.plot(time - self.msg.header.stamp.to_sec(), vx, 'r', label='actual x')
            ax.plot(time - self.msg.header.stamp.to_sec(), vy, 'b', label='actual y')
            ax.plot(time - self.msg.header.stamp.to_sec(), vtheta, 'g', label='actual yaw')


            # Finish up the graph
            ax.axes.set_xlabel('Timestep')
            ax.axes.set_ylabel('Velocity')
            ax.legend(ncol=3, bbox_to_anchor=(0.9, 1.15))
            width, height = fig.get_size_inches() * fig.get_dpi() 
            canvas.draw()       # draw the canvas, cache the renderer
            img = np.frombuffer(canvas.tostring_rgb(), dtype='uint8').reshape(int(height), int(width), 3)

            self.img_vel_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))         

    # spin() simply keeps python from exiting until this node is stopped
    def goalCallback(self, msg):
        # Start the subscriber
        print("Recording the trajectory")
        self.actual_base_sub = rospy.Subscriber("/hsrb/omni_base_controller/state", JointTrajectoryControllerState, self.recordActualBase)
        start = rospy.Time.now()
        end = rospy.Time.now()
        elapsed = (end-start).to_sec()
        while elapsed < self.record_time:
            end = rospy.Time.now()
            elapsed = (end-start).to_sec()
        
        self.actual_base_sub.unregister()
        print("Final buffer length is {0}".format(len(self.base_time_buffer)))

        # We have both the desired and the actual
        if self.msg:
            self.delta_t = self.msg.delta_t

            # Plot the positions
            self.plotPositions()
            self.plotVelocities()

        # Clear the buffers
        self.base_time_buffer = []
        self.base_x_buffer = []
        self.base_y_buffer = []
        self.base_t_buffer = []
        self.base_vx_buffer = []
        self.base_vy_buffer = []
        self.base_vt_buffer = []
        self.msg = None

    def recordActualBase(self, msg):
        self.base_time_buffer.append(msg.header.stamp)
        self.base_x_buffer.append(msg.actual.positions[0])
        self.base_y_buffer.append(msg.actual.positions[1])
        self.base_t_buffer.append(msg.actual.positions[2])

        self.base_vx_buffer.append(msg.actual.velocities[0])
        self.base_vy_buffer.append(msg.actual.velocities[1])
        self.base_vt_buffer.append(msg.actual.velocities[2])

    def callback(self, msg):
        self.msg = msg
        self.delta_t = 0.5
        self.plotPositions()
        self.plotVelocities()

if __name__ == '__main__':
    listener = listener()
    rospy.spin()
