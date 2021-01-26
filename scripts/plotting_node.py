import rospy
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import String
from sdf_mp_integration.msg import GtsamValues
from sdf_mp_integration.msg import GtsamValue
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure

class listener():
    def __init__(self):
        rospy.init_node('plotting_node', anonymous=True)

        self.sub = rospy.Subscriber("gpmp2_results", GtsamValues, self.callback)
        self.img_pub = rospy.Publisher("gpmp2_plots", Image, queue_size=1)
        self.bridge = CvBridge()
        self.msg = None

    # spin() simply keeps python from exiting until this node is stopped

    def callback(self, msg):
        self.msg = msg
        num_points = len(msg.values)
        x = np.zeros(num_points)
        y = np.zeros(num_points)
        theta = np.zeros(num_points)
        for i in msg.values:
            seq = i.seq
            x[seq] = i.x[0]
            y[seq] = i.x[1]
            theta[seq] = (i.x[2] + 3.141)/(2 * 3.141)
        
        fig = Figure()
        fig.set_dpi(300.0)
        canvas = FigureCanvas(fig)
        ax = fig.gca()
        ax.plot(range(num_points),(x-min(x))/(max(x)-min(x)), 'r', label='x')
        ax.plot(range(num_points),(y-min(y))/(max(y)-min(y)), 'b', label='y')
        ax.plot(range(num_points), theta, 'g', label='yaw')
        ax.axes.set_xlabel('Timestep')
        ax.axes.set_ylabel('Position')
        ax.legend()

        width, height = fig.get_size_inches() * fig.get_dpi() 
        canvas.draw()       # draw the canvas, cache the renderer
        img = np.frombuffer(canvas.tostring_rgb(), dtype='uint8').reshape(int(height), int(width), 3)

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
    
if __name__ == '__main__':
    listener = listener()
    rospy.spin()
