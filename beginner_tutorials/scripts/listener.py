import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String



def CallBack(data):
    # pass
    rospy.loginfo(rospy.get_caller_id() + ': I heard %s', check(data.ranges))
    # cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    # print(cv_image)
def check(msg):
    return np.amin(msg)

def listener():

    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber('gazebo/model_states', ModelStates, self.ModelStateCallBack)
    # rospy.Subscriber('camera/rgb/image_raw', Image, self.RGBImageCallBack)
    # rospy.Subscriber('camera/depth/image_raw', Image, self.DepthImageCallBack)
    rospy.Subscriber('scan', LaserScan, CallBack)
    # rospy.Subscriber('odom', Odometry, self.OdometryCallBack)
    # rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.BumperCallBack)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':

    listener()
