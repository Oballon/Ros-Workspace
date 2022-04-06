import numpy as np
# from lfd.rapprentice import transformations
try:
    import geometry_msgs.msg as gm
    import rospy
except ImportError:
    print ("couldn't import ros stuff")

def point_stamed_to_pose_stamped(pts=(3,4,0),orientation=(0,0,0,1)):
    """convert pointstamped to posestamped"""
    
    ps = gm.PoseStamped()
    ps.pose.position = pts
    ps.pose.orientation = orientation
    ps.header.frame_id = "map"
    return ps

if __name__ == '__main__':
    rospy.init_node('pub', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal', gm.PointStamped, queue_size=10)
    rate = rospy.Rate(0.1)
    pub.publish(point_stamed_to_pose_stamped())
