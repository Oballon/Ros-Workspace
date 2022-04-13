import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from respawnGoal import Respawn
from setGoal import Respawn


class Env:
    def __init__(self, action_size):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0  # Orientation
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()
        self.last_goal_distance = 0

    def getGoalDistance(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)

        return goal_distance

    # get Odometry and orientation
    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)

    # getState used by Ros message
    def getState(self, scan):
        scan_range = []
        heading = self.heading
        min_range = 0.13
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(round(scan.ranges[i], 2))

        if min_range > min(scan_range) > 0:
            done = True

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        if current_distance < 0.4:
            self.get_goalbox = True

        return scan_range + [heading, current_distance], done

    def setReward(self, state, done, action):
        # yaw_reward = []
        # current_distance = state[-1]
        # heading = state[-2]

        # for i in range(5):
        #     angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
        #     tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
        #     yaw_reward.append(tr)

        # distance_rate = 2 ** (current_distance / self.goal_distance)
        # reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate)
        reward = 0
        heading = state[-2]
        current_distance = state[-1]

        scans = state.copy()
        scans.remove(heading)
        scans.remove(current_distance)

        min_scans = min(scans)

        goal_distance = self.getGoalDistance()

        if done:
            rospy.loginfo("Collision!!")
            reward -= 50
            self.pub_cmd_vel.publish(Twist())
        elif min_scans < 0.3:
            reward -= 10

        if current_distance < 1:
            reward += 20

        if self.last_goal_distance < goal_distance:
            reward -= 1
        else:
            reward += 1

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward += 50
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)

        return reward

    def step(self, action):
        max_angular_vel = 1
        ang_vel = ((self.action_size - 1) / 2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.3
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except Exception as e:
                raise e

        state, done = self.getState(data)
        reward = self.setReward(state, done, action)

        return np.asarray(state), reward, done

    # Refresh laserscan
    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except rospy.ServiceException as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()

            self.initGoal = False

        # self.goal_distance = self.getGoalDistace()
        state, done = self.getState(data)

        return np.asarray(state)
