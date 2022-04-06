#!/usr/bin/env python
# encoding=utf-8
# The MIT License (MIT)
#
# Copyright (c) 2018 Bluewhale Robot
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Randoms
#

import rospy
from rospy import topics
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from body_pose.srv import BodyPose, BodyPoseRequest
import time
import cv2

current_image = None


def update_image(image):
    global current_image
    current_image = image


def draw_point(cv_image, location):
    cv2.circle(cv_image, (int(location[0]), int(
        location[1])), 4, (255, 0, 0), thickness=4)


if __name__ == "__main__":
    rospy.init_node("body_pose_test", anonymous=True)
    rospy.Subscriber("~image", Image, update_image)
    rospy.loginfo("Waitting for get_body_pose service")
    rospy.wait_for_service("~get_body_pose")
    rospy.loginfo("Waitting for get_body_pose service succeed")
    get_body_pose = rospy.ServiceProxy('~get_body_pose', BodyPose)
    while current_image is None and not rospy.is_shutdown():
        time.sleep(1)
        rospy.logwarn("Cannot get image from {topic}".format(
            topic="~image"))

    # publish processed image
    image_pub = rospy.Publisher("~processed_image", Image, queue_size=10)

    while not rospy.is_shutdown():
        req = BodyPoseRequest()
        req.image = current_image
        res = get_body_pose(current_image)
        # draw res in message
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(req.image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        for body in res.body_poses:
            if body.nose[0] > 0:
                draw_point(cv_image, (body.nose[0], body.nose[1]))
            if body.right_eye[0] > 0:
                draw_point(cv_image, (body.right_eye[0], body.right_eye[1]))
            if body.left_eye[0] > 0:
                draw_point(cv_image, (body.left_eye[0], body.left_eye[1]))
            if body.right_ear[0] > 0:
                draw_point(cv_image, (body.right_ear[0], body.right_ear[1]))
            if body.left_ear[0] > 0:
                draw_point(cv_image, (body.left_ear[0], body.left_ear[1]))
            if body.right_arm_top[0] > 0:
                draw_point(
                    cv_image, (body.right_arm_top[0], body.right_arm_top[1]))
            if body.left_arm_top[0] > 0:
                draw_point(
                    cv_image, (body.left_arm_top[0], body.left_arm_top[1]))
            if body.right_arm_middle[0] > 0:
                draw_point(
                    cv_image, (body.right_arm_middle[0], body.right_arm_middle[1]))
            if body.left_arm_middle[0] > 0:
                draw_point(
                    cv_image, (body.left_arm_middle[0], body.left_arm_middle[1]))
            if body.right_arm_bottom[0] > 0:
                draw_point(
                    cv_image, (body.right_arm_bottom[0], body.right_arm_bottom[1]))
            if body.left_arm_bottom[0] > 0:
                draw_point(
                    cv_image, (body.left_arm_bottom[0], body.left_arm_bottom[1]))
            if body.right_leg_top[0] > 0:
                draw_point(
                    cv_image, (body.right_leg_top[0], body.right_leg_top[1]))
            if body.left_leg_top[0] > 0:
                draw_point(
                    cv_image, (body.left_leg_top[0], body.left_leg_top[1]))
            if body.right_leg_middle[0] > 0:
                draw_point(
                    cv_image, (body.right_leg_middle[0], body.right_leg_middle[1]))
            if body.left_leg_middle[0] > 0:
                draw_point(
                    cv_image, (body.left_leg_middle[0], body.left_leg_middle[1]))
            if body.right_leg_bottom[0] > 0:
                draw_point(
                    cv_image, (body.right_leg_bottom[0], body.right_leg_bottom[1]))
            if body.left_leg_bottom[0] > 0:
                draw_point(
                    cv_image, (body.left_leg_bottom[0], body.left_leg_bottom[1]))
        try:
            image_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        image_pub.publish(image_msg)
