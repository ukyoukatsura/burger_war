#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2
import random
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
import numpy as np
import math
from tf import TransformListener
from geometry_msgs.msg import PointStamped

camera_preview = True

camera_fov = 60.0
camera_width = 640.0

class MyRobot():
    def __init__(self):
        # bot name
        robot_name = rospy.get_param('~robot_name')
        self.name = robot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # move_base
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # lidar scan subscriber
        self.scan = LaserScan()
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)
        # camera subscriber
        self.img = None
        self.camera_preview = camera_preview
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
        # move_base
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # mode
        self.mode = 0

    # RESPECT
    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.name + "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def lidarCallback(self, data):
        self.scan = data

    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        if self.camera_preview:
            cv2.imshow("Image window", self.img)
            cv2.waitKey(1)
        # 緑の検出
        hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV_FULL)
        bgrLower = np.array([60, 0, 0])
        bgrUpper = np.array([120, 255, 255])
        hsv_mask = cv2.inRange(hsv_img, bgrLower, bgrUpper)
        rects = []
        labels, contours, hierarchy = cv2.findContours(hsv_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        for i in range(0, len(contours)):
            if len(contours[i]) > 0:

                rect = contours[i]
                x, y, w, h = cv2.boundingRect(rect)
                rects.append(x)
        #print(rects)
        if len(rects) != 0:
            self.mode = 1
            # robot正面から何度の方向に緑の物体が存在するか計算
            angle = (rects[0] - (camera_width / 2.0)) * (camera_fov / camera_width)
            print("#####角度#####")
            print(angle)
            # rectの大きさまで考慮する必要ありか？
            # lidarの点群からおおよその距離を算出
            if angle >= 0:
                distance = self.scan.ranges[int(angle)]
            else:
                distance = self.scan.ranges[int(359 + angle)]
            print("#####距離#####")
            print(distance)
            # robotから見た座標値を算出　前がx軸、左がy軸
            robot_x = math.cos(math.radians(angle)) * distance
            robot_y = -math.sin(math.radians(angle)) * distance
            print("#####x軸######")
            print(robot_x)
            print("#####y軸######")
            print(robot_y)
            
            ######要修正######
            '''
            # 地図座標系に変換
            listener = tf.TransformListener()
            listener.waitForTransform("/red_bot/map","/red_bot/base_footprint",rospy.Time(0),rospy.Duration(4.0))
            laser_point = PointStamped()
            laser_point.header.frame_id = "/red_bot/base_footprint"
            laser_point.header.stamp = rospy.Time(0)
            laser_point.point.x = robot_x
            laser_point.point.y = robot_y
            laser_point.point.z = 0.0
            p = PointStamped()
            p = listener.transformPoint("/red_bot/map", laser_point)
            # 方向と位置をゴールとして指定
            # 一旦方向は無視して位置でデバッグ
            print("#####x_map#####")
            print(p.point.x)
            print("#####y_map#####")
            print(p.point.y)
            self.setGoal(p.point.x,p.point.y,0)
            '''

    def basic_move(self):
        self.setGoal(-0.5,0,0)
        self.setGoal(-0.5,0,3.1415*0.25)
        
        self.setGoal(0,0.5,0)
        self.setGoal(0,0.5,3.1415*1.5)
        self.setGoal(0,0.5,3.1415)
        self.setGoal(0,0.5,3.1415*1.75)
        
        self.setGoal(0.5,0,3.1415*1.5)
        self.setGoal(0.5,0,3.1415)
        self.setGoal(0.5,0,3.1415*1.25)
        
        self.setGoal(0,-0.5,3.1415)
        self.setGoal(0,-0.5,3.1415*0.5)
        self.setGoal(0,-0.5,0)

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps
        while not rospy.is_shutdown():
            if self.mode == 0:
                self.basic_move()
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('my_robot_run')
    bot = MyRobot()
    bot.strategy()

