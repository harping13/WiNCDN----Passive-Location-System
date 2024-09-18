#!/usr/bin/env python2
# coding=utf8

import rospy
import time
import tf
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalID
from capture_images import CaptureImages
from tf.transformations import euler_from_quaternion

class MoveAndCapture:
    def __init__(self):
        rospy.init_node('move_and_capture_node', anonymous=True)
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.pub_initialpose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_goal_result = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.goal_result_callback)
        self.sub_amcl_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.capture_images = CaptureImages("192.168.1.112", 8081)
        self.listener = tf.TransformListener()
        self.goal_reached = False
        self.current_pose = None
        self.last_goal = None
        self.previous_pose = None
        self.previous_goal = None
        self.i = 0

    def send_goal(self, x, y):
        # 在发送新目标之前记录当前位置和位姿信息
        if self.current_pose:
            self.previous_pose = PoseStamped()
            self.previous_pose.header.frame_id = "map"
            self.previous_pose.header.stamp = rospy.Time.now()
            self.previous_pose.pose.position = self.current_pose.position
            self.previous_pose.pose.orientation = self.current_pose.orientation
            rospy.loginfo("Recorded previous pose: ({}, {})".format(
                self.previous_pose.pose.position.x, self.previous_pose.pose.position.y))

        # 记录上一个目标
        if self.last_goal:
            self.previous_goal = self.last_goal

        # 发布当前位置进行重定位
        self.record_pose()
        rospy.sleep(3)  # 等待3秒以确保重定位完成

        rospy.sleep(1)  # 延时1秒以确保move_base初始化完成
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0  # 通常在2D导航中，z值设置为0

        goal.pose.orientation.x = 0
        goal.pose.orientation.y = 0
        goal.pose.orientation.z = 0
        goal.pose.orientation.w = 1

        self.pub_goal.publish(goal)
        self.goal_reached = False
        self.last_goal = goal
        rospy.loginfo("Sent goal: ({}, {})".format(x, y))

    def goal_result_callback(self, result):
        if result.status.status == 3:
            rospy.loginfo("Goal reached!")
            self.goal_reached = True
            self.capture_images.reset_image_flags()
            self.capture_images.set_capture_images(True)
            rospy.loginfo("Set capture_images to True")

            # 开始取流并旋转
            self.capture_and_rotate()
        elif result.status.status == 4:
            rospy.logwarn("Goal aborted!")
            self.cancel_goal()
            if self.previous_goal:
                rospy.loginfo("Resending previous goal: ({}, {})".format(
                    self.previous_goal.pose.position.x, self.previous_goal.pose.position.y))
                self.send_goal(self.previous_goal.pose.position.x, self.previous_goal.pose.position.y)
            else:
                rospy.logwarn("No previous goal to return to")

    def amcl_pose_callback(self, data):
        self.current_pose = data.pose.pose

    def record_pose(self):
        if self.current_pose and self.last_goal:
            rospy.loginfo("Recording pose: ({}, {}) with orientation w: {}".format(
                self.last_goal.pose.position.x, self.last_goal.pose.position.y, self.current_pose.orientation.w))
            
            initial_pose_msg = PoseWithCovarianceStamped()
            initial_pose_msg.header.frame_id = "map"
            initial_pose_msg.header.stamp = rospy.Time.now()
            initial_pose_msg.pose.pose.position.x = self.last_goal.pose.position.x
            initial_pose_msg.pose.pose.position.y = self.last_goal.pose.position.y
            initial_pose_msg.pose.pose.position.z = 0

            initial_pose_msg.pose.pose.orientation.x = 0
            initial_pose_msg.pose.pose.orientation.y = 0
            initial_pose_msg.pose.pose.orientation.z = self.current_pose.orientation.z
            initial_pose_msg.pose.pose.orientation.w = self.current_pose.orientation.w

            # 确保协方差矩阵包含36个浮点数
            initial_pose_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

            self.pub_initialpose.publish(initial_pose_msg)
            rospy.loginfo("Published initial pose for next goal")

    def cancel_goal(self):
        cancel_msg = GoalID()
        self.pub_cancel.publish(cancel_msg)
        rospy.loginfo("Current goal cancelled")

    def capture_and_rotate(self):
        # 旋转360度
        angular_speed = 0.5  # 每秒旋转0.5弧度
        total_rotation = 2 * 3.14159  # 360度的弧度值
        twist = Twist()
        twist.angular.z = angular_speed
        start_time = rospy.Time.now()
        
        i = 0
        while rospy.Time.now() - start_time < rospy.Duration(total_rotation / angular_speed):
            self.pub_cmd_vel.publish(twist)
            self.i = i
            self.capture_images_and_save()  # 在旋转过程中持续取流并保存图像
            rospy.sleep(0.1)
            i += 1

        # 停止旋转
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)
        
        # 停止取流
        self.capture_images.set_capture_images(False)
        rospy.loginfo("Capture and rotation complete")

    def capture_images_and_save(self):
        try:
            (trans, rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
            # tx = trans[0]
            # ty = trans[1]
            # (roll, pitch, yaw) = euler_from_quaternion(rot)
            # tw = yaw
            
            # filename = "pose_x{}_y{}_w{}.png".format(tx, ty, tw)
            filename = "{}_pose_x{}_y{}_z{}_ori_x{}_y{}_z{}_w{}.png".format(self.i, round(trans[0], 2), round(trans[1], 2), round(trans[2], 2), round(rot[0], 2),round(rot[1], 2),round(rot[2], 2), round(rot[3], 2))
            self.capture_images.capture_and_save(filename)
            rospy.loginfo("Captured and saved image with filename: {}".format(filename))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            rospy.logerr("Transform error: {}".format(ex))

if __name__ == '__main__':
    try:
        move_and_capture = MoveAndCapture()
        rospy.spin()  # 保持节点运行
    except rospy.ROSInterruptException:
        rospy.loginfo("Move and Capture node terminated.")

