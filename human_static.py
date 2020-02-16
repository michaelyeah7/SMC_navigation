import time
import rospy
import copy
import tf
import numpy as np
import os
import random


from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty
from std_msgs.msg import Int8

class Human():
    def __init__(self):
        index = 1
        node_name = 'human' + str(index)
        rospy.init_node(node_name, anonymous=None)

        # -----------Publisher and Subscriber-------------
        # goal_topic = 'robot_0' + '/goal_pose'
        # self.goal_pub = rospy.Publisher(goal_topic, Pose, queue_size=10)

        cmd_vel_topic = 'robot_' + str(index) + '/cmd_vel'
        self.cmd_vel = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        cmd_pose_topic = 'robot_' + str(index) + '/cmd_pose'
        self.cmd_pose = rospy.Publisher(cmd_pose_topic, Pose, queue_size=2)

        object_state_topic = 'robot_' + str(index) + '/base_pose_ground_truth'
        self.object_state_sub = rospy.Subscriber(object_state_topic, Odometry, self.ground_truth_callback)

        # laser_topic = 'robot_' + str(index) + '/base_scan'

        # self.laser_sub = rospy.Subscriber(laser_topic, LaserScan, self.laser_scan_callback)

        # odom_topic = 'robot_' + str(index) + '/odom'
        # self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)

        # crash_topic = 'robot_' + str(index) + '/is_crashed'
        # self.check_crash = rospy.Subscriber(crash_topic, Int8, self.crash_callback)



        # self.sim_clock = rospy.Subscriber('clock', Clock, self.sim_clock_callback)

        # -----------Service-------------------
        self.reset_stage = rospy.ServiceProxy('reset_positions', Empty)

        self.speed_GT = None
        self.state_GT = None
        while self.speed_GT is None or self.state_GT is None:
            pass

    def control_vel(self, action):
        move_cmd = Twist()
        move_cmd.linear.x = action[0]
        move_cmd.linear.y = 0.
        move_cmd.linear.z = 0.
        move_cmd.angular.x = 0.
        move_cmd.angular.y = 0.
        move_cmd.angular.z = action[1]
        self.cmd_vel.publish(move_cmd)

    def ground_truth_callback(self, GT_odometry):
        Quaternious = GT_odometry.pose.pose.orientation
        Euler = tf.transformations.euler_from_quaternion([Quaternious.x, Quaternious.y, Quaternious.z, Quaternious.w])
        self.state_GT = [GT_odometry.pose.pose.position.x, GT_odometry.pose.pose.position.y, Euler[2]]
        v_x = GT_odometry.twist.twist.linear.x
        v_y = GT_odometry.twist.twist.linear.y
        v = np.sqrt(v_x**2 + v_y**2)
        self.speed_GT = [v, GT_odometry.twist.twist.angular.z]

    def control_pose(self, pose):
        pose_cmd = Pose()
        assert len(pose)==3
        pose_cmd.position.x = pose[0]
        pose_cmd.position.y = pose[1]
        pose_cmd.position.z = 0

        qtn = tf.transformations.quaternion_from_euler(0, 0, pose[2], 'rxyz')
        pose_cmd.orientation.x = qtn[0]
        pose_cmd.orientation.y = qtn[1]
        pose_cmd.orientation.z = qtn[2]
        pose_cmd.orientation.w = qtn[3]
        self.cmd_pose.publish(pose_cmd)

def run(human):
    action = [0.3,0]
    while not rospy.is_shutdown():
        human.control_vel(action)
        if human.state_GT[0] < -6:
            pose = [4,0,np.pi]
            human.control_pose(pose)




if __name__ == '__main__':
    human1 = Human()
    run(human1)
    
