import time
import rospy
import tf
import numpy as np
from datetime import datetime


from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8

class PoseController():
    def __init__(self,pose_topic):
        self.pose_pub = rospy.Publisher(pose_topic, Pose, queue_size=10)   #publish pose to robor
        self.target_pose = Pose()
        self.target_pose.orientation.z = 1
        return

    def move_pose(self,x,y):
        self.target_pose.position.x = x
        self.target_pose.position.y = y
        self.target_pose.orientation.x = 1.0
        self.pose_pub.publish(self.target_pose) # reset pose here 
        rospy.sleep(0.01)
        # time.sleep(0.01)
        return



class VelController():
    def __init__(self,cmd_topic,odom_topic,pose_topic,crash_topic):
        self.reset_pose = None #can be the first pose
        self.current_pose = Pose()
        self.pc = PoseController(pose_topic)
        self.init_pose = None
        self.is_crashed = 0

        self.odom_sub = rospy.Subscriber(odom_topic,Odometry,self.odom_callback)
        self.cmd_pub = rospy.Publisher(cmd_topic,Twist,queue_size=10)
        self.check_crash = rospy.Subscriber(crash_topic, Int8, self.crash_callback)

        return
    
    def move_pose(self,target_position):
        """
        make robot move to target position, publish velocity while not crashed, publish pose while crashed
        """
        rospy.sleep(3.0)
        exe_time = 1.0# 0.5
        if(self.is_crashed == 1):
            self.pc.move_pose(target_position[0],target_position[1])
            self.is_crashed = 0
        else:
            trans_vel = self.calculate_twist_omni(self.current_pose,target_position,exe_time)
            self.execute_vel(trans_vel,exe_time)
        delta_translation = np.sqrt((self.current_pose.position.x - target_position[0])**2 + (self.current_pose.position.y - target_position[1])**2)
        return
    
    def execute_vel(self,vel,exe_time):
        start_time = time.time()
        while((time.time()-start_time)<=exe_time):
            self.cmd_pub.publish(vel)
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.01)
        return
    
    def calculate_twist_diff(self,current_pose,target_position,exe_time):
        """
        pose1,pose2: geometry_msg/Pose
        """
        trans_vel = Twist()
        ang_vel = Twist()
        translation = np.sqrt((current_pose.position.x - target_position[0])**2 + (current_pose.position.y - target_position[1])**2)
        target_yaw = np.arctan2(target_position[1] - current_pose.position.y,target_position[0] - current_pose.position.x)
        current_quaternion = (current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
        current_euler = tf.transformations.euler_from_quaternion(current_quaternion) 
        theta = target_yaw - current_euler[2]
        if(np.abs(theta)>np.pi):
            theta = theta - (2 * np.pi)
        trans_vel.linear.x = translation / exe_time
        ang_vel.angular.z = theta / exe_time

        return trans_vel,ang_vel

    def calculate_twist_omni(self,current_pose,target_position,exe_time):
        """
        pose1,pose2: geometry_msg/Pose
        """
        trans_vel = Twist()
        x_translation = (target_position[0] - current_pose.position.x)
        y_translation = (target_position[1] - current_pose.position.y)
        trans_vel.linear.x = x_translation / exe_time
        trans_vel.linear.y = y_translation / exe_time
        return trans_vel

    def odom_callback(self,msg):
        self.current_pose = msg.pose.pose
        if(self.init_pose == None):
            self.init_pose = msg.pose.pose
        return
    
    def reset(self):
        self.pc.move_pose(self.init_pose.position.x,self.init_pose.position.y)
        return
    
    def set_pose(self,x,y):
        self.pc.move_pose(x,y)
        return
    
    def crash_callback(self,flag):
        self.is_crashed = flag.data #1 if crashed
        return