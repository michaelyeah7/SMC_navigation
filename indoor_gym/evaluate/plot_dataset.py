import time
import rospy
import tf
import numpy as np
from datetime import datetime


from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry

class human_controller():
    def __init__(self,obj_topic,pose_topic,pose_stamp_topic,cmd_topic,odom_topic):
        self.sub = rospy.Subscriber(obj_topic, PoseStamped, self.callback) #subscribe object pose
        self.pose_pub = rospy.Publisher(pose_topic, Pose, queue_size=10)   #publish pose to robor
        self.pose_stamp_pub = rospy.Publisher(pose_stamp_topic, PoseStamped, queue_size=10)
        self.cmd_pub = rospy.Publisher(cmd_topic,Twist,queue_size=10)   #publish vel to robot

        self.last_sec = 0
        self.first_flag = True
        self.pose_topic = pose_topic
        self.traj_controller = trajectory_controller(cmd_topic,odom_topic)
        time.sleep(0.1)
        return

    def callback(self,msg):
        now_sec = msg.header.stamp.secs 
        if(not self.first_flag):
            if((now_sec - self.last_sec) > 2):
                self.first_flag = True # start new traj
            else:
                self.traj_controller.move_pose(msg.pose) # continue moving
            self.last_sec = now_sec # record time

        if(self.first_flag):
            self.first_flag = False
            self.pose_pub.publish(msg.pose) # reset pose here 
            self.last_sec = now_sec #record time
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            print("restting pose",self.pose_topic,current_time)
            time.sleep(0.05)

        # self.pose_pub.publish(msg.pose)
        # now = datetime.now()
        # current_time = now.strftime("%H:%M:%S")
        # print("restting pose",self.pose_topic,current_time)
        return

class trajectory_controller():
    def __init__(self,cmd_topic,odom_topic):
        self.start_pose = None
        self.trajecotry = []
        self.vel_list = None
        self.current_pose = Pose()

        self.odom_sub = rospy.Subscriber(odom_topic,Odometry,self.odom_callback)
        self.cmd_pub = rospy.Publisher(cmd_topic,Twist,queue_size=10)
        return
    
    def move_pose(self,pose):
        self.trajecotry.append(pose)
        exe_time =  0.05
        trans_vel,ang_vel = self.calculate_twist(self.current_pose,pose,exe_time)
        self.execute_vel(ang_vel,exe_time)
        self.execute_vel(trans_vel,exe_time)
        return
    
    def execute_vel(self,vel,exe_time):
        self.cmd_pub.publish(vel)
        time.sleep(exe_time)
        self.cmd_pub.publish(Twist())
        return
    
    def calculate_twist(self,pose1,pose2,exe_time):
        """
        pose1,pose2: geometry_msg/Pose
        """
        trans_vel = Twist()
        ang_vel = Twist()
        translation = np.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2)
        theta = np.arctan2(pose2.position.y - pose1.position.y,pose2.position.x - pose1.position.x)
        # quaternion1 = (pose1.orientation.x,pose1.orientation.y,pose1.orientation.z,pose1.orientation.w)
        # quaternion2 = (pose2.orientation.x,pose2.orientation.y,pose2.orientation.z,pose2.orientation.w)
        # euler1 = tf.transformations.euler_from_quaternion(quaternion1) 
        # euler2 = tf.transformations.euler_from_quaternion(quaternion2) 
        # theta = euler2[2] - euler1[2]
        if(np.abs(theta)>np.pi):
            theta = theta - (2 * np.pi)
        trans_vel.linear.x = translation / exe_time
        ang_vel.angular.z = theta / exe_time

        return trans_vel,ang_vel

    def odom_callback(self,msg):
        self.current_pose = msg.pose.pose
        return

class scenario_controller():
    def __init__(self):

        return

if __name__ == "__main__":
    rospy.init_node('plot_human', anonymous=True)
    
    controller_list = []
    for i in range(10):
        obj_topic = "/object_%d"%(i+3)
        pose_topic = "/robot_%d/cmd_pose"%i
        odom_topic = "/robot_%d/odom"%i
        pose_stamp_topic =  "/robot_%d/cmd_pose_stamped"%i
        cmd_topic = "/robot_%d/cmd_vel"%i
        controller_list.append(human_controller(obj_topic,pose_topic,pose_stamp_topic,cmd_topic,odom_topic))

    rospy.spin()

