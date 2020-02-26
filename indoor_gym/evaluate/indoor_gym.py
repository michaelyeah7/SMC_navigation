from multiprocessing import Pool,Process
from contextlib import contextmanager
import time
import numpy as np
import os
from threading import Thread 
import rospy

from ..evaluate.load_tsv import TsvLoader
from ..evaluate.stage_controller import PoseController,VelController


class controller_worker(Thread):
    def __init__(self,trajs,rank=0,type="pose"):
        """
        rank: determine the topic to be listened and published
        type: determine the moving type
        delta_time: the stepping time between each index in trajs
        """
        Thread.__init__(self)
        self.rank = rank
        self.trajs = np.copy(trajs)
        self.type = type
        self.begin_index = 0
        self.end_index = 20000
        self.delta_time = 0.01

    def run(self):
        """
        all thread start learning from here
        """
        if(self.type == "pose"):
            self.move_obj_pose(rank = self.rank, trajs=self.trajs)
        elif (self.type == "vel"):
            self.move_obj_vel(rank = self.rank,trajs = self.trajs)
        else:
            print("no moving object in the thread%d"%self.rank)
        return

    def move_obj_vel(self,rank,trajs):
        # decide robot topic and world fram origin
        cmd_topic = "/robot_%d/cmd_vel"%rank
        odom_topic = "/robot_%d/odom"%rank
        pose_topic = "/robot_%d/cmd_pose"%rank
        crash_topic = 'robot_%d/is_crashed'%rank
        xindex = 2 * rank
        yindex = 2 * rank + 1

        #use velocity controller to move obs
        vc = VelController(cmd_topic,odom_topic,pose_topic,crash_topic)
        for i in range(self.begin_index,self.end_index,int(1/self.delta_time)):
            if(trajs[i,[xindex]] ==0 and trajs[i,[yindex]] ==0):
                rospy.sleep(1.0)
                vc.reset()
                continue
            x = (trajs[i,[xindex]]) /1000.0
            y = (trajs[i,[yindex]]) /1000.0
            vc.move_pose([x,y])
            if(x>10.0 or y>10.0):
                print("unexpectical position",x,y,"in rank: ",rank)
        vc.reset()
        return

    def move_obj_pose(self,rank,trajs):
        # move object using pose controller
        pc = PoseController(pose_topic = "/robot_%d/cmd_pose"%rank)    
        xindex = 2 * rank
        yindex = 2 * rank + 1
        for i in range(self.begin_index,self.end_index,int(1/self.delta_time)):
            if(trajs[i,[xindex]] ==0):
                time.sleep(1)
                continue
            x = trajs[i,[xindex]] /1000.0
            y = trajs[i,[yindex]] /1000.0
            pc.move_pose(x,y)
            time.sleep(1)

    def set_exetime(self,begin_time,end_time,delta_time = 0.01):
        """ 
        set the time of the testing scenarios,every 100 is one second
        delta_time: the stepping time between each index in trajs
        return: True means the thread is legal, False means the thread is illegal
        """
        length = self.trajs.shape[0]
        self.delta_time = delta_time
        self.end_index = end_time * int(1/delta_time)
        self.begin_index = begin_time * int(1/delta_time)
        if(self.end_index > length):
            print("end time over dataset")
            return False
        return True

class IndoorGym():
    def __init__(self,launch_stage = True,world_path ="worlds/room1.world"):
        """
        open a terminal to launch the world
        launch_satge: launch stage if needed
        Goal list: Goal 1: (4440, 8550) Goal 2: (9030, 3580) Goal 3: (1110, -3447) Goal 4: (-5468, -6159) Goal 5: (-130, 4150)
        """
        # self.goal_list = [[4.44,8.55],[9.030,3.580],[1.11,-3.447],[-5.468,-6.159],[-0.130,4.15]]
        self.goal_list = [[5.5,7.0],[-4.4, -5.1]]
        if(launch_stage):
            self.launch_stage(world_path=world_path)
        return
    
    def launch_stage(self,world_path = "worlds/room1.world"):
        """
        this is for launch stage simulator
        """
        cmd_stage = "rosrun stage_ros_add_pose_and_crash stageros  %s"%(world_path)
        cmd_bash = "exec bash"
        cmd =  cmd_stage + ";"+ cmd_bash
        os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmd)
        time.sleep(3)
        return

    def make(self,dataset_dir = "THOR_dataset", obs_num = 1, exp_id = 1):
        """
        load trajectory data from file,
        """
        # read trajectory of nine people into a array
        # script_dir = os.path.abspath(os.getcwd())
        # rel_path = '/THOR_dataset/Exp_%d_run_%d.tsv'%(obs_num,exp_id)
        # file_path = script_dir+rel_path
        # file_dir = os.path.dirname(__file__)
        file_path = dataset_dir + '/Exp_%d_run_%d.tsv'%(obs_num,exp_id)
        loader = TsvLoader(file_path = file_path)
        trajs = loader.extract_trajectory()
        self.trajs = np.copy(trajs)
        return
    
    def step(self,rank_range = [0],move_type = "pose"):
        """
        make a stage world simulating the human trjectory dataset 
        rank_range: determine the robot topic  needed to listen and publish
        move_type: "pose" or "vel": the type of driving robot
        """
        woker_list = []
        for rank in rank_range:
            temp_worker = controller_worker(trajs=self.trajs,rank=rank,type=move_type)
            woker_list.append(temp_worker)
        for worker in woker_list:
            print("thread_%d starts here"%(worker.rank))
            worker.start()
        return

    def step_scenarios(self,rank_range=[0],move_type="pose",test_time =20 ,scenario_id = 0):
        """
        make a stage world simulating the human trjectory dataset
        rank_range: determine the robot topic  needed to listen and publish
        move_type: "pose" or "vel": the type of driving robot
        test_time: the time of each test scenario
        scenarios_id: the id of testing scenarios 
        return: goal pose for robot
        """
        woker_list = []
        begin_time = scenario_id * test_time
        end_time = scenario_id * test_time + test_time
        print("scenatios %d: %ds -> %ds "%(scenario_id,begin_time,end_time))
        for rank in rank_range:
            temp_worker = controller_worker(trajs=self.trajs,rank=rank,type=move_type)
            woker_list.append(temp_worker)
        for worker in woker_list:
            worker.set_exetime(begin_time,end_time)
            worker.start()
        goal = self.goal_list[scenario_id % len(self.goal_list)]
        return goal

    def reset(self):
        """
        reset the stage world with static object
        """

        return



if __name__ == '__main__':
    pass