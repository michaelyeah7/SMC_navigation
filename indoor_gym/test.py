import os
import time 
import rospy 

from evaluate.load_tsv import TsvLoader
from evaluate.stage_controller import PoseController,VelController
from evaluate.indoor_gym import IndoorGym


if __name__ == "__main__":
    rospy.init_node('plot_human', anonymous=True)
    script_dir = os.path.abspath(os.getcwd())

    ## launch stage and load dataset
    env = IndoorGym(launch_stage=True, world_path=script_dir+"/worlds/room1.world")
    env.make(dataset_dir=script_dir+"/THOR_dataset/", obs_num=1, exp_id=1)

    ## move human
    for i in range(20):
        goal = env.step_scenarios(rank_range=range(9),scenario_id= 8,move_type="vel",test_time=20)
        rospy.sleep(22)