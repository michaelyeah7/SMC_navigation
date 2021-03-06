#!/usr/bin/env python
import os
import logging
import sys
import socket
import numpy as np
import rospy
import torch
import torch.nn as nn
from mpi4py import MPI
import argparse

from torch.optim import Adam
from collections import deque

from model.net import MLPPolicy, CNNPolicy
from robot_env import StageWorld
from model.ppo import ppo_update_stage1, generate_train_data
from model.ppo import generate_action
from model.ppo import transform_buffer

#for test 
from indoor_gym.evaluate.load_tsv import TsvLoader
from indoor_gym.evaluate.stage_controller import PoseController,VelController
from indoor_gym.evaluate.indoor_gym import IndoorGym



MAX_EPISODES = 5000
EP_LEN = 400
LASER_BEAM = 360
LASER_HIST = 3
HORIZON = 128
GAMMA = 0.99
LAMDA = 0.95
BATCH_SIZE = 1024
EPOCH = 2
COEFF_ENTROPY = 5e-4
CLIP_VALUE = 0.1
NUM_ENV = 2
OBS_SIZE = 360
ACT_SIZE = 2
LEARNING_RATE = 5e-5


def run(env, policy, policy_path, action_bound, optimizer, test_env):

    # rate = rospy.Rate(5)
    buff = []
    global_update = 3180
    global_step = 0


    # if env.mpi_rank == 0:
    #     env.reset_world()
    # env.store_resetPose()

    if env.mode == 'test':
        MAX_EPISODES = 50
    else:
        MAX_EPISODES = 5000

    for id in range(MAX_EPISODES):
        if env.mode == 'test':
            goal = test_env.step_scenarios(rank_range=range(9),scenario_id= 8,move_type="vel",test_time=20)

        rospy.sleep(3.0)
        env.reset_pose()
        # env.reset_world()

        env.generate_goal_point()
        terminal = False
        next_ep = False
        ep_reward = 0
        step = 1

        obs = env.get_laser_observation()
        obs_stack = deque([obs, obs, obs])
        goal = np.asarray(env.get_local_goal())
        speed = np.asarray(env.get_self_speed())
        state = [obs_stack, goal, speed]
        rospy.sleep(0.1)
        while not terminal and not rospy.is_shutdown():
            state_list = comm.gather(state, root=0)
            # state_list = [state]
            while state_list == None and env.mpi_rank == 0:
                obs = env.get_laser_observation()
                obs_stack = deque([obs, obs, obs])
                goal = np.asarray(env.get_local_goal())
                speed = np.asarray(env.get_self_speed())
                state = [obs_stack, goal, speed]
                            
                state_list = comm.gather(state, root=0)

            # generate actions at rank==0
            v, a, logprob, scaled_action=generate_action(env=env, state_list=state_list,
                                                         policy=policy, action_bound=action_bound)

            # execute actions
            real_action = comm.scatter(scaled_action, root=0)
            # print('env.mpi_rank {} real_action{}'.format(env.mpi_rank,real_action))
            # real_action = scaled_action[0]
            env.control_vel(real_action)

            # rate.sleep()
            rospy.sleep(0.001)

            # get informtion
            r, terminal, result = env.get_reward_and_terminate(step)
            ep_reward += r
            global_step += 1


            # get next state
            s_next = env.get_laser_observation()
            left = obs_stack.popleft()
            obs_stack.append(s_next)
            goal_next = np.asarray(env.get_local_goal())
            speed_next = np.asarray(env.get_self_speed())
            state_next = [obs_stack, goal_next, speed_next]

            if global_step % HORIZON == 0:
                state_next_list = comm.gather(state_next, root=0)
                # state_next_list = [state_next]
                last_v, _, _, _ = generate_action(env=env, state_list=state_next_list, policy=policy,
                                                               action_bound=action_bound)
            # add transitons in buff and update policy
            r_list = comm.gather(r, root=0)
            # r_list = [r]
            terminal_list = comm.gather(terminal, root=0)
            # terminal_list = [terminal]

            if env.mpi_rank == 0 and env.mode == 'train':
                buff.append((state_list, a, r_list, terminal_list, logprob, v))
                if len(buff) > HORIZON - 1:
                    s_batch, goal_batch, speed_batch, a_batch, r_batch, d_batch, l_batch, v_batch = \
                        transform_buffer(buff=buff)
                    t_batch, advs_batch = generate_train_data(rewards=r_batch, gamma=GAMMA, values=v_batch,
                                                              last_value=last_v, dones=d_batch, lam=LAMDA)
                    memory = (s_batch, goal_batch, speed_batch, a_batch, l_batch, t_batch, v_batch, r_batch, advs_batch)
                    print("PPO update start")
                    ppo_update_stage1(policy=policy, optimizer=optimizer, batch_size=BATCH_SIZE, memory=memory,
                                            epoch=EPOCH, coeff_entropy=COEFF_ENTROPY, clip_value=CLIP_VALUE, num_step=HORIZON,
                                            num_env=NUM_ENV, frames=LASER_HIST,
                                            obs_size=OBS_SIZE, act_size=ACT_SIZE)

                    print("PPO update finish")
                    buff = []
                    global_update += 1

            step += 1
            state = state_next


        if env.mpi_rank == 0 and env.mode == 'train':
            if global_update != 0 and global_update % 20 == 0:
                torch.save(policy.state_dict(), policy_path + '/Stage1_{}'.format(global_update))
                logger.info('########################## model saved when update {} times#########'
                            '################'.format(global_update))
        # distance = np.sqrt((env.goal_point[0] - env.init_pose[0])**2 + (env.goal_point[1]-env.init_pose[1])**2)
        distance = np.sqrt((env.goal_point[0] - env.init_pose[0])**2 + (env.goal_point[1]-env.init_pose[1])**2)

        logger.info('Env %02d, Goal (%05.1f, %05.1f), Episode %05d, setp %03d, Reward %-5.1f, Distance %05.1f, %s' % \
                    (env.mpi_rank, env.goal_point[0], env.goal_point[1], id + 1, step, ep_reward, distance, result))
        logger_cal.info(ep_reward)





if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--humans', type=int, default=8)
    parser.add_argument('--groups_num', type=int, default=1)
    parser.add_argument('--mode', default='test')
    args = parser.parse_args()

    NUM_ENV = args.groups_num

    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    size = comm.Get_size()


    if args.mode == 'train':
        robot_index = rank * 9
    else:
        robot_index = 9
        script_dir = os.path.abspath(os.getcwd())

        ## launch stage and load dataset
        test_env = IndoorGym(launch_stage=True, world_path=script_dir+"/worlds/room1.world")
        test_env.make(dataset_dir=script_dir+"/indoor_gym/THOR_dataset/", obs_num=1, exp_id=1)


    #set policy folder and log folder
    policydir = 'policy/robot%dhuman_policy'%args.humans
    if not os.path.exists(policydir):
        os.makedirs(policydir)

    # hostname = socket.gethostname()
    if not os.path.exists('./log_robot_%s_%dhuman/' %(args.mode,args.humans)):
        os.makedirs('./log_robot_%s_%dhuman/' %(args.mode,args.humans))
    output_file = './log_robot_%s_%dhuman/' %(args.mode,args.humans) + 'output.log'
    cal_file = './log_robot_%s_%dhuman/' %(args.mode,args.humans) + 'cal_on2human.log'

    # config log
    logger = logging.getLogger('mylogger')
    logger.setLevel(logging.INFO)

    file_handler = logging.FileHandler(output_file, mode='a')
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(logging.Formatter("%(asctime)s - %(levelname)s - %(message)s"))
    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setLevel(logging.INFO)
    logger.addHandler(file_handler)
    logger.addHandler(stdout_handler)

    logger_cal = logging.getLogger('loggercal')
    logger_cal.setLevel(logging.INFO)
    cal_f_handler = logging.FileHandler(cal_file, mode='a')
    file_handler.setLevel(logging.INFO)
    logger_cal.addHandler(cal_f_handler)


    if rank == 0:
        policy_path = policydir
        # policy_path = 'policy/robot2human_policy'
        # policy = MLPPolicy(obs_size, act_size)
        policy = CNNPolicy(frames=LASER_HIST, action_space=2)
        policy.cuda()
        opt = Adam(policy.parameters(), lr=LEARNING_RATE)
        mse = nn.MSELoss()

        file = policy_path + '/Stage1_8320'
        if os.path.exists(file):
            logger.info('####################################')
            logger.info('############Loading Model###########')
            logger.info('####################################')
            state_dict = torch.load(file)
            policy.load_state_dict(state_dict)
        else:
            logger.info('#####################################')
            logger.info('############Start Training###########')
            logger.info('#####################################')
    else:
        policy = None
        policy_path = None
        opt = None

    env = StageWorld(beam_num=360,mpi_rank = rank, robot_index=robot_index, mode=args.mode)
    reward = None
    action_bound = [[0, -1], [1, 1]]
    try:
        if args.mode == 'train':
            print('env.mpi_rank',env.mpi_rank)
            run(env=env, policy=policy, policy_path=policy_path, action_bound=action_bound, optimizer=opt, test_env=None)
        else:
            run(env=env, policy=policy, policy_path=policy_path, action_bound=action_bound, optimizer=opt, test_env=test_env)
    except KeyboardInterrupt:
        pass