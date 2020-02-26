def test_pc(trajs,rank):
    # test pose controller
    pc = PoseController(pose_topic = "/robot_%d/cmd_pose"%rank)    
    xindex = 2 * rank
    yindex = 2 * rank + 1
    for i in range(0,20000,50):
        if(trajs[i,[xindex]] ==0):
            # time.sleep(0.5)
            continue
        x = trajs[i,[xindex]] /1000.0
        y = trajs[i,[yindex]] /1000.0
        print(x,y)
        pc.move_pose(x,y)
        time.sleep(0.1)
    print(trajs.shape) 

def test_vc(trajs,rank):
    # decide robot topic and world fram origin
    cmd_topic = "/robot_%d/cmd_vel"%rank
    odom_topic = "/robot_%d/odom"%rank
    pose_topic = "/robot_%d/cmd_pose"%rank
    crash_topic = 'robot_%d/is_crashed'%rank
    xindex = 2 * rank
    yindex = 2 * rank + 1

    #test velocity controller
    vc = VelController(cmd_topic,odom_topic,pose_topic,crash_topic)
    for i in range(0,20000,100):
        if(trajs[i,[xindex]] ==0 and trajs[i,[yindex]] ==0):
            # rospy.sleep(1.0)
            # vc.reset()
            continue
        x = (trajs[i,[xindex]]) /1000.0
        y = (trajs[i,[yindex]]) /1000.0
        vc.move_pose([x,y])
        if(x>10.0 or y>10.0):
            print("unexpectical position",x,y,"in rank: ",rank)
    vc.reset()

def mpi_control():
    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    size = comm.Get_size()

    # read trajectory of nine people into a array
    script_dir = os.path.abspath(os.getcwd())
    rel_path = '/THOR_dataset/Exp_3_run_2.tsv'
    file_path = script_dir+rel_path
    loader = TsvLoader(file_path = file_path)
    trajs = loader.extract_trajectory()

    test_pc(trajs,rank)
    # test_vc(trajs,rank = rank)