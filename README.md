#Instructions:
##Setup Environment
```
mkdir -p SMC_ws/src'
cd SMC_ws/src
git clone https://github.com/michaelyeah7/SMC_navigation.git
cd SMC_ws
catkin_make

echo 'export ROS_HOSTNAME=localhost' >> ~/.bashrc
echo 'export ~/SMC_ws/devel/setup.bash'
```

##Running
open a new terminal
```
roscore
```
open second tab
```
cd src/SMC_navigation
rosrun stage_ros_add_pose_and_crash stageros  worlds/turtlebot2human.world
```
open third tab
```
python robot.py --human 2
```
opent fourth tab 
```
roslaunch SMC_navigation smc2human.launch
