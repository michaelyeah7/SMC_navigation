# Instructions:
## Setup Environment
```
mkdir -p SMC_ws/src
cd SMC_ws/src
git clone https://github.com/michaelyeah7/SMC_navigation.git
cp -r SMC_navigation/stage_ros_add_pose_and_crash ./
cd SMC_ws
. /opt/ros/melodic/setup.bash
catkin_make

echo 'export ROS_HOSTNAME=localhost' >> ~/.bashrc
echo 'export $HOME/SMC_ws/devel/setup.bash' >> ~/.bashrc

pip install torch --no-cache-dir
pip install future
```

## Running
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
