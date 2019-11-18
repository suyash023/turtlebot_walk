[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## ROS turtlebot walk

The code in this repository is for the turtlebot walk, using obstacle detection from depth image of kinect.
## Dependencies for code

The system runs on ROS Kinetic-kame with Ubuntu machine with packages roscpp, nav_msgs, geometry_msgs, cv_bridge, tf installed.

The code also additionally uses the boost library


Additional rqt package components must also be installed to view the ros messages.

## Building the code.
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/suyash023/turtlebot_walk.git
cd ../..
catkin_make
```


## Running the code

Source the environment of the catkin workspace.

```
source ./devel/setup.bash
```
Launch the talker_listener.launch file
```
roslaunch turtlebot_walk turtlebot_walk.launch
```

Open a separate terminal and run 
```
rqt_console 
```
to view the ros messages being passed.


## To record bag files

To record the data published on all topics except camera topic in a bag file pass an argument record:=true to the roslaunch command as follows
```
roslaunch turtlebot_walk turtlbot_walk.launch record:=true
```
This creates a bag file in the ~/.ros/log folder.
to move the file use the following commnd with appropriate arguments
```
mv ~/.ros/log/(bag_file_name).bag (catkin_ws folder location)
```
## To play the bag files
To replay the recorded bag files use the following command:

First start roscore in a different terminal
```
roscore
```
Then run following command:
```
rosbag play results/data.bag (corresponding bag file name)
```



