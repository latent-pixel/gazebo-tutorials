[![License: MIT](https://img.shields.io/badge/License-MIT-pink.svg)](https://opensource.org/licenses/MIT)

# ROS2 Gazebo Tutorials
This is a basic tutorial on obstacle avoidance using turtlebot3 and ROS2. Firstly, install all the dependencies required 

## **Setting up and sourcing ROS**
You will have to install ROS2 first, the instructions for which can be found on http://docs.ros.org/en/foxy/Installation.html. We will be using ROS2 Foxy for the purposes of this tutorial.

First, source your ROS2 environment:

```
source /opt/ros/foxy/setup.bash
```
Alternatively, you can add this to your system's .bashrc file to avoid sourcing ROS2 underlay each time a new terminal is opened: 
```
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```

## **Setting up the project**
Now, create a workspace to work with new packages. This is done so that you don't have to build all the packages each time you try building a new package. 
```
mkdir -p <your_dir>/ros2_ws/src
cd ros2_ws/src/
```
Download the code from the repository:
```
git clone git@github.com:Bhargav-Soothram/gazebo-tutorials.git
```
Now rename the package using (this is for build purposes)
```
mv gazebo_tutorials simple_walker
```

## **Building the package**
It is time to build the package! But before that, check if you have all the dependencies using resdep before going ahead:
```
rosdep install -i --from-path src --rosdistro foxy -y
```
Build the package:
```
cd ..
colcon build --packages-select simple_walker
```

## **Running the package**
Open a new terminal, navigate to `ros2_ws`, and source the setup files:

```
. install/setup.bash
```

### **Using Launch files**
We can use launch files to 'spin' multiple nodes at once with arguments passed to each of them. The arguments provided are the parameters to the nodes being executed. To run the obstacle avoidance simulation, use the following command:
```
ros2 launch simple_walker simple_walker.launch record:=<True/False>
```

* `record`  *#whether or not to record a rosbag file for the current execution (Set to False by default)* 


## **Inspecting and playing the rosbag file**
Open a new terminal, navigate to `ros2_ws`, and source the setup files as we did before.

For inspection:
```
ros2 bag info <rosbag_file>
```
*(rosbag_file is a `.bag` file)

To play a recorded rosbag file:
```
ros2 bag play <rosbag_file>
```
* `_launch.py`: Runs a publisher and subsriber with the given message
* `_launch_bag.py`: Inherits the functionality of `_launch.py` and adds the option to record rosbags

## **Sample Output**
![](results/output.gif)