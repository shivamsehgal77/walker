# Walker algorithm for Turtlebot3
Making a walker algorithm in ROS2 for turlebot to avoid obstacles and keep moving, similar to the Roomba robot.

## Build Instructions

Navigate to workspace src directory in you system
```bash
cd ~/path/workspace/src
cd ~/ros2_ws/src # Path is my system
```
Cloning the Repository
```bash
git clone git@github.com:shivamsehgal77/walker.git
```
To build the package go the root folder
```bash
cd ..
```
Check for dependencies
```bash
rosdep install -i --from-path src --rosdistro humble -y
```
build the package
```bash
colcon build --symlink-install --packages-select walker
```
In another terminal source install/setup.bash
```bash
source install/setup.bash
```

## Running the package
without ros bag
```bash
TURTLEBOT3=burger ros2 launch walker gazebo_launch.launch 
```
with ross bag
```bash
TURTLEBOT3=burger ros2 launch walker gazebo_launch.launch record:=True
```
## Result
- Ros bag inspection `ros2 bag info outputs/rosbag_walker.db3`
- Ros bag play `ros2 bag play outputs/rosbag_walker.db3`


![ezgif com-gif-maker](https://github.com/shivamsehgal77/walker/assets/112571645/ee636e30-4942-475a-ae38-731cd8be4e04)

