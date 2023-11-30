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
## Results
<video width="320" height="240" controls>
  <source src="outputs/turtlebot_sim2.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>



