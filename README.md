# Multi-dimensional RRT Planner

## Authors and Contributors
Author: David Dorf

## Description
3D and 2D implementation of Rapid Exploring Random Tree path planning for ROS 2. The main ROS 2 nodes are rrt2D and rrt3D, which can take user inputs and publish the nodes of the RRT and the path to the goal. The ROS 2 nodes are currently in development. The nodes can subscribe to topics for map data, marker obstacles, and can be launched with rrt2Dlaunch.xml and rrt3Dlaunch.xml, respectively. C++ version: https://github.com/daviddorf2023/multidim_rrt_cpp.




https://github.com/daviddorf2023/multidim_rrt_planner/assets/113081373/15a47a30-bc51-4ac6-bd49-b3ec30a9a2c5




## Installation
### Dependencies
* ROS 2 Iron
* Python 3.5+
* numpy 1.13.3+
### Building
Clone the repository into your ROS 2 workspace and build with colcon build.

## Usage
Install with `sudo apt install ros-iron-multidim-rrt-planner`
### ROS 2 Nodes
#### rrt2D
rrt2D is a ROS 2 node that implements the RRT algorithm in 2D. It can be launched with the rrt2Dlaunch.xml file using `ros2 launch multidim_rrt_planner rrt2Dlaunch.xml` or `ros2 launch multidim_rrt_planner rrt2Dlaunch.xml example_publishers:=True` for example usage. There are also parameters for the start, goal, map bounds, map sub, obstacle sub, step size, node limit, goal tolerance, and wall confidence. The node subscribes to the /occupancy_grid topic for map data and the /obstacle_markers_2D topic for marker obstacles. The node publishes the nodes of the RRT to the /rrt_markers topic and the path to the goal to the /rrt_path topic.
#### rrt3D
rrt3D is a ROS 2 node that implements the RRT algorithm in 3D. It can be launched with the rrt2Dlaunch.xml file using `ros2 launch multidim_rrt_planner rrt3Dlaunch.xml` or `ros2 launch multidim_rrt_planner rrt3Dlaunch.xml example_publishers:=True` for example usage. There are also parameters for the start, goal, map bounds, map sub, obstacle sub, step size, node limit, and goal tolerance. The node subscribes to the /occupancy_grid topic for map data and the /obstacle_markers_3D topic for marker obstacles. The node publishes the nodes of the RRT to the /rrt_markers topic and the path to the goal to the /rrt_path topic.
### Services
`ros2 service call /run_rrt std_srvs/srv/Empty` runs the RRT function in the main 2D and 3D nodes.
