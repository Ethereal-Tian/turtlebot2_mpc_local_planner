# turtlebot2_mpc_local_planner

## run turtlebot gazebo simulator
```
roslaunch turtlebot_gazebo turtlebot_kinect_hokuyo_world_maze.launch
```

## run mpc_local_planner demo without gazebo
```
roslaunch mpc_local_planner_examples diff_drive_minimum_time_costmap_conversion.launch
roslaunch mpc_local_planner_examples diff_drive_minimum_time.launch
roslaunch mpc_local_planner_examples diff_drive_quadratic_form.launch

```

## run mpc_local_planner demo with gazebo
```
roslaunch mpc_local_planner_examples turtlebot_minimum_time_costmap_conversion.launch
roslaunch mpc_local_planner_examples turtlebot_minimum_time.launch
roslaunch mpc_local_planner_examples turtlebot_quadratic_form.launch

```


## The Nice Code

mpc_local_planner : https://github.com/rst-tu-dortmund/mpc_local_planner

