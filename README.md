# notesrobocup
ros2 wtf -->to find error
failed to create domain

https://github.com/flexbe/flexible_navigation

source install/setup.bash 

ros2 launch flexbe_onboard behavior_onboard.launch.py

ros2 launch flexbe_webui flexbe_full.launch.py

/home/robotino/flexbe_ws/flexible_navigation/flex_nav_planners/src/follow_topic.cpp:325:52: error: no matching function for call to ‘nav2_core::GlobalPlanner::createPlan(geometry_msgs::msg::PoseStamped&, geometry_msgs::msg::PoseStamped&)’
325 |     nav_msgs::msg::Path path = planner_->createPlan(start_pose, goal_pose);

means it cant communicate with nav2_core->globalplanner file. problem is that there is an extra function.so we need to add another function to /home/robotino/flexbe_ws/flexible_navigation/flex_nav_planners/src/follow_topic.cpp

man (sth) to check the fuction if u r not sure how to use it

ros2 launch robotino_simulation robotino_simulation.launch.py namespace:=robotinobase1 launch_rviz:=true use_sim_time:=true

ros2 launch robotino_navigation robotino_bringup.launch.py namespace:=robotinobase1 use_sim_time:=true launch_nav2rviz:=true map:=map_webots.yaml

important topics for flexbe to launch the robot

/robotinobase1/goal_pose : to set position

https://ocw.tudelft.nl/courses/hello-real-world-ros-robot-operating-system/?view=lectures&paging=8 video tutorial for Flexbe

git checkout <distro> ->to switch version of the code at github

https://github.com/carologistics/fawkes-robotino/blob/master/cfg/conf.d/static_transforms.yaml axis coordinates
dyn transform needed gripper xyz dyn

grep -r ->

https://github.com/carologistics/fawkes-robotino/tree/master/src/lua/skills/robotino 

train picture https://colab.research.google.com/github/ultralytics/ultralytics/blob/main/examples/tutorial.ipynb

actions must have capital letter zB Gripper.action

![image](https://github.com/user-attachments/assets/f2ea371e-1ced-4fe9-b611-d8d649c049d3)

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
"
ros2 topic echo /tf | grep -B 1 -A 1 plate_top to get message before and after

to run action server
ros2 launch dynamic_tf dynamic_tf.launch.py
ros2 run dynamic_tf gripper_action_server
ros2 action send_goal /gripper gripper_msgs/action/Gripper "x_target: 0.0
y_target: 0.0
z_target: 0.0
frame: 'gripper_x_origin'
"

very important. never touch _init_.py !!!!!!!!!!!!!!!!

chmod +x /chown +X to allow program to be executable
ping ip address - to check whether it"s responding
/home/robotino/ros2/flexbe_ws/install/flex_nav_flexbe_states/lib/python3.12/site-packages/flex_nav_flexbe_states

ssh-copy-id robot@192.168.0.208
touch COLCON_IGNORE skip a certain file when its not compatible to build(colcon build)
