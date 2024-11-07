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

git checkout ->to switch version of the code at github

grep -r ->

https://github.com/carologistics/fawkes-robotino/tree/master/src/lua/skills/robotino
