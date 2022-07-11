# Prog_RP_2022
In the following text there are the steps to follow to execute the project:
- execute the web control through the following instructions:
  1. cd ~/robotprogramming_2021_22/source/srrg2_workspace/src/srrg2_navigation_2d/config
  2. source ~/robotprogramming_2021_22/source/srrg2_workspace/devel/setup.bash
  3. ~/robotprogramming_2021_22/source/srrg2_workspace/src/srrg2_webctl/proc_webctl run_navigation.webctl
  4. go to the http://localhost:9001/ web page
  5. start roscore, stage, mapserver and rviz
- take rviz and make the following things:
  1. change the fixed frame from "map" to "odom"
  2. add a laser_scan, put its topic at /base_scan
- open another prompt and activate the keyboard (to move the robot) through " rosrun teleop_twist_keyboard  teleop_twist_keyboard.py "
- open another prompt, go in the folder of the project (in my VM the instruction is " cd Scrivania/prog_rp_2022/ ") and make here the catkin_make. Then make source devel/setup.bash and finally " rosrun lidar_matcher lidar_matcher " that starts the project
- at this point the map frame is created and so, we can change from "odom" to "map" our fixed frame
- start to move the robot with the keyboard and look at the transformations calculated by my project.



