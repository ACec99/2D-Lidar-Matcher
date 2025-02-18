# **2D Lidar Matcher**

This project implements a 2D scan matcher that takes 2D laser scans as input and estimates the transformation matrix between them. The algorithm is based on the Iterative Closest Point (ICP) method, which aligns successive scans by minimizing the difference between corresponding points.

# **Key Points**

- **ICP-based Registration**: Computes the optimal transformation (rotation + translation) to align two scans;
- **ROS Integration**: Designed to work within the Robot Operating System (ROS) for real-time applications;
- **Robust and Efficient**: Provides an accurate and reliable method for scan matching in robotics and autonomous navigation.

# **Execution**

- **Clone the robotprogramming_2021_22 repository**:

  ```bash
  git clone https://gitlab.com/grisetti/robotprogramming_2021_22.git
  ```

- **execute the web control**:

  ```bash
  cd ~/robotprogramming_2021_22/source/srrg2_workspace/src/srrg2_navigation_2d/config
  source ~/robotprogramming_2021_22/source/srrg2_workspace/devel/setup.bash
  ~/robotprogramming_2021_22/source/srrg2_workspace/src/srrg2_webctl/proc_webctl run_navigation.webctl
  ```

  - go to the http://localhost:9001/ web page
 
  - start roscore, stage, mapserver and rviz

- **take rviz and make the following things**:

  - change the fixed frame from *map* to *odom*
 
  - add a *laser_scan*, put its topic at */base_scan*
 
- **open another prompt** and **activate the keyboard** (to move the robot) through:

  ```bash
  rosrun teleop_twist_keyboard  teleop_twist_keyboard.py
  ```

- **open another prompt**, **go in the folder of the project** and start it in the following way:

  ```bash
  catkin_make
  source devel/setup.bash
  rosrun lidar_matcher lidar_matcher
  ```

- at this point the map frame is created and so, we can change from *odom* to *map* our fixed frame

- start to move the robot with the keyboard and look at the transformations calculated by my project.
  





