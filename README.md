Collection of packages for the development of e2, an exhibition robot created by Airlab[http://airwiki.elet.polimi.it/index.php/AIRWiki]

Installation 
--------------------------------- 
To correctly install this packages, you'll need a working installation of ROS (>= hydro) [http://wiki.ros.org/ROS/Installation].
You had to clone this git repository in your catkin worspace and build by run catkin_make.

To completely exploit each function is required

  * ROS (hydro)
  * ros-navigation-stack
  * ros-actionlib
  * ros-ar-track-alvar
  * espeak (robot voice)


Testing
--------------------------------- 
You can test the package by running the simulated enviroment, using V-rep simulator [http://www.coppeliarobotics.com/]
Once open v-rep open e2simulation_fiera.ttt in vrep_scenario folder. Start simulation and launch e2_navigation stack using the following command:

roslaunch e2_launch e2_simulation_robot.launch

You can use e2_navigation service call to test some functionality:

example:

rosservice call /e2_navigation/start 

will start a navigation plan to bring a user in a known location.
More informations can be found in README.md of each package. 






