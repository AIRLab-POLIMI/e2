Collection of packages for the development of e2, an exhibition robot created by Airlab[http://airwiki.elet.polimi.it/index.php/AIRWiki]

Installation 
--------------------------------- 
To correctly install this packages, you'll need a working installation of ROS (>= hydro) [http://wiki.ros.org/ROS/Installation].
You had to clone this git repository in your catkin worspace and build by run catkin_make.

To completely exploit this packages is required

  * ROS (>=hydro)
  * ros-hydro-navigation
  * ros-hydro-openni-tracker
  * ros-actionlib
  * ros-hydro-openni-launch 
  * pico2wave (robot voice)
  * V-REP simulator [http://www.coppeliarobotics.com/]

Testing
--------------------------------- 
You can test the package by running the simulated enviroment, using V-rep simulator [http://www.coppeliarobotics.com/]
Once open v-rep open e2simulation_fiera.ttt in vrep_scenario folder. Start simulation and launch e2  stack using the following command:

roslaunch e2_launch e2_simulation_robot.launch

Here are some service you can invoke to make some test

$ rosservice call /e2_brain/nav_abort
$ rosservice call /e2_brain/nav_auto
$ rosservice call /e2_brain/nav_goto
$ rosservice call /e2_brain/nav_start
$ rosservice call /e2_brain/test_detect
$ rosservice call /e2_brain/test_kinect_motor
$ rosservice call /e2_brain/test_neck
$ rosservice call /e2_brain/test_train
$ rosservice call /e2_brain/test_voice

