Collection of packages for the development of e2, an exhibition robot created by Airlab[http://airwiki.elet.polimi.it/index.php/AIRWiki]

Installation 
--------------------------------- 
To correctly install this packages, you'll need a working installation of ROS (>= hydro) [http://wiki.ros.org/ROS/Installation].

To completely exploit this packages is required

  * ROS (>=hydro)
  * ros-hydro-navigation
  * ros-hydro-openni-tracker
  * ros-actionlib
  * ros-hydro-openni-launch 
  * pico2wave (robot voice)
  * V-REP simulator [http://www.coppeliarobotics.com/]

Download the e2 repository in your catkin workspace src directory

$ git clone https://github.com/boottp/e2


Then go in ann directory to compile library

$ cd e2/ann/ann-build

compile

$ make installed

copy to top folder

cp ann/lib ../ -R

after this you can compile the workspace with 

$ catkin_make 

Testing
--------------------------------- 
You can test the package by running the simulated enviroment, using V-rep simulator [http://www.coppeliarobotics.com/]
Once open v-rep open e2simulation_fiera_devel.ttt in vrep_scenario folder. Start simulation and launch e2  stack using the following command:

$ roslaunch e2_launch simulation_robot.launch

On the robot

$ roslaunch e2_launch e2_robot.launch

Here are some service you can invoke to make some test

# ROBOT INTERACTION - Complete on real robot only
$ rosservice call /e2_brain/start

# Navigation TEST - To test in simulation
$ rosservice call /e2_nav/nav_abort
$ rosservice call /e2_nav/nav_auto
$ rosservice call /e2_nav/nav_goto
$ rosservice call /e2_nav/nav_start
$ rosservice call /e2_nav/test_detect
$ rosservice call /e2_nav/test_kinect_motor
$ rosservice call /e2_nav/test_neck
$ rosservice call /e2_nav/test_train
$ rosservice call /e2_nav/test_voice


# A video of the robot can be found at link:

coming soon...
