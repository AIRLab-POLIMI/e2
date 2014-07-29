Collection of packages for the development of e2[http://airlab.elet.polimi.it/index.php/E2GoHome], an exhibition robot created by Airlab the Artificial intelligence Lab of the Politecnico di Milano.
The stack contains simulation environments for the V-Rep simulator, and the packages for facial analysis, moving in the space , track people and behaviour handling.

Installation 
--------------------------------- 
To correctly install these packages, you'll need a working installation of ROS (>= hydro) [http://wiki.ros.org/ROS/Installation].

To completely exploit the stack are required

  * ROS (>=hydro)
  * ros-hydro-navigation
  * ros-hydro-openni-tracker
  * ros-hydro-pocketshinx
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
$ rosservice call /e2_brain/start    # Will make the robot start finding people and interacting with them
$ rosservice call /e2_brain/abort    # Abort current action

# Navigation TEST - To test in simulation - Avoid facial analysis
$ rosservice call /e2_nav/nav_abort
$ rosservice call /e2_nav/nav_start
$ rosservice call /e2_nav/test_detect
$ rosservice call /e2_nav/test_kinect_motor
$ rosservice call /e2_nav/test_neck
$ rosservice call /e2_nav/test_train
$ rosservice call /e2_nav/test_voice

# A video of the robot can be found at link:
http://airlab.elet.polimi.it/index.php/E2GoHome
