# Cozmo-linefollow
ROS

copy file linefollow.py to ROS package /cozmo_driver/nodes/

open a terminal
$ roscore

open another terminal, fill in the path according to your workspace
$ source ~/cozmo_drive/devel/setup.bash
$ python3.5 cozmo_lab/src/cozmo_driver-master/nodes/coz_driver.py

open another terminal
$ python linefollow.py

the following instruction use rostopic to send the message to control the robot individually
$ rostopic pub xxxxxxx
