* Turn on the robot

* Open the brakes

* Load external control program

* Run the following command in a terminal:

roslaunch ur_robot_driver ur10_bringup.launch

* Press play on the robot

* Should read: "Ready to receive control commands."

If the robotiq gripper is not connecting open a new terminal and enter this command:
* sudo chmod 777 /dev/ttyUSB0
