# IONA stability package
This package calculates the stability of the IONA robot.
## Nodes description
There are 4 nodes in this package. 
**kinova_com.py** capclulates the center of mass of the robot's arm. **iona_com.py** aclculates the center of mass of the entire robot. 
**forward_zmp.py** constrains the linear velocity and linear and angular accelerations according based on the position of the center of mass of the robot.
**inverse_zmp.py** maximizes the linear velocity and linear and angular accelerations by lowering the chest of the robot.
## Launch
First it is needed to launch **kinova_com.py** by executing the command: `roslaunch iona_stability kinova_com.launch`. Then the **iona_com.py** should be launched by executing the commmand: `roslaunch iona_stability iona_com.launch`.
Left two nodes cannet be used simultaneously, so if the accelearion restriction is qequired than roslaunch `iona_stability forward_zmp.launch` command should be used. If there is the requirement to maximize the accelerations by lowering the chest 
the inverse node should be launched by executing  `roslaunch iona_stability inverse_zmp.launch` command. 
