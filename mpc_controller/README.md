This Package contains the implementation of a Path Tracking Nonlinear Model Predictive Control.
It has been implemented through CasADi.

To launch the script:
 - make sure the right controller has been selected in the gazebos config files (ernest_gazebo etc.) 
 #TODO: make the selection of the controller using args
 - ros2 launch mpc_controller effort_controller.launch.py ---> to use the effort joint controller
 - ros2 launch mpc_controller position_controller.launch.py ---> to use the position / velocity controllers

in the implementation folder (mpc_controller) we have:
1. models --> different Kinematics (Ackermann, 4WS, 1 DOF steering, 5 DOF)
2. mpc_node --> node that imports the kinematics and solve the optimization publishing the input on the topics of reference inputs
3. pid_node --> node that is used for the local control of the actuators. It subcribes to the reference inputs tocics.

results folder contains:
- The data that are by default saved in a txt file, to avoid saving bags.
- the matlab code to analyse the results



