# ERNEST Rover Gazebo Simulation with MPC Navigation

This workspace implements a Gazebo simulation for the Ernest rover and includes an algorithm for navigation based on Model Predictive Control (MPC).

## Overview

The Ernest rover is simulated in the Gazebo environment, providing a realistic testing ground for navigation algorithms. This project focuses on implementing an MPC-based navigation system, allowing for efficient and predictive control of the rover's movements.

## Features

- Gazebo simulation of the Ernest rover
- MPC-based navigation algorithm
- ROS2 integration for robot control and simulation
- Support for both effort and position control modes

## Prerequisites

Before you begin, ensure you have the following installed:

- [ROS2](https://docs.ros.org/en/foxy/Installation.html) (Foxy or newer recommended)
- [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu)
- [Python](https://www.python.org/downloads/) (3.8 or newer)

## Installation

1. Clone this repository into your ROS2 workspace:
   ```
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/ernest-rover-simulation.git
   ```

2. Build the workspace:
   ```
   cd ~/ros2_ws
   colcon build
   ```

3. Source the setup file:
   ```
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

To spawn ERNEST in Gazebo and run the simulation:

1. Launch the Gazebo simulation with effort control:
   ```
   ros2 launch bringup effort.launch.py
   ```

2. In a new terminal, launch the MPC controller for effort control:
   ```
   ros2 launch mpc_controller effort_controller.launch.py
   ```

Alternatively, for position control:

1. Launch the Gazebo simulation with position control:
   ```
   ros2 launch bringup position.launch.py
   ```

2. In a new terminal, launch the MPC controller for position control:
   ```
   ros2 launch mpc_controller position_controller.launch.py
   ```

Acknowledgments

NASA Jet Propulsion Laboratory (JPL) for the development of the Ernest rover and its simulation
The ROS2 and Gazebo communities for their excellent simulation tools
Contributors to the MPC and robotics control fields

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
