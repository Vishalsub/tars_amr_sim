# Tars-Autonomous Mobile Robot (Simulated Envirnoment)

![example branch parameter](https://github.com/github/docs/actions/workflows/main.yml/badge.svg?branch=humble)

## Description
This repo provides a set of functionalities for controlling an Autonomous Mobile Robot (AMR) in Sim-Env.It's works only on **ROS2 (humble & jazzy)** with integration of gazebo-classic 11, ignition(fortress) for humble and gz-sim(harmonic) for jazzy.

Default branch is `sim_template`, Change branch to `humble` or `jazzy`


### To-do

- [x] Make urdf for diffdrive robot
- [x] Integrattion ign gazebo to ros
- [x] Add plugin for diff_drive, laser_scan, raw_camera
- [x] Add control package
- [x] Add world 
- [x] Add depth vision
- [ ] integrate with ros2_controls
- [ ] integrate with Slamtool box
- [ ] Generate map
- [ ] Autonomous navigation with nav2


## Installation
```bash
# Make dir
mkdir -p my_ros2_ws/src

# change dir
cd my_ros2_projects/src 

# Clone the repository
git clone https://github.com/Vishalsub/tars_amr_sim.git -b <branch_name>
```

### build from scratch(optional)
```bash
# Make dir
mkdir -p my_ros2_ws/src

# change dir
cd my_ros2_projects/src 

# Clone the repository
git clone https://github.com/Vishalsub/tars_amr_sim.git 
```

## How to Build and Run

### Using Docker

Build the Docker image:

```bash
cd docker  

docker build -t my_ros2_project .

```

### Without Docker

Source ROS2 environment and build:

```bash
# source ros env
source /opt/ros/humble/setup.bash

# change dir
cd my_ros2_projects/src 

# buid ros workspace
colcon build --symlink-install

# source ros workspace
source install/setup.bash

# Run your sim(it's ign gazebo)
ros2 launch tars_amr_sim gz.launch.py
```

> 💡 **NOTE:** Useful information that users should know, even when skimming content.


## Contributing
Thank you for considering contributing to this project! Please check out the [Contributing Guidelines](/CONTRIBUTING.md).

## License
This project is licensed under the MIT License - see the [LICENSE](/LICENSE) file for details.

## Code of Conduct
Please adhere to our [Code of Conduct](/CODE_OF_CONDUCT.md) to foster an open and welcoming environment.

## Security Policy
If you discover any security-related issues, please contact us at Discussion session. All security vulnerabilities will be promptly addressed.

## Issues
To report a bug or request a feature, please open an [Issue](https://github.com/Vishalsub/tars_amr_sim/issues).

## Pull Requests
We welcome Pull Requests! Please follow our [Pull Request Template]()when submitting one.
