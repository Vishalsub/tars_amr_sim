# my_amr_robot

## Description
This package provides a set of functionalities for controlling an Autonomous Mobile Robot (AMR).



## Installation
```bash
# Clone the repository
git clone https://github.com/Vishalsub/ros2-example.git
# change dir
cd my_ros2_projects
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

# buid ros workspace
colcon build

# source ros workspace
source install/setup.bash

# Run your first node
ros2 run my_package my_node
```



## Contributing
Thank you for considering contributing to this project! Please check out the Contributing Guidelines.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Code of Conduct
Please adhere to our Code of Conduct to foster an open and welcoming environment.

## Security Policy
If you discover any security-related issues, please contact us at []. All security vulnerabilities will be promptly addressed.

## Issues
To report a bug or request a feature, please open an Issue.

## Pull Requests
We welcome Pull Requests! Please follow our Pull Request Template when submitting one.