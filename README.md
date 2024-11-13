# autonomous
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg?style=plastic)](https://github.com/wltjr/autonomous/blob/master/LICENSE.txt)
![Build Status](https://github.com/wltjr/autonomous/actions/workflows/docker_build.yml/badge.svg)
[![Code Quality](https://sonarcloud.io/api/project_badges/measure?project=wltjr_autonomous&metric=alert_status)](https://sonarcloud.io/dashboard?id=wltjr_autonomous)

autonomous is a ROS 2 package that is used to set one or more way points, or goal
pose destinations, that is then used by [Nav2](https://nav2.org/) to navigate
the robot.

## Download
Download and unpack or clone this repositories contents into your ros2
workspace; ex `~/ros2_ws/src/autonomous`.


## Build
The package is built using the standard ROS 2 build process. Building is done
using colcon which will invoke cmake and run the necessary commands. Run the
following command in your ros2 workspace; ex `~/ros2_ws/`.
```bash
colcon build --symlink-install --packages-select  autonomous
```

### Source install
Make sure to run the following command after install and login. Run the
following command in your ros2 workspace; ex `~/ros2_ws/`.
```bash
source install/setup.bash
```

You may want to have your development user environment do this on login via
`~/.bashrc` file; add the following to the end of that file.
```bash
source ~/ros2_ws/install/setup.bash
```

