# autonomous
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg?style=plastic)](https://github.com/wltjr/autonomous/blob/master/LICENSE.txt)
[![Build Status](https://github.com/wltjr/autonomous/actions/workflows/docker_build.yml/badge.svg)](https://github.com/wltjr/autonomous/actions)
[![Code Quality](https://sonarcloud.io/api/project_badges/measure?project=wltjr_autonomous&metric=alert_status)](https://sonarcloud.io/dashboard?id=wltjr_autonomous)

autonomous is a ROS 2 package that is used to set one or more way points, or goal
pose destinations, that is then used by [Nav2](https://nav2.org/) to navigate
the robot. Only a single goal is supported at this time, multi-goal poses will
be supported at a later date TBD.

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

## Configuration
The package can be configured using an external yaml config file or directly
when launched as included in the provided launch script. The follow are all
required parameters, where `goal_poses` are a vector of `(X,Y,Z)` goal
coordinates and `frame_id` is the frame id of the map for the published `goal_pose`.
```yaml
goal_poses: [3.0, 3.0, 0.0]
frame_id: 'map'
```

## Launch
There is a default launcher provided, though this can be integrated into
existing rather than used directly. For completion sake, the launcher is
provided, and can be used from another launcher to invoke this stack.

```bash
ros2 launch autonomy autonomous.py
```

## Usage
To use the package, or to publish the desired goal pose location run the
following command, or have the following message be published.
```bash
ros2 topic pub /autonomy std_msgs/String "data: 'go'" -1
```
