# Baxter Robot Raconteur Driver

## Introduction

Robot Raconteur standard robot driver for the Rethink Robotics Baxter Robot

This driver communicates with the Baxter robot using ROS 1 topics and provides a standard Robot Raconteur service with type `com.robotraconteur.robotics.robot.Robot`.

This driver uses the `ros_csharp_interop` library (https://github.com/johnwason/ros_csharp_interop).

It is highly recommended that the docker image be used. See below for instructions.

Example driver clients are in the `examples/` directory. Position control mode is available but there is
not an example. See the SawyerRobotRaconteurDriver for an example of position control.

## Connection Info

The default connection information is as follows. These details may be changed using `--robotraconteur-*` command
line options when starting the service. Also see the
[Robot Raconteur Service Browser](https://github.com/robotraconteur/RobotRaconteur_ServiceBrowser) to detect
services on the network.

- URL: `rr+tcp://localhost:58660?service=robot`
  - When `--right-arm` is used, `rr+tcp://localhost:58661?service=robot`
- Device Name: `baxter_robot`
- Node Name: `baxter_robot`
- Service Name: `robot`
- Root Object Type:
  - `com.robotraconteur.robotics.robot.Robot`

## ROS Settings

The Baxter robot acts as a ROS master on the network. Before starting the driver, the following environmental variables
must be configured:

* `ROS_MASTER_URI` - The ROS master URI. This is typically something like `http://192.168.1.5:11311`. Replace
`192.168.1.5` with the actual IP address of the Baxter robot.
* `ROS_IP` - The IP address of the computer running the driver. This must be the ethernet interface on the same network with the Baxter robot. Firewalls must be disabled so that the incoming connections from the robot can be accepted. A typical IP address is `192.168.1.6`.

## Command Line Arguments

The following command line arguments are available:

* `--robot-info-file=` - The robot info file. Info files are available in the `config/` directory. See [robot info file documentation](https://github.com/robotraconteur/robotraconteur_standard_robdef/blob/master/docs/info_files/robot.md)
* `--left-arm` - Only control the left arm
* `--right-arm` - Only control the right arm
* `--left-electric-gripper` - A Rethink electric gripper is attached to the left arm.
* `--right-electric-gripper` - A Rethink electric gripper is attached to the left arm.
* `--left-gripper-info-file=` - The gripper info file for the left gripper. Info files are available in the `config/` directory. See [tool info file documentation](https://github.com/robotraconteur/robotraconteur_standard_robdef/blob/master/docs/info_files/tool.md)
* `--right-gripper-info-file=` - The gripper info file for the right gripper.

By default, the driver will control both robot arms. Two kinematic chains will be contained in the robot information
structures. The first seven joints control the left arm, and the right seven joints control the right arm. When
`--left-arm` or `--right-arm` are used, only one arm is controlled by the driver. Start two driver instances to
control the arms independently. This is useful if the client does not understand two arm robots.

## Running With Docker 

Docker is the recommended way to run the driver. Building and running can be complicated due to the dependencies on ROS and C\#.

### Dual Arm Control

```
sudo docker run --rm --net=host --privileged  -v /var/run/robotraconteur:/var/run/robotraconteur -v /var/lib/robotraconteur:/var/lib/robotraconteur -e ROS_MASTER_URI=http://192.168.1.5:11311 -e ROS_IP=192.168.1.6 wasontech/baxter-robotraconteur-driver /opt/baxter_robotraconteur_driver/bin/BaxterRobotRaconteurDriver --robot-info-file=/config/baxter_both_arm_robot_default_config.yml
```

Replace `ROS_MASTER_URI` and `ROS_IP` with the settings from your network. Use the available command line options to configure the gripper and robot info files.  It may be necessary to mount a docker "volume" to access configuration yml files that are not included in the docker image. See the docker documentation for instructions on mounting a local directory as a volume so it can be accessed inside the docker.

### Independent Arm Control

Start two docker images concurrently in different terminals:

```
sudo docker run --rm --net=host --privileged  -v /var/run/robotraconteur:/var/run/robotraconteur -v /var/lib/robotraconteur:/var/lib/robotraconteur -e ROS_MASTER_URI=http://192.168.1.5:11311 -e ROS_IP=192.168.1.6 wasontech/baxter-robotraconteur-driver /opt/baxter_robotraconteur_driver/bin/BaxterRobotRaconteurDriver --left-arm --robot-info-file=/config/baxter_left_arm_robot_default_config.yml
```

```
sudo docker run --rm --net=host --privileged  -v /var/run/robotraconteur:/var/run/robotraconteur -v /var/lib/robotraconteur:/var/lib/robotraconteur -e ROS_MASTER_URI=http://192.168.1.5:11311 -e ROS_IP=192.168.1.6 wasontech/baxter-robotraconteur-driver /opt/baxter_robotraconteur_driver/bin/BaxterRobotRaconteurDriver --right-arm --robot-info-file=/config/baxter_right_arm_robot_default_config.yml
```


## Building

Building the driver is complicated and not recommended. Refer to the `Dockerfile` for the required commands.

