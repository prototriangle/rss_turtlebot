# RSS Turtlebot

This is the lab project for Robotics System and Science at Edinburgh Uni.

Authors:
* Ignat Georgiev
* Alex Roy

## Setup

### SSH Key

Set up your [SSH key](https://dev.to/sndrx/how-to-set-up-an-ssh-key-and-use-it-in-gitlab--42p1).

### Install ROS Kinetic

[link](http://wiki.ros.org/kinetic/Installation/Ubuntu)

### Clone repo

```bash
mkdir ~/ros
git clone git@gitlab.com:imgeorgiev/rss_turtlebot.git ~/rss_turtlebot
```

### Install ROS dependencies

```bash
cd ~/ros/rss_turtlebot
rosdep install -iy --from-path src
```

### Build

```bash
catkin build
```

### Run simulation

```bash
source devel/setup.bash
roslaunch launch/simulation.launch rviz:=false record:=false
```
