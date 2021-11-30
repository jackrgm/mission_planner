# Mission

Mission is a lightweight terminal-based application that allows users to
construct, manage, and execute autonomous navigation missions for robots.

This ROS package which provides Mission uses a server-client architecture, which
includes a ROS server that handles the backend processing, allowing alternative
clients to be developed that can interface with the server. The ROS server uses
the [ROS Navigation Stack](https://github.com/ros-planning/navigation) as the
infastructure which implements the execution of autonomous navigation.

## Getting Started

1. Set up the `mission_planner` ROS package (e.g. put it in your Catkin
   workspace and run `catkin_make`)
2. Start the server (`missionsrv`): `roslaunch mission_planner missionsrv`
3. Start a compatible client, e.g. Mission: `roslaunch mission_planner mission`

## Alternative Clients

The following known front end applications can be used to interface with
the server:

* Mission Qt (mission_planner_qt) - a Qt-based desktop GUI application.
