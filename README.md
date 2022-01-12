# Mission

Mission is a lightweight terminal-based application that allows users to
construct, manage, and execute autonomous navigation missions for robots.

This ROS package which provides Mission uses a server-client architecture, which
includes a ROS server that handles the backend processing, allowing alternative
clients to be developed that can interface with the server. The ROS server uses
the [ROS Navigation Stack](https://github.com/ros-planning/navigation) as the
infastructure which implements the execution of autonomous navigation.

## Work in Progress - Here Be Dragons

Mission is currently under development, being taken from an old personal
project and being redesigned, therefore the project code is currently in a
work-in-progress state and is not yet currently ready for release and public
use. However, you can view the current code in the
[dev](https://github.com/jackrgm/mission_planner/tree/dev) branch. Changes are
rapid and unpredictable in this state until a first major version release.

## Getting Started

1. Set up the `mission_planner` ROS package (e.g. put it in your Catkin
   workspace and run `catkin_make`)
2. Start the server (`missionsrv`): `roslaunch mission_planner missionsrv`
3. Start a compatible client, e.g. Mission: `roslaunch mission_planner mission`

## Alternative Clients

The following known front end applications can be used to interface with
the server:

* Mission Qt
  ([mission_planner_qt](https://github.com/jackrgm/mission_planner_qt/)) - a
  Qt-based desktop GUI application.
