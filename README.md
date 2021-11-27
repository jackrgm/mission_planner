# Mission

Mission is a lightweight terminal-based application that allows users to
construct, manage, and execute autonomous navigation missions.

This ROS package which provides Mission uses a server-client architecture, which
includes a ROS server that handles the backend processing, allowing alternative
clients to be developed that can interface with the server.

## Getting Started

1. Start the server (`missionsrv`): `roslaunch mission_planner missionsrv`
2. Start a compatible client, e.g. Mission: `roslaunch mission_planner mission`

## Alternative Clients

The following known front end applications can be used to interface with
the server:

* Mission Qt (mission_planner_qt) - a Qt-based desktop GUI application.
