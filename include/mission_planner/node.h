#ifndef NODE_H
#define NODE_H

#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <map>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>

namespace mission_planner {
  // TODO Split between a base class (Node) and derived class (NodeMission).
  /* 
   * TODO Add more variations to make Node more generalised (e.g. different msg
   * types, subscriber map, publisher map)
   */
  /*
   * Encapsulates ROS communication and ROS-related logic, used as a gateway
   * between ROS and the rest of the system.
   *
   * The main benefit to this over more common approaches (where ROS logic is
   * used in main etc) is that ROS is abstracted from the system and potential
   * middleware alternatives could be used without major adaptions to a system.
   */
  class Node
  {
    public:
	  // Creates and manages its own NodeHandle if one isn't supplied.
      Node();

	  /*
	   * Makes a local copy of a provided NodeHandle. This constructor is likely
	   * to be used when working with multiple ROS-related objects that should
	   * use the same NodeHandle reference.
	   */
      Node(ros::NodeHandle *n);

      ~Node();

      /*
	   * Models the same states used by move_base, to reflect updates from the
	   * ROS Navigation Stack.
	   */
      enum GoalState
      {
        PENDING,
        ACTIVE,
        PREEMPTED,
        SUCCEEDED,
        ABORTED,
        REJECTED,
        PREEMPTING,
        RECALLING,
        RECALLED,
        LOST
      };

      /*
	   * Send a goal indirectly to move_base, by first sending the coordinates
	   * to a middle-man node that should convert the satellite coordinates into
	   * the standardised ROS coordinate frames (see ROS REP 105).
	   */
      void sendGoal(double lat, double lon, double yaw);

	  // Sends a cancel request to move_base.
      void cancelGoal();

	  // Return the stored move_base status from the last update received.
      GoalState getStatus() const;

	  // Return the latitude of the robot's last known position.
      double getRobotLat() const;

	  // Return the longitude of the robot's last known position.
      double getRobotLon() const;

	  // Return the (orientation) of the robot's last known position.
      double getRobotYaw() const;

	  // Return whether the robot has reached the current goal or not.
      bool goalAchieved();

	  // Return if the goal is still active (or false if cancelled etc.)
      bool goalStillActive();

	  // Run all callback functions to get an update from subscribed topics.
      int update();

      // Return whether the robot is currently moving.
      bool isMoving();

    private:

      sensor_msgs::NavSatFix msg_goal;
      actionlib_msgs::GoalStatusArray msg_status;
      ros::NodeHandle nh;
      double pos_lat = 0;
      double pos_lon = 0;
      double pos_yaw = 0;
      double robot_velocity = 0;
      ros::Publisher pub_cancel;
      ros::Publisher pub_goal;
      GoalState status = PENDING;
      ros::Subscriber sub_pos;
      ros::Subscriber sub_status;
      ros::Subscriber sub_yaw;
      std::string topic_cancel;
      std::string topic_goal;
      std::string topic_pos;
      std::string topic_status;
      std::string topic_frameid;
      std::string topic_odometry;

      void callbackPos(const sensor_msgs::NavSatFix& msg_pos);
      void callbackStatus(const actionlib_msgs::GoalStatusArray::ConstPtr&
        msg_status);
      void callbackOdometry(const nav_msgs::Odometry& msg_odom);
      bool setup();
  };
}
#endif
