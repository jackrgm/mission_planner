#ifndef NODE_H
#define NODE_H

// NOTE might be worth having 'node' for generalised ros comms
  // e.g. 'n.publish(type, topic)', etc.
  // + 'nodeMission' for application-specific 'nm.sendGoal(lat, lon, orient)'

#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <map>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>

namespace mission_planner {
  // ros node and all its ros communication functions
  // TODO add a setMsgType to go between NavSatFix, PoseStamped, etc.(?)
  class Node
  {
    public:
      Node();
      Node(ros::NodeHandle *n);
      ~Node();

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

      void sendGoal(double lat, double lon, double yaw);
      void cancelGoal();
      GoalState getStatus() const;
      double getRobotLat() const;
      double getRobotLon() const;
      double getRobotYaw() const;
      bool goalAchieved();
      bool goalStillActive();
      int update();
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
