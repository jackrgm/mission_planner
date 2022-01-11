// TODO check best-practice correct ordering of header files (system vs etc)
#include "geometry_msgs/PoseStamped.h"
#include "mission_planner/node.h"
#include <iostream>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

namespace mission_planner {
  Node::Node()
  {
    this->nh = ros::NodeHandle();
    this->setup();
  }

  Node::Node(ros::NodeHandle *n)
  {
    this->nh = *n;
    this->setup();
  }

  Node::~Node()
  {
    ROS_DEBUG("Destroying node...");
  }

  // TODO implement getRobotLat
  double Node::getRobotLat() const
  {
    return pos_lat;
  }

  // TODO implement getRobotLon
  double Node::getRobotLon() const
  {
    return pos_lon;
  }

  double Node::getRobotYaw() const
  {
    return pos_yaw;
  }

  Node::GoalState Node::getStatus() const
  {
    return status;
  }

  bool Node::goalAchieved()
  {
    if(status == SUCCEEDED)
    {
      status = PENDING;
      return true;
    }
    else
    {
      return false;
    }
  }

  bool Node::setup()
  {
    ROS_DEBUG("Starting node...");

    topic_cancel = "/move_base/cancel";
    topic_frameid = "/odom";
    topic_odometry = "/odometry/filtered";
    topic_goal = "/gps_goal_pose";

    topic_pos = "/navsat/fix";
    topic_status = "/move_base/status";

    pub_cancel = nh.advertise<actionlib_msgs::GoalID>(topic_cancel, 5, true);
    pub_goal = nh.advertise<geometry_msgs::PoseStamped>(topic_goal, 5);

    sub_pos = nh.subscribe(topic_pos, 1, &Node::callbackPos, this);
    sub_status = nh.subscribe(topic_status, 1, &Node::callbackStatus, this);
    sub_yaw = nh.subscribe(topic_odometry, 1, &Node::callbackOdometry, this);

    ROS_DEBUG("Node online");
  }

  void Node::sendGoal(double lat, double lon, double yaw)
  {
    geometry_msgs::Quaternion quat;
    tf2::Quaternion quat_tf2;
    geometry_msgs::PoseStamped msg_pstamp;

    status = PENDING;

    ROS_DEBUG("Converting yaw to quaternion...");
    quat_tf2.setRPY(0, 0, yaw);
    quat_tf2.normalize();
    quat = tf2::toMsg(quat_tf2);

    msg_pstamp.header.frame_id = topic_frameid;
    msg_pstamp.pose.position.x = lat;
    msg_pstamp.pose.position.y = lon;
    msg_pstamp.pose.position.z = 0;

    msg_pstamp.pose.orientation.x = quat.x;
    msg_pstamp.pose.orientation.y = quat.y;
    msg_pstamp.pose.orientation.z = quat.z;
    msg_pstamp.pose.orientation.w = quat.w;

    ROS_DEBUG("Publishing goal...");
    this->pub_goal.publish(msg_pstamp);
  }

  // TODO verify navigation status is updated (to 'cancelled' or whatever)
  void Node::cancelGoal()
  {
    status = ABORTED;
    actionlib_msgs::GoalID msg_cancel;
    msg_cancel.id = {};
    pub_cancel.publish(msg_cancel);
  }

  void Node::callbackPos(const sensor_msgs::NavSatFix &msg_pos)
  {
    pos_lat = msg_pos.latitude;
    pos_lon = msg_pos.longitude;
    ROS_DEBUG("msg_pos callback triggered, received robot's reported position");
  }

  // TODO algorithm probably needs optimising (read up on msgs types etc)
  void Node::callbackStatus(const actionlib_msgs::GoalStatusArray::ConstPtr &msg_status)
  {
    if (!msg_status->status_list.empty())
    {
      actionlib_msgs::GoalStatus gstat = msg_status->status_list[0];
      status = static_cast<GoalState>(gstat.status);
    }

    ROS_DEBUG("Callback triggered, received new goal status");
    return;
  }

  void Node::callbackOdometry(const nav_msgs::Odometry &msg_odom)
  {
    double roll, pitch, yaw;
    tf::Quaternion quat(msg_odom.pose.pose.orientation.x,
      msg_odom.pose.pose.orientation.y, msg_odom.pose.pose.orientation.z,
      msg_odom.pose.pose.orientation.w);

    tf::Matrix3x3 matr(quat);
    matr.getRPY(roll, pitch, yaw);

    pos_yaw = yaw;

    robot_velocity = msg_odom.twist.twist.linear.x;

    ROS_DEBUG("Callback triggered, received robot's reported yaw");
    return;
  }

  bool Node::isMoving()
  {
    // TODO magic numbers; they would be better as member variables
      // values seem like a reasonable definition of 'stopped' with some margin
    if (robot_velocity < -0.05 || robot_velocity > 0.05)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  int Node::update()
  {
    ros::spinOnce();
    return 0;
  }
}
