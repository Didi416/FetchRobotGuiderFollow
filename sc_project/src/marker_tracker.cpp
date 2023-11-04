#include "../include/sc_project/track_marker.h"

TrackMarker::TrackMarker(ros::NodeHandle nh)
    : nh_(nh)
{
  marker_sub_ = nh_.subscribe("/aruco_single/position", 1000, &TrackMarker::markerCallback, this);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  marker_.threshold_distance = 1.0 - marker_.head_to_base_offset;
  ROS_INFO_STREAM("init");

  duration_ = elapsed_time_ - elapsed_time_;

  sweep_complete_ = false;
  obstacle_reported_ = false;
  search_reported_ = false;
}

TrackMarker::~TrackMarker()
{
}

void TrackMarker::markerCallback(const geometry_msgs::Vector3StampedPtr &msg)
{
  if (!marker_.detected)
  {
    marker_.detected = true;
    search_reported_ = false;
    ROS_INFO_STREAM("marker Detected!");
    sweep_complete_ = false;
  }

  marker_.pose.vector.x = msg->vector.z;
  marker_.pose.vector.y = msg->vector.x;
  marker_.pose.vector.z = msg->vector.y;

  marker_.shortest_distance = roundf64(sqrt(pow(marker_.pose.vector.x, 2) + pow(marker_.pose.vector.y, 2)) * 10) / 10;

  if (marker_.shortest_distance <= (marker_.threshold_distance + 0.01) && marker_.shortest_distance >= (marker_.threshold_distance - 0.01))
  {
    twist_message_.linear.x = 0;
    twist_message_.angular.z = 0;
    if (!marker_.reached)
    {
      ROS_INFO_STREAM("marker is stationary");
      marker_.reached = true;
    }
  }
  else
  {
    marker_.reached = false;
    if (!obstacle_detected_)
    {
      double error = marker_.shortest_distance - marker_.threshold_distance;

      if (marker_.shortest_distance > marker_.threshold_distance * 1.5)
      {
        twist_message_.linear.x = error;
      }
      else if (marker_.shortest_distance < marker_.threshold_distance * 0.95)
      {
        twist_message_.linear.x = -0.3;
      }
      else
      {
        twist_message_.linear.x = error / 2;
      }

      if (marker_.pose.vector.y == 0)
      {
        twist_message_.angular.z = 0;
      }
      else
      {
        twist_message_.angular.z = -marker_.pose.vector.y;
      }
      obstacle_reported_ = false;
    }
  }
  vel_pub_.publish(twist_message_);

  elapsed_time_ = ros::Time::now();
}

// Replace laserCallBack function with the appropriate logic for your context.

void TrackMarker::stop()
{
  while (ros::ok)
  {
    if (duration_ > ros::Duration(20.0))
    {
      elapsed_time_ = ros::Time::now();
    }

    duration_ = ros::Time::now() - elapsed_time_;

    if (duration_ >= ros::Duration(2.0) && duration_ <= ros::Duration(8.0) && !marker_.detected && !sweep_complete_)
    {
      twist_message_.angular.z = -1.57;
      twist_message_.linear.x = 0.0;
      vel_pub_.publish(twist_message_);
      if (!search_reported_)
      {
        ROS_INFO_STREAM("Searching for marker");
        search_reported_ = true;
      }
    }

    if (duration_ > ros::Duration(8.0) && duration_ <= ros::Duration(14.0) && !marker_.detected && !sweep_complete_)
    {
      twist_message_.angular.z = 1.57;
      twist_message_.linear.x = 0.0;
      vel_pub_.publish(twist_message_);
    }

    if (duration_ == ros::Duration(14.0) && !marker_.detected && !sweep_complete_)
    {
      sweep_complete_ = true;
      search_reported_ = false;
      ROS_INFO_STREAM("No marker detected. Cease searching");
    }

    if (duration_ >= ros::Duration(3.0) && marker_.detected)
    {
      ROS_INFO_STREAM("No marker detected. Cease search and operation.");

      marker.detected = false;
      sweep_complete_ = false;
      elapsed_time_ = ros::Time::now();
    }
  }
}
