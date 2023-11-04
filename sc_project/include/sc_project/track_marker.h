#ifndef TRACK_MARKER_H
#define TRACK_MARKER_H

#include "ros/ros.h"

#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2_msgs/TFMessage.h"
#include <chrono>
#include <vector>

class CustomRobotFollower
{
public:
    TrackMarker(ros::NodeHandle nh);
    ~TrackMarker();

    void markerDetectionCallback(const geometry_msgs::Vector3StampedPtr &msg);
    void stopFollowing();

private:
    ros::NodeHandle nh_;

    ros::Subscriber marker_detection_sub_;
    ros::Publisher velocity_pub_;
    geometry_msgs::Twist twist_message_;
    ros::Subscriber pose_tracker_;

    double marker_readings_;
    bool obstacle_detected_;
    bool obstacle_reported_;
    bool search_reported_;
    bool sweep_complete_;
    tf2_msgs::TFMessageConstPtr pose_fetch_;

    struct Guider
    {
        geometry_msgs::Vector3Stamped pose;
        double detection_threshold;
        double closest_distance;
        bool detected;
        bool reached;
    };
    Guider guider_;

    ros::Time start_time_;   //!< start time
    ros::Duration duration_; //!< duration since start time (seconds)
};

#endif // TRACK_MARKER_H
