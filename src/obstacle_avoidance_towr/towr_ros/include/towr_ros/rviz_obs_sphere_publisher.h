#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

#include <xpp_msgs/SphereObstacle.h>  // sphere obstacle message

namespace SGVR{

class ObsVisualizer {
public:
    ObsVisualizer(ros::NodeHandle& nh);
    ~ObsVisualizer();

private:
    void ObstacleCallback(const xpp_msgs::SphereObstacle& msg_in);
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher  rviz_pub;
};


}