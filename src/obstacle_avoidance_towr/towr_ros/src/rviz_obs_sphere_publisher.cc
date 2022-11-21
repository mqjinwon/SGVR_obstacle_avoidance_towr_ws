#include <towr_ros/rviz_obs_sphere_publisher.h>
#include <xpp_msgs/topic_names.h>

namespace SGVR {

ObsVisualizer::ObsVisualizer(ros::NodeHandle& nh)
{
  sub_ = nh.subscribe(xpp_msgs::obstacle_info, 1, &ObsVisualizer::ObstacleCallback, this);
  rviz_pub = nh.advertise<visualization_msgs::MarkerArray>("sgvr/obstacle", 1);
}

ObsVisualizer::~ObsVisualizer()
{
}

void ObsVisualizer::ObstacleCallback(const xpp_msgs::SphereObstacle& msg_in){
  
  visualization_msgs::MarkerArray msg;
  visualization_msgs::Marker m;
  
  int id = 0;
  m.type = visualization_msgs::Marker::SPHERE;
  m.ns = "obstacle";
  m.color.a = 0.6;

  for(int idx=0; idx<msg_in.radius.size(); idx++){
    
    m.header.frame_id = msg_in.frame_id;
    m.pose.position.x = msg_in.center[idx].x;
    m.pose.position.y = msg_in.center[idx].y;
    m.pose.position.z = msg_in.center[idx].z;
    m.scale.x = msg_in.radius[idx]*2;
    m.scale.y = msg_in.radius[idx]*2;
    m.scale.z = msg_in.radius[idx]*2;

    if(msg_in.collisionState[idx] == true){
      m.color.r = 1; m.color.g  = 0; m.color.b  = 0; // red
    }
    else{
      // m.color.r = 0.48235; m.color.g  = 0.0117; m.color.b  = 0.988235; // purple
      m.color.r = 0.96484; m.color.g  = 0.95312; m.color.b  = 0.09765; // yellow
    }

    m.id = id++;
    msg.markers.push_back(m);
  }

  rviz_pub.publish(msg);
}

} // namespace SGVR

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rviz_obstacle_visualizer");

  ros::NodeHandle nh;

  SGVR::ObsVisualizer obs_visualizer(nh);

  ros::spin();

  return 1;
}
