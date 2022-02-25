#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "omniSystem/OmniRotationCommand.h"

ros::Publisher marker_pub;
ros::Time lastTime; 
void magCommandCallback(const omniSystem::OmniRotationCommand::ConstPtr& msg)
{

  lastTime = msg->header.stamp;


}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "center_line");
  ros::NodeHandle n;
  ros::Rate r(10);
  marker_pub = n.advertise<visualization_msgs::Marker>("center_line_marker", 10);
  ros::Subscriber omni1Sub = n.subscribe("omniMagnet", 10, magCommandCallback);

  ros::Duration(2.5).sleep(); 
  lastTime = ros::Time::now();

  while (ros::ok())
  {
    visualization_msgs::Marker marker;

    marker.header.stamp = lastTime;
    marker.ns = "center_line";
   
    marker.type =  visualization_msgs::Marker::POINTS;;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "center";
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
 
    marker.id = 0;
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    //marker.scale.z = 0.05;
    marker.color.a = 0.70;


    float x = 0,z=0;
    float y_start = -0.2;
    float y_end = 0.2;
    int steps = 20;

    for(int i=0;i<steps;i++){
      geometry_msgs::Point p;
      float y = ((steps-i)*y_start + i*y_end)/steps;
      p.x = x;
      p.y = y;
      p.z = z;
  
      marker.points.push_back(p);
    }




    marker.lifetime = ros::Duration();

    marker_pub.publish(marker);

    r.sleep();  
    ros::spinOnce();

  }
}

