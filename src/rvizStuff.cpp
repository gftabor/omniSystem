#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "omniSystem/OmniRotationCommand.h"

int currentMag = -1;
ros::Publisher marker_pub;
void magCommandCallback(const omniSystem::OmniRotationCommand::ConstPtr& msg)
{
  currentMag = msg->magnetNumber;
  visualization_msgs::Marker line_strip;
  line_strip.header.stamp =  msg->header.stamp;
  line_strip.header.frame_id =  msg->rotationAxis.header.frame_id;

  
  line_strip.ns ="points_and_lines";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;


  line_strip.pose.orientation.w =  1.0;
  line_strip.id = 10;

  line_strip.scale.x = 0.1;
  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;


  geometry_msgs::Point p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  line_strip.points.push_back(p);


  geometry_msgs::Point p2;
  p2.x = msg->rotationAxis.vector.x;
  p2.y = msg->rotationAxis.vector.y;
  p2.z = msg->rotationAxis.vector.z;
  line_strip.points.push_back(p2);
  marker_pub.publish(line_strip);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(10);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Subscriber omni1Sub = n.subscribe("omniMagnet", 1000, magCommandCallback);


  while (ros::ok())
  {
    for(int i =0;i<7;i++){
      visualization_msgs::Marker marker;
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.


      marker.header.stamp = ros::Time::now();
      marker.ns = "omniSystem";
      marker.id = i;
      ROS_INFO("frame is %d",i);
      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type =  visualization_msgs::Marker::CUBE;;

      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.125;
      marker.scale.y = 0.125;
      marker.scale.z = 0.125;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.20;

      switch (i)
      {
      case 0:
        marker.header.frame_id = "/omni0";
        break;
      case 1:
        marker.header.frame_id = "/omni1";
        break;
      case 2:
        marker.header.frame_id = "/omni2";
        break;
      case 3:
        marker.header.frame_id = "/omni3";
        break;
      case 4:
        marker.header.frame_id = "/omni4";
        break;
      case 5:
        marker.header.frame_id = "/center";
        marker.scale.x = 0.62;
        marker.scale.y = 0.38;
        marker.scale.z = 0.125;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        break;
      case 6:
        marker.header.frame_id = "/object";
        marker.type =  visualization_msgs::Marker::SPHERE;
        marker.color.r = 0.722f;
        marker.color.g = 0.451f;
        marker.color.b = 0.20f;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 0.80;

        break;
      } 
      if(i ==currentMag){
        marker.color.a = 0.60;
        ROS_INFO("mag %d",i);
      }
      marker.lifetime = ros::Duration();

      marker_pub.publish(marker);
    }

    r.sleep();
    ros::spinOnce();
  }
}

