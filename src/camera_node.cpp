#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

#include <sstream>
#include <vector>
#include "OmniMagnetProjectCameraTracking/cameratrack.hpp"
#include <iostream>
#include <fstream>
#include <tf/tf.h>
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "tracker");
  ros::NodeHandle n;
  ROS_INFO( "Started tracker" );

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Setup for the Cameras: --- DON'T MODIFIY ---
    SystemPtr system = System::GetInstance();
    
    // Retrieve list of cameras from the system
    std::cout << "\nGetting Camera(s)\n";

    CameraList camList = system->GetCameras();
    const unsigned int numCameras = camList.GetSize();
    ROS_INFO( "Number of cameras detected: %d", numCameras );

    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();
        // Release system
        system->ReleaseInstance();
        ROS_ERROR( "Not enough cameras!");

        return -1;
    }



    ROS_INFO( "Setup target tracking");

    /////////////////////////////////////////////////////////////////////////////////
    // Defining the cameras
    std::cout << "Setup target tracking";
    Eigen::Vector3d targetPose;

    CameraTracker camTrack1(camList.GetByIndex(0), targetPose, 1, 30.0, "/home/ashkan/catkin_ws/src/omniSystem/src/camera_calibration.xml"); //Defines a tracker with a given camera * and 3 IDs for coordinate frame and an ID for the target and address to the calibration file,  
    cv::Mat Image = camTrack1.GetCurrentFrame();
    for (int i = 0; i<3; i++){
      camTrack1.UpdateTargetPose();
      // std::chrono::seconds dura( 1);
      // std::this_thread::sleep_for( dura );
    }

  // Test/Example Camera function
  bool target_marker = true;     //Show target marker
  bool origin_markers = false;   //Show orgin outlines 
  camTrack1.SaveCurrentFrame("frame.jpg",target_marker,origin_markers);
  std::cout << "\nTarget Position:\n" << camTrack1.GetTargetLocation() << "\n"; // print target location 
  std::cout << "\nTarget Orientation:\n" << camTrack1.GetTargetOrientaion() << "\n"; // print target location 

  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 10);

  ros::Rate loop_rate(3);

  while (ros::ok())
  {
    geometry_msgs::PoseStamped msg;
    camTrack1.UpdateTargetPose();
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    Eigen::Vector3d p = camTrack1.GetTargetLocation() ;
    Eigen::Vector3d o = camTrack1.GetTargetOrientaion() ;
    msg.pose.position.x = p[0] / 1000.0;
    msg.pose.position.y = p[1] / 1000.0;
    msg.pose.position.z = p[2] / 1000.0;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;
    //msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (o[0],o[1],o[2]);
    pose_pub.publish(msg);
    //std::cout << "\nTarget Position:" << p[0] << "\n"; // print target location 
    //std::cout << "\nTarget orientation:" << o << "\n"; // print target location 

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}