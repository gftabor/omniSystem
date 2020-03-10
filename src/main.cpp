#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <vector>
#include "OmniMagnetProjectCameraTracking/omnimagnet.hpp"
#include "OmniMagnetProjectCameraTracking/cameratrack.hpp"
#include <iostream>
#include <fstream>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;




 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Setup for the D2A card for Omni system: --- DON'T MODIFIY ---

    //opening the D2A card:
    int subdev = 0;     /* change this to your input subdevice */
    int chan = 10;      /* change this to your channel */
    int range = 16383;      /* more on this later */
    int aref = AREF_GROUND; /* more on this later */
    unsigned int subdevice = 0;
    int res;
    comedi_t *D2A;
    D2A = comedi_open("/dev/comedi0");
        if(D2A == NULL) {
        printf("did not work");
        comedi_perror("comedi_open");
        return 1;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Activating the amplifiers inhibits. Pins are 25&26   --- DON'T MODIFIY ---
    int retval;
    retval = comedi_data_write(D2A, subdev, 25, 0, AREF_GROUND, 16383.0*3/4);
    retval = comedi_data_write(D2A, subdev, 26, 0, AREF_GROUND, 16383.0*3/4);
    int rrr;


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Setup for the Cameras: --- DON'T MODIFIY ---
    SystemPtr system = System::GetInstance();
    
    // Retrieve list of cameras from the system
    std::cout << "\nGetting Camera(s)\n";

    CameraList camList = system->GetCameras();
    const unsigned int numCameras = camList.GetSize();
    std::cout << "Number of cameras detected: " << numCameras << std::endl;
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();
        // Release system
        system->ReleaseInstance();
        std::cout << "Not enough cameras!" << std::endl;
        std::cout << "Done! Press Enter to exit..." << std::endl;
        getchar();
        return -1;
    }




    /////////////////////////////////////////////////////////////////////////////////
    // Defining the cameras
    std::cout << "Setup target tracking";
    Eigen::Vector3d targetPose;

    CameraTracker camTrack1(camList.GetByIndex(0), targetPose, 1, 30.0, "/home/ashkan/catkin_ws/src/omniSystem/src/camera_calibration.xml"); //Defines a tracker with a given camera * and 3 IDs for coordinate frame and an ID for the target and address to the calibration file,  
    cv::Mat Image = camTrack1.GetCurrentFrame();
    for (int i = 0; i<30; i++){
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




  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {
    std::cout << "\nTarget Position:\n" << camTrack1.GetTargetLocation() << "\n"; // print target location 
    std::cout << "\nTarget Orientation:\n" << camTrack1.GetTargetOrientaion() << "\n"; // print target location 



    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;

    msg.data = ss.str();
    //ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}