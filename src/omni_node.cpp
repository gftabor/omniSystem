#include "ros/ros.h"

#include <sstream>
#include <vector>
#include "OmniMagnetProjectCameraTracking/omnimagnet.hpp"
#include <iostream>
#include <fstream>
#include <tf/tf.h>
#include "omniSystem/OmniRotationCommand.h"


int num_omni = 5;                   // number of omnimagnets using in system. !! REQUIRED UPDATE !!
std::vector<OmniMagnet> omni_system(num_omni); // defines 3 omnis use omni_system.SetProp(double wire_width, double wire_len_in, double wire_len_mid, double wire_len_out, double core_size, int pinin, int pinmid, int pinout,bool estimate, comedi *D2A).
// -- (EXAMPLE) Run Omnimagnet Rotation -- 
             // [ms] run time of demo 
Eigen::Vector3d dipole_axis;        // dipole inital vector position
Eigen::Vector3d rotation_axis;      // rotation axis 

void magCommandCallback(const omniSystem::OmniRotationCommand::ConstPtr& msg)
{
  int magnet = msg->magnetNumber;
  dipole_axis << msg->dipoleAxis[0],msg->dipoleAxis[1], msg->dipoleAxis[2];
  rotation_axis << msg->rotationAxis[0],msg->rotationAxis[1], msg->rotationAxis[2];
  float dipole_strength = msg->strength;
  float run_time = msg-> runTime;
  float freq = msg->frequency;
  ROS_INFO("commanding magnet %d with power of %f",magnet,dipole_strength);
  omni_system[magnet].RotatingDipole(dipole_strength*dipole_axis, -rotation_axis, freq, run_time); // rotates the dipole aling the given axis, with the given frequency for a given duration. (dipole, axis, frequency(Hz), duration(ms))
  Eigen::Vector3d OFFCommand;
  OFFCommand << 0,0,0;
  omni_system[magnet].SetCurrent(OFFCommand); 
}
int main(int argc, char **argv)
{

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
  /////////////////////////////////////////////////////////////////////////////////
  //Defining the Omni_magnet system
  
  omni_system[0].SetProp(1.35/1000.0 , 121, 122, 132, 17, 2, 0, 18, true, D2A); omni_system[0].UpdateMapping();       // Omni #1, left upper 
  omni_system[1].SetProp(1.35/1000.0 , 121, 122, 132, 17, 3, 11, 19, true, D2A); omni_system[1].UpdateMapping();     // Omni #2, center upper 
  omni_system[2].SetProp(1.35/1000.0 , 121, 122, 132, 17, 4, 12, 20, true, D2A); omni_system[2].UpdateMapping();     // Omni #3, right upper 
  omni_system[3].SetProp(1.35/1000.0 , 121, 122, 132, 17, 5, 13, 21, true, D2A); omni_system[3].UpdateMapping();     // Omni #4, right lower 
  omni_system[4].SetProp(1.35/1000.0 , 121, 122, 132, 17, 6, 14, 22, true, D2A); omni_system[4].UpdateMapping();     // Omni #5, left lower
  // omni_system[5].SetProp(1.35/1000.0 , 121, 122, 132, 17, 7, 15, 23, true, D2A); omni_system[5].UpdateMapping();  // spare (potentially for Super Omni), DON'T TURN ON TILL CONNECTED



  ros::init(argc, argv, "omni");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Subscriber omni1Sub = n.subscribe("omniMagnet", 1000, magCommandCallback);


  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }


  //////////////////////////////////////////////////////////
  // Shutting down the system:  --- DON'T MODIFIY ---
  Eigen::Vector3d off;
  off << 0.0, 0.0, 0.0;
  for (auto &omni : omni_system) // access by reference to avoid copying
  {  
      omni.SetCurrent(off);
      std::cout <<"\nSystem Omnimagnet OFF\n";    //will repeat for each omnimagnet
  }
  std::cout << "\nZero current to all Omnimagnets\n";
  
  //Shutting down the amplifiers inhibits. Pins are 25&26 
  retval = comedi_data_write(D2A, subdev, 25, 0, AREF_GROUND, 16383.0*2/4);
  retval = comedi_data_write(D2A, subdev, 26, 0, AREF_GROUND, 16383.0*2/4);
  std::cout<<retval;
  //////////////////////////////////////////////////////////
  return 0;
}