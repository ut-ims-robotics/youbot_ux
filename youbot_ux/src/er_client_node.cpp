#include "ros/ros.h"
#include "temoto_er_manager/temoto_er_manager_interface.h"
#include <std_msgs/String.h>
#include <string>
#include "iostream"

std::string currentState = "safeMode";
std::string lastState = "safeMode";

using namespace std;
void stateCallback(const std_msgs::String::ConstPtr& str) 
{
  cout << str;
  currentState = str -> data;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "er_client_node");

  /*
   * Create External Resource Manager Interface object that provides a simplified
   * API for communicating with the External Resource Manager. The boolean "true", that's passed
   * to the constructor of ERM interface tells it whether it should be initialised immediately,
   * or that's done later by the user.
   */
  ROS_INFO("ermi init");
  temoto_er_manager::ERManagerInterface ermi(true);
  ros::NodeHandle nh;
  ros::Subscriber stateSub = nh.subscribe("state", 1, stateCallback);

  // load all required nodes into variables for later use
  temoto_er_manager::LoadExtResource load_resource_msg_drive;
  temoto_er_manager::LoadExtResource load_resource_msg_velocity;
  temoto_er_manager::LoadExtResource load_resource_msg_velocity_grasp;
  temoto_er_manager::LoadExtResource load_resource_msg_trajectory_record;

  while(ros::ok())
  {
    //cout << currentState;
    if (currentState != lastState) {
      if (currentState == "driving") {
        temoto_er_manager::LoadExtResource load_resource_msg_drive = ermi.loadRosResource("youbot_ux", "youbot_drive_joy.py");
        ermi.unloadResource(load_resource_msg_velocity);
        ermi.unloadResource(load_resource_msg_velocity_grasp);
        ermi.unloadResource(load_resource_msg_trajectory_record);
      }
      else if (currentState == "manipulatorPerJoint") {
        temoto_er_manager::LoadExtResource load_resource_msg_velocity = ermi.loadRosResource("youbot_ux", "youbot_velocity_move.py");
        temoto_er_manager::LoadExtResource load_resource_msg_velocity_grasp = ermi.loadRosResource("youbot_ux", "youbot_velocity_move_grasp.py");
        ermi.unloadResource(load_resource_msg_drive);
        ermi.unloadResource(load_resource_msg_trajectory_record);
      }
      else if (currentState == "trajectoryRecord") {
        temoto_er_manager::LoadExtResource load_resource_msg_trajectory_record = ermi.loadRosResource("youbot_ux", "youbot_trajectory_record.py");
        ermi.unloadResource(load_resource_msg_drive);
        ermi.unloadResource(load_resource_msg_velocity);
        ermi.unloadResource(load_resource_msg_velocity_grasp);
      }
      lastState = currentState;
    }
  }
  
  return 0;
}
