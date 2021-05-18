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
  //cout << "SUBSCRIBE";
  //cout << str->data;
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
  ros::Duration(2).sleep();
  //ROS_INFO("sub load");
  ros::Subscriber stateSub = nh.subscribe("state", 1, stateCallback);
  //ROS_INFO("sub loaded");
  ros::Rate r(10); // 10 hz

  //ROS_INFO("ROS ok?");
  while(!(ros::ok())) {
    r.sleep();
  }
  //ROS_INFO("ROS ok");
  // create variables for all loadable/unloadable resources
  temoto_er_manager::LoadExtResource load_resource_msg_drive;
  temoto_er_manager::LoadExtResource load_resource_msg_velocity;
  temoto_er_manager::LoadExtResource load_resource_msg_velocity_grasp;
  temoto_er_manager::LoadExtResource load_resource_msg_youbot_trajectory_record;
  temoto_er_manager::LoadExtResource load_resource_msg_trajectory_replayer;
  
  // block for loading and unloading of nodes
  while(ros::ok())
  {
    //cout << "got to the start";
    //cout << currentState;
    if (currentState != lastState) {
      if (currentState == "driving") {
	//cout << "DRIVING";
        load_resource_msg_drive = ermi.loadRosResource("youbot_ux", "youbot_drive_joy.py");
        ermi.unloadResource(load_resource_msg_velocity);
        ermi.unloadResource(load_resource_msg_velocity_grasp);
        ermi.unloadResource(load_resource_msg_youbot_trajectory_record);
	ermi.unloadResource(load_resource_msg_trajectory_replayer);
      }
      else if (currentState == "manipulatorPerJoint") {
	//cout << "MANIPULATORPERJOINT";
        load_resource_msg_velocity = ermi.loadRosResource("youbot_ux", "youbot_velocity_move.py");
        load_resource_msg_velocity_grasp = ermi.loadRosResource("youbot_ux", "youbot_velocity_move_grasp.py");
        ermi.unloadResource(load_resource_msg_drive);
        ermi.unloadResource(load_resource_msg_youbot_trajectory_record);
	ermi.unloadResource(load_resource_msg_trajectory_replayer);
      }
      else if (currentState == "trajectoryRecord") {
	//cout << "TRAJECTORYRECORD";
        load_resource_msg_youbot_trajectory_record = ermi.loadRosResource("youbot_ux", "youbot_trajectory_record.py");
	load_resource_msg_trajectory_replayer = ermi.loadRosResource("trajectory_replayer", "trajectory_replayer_node");
        ermi.unloadResource(load_resource_msg_drive);
        ermi.unloadResource(load_resource_msg_velocity);
        //ermi.unloadResource(load_resource_msg_velocity_grasp);
      }
      else if (currentState == "safeMode") {
	//cout << "SAFEMODE";
        ermi.unloadResource(load_resource_msg_drive);
        ermi.unloadResource(load_resource_msg_velocity);
        ermi.unloadResource(load_resource_msg_velocity_grasp);
        ermi.unloadResource(load_resource_msg_youbot_trajectory_record);
	ermi.unloadResource(load_resource_msg_trajectory_replayer);
      }
      lastState = currentState;
      
      //cout << "got to the end";
    }
    r.sleep();
    ros::spinOnce();
    //cout << "without";
  }

  return 0;
}
