#include "ros/ros.h"
#include "temoto_er_manager/temoto_er_manager_interface.h"
#include <std_msgs/String.h>
#include <string>

std::string currentState = "safeMode";
std::string lastState = "safeMode";

void stateCallback(const std_msgs::StringConstPtr& str) 
{
  currentState = str -> data.c_str();

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

  /*
   * Load a ROS program an example of a ROS executable (regularly invoked via 'rosrun'). The first
   * parameter indicates the ROS package name and the second indicates the executable. Additional
   * arguments can also be passed as a third std::string variable. The same method can be used to
   * load ROS launch files
   */
  ROS_INFO("Load youbot_drive_joy");
  ermi.loadRosResource("youbot_ux", "youbot_drive_joy.py");
  //temoto_er_manager::LoadExtResource load_resource_msg_drive = ermi.loadRosResource("youbot_ux", "youbot_drive_joy");
  ROS_INFO("Load velocity_move_youbot");
  temoto_er_manager::LoadExtResource load_resource_msg_velocity = ermi.loadRosResource("youbot_ux", "velocity_move_youbot.py");
  ROS_INFO("Load velocity_move_youbot_grasp");
  temoto_er_manager::LoadExtResource load_resource_msg_velocity_grasp = ermi.loadRosResource("youbot_ux", "velocity_move_youbot_grasp.py");
  ROS_INFO("Load youbot_trajectory_record");
  temoto_er_manager::LoadExtResource load_resource_msg_trajectory_record = ermi.loadRosResource("youbot_ux", "youbot_trajectory_record.py");

  /*
  if (currentState != lastState) {
    if (currentState == "Driving") {
      ermi.loadRosResource("youbot_ux", "youbot_drive_joy");
      ermi.unloadResource(load_resource_msg_velocity);
      ermi.unloadResource(load_resource_msg_velocity_grasp);
      ermi.unloadResource(load_resource_msg_trajectory_record);
    }
    else if (currentState == "ManipulatorPerJoint") {
      ermi.loadRosResource("youbot_ux", "velocity_move_youbot");
      ermi.loadRosResource("youbot_ux", "velocity_move_youbot_grasp");
      ermi.unloadResource(load_resource_msg_drive);
      ermi.unloadResource(load_resource_msg_trajectory_record);
    }
    else if (currentState == "TrajectoryRecord") {
      ermi.loadRosResource("youbot_ux", "youbot_trajectory_record");
      ermi.unloadResource(load_resource_msg_drive);
      ermi.unloadResource(load_resource_msg_velocity);
      ermi.unloadResource(load_resource_msg_velocity_grasp);
    }
    lastState = currentState;
  }

  */
  /*
   * Note that this time the "unloadResource" was not invoked, as the destructor of "ermi" automatically
   * unloads all loaded resources.
   */ 
  while (true);
  return 0;
}
