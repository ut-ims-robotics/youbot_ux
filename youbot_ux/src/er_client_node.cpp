#include "ros/ros.h"
#include "temoto_er_manager/temoto_er_manager_interface.h"
#include <std_msgs/String.h>

std_msgs::String currentState;
std_msgs::String lastState;

currentState = "safeMode";
lastState = "safeMode";

void stateCallback(const std_msgs::StringConstPtr& str) 
{
  currentState = str;

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
  temotoErManager::ERManagerInterface ermi(true);
  ros::Subscriber stateSub = nh.subscribe("state", 1, stateCallback);

  /*
   * Load a ROS program an example of a ROS executable (regularly invoked via 'rosrun'). The first
   * parameter indicates the ROS package name and the second indicates the executable. Additional
   * arguments can also be passed as a third std::string variable. The same method can be used to
   * load ROS launch files
   */

  if (currentState != lastState) {
    if (currentState == "Driving") {
      ermi.loadRosResource("youbot_ux", "youbot_drive_joy");
    }
    else if (currentState == "ManipulatorPerJoint") {
      ermi.loadRosResource("youbot_ux", "velocity_move_youbot");
      ermi.loadRosResource("youbot_ux", "velocity_move_youbot_grasp");
    }
    else if (currentState == "TrajectoryRecord") {
      ermi.loadRosResource("youbot_ux", "youbot_trajectory_record");
    }
    lastState = currentState;
  }

  
  /*
   * Note that this time the "unloadResource" was not invoked, as the destructor of "ermi" automatically
   * unloads all loaded resources.
   */ 
  return 0;
}
