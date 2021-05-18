#include "trajectory_replayer/trajectory_replayer.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_replayer_node");

  trajectory_replayer::TrajectoryReplayer tr("joint_states", true);

  /*
   * Set up the spinner
   */
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
