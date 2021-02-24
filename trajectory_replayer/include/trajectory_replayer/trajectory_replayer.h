#ifndef TRAJECTORY_REPLAYER__TRAJECTORY_REPLAYER_H
#define TRAJECTORY_REPLAYER__TRAJECTORY_REPLAYER_H

#include <string>
#include <vector>
#include <mutex>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_replayer/SendHackedTrajectory.h"

namespace trajectory_replayer
{

class TrajectoryReplayer
{
public:


  TrajectoryReplayer(const std::string& joint_states_topic, float frequency = 20, bool enable_services = true);

private:



  bool sendHackedTrajectoryCb(SendHackedTrajectory::Request& req
  , SendHackedTrajectory::Response& res);

  ros::NodeHandle nh_;
  ros::ServiceServer trajectory_playback_server_;

};

} // trajectory_replayer namespace
#endif