#include "trajectory_replayer/trajectory_replayer.h"
#include "iostream"
#include "std_srvs/Empty.h"
#include "trajectory_recorder/trajectory_recorder.h"

#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include "brics_actuator/CartesianWrench.h"

#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>

#include <ros/console.h>

ros::Publisher armPositionsPublisher;


namespace trajectory_replayer
{

TrajectoryReplayer::TrajectoryReplayer(const std::string& joint_states_topic, float frequency, bool enable_services)

{

  /*
   * Enable the ROS servers to control the trajectory recorder
   */
  if (enable_services)
  {
    trajectory_playback_server_ = nh_.advertiseService("trajectory_playback"
    , &TrajectoryReplayer::sendHackedTrajectoryCb
    , this);
  } 
  ROS_INFO_STREAM("The trajectory replayer is initialized");
}

using namespace std;
//void sendHackedTrajectory(const trajectory_msgs::JointTrajectory traj, bool joint1changed, double joint1value)
bool TrajectoryReplayer::sendHackedTrajectoryCb(SendHackedTrajectory::Request& req
, SendHackedTrajectory::Response& res)
{
	// recorded_trajectory_ may be the values and all for use
	int numberOfJoints = 7;
	bool joint1changed = false;
	double joint1value = 0.0;
	armPositionsPublisher = nh_.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 100);

	std_srvs::Empty empty;
	//trajectory_msgs::JointTrajectory traj;

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<trajectory_recorder::GetRecordedTrajectory>("get_recorded_trajectory");
	trajectory_recorder::GetRecordedTrajectory getRecordedTrajectorySrv;
	
	trajectory_msgs::JointTrajectory traj = trajectory_msgs::JointTrajectory();
	// get trajectory message from service
	if (client.call(getRecordedTrajectorySrv))
	{
		
		if (getRecordedTrajectorySrv.response.response_message == "ok") 
		{
			traj.joint_names = getRecordedTrajectorySrv.response.trajectory.joint_names; // load joint anmes and points to local variable
			traj.points = getRecordedTrajectorySrv.response.trajectory.points;
		}
		else
		{
			return false;
		}			
	}
	else 
	{
		return false;
	}

	if (traj.joint_names.size() < 5) {
		return false;
	}

	brics_actuator::JointPositions command;	
        std::vector <brics_actuator::JointValue> armJointPositions;
        armJointPositions.resize(numberOfJoints);

        ROS_INFO("** trajectory hack **");

        for (int i=0; i<numberOfJoints; i++)
        {
            armJointPositions[i].joint_uri = traj.joint_names[i];
            armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);

            if (i == 0)
            {
                if (joint1changed)
                    armJointPositions[i].value = joint1value;
                else
                    armJointPositions[i].value = traj.points[0].positions[i];
            }
            else
            {
                armJointPositions[i].value = traj.points[0].positions[i];
            }
        }

        ROS_INFO("sendHackedTrajectory: Moving to the starting point of the trajectory");
        command.positions = armJointPositions;
        armPositionsPublisher.publish(command);

        ros::Duration(1.0).sleep();

        for (int j=0; j<traj.points.size(); j++)
        {
            for (int i=0; i<numberOfJoints; i++)
            {
                if (i == 0)
                {
                    if (joint1changed)
                        armJointPositions[0].value = joint1value;
                    else
                        armJointPositions[0].value = traj.points[j].positions[0];
                }
                else
                {
                    armJointPositions[i].value = traj.points[j].positions[i];
                }
            }

            command.positions = armJointPositions;
            armPositionsPublisher.publish(command);
            ros::Duration(0.1).sleep();
        }
        ROS_INFO("sendHackedTrajectory: Finished");
	res.response_message = "sendHackedTrajectory: Finished";
	return true;

}

} // trajectory_replayer namespace
