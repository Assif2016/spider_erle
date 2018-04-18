
#ifndef CONTROLLER_SUB_HPP_
#define CONTROLLER_SUB_HPP_


#include <ros/ros.h>
#include <crab_msgs/LegsJointsState.h>
#include "PolstroSerialInterface.h"
#include <crab_msgs/GaitCommand.h>

#define N_CHANNELS 18

class Controller {
	public:
		Controller();
		void init();

	private:

		ros::NodeHandle node;
		std::string port_name;
		Polstro::SerialInterface* maestro;
		crab_msgs::GaitCommand gait_command;
		double joint_lower_limit, joint_upper_limit, limit_coef;
		const static unsigned int num_joints = 3;
		const static unsigned int num_legs = 6;

		int channels[N_CHANNELS];

		ros::Subscriber sub;
		ros::Subscriber gait_control_sub;

		void chatterLegsState (const crab_msgs::LegsJointsStateConstPtr &legs_jnts);
		void teleopGaitCtrl (const crab_msgs::GaitCommandConstPtr &gait_cmd);
};

#endif /* CONTROLLER_SUB_HPP_ */
