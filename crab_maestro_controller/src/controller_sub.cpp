#include "controller_sub.hpp"

	const int rotation_direction[18] = 	{1,-1, 1,
								 	 	 1,-1, 1,
								 	 	 1,-1, 1,
								 	 	 1, 1,-1,
								 	 	 1, 1,-1,
								 	 	 1, 1,-1};


Controller::Controller(){

  // Get Min and Max joints limits
	node.param("joint_lower_limit", joint_lower_limit, -1.570796327);
	node.param("joint_upper_limit", joint_upper_limit, 1.570796327);
	limit_coef = 127 / ((joint_upper_limit - joint_lower_limit) / 2);

	// Get the interface to connect Erle-Brain with Mini-Maestro
	node.param("port_name", port_name, std::string("/dev/ttyACM0"));

	//LEG 1
	node.param("leg1_1", channels[0], 0);
	node.param("leg1_2", channels[1], 1);
	node.param("leg1_3", channels[2], 2);

	//LEG 2
	node.param("leg2_1", channels[3], 3);
	node.param("leg2_2", channels[4], 4);
	node.param("leg2_3", channels[5], 5);

	//LEG 3
	node.param("leg3_1", channels[6], 6);
	node.param("leg3_2", channels[7], 7);
	node.param("leg3_3", channels[8], 8);

	//LEG 4
	node.param("leg4_1", channels[9], 9);
	node.param("leg4_2", channels[10], 10);
	node.param("leg4_3", channels[11], 11);

	//LEG 5
	node.param("leg5_1", channels[12], 12);
	node.param("leg5_2", channels[13], 13);
	node.param("leg5_3", channels[14], 14);

	//LEG 6
	node.param("leg6_1", channels[15], 15);
	node.param("leg6_2", channels[16], 16);
	node.param("leg6_3", channels[17], 17);
	

	maestro = Polstro::SerialInterface::createSerialInterface(port_name, 115200);
	gait_control_sub = node.subscribe<crab_msgs::GaitCommand>("/teleop/gait_control", 1, &Controller::teleopGaitCtrl, this);
	sub = node.subscribe("joints_to_controller", 1, &Controller::chatterLegsState, this);
	ROS_INFO("Maestro servo controller is ready...");
}

void Controller::init() {
	//gait_control_sub = node.subscribe<crab_msgs::GaitCommand>("/teleop/gait_control", 1, &Controller::teleopGaitCtrl, this);
}

void Controller::chatterLegsState (const crab_msgs::LegsJointsStateConstPtr &legs_jnts){

	float target_value;
	int s_num;

	if (gait_command.cmd == gait_command.BACKLEGBROKEN){
		for (int i=0; i<num_legs; i++){
			for (int j=0; j<num_joints; j++){
				s_num = i*3+j;
				
				//Pata rota
				if(i==5 && j==0){target_value = 127;}
				else if(i==5 && j==1){target_value = 127 + rotation_direction[s_num] * 7 * limit_coef;}
				else if(i==5 && j==2){target_value = 127;}
				//Resto
				else{target_value = 127 + rotation_direction[s_num] * legs_jnts->joints_state[i].joint[j] * limit_coef;}

				maestro->setTargetMSS(channels[s_num], (unsigned char) target_value);
			}
		}
	}
	else if (gait_command.cmd == gait_command.MIDDLELEGBROKEN){
		for (int i=0; i<num_legs; i++){
			for (int j=0; j<num_joints; j++){
				s_num = i*3+j;
				
				//Pata rota
				if(i==1 && j==0){target_value = 127;}
				else if(i==1 && j==1){target_value = 127 + rotation_direction[s_num] * 2 * limit_coef;}
				else if(i==1 && j==2){target_value = 127;}
				//Coxa de las patas de la parte que tiene rota la pata
				else if(i==0 && j==0){target_value = 127 + rotation_direction[s_num] * legs_jnts->joints_state[0].joint[0] * limit_coef * 1.4;}
				else if(i==2 && j==0){target_value = 127 + rotation_direction[s_num] * legs_jnts->joints_state[2].joint[0] * limit_coef * 1.4;}
				//Resto
				else{target_value = 127 + rotation_direction[s_num] * legs_jnts->joints_state[i].joint[j] * limit_coef;}

				maestro->setTargetMSS(channels[s_num], (unsigned char) target_value);
			}
		}
	}
	else if (gait_command.cmd == gait_command.FOURLEGMOVEMENT){
		for (int i=0; i<num_legs; i++){
			for (int j=0; j<num_joints; j++){
				s_num = i*3+j;
					
				//Patas rotas
				if(i==2 && j==0){target_value = 127;}
				//else if(i==2 && j==1){target_value = 127 + rotation_direction[s_num] * 2 * limit_coef;}
				else if(i==2 && j==1){target_value = 90;}
				else if(i==2 && j==2){target_value = 127;}
				else if(i==5 && j==0){target_value = 127;}
				//else if(i==5 && j==1){target_value = 127 + rotation_direction[s_num] * 7 * limit_coef;}
				else if(i==5 && j==1){target_value = 164;}
				else if(i==5 && j==2){target_value = 127;}
				//Coxa de las patas del medio
				else if(i==1 && j==0){target_value = 194 + rotation_direction[s_num] * legs_jnts->joints_state[i].joint[j] * limit_coef;}
				else if(i==4 && j==0){target_value = 60  + rotation_direction[s_num] * legs_jnts->joints_state[i].joint[j] * limit_coef;}
				//Resto
				else{target_value = 127 + rotation_direction[s_num] * legs_jnts->joints_state[i].joint[j] * limit_coef;}

				maestro->setTargetMSS(channels[s_num], (unsigned char) target_value);
			}
		}
	}
	else if (gait_command.cmd == gait_command.RUNSINGLE || gait_command.cmd == gait_command.RUNRIPPLE || gait_command.cmd == gait_command.RUNTRIPOD){
		for (int i=0; i<num_legs; i++){
			for (int j=0; j<num_joints; j++){
				s_num = i*3+j;
				target_value = 127 + rotation_direction[s_num] * legs_jnts->joints_state[i].joint[j] * limit_coef;

				maestro->setTargetMSS(channels[s_num], (unsigned char) target_value);
			}
		}
	}
	else{
		for (int i=0; i<num_legs; i++){
			for (int j=0; j<num_joints; j++){
				s_num = i*3+j;
				target_value = 127 + rotation_direction[s_num] * legs_jnts->joints_state[i].joint[j] * limit_coef;

				maestro->setTargetMSS(channels[s_num], (unsigned char) target_value);
			}
		}
	}
}

void Controller::teleopGaitCtrl (const crab_msgs::GaitCommandConstPtr &gait_cmd){
	gait_command.cmd = gait_cmd->cmd;
}

//void Controller::gaitGenerator (){


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_sub");
    Controller c;
    ros::spin();
}







