#include "gait.hpp"

Gait::Gait(){
	legs_queue.push(5);
	legs_queue.push(0);
	legs_queue.push(4);
	legs_queue.push(2);
	legs_queue.push(3);
	legs_queue.push(1);

	legs_queue_back_broken.push(0);
	legs_queue_back_broken.push(4);
	legs_queue_back_broken.push(2);
	legs_queue_back_broken.push(3);
	legs_queue_back_broken.push(1);

	legs_queue_middle_broken.push(5);
	legs_queue_middle_broken.push(0);
	legs_queue_middle_broken.push(4);
	legs_queue_middle_broken.push(2);
	legs_queue_middle_broken.push(3);

	legs_queue_fourlegmovement.push(0);
	legs_queue_fourlegmovement.push(4);
	legs_queue_fourlegmovement.push(1);
	legs_queue_fourlegmovement.push(3);
}

void Gait::setTrapezoid(double low_r, double high_r, double h, double z){
	low_rad = low_r;
	high_rad = high_r;
	height = z - h;
	z_body = z;
}

void Gait::setFi (double fi){
	//Set vectors like x = r * cos(fi), y = r * sin(fi) in 3d space
	//     B--high_rad---C
	//    /               \  <- h
	//   A-----low_rad-----D <- z

	a.p = KDL::Vector (-low_rad * cos(fi), -low_rad * sin(fi), -z_body);
	b.p = KDL::Vector (-high_rad * cos(fi), -high_rad * sin(fi), -height);
	c.p = KDL::Vector (high_rad * cos(fi), high_rad * sin(fi), -height);
	d.p = KDL::Vector (low_rad * cos(fi), low_rad * sin(fi), -z_body);
}

void Gait::setAlpha (double alpha){
	a.M = KDL::Rotation::RotZ(-alpha);
	b.M = KDL::Rotation::RotZ(-alpha/2);
	c.M = KDL::Rotation::RotZ(alpha/2);
	d.M = KDL::Rotation::RotZ(alpha);
}

void Gait::setPath (){
	path_support = new KDL::Path_Line(d, a, &rot, 0.005, true);

	path_transfer = new KDL::Path_RoundedComposite (0.02,0.005,&rot);
	path_transfer -> Add(a);
	path_transfer -> Add(b);
	path_transfer -> Add(c);
	path_transfer -> Add(d);
	path_transfer -> Finish();
}

void Gait::setTrajectory (double sup_path_duration, double tran_path_duration){
	prof_support.SetProfileDuration(0,path_support->PathLength(), sup_path_duration);
	prof_transfer.SetProfileDuration(0,path_transfer->PathLength(), tran_path_duration);
	trajectory_transfer = new KDL::Trajectory_Segment (path_transfer, &prof_transfer);
	trajectory_support  = new KDL::Trajectory_Segment (path_support, &prof_support);
}

//-------------------------------------------------------------  RUNTRIPOD  -----------------------------------------------------------

KDL::Vector* Gait::RunTripod (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration){
	setFi(fi);
	setAlpha(alpha);
	setPath();
	setTrajectory(duration, duration);
	KDL::Frame frame;
	if (run_state == false){
		run_state = true;
		phase = 0;
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;
	}
	if (pause_state == true){
		begin_sec = ros::Time::now().toSec() - passed_sec;
		pause_state = false;
	}

	passed_sec = ros::Time::now().toSec() - begin_sec;

	for (int i=phase; i<num_legs; i+=2){
		frame = trajectory_transfer -> Pos(passed_sec);
		frame.p.x(frame.p.data[0]*scale);
		frame.p.y(frame.p.data[1]*scale);
		final_vector[i] = frame.M *(*(vector_iter+i)).p + frame.p;
	}

	for (int i=!phase; i<num_legs; i+=2){
		frame = trajectory_support -> Pos(passed_sec);
		frame.p.x(frame.p.data[0]*scale);
		frame.p.y(frame.p.data[1]*scale);
		final_vector[i] = frame.M * (*(vector_iter+i)).p  + frame.p;
	}

	if (passed_sec >= duration-0.02 && run_state==true){
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;
		phase = !phase;
	}
	return final_vector;
}

//-------------------------------------------------------------  RUNRIPPLE  -----------------------------------------------------------

KDL::Vector* Gait::RunRipple (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration){
	double phase_offset = duration / num_legs;
	setFi(fi);
	setAlpha(alpha);
	setPath();
	setTrajectory(duration*2/3, duration/3);
	KDL::Frame frame;

	if (run_state == false){
		run_state = true;
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;
	}

	if (pause_state == true){
		begin_sec = ros::Time::now().toSec() - passed_sec;
		pause_state = false;
	}

	passed_sec = ros::Time::now().toSec() - begin_sec;

	getTipVector(trajectory_transfer,	phase_offset,	vector_iter, scale);
	getTipVector(trajectory_transfer,	0,		vector_iter, scale);
	getTipVector(trajectory_support,	3*phase_offset,	vector_iter, scale);
	getTipVector(trajectory_support,	2*phase_offset,	vector_iter, scale);
	getTipVector(trajectory_support,	phase_offset,	vector_iter, scale);
	getTipVector(trajectory_support,	0,		vector_iter, scale);

	if (passed_sec >= phase_offset-0.02 && run_state==true){
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;

		legs_queue.push(legs_queue.front());
		legs_queue.pop();
	}
	return final_vector;
}

void Gait::getTipVector (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale){
	KDL::Frame frame;
	frame = trajectory -> Pos(passed_sec + phase_offset);
	frame.p.x(frame.p.data[0]*scale);
	frame.p.y(frame.p.data[1]*scale);
	final_vector[legs_queue.front()] = frame.M * (*(vector_iter + legs_queue.front())).p + frame.p;
	legs_queue.push(legs_queue.front());
	legs_queue.pop();
}


void Gait::Pause (){
	pause_state = true;
}

void Gait::Stop (){
	run_state = false;
}

//-------------------------------------------------------------  RUNSINGLE  -----------------------------------------------------------

KDL::Vector* Gait::RunSingle (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration){
	double phase_offset = duration / num_legs;
	setFi(fi);
	setAlpha(alpha);
	setPath();
	setTrajectory(duration*5/6, duration/6);
	KDL::Frame frame;

	if (run_state == false){
		run_state = true;
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;
	}

	if (pause_state == true){
		begin_sec = ros::Time::now().toSec() - passed_sec;
		pause_state = false;
	}

	passed_sec = ros::Time::now().toSec() - begin_sec;

	getTipVector(trajectory_transfer,	0,		vector_iter, scale);
	getTipVector(trajectory_support,	4*phase_offset,	vector_iter, scale);
	getTipVector(trajectory_support,	3*phase_offset,	vector_iter, scale);
	getTipVector(trajectory_support,	2*phase_offset,	vector_iter, scale);
	getTipVector(trajectory_support,	phase_offset,	vector_iter, scale);
	getTipVector(trajectory_support,	0,		vector_iter, scale);

	if (passed_sec >= phase_offset-0.02 && run_state==true){
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;

		legs_queue.push(legs_queue.front());
		legs_queue.pop();
	}
	return final_vector;
}


//-------------------------------------------------------------  BACKLEGBROKEN  -----------------------------------------------------------

KDL::Vector* Gait::BackLegBroken (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration){
	double phase_offset = duration / num_legs;
	setFi(fi);
	setAlpha(alpha);
	setPath();
	setTrajectory(duration*5/6, duration/6);
	KDL::Frame frame;

	if (run_state == false){
		run_state = true;
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;
	}

	if (pause_state == true){
		begin_sec = ros::Time::now().toSec() - passed_sec;
		pause_state = false;
	}

	passed_sec = ros::Time::now().toSec() - begin_sec;

	getTipVectorBackMoving(trajectory_transfer,	0,		vector_iter, scale);
	getTipVectorBackMoving(trajectory_support,	3*phase_offset,	vector_iter, scale);
	getTipVectorBackMoving(trajectory_support,	2*phase_offset,	vector_iter, scale);
	getTipVectorBackMoving(trajectory_support,	phase_offset,	vector_iter, scale);
	getTipVectorBackMoving(trajectory_support,	0,		vector_iter, scale);
	getTipVectorBackBroken(trajectory_support,	0,		vector_iter, scale);

	if (passed_sec >= phase_offset-0.02 && run_state==true){
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;

		legs_queue_back_broken.push(legs_queue_back_broken.front());
		legs_queue_back_broken.pop();
	}
	return final_vector;
}

void Gait::getTipVectorBackMoving (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale){
	KDL::Frame frame;
	frame = trajectory -> Pos(passed_sec + phase_offset);
	frame.p.x(frame.p.data[0]*scale);
	frame.p.y(frame.p.data[1]*scale);
	final_vector[legs_queue_back_broken.front()] = frame.M * (*(vector_iter + legs_queue_back_broken.front())).p + frame.p;
	legs_queue_back_broken.push(legs_queue_back_broken.front());
	legs_queue_back_broken.pop();
}

void Gait::getTipVectorBackBroken (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale){
	KDL::Frame frame;
        frame = trajectory_support -> Pos(passed_sec);
        frame.p.x(0.0);
        frame.p.y(0.0);
        final_vector[5] = frame.M *(*(vector_iter+5)).p + frame.p;
}


//-------------------------------------------------------------  MIDDLELEGBROKEN  -----------------------------------------------------------

KDL::Vector* Gait::MiddleLegBroken (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration){
	double phase_offset = duration / num_legs;
	setFi(fi);
	setAlpha(alpha);
	setPath();
	setTrajectory(duration*5/6, duration/6);
	KDL::Frame frame;

	if (run_state == false){
		run_state = true;
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;
	}

	if (pause_state == true){
		begin_sec = ros::Time::now().toSec() - passed_sec;
		pause_state = false;
	}

	passed_sec = ros::Time::now().toSec() - begin_sec;

	getTipVectorMiddleMoving(trajectory_transfer,	0,		vector_iter, scale);
	getTipVectorMiddleMoving(trajectory_support,	3*phase_offset,	vector_iter, scale);
	getTipVectorMiddleMoving(trajectory_support,	2*phase_offset,	vector_iter, scale);
	getTipVectorMiddleMoving(trajectory_support,	phase_offset,	vector_iter, scale);
	getTipVectorMiddleMoving(trajectory_support,	0,		vector_iter, scale);
	getTipVectorMiddleBroken(trajectory_support,	0,		vector_iter, scale);

	if (passed_sec >= phase_offset-0.02 && run_state==true){
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;

		legs_queue_middle_broken.push(legs_queue_middle_broken.front());
		legs_queue_middle_broken.pop();
	}
	return final_vector;
}

void Gait::getTipVectorMiddleMoving (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale){
	KDL::Frame frame;
	frame = trajectory -> Pos(passed_sec + phase_offset);
	frame.p.x(frame.p.data[0]*scale);
	frame.p.y(frame.p.data[1]*scale);
	final_vector[legs_queue_middle_broken.front()] = frame.M * (*(vector_iter + legs_queue_middle_broken.front())).p + frame.p;
	legs_queue_middle_broken.push(legs_queue_middle_broken.front());
	legs_queue_middle_broken.pop();
}

void Gait::getTipVectorMiddleBroken (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale){
	KDL::Frame frame;
        frame = trajectory_support -> Pos(passed_sec);
        frame.p.x(0.0);
        frame.p.y(0.0);
        final_vector[1] = frame.M *(*(vector_iter+1)).p + frame.p;
}

//-------------------------------------------------------------  FOURLEGMOVEMENT  -----------------------------------------------------------

KDL::Vector* Gait::FourLegMovement (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration){
	double phase_offset = duration / num_legs;
	setFi(fi);
	setAlpha(alpha);
	setPath();
	setTrajectory(duration*5/6, duration/6);
	KDL::Frame frame;

	if (run_state == false){
		run_state = true;
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;
	}

	if (pause_state == true){
		begin_sec = ros::Time::now().toSec() - passed_sec;
		pause_state = false;
	}

	passed_sec = ros::Time::now().toSec() - begin_sec;

	getTipVectorFourLegsMoving(trajectory_transfer,	0,		vector_iter, scale);
	getTipVectorFourLegsMoving(trajectory_support,	2*phase_offset,	vector_iter, scale);
	getTipVectorFourLegsMoving(trajectory_support,	phase_offset,	vector_iter, scale);
	getTipVectorFourLegsMoving(trajectory_support,	0,		vector_iter, scale);
	getTipVectorFirstLegBroken(trajectory_support,	0,		vector_iter, scale);
	getTipVectorSecondLegBroken(trajectory_support,	0,		vector_iter, scale);

	if (passed_sec >= phase_offset-0.02 && run_state==true){
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;

		legs_queue_fourlegmovement.push(legs_queue_fourlegmovement.front());
		legs_queue_fourlegmovement.pop();
	}
	return final_vector;
}

void Gait::getTipVectorFourLegsMoving (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale){
	KDL::Frame frame;
	frame = trajectory -> Pos(passed_sec + phase_offset);
	frame.p.x(frame.p.data[0]*scale);
	frame.p.y(frame.p.data[1]*scale);
	final_vector[legs_queue_fourlegmovement.front()] = frame.M * (*(vector_iter + legs_queue_fourlegmovement.front())).p + frame.p;
	legs_queue_fourlegmovement.push(legs_queue_fourlegmovement.front());
	legs_queue_fourlegmovement.pop();
}

void Gait::getTipVectorFirstLegBroken (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale){
	KDL::Frame frame;
        frame = trajectory_support -> Pos(passed_sec);
        frame.p.x(0.0);
        frame.p.y(0.0);
        final_vector[2] = frame.M *(*(vector_iter+2)).p + frame.p;
}

void Gait::getTipVectorSecondLegBroken (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale){
	KDL::Frame frame;
        frame = trajectory_support -> Pos(passed_sec);
        frame.p.x(0.0);
        frame.p.y(0.0);
        final_vector[5] = frame.M *(*(vector_iter+5)).p + frame.p;
}





																			
