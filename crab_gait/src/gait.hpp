#ifndef GAIT_HPP_
#define GAIT_HPP_


#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/path_point.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <queue>

#define NUM_LEGS 6
#define NUM_JOINTS 3

class Gait {
	public:
		Gait();
		void setTrapezoid (double low_rad, double high_rad, double height, double z);
		KDL::Vector* RunTripod   (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration);
		KDL::Vector* RunRipple   (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration);
		KDL::Vector* RunSingle   (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration);          
		KDL::Vector* BackLegBroken (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration);        
		KDL::Vector* MiddleLegBroken (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration);      
		KDL::Vector* FourLegMovement (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration);      
		void Pause ();
		void Stop ();

		KDL::Frame a, b, c, d;
		KDL::Trajectory_Segment *trajectory_transfer, *trajectory_support;

	private:
		const static unsigned int num_joints = NUM_JOINTS;
		const static unsigned int num_legs = NUM_LEGS;

		void getTipVector (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale);
		void getTipVectorBackBroken (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale);
		void getTipVectorBackMoving (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale);
		void getTipVectorMiddleBroken (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale);
		void getTipVectorMiddleMoving (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale);
		void getTipVectorFourLegsMoving (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale);
		void getTipVectorFirstLegBroken (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale);
		void getTipVectorSecondLegBroken (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale);
		void setFi (double fi);
		void setAlpha (double alpha);
		void setPath ();
		void setTrajectory (double sup_path_duration, double tran_path_duration);

		std::queue<int> legs_queue, legs_queue_back_broken, legs_queue_middle_broken, legs_queue_fourlegmovement;
		bool run_state, pause_state;
		int phase;
		double passed_sec, begin_sec;
		double low_rad, high_rad, height, z_body;
		KDL::Vector final_vector [num_legs];
		KDL::RotationalInterpolation_SingleAxis rot;
		KDL::Path_Line *path_support;
		KDL::Path_RoundedComposite *path_transfer;
		//KDL::VelocityProfile_Spline prof_support, prof_transfer;
		KDL::VelocityProfile_Spline prof_support, prof_transfer;

};



#endif /* GAIT_HPP_ */
