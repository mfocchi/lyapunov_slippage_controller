#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include <math.h> 

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "lyapunov_slippage_controller/lyapunovController.h"
#include "lyapunov_slippage_controller/coppeliaSimNode.h"
#include "lyapunov_slippage_controller/differential_drive_model.h"
#include "lyapunov_slippage_controller/error_codes.h"

#define NANO 0.000000001
#define QUEUE_DEPTH_OPTITRACK 2
#define MAX_LONG_SLIP 0.8 // 1 but problem with singularities
#define MIN_LONG_SLIP -10
#define SIDE_SLIP_EPSILON 0.01
#define LONG_SLIP_EPSILON 0.01

#define RESET   "\033[0m"
#define RED     "\033[31m"      /* Red */
#define prt(x) std::cout << RED << #x " = \n" << x << "\n" << RESET<< std::endl;
#define prt_vec(x) for( int i = 0; i < x.size(); i++) {std::cout << x[i] << " \n";};


using namespace std::chrono_literals;
// specify the level of prints during the execution of the code ABSENT, DEBUG, MINIMAL
const Verbosity code_verbosity_sub = ABSENT; 
const Verbosity code_verbosity_setup = DEBUG; 
const Verbosity code_verbosity_pub = DEBUG; 

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class SlippageControllerNode : public CoppeliaSimNode
{
private:
    rclcpp::TimerBase::SharedPtr timer_cmd;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_cmd;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_trk_error;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_reference;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_actual;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_slippage_commands;	
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_unicycle_commands;	
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_alpha;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;
    LyapControllerPtr Ctrl;
	std::shared_ptr<DifferentialDriveModel> Model;
    Eigen::Vector3d pose;
    double t_start;
	double t_pose_init;

	Eigen::Vector2d u_bar;
	Eigen::Vector2d u;

    std::vector<double> origin_RF;
	//OLD
	// std::vector<double> i_inner_coeff;
	// std::vector<double> i_outer_coeff;
	// std::vector<double> alpha_coeff;

//side slip
	std::vector<double> side_slip_angle_coefficients_left ;
	std::vector<double> side_slip_angle_coefficients_right;
	//longitudinal slip coefficients
	std::vector<double> beta_slip_outer_coefficients_left ;
	std::vector<double> beta_slip_outer_coefficients_right;
	std::vector<double> beta_slip_inner_coefficients_left ;
	std::vector<double> beta_slip_inner_coefficients_right;



	bool enable_pose_init;
	bool enable_slippage;
	double n_samples_pose_init;


	//new imoplem
	double alpha_f = 0.;
	double alpha_f_old = 0.;
	double alpha = 0.;
	double alpha_dot =0.;


	void timerCmdCallback()
    {
		if(enable_pose_init == true)
		{
			return;// do not publish anything till is initialized
		}
		Eigen::Vector2d cmd = (enable_slippage) ? trackTrajectorySlippage() : trackTrajectoryClassic();
		sensor_msgs::msg::JointState msg_cmd;
		msg_cmd.name = {"left_sprocket", "right_sprocket"};
		msg_cmd.velocity = {cmd(0), cmd(1)};
		pub_cmd->publish(msg_cmd);

		Eigen::Vector3d tracking_error= getTrackingErrorMsg();
		geometry_msgs::msg::Vector3Stamped msg_err;
		msg_err.vector.x = tracking_error(0);
		msg_err.vector.y = tracking_error(1);
		msg_err.vector.z = tracking_error(2);
		msg_err.header.stamp = this->get_clock()->now();
		msg_err.header.frame_id = "world";		
		pub_trk_error->publish(msg_err);

		geometry_msgs::msg::Vector3Stamped msg_actual_pos;
	    msg_actual_pos.vector.x = this->pose(0);
		msg_actual_pos.vector.y = this->pose(1);
		msg_actual_pos.vector.z = this->pose(2);
		msg_actual_pos.header.stamp = this->get_clock()->now();
		msg_actual_pos.header.frame_id = "world";		
		pub_actual->publish(msg_actual_pos);

		geometry_msgs::msg::Vector3Stamped msg_reference_pos;
		Eigen::Vector3d pose_ref = Ctrl->getPoseDesiredOnTime(getCurrentTime() - t_start);
		msg_reference_pos.vector.x = pose_ref(0);
		msg_reference_pos.vector.y = pose_ref(1);
		msg_reference_pos.vector.z = pose_ref(2);
		msg_reference_pos.header.stamp = this->get_clock()->now();
		msg_reference_pos.header.frame_id = "world";		
		pub_reference->publish(msg_reference_pos);
	
		std_msgs::msg::Float64MultiArray msg_slippage_commands;
		msg_slippage_commands.data.push_back(u(0));  
		msg_slippage_commands.data.push_back(u(1));
		pub_slippage_commands->publish(msg_slippage_commands);

		std_msgs::msg::Float64MultiArray msg_unicycle_commands;
		msg_unicycle_commands.data.push_back(u_bar(0));  
		msg_unicycle_commands.data.push_back(u_bar(1));
		pub_unicycle_commands->publish(msg_unicycle_commands);

		std_msgs::msg::Float64MultiArray msg_alpha;
		msg_alpha.data.push_back(this->alpha_f);
		msg_alpha.data.push_back(this->alpha_dot);
		pub_alpha->publish(msg_alpha);
    }

	

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {		
		tf2::Quaternion q(
			msg->pose.orientation.x,
			msg->pose.orientation.y,
			msg->pose.orientation.z,
			msg->pose.orientation.w);
		

		double x = msg->pose.position.x;
		double y = msg->pose.position.y;
		//we perform rotation already in optitrack
		// R = tf2::Matrix3x3(
		// 	origin_RF[0], origin_RF[1], origin_RF[2],
		// 	origin_RF[3], origin_RF[4], origin_RF[5],
		// 	origin_RF[6], origin_RF[7], origin_RF[8]) * R;

		// // Transform the coordinate system into the classic x,y on plane,z upwards
		// double x = msg->pose.position.x * origin_RF[0] + msg->pose.position.y * origin_RF[1] + msg->pose.position.z * origin_RF[2]; 
		// double y = msg->pose.position.x * origin_RF[3] + msg->pose.position.y * origin_RF[4] + msg->pose.position.z * origin_RF[5]; 
		//double z = msg->pose.position.x * origin_RF[6] + msg->pose.position.y * origin_RF[7] + msg->pose.position.z * origin_RF[8]; 

		double roll, pitch, yaw;
		Eigen::Vector3d rpy = euler_from_quaternion(q);
		
		
		yaw = angleWithinPI(rpy[2]);

		if(code_verbosity_sub == DEBUG)
		{
			std::cout << "Pose callback: x=" << x << " y=" << y << " yaw=" << yaw << std::endl;
		}
		if(this->enable_pose_init)
		{
			if(code_verbosity_sub == DEBUG)
			{
				std::cout << "Performing pose initiation" << yaw << std::endl;
			}
			try
				{initProcedure(x,y,yaw);}
			catch(Error e)
				{printErrorCode(e);}
		}
		else
		{
			this->pose << x,y,yaw;
		}
    }


	//OLD IMPLEMENTATION
	// /* It tracks a trajectory defined in terms of velocities. It has to set the current time 
	// to extract the desired velocities and poses, then it computes the control with the standard
	// controller and finally the effects of slippage are compensated by a transformer */
    // Eigen::Vector2d trackTrajectorySlippage()
    // {


	// 	// Ctrl->setCurrentTime(getCurrentTime() - t_start);
	// 	// u = Ctrl->run(this->pose);
	// 	// if(code_verbosity_pub == DEBUG)
	// 	// 	std::cout<<"u control input [m/s, rad/s]: LIN " << u(0) << " ANG " << u(1) << std::endl;
	// 	// // conversion block from unicycle to differential drive
	// 	// Model->setUnicycleSpeed(u(0), u(1));
	// 	// Eigen::Vector2d motor_vel;
	// 	// motor_vel << Model->getLeftMotorRotationalSpeed(), Model->getRightMotorRotationalSpeed();
	// 	// if(code_verbosity_pub == DEBUG)
	// 	// 	std::cout<<"Motors control input [rad/s]: LEFT " << motor_vel(0) << " RIGHT " << motor_vel(1) << std::endl;

	// 	// // compute an estimation of the longitudinal and lateral slips
	// 	Eigen::Vector3d pose_offset(0.0, 0.0, this->alpha_prev_1.data<);
	// 	Eigen::Vector3d pose_bar = this->pose + pose_offset; 
	
	// 	Ctrl->setCurrentTime(getCurrentTime() - t_start);
		
	// 	u_bar = Ctrl->run(pose_bar);
	
	// 	// compute commands with side slip compensation
	// 	u = convertskidSteering(u_bar);
	// 	if(code_verbosity_pub == DEBUG)
	// 	 	std::cout<<"u_bar control input [m/s, rad/s]: LIN " << u_bar(0) << " ANG " << u_bar(1) << std::endl;
	// 		std::cout<<"u control input [m/s, rad/s]: LIN " << u(0) << " ANG " << u(1) << std::endl;
	// 	// conversion block from unicycle to differential drive
	// 	Model->setUnicycleSpeed(u(0), u(1));
	// 	double motor_vel_L = Model->getLeftMotorRotationalSpeed();
	// 	double motor_vel_R = Model->getRightMotorRotationalSpeed();
		
	// 	// // compensate the longitudinal slip
	// 	// // double motor_vel_comp_L = motor_vel_L / (1-this->i_L_prev);
	// 	// // double motor_vel_comp_R = motor_vel_R / (1-this->i_R_prev);	
	// 	// //Model->setDifferentialSpeed(motor_vel_comp_L, motor_vel_comp_R);		
	// 	// //Eigen::Vector2d motor_vel(motor_vel_comp_L, motor_vel_comp_R);
		
	// 	// //TESTING 2 : do not compensate long slip (uncomment the following and comment the previous)
	// 	Model->setDifferentialSpeed(motor_vel_L, motor_vel_L);
	// 	Eigen::Vector2d motor_vel(motor_vel_L, motor_vel_R);

	// 	// // estimate alpha, alpha_dot
	// 	// //compute alpha based on slip compensated control input 
	// 	// updateSlipVariables(u);
	// 	// // TESTING 1: compute alpha based on desired control input
	// 	// updateSlipVariables(Ctrl->getControlInputDesiredOnTime(getCurrentTime() - t_start)); 
		
		
	// 	// if(code_verbosity_pub == DEBUG)
	// 	// 	std::cout<<"Motors control input [rad/s]: LEFT " << motor_vel(0) << " RIGHT " << motor_vel(1) << std::endl;

	// 	return motor_vel;
    // }


/* It tracks a trajectory defined in terms of velocities. It has to set the current time 
	to extract the desired velocities and poses, then it computes the control with the standard
	controller and finally the effects of slippage are compensated by a transformer */
    Eigen::Vector2d trackTrajectorySlippage()
    {
		Ctrl->setCurrentTime(getCurrentTime() - t_start);
		std::cout<<"---------------------------------------------------------- "<< std::endl;
		// // compute an estimation of the longitudinal and lateral slips
		Eigen::Vector3d pose_offset(0.0, 0.0, sign(alpha)*this->alpha_f);
		Eigen::Vector3d pose_bar = this->pose + pose_offset; //theta+alpha
		
		//make theta+alpha track theta_des
		u_bar = Ctrl->run(pose_bar);
	
		// compute commands with side slip compensation
		u(0) = u_bar(0) * cos(sign(alpha)*this->alpha_f);

		if ((getCurrentTime() - t_start)< 0.05)
		{			
			std::cout <<RED<<"Not applying alpha dot"<<RESET<<std::endl;
			u(1) = u_bar(1);
		}
		else{
			
			u(1) = u_bar(1) -this->alpha_dot;
		}

			
		if(code_verbosity_pub == DEBUG)
		{
			std::cout<<"u_bar  [m/s, rad/s]: LIN " << u_bar(0) << " ANG " << u_bar(1) << std::endl;
			std::cout<<RED<<"u  [m/s, rad/s]: LIN " << u(0) << " ANG " << u(1) << RESET<<std::endl;
		}

		//compute ideal wheel speed
		// conversion block from unicycle to differential drive (this assumes there is no longitudinal slippage)
		double motor_vel_L   = (u(0) - u(1) * (0.5*Model->wheel_distance))/ Model->wheel_radius*Model->gearbox;
	    double motor_vel_R = (u(0) + u(1) * (0.5*Model->wheel_distance))/ Model->wheel_radius*Model->gearbox;


        //  estimate slippage params alpha, alpha_dot. Note that all estimates are computed using the controller output and are used the next cycle, thus
		// the have a delay of one step

		//alpha 
		double dt = this->get_parameter("pub_dt_ms").as_int()/1000.0;
		double filter_gain = dt/(dt+0.005);
		this->alpha = computeSideSlipAngle(u_bar);
		//filter always positive
		this->alpha_f = (1.-filter_gain)*this->alpha_f + filter_gain * abs(this->alpha);
		//alpha dot	
		this->alpha_dot =  sign(alpha) * (abs(this->alpha_f)  - abs(alpha_f_old)) / dt;
		this->alpha_f_old = this->alpha_f;

		Eigen::Vector2d motor_vel_compensated, motor_vel;
		motor_vel << motor_vel_L, motor_vel_R; //for debug
		//compute compensated wheel speed after accounting longitudinal slippage
		motor_vel_compensated = computeLongSlipCompensation(motor_vel_L, motor_vel_R, u_bar);
				
		if(code_verbosity_pub == DEBUG)
		{
			std::cout<<"alpha:  " << this->alpha << " alpha_f:  " << this->alpha_f << " alpha_dot " << this->alpha_dot << std::endl;	
			std::cout<<"Motors control input [rad/s]: LEFT " << motor_vel_L << " RIGHT " << motor_vel_R << std::endl;
			std::cout<<"Motors control input comp [rad/s]: LEFT " << motor_vel_compensated(0) << " RIGHT " << motor_vel_compensated(1) << std::endl;
		}
		return motor_vel;
    }


	/* It tracks a trajectory defined in terms of velocities. It has to set the current time 
	to extract the desired velocities and poses, then it computes the control with the standard OLyapunov Unicycle
	controller*/
    Eigen::Vector2d trackTrajectoryClassic()
    {
		Ctrl->setCurrentTime(getCurrentTime() - t_start);

		u = Ctrl->run(this->pose);
		if(code_verbosity_pub == DEBUG)
			std::cout<<"u control input [m/s, rad/s]: LIN " << u(0) << " ANG " << u(1) << std::endl;

		// conversion block from unicycle to differential drive
		Model->setUnicycleSpeed(u(0), u(1));

		Eigen::Vector2d motor_vel;
		motor_vel << Model->getLeftMotorRotationalSpeed(), Model->getRightMotorRotationalSpeed();
		if(code_verbosity_pub == DEBUG)
			std::cout<<"Motors control input [rad/s]: LEFT " << motor_vel(0) << " RIGHT " << motor_vel(1) << std::endl;

		return motor_vel;
    }

	Eigen::Vector3d getTrackingErrorMsg()
	{
		double e_x = Ctrl->getErrorX();
		double e_y = Ctrl->getErrorY();
		double e_th = Ctrl->getErrorTheta();
		Eigen::Vector3d e;
		e << e_x, e_y, e_th;
		return e;
	}

	double getCurrentTime()
	{
		double t;
		if(this->isCoppeliaSimEnabled())
		{
			t = this->getSimTime();
		}
		else
		{
			t = double((this->get_clock()->now()).nanoseconds()) * NANO;
		}
		return t;
	}

	void updatePoseWithMovingAverage(double x, double y, double yaw)
	{
		// Current pose is treated as the current pose average
		// handle the periodicity of the angle
		double yaw_diff = this->pose(2) - yaw;
		double yaw_tmp = yaw;
		if(yaw_diff > M_PI)
		{
			yaw_tmp = this->pose(2) + (2*M_PI - yaw_diff);
		} 
		else if(yaw_diff < -M_PI)
		{
			yaw_tmp = this->pose(2) - (2*M_PI - yaw_diff);
		}

		// perform moving average
		double x_avg = this->pose(0) + (x - this->pose(0)) / 
			(this->n_samples_pose_init+1);
		double y_avg = this->pose(1) + (y - this->pose(1)) / 
			(this->n_samples_pose_init+1);
		double yaw_avg = this->pose(2) + (yaw_tmp - this->pose(2)) / 
			(this->n_samples_pose_init+1);

		// update values
		yaw_avg = angleWithinPI(yaw_avg);
		this->pose << x_avg, y_avg, yaw_avg;
		this->n_samples_pose_init += 1.0;
	}

	void initPoseMovingAverage(double x, double y, double yaw)
	{
		this->pose << x,y,angleWithinPI(yaw);
		this->n_samples_pose_init += 1.0;
	}

	void setupTrajectory()
	{
		std::vector<double> v_vec = this->get_parameter("v_des_mps").as_double_array();
		std::vector<double> omega_vec = this->get_parameter("omega_des_radps").as_double_array();
		std::vector<double> x_vec = this->get_parameter("x_des_m").as_double_array();
		std::vector<double> y_vec = this->get_parameter("y_des_m").as_double_array();
		std::vector<double> theta_vec = this->get_parameter("theta_des_rad").as_double_array();
		bool COPY_WHOLE_TRAJ = this->get_parameter("copy_trajectory").as_bool();
		
		this->t_start = getCurrentTime();
		try{
			if(COPY_WHOLE_TRAJ)
			{
				if(code_verbosity_setup == DEBUG || code_verbosity_setup == MINIMAL)
					std::cout << "Trajectory: copying the whole inputs and trajectory" << std::endl;
				this->addTrajectory(v_vec, omega_vec, x_vec, y_vec, theta_vec);
			}
			else
			{
				if(code_verbosity_setup == DEBUG || code_verbosity_setup == MINIMAL)
					std::cout << "Trajectory: copying only control inputs" << std::endl;
				this->addControlsTrajectory(v_vec, omega_vec);
			}
		}
		catch(Error code){
			printErrorCode(code);
		}
	}
	

	double computeSideSlipAngle(const Eigen::Vector2d& u) const
	{
	
		double R = computeTurningRadius(u(0), u(1));
			

		if(code_verbosity_pub == DEBUG)
		{
			std::cout << "R=" << R << std::endl;
		}
		
		double a0,a1,a2,a3;
		double alpha;
		if(R > 0.0) // turning left
		{
			
			a0 = this->side_slip_angle_coefficients_left.at(0);
			a1 = this->side_slip_angle_coefficients_left.at(1);
			a2 = this->side_slip_angle_coefficients_left.at(2);
			a3 = this->side_slip_angle_coefficients_left.at(3);		
			alpha = a0*exp(a1*R) + a2*exp(a3*R);
			if (alpha < -M_PI/2)
				alpha = -M_PI/2;
		}
		else // turning right
		{
		    a0 = this->side_slip_angle_coefficients_right.at(0);
			a1 = this->side_slip_angle_coefficients_right.at(1);
			a2 = this->side_slip_angle_coefficients_right.at(2);
			a3 = this->side_slip_angle_coefficients_right.at(3);	
			alpha = a0*exp(a1*R) + a2*exp(a3*R);
			if (alpha > M_PI/2)
				alpha = M_PI/2;	
		}
		return alpha;
	}
	
	Eigen::Vector2d computeLongSlipCompensation(const double ideal_wheel_L,const double ideal_wheel_R, const Eigen::Vector2d& u) const
	{
	
	    double R = computeTurningRadius(u(0), u(1));
		// compute track velocity from encoder
		double  v_enc_l, v_enc_r;
		v_enc_l = Model->wheel_radius/Model->gearbox*ideal_wheel_L;
        v_enc_r = Model->wheel_radius/Model->gearbox*ideal_wheel_R;

		// estimate beta_inner, beta_outer from turning radius
		double a0, a1, beta_inner, beta_outer;
		if(R >= 0.0) // turning left, left wheel is inner there is discontinuity
		{
			a0 = this->beta_slip_inner_coefficients_left.at(0);
			a1 = this->beta_slip_inner_coefficients_left.at(1);
			beta_inner = a0*exp(a1*R);
			v_enc_l-=beta_inner;

			a0 = this->beta_slip_outer_coefficients_left.at(0);
			a1 = this->beta_slip_outer_coefficients_left.at(1);
			beta_outer = a0*exp(a1*R);
			v_enc_r+=beta_outer;
		}
		else // turning right , left wheel is outer
		{
			a0 = this->beta_slip_inner_coefficients_right.at(0);
			a1 = this->beta_slip_inner_coefficients_right.at(1);
			beta_inner = a0*exp(a1*R);
			v_enc_r-=beta_inner;

			a0 = this->beta_slip_outer_coefficients_right.at(0);
			a1 = this->beta_slip_outer_coefficients_right.at(1);
			beta_outer = a0*exp(a1*R);
			v_enc_l+=beta_outer;
		}
		Eigen::Vector2d wheel_speed_comp;
		wheel_speed_comp(0) = Model->gearbox / Model->wheel_radius* v_enc_l;
		wheel_speed_comp(1) = Model->gearbox / Model->wheel_radius* v_enc_r;

		return wheel_speed_comp;
	}

	//OLD
	// double computeLongSlip(double R, double a0, double a1) const
	// {
	// 	double slip;
	// 	if(abs(a1 + R) <= LONG_SLIP_EPSILON)
	// 	{
	// 		// close to singularity
	// 		if(code_verbosity_pub == DEBUG)
	// 		{
	// 			std::cout << "Long Slip: SINGULARITY" << std::endl;
	// 		}
	// 		if(abs(a1 + R) * a0 >= 0.0)
	// 		{
	// 			slip = MAX_LONG_SLIP;
	// 		}
	// 		else
	// 		{
	// 			slip = MIN_LONG_SLIP;
	// 		}
	// 	}
	// 	else
	// 	{
	// 		slip = a0 / (a1 + R);
	// 		if(slip > MAX_LONG_SLIP)
	// 		{
	// 			slip = MAX_LONG_SLIP;
	// 		}
	// 		else if(slip < MIN_LONG_SLIP)
	// 		{
	// 			slip = MIN_LONG_SLIP;
	// 		}
	// 	}
	// 	return slip;
	// }

	
	void initSlipVariables()
	{
		if(this->enable_slippage == false)
		{
			return;
		}
		Eigen::Vector2d u0 = Ctrl->getControlInputDesiredOnTime(0.0);
		int dt = this->get_parameter("pub_dt_ms").as_int();
		
	
		this->alpha_dot = 0.0;
		// init side slip with expected value from desired velocities
		this->alpha_f_old = computeSideSlipAngle(u0);

	}



public:
    SlippageControllerNode() : CoppeliaSimNode("lyapunov_slippage_controller")
    {
		declare_parameter("track_distance_m", 0.606);
		declare_parameter("sprocket_radius_m", 0.0856);
		declare_parameter("gearbox_ratio", 34.45);
		declare_parameter("Kp", 1.0);
		declare_parameter("Ktheta", 5.0);
		declare_parameter("dt", 0.1);
		declare_parameter("pub_dt_ms", 100);
		declare_parameter("time_for_pose_init_s", 0.5);
		declare_parameter("automatic_pose_init", false);
		declare_parameter("pose_init_m_m_rad", std::vector<double>({0.0,0.0,0.0}));

		//OLD
		// declare_parameter("long_slip_inner_coefficients", std::vector<double>({0.0,0.0,0.0}));
		// declare_parameter("long_slip_outer_coefficients", std::vector<double>({0.0,0.0,0.0}));
		// declare_parameter("side_slip_angle_coefficients", std::vector<double>({0.0,0.0,0.0}));


		declare_parameter("side_slip_angle_coefficients_left", std::vector<double>({0.0,0.0,0.0}));
		declare_parameter("side_slip_angle_coefficients_right", std::vector<double>({0.0,0.0,0.0}));
		declare_parameter("beta_slip_outer_coefficients_left", std::vector<double>({0.0,0.0,0.0}));
		declare_parameter("beta_slip_outer_coefficients_right", std::vector<double>({0.0,0.0,0.0}));
		declare_parameter("beta_slip_inner_coefficients_left", std::vector<double>({0.0,0.0,0.0}));
		declare_parameter("beta_slip_inner_coefficients_right", std::vector<double>({0.0,0.0,0.0}));

		declare_parameter("derivative_filter_time_constant_s", 0.01); // for the alpha dot calculation

    	declare_parameter("v_des_mps", std::vector<double>({0.0,0.0}));
    	declare_parameter("omega_des_radps", std::vector<double>({0.0,0.0}));
    	declare_parameter("x_des_m", std::vector<double>({0.0,0.0})); // it is used only when copy_trajectory is true
    	declare_parameter("y_des_m", std::vector<double>({0.0,0.0})); // it is used only when copy_trajectory is true
    	declare_parameter("theta_des_rad", std::vector<double>({0.0,0.0})); // it is used only when copy_trajectory is true
		declare_parameter("origin_RF", std::vector<double>({1,0,0,0,1,0,0,0,1}));
		// true: the desired pose is set by the user as well as the desired control inputs
		// false: only the desired control inputs are specified by user, the pose is obtained via integration of
		//		of the unicycle kinematic model
		declare_parameter("copy_trajectory", false); 
		// set it to false for classic differential drive model, true for skid-steering control (requires slip models)
		declare_parameter("consider_slippage", true); 
				
		double r = this->get_parameter("sprocket_radius_m").as_double();
		double gear_ratio = this->get_parameter("gearbox_ratio").as_double();
		double d = this->get_parameter("track_distance_m").as_double();
		double Kp = this->get_parameter("Kp").as_double();
		double Ktheta = this->get_parameter("Ktheta").as_double();
		double dt = this->get_parameter("dt").as_double();
		int pub_dt = this->get_parameter("pub_dt_ms").as_int();
		this->enable_pose_init = this->get_parameter("automatic_pose_init").as_bool();
		this->enable_slippage = this->get_parameter("consider_slippage").as_bool();
		this->t_pose_init = this->get_parameter("time_for_pose_init_s").as_double();
		std::vector<double> pose_init = this->get_parameter("pose_init_m_m_rad").as_double_array();

		//OLD
		// i_inner_coeff = this->get_parameter("long_slip_inner_coefficients").as_double_array();
		// i_outer_coeff = this->get_parameter("long_slip_outer_coefficients").as_double_array();
		// alpha_coeff   = this->get_parameter("side_slip_angle_coefficients").as_double_array();

		//side slip
		side_slip_angle_coefficients_left = this->get_parameter("side_slip_angle_coefficients_left").as_double_array();
		side_slip_angle_coefficients_right = this->get_parameter("side_slip_angle_coefficients_right").as_double_array();
		//longitudinal slip
		beta_slip_outer_coefficients_left = this->get_parameter("beta_slip_outer_coefficients_left").as_double_array();
		beta_slip_outer_coefficients_right = this->get_parameter("beta_slip_outer_coefficients_right").as_double_array();
		beta_slip_inner_coefficients_left = this->get_parameter("beta_slip_inner_coefficients_left").as_double_array();
		beta_slip_inner_coefficients_right = this->get_parameter("beta_slip_inner_coefficients_right").as_double_array();

		origin_RF = this->get_parameter("origin_RF").as_double_array();
		
		Ctrl = std::make_shared<LyapController>(Kp, Ktheta, dt); 
		Model.reset(new DifferentialDriveModel(r, d, gear_ratio));
		
		n_samples_pose_init = 0.0;
		if(enable_pose_init)
		{
			this->pose << 0.0,0.0,0.0;
		}
		else
		{
			this->pose << pose_init.at(0), pose_init.at(1), pose_init.at(2);
			Ctrl->setStateOffset(pose_init.at(0), pose_init.at(1), pose_init.at(2));
			if(code_verbosity_setup == DEBUG)
				std::cout << "Initialization procedure for initial pose" << std::endl;
			setupTrajectory();
			if(code_verbosity_setup == DEBUG)
				std::cout << "COMPLETED" << std::endl;
			if(code_verbosity_setup == DEBUG)
				std::cout << "Initialization of slip variables" << std::endl;
			initSlipVariables();
			if(code_verbosity_setup == DEBUG)
				std::cout << "COMPLETED" << std::endl;
		}
		
		

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos_optitrack = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, QUEUE_DEPTH_OPTITRACK), qos_profile);
		auto qos_ctrl = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

		pub_cmd = this->create_publisher<sensor_msgs::msg::JointState>("/command", qos_ctrl);
		pub_trk_error = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/tracking_error", qos_ctrl);
		pub_actual = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/pose_act", qos_ctrl);
		pub_reference = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/pose_ref", qos_ctrl);
		pub_slippage_commands = this->create_publisher<std_msgs::msg::Float64MultiArray>("/slippage_commands", qos_ctrl);
		pub_unicycle_commands = this->create_publisher<std_msgs::msg::Float64MultiArray>("/unicycle_commands", qos_ctrl);
		pub_alpha = this->create_publisher<std_msgs::msg::Float64MultiArray>("/alpha_alpha_dot", qos_ctrl);
		
		
		sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"/optitrack/pose", 
			qos_optitrack, 
			std::bind(&SlippageControllerNode::poseCallback, this, std::placeholders::_1));
		timer_cmd = this->create_wall_timer(
			std::chrono::milliseconds(pub_dt), 
			std::bind(&SlippageControllerNode::timerCmdCallback, this));
		if(this->isCoppeliaSimEnabled())
		{
			this->enableSyncWithCoppeliaSim();
			this->startCoppeliaSim();
			this->setCoppeliaSimRender();
		}
		
		t_start = getCurrentTime();
	}
    
    /* add the controls to generate poses via unicycle model*/
    void addControlsTrajectory(const std::vector<double>& v, const std::vector<double>& omega)
    {
		if(v.size() != omega.size())
		{
			throw TRAJECTORY_CTRL_INPUTS_DIFFERENT_SIZE;
		}
		for(int i = 0; i < int(v.size()); i++)
		{
			Ctrl->addToInputDesired(v.at(i), omega.at(i));
		}
		// saves time value such that the first control input is at delta time 0
		Ctrl->generateTrajectory();
		RCLCPP_INFO(this->get_logger(), "%s", (Ctrl->stringSetupInfo()).c_str());
    }

	/* add the trajectory defined as squence of controls and poses, no models are used*/
	void addTrajectory(const std::vector<double>& v, const std::vector<double>& omega, 
		const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& theta)
	{
		if(v.size() != omega.size())
		{
			throw TRAJECTORY_CTRL_INPUTS_DIFFERENT_SIZE;
		}
		if(x.size() != y.size() || theta.size() != y.size())
		{
			throw TRAJECTORY_POSE_DIFFERENT_SIZE;
		}
		Ctrl->copyTrajectory(v, omega, x,y,theta);
		RCLCPP_INFO(this->get_logger(), "%s", (Ctrl->stringSetupInfo()).c_str());
	}

	void initProcedure(double x, double y, double yaw) 
	{	
		if((getCurrentTime() - t_start) > this->t_pose_init)
		{
			if(this->n_samples_pose_init == 0.0)
				throw NO_POSE_FEEDBACK_4_INIT;
			Ctrl->setStateOffset(x, y, yaw);
			this->enable_pose_init = false;
			if(code_verbosity_setup == DEBUG)
				std::cout << "Pose initialized as ["<< pose(0) << ", "<< pose(1) << ", "<< pose(2) << "]"<<std::endl;
			setupTrajectory();
			this->t_start = getCurrentTime();

			if(code_verbosity_setup == DEBUG)
				std::cout << "Initialization of slip variables" << std::endl;
			//initSlipVariables();
			if(code_verbosity_setup == DEBUG)
				std::cout << "INIT COMPLETED with tstart: " << this->t_start  <<std::endl;
			return;
		}
		if(this->n_samples_pose_init == 0.0)
		{
			initPoseMovingAverage(x, y, yaw);
		}
		else
		{
			updatePoseWithMovingAverage(x, y, yaw);
		}
	}
};

// MAIN /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	printf("Lyapunov Controller node start up\n");

	std::shared_ptr<SlippageControllerNode> Ctrl(new SlippageControllerNode());

	rclcpp::spin(Ctrl);

	if(Ctrl->isCoppeliaSimEnabled())
	{
		Ctrl->stopCoppeliaSim();
	}
	rclcpp::shutdown();
	printf("Lyapunov Controller node shutdown\n");

  return 0;
}

