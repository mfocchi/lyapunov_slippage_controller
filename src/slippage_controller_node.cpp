#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"

#include "std_msgs/msg/string.hpp"
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
#define LONG_SLIP_ENABLED false

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class SlippageControllerNode : public CoppeliaSimNode
{
private:
    rclcpp::TimerBase::SharedPtr timer_cmd;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_cmd;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_trk_error;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;
    LyapControllerPtr Ctrl;
	std::shared_ptr<DifferentialDriveModel> Model;
    Eigen::Vector3d pose;
    double t_start;
	double t_pose_init;
	double tau_derivative_filter;
	double alpha_dot_prev;
	double alpha_dot;

	Measurement1D alpha_prev_1; // store alpha value at time dt*(k-2)
	Measurement1D alpha_prev_2; // store alpha value at time dt*(k-1)
	double i_L_prev; // store i_L value at time dt*(k-1)
	double i_R_prev; // store i_R value at time dt*(k-1)
    std::vector<double> origin_RF;
	std::vector<double> i_inner_coeff;
	std::vector<double> i_outer_coeff;
	std::vector<double> alpha_coeff;
	double long_slip_epsilon;

	bool enable_pose_init;
	double n_samples_pose_init;
	

	void timerCmdCallback()
    {
		if(enable_pose_init == true)
		{
			return;
		}
		sensor_msgs::msg::JointState msg_cmd;
		geometry_msgs::msg::Vector3Stamped msg_err;

		trackTrajectory(&msg_cmd);
		getTrackingErrorMsg(&msg_err);
		pub_cmd->publish(msg_cmd);
		pub_trk_error->publish(msg_err);
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {		
		tf2::Quaternion q(
			msg->pose.orientation.x,
			msg->pose.orientation.y,
			msg->pose.orientation.z,
			msg->pose.orientation.w);
		tf2::Matrix3x3 R(q);
		R = tf2::Matrix3x3(
			origin_RF[0], origin_RF[1], origin_RF[2],
			origin_RF[3], origin_RF[4], origin_RF[5],
			origin_RF[6], origin_RF[7], origin_RF[8]) * R;

		// Transform the coordinate system into the classic x,y on plane,z upwards
		double x = msg->pose.position.x * origin_RF[0] + msg->pose.position.y * origin_RF[1] + msg->pose.position.z * origin_RF[2]; 
		double y = msg->pose.position.x * origin_RF[3] + msg->pose.position.y * origin_RF[4] + msg->pose.position.z * origin_RF[5]; 
		//double z = msg->pose.position.x * origin_RF[6] + msg->pose.position.y * origin_RF[7] + msg->pose.position.z * origin_RF[8]; 

		double roll, pitch, yaw;
		R.getEulerYPR(yaw, pitch, roll);

		if(this->enable_pose_init)
		{
			try
				{initProcedure(x,y,angleWithinPI(yaw));}
			catch(Error e)
				{printErrorCode(e);}
		}
		else
		{
			this->pose << x,y,angleWithinPI(yaw);
		}
    }

	/* It tracks a trajectory defined in terms of velocities. It has to set the current time 
	to extract the desired velocities and poses, then it computes the control with the stantdard
	controller and finally the effects of slippage are compensated by a transformer */
    void trackTrajectory(sensor_msgs::msg::JointState* msg_out)
    {
		Eigen::Vector2d u;
		Eigen::Vector2d u_bar;
		double t_now = getCurrentTime();
		double dt = t_now - t_start;

		// compute an estimation of the longitudinal and lateral slips
		Eigen::Vector3d pose_offset(0.0, 0.0, this->alpha_prev_1.data);
		Eigen::Vector3d pose_bar = pose + pose_offset; 
	
		Ctrl->setCurrentTime(dt);

		Ctrl->step(pose_bar, u_bar);
		std::cout<< std::endl;

		std::cout<<"u_bar control inputs: " << u_bar(0) << ", " << u_bar(1) << std::endl;
		// conversion block u = f(alpha, alpha_dot, u_bar) 
		convertskidSteering(u_bar, &u);
		std::cout<<"u control inputs: " << u(0) << ", " << u(1) << std::endl;
		std::cout<< std::endl;

		double v = u(0);
		double omega = u(1);
		// conversion block from unicycle to differential drive
		Model->setUnicycleSpeed(v, omega);
		double motor_vel_L = Model->getLeftMotorRotationalSpeed();
		double motor_vel_R = Model->getRightMotorRotationalSpeed();// mettere Wheel
		
		// compensate the longitudinal slip
		double motor_vel_comp_L = motor_vel_L;
		double motor_vel_comp_R = motor_vel_R;

		if(LONG_SLIP_ENABLED)
		{
			motor_vel_comp_L /= (1-this->i_L_prev);
			motor_vel_comp_R /= (1-this->i_R_prev);
		}

		std::cout<<"wheels control input: " << motor_vel_comp_L << ", " << motor_vel_comp_R << std::endl;
		msg_out->name = {"left_sprocket", "right_sprocket"};
		msg_out->velocity = std::vector<double>{
			motor_vel_comp_L, 
			motor_vel_comp_R
		};
		// estimate alpha, alpha_dot
		updateSlipsPrev(u);

    }

	void getTrackingErrorMsg(geometry_msgs::msg::Vector3Stamped* msg)
	{
		double e_x = Ctrl->getErrorX();
		double e_y = Ctrl->getErrorY();
		double e_th = Ctrl->getErrorTheta();

		msg->vector.x = e_x;
		msg->vector.y = e_y;
		msg->vector.z = e_th;

		msg->header.stamp = this->get_clock()->now();
		msg->header.frame_id = "world";
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
		std::vector<double> v_vec 		= this->get_parameter("v_des_mps").as_double_array();
		std::vector<double> omega_vec	= this->get_parameter("omega_des_radps").as_double_array();
		std::vector<double> x_vec		= this->get_parameter("x_des_m").as_double_array();
		std::vector<double> y_vec		= this->get_parameter("y_des_m").as_double_array();
		std::vector<double> theta_vec	= this->get_parameter("theta_des_rad").as_double_array();
		bool COPY_WHOLE_TRAJ  = this->get_parameter("copy_trajectory").as_bool();
		
		t_start = getCurrentTime();
		try{
			if(COPY_WHOLE_TRAJ)
			{
				this->addTrajectory(v_vec, omega_vec, x_vec, y_vec, theta_vec);
			}
			else
			{
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
		if(u(0) == 0.0 || u(1) == 0.0)
		{
			/* In this situation the vehicle is going straight
			*  or turning on the spot. In both cases there is no
			*  side slip angle
			*/
			return 0.0;
		}
		double a0,a1,a2;
		a0 = this->alpha_coeff.at(0);
		a1 = this->alpha_coeff.at(1);
		a2 = this->alpha_coeff.at(2);

		// model side slip as a plyinomial in the curvature
		double alpha = a0 + a1*pow((a2 + 1/R), 2);

		if(R > 0.0) // turning left
		{
			return alpha;
		}
		else
		{
			return -alpha;
		}
		
	}
	double computeLeftWheelLongSlip(const Eigen::Vector2d& u) const
	{
		double i_L, a0, a1;
		double R = computeTurningRadius(u(0), u(1));

		if(R >= 0.0) // turning left, left wheel is inner
		{
			a0 = this->i_inner_coeff.at(0);
			a1 = this->i_inner_coeff.at(1);
		}
		else
		{
			a0 = this->i_outer_coeff.at(0);
			a1 = this->i_outer_coeff.at(1);
		}
		if(abs(a1 + R) < this->long_slip_epsilon)
		{
			// close to singularity
			if(abs(a1 + R) * a0 > 0.0)
				i_L =  1.0;
			else
				i_L = -10.0;
		}
		else
		{
			i_L = a0 / (a1 + R);
		}
		return i_L;
	}

	double computeRightWheelLongSlip(const Eigen::Vector2d& u) const
	{
		double i_R, a0, a1;
		double R = computeTurningRadius(u(0), u(1));

		if(R < 0.0) // turning right, right wheel is inner
		{
			a0 = this->i_inner_coeff.at(0);
			a1 = this->i_inner_coeff.at(1);
		}
		else
		{
			a0 = this->i_outer_coeff.at(0);
			a1 = this->i_outer_coeff.at(1);
		}
		if(abs(a1 + R) < this->long_slip_epsilon)
		{
			// close to singularity
			if(abs(a1 + R) * a0 > 0.0)
				i_R =  1.0;
			else
				i_R = -10.0;
		}
		else
		{
			i_R = a0 / (a1 + R);
		}
		return i_R;
	}

	double computeAlphaDot() const
	{
		//double dt = this->alpha_prev_1.time - this->alpha_prev_2.time;

		double dt = 0.001;
		if(dt == 0.0){
			RCLCPP_ERROR(this->get_logger(), "dt = 0 in alpha derivative!");
			throw SINGULARITY_DT_ALPHA_DOT;
			}
		if(dt <= 0.0)
			throw NEGATIVE_DT_ALPHA_DOT;
		return (alpha_dot_prev * tau_derivative_filter + alpha_prev_1.data - alpha_prev_2.data) / (tau_derivative_filter + dt);
	}

	void updateSlipsPrev(const Eigen::Vector2d& u)
	{
		this->i_L_prev = computeLeftWheelLongSlip(u);
		this->i_R_prev = computeRightWheelLongSlip(u);
		this->alpha_prev_2.time = this->alpha_prev_1.time;
		this->alpha_prev_2.data = this->alpha_prev_1.data;
		this->alpha_prev_1.time = getCurrentTime();
		this->alpha_prev_1.data = computeSideSlipAngle(u);
		this->alpha_dot_prev = this->alpha_dot;
		std::cout << "i_L_prev " << i_L_prev << " i_R_prev " << i_R_prev << std::endl;
		std::cout << "alpha_prev_2.data " << this->alpha_prev_2.data << " alpha_prev_2.time " << this->alpha_prev_2.time << std::endl;
		std::cout << "alpha_prev_1.data " << this->alpha_prev_1.data << " alpha_prev_1.time " << this->alpha_prev_1.time << std::endl;

	}

	void convertskidSteering(const Eigen::Vector2d& u_bar, Eigen::Vector2d* u_out)
	{
		/* Convert control inputs to compensate for a skid-steering 
		*  vehicle
		*/
		this->alpha_dot = computeAlphaDot();

		double v_conv = u_bar(0) * cos(this->alpha_prev_1.data);
		double omega_conv = u_bar(1) + this->alpha_dot;
		*u_out << v_conv, omega_conv;
	}



public:
    SlippageControllerNode() : CoppeliaSimNode("lyapunov_slippage_controller")
    {
		double d,r,gear_ratio;
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

		// declare_parameter("verbose_level", 0); // 0:no print 1: only controller

		declare_parameter("long_slip_inner_coefficients", std::vector<double>({0.0,0.0,0.0}));
		declare_parameter("long_slip_outer_coefficients", std::vector<double>({0.0,0.0,0.0}));
		declare_parameter("side_slip_angle_coefficients", std::vector<double>({0.0,0.0,0.0}));
		declare_parameter("long_slip_singularity_epsilon", 0.01);

    	declare_parameter("v_des_mps", std::vector<double>({0.0,0.0}));
    	declare_parameter("omega_des_radps", std::vector<double>({0.0,0.0}));
    	declare_parameter("x_des_m", std::vector<double>({0.0,0.0})); // it is used only when copy_trajectory is true
    	declare_parameter("y_des_m", std::vector<double>({0.0,0.0})); // it is used only when copy_trajectory is true
    	declare_parameter("theta_des_rad", std::vector<double>({0.0,0.0})); // it is used only when copy_trajectory is true
		declare_parameter("origin_RF", std::vector<double>({1,0,0,0,1,0,0,0,1}));
		// true: the desired pose is set by the user as well as the desired control inputs
		// false: only the desired control inputs are specified by user, the pose is obtained via integration of
		//		of the unicycle kinematic model
		declare_parameter("copy_trajectory",     false); 
		
		
		get_parameter("track_distance_m", d);
		get_parameter("sprocket_radius_m", r);
		get_parameter("gearbox_ratio", gear_ratio);
		long_slip_epsilon = this->get_parameter("long_slip_singularity_epsilon").as_double();
		double Kp 		= this->get_parameter("Kp").as_double();
		double Ktheta 	= this->get_parameter("Ktheta").as_double();
		double dt 		= this->get_parameter("dt").as_double();
		int pub_dt 		= this->get_parameter("pub_dt_ms").as_int();
		this->enable_pose_init  = this->get_parameter("automatic_pose_init").as_bool();
		this->t_pose_init = this->get_parameter("time_for_pose_init_s").as_double();
		std::vector<double> pose_init   = this->get_parameter("pose_init_m_m_rad").as_double_array();

		i_inner_coeff = this->get_parameter("long_slip_inner_coefficients").as_double_array();
		i_outer_coeff = this->get_parameter("long_slip_outer_coefficients").as_double_array();
		alpha_coeff   = this->get_parameter("side_slip_angle_coefficients").as_double_array();

		origin_RF = this->get_parameter("origin_RF").as_double_array();
		tau_derivative_filter = 0.01; // [s]
		alpha_dot_prev = 0.0;
		alpha_dot = 0.0;

		alpha_prev_1.data = pub_dt / 1000.0; // store alpha value at time dt*(k-2)
		alpha_prev_1.time = 0.0; // store alpha value at time dt*(k-2)
		alpha_prev_2.data = 0.0; // store alpha value at time dt*(k-1)
		alpha_prev_2.time = 0.0; // store alpha value at time dt*(k-1)
		i_L_prev = 0.0; // store i_L value at time dt*(k-1)
		i_R_prev = 0.0; // store i_R value at time dt*(k-1)
		
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
			setupTrajectory();
		}
		
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos_optitrack = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, QUEUE_DEPTH_OPTITRACK), qos_profile);
		auto qos_ctrl = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

		pub_cmd = this->create_publisher<sensor_msgs::msg::JointState>("/command", qos_ctrl);
		pub_trk_error = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/tracking_error", qos_ctrl);
		
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

	void initProcedure(double x, double y, double yaw) /* Cannot find a way to use it*/
	{	
		if(getCurrentTime() - t_start > this->t_pose_init)
		{
			if(this->n_samples_pose_init == 0.0)
				throw NO_POSE_FEEDBACK_4_INIT;
			Ctrl->setStateOffset(x, y, yaw);
			this->enable_pose_init = false;
			std::cout << "Pose initialized as ["<< pose(0) << ", "<< pose(1) << ", "<< pose(2) << "]"<<std::endl;
			setupTrajectory();
			this->t_start = getCurrentTime();
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

