#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "unicycle_controller/coppeliaSimNode.h"
#include "unicycle_controller/differential_drive_model.h"
#include "unicycle_controller/error_codes.h"

#define NANO 0.000000001
#define QUEUE_DEPTH_OPTITRACK 2

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class LyapControllerNode : public CoppeliaSimNode
{
private:
    rclcpp::TimerBase::SharedPtr timer_cmd;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_cmd;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_trk_error;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;
    SlippageControllerPtr Ctrl;
	std::shared_ptr<DifferentialDriveModel> Model;
    Vector3_t pose;
    data_t t_start;
	data_t t_pose_init;
    std::vector<data_t> origin_RF;
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
		data_t x = msg->pose.position.x * origin_RF[0] + msg->pose.position.y * origin_RF[1] + msg->pose.position.z * origin_RF[2]; 
		data_t y = msg->pose.position.x * origin_RF[3] + msg->pose.position.y * origin_RF[4] + msg->pose.position.z * origin_RF[5]; 
		//data_t z = msg->pose.position.x * origin_RF[6] + msg->pose.position.y * origin_RF[7] + msg->pose.position.z * origin_RF[8]; 

		data_t roll, pitch, yaw;
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
		Vector2_t u;
		data_t t_now = getCurrentTime();
		data_t dt = t_now - t_start;

		// get alpha
		// compute desired pose (x,y,theta)_des by integration
		// set current P_bar = (x,y,theta)_act + (0,0,alpha)
		// P_bar input to Lyap controller -> u_bar
		// conversion block u = f(alpha, alpha_dot, u_bar) 

		Ctrl->setCurrentTime(dt);
		Ctrl->step(pose, &u);
		
		double v = u(0);
		double omega = u(1);
		Model->setUnicycleSpeed(v, omega);
		double motor_vel_left  = Model->getLeftMotorRotationalSpeed();
		double motor_vel_right = Model->getRightMotorRotationalSpeed();

		// conpensate with long slip model to get (w_l, w_r)_final
		msg_out->name = {"left_sprocket", "right_sprocket"};
		msg_out->velocity = std::vector<double>{
			motor_vel_left, 
			motor_vel_right
		};

		// estimate alpha, alpha_dot
    }

	void getTrackingErrorMsg(geometry_msgs::msg::Vector3Stamped* msg)
	{
		data_t e_x = LyapCtrl->getErrorX();
		data_t e_y = LyapCtrl->getErrorY();
		data_t e_th = LyapCtrl->getErrorTheta();

		msg->vector.x = e_x;
		msg->vector.y = e_y;
		msg->vector.z = e_th;

		msg->header.stamp = this->get_clock()->now();
		msg->header.frame_id = "world";
	}

	data_t getCurrentTime()
	{
		data_t t;
		if(this->isCoppeliaSimEnabled())
		{
			t = this->getSimTime();
		}
		else
		{
			t = data_t((this->get_clock()->now()).nanoseconds()) * NANO;
		}
		return t;
	}

	void updatePoseWithMovingAverage(data_t x, data_t y, data_t yaw)
	{
		// Current pose is treated as the current pose average
		// handle the periodicity of the angle
		data_t yaw_diff = this->pose(2) - yaw;
		data_t yaw_tmp = yaw;
		if(yaw_diff > M_PI)
		{
			yaw_tmp = this->pose(2) + (2*M_PI - yaw_diff);
		} 
		else if(yaw_diff < -M_PI)
		{
			yaw_tmp = this->pose(2) - (2*M_PI - yaw_diff);
		}

		// perform moving average
		data_t x_avg = this->pose(0) + (x - this->pose(0)) / 
			(this->n_samples_pose_init+1);
		data_t y_avg = this->pose(1) + (y - this->pose(1)) / 
			(this->n_samples_pose_init+1);
		data_t yaw_avg = this->pose(2) + (yaw_tmp - this->pose(2)) / 
			(this->n_samples_pose_init+1);

		// update values
		yaw_avg = angleWithinPI(yaw_avg);
		this->pose << x_avg, y_avg, yaw_avg;
		this->n_samples_pose_init += 1.0;
	}

	void initPoseMovingAverage(data_t x, data_t y, data_t yaw)
	{
		this->pose << x,y,angleWithinPI(yaw);
		this->n_samples_pose_init += 1.0;
	}

	void setupTrajectory()
	{
		std::vector<data_t> v_vec 		= this->get_parameter("v_des_mps").as_double_array();
		std::vector<data_t> omega_vec	= this->get_parameter("omega_des_radps").as_double_array();
		std::vector<data_t> x_vec		= this->get_parameter("x_des_m").as_double_array();
		std::vector<data_t> y_vec		= this->get_parameter("y_des_m").as_double_array();
		std::vector<data_t> theta_vec	= this->get_parameter("theta_des_rad").as_double_array();
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

	
public:
    LyapControllerNode() : CoppeliaSimNode("lyapunov_controller")
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
		declare_parameter("pose_init_m_m_rad", std::vector<data_t>({0.0,0.0,0.0}));
    	declare_parameter("v_des_mps", std::vector<data_t>({0.0,0.0}));
    	declare_parameter("omega_des_radps", std::vector<data_t>({0.0,0.0}));
    	declare_parameter("x_des_m", std::vector<data_t>({0.0,0.0})); // it is used only when copy_trajectory is true
    	declare_parameter("y_des_m", std::vector<data_t>({0.0,0.0})); // it is used only when copy_trajectory is true
    	declare_parameter("theta_des_rad", std::vector<data_t>({0.0,0.0})); // it is used only when copy_trajectory is true
		declare_parameter("origin_RF", std::vector<double>({1,0,0,0,1,0,0,0,1}));
		// true: the desired pose is set by the user as well as the desired control inputs
		// false: only the desired control inputs are specified by user, the pose is obtained via integration of
		//		of the unicycle kinematic model
		declare_parameter("copy_trajectory",     false); 
		
		
		get_parameter("track_distance_m", d);
		get_parameter("sprocket_radius_m", r);
		get_parameter("gearbox_ratio", gear_ratio);
		data_t Kp 		= this->get_parameter("Kp").as_double();
		data_t Ktheta 	= this->get_parameter("Ktheta").as_double();
		data_t dt 		= this->get_parameter("dt").as_double();
		int pub_dt 		= this->get_parameter("pub_dt_ms").as_int();
		this->enable_pose_init  = this->get_parameter("automatic_pose_init").as_bool();
		this->t_pose_init = this->get_parameter("time_for_pose_init_s").as_double();
		std::vector<data_t> pose_init   = this->get_parameter("pose_init_m_m_rad").as_double_array();
		origin_RF = this->get_parameter("origin_RF").as_double_array();

		
		LyapCtrl = std::make_shared<LyapController>(Kp, Ktheta, dt); 
		Model.reset(new DifferentialDriveModel(r, d, gear_ratio));
		
		n_samples_pose_init = 0.0;
		if(enable_pose_init)
		{
			this->pose << 0.0,0.0,0.0;
		}
		else
		{
			this->pose << pose_init.at(0), pose_init.at(1), pose_init.at(2);
			LyapCtrl->setStateOffset(pose_init.at(0), pose_init.at(1), pose_init.at(2));
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
			std::bind(&LyapControllerNode::poseCallback, this, std::placeholders::_1));
		timer_cmd = this->create_wall_timer(
			std::chrono::milliseconds(pub_dt), 
			std::bind(&LyapControllerNode::timerCmdCallback, this));
		if(this->isCoppeliaSimEnabled())
		{
			this->enableSyncWithCoppeliaSim();
			this->startCoppeliaSim();
			this->setCoppeliaSimRender();
		}
		
		t_start = getCurrentTime();
	}
    
    /* add the controls to generate poses via unicycle model*/
    void addControlsTrajectory(const std::vector<data_t>& v, const std::vector<data_t>& omega)
    {
		if(v.size() != omega.size())
		{
			throw TRAJECTORY_CTRL_INPUTS_DIFFERENT_SIZE;
		}
		for(int i = 0; i < int(v.size()); i++)
		{
			LyapCtrl->addToInputDesired(v.at(i), omega.at(i));
		}
		// saves time value such that the first control input is at delta time 0
		LyapCtrl->generateTrajectory();
		RCLCPP_INFO(this->get_logger(), "%s", (LyapCtrl->stringSetupInfo()).c_str());
    }

	/* add the trajectory defined as squence of controls and poses, no models are used*/
	void addTrajectory(const std::vector<data_t>& v, const std::vector<data_t>& omega, 
		const std::vector<data_t>& x, const std::vector<data_t>& y, const std::vector<data_t>& theta)
	{
		if(v.size() != omega.size())
		{
			throw TRAJECTORY_CTRL_INPUTS_DIFFERENT_SIZE;
		}
		if(x.size() != y.size() || theta.size() != y.size())
		{
			throw TRAJECTORY_POSE_DIFFERENT_SIZE;
		}
		LyapCtrl->copyTrajectory(v, omega, x,y,theta);
		RCLCPP_INFO(this->get_logger(), "%s", (LyapCtrl->stringSetupInfo()).c_str());
	}

	void initProcedure(data_t x, data_t y, data_t yaw) /* Cannot find a way to use it*/
	{	
		if(getCurrentTime() - t_start > this->t_pose_init)
		{
			if(this->n_samples_pose_init == 0.0)
				throw NO_POSE_FEEDBACK_4_INIT;
			LyapCtrl->setStateOffset(x, y, yaw);
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

	std::shared_ptr<LyapControllerNode> Ctrl(new LyapControllerNode());

	rclcpp::spin(Ctrl);

	if(Ctrl->isCoppeliaSimEnabled())
	{
		Ctrl->stopCoppeliaSim();
	}
	rclcpp::shutdown();
	printf("Lyapunov Controller node shutdown\n");

  return 0;
}

