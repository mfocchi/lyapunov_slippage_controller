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
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "unicycle_controller/coppeliaSimNode.h"
#include "unicycle_controller/lyapunovController.h"

#define NANO 0.000000001
#define QUEUE_DEPTH_OPTITRACK 2

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class LyapControllerNode : public CoppeliaSimNode
{
private:
    rclcpp::TimerBase::SharedPtr timer_cmd;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;
    LyapControllerPtr LyapCtrl;
    Vector3_t pose;
    data_t t_start;
    std::vector<data_t> origin_RF;
	

	void timerCmdCallback()
    {
		geometry_msgs::msg::TwistStamped msg;
		trackTrajectory(&msg);
		pub_cmd->publish(msg);
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

		pose << x,y,angleWithinPI(yaw);
    }

	/* It tracks a trajectory defined in terms of velocities. */
    void trackTrajectory(geometry_msgs::msg::TwistStamped* msg_out)
    {
		Vector2_t u;
		data_t t_now = getCurrentTime();
		data_t dt = t_now - t_start;

		LyapCtrl->setCurrentTime(dt);
		LyapCtrl->step(pose, &u);

		msg_out->header.frame_id = "Doretta";
		msg_out->header.stamp    = this->get_clock()->now();
		msg_out->twist.linear.x  = u(0);
		msg_out->twist.angular.z = u(1);
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
	
public:
    LyapControllerNode() : CoppeliaSimNode("lyapunov_controller")
    {
		declare_parameter("Kp", 1.0);
		declare_parameter("Ktheta", 5.0);
		declare_parameter("dt", 0.1);
		declare_parameter("pub_dt_ms", 100);
		declare_parameter("pose_init_m_m_rad", std::vector<data_t>({0.0,0.0,0.0}));
    	declare_parameter("v_des_mps", std::vector<data_t>({0.0,0.0}));
    	declare_parameter("omega_des_radps", std::vector<data_t>({0.0,0.0}));
		declare_parameter("origin_RF", std::vector<double>({1,0,0,0,1,0,0,0,1}));
		declare_parameter("x_des_m", std::vector<data_t>({0.0,0.0})); // it is used only when copy_trajectory is true
    	declare_parameter("y_des_m", std::vector<data_t>({0.0,0.0})); // it is used only when copy_trajectory is true
    	declare_parameter("theta_des_rad", std::vector<data_t>({0.0,0.0})); // it is used only when copy_trajectory is true
		// true: the desired pose is set by the user as well as the desired control inputs
		// false: only the desired control inputs are specified by user, the pose is obtained via integration of
		//		of the unicycle kinematic model
		declare_parameter("copy_trajectory", false); 

		data_t Kp 		= this->get_parameter("Kp").as_double();
		data_t Ktheta 	= this->get_parameter("Ktheta").as_double();
		data_t dt 		= this->get_parameter("dt").as_double();
		int pub_dt 		= this->get_parameter("pub_dt_ms").as_int();
		std::vector<data_t> pose_init   = this->get_parameter("pose_init_m_m_rad").as_double_array();
		bool COPY_WHOLE_TRAJ  = this->get_parameter("copy_trajectory").as_bool();
		std::vector<data_t> v_vec 		= this->get_parameter("v_des_mps").as_double_array();
		std::vector<data_t> omega_vec	= this->get_parameter("omega_des_radps").as_double_array();
		std::vector<data_t> x_vec		= this->get_parameter("x_des_m").as_double_array();
		std::vector<data_t> y_vec		= this->get_parameter("y_des_m").as_double_array();
		std::vector<data_t> theta_vec	= this->get_parameter("theta_des_rad").as_double_array();
		origin_RF = this->get_parameter("origin_RF").as_double_array();

		this->pose << pose_init.at(0), pose_init.at(1), pose_init.at(2);

		LyapCtrl = std::make_shared<LyapController>(Kp, Ktheta, dt); 
		
		
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos_optitrack = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, QUEUE_DEPTH_OPTITRACK), qos_profile);
		auto qos_ctrl = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

		pub_cmd = this->create_publisher<geometry_msgs::msg::TwistStamped>("/vel_cmd", qos_ctrl);

		sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"/optiTrack/pose", 
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
		LyapCtrl->setStateOffset(pose_init.at(0), pose_init.at(1), pose_init.at(2));
		if(COPY_WHOLE_TRAJ)
		{
			this->addTrajectory(v_vec, omega_vec, x_vec, y_vec, theta_vec);
		}
		else
		{
			this->addControlsForTrajectory(v_vec, omega_vec);
		}		
	}
    
    /* add the controls to generate poses via unicycle model*/
    void addControlsForTrajectory(const std::vector<data_t>& v, const std::vector<data_t>& omega)
    {
		if(v.size() != omega.size())
		{
			std::cout << "ERROR: Control inputs sequence has different size!" << std::endl;
			return;
		}
		for(int i = 0; i < int(v.size()); i++)
		{
			LyapCtrl->addToInputDesired(v.at(i), omega.at(i));
		}
		// saves time value such that the first control input is at delta time 0
		LyapCtrl->generateTrajectory();
		t_start = getCurrentTime();
		RCLCPP_INFO(this->get_logger(), "%s", (LyapCtrl->stringSetupInfo()).c_str());
    }

	/* add the trajectory defined as squence of controls and poses, no models are used*/
	void addTrajectory(const std::vector<data_t>& v, const std::vector<data_t>& omega, 
		const std::vector<data_t>& x, const std::vector<data_t>& y, const std::vector<data_t>& theta)
	{
		if(v.size() != omega.size())
		{
			std::cout << "ERROR: Control inputs vectors have different size" << std::endl;
			return;
		}
		if(x.size() != y.size() || theta.size() != y.size())
		{
			std::cout << "ERROR: Desired poses vectors have different size" << std::endl;
			return;
		}
		LyapCtrl->copyTrajectory(v, omega, x,y,theta);
		t_start = getCurrentTime();
		RCLCPP_INFO(this->get_logger(), "%s", (LyapCtrl->stringSetupInfo()).c_str());
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

