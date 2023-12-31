#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "lyapunov_slippage_controller/coppeliaSimNode.h"
#include "lyapunov_slippage_controller/differential_drive_model.h"
#include "lyapunov_slippage_controller/motionModels.h"


using namespace std::chrono_literals;

class DifferentialDriveOpenLoopNode : public CoppeliaSimNode
{
private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_cmd;
    rclcpp::TimerBase::SharedPtr timer_cmd;
    std::vector<double> longitudinal_velocity;
    std::vector<double> angular_velocity;
	std::shared_ptr<DifferentialDriveModel> Model;
    size_t count_; 
    long iter;

    void timer_cmd_callback()
    {
        if(iter >= long(angular_velocity.size()) || iter >= long(longitudinal_velocity.size()))
        {
            return;
        }
        sensor_msgs::msg::JointState msg_cmd;
        double v = longitudinal_velocity.at(iter);
        double omega = angular_velocity.at(iter);
        Model->setUnicycleSpeed(v, omega);
		double motor_vel_left  = Model->getLeftMotorRotationalSpeed();
		double motor_vel_right = Model->getRightMotorRotationalSpeed();

		msg_cmd.name = {"left_sprocket", "right_sprocket"};
		msg_cmd.velocity = std::vector<double>{
			motor_vel_left, 
			motor_vel_right
		};
        
		pub_cmd->publish(msg_cmd);
        iter++;
    }

public:
    DifferentialDriveOpenLoopNode() : CoppeliaSimNode("differential_drive_open_loop")
    {
        double d,r,gear_ratio;
		declare_parameter("track_distance_m", 0.606);
		declare_parameter("sprocket_radius_m", 0.0979);
		declare_parameter("gearbox_ratio", 39.5);
        declare_parameter("pub_dt_ms", 200);
    	declare_parameter("v_des_mps", std::vector<double>({0.0,0.0}));
    	declare_parameter("omega_des_radps", std::vector<double>({0.0,0.0}));

        get_parameter("track_distance_m", d);
		get_parameter("sprocket_radius_m", r);
		get_parameter("gearbox_ratio", gear_ratio);
		int pub_dt 		        = get_parameter("pub_dt_ms").as_int();
		longitudinal_velocity   = get_parameter("v_des_mps").as_double_array();
		angular_velocity	    = get_parameter("omega_des_radps").as_double_array();

		Model.reset(new DifferentialDriveModel(r, d, gear_ratio));

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos_ctrl = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

		pub_cmd = this->create_publisher<sensor_msgs::msg::JointState>("/command", qos_ctrl);

		timer_cmd = this->create_wall_timer(
			std::chrono::milliseconds(pub_dt), 
			std::bind(&DifferentialDriveOpenLoopNode::timer_cmd_callback, this));
        
        if(this->isCoppeliaSimEnabled())
		{
			this->enableSyncWithCoppeliaSim();
			this->startCoppeliaSim();
			this->setCoppeliaSimRender();
		}
        iter = 0;
    }
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<DifferentialDriveOpenLoopNode> Ctrl(new DifferentialDriveOpenLoopNode());

	rclcpp::spin(Ctrl);

	if(Ctrl->isCoppeliaSimEnabled())
	{
		Ctrl->stopCoppeliaSim();
	}
	rclcpp::shutdown();
	printf("Open loop velocity publisher node shutdown\n");

  return 0;
}
