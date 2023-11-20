#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "../../robot_model/include/robot_model/motionModels.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "unicycle_controller/coppeliaSimNode.h"


using namespace std::chrono_literals;

class OpenLoopUnicycleNode : public CoppeliaSimNode
{
private:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd;
    rclcpp::TimerBase::SharedPtr timer_cmd;
    std::vector<data_t> longitudinal_velocity;
    std::vector<data_t> angular_velocity;
    size_t count_; 
    long iter;

    void timer_cmd_callback()
    {
        if(iter >= long(angular_velocity.size()) || iter >= long(longitudinal_velocity.size()))
        {
            return;
        }
        geometry_msgs::msg::TwistStamped msg_vel;
        msg_vel.header.frame_id = "Doretta";
		msg_vel.header.stamp    = Node::get_clock()->now();
        msg_vel.twist.linear.x  = longitudinal_velocity.at(iter);
        msg_vel.twist.angular.z = angular_velocity.at(iter);

		pub_cmd->publish(msg_vel);
        iter++;
    }

public:
    OpenLoopUnicycleNode() : CoppeliaSimNode("open_loop_unicycle")
    {
        declare_parameter("pub_dt_ms", 200);
    	declare_parameter("v_des_mps", std::vector<data_t>({0.0,0.0}));
    	declare_parameter("omega_des_radps", std::vector<data_t>({0.0,0.0}));

		int pub_dt 		        = this->get_parameter("pub_dt_ms").as_int();
		longitudinal_velocity   = this->get_parameter("v_des_mps").as_double_array();
		angular_velocity	    = this->get_parameter("omega_des_radps").as_double_array();

		pub_cmd = this->create_publisher<geometry_msgs::msg::TwistStamped>("/vel_cmd", 10); 

		timer_cmd = this->create_wall_timer(
			std::chrono::milliseconds(pub_dt), 
			std::bind(&OpenLoopUnicycleNode::timer_cmd_callback, this));
        
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
	std::shared_ptr<OpenLoopUnicycleNode> Ctrl(new OpenLoopUnicycleNode());

	rclcpp::spin(Ctrl);

	if(Ctrl->isCoppeliaSimEnabled())
	{
		Ctrl->stopCoppeliaSim();
	}
	rclcpp::shutdown();
	printf("Open loop velocity publisher node shutdown\n");

  return 0;
}
