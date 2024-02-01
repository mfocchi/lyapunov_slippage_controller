#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "lyapunov_slippage_controller/motionModels.h"
#include  <iomanip>

#define NANO 0.000000001
#define QUEUE_DEPTH 2

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class feedbackTestNode : public rclcpp::Node
{
private:
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_stamped;
    std::vector<double> origin_RF;
    rclcpp::TimerBase::SharedPtr timer;

	double x;
	double y;
	double z;
	double yaw_deg;
	double pitch_deg;
	double roll_deg;

    void pose_stamped_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
		tf2::Quaternion q(
			msg->pose.orientation.x,
			msg->pose.orientation.y,
			msg->pose.orientation.z,
			msg->pose.orientation.w);
		tf2::Matrix3x3 R(q);
		double roll, pitch, yaw;
		// multiply by origin_RF
		R = tf2::Matrix3x3(
			origin_RF[0], origin_RF[1], origin_RF[2],
			origin_RF[3], origin_RF[4], origin_RF[5],
			origin_RF[6], origin_RF[7], origin_RF[8]) * R;

		R.getEulerYPR(yaw, pitch, roll);

        yaw_deg = angleWithinPI(yaw) * 180.0 / M_PI;
        pitch_deg = angleWithinPI(pitch) * 180.0 / M_PI;
        roll_deg = angleWithinPI(roll) * 180.0 / M_PI;
		x = msg->pose.position.x * origin_RF[0] + msg->pose.position.y * origin_RF[1] + msg->pose.position.z * origin_RF[2]; 
		y = msg->pose.position.x * origin_RF[3] + msg->pose.position.y * origin_RF[4] + msg->pose.position.z * origin_RF[5]; 
		z = msg->pose.position.x * origin_RF[6] + msg->pose.position.y * origin_RF[7] + msg->pose.position.z * origin_RF[8];
    }

public:
    feedbackTestNode() : Node("tester_fbk")
    {
		declare_parameter("origin_RF", std::vector<double>({1,0,0,0,0,-1,0,1,0}));
		origin_RF = this->get_parameter("origin_RF").as_double_array();
		
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, QUEUE_DEPTH), qos_profile);
		sub_stamped = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"/optiTrack/pose", 
			qos, 
			std::bind(&feedbackTestNode::pose_stamped_callback, this, std::placeholders::_1));

		timer = this->create_wall_timer(
			std::chrono::milliseconds(500), 
			std::bind(&feedbackTestNode::printPose, this));
		
		x = 0.0;
		y = 0.0;
		z = 0.0;
		yaw_deg = 0.0;
		pitch_deg = 0.0;
		roll_deg = 0.0;
	}

	void printPose()
	{
		std::cout << std::fixed << std::setprecision(2);
		std::cout << "POSITION [m]     | x="  <<x<<      " \t| y=    "<<y<<        " \t| z="<< z <<std::endl;
		std::cout << std::fixed << std::setprecision(1);
        std::cout << "ORIENTATION [deg]| yaw="<<yaw_deg<<" \t| pitch="<<pitch_deg<<" \t| roll="<<roll_deg<<std::endl;
		std::cout << std::fixed << std::setprecision(3);
		std::cout << "[" << x << ", " << y << ", " << yaw_deg * M_PI / 180.0 << "]" << std::endl;
		std::cout << "------------------------------------------------------------------------" << std::endl;
	}
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	printf("Optitrack feedback test node starting\n");

	std::shared_ptr<feedbackTestNode> Test(new feedbackTestNode());
	rclcpp::spin(Test);

	rclcpp::shutdown();
	printf("Optitrack feedback test node shutdown\n");

  return 0;
}

