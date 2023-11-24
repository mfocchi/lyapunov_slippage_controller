#ifndef COPPELIASIM_NODE_H
#define COPPELIASIM_NODE_H

#include <cstdio>
#include <chrono>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

#define COPPELIASIM_STOPPED 0
#define COPPELIASIM_RUNNING 1
#define COPPELIASIM_PAUSED 2

using namespace std::chrono_literals;


class CoppeliaSimNode : public rclcpp::Node
{
protected:
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_sim_state;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_sim_time;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sim_start;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sim_stop;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sim_step;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sim_render;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sim_sync;

    rclcpp::TimerBase::SharedPtr timer_sim;

    bool enable_coppeliaSim;
    bool enable_sim_render;
    bool sim_state;
    int start_wait_time;
    int start_timeout;
    float sim_time;
    void timer_sim_callback() const;
    void runNextSimStep() const;
    void publishBoolROS(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub, bool val) const;
    void sim_state_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void sim_time_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void publishStartCoppeliaSim() {}
public:
    CoppeliaSimNode(const std::string& node_name);

	void startCoppeliaSim() const;
	void stopCoppeliaSim() const;
	void setCoppeliaSimRender() const;
    void disableSyncWithCoppeliaSim() const;
    void enableSyncWithCoppeliaSim() const;
	bool isCoppeliaSimEnabled() const {return enable_coppeliaSim;}
	bool isCoppeliaSimRunning() const {return sim_state == COPPELIASIM_RUNNING;}
    bool isCoppeliaSimStopped() const {return sim_state == COPPELIASIM_STOPPED;}

    float getSimTime() const {return sim_time;}
};


#endif