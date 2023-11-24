#include "coppeliaSimNode.h"

CoppeliaSimNode::CoppeliaSimNode(const std::string& node_name)
    : Node(node_name)
{
    declare_parameter("enable_coppeliasim", false);
    declare_parameter("sim_dt_ms", 100);
    declare_parameter("enable_sim_render",  false);
    declare_parameter("start_wait_time_ms",  100);
    declare_parameter("start_timeout_ms" ,   1000);
    int sim_dt = this->get_parameter("sim_dt_ms").as_int();
    enable_sim_render  = this->get_parameter("enable_sim_render").as_bool(); 
    enable_coppeliaSim = this->get_parameter("enable_coppeliasim").as_bool(); 
    start_wait_time    = this->get_parameter("start_wait_time_ms").as_int(); 
    start_timeout      = this->get_parameter("start_timeout_ms").as_int(); 
    sim_state = COPPELIASIM_STOPPED;
    sim_time  = 0.0;
    if(this->isCoppeliaSimEnabled())
    {
        sub_sim_state = this->create_subscription<std_msgs::msg::Int32>(
			"/simulationState", 
			10, 
			std::bind(&CoppeliaSimNode::sim_state_callback, this, std::placeholders::_1));
		
        sub_sim_time = this->create_subscription<std_msgs::msg::Float32>(
			"/simulationTime", 
			10, 
			std::bind(&CoppeliaSimNode::sim_time_callback, this, std::placeholders::_1));
		

        pub_sim_start = this->create_publisher<std_msgs::msg::Bool>("/startSimulation", 10); 
        pub_sim_stop  = this->create_publisher<std_msgs::msg::Bool>("/stopSimulation", 10); 
        pub_sim_step  = this->create_publisher<std_msgs::msg::Bool>("/triggerNextStep", 10); 
        pub_sim_render= this->create_publisher<std_msgs::msg::Bool>("/renderSimulation", 10); 
        pub_sim_sync  = this->create_publisher<std_msgs::msg::Bool>("/enableSyncMode", 10); 
        
        timer_sim = this->create_wall_timer(
            std::chrono::milliseconds(sim_dt), 
            std::bind(&CoppeliaSimNode::timer_sim_callback, this));
    }
}

void CoppeliaSimNode::sim_state_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    sim_state = msg->data;
}
void CoppeliaSimNode::sim_time_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    sim_time = msg->data;
}

void CoppeliaSimNode::timer_sim_callback() const
{
    runNextSimStep();
}

void CoppeliaSimNode::runNextSimStep() const
{
    publishBoolROS(pub_sim_step, true);
}

void CoppeliaSimNode::startCoppeliaSim() const
{   
    for(int t = 0; 
        t <  start_timeout && !isCoppeliaSimRunning(); 
        t += start_wait_time)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(start_wait_time));
        publishBoolROS(pub_sim_start, true);
    }
}

void CoppeliaSimNode::stopCoppeliaSim() const
{
    publishBoolROS(pub_sim_stop, false);
}

void CoppeliaSimNode::disableSyncWithCoppeliaSim() const
{
    publishBoolROS(pub_sim_sync, false);
}

void CoppeliaSimNode::enableSyncWithCoppeliaSim() const
{
    publishBoolROS(pub_sim_sync, true);
}

void CoppeliaSimNode::setCoppeliaSimRender() const
{
    publishBoolROS(pub_sim_render, enable_sim_render);
}

void CoppeliaSimNode::publishBoolROS(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub, bool val) const
{
    std_msgs::msg::Bool msg;
    msg.data = val;
    pub->publish(msg);
}

