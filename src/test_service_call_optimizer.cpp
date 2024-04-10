#include "rclcpp/rclcpp.hpp"
#include "optim_interfaces/srv/optim.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto optimNode = rclcpp::Node::make_shared("optimizer_client");

    auto client = optimNode->create_client<optim_interfaces::srv::Optim>("/optim");

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(optimNode->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 1;
        }
        RCLCPP_INFO(optimNode->get_logger(), "Service not available, retrying...");
    }

    auto request = std::make_shared<optim_interfaces::srv::Optim::Request>();
    
    request->x0 = 0.0;
    request->y0 = 0.0;
    request->theta0 = -0.0;
    request->xf = -0.4758;
    request->yf = -1.1238;
    request->thetaf = 0.9638;
    request->plan_type = "optim";

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(optimNode, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result.get();
        if(response){
            for(size_t i = 0; i < response->des_x.size(); ++i){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Step %zu", i);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "X[%zu]: %.2f", i, response->des_x[i]);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Y[%zu]: %.2f", i, response->des_y[i]);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Theta[%zu]: %.2f", i, response->des_theta[i]);
            }
        } else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response has failed");
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /optim");
    }

    
    rclcpp::shutdown();

    return 0;
}
