#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/message_utils.hpp"
#include "booster_interface/msg/booster_api_req_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>
using namespace std::chrono_literals;

rclcpp::Client<booster_interface::srv::RpcService>::SharedPtr client;
std::shared_ptr<rclcpp::Node> node;
rclcpp::TimerBase::SharedPtr stop_timer;

void sendStopCommand() {
    if (!client->service_is_ready()) return;

    auto request = std::make_shared<booster_interface::srv::RpcService::Request>();
    request->msg = booster_interface::CreateMoveMsg(0.0, 0.0, 0.0);
    auto result = client->async_send_request(request);
    RCLCPP_INFO(node->get_logger(), "Stop command sent.");
}

void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!client->service_is_ready()) {
        RCLCPP_WARN(node->get_logger(), "Service not ready, skipping this cmd_vel");
        return;
    }
    RCLCPP_INFO(node->get_logger(), "Received cmd_vel: linear=(%.2f, %.2f), angular=%.2f",
                msg->linear.x, msg->linear.y, msg->angular.z);
      // 将 cmd_vel 转换为 SDK 指令
    float x = static_cast<float>(msg->linear.x);   // 前后
    float y = static_cast<float>(msg->linear.y);   // 左右
    float z = static_cast<float>(msg->angular.z);  // 角速度（旋转）
    if(x>0.5) {
        RCLCPP_WARN(node->get_logger(), "Command values out of range, skipping this cmd_vel");
        x = 0.5;
    }
    if(y>0.5) {
        RCLCPP_WARN(node->get_logger(), "Command values out of range, skipping this cmd_vel");
        y = 0.5;
    }
    // if(z>0.5) {
    //     RCLCPP_WARN(node->get_logger(), "Command values out of range, skipping this cmd_vel");
    //     z = 0.5;
    // }
    RCLCPP_INFO(node->get_logger(), "corrected cmd_vel: x=%.2f, y=%.2f, z=%.2f", x, y, z);

    auto request = std::make_shared<booster_interface::srv::RpcService::Request>();
    request->msg = booster_interface::CreateMoveMsg(
        x, y, z);
    // 发送移动命令
    client->async_send_request(request);
    RCLCPP_INFO(node->get_logger(), "Move command sent.");

    // // 创建一个定时器，2秒后发送停止命令
    // stop_timer = node->create_wall_timer(3s, [](){
    //     sendStopCommand();
    //     stop_timer->cancel();  // 只执行一次
    // });
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("rpc_client_node");
    
    // 1. 创建客户端
    client = node->create_client<booster_interface::srv::RpcService>(
            "booster_rpc_service");

    // auto request =
    //     std::make_shared<booster_interface::srv::RpcService::Request>();
 
    // auto req_move_msg = booster_interface::CreateMoveMsg(0.5, 0.0, 0.0);
    // auto req_move_msg_zero = booster_interface::CreateMoveMsg(0.0, 0.0, 0.0);


    // auto req_msg = booster_interface::CreateChangeModeMsg(booster::robot::RobotMode::kDamping);
    // auto req_msg = booster_interface::CreateLieDownMsg();
    
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "service not available, waiting again...");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service is available, ready to call.");

     // 订阅 /cmd_vel
    auto subscription = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, &cmdVelCallback);

    RCLCPP_INFO(node->get_logger(), "Ready to forward /cmd_vel to RPC service...");


    // if (rclcpp::ok()) {
    //     // start to move
    //     request->msg = req_move_msg;
    //     auto result = client->async_send_request(request);
    //     // Wait for the result.
    //     if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %s",
    //                     result.get()->msg.body.c_str());
    //     } else {
    //         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call rpc service");
    //     }
    //     std::this_thread::sleep_for(2s);

    //     // stop moving
    //     request->msg = req_move_msg_zero;
    //     result = client->async_send_request(request);
    //     // Wait for the result.
    //     if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %s",
    //                     result.get()->msg.body.c_str());
    //     } else {
    //         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call rpc service");
    //     }
    //     std::this_thread::sleep_for(2s);
    // }
    
    // Spin 主循环
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}