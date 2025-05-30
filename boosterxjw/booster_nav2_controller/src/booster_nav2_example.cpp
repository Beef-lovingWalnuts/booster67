#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
// #include <nav2_msgs/action/navigate_to_pose.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "booster/robot/b1/b1_loco_client.hpp"

using namespace std::chrono_literals;

enum class RobotState {
  INIT,
  DAMPING,
  PREPARE,
  WALK,
  ERROR
};

class B1Controller : public rclcpp::Node {
public:
  B1Controller(const std::string& net_interface) 
  : Node("b1_controller"), net_interface_(net_interface)  {
    // 初始化网络通道
    booster::robot::ChannelFactory::Instance()->Init(0, net_interface_.c_str());
    client_.Init();
    initRobot();  // 初始化机器人
    // 订阅nav2话题 /cmd_vel
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&B1Controller::cmdVelCallback, this, std::placeholders::_1));

    // // 动作客户端用于导航目标
    // nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    //   this, "navigate_to_pose");

    // 定时器检查目标状态
    timer_ = create_wall_timer(500ms, [this]() { this->checkNavigationStatus(); });
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (robot_state_ != RobotState::WALK) {
      RCLCPP_WARN(this->get_logger(), "机器人未进入 WALK 模式，忽略运动指令");
      return;
    }
    // 将 cmd_vel 转换为 SDK 指令
    float x = static_cast<float>(msg->linear.x);   // 前后
    float y = static_cast<float>(msg->linear.y);   // 左右
    float z = static_cast<float>(msg->angular.z);  // 角速度（旋转）

    int32_t res = client_.Move(x, y, z);
    if (res != 0) {
        RCLCPP_ERROR(this->get_logger(), "Move command failed: %d", res);
    } else {
        RCLCPP_INFO(this->get_logger(), "Move: x=%.2f y=%.2f z=%.2f", x, y, z);
    }

    // 更新最后命令时间
    last_cmd_time_ = now();
    is_moving_ = true;           // 标志设为正在动
  }

  void checkNavigationStatus() {
    // if (!nav_goal_handle_) return;

    // auto status = nav_goal_handle_->get_status();
    // if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    //   RCLCPP_INFO(get_logger(), "Goal reached! Stopping robot.");
    //   client_.Move(0.0, 0.0, 0.0); // 停止运动
    //   nav_goal_handle_.reset();
    // }

      if (!is_moving_) return;

      auto time_now = now();
      rclcpp::Duration delta = time_now - last_cmd_time_;

      if (delta.seconds() > 2.0) {
        RCLCPP_INFO(get_logger(), "No cmd_vel for 2s, stopping robot.");
        client_.Move(0.0, 0.0, 0.0);
        is_moving_ = false;
      }
  }

  void initRobot() {
    RCLCPP_INFO(this->get_logger(), "初始化机器人中...");
    // 1. 切换到阻尼模式
    if (client_.ChangeMode(booster::robot::RobotMode::kDamping) != 0) {
      RCLCPP_ERROR(this->get_logger(), "切换到阻尼模式失败");
      robot_state_ = RobotState::ERROR;
      return;
    }
    robot_state_ = RobotState::DAMPING;
    rclcpp::sleep_for(1s);  // 等待模式切换完成
    // 2. 切换到准备模式
    if (client_.ChangeMode(booster::robot::RobotMode::kPrepare) != 0) {
      RCLCPP_ERROR(this->get_logger(), "切换到准备模式失败");
      robot_state_ = RobotState::ERROR;
      return;
    }
    robot_state_ = RobotState::PREPARE;
    rclcpp::sleep_for(1s);  // 等待模式切换完成
    // 3. 切换到行走模式
    if (client_.ChangeMode(booster::robot::RobotMode::kWalking) != 0) {
      RCLCPP_ERROR(this->get_logger(), "切换到行走模式失败");
      robot_state_ = RobotState::ERROR;
      return;
    }
    robot_state_ = RobotState::WALK;
    rclcpp::sleep_for(1s);  // 等待模式切换完成 
    RCLCPP_INFO(this->get_logger(), "机器人已进入行走模式");
  }


  // IP 地址
  std::string net_interface_;
  // SDK 客户端
  booster::robot::b1::B1LocoClient client_;
  
  // ROS 组件
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  // rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  // rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr nav_goal_handle_;
  // tf2_ros::Buffer tf_buffer_;
  // tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_cmd_time_;

  // 机器人状态
  bool is_moving_ = false;
  RobotState robot_state_ = RobotState::INIT;
};

int main(int argc, char** argv) {
  if (argc < 2) {
      std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
      exit(-1);
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<B1Controller>(argv[1]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}