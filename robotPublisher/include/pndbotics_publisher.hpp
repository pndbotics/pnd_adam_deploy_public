#include "robotstatepub/msg/robot_state.hpp"
#include "robotstatepub/msg/joint_state_cmd.hpp"
#include "robotstatepub/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include "pnd_algorithm.h"

class RobotPublisher : public rclcpp::Node
{
public:
  RobotPublisher() : Node("pndboticstate")
  {
    // 订阅器
    jointcmd_sub_ = this->create_subscription<robotstatepub::msg::JointStateCmd>(
        "joint_state_cmd", 10, [this](const robotstatepub::msg::JointStateCmd::SharedPtr msg)
        {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_cmd_ = *msg;
        new_cmd_available_ = true; });

    // 发布器
    robotstate_pub_ = create_publisher<robotstatepub::msg::RobotState>("robot_state_actual", 10);
    imu_pub_ = create_publisher<robotstatepub::msg::Imu>("imu_data", 10);
  }

  void get_latest_command(RobotData &data)
  {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!new_cmd_available_)
          return;
  
      // 从第6个元素（索引5）开始赋值
      if (!latest_cmd_.dpos.empty()) {
          size_t size = latest_cmd_.dpos.size();
          data.q_d_.segment(6, size) = Eigen::VectorXd::Map(latest_cmd_.dpos.data(), size);
      }
      if (!latest_cmd_.dvel.empty()) {
          size_t size = latest_cmd_.dvel.size();
          data.q_dot_d_.segment(6, size) = Eigen::VectorXd::Map(latest_cmd_.dvel.data(), size);
      }
      if (!latest_cmd_.dtau.empty()) {
          size_t size = latest_cmd_.dtau.size();
          data.tau_d_.segment(6, size) = Eigen::VectorXd::Map(latest_cmd_.dtau.data(), size);
      }
  
      new_cmd_available_ = false;
  }

  void publish_robot_state(const RobotData &data)
  {
    robotstatepub::msg::RobotState msg;
    msg.apos = {data.q_a_.data(), data.q_a_.data() + data.q_a_.size()};
    msg.avel = {data.q_dot_a_.data(), data.q_dot_a_.data() + data.q_dot_a_.size()};
    msg.atau = {data.tau_a_.data(), data.tau_a_.data() + data.tau_a_.size()};
    robotstate_pub_->publish(msg);
  }

  void publish_imu(const RobotData& data) {
    robotstatepub::msg::Imu imu_msg;
    imu_msg.yaw = data.imu_data_[0];
    imu_msg.pitch = data.imu_data_[1];
    imu_msg.roll = data.imu_data_[2];
    imu_msg.angular_velocity = {
        data.imu_data_[3], 
        data.imu_data_[4], 
        data.imu_data_[5]
    };
    imu_msg.linear_acceleration = {
        data.imu_data_[6], 
        data.imu_data_[7], 
        data.imu_data_[8]
    };
    imu_pub_->publish(imu_msg);
  }

private:
  std::mutex mutex_;
  bool new_cmd_available_ = false;
  robotstatepub::msg::JointStateCmd latest_cmd_;
  rclcpp::Subscription<robotstatepub::msg::JointStateCmd>::SharedPtr jointcmd_sub_;
  rclcpp::Publisher<robotstatepub::msg::RobotState>::SharedPtr robotstate_pub_;
  rclcpp::Publisher<robotstatepub::msg::Imu>::SharedPtr imu_pub_;
};
