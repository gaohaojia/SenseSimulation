
#include <cstring>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/utilities.hpp>

#include "communication_server/communication_server.hpp"

namespace communication_server {
CommunicationServerNode::CommunicationServerNode(
  const rclcpp::NodeOptions& options)
    : Node("communication_server", options) {
  rclcpp::QoS clock_qos(rclcpp::KeepLast(5));
  clock_qos.best_effort();
  clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock", 5,
    std::bind(&CommunicationServerNode::ClockCallBack, this,
              std::placeholders::_1));

  clock_pub_ = this->create_publisher<builtin_interfaces::msg::Time>(
    "/changeable_clock", clock_qos);
}

void CommunicationServerNode::ClockCallBack(
  const rosgraph_msgs::msg::Clock::ConstSharedPtr clock_msg) {
  builtin_interfaces::msg::Time time_msg;
  time_msg = clock_msg->clock;
  clock_pub_->publish(time_msg);
}

}  // namespace communication_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(communication_server::CommunicationServerNode)