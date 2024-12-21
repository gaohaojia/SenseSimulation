#ifndef COMMUNICATION_SERVER_HPP
#define COMMUNICATION_SERVER_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdint>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <memory>
#include <queue>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include "rosgraph_msgs/msg/clock.hpp"
#include <vector>

namespace communication_server {
class CommunicationServerNode : public rclcpp::Node {
 public:
  CommunicationServerNode(const rclcpp::NodeOptions& options);

 private:
  int robot_id;

  void ClockCallBack(
    const rosgraph_msgs::msg::Clock::ConstSharedPtr time_msg);

  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;

  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr clock_pub_;
};
}  // namespace communication_server

#endif