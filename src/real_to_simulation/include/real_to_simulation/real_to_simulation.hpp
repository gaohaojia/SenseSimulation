#ifndef REAL_TO_SIMULATION_HPP
#define REAL_TO_SIMULATION_HPP

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace real_to_simulation
{
class RealToSimulationNode : public rclcpp::Node
{
public:
  RealToSimulationNode(const rclcpp::NodeOptions & options);

private:

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sim_pub_;

  void CmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

};
} // namespace real_to_simulation

#endif