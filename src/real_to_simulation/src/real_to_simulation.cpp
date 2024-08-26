#include "real_to_simulation/real_to_simulation.hpp"
#include <memory>

namespace real_to_simulation
{
RealToSimulationNode::RealToSimulationNode(const rclcpp::NodeOptions & options)
  : Node("real_to_simulation", options)
{
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 5, std::bind(&RealToSimulationNode::CmdVelCallback, this, std::placeholders::_1));
  cmd_vel_sim_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_sim", 10);
}

void RealToSimulationNode::CmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
  std::shared_ptr<geometry_msgs::msg::Twist> cmd_vel_sim = std::make_shared<geometry_msgs::msg::Twist>();
  cmd_vel_sim->linear.x = msg->twist.linear.x;
  cmd_vel_sim->linear.y = msg->twist.linear.y;
  cmd_vel_sim->angular.z = msg->twist.angular.z;
  cmd_vel_sim_pub_->publish(*cmd_vel_sim);
}

} // namespace real_to_simulation

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(real_to_simulation::RealToSimulationNode)