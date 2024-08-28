#ifndef COMMUNICATION_CLIENT_HPP
#define COMMUNICATION_CLIENT_HPP

#include <cstdint>
#include <memory>
#include <netinet/in.h>
#include <queue>
#include <thread>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

namespace communication_client
{
class CommunicationClientNode : public rclcpp::Node
{
public:
  CommunicationClientNode(const rclcpp::NodeOptions & options);
  ~CommunicationClientNode() override;

private:
  int port;
  std::string ip;
  int sockfd;
  struct sockaddr_in server_addr;

  int robot_id;

  std::thread send_thread_;
  std::thread recv_thread_;
  std::thread heart_beat_thread_;
  std::thread prepare_buffer_thread_;
  std::thread parse_buffer_thread_;

  struct SendBuffer
  {
    std::vector<uint8_t> buffer;
  };
  std::queue<SendBuffer> buffer_queue;

  struct PrepareBuffer
  {
    int id;
    std::vector<uint8_t> buffer;
    int msg_type;
  };
  std::queue<PrepareBuffer> prepare_buffer_queue;
  std::queue<std::vector<uint8_t>> parse_buffer_queue;

  void InitClient();

  void NetworkSendThread();
  void NetworkRecvThread();
  void HeartBeatThread();
  void PrepareBufferThread();
  void ParseBufferThread();

  void CmdVelCallBack(const geometry_msgs::msg::TwistStamped::ConstSharedPtr cmd_vel_msg);

  livox_ros_driver2::msg::CustomMsg ConvertPointCloud2ToCustomMsg(const sensor_msgs::msg::PointCloud2 &input_cloud);

  template <class T> std::vector<uint8_t> SerializeMsg(const T & msg);
  template <class T> T DeserializeMsg(const std::vector<uint8_t> & data);

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;

  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr livox_imu_pub_;
};
} // namespace communication_client

#endif