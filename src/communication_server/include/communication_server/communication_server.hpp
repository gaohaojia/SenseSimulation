#ifndef COMMUNICATION_SERVER_HPP
#define COMMUNICATION_SERVER_HPP

#include <netinet/in.h>

#include <cstdint>
#include <memory>
#include <queue>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#define MAX_ROBOT_COUNT 5

namespace communication_server
{
class CommunicationServerNode : public rclcpp::Node
{
public:
  CommunicationServerNode(const rclcpp::NodeOptions & options);
  ~CommunicationServerNode() override;

private:
  int port;
  std::string ip;
  int sockfd;
  struct sockaddr_in server_addr, saved_client_addr[MAX_ROBOT_COUNT];

  int robot_count = 3;

  std::thread send_thread_;
  std::thread recv_thread_;
  std::thread prepare_buffer_thread_;
  std::thread parse_buffer_thread_[MAX_ROBOT_COUNT];

  struct SendBuffer
  {
    int id;
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
  std::queue<std::vector<uint8_t>> parse_buffer_queue[MAX_ROBOT_COUNT];

  void InitServer();

  void NetworkSendThread();
  void NetworkRecvThread();
  void PrepareBufferThread();
  void ParseBufferThread(const int robot_id);

  void LivoxScanCallBack(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr livox_scan_msg,
                         const int robot_id);
  void LivoxImuCallBack(const sensor_msgs::msg::Imu::ConstSharedPtr livox_imu_msg,
                        const int robot_id);

  template <class T> std::vector<uint8_t> SerializeMsg(const T & msg);
  template <class T> T DeserializeMsg(const std::vector<uint8_t> & data);

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_scan_sub_[MAX_ROBOT_COUNT];
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr livox_imu_sub_[MAX_ROBOT_COUNT];

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_[MAX_ROBOT_COUNT];
};
} // namespace communication_server

#endif