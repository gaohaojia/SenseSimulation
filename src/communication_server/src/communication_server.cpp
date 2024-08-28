#include <arpa/inet.h>
#include <netinet/in.h>
#include <rclcpp/logging.hpp>
#include <rmw/qos_profiles.h>
#include <rmw/rmw.h>
#include <rmw/types.h>
#include <sys/socket.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/utilities.hpp>
#include <string>

#include "communication_server/communication_server.hpp"

#define MAX_PACKET_SIZE 64000
#define BUFFER_SIZE 65535
#define MAX_BUFFER_QUEUE_SIZE 256

namespace communication_server
{
CommunicationServerNode::CommunicationServerNode(const rclcpp::NodeOptions & options)
  : Node("communication_server", options)
{
  this->declare_parameter<int>("robot_count", 3);
  this->declare_parameter<int>("network_port", 12131);
  this->declare_parameter<std::string>("network_ip", "192.168.31.207");

  this->get_parameter("robot_count", robot_count);
  this->get_parameter("network_port", port);
  this->get_parameter("network_ip", ip);

  for (int i = 0; i < robot_count; i++) {
    livox_scan_sub_[i] = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/robot_" + std::to_string(i) + "/livox/lidar",
      5,
      [this, i](const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
        LivoxScanCallBack(msg, i);
      });
    livox_imu_sub_[i] = this->create_subscription<sensor_msgs::msg::Imu>(
      "/robot_" + std::to_string(i) + "/livox/imu",
      5,
      [this, i](const sensor_msgs::msg::Imu::ConstSharedPtr msg) { LivoxImuCallBack(msg, i); });
    cmd_vel_pub_[i] = this->create_publisher<geometry_msgs::msg::Twist>(
      "/robot_" + std::to_string(i) + "/cmd_vel_sim", 2);
  }

  InitServer();
}

CommunicationServerNode::~CommunicationServerNode()
{
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (prepare_buffer_thread_.joinable()) {
    prepare_buffer_thread_.join();
  }
  for (int i = 0; i < robot_count; i++) {
    if (parse_buffer_thread_[i].joinable()) {
      parse_buffer_thread_[i].join();
    }
  }
  close(sockfd);
}

// initialize the socket client
void CommunicationServerNode::InitServer()
{
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
    return;
  }

  memset(&server_addr, 0, sizeof(server_addr));
  memset(&saved_client_addr, 0, sizeof(saved_client_addr));

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  server_addr.sin_addr.s_addr = inet_addr(ip.c_str());

  if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Bind failed!");
    close(sockfd);
    return;
  }

  send_thread_ = std::thread(&CommunicationServerNode::NetworkSendThread, this);
  for (int i = 0; i < robot_count; i++) {
    parse_buffer_thread_[i] = std::thread(&CommunicationServerNode::ParseBufferThread, this, i);
  }
  prepare_buffer_thread_ = std::thread(&CommunicationServerNode::PrepareBufferThread, this);
  recv_thread_ = std::thread(&CommunicationServerNode::NetworkRecvThread, this);
  RCLCPP_INFO(this->get_logger(), "Server start at ip: %s, port: %d", ip.c_str(), port);
}

void CommunicationServerNode::LivoxScanCallBack(
  const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr livox_scan_msg, const int robot_id)
{
  std::vector<uint8_t> data_buffer = SerializeMsg<livox_ros_driver2::msg::CustomMsg>(*livox_scan_msg);
  PrepareBuffer pthread_buffer = {robot_id, data_buffer, 0};
  if (prepare_buffer_queue.size() >= MAX_BUFFER_QUEUE_SIZE) {
    return;
  }
  prepare_buffer_queue.push(pthread_buffer);
}

void CommunicationServerNode::LivoxImuCallBack(
  const sensor_msgs::msg::Imu::ConstSharedPtr livox_imu_msg, const int robot_id)
{
  std::vector<uint8_t> data_buffer = SerializeMsg<sensor_msgs::msg::Imu>(*livox_imu_msg);
  PrepareBuffer pthread_buffer = {robot_id, data_buffer, 1};
  if (prepare_buffer_queue.size() >= MAX_BUFFER_QUEUE_SIZE) {
    return;
  }
  prepare_buffer_queue.push(pthread_buffer);
}

void CommunicationServerNode::NetworkSendThread()
{
  while (rclcpp::ok()) {
    if (buffer_queue.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(1));
      continue;
    }
    SendBuffer s_buffer = buffer_queue.front();
    buffer_queue.pop();
    if (saved_client_addr[s_buffer.id].sin_addr.s_addr == 0) {
      continue;
    }
    if (sendto(sockfd,
               s_buffer.buffer.data(),
               s_buffer.buffer.size(),
               0,
               (const struct sockaddr *)&saved_client_addr[s_buffer.id],
               sizeof(saved_client_addr[s_buffer.id])) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Send failed!");
    }
  }
}

void CommunicationServerNode::NetworkRecvThread()
{
  struct sockaddr_in client_addr;
  memset(&client_addr, 0, sizeof(client_addr));
  int n, len = sizeof(client_addr);
  while (rclcpp::ok()) {
    std::vector<uint8_t> buffer_tmp(BUFFER_SIZE);
    n = recvfrom(sockfd,
                 buffer_tmp.data(),
                 BUFFER_SIZE,
                 MSG_WAITALL,
                 (struct sockaddr *)&client_addr,
                 (socklen_t *)&len);
    if (n < 0) {
      continue;
    }
    buffer_tmp.resize(n);

    uint8_t id;
    std::memcpy(&id, buffer_tmp.data(), sizeof(id));
    saved_client_addr[id] = client_addr;
    if (parse_buffer_queue[id].size() >= MAX_BUFFER_QUEUE_SIZE) {
      continue;
    }
    parse_buffer_queue[id].push(buffer_tmp);
  }
}

void CommunicationServerNode::PrepareBufferThread()
{
  while (rclcpp::ok()) {
    if (prepare_buffer_queue.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(10));
      continue;
    }
    PrepareBuffer pthread_buffer = prepare_buffer_queue.front();
    prepare_buffer_queue.pop();
    const int total_packet = (pthread_buffer.buffer.size() + MAX_PACKET_SIZE - 1) / MAX_PACKET_SIZE;
    if (prepare_buffer_queue.size() >= MAX_BUFFER_QUEUE_SIZE) {
      return;
    }
    for (int i = 0; i < total_packet; i++) {
      uint8_t id = pthread_buffer.id;
      uint8_t type = pthread_buffer.msg_type;
      uint16_t idx = i;
      uint8_t max_idx = total_packet;
      std::vector<uint8_t> header(sizeof(uint32_t) + sizeof(uint8_t));
      std::memcpy(header.data(), &id, sizeof(id));
      std::memcpy(header.data() + sizeof(uint8_t), &type, sizeof(type));
      std::memcpy(header.data() + sizeof(uint16_t), &idx, sizeof(idx));
      std::memcpy(header.data() + sizeof(uint32_t), &max_idx, sizeof(max_idx));
      std::vector<uint8_t> packet;
      packet.insert(packet.end(), header.begin(), header.end());
      packet.insert(packet.end(),
                    pthread_buffer.buffer.begin() + i * MAX_PACKET_SIZE,
                    i == total_packet - 1 ? pthread_buffer.buffer.end()
                                          : pthread_buffer.buffer.begin() + (i + 1) * MAX_PACKET_SIZE);
      SendBuffer s_buffer;
      s_buffer.buffer = packet;
      s_buffer.id = pthread_buffer.id;
      buffer_queue.push(s_buffer);
    }
  }
}

void CommunicationServerNode::ParseBufferThread(const int robot_id)
{
  int packet_idx = 0;
  int packet_type = -1;
  std::vector<uint8_t> buffer;
  while (rclcpp::ok()) {
    if (parse_buffer_queue[robot_id].empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(10));
      continue;
    }
    std::vector<uint8_t> buffer_tmp = parse_buffer_queue[robot_id].front();
    parse_buffer_queue[robot_id].pop();

    uint8_t id, type, max_idx;
    uint16_t idx;
    std::memcpy(&id, buffer_tmp.data(), sizeof(id));
    std::memcpy(&type, buffer_tmp.data() + sizeof(uint8_t), sizeof(type));
    std::memcpy(&idx, buffer_tmp.data() + sizeof(uint16_t), sizeof(idx));
    std::memcpy(&max_idx, buffer_tmp.data() + sizeof(uint32_t), sizeof(max_idx));

    if (packet_type == -1) {
      packet_type = type;
    } else if (packet_type < type) {
      continue;
    } else if (packet_type > type) {
      packet_type = type;
      packet_idx = 0;
      buffer = std::vector<uint8_t>(0);
    }
    if (idx == 0) {
      packet_idx = 0;
      buffer = std::vector<uint8_t>(0);
    } else if (packet_idx != idx) {
      packet_idx = 0;
      packet_type = -1;
      buffer = std::vector<uint8_t>(0);
      continue;
    }

    packet_idx++;
    if (packet_idx == 1) {
      buffer.insert(
        buffer.begin(), buffer_tmp.begin() + sizeof(uint32_t) + sizeof(uint8_t), buffer_tmp.end());
    } else {
      buffer.insert(
        buffer.end(), buffer_tmp.begin() + sizeof(uint32_t) + sizeof(uint8_t), buffer_tmp.end());
    }

    if (packet_idx != max_idx) {
      continue;
    }
    try {
      if (type == 1) {
        geometry_msgs::msg::Twist twist_msg = DeserializeMsg<geometry_msgs::msg::Twist>(buffer);
        cmd_vel_pub_[robot_id]->publish(twist_msg);
      }
    } catch (...) {
    }

    packet_idx = 0;
    packet_type = -1;
    buffer = std::vector<uint8_t>(0);
  }
}

// Serialization
template <class T> std::vector<uint8_t> CommunicationServerNode::SerializeMsg(const T & msg)
{
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<T> serializer;
  serializer.serialize_message(&msg, &serialized_msg);

  std::vector<uint8_t> buffer_tmp(serialized_msg.size());
  std::memcpy(
    buffer_tmp.data(), serialized_msg.get_rcl_serialized_message().buffer, serialized_msg.size());

  return buffer_tmp;
}

// Deserialization
template <class T> T CommunicationServerNode::DeserializeMsg(const std::vector<uint8_t> & data)
{
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<T> serializer;

  serialized_msg.reserve(data.size());
  std::memcpy(serialized_msg.get_rcl_serialized_message().buffer, data.data(), data.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = data.size();

  T msg;
  serializer.deserialize_message(&serialized_msg, &msg);

  return msg;
}

} // namespace communication_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(communication_server::CommunicationServerNode)