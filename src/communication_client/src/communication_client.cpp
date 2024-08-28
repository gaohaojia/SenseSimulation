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
#include <functional>
#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <thread>
#include <vector>

#include "communication_client/communication_client.hpp"

#define MAX_PACKET_SIZE 64000
#define BUFFER_SIZE 65535
#define MAX_BUFFER_QUEUE_SIZE 256

namespace communication_client
{
CommunicationClientNode::CommunicationClientNode(const rclcpp::NodeOptions & options)
  : Node("communication_client", options)
{
  this->declare_parameter<int>("robot_id", 0);
  this->declare_parameter<int>("network_port", 12131);
  this->declare_parameter<std::string>("network_ip", "192.168.31.207");

  this->get_parameter("robot_id", robot_id);
  this->get_parameter("network_port", port);
  this->get_parameter("network_ip", ip);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel",
    10,
    std::bind(&CommunicationClientNode::CmdVelCallBack, this, std::placeholders::_1));

  livox_scan_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("livox/lidar", 5);
  livox_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("livox/imu", 5);

  InitClient();
}

CommunicationClientNode::~CommunicationClientNode()
{
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (heart_beat_thread_.joinable()) {
    heart_beat_thread_.join();
  }
  if (prepare_buffer_thread_.joinable()) {
    prepare_buffer_thread_.join();
  }
  if (parse_buffer_thread_.joinable()) {
    parse_buffer_thread_.join();
  }
  close(sockfd);
}

// initialize the socket client
void CommunicationClientNode::InitClient()
{
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
    return;
  }

  memset(&server_addr, 0, sizeof(server_addr));

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  server_addr.sin_addr.s_addr = inet_addr(ip.c_str());

  send_thread_ = std::thread(&CommunicationClientNode::NetworkSendThread, this);
  recv_thread_ = std::thread(&CommunicationClientNode::NetworkRecvThread, this);
  heart_beat_thread_ = std::thread(&CommunicationClientNode::HeartBeatThread, this);
  prepare_buffer_thread_ = std::thread(&CommunicationClientNode::PrepareBufferThread, this);
  parse_buffer_thread_ = std::thread(&CommunicationClientNode::ParseBufferThread, this);
  RCLCPP_INFO(this->get_logger(), "Client start! Target ip: %s, port: %d", ip.c_str(), port);
}

void CommunicationClientNode::CmdVelCallBack(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr cmd_vel_msg)
{
  geometry_msgs::msg::Twist twist = cmd_vel_msg->twist;
  std::vector<uint8_t> data_buffer = SerializeMsg<geometry_msgs::msg::Twist>(twist);
  PrepareBuffer pthread_buffer = {robot_id, data_buffer, 1};
  if (prepare_buffer_queue.size() >= MAX_BUFFER_QUEUE_SIZE) {
    return;
  }
  prepare_buffer_queue.push(pthread_buffer);
}

void CommunicationClientNode::NetworkSendThread()
{
  while (rclcpp::ok()) {
    if (buffer_queue.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(10));
      continue;
    }
    SendBuffer s_buffer = buffer_queue.front();
    buffer_queue.pop();
    if (sendto(sockfd,
               s_buffer.buffer.data(),
               s_buffer.buffer.size(),
               0,
               (struct sockaddr *)&server_addr,
               sizeof(server_addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Send failed!");
    }
  }
}

void CommunicationClientNode::NetworkRecvThread()
{
  int n, len = sizeof(server_addr);
  while (rclcpp::ok()) {
    std::vector<uint8_t> buffer_tmp(BUFFER_SIZE);
    n = recvfrom(sockfd,
                 buffer_tmp.data(),
                 BUFFER_SIZE,
                 MSG_WAITALL,
                 (struct sockaddr *)&server_addr,
                 (socklen_t *)&len);
    if (n < 0) {
      continue;
    }
    buffer_tmp.resize(n);
    if (parse_buffer_queue.size() >= MAX_BUFFER_QUEUE_SIZE) {
      continue;
    }
    parse_buffer_queue.push(buffer_tmp);
  }
}

void CommunicationClientNode::HeartBeatThread()
{
  while (rclcpp::ok()) {
    std::vector<uint8_t> data_buffer = {0};
    PrepareBuffer pthread_buffer = {robot_id, data_buffer, 0};
    if (prepare_buffer_queue.size() >= MAX_BUFFER_QUEUE_SIZE) {
      continue;
    }
    prepare_buffer_queue.push(pthread_buffer);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void CommunicationClientNode::PrepareBufferThread()
{
  while (rclcpp::ok()) {
    if (prepare_buffer_queue.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(10));
      continue;
    }
    PrepareBuffer pthread_buffer = prepare_buffer_queue.front();
    prepare_buffer_queue.pop();
    const int total_packet = (pthread_buffer.buffer.size() + MAX_PACKET_SIZE - 1) / MAX_PACKET_SIZE;
    if (buffer_queue.size() >= MAX_BUFFER_QUEUE_SIZE) {
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
      buffer_queue.push(s_buffer);
    }
  }
}

void CommunicationClientNode::ParseBufferThread()
{
  int packet_idx = 0;
  int packet_type = -1;
  std::vector<uint8_t> buffer;
  while (rclcpp::ok()) {
    if (parse_buffer_queue.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(10));
      continue;
    }
    std::vector<uint8_t> buffer_tmp = parse_buffer_queue.front();
    parse_buffer_queue.pop();

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
      if (type == 0) { // livox scan
        livox_ros_driver2::msg::CustomMsg scan_msg =
          DeserializeMsg<livox_ros_driver2::msg::CustomMsg>(buffer);
        // livox_ros_driver2::msg::CustomMsg custom_msg = ConvertPointCloud2ToCustomMsg(scan_msg);
        livox_scan_pub_->publish(scan_msg);
      } else if (type == 1) { // livox imu
        sensor_msgs::msg::Imu imu_msg = DeserializeMsg<sensor_msgs::msg::Imu>(buffer);
        livox_imu_pub_->publish(imu_msg);
      }
    } catch (...) {
    }

    packet_idx = 0;
    packet_type = -1;
    buffer = std::vector<uint8_t>(0);
  }
}

// livox_ros_driver2::msg::CustomMsg CommunicationClientNode::ConvertPointCloud2ToCustomMsg(
//   const sensor_msgs::msg::PointCloud2 & input_cloud)
// {
//   livox_ros_driver2::msg::CustomMsg custom_msg;

//   custom_msg.header.stamp = input_cloud.header.stamp;
//   custom_msg.header.frame_id = input_cloud.header.frame_id;

//   pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
//   pcl::fromROSMsg(input_cloud, pcl_cloud);

//   for (const pcl::PointXYZI & point : pcl_cloud.points) {
//     livox_ros_driver2::msg::CustomPoint custom_point;
//     custom_point.x = point.x;
//     custom_point.y = point.y;
//     custom_point.z = point.z;
//     custom_point.reflectivity = point.intensity;

//     custom_msg.points.push_back(custom_point);
//   }

//   return custom_msg;
// }

// Serialization
template <class T> std::vector<uint8_t> CommunicationClientNode::SerializeMsg(const T & msg)
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
template <class T> T CommunicationClientNode::DeserializeMsg(const std::vector<uint8_t> & data)
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

} // namespace communication_client

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(communication_client::CommunicationClientNode)