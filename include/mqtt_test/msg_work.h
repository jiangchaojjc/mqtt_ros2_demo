#ifndef __MSG_WORK_H_
#define __MSG_WORK_H_

//#include "mqtt_test/ros2_mqtt_bridge.h"
#include <chrono>

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include "charge_interface/action/charge_back.hpp"

#include <functional>

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <condition_variable>
#include <json/json.h>

#include <mutex>
#include <queue>

#include "rcsbot_interface/msg/error_pub.hpp"
#include "rcsbot_interface/msg/io_pub.hpp"
#include "rcsbot_interface/msg/rob_state_pub.hpp"
#include "rcsbot_interface/msg/sonar_pub.hpp"

#include "charge_interface/action/charge_back.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#define ROS_TO_MQTTBRIDGE "Ros_To_MqttBridge"
#define BROKER_TO_MQTTBRIDGE "Broker_To_MqttBridge"
#define MQTTBRIDGE_TO_BROKER "MqttBridge_To_Broker"
#define CHARGEBACK "chargeback"
#define MQTTBRIDGE_TO_ROS "MqttBridge_To_Ros"

// mqtt qos
#define QOS_ZERO_LEVEL 0 // 至多一次
#define QOS_ONE_LEVEL 1  // 至少一次
#define QOS_TWO_LEVEL 2  // 只有一次

static std::map<std::string, int> topicParam = {{CHARGEBACK, 1},
                                                {MQTTBRIDGE_TO_BROKER, 2},
                                                {MQTTBRIDGE_TO_ROS, 3},
                                                {BROKER_TO_MQTTBRIDGE, 4},
                                                {ROS_TO_MQTTBRIDGE, 5}};

class CMsgWork {
public:
  CMsgWork();
  ~CMsgWork();
  void startServer();
  void notifyMsg(std::string &ptopic, std::string &pRecvMsg);
  std::string getBrokerMsgForRos(std::string &msg);
  std::queue<std::pair<std::string, std::string>> m_recvMsgQueue;
  std::condition_variable m_conVar;
  std::mutex m_mtx; // 互斥锁

private:
  std::string rosinfo;
  // std::shared_ptr<Ros2MqttBridge> g_MqttRos;
};

extern CMsgWork g_cMsgWork;

#endif // !__MSG_WORK_H_