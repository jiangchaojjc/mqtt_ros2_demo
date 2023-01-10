/*
author:jiangchao
date:2022.12.08
description:database transform between ros2 database mqtt database
*/

#ifndef ROS2MQTTBRIDGE_H
#define ROS2MQTTBRIDGE_H

#include "mqtt_test/msg_work.h"

enum class ActionStatus {
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3
};

class Ros2MqttBridge : public nav2_util::LifecycleNode {
public:
  Ros2MqttBridge();
  ~Ros2MqttBridge();

  void startServer();
  void getBrokerMsg(std::pair<std::string, std::string> &topicMsg);

  using ClientT = nav2_msgs::action::NavigateToPose;

  using ActionClient = rclcpp_action::Client<ClientT>;

  using ChargeBackAction = charge_interface::action::ChargeBack;
  using GoalHandleChargeBackAction = rclcpp_action::Client<ChargeBackAction>;

protected:
  void resultCallback(
      const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult &result);

  /**
   * @brief Action client goal response callback
   * @param future Shared future to goalhandle
   */
  void goalResponseCallback(
      std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr>
          future);

  void chargeResultCallback(
      const rclcpp_action::ClientGoalHandle<ChargeBackAction>::WrappedResult
          &result);

  /**
   * @brief Action client goal response callback
   * @param future Shared future to goalhandle
   */
  void chargeResponseCallback(
      std::shared_future<
          rclcpp_action::ClientGoalHandle<ChargeBackAction>::SharedPtr>
          future);

  // Our action server
  ActionClient::SharedPtr nav_to_pose_client_;
  GoalHandleChargeBackAction::SharedPtr charge_back;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr>
      future_goal_handle_;
  bool stop_on_failure_;
  ActionStatus current_goal_status_;
  int loop_rate_;
  std::vector<int> failed_ids_;

private:
  // std::condition_variable m_conVar;
  // std::mutex m_mtx; // 互斥锁
  // std::shared_ptr<CMsgWork> rosMsgWork;
  //  void publishMqttMsg();
  //   std::shared_ptr<CMqtt> m_pMqtt;
  //   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
};

// extern std::shared_ptr<Ros2MqttBridge> g_MqttRos;
#endif // ROS2MQTTBRIDGE_H
