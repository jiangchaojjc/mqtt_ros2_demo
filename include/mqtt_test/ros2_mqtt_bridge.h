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

  using Nav2ActionClientT = nav2_msgs::action::NavigateToPose;
  using GoalHandleActionClient = rclcpp_action::Client<Nav2ActionClientT>;

  using ChargeBackAction = charge_interface::action::ChargeBack;
  using GoalHandleChargeBackAction = rclcpp_action::Client<ChargeBackAction>;

  using ChargeBackGoalHandle =
      rclcpp_action::ClientGoalHandle<charge_interface::action::ChargeBack>;

  using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<Nav2ActionClientT>;
  // tf2::Quaternion getOriFromYaw(float yaw);

protected:
  void resultCallback(
      const rclcpp_action::ClientGoalHandle<Nav2ActionClientT>::WrappedResult
          &result);

  /**
   * @brief Action client goal response callback
   * @param future Shared future to goalhandle
   */
  void goalResponseCallback(
      std::shared_future<
          rclcpp_action::ClientGoalHandle<Nav2ActionClientT>::SharedPtr>
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

  bool stop_on_failure_;
  ActionStatus current_goal_status_;
  int loop_rate_;
  std::vector<int> failed_ids_;

private:
  ChargeBackGoalHandle::SharedPtr back_charge_goal_handle;
  std::shared_future<
      rclcpp_action::ClientGoalHandle<Nav2ActionClientT>::SharedPtr>
      future_nav2_goal_handle_;
  GoalHandleActionClient::SharedPtr nav_to_pose_client_;
  GoalHandleChargeBackAction::SharedPtr charge_back;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  rclcpp::Node::SharedPtr client_node_;

  // std::condition_variable m_conVar;
  // std::mutex m_mtx; // 互斥锁
  // std::shared_ptr<CMsgWork> rosMsgWork;
  //  void publishMqttMsg();
  //   std::shared_ptr<CMqtt> m_pMqtt;
  //   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
};

// extern std::shared_ptr<Ros2MqttBridge> g_MqttRos;
#endif // ROS2MQTTBRIDGE_H
