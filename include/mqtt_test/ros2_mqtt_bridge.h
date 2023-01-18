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
  std::promise<void> exitSignal;
  bool circleReturn;
  void startServer();
  void getBrokerMsg(std::pair<std::string, std::string> &topicMsg);

  using FollowWaypointsInterface = nav2_msgs::action::FollowWaypoints;
  using Nav2ActionInterface = nav2_msgs::action::NavigateToPose;
  using ChargeBackActionInterface = charge_interface::action::ChargeBack;

  using GoalHandleFollowWaypointsClient =
      rclcpp_action::Client<FollowWaypointsInterface>;
  using GoalHandleActionClient = rclcpp_action::Client<Nav2ActionInterface>;
  using GoalHandleChargeBackAction =
      rclcpp_action::Client<ChargeBackActionInterface>;

  using FollowWaypointsGoalHandle =
      rclcpp_action::ClientGoalHandle<FollowWaypointsInterface>;

  using ChargeBackGoalHandle =
      rclcpp_action::ClientGoalHandle<charge_interface::action::ChargeBack>;

  using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<Nav2ActionInterface>;

  // tf2::Quaternion getOriFromYaw(float yaw);

protected:
  void nav2resultCallback(
      const rclcpp_action::ClientGoalHandle<Nav2ActionInterface>::WrappedResult
          &result);

  /**
   * @brief Action client goal response callback
   * @param future Shared future to goalhandle
   */
  void nav2goalResponseCallback(
      std::shared_future<
          rclcpp_action::ClientGoalHandle<Nav2ActionInterface>::SharedPtr>
          future);

  void followWaypointsResultCallback(
      const rclcpp_action::ClientGoalHandle<
          FollowWaypointsInterface>::WrappedResult &result);

  /**
   * @brief Action client goal response callback
   * @param future Shared future to goalhandle
   */

  void followWaypointsFeedbackCallback(
      FollowWaypointsGoalHandle::SharedPtr,
      const std::shared_ptr<const FollowWaypointsInterface::Feedback> feedback);

  void followWaypointsGoalResponseCallback(
      std::shared_future<
          rclcpp_action::ClientGoalHandle<FollowWaypointsInterface>::SharedPtr>
          future);

  void chargeResultCallback(const rclcpp_action::ClientGoalHandle<
                            ChargeBackActionInterface>::WrappedResult &result);

  /**
   * @brief Action client goal response callback
   * @param future Shared future to goalhandle
   */
  void chargeResponseCallback(
      std::shared_future<
          rclcpp_action::ClientGoalHandle<ChargeBackActionInterface>::SharedPtr>
          future);

  // FOLLOWWAYPOINTS CIRCLE
  void gotoCircleFollowWaypoints();

  void gotoCircleGoals(std::vector<geometry_msgs::msg::PoseStamped> poses);

  bool stop_on_failure_;
  ActionStatus current_goal_status_;
  int loop_rate_;
  std::vector<int> failed_ids_;
  bool circleFollowTrig;

private:
  // FollowWaypoints目标点
  nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
  //为了取消任务
  ChargeBackGoalHandle::SharedPtr back_charge_goal_handle;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  FollowWaypointsGoalHandle::SharedPtr followWaypoints_goal_handle_;
  // std::shared_future<
  //     rclcpp_action::ClientGoalHandle<FollowWaypointsInterface>::SharedPtr>
  //     future_nav2_goal_handle_;
  //为了创建服务
  GoalHandleActionClient::SharedPtr nav_to_pose_client_;
  GoalHandleChargeBackAction::SharedPtr charge_back_client;
  GoalHandleFollowWaypointsClient::SharedPtr followWaypoints_client_;

  std::shared_future<
      std::shared_ptr<rclcpp_action::ClientGoalHandle<Nav2ActionInterface>>>
      future_nav2_goal_handle_;
  std::shared_future<std::shared_ptr<
      rclcpp_action::ClientGoalHandle<FollowWaypointsInterface>>>
      future_follow_goal_handle;
  std::shared_future<std::shared_ptr<
      rclcpp_action::ClientGoalHandle<ChargeBackActionInterface>>>
      future_back_charge_goal_handle;

  //启动的节点
  rclcpp::Node::SharedPtr client_node_;
  // std::vector<geometry_msgs::msg::PoseStamped> poses;

  // std::condition_variable m_conVar;
  // std::mutex m_mtx; // 互斥锁
  // std::shared_ptr<CMsgWork> rosMsgWork;
  //  void publishMqttMsg();
  //   std::shared_ptr<CMqtt> m_pMqtt;
  //   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
};

// extern std::shared_ptr<Ros2MqttBridge> g_MqttRos;
#endif // ROS2MQTTBRIDGE_H
