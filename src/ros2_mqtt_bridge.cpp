/*
author:jiangchao
date:2022.12.08
description:database transform between ros2 database mqtt database
*/

#include "mqtt_test/ros2_mqtt_bridge.h"

// #include "nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp"

using namespace std;
// std::shared_ptr<Ros2MqttBridge> g_MqttRos =
// std::make_shared<Ros2MqttBridge>();

Ros2MqttBridge::Ros2MqttBridge()
    : nav2_util::LifecycleNode("Ros2MqttBridge", "", false) {

  // rosMsgWork = std::make_shared<CMqtt>();
  //  publish ros2 navi pose

  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("stop_on_failure", true);
  declare_parameter("loop_rate", 20);
  auto state = rclcpp_lifecycle::State();

  loop_rate_ = get_parameter("loop_rate").as_int();

  std::vector<std::string> new_args = rclcpp::NodeOptions().arguments();
  new_args.push_back("--ros-args");
  new_args.push_back("-r");
  new_args.push_back(std::string("__node:=") + this->get_name() +
                     "_rclcpp_node");
  new_args.push_back("--");
  client_node_ = std::make_shared<rclcpp::Node>(
      "_", "", rclcpp::NodeOptions().arguments(new_args));

  // jc:创建NavigateToPose的client
  nav_to_pose_client_ = rclcpp_action::create_client<Nav2ActionInterface>(
      client_node_, "navigate_to_pose");

  // jc:创建ChargeBackAction的client
  charge_back_client = rclcpp_action::create_client<ChargeBackActionInterface>(
      client_node_, "chargeback");

  // jc:创建FollowWaypoints的client
  followWaypoints_client_ =
      rclcpp_action::create_client<FollowWaypointsInterface>(client_node_,
                                                             "FollowWaypoints");
  RCLCPP_INFO(this->get_logger(), "Ros2MqttBridge construct");

  // publishMqttMsg();

  // getBrokerMsgForRos(g_cMsgWork.);
}

Ros2MqttBridge::~Ros2MqttBridge() {}

void Ros2MqttBridge::getBrokerMsg(
    std::pair<std::string, std::string> &topicMsg) {

  //没有获取数据或者数据格式不对就返回
  // 将mqtt_clien发布的json数据反序列化
  std::string msg = topicMsg.second;
  JSONCPP_STRING err;
  Json::Value root;
  Json::CharReaderBuilder builder;
  // 对反序列化消息的格式判断
  const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  if (!reader->parse(msg.c_str(), msg.c_str() + msg.length(), &root, &err)) {
    // LogPrint(EN_LOG_LEVEL::ERR, "error");
    cout << "格式不对" << endl;
    return;
  }
  std::chrono::milliseconds server_timeout_{30};
  rclcpp::WallRate r(loop_rate_);
  int caseKey = topicParam[topicMsg.first];
  switch (caseKey) {

  case 1: {
    RCLCPP_INFO(this->get_logger(), "charge_back trig...");
    auto back_trig = ChargeBackActionInterface::Goal();
    back_trig.back_charge = root["chargeTrig"].asBool();
    if (!charge_back_client->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    // auto back_trig = ChargeBackActionInterface::Goal();
    // back_trig.back_charge = true;
    if (back_trig.back_charge) {
      RCLCPP_INFO(this->get_logger(), "send_back_goal_options");

      auto charge_back_goal_options =
          rclcpp_action::Client<ChargeBackActionInterface>::SendGoalOptions();

      charge_back_goal_options.goal_response_callback = std::bind(
          &Ros2MqttBridge::chargeResponseCallback, this, std::placeholders::_1);

      // charge_back_goal_options.feedback_callback =
      //     std::bind(&Ros2MqttBridge::charge_feedback_callback, this, _1, _2);

      charge_back_goal_options.result_callback = std::bind(
          &Ros2MqttBridge::chargeResultCallback, this, std::placeholders::_1);

      future_back_charge_goal_handle = charge_back_client->async_send_goal(
          back_trig, charge_back_goal_options);
      rclcpp::spin_until_future_complete(
          client_node_, future_back_charge_goal_handle, server_timeout_);
      back_charge_goal_handle = future_back_charge_goal_handle.get();
    }

    break;
  }

  case 2: {
    RCLCPP_INFO(this->get_logger(), "chargeback stop trig...");
    auto back_trig =
        root["chargeStop"]
            .asBool(); // mqtt发送过来取消的消息就直接取消，不需要再发送action的消息类型
    if (!charge_back_client->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    // auto back_trig = ChargeBackActionInterface::Goal();
    // back_trig.back_charge = true;

    if (back_trig) {
      RCLCPP_INFO(this->get_logger(), "send_back_goal_stop");

      auto cancel_charge_back =
          charge_back_client->async_cancel_goal(back_charge_goal_handle);
      if (rclcpp::spin_until_future_complete(client_node_, cancel_charge_back,
                                             server_timeout_) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(client_node_->get_logger(),
                     "Failed to cancel back charge");
        return;
      }
    }

    break;
  }
  //导航到某个点的任务
  case 3: {

    RCLCPP_INFO(this->get_logger(), "navigator to a goal trig...");
    // mqtt发送过来取消的消息就直接取消，不需要再发送action的消息类型
    bool navi_trig = root["naviStart"].asBool();
    //再发一次任务就会penging之前的任务，执行现在的任务，这些都在server端执行
    if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    if (navi_trig) {
      using namespace std::placeholders;
      Nav2ActionInterface::Goal
          client_goal; // jc:这个主要是发一个NavigateToPose_Goal_，包含posestamp
      RCLCPP_INFO(get_logger(), "NavigateToPose_Goal_ .......");
      // client_goal.pose.header.frame_id = root["frame_id"].asString();
      // int sec = root["stamp"][0].asInt();
      // int nanasec = root["stamp"][1].asInt();
      // auto &&ros_time = rclcpp::Time(sec, nanasec);
      // client_goal.pose.header.stamp = ros_time;
      // client_goal.pose.pose.position.x = root["RobotPose"][0].asFloat();
      // client_goal.pose.pose.position.y = root["RobotPose"][1].asFloat();
      // client_goal.pose.pose.orientation.z = root["RobotPose"][2].asFloat();
      // client_goal.pose.pose.orientation.w = root["RobotPose"][3].asFloat();
      // client_goal.pose.pose.orientation.x = root["RobotPose"][4].asFloat();
      // client_goal.pose.pose.orientation.y = root["RobotPose"][5].asFloat();

      client_goal.pose.header.frame_id = "map";
      auto rtime = std::chrono::system_clock::now().time_since_epoch().count();
      int sec = rtime / lround(floor((pow(10, 9))));
      int nanasec = rtime % lround(floor((pow(10, 9))));
      auto &&ros_time = rclcpp::Time(sec, nanasec);
      client_goal.pose.header.stamp = ros_time;
      client_goal.pose.pose.position.x = root["x"].asFloat();
      client_goal.pose.pose.position.y = root["y"].asFloat();
      client_goal.pose.pose.position.y = root["z"].asFloat();

      auto yaw = root["theta"].asFloat();
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, yaw); // 单位是弧度
      client_goal.pose.pose.orientation.z = orientation.getZ();
      client_goal.pose.pose.orientation.w = orientation.getW();
      client_goal.pose.pose.orientation.x = orientation.getX();
      client_goal.pose.pose.orientation.y = orientation.getY();

      RCLCPP_INFO(get_logger(), "jcjcjcjc get the goal pose. x %f y %f ",
                  client_goal.pose.pose.position.x,
                  client_goal.pose.pose.position.y);
      auto send_goal_options = rclcpp_action::Client<Nav2ActionInterface>::
          SendGoalOptions(); // jc:返回结构体SendGoalOptions，包含3个函数指针
      send_goal_options.result_callback = std::bind(
          &Ros2MqttBridge::nav2resultCallback, this, std::placeholders::_1);
      send_goal_options
          .goal_response_callback = // jc:相当于激活之后执行的回调函数
          std::bind(&Ros2MqttBridge::nav2goalResponseCallback, this,
                    std::placeholders::_1);
      future_nav2_goal_handle_ = nav_to_pose_client_->async_send_goal(
          client_goal, send_goal_options); // jc:发送请求，执行三个回调函数

      if (rclcpp::spin_until_future_complete(
              client_node_, future_nav2_goal_handle_, server_timeout_) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
        return;
      }
      // Get the goal handle and save so that we can check on completion in the
      // timer callback
      RCLCPP_INFO(this->get_logger(), "navi2 spin_until_future_complete");
      navigation_goal_handle_ = future_nav2_goal_handle_.get();
      if (!navigation_goal_handle_) {
        RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
        return;
      }
    }

    break;
  }

  case 4: {
    RCLCPP_INFO(this->get_logger(), "new navigator to a goal trig...");
    bool navi_trig =
        root["newNaviStart"]
            .asBool(); // mqtt发送过来取消的消息就直接取消，不需要再发送action的消息类型
    // if (!nav_to_pose_client_
    //          ->wait_for_action_server()) {
    //          //再发一次任务就会penging之前的任务，执行现在的任务，这些都在server端执行
    //   RCLCPP_ERROR(this->get_logger(),
    //                "Action server not available after waiting");
    //   rclcpp::shutdown();
    // }

    if (navi_trig) {
      using namespace std::placeholders;
      Nav2ActionInterface::Goal
          client_goal; // jc:这个主要是发一个NavigateToPose_Goal_，包含posestamp
      RCLCPP_INFO(get_logger(), "preempt NavigateToPose_Goal_ .......");
      // client_goal.pose.header.frame_id = root["frame_id"].asString();
      // int sec = root["stamp"][0].asInt();
      // int nanasec = root["stamp"][1].asInt();
      // auto &&ros_time = rclcpp::Time(sec, nanasec);
      // client_goal.pose.header.stamp = ros_time;
      // client_goal.pose.pose.position.x = root["RobotPose"][0].asFloat();
      // client_goal.pose.pose.position.y = root["RobotPose"][1].asFloat();
      // client_goal.pose.pose.orientation.z = root["RobotPose"][2].asFloat();
      // client_goal.pose.pose.orientation.w = root["RobotPose"][3].asFloat();
      // client_goal.pose.pose.orientation.x = root["RobotPose"][4].asFloat();
      // client_goal.pose.pose.orientation.y = root["RobotPose"][5].asFloat();

      client_goal.pose.header.frame_id = "map";
      auto rtime = std::chrono::system_clock::now().time_since_epoch().count();
      int sec = rtime / lround(floor((pow(10, 9))));
      int nanasec = rtime % lround(floor((pow(10, 9))));
      auto &&ros_time = rclcpp::Time(sec, nanasec);
      client_goal.pose.header.stamp = ros_time;

      client_goal.pose.pose.position.x = root["x"].asFloat();
      client_goal.pose.pose.position.y = root["y"].asFloat();
      client_goal.pose.pose.position.y = root["z"].asFloat();

      auto yaw = root["theta"].asFloat();
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, yaw); // 单位是弧度
      client_goal.pose.pose.orientation.z = orientation.getZ();
      client_goal.pose.pose.orientation.w = orientation.getW();
      client_goal.pose.pose.orientation.x = orientation.getX();
      client_goal.pose.pose.orientation.y = orientation.getY();

      RCLCPP_INFO(get_logger(), "jcjcjcjc get new goal pose. x %f y %f ",
                  client_goal.pose.pose.position.x,
                  client_goal.pose.pose.position.y);
      auto send_goal_options = rclcpp_action::Client<Nav2ActionInterface>::
          SendGoalOptions(); // jc:返回结构体SendGoalOptions，包含3个函数指针
      send_goal_options.result_callback = std::bind(
          &Ros2MqttBridge::nav2resultCallback, this, std::placeholders::_1);
      send_goal_options
          .goal_response_callback = // jc:相当于激活之后执行的回调函数
          std::bind(&Ros2MqttBridge::nav2goalResponseCallback, this,
                    std::placeholders::_1);
      future_nav2_goal_handle_ = nav_to_pose_client_->async_send_goal(
          client_goal, send_goal_options); // jc:发送请求，执行三个回调函数

      if (rclcpp::spin_until_future_complete(
              client_node_, future_nav2_goal_handle_, server_timeout_) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(client_node_->get_logger(), "Send a new goal call failed");
        return;
      }

      RCLCPP_INFO(this->get_logger(),
                  "navi2 preempt spin_until_future_complete");
      // Get the goal handle and save so that we can check on completion in the
      // timer callback
      navigation_goal_handle_ = future_nav2_goal_handle_.get();
      if (!navigation_goal_handle_) {
        RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
        return;
      }
    }
    break;
  }

  case 5: {

    RCLCPP_INFO(this->get_logger(), "navi2 stop trig...");
    bool navi_stop_trig =
        root["nav2_Goal_Stop"]
            .asBool(); // mqtt发送过来取消的消息就直接取消，不需要再发送action的消息类型
    if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    // auto back_trig = ChargeBackActionInterface::Goal();
    // back_trig.back_charge = true;

    if (navi_stop_trig) {
      RCLCPP_INFO(this->get_logger(), "navi_stop_trig_goal_stop");

      auto cancel_nav2_goal =
          nav_to_pose_client_->async_cancel_goal(navigation_goal_handle_);
      if (rclcpp::spin_until_future_complete(client_node_, cancel_nav2_goal,
                                             server_timeout_) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(client_node_->get_logger(),
                     "Failed to cancel back charge");
        return;
      }
    }
    break;
  }

  case 6: {
    RCLCPP_INFO(this->get_logger(), "followWayPoints trig...");
    circleFollowTrig = true;
    // mqtt发送过来取消的消息就直接取消，不需要再发送action的消息类型
    // bool followWaypoints_trig = root["FWPointsStart"].asBool();

    using namespace std::placeholders;
    geometry_msgs::msg::PoseStamped client_goal;
    RCLCPP_INFO(get_logger(), "followWayPoints start .......");

    int num = root["cruise_points"].size();
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    for (int i = 0; i < num; i++) {
      client_goal.header.frame_id = "map";
      auto rtime = std::chrono::system_clock::now().time_since_epoch().count();
      int sec = rtime / lround(floor((pow(10, 9))));
      int nanasec = rtime % lround(floor((pow(10, 9))));
      auto &&ros_time = rclcpp::Time(sec, nanasec);
      client_goal.header.stamp = ros_time;
      client_goal.pose.position.x = root["cruise_points"][i]["x"].asFloat();

      client_goal.pose.position.y = root["cruise_points"][i]["y"].asFloat();
      client_goal.pose.position.z = root["cruise_points"][i]["z"].asFloat();

      auto yaw = root["cruise_points"][i]["theta"].asFloat();
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, yaw); // 单位是弧度
      client_goal.pose.orientation.z = orientation.getZ();
      client_goal.pose.orientation.w = orientation.getW();
      client_goal.pose.orientation.x = orientation.getX();
      client_goal.pose.orientation.y = orientation.getY();
      RCLCPP_INFO(get_logger(), "jcjcjcjc get new goal pose. x %f y %f ",
                  client_goal.pose.position.x, client_goal.pose.position.y);
      poses.push_back(client_goal);
    }
    waypoint_follower_goal_.poses = poses;

    RCLCPP_INFO(client_node_->get_logger(), "Sending a path of %zu waypoints:",
                waypoint_follower_goal_.poses.size());

    // // Send the goal poses
    // if (!followWaypoints_client_->wait_for_action_server(
    //         std::chrono::seconds(5))) {
    //   //再发一次任务就会penging之前的任务，执行现在的任务，这些都在server端执行
    //   RCLCPP_ERROR(this->get_logger(),
    //                "Action server not available after waiting");
    //   rclcpp::shutdown();
    // }

    // // Enable result awareness by providing an empty lambda function
    // auto send_goal_options =
    //     rclcpp_action::Client<FollowWaypointsInterface>::SendGoalOptions();

    // // send_goal_options.result_callback = [](auto) {};

    // send_goal_options.result_callback =
    //     std::bind(&Ros2MqttBridge::followWaypointsResultCallback, this,
    //               std::placeholders::_1);
    // send_goal_options
    //     .goal_response_callback = // jc:相当于激活之后执行的回调函数
    //     std::bind(&Ros2MqttBridge::followWaypointsGoalResponseCallback, this,
    //               std::placeholders::_1);

    // auto future_goal_handle = followWaypoints_client_->async_send_goal(
    //     waypoint_follower_goal_, send_goal_options);

    // if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle,
    //                                        server_timeout_) !=
    //     rclcpp::FutureReturnCode::SUCCESS) {
    //   RCLCPP_ERROR(client_node_->get_logger(), "Send a new goal call
    //   failed"); return;
    // }

    // RCLCPP_INFO(client_node_->get_logger(),
    //             "followwaypoints spin_until_future_complete");
    // // Get the goal handle and save so that we can check on completion in
    // // the timer callback
    // followWaypoints_goal_handle_ = future_goal_handle.get();
    // if (!followWaypoints_goal_handle_) {
    //   RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by
    //   server"); return;
    // }
    // 开辟子线程循环执行Followwaypoints
    std::thread circleFollowWayPoints(
        &Ros2MqttBridge::gotoCircleFollowWaypoints, this);
    circleFollowWayPoints.detach();
    break;
  }

  case 7: {

    RCLCPP_INFO(this->get_logger(), "followWaypoints stop start...");
    bool followwaypoints_stop_trig =
        root["FollowWaypoints_Stop"]
            .asBool(); // mqtt发送过来取消的消息就直接取消，不需要再发送action的消息类型
    if (!followWaypoints_client_->wait_for_action_server(
            std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    // auto back_trig = ChargeBackActionInterface::Goal();

    // back_trig.back_charge = true;

    if (followwaypoints_stop_trig) {
      RCLCPP_INFO(this->get_logger(), "followwaypoints_stop_trig...");

      auto cancel_followwaypoints = followWaypoints_client_->async_cancel_goal(
          followWaypoints_goal_handle_);
      if (rclcpp::spin_until_future_complete(
              client_node_, cancel_followwaypoints, server_timeout_) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(client_node_->get_logger(),
                     "Failed to cancel back charge");
        return;
      }
    }
    break;
  }

  default: {
    break;
  }
  }

  // rclcpp::spin_some(client_node_);
  // r.sleep();
}

void Ros2MqttBridge::gotoCircleFollowWaypoints() {
  std::chrono::milliseconds server_timeout_{30};
  while (rclcpp::ok()) {

    if (circleFollowTrig) {
      circleFollowTrig = false;
      RCLCPP_INFO(this->get_logger(), "followWayPoints circle...");
      if (!followWaypoints_client_->wait_for_action_server(
              std::chrono::seconds(5))) {
        //再发一次任务就会penging之前的任务，执行现在的任务，这些都在server端执行
        RCLCPP_ERROR(this->get_logger(),
                     "Action server not available after waiting");
        rclcpp::shutdown();
      }

      // Enable result awareness by providing an empty lambda function
      auto send_goal_options =
          rclcpp_action::Client<FollowWaypointsInterface>::SendGoalOptions();

      // send_goal_options.result_callback =
      //     std::bind(&Ros2MqttBridge::followWaypointsResultCallback, this,
      //               std::placeholders::_1);
      // send_goal_options.feedback_callback =
      //     std::bind(&Ros2MqttBridge::followWaypointsFeedbackCallback, this,
      //               std::placeholders::_1, std::placeholders::_2);
      // send_goal_options.goal_response_callback = // jc:执行的回调函数
      //     std::bind(&Ros2MqttBridge::followWaypointsGoalResponseCallback,
      //     this,
      //               std::placeholders::_1);

      future_follow_goal_handle = followWaypoints_client_->async_send_goal(
          waypoint_follower_goal_, send_goal_options);

      if (rclcpp::spin_until_future_complete(
              client_node_, future_follow_goal_handle, server_timeout_) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(client_node_->get_logger(), "Send a new goal call failed");
        return;
      }

      followWaypoints_goal_handle_ = future_follow_goal_handle.get();
      if (!followWaypoints_goal_handle_) {
        RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
        return;
      }
      RCLCPP_INFO(client_node_->get_logger(),
                  "followwaypoints Wait for the result");

      auto future_result = followWaypoints_client_->async_get_result(
          followWaypoints_goal_handle_);

      // Wait for the result阻塞在这个地方直到获取结果
      rclcpp::spin_until_future_complete(client_node_, future_result);

      // The final result
      rclcpp_action::ClientGoalHandle<FollowWaypointsInterface>::WrappedResult
          result = future_result.get();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        circleFollowTrig = true;
      } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_INFO(client_node_->get_logger(),
                    "rclcpp_action::ResultCode::CANCELED");
        return;
      }

      RCLCPP_INFO(client_node_->get_logger(),
                  "followwaypoints spin_until_future_complete");
      // Get the goal handle and save so that we can check on completion in
      // the timer callback
      // followWaypoints_goal_handle_ = future_follow_goal_handle.get();
      // if (!followWaypoints_goal_handle_) {
      //   RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by
      //   server"); return;
      // }
    }
  }
}

void Ros2MqttBridge::nav2resultCallback(
    const rclcpp_action::ClientGoalHandle<Nav2ActionInterface>::WrappedResult
        &result) {
  RCLCPP_INFO(get_logger(), "followWaypointsresultCallback");
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), "Goal was SUCCEEDED");
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }
}

void Ros2MqttBridge::nav2goalResponseCallback(
    std::shared_future<
        rclcpp_action::ClientGoalHandle<Nav2ActionInterface>::SharedPtr>
        future) {
  RCLCPP_INFO(get_logger(), "goalResponseCallback");
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(
        get_logger(),
        "navigate_to_pose action client failed to send goal to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
}

void Ros2MqttBridge::followWaypointsResultCallback(
    const rclcpp_action::ClientGoalHandle<
        FollowWaypointsInterface>::WrappedResult &result) {
  RCLCPP_INFO(get_logger(), "followWaypointsresultCallback");
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    current_goal_status_ = ActionStatus::SUCCEEDED;
    RCLCPP_INFO(this->get_logger(), "circleFollow = true");
    // circleFollow = true;
    return;
  case rclcpp_action::ResultCode::ABORTED:
    current_goal_status_ = ActionStatus::FAILED;
    return;
  case rclcpp_action::ResultCode::CANCELED:
    current_goal_status_ = ActionStatus::FAILED;
    return;
  default:
    current_goal_status_ = ActionStatus::UNKNOWN;
    return;
  }
}

void Ros2MqttBridge::followWaypointsFeedbackCallback(
    FollowWaypointsGoalHandle::SharedPtr,
    const std::shared_ptr<const FollowWaypointsInterface::Feedback> feedback) {
  // std::stringstream ss;
  // ss << "feedback_callback received: ";
  RCLCPP_INFO(get_logger(), "followWaypoints feedback_callback received");
  // ss << feedback->current_waypoint << " ";

  RCLCPP_INFO(this->get_logger(), "followWaypoints %i",
              feedback->current_waypoint);
}

void Ros2MqttBridge::followWaypointsGoalResponseCallback(
    std::shared_future<
        rclcpp_action::ClientGoalHandle<FollowWaypointsInterface>::SharedPtr>
        future) {
  RCLCPP_INFO(get_logger(), "followWaypointsgoalResponseCallback");
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(
        get_logger(),
        "navigate_to_pose action client failed to send goal to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
}

void Ros2MqttBridge::chargeResponseCallback(
    std::shared_future<
        rclcpp_action::ClientGoalHandle<ChargeBackActionInterface>::SharedPtr>
        future) {
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void Ros2MqttBridge::chargeResultCallback(
    const rclcpp_action::ClientGoalHandle<
        ChargeBackActionInterface>::WrappedResult &result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), "Goal was SUCCEEDED");
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }
  rclcpp::shutdown();
}

void Ros2MqttBridge::startServer() {
  std::pair<std::string, std::string> sData;
  while (true) {
    {
      std::unique_lock<std::mutex> lck(g_cMsgWork.m_mtx);
      if (g_cMsgWork.m_recvMsgQueue.empty()) {
        // 当 std::condition_variable对象的某个wait 函数被调用的时候，它使用
        // std::unique_lock(通过 std::mutex) 来锁住当前线程。
        // 当前线程会一直被阻塞，直到另外一个线程在相同的
        // std::condition_variable 对象上调用了 notification
        // 函数来唤醒当前线程。

        RCLCPP_INFO(get_logger(), "mqtt empty");
        g_cMsgWork.m_conVar.wait(lck);
      }

      sData = g_cMsgWork.m_recvMsgQueue.front();
      g_cMsgWork.m_recvMsgQueue.pop();
    }
    RCLCPP_INFO(get_logger(), "ptopic %s ", sData.first.c_str());
    RCLCPP_INFO(get_logger(), "pRecvMsg %s ", sData.second.c_str());
    if (!sData.first.empty() && !sData.second.empty()) {
      // 业务处理函数
      RCLCPP_INFO(get_logger(), "mqtt info callback");
      getBrokerMsg(sData);
    }
  }
}

// tf2::Quaternion Ros2MqttBridge::getOriFromYaw(float yaw) {
//   // geometry_msgs::msg::Quaternion;
//   // return orientation;
// }
