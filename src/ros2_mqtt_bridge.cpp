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

  nav_to_pose_client_ = rclcpp_action::create_client<
      Nav2ActionClientT>( // jc:创建NavigateToPose的client
      client_node_, "navigate_to_pose");

  charge_back = rclcpp_action::create_client<ChargeBackAction>(
      client_node_,
      "chargeback"); // jc:创建ChargeBackAction的client
  RCLCPP_INFO(this->get_logger(), "Ros2MqttBridge construct");

  // publishMqttMsg();

  // getBrokerMsgForRos(g_cMsgWork.);
}

Ros2MqttBridge::~Ros2MqttBridge() {}

// 订阅到ros话题的回调函数
// void Ros2MqttBridge::publishMqttMsg()
// {

//   Nav2ActionClientT::Goal client_goal; //
//   jc:这个主要是发一个NavigateToPose_Goal_，包含posestamp
//   // client_goal.pose = goal->poses[goal_index];

//   client_goal.pose.header.frame_id = "map";
//   client_goal.pose.header.stamp = rclcpp::Clock().now();
//   client_goal.pose.pose.position.x = 0.48;
//   client_goal.pose.pose.position.y = 1.74;
//   client_goal.pose.pose.orientation.z = -0.707;
//   client_goal.pose.pose.orientation.w = 0.707;
//   client_goal.pose.pose.orientation.x = 0.0;
//   client_goal.pose.pose.orientation.y = 0.0;

//   // 这里先将ros的自定义消息转换为json格式消息
//   Json::Value root;
//   Json::Value Rtime;
//   Json::Value pose;
//   // 将stamp时间转换为字符串
//   root["frame_id"] = "map";
//   Rtime = rclcpp::Clock().now();
//   root["stamp"] = Rtime;
//   pose[0] = 0.48;
//   pose[1] = 1.74;
//   pose[2] = -0.707;
//   pose[3] = 0.707;
//   pose[4] = 0.0;
//   pose[5] = 0.0;
//   root["RobotPose"] = pose;
//   // json序列化为std::string类型数据，发送到broker
//   Json::StreamWriterBuilder builder;
//   std::string sJsonData = Json::writeString(builder, root);
//   m_pMqtt.PublishMsg(MQTTBRIDGE_TO_BROKER, QOS_TWO_LEVEL, sJsonData);
//   RCLCPP_INFO(get_logger(), "PublishMsg .......");

//   // // 这里先将ros的自定义消息转换为json格式消息
//   // Json::Value root;
//   // Json::Value time;
//   // Json::Value pose;
//   // // 将stamp时间转换为字符串
//   // time[0] = msg.header.stamp.sec;
//   // time[1] = msg.header.stamp.nsec;
//   // root["myHeader"] = time;
//   // root["mystring"] = msg.sMsg;
//   // root["myint"] = msg.nMsg;
//   // root["mybool"] = msg.bMsg;
//   // pose[0] = msg.gmRobotPose.x;
//   // pose[1] = msg.gmRobotPose.y;
//   // pose[2] = msg.gmRobotPose.theta;
//   // root["RobotPose"] = pose;
//   // // json序列化为std::string类型数据，发送到broker
//   // Json::StreamWriterBuilder builder;
//   // std::string sJsonData = Json::writeString(builder, root);
//   // m_pMqtt.PublishMsg(MQTTBRIDGE_TO_BROKER, QOS_TWO_LEVEL, sJsonData);

//   // m_pMqtt.PublishMsg(MQTTBRIDGE_TO_BROKER,QOS_TWO_LEVEL,msg.data);
// }

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
  std::chrono::milliseconds server_timeout_{20};
  rclcpp::WallRate r(loop_rate_);
  int caseKey = topicParam[topicMsg.first];
  switch (caseKey) {

  case 1: {
    RCLCPP_INFO(this->get_logger(), "charge_back trig...");
    auto back_trig = ChargeBackAction::Goal();
    back_trig.back_charge = root["chargeTrig"].asBool();
    if (!charge_back->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    // auto back_trig = ChargeBackAction::Goal();
    // back_trig.back_charge = true;
    if (back_trig.back_charge) {
      RCLCPP_INFO(this->get_logger(), "send_back_goal_options");

      auto charge_back_goal_options =
          rclcpp_action::Client<ChargeBackAction>::SendGoalOptions();

      charge_back_goal_options.goal_response_callback = std::bind(
          &Ros2MqttBridge::chargeResponseCallback, this, std::placeholders::_1);

      // charge_back_goal_options.feedback_callback =
      //     std::bind(&Ros2MqttBridge::charge_feedback_callback, this, _1, _2);

      charge_back_goal_options.result_callback = std::bind(
          &Ros2MqttBridge::chargeResultCallback, this, std::placeholders::_1);

      auto future_back_charge_goal_handle =
          charge_back->async_send_goal(back_trig, charge_back_goal_options);
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
    if (!charge_back->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    // auto back_trig = ChargeBackAction::Goal();
    // back_trig.back_charge = true;

    if (back_trig) {
      RCLCPP_INFO(this->get_logger(), "send_back_goal_stop");

      auto cancel_charge_back =
          charge_back->async_cancel_goal(back_charge_goal_handle);
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
    auto navi_trig = root["naviStart"].asBool();
    if (!nav_to_pose_client_
             ->wait_for_action_server()) { //再发一次任务就会penging之前的任务，执行现在的任务，这些都在server端执行
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    if (navi_trig) {
      using namespace std::placeholders;
      Nav2ActionClientT::Goal
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
      auto send_goal_options = rclcpp_action::Client<Nav2ActionClientT>::
          SendGoalOptions(); // jc:返回结构体SendGoalOptions，包含3个函数指针
      send_goal_options.result_callback = std::bind(
          &Ros2MqttBridge::resultCallback, this, std::placeholders::_1);
      send_goal_options
          .goal_response_callback = // jc:相当于激活之后执行的回调函数
          std::bind(&Ros2MqttBridge::goalResponseCallback, this,
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
    auto navi_trig =
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
      Nav2ActionClientT::Goal
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
      auto send_goal_options = rclcpp_action::Client<Nav2ActionClientT>::
          SendGoalOptions(); // jc:返回结构体SendGoalOptions，包含3个函数指针
      send_goal_options.result_callback = std::bind(
          &Ros2MqttBridge::resultCallback, this, std::placeholders::_1);
      send_goal_options
          .goal_response_callback = // jc:相当于激活之后执行的回调函数
          std::bind(&Ros2MqttBridge::goalResponseCallback, this,
                    std::placeholders::_1);
      future_nav2_goal_handle_ = nav_to_pose_client_->async_send_goal(
          client_goal, send_goal_options); // jc:发送请求，执行三个回调函数

      if (rclcpp::spin_until_future_complete(
              client_node_, future_nav2_goal_handle_, server_timeout_) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(client_node_->get_logger(), "Send a new goal call failed");
        return;
      }
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
    auto navi_stop_trig =
        root["nav2_Goal_Stop"]
            .asBool(); // mqtt发送过来取消的消息就直接取消，不需要再发送action的消息类型
    if (!nav_to_pose_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    // auto back_trig = ChargeBackAction::Goal();
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

  default: {
    break;
  }
  }

  // rclcpp::spin_some(client_node_);
  // r.sleep();
}

void Ros2MqttBridge::resultCallback(
    const rclcpp_action::ClientGoalHandle<Nav2ActionClientT>::WrappedResult
        &result) {
  RCLCPP_INFO(get_logger(), "resultCallback");
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    current_goal_status_ = ActionStatus::SUCCEEDED;
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

void Ros2MqttBridge::goalResponseCallback(
    std::shared_future<
        rclcpp_action::ClientGoalHandle<Nav2ActionClientT>::SharedPtr>
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

void Ros2MqttBridge::chargeResponseCallback(
    std::shared_future<
        rclcpp_action::ClientGoalHandle<ChargeBackAction>::SharedPtr>
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
    const rclcpp_action::ClientGoalHandle<ChargeBackAction>::WrappedResult
        &result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
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
