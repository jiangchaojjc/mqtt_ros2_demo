/*
author:jiangchao
date:2022.12.08
description:database transform between ros2 database mqtt database
*/

#include "mqtt_test/mqtt.h"
#include "mqtt_test/ros2_mqtt_bridge.h"
#include <unistd.h>

// CMqtt m_pMqtt;
void mqttBridgeThread()
{
  m_pMqtt.mqttBridgeThread();
}



std::shared_ptr<Ros2MqttBridge> ros2Node;


void startServer()
{
  ros2Node->startServer();
}

int main(int argc, char **argv)
{
  //另一线程启动mqtt_client,给初始化一定延时
  std::thread MqttBridgeThread(mqttBridgeThread); // 建立连接，储存mqtt消息
  MqttBridgeThread.detach();
  sleep(5);
  rclcpp::init(argc, argv);
  cout << "init ------------" << endl;
  ros2Node = std::make_shared<Ros2MqttBridge>();
  
  RCLCPP_INFO(ros2Node->get_logger(), " init 222");

  RCLCPP_INFO(ros2Node->get_logger(), " init 333");
  std::thread MsgWorkThread(startServer); // 读取mqtt消息
  MsgWorkThread.detach();
  //  rclcpp::Rate(0.5).sleep();
  //  std::this_thread::sleep_for(5s);
  

  //RCLCPP_INFO(g_MqttRos->get_logger(), "backcharge init");
  rclcpp::spin(ros2Node->get_node_base_interface());
  //RCLCPP_INFO(g_MqttRos->get_logger(), "backcharge spin");
  rclcpp::shutdown();
  return 0;
}