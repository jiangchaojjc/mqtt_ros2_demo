
#include "mqtt_test/msg_work.h"

CMsgWork::CMsgWork() {
  // g_MqttRos = std::make_shared<Ros2MqttBridge>();
  // rclcpp::spin(g_MqttRos->get_node_base_interface());
}

CMsgWork g_cMsgWork;
CMsgWork::~CMsgWork() {}

void CMsgWork::startServer() //这里我已经不再调用，转到ros2_mqtt_bridge下了
{
  std::pair<std::string, std::string> sData;
  while (true) {
    {
      std::unique_lock<std::mutex> lck(m_mtx);
      if (m_recvMsgQueue.empty()) {
        // 当 std::condition_variable对象的某个wait 函数被调用的时候，它使用
        // std::unique_lock(通过 std::mutex) 来锁住当前线程。
        // 当前线程会一直被阻塞，直到另外一个线程在相同的
        // std::condition_variable 对象上调用了 notification
        // 函数来唤醒当前线程。
        m_conVar.wait(lck);
      }

      sData = m_recvMsgQueue.front();
      m_recvMsgQueue.pop();
    }

    if (!sData.first.empty() && !sData.second.empty()) {
      // 业务处理函数
      // g_MqttRos->getBrokerMsg(sData.second);
    }
  }
}

void CMsgWork::notifyMsg(std::string &ptopic, std::string &pRecvMsg) {
  std::cout << "DEBUG: notify to do " << std::endl;
  std::unique_lock<std::mutex> lck(m_mtx);
  std::pair<std::string, std::string> RecvMsg;
  RecvMsg.first = ptopic;
  RecvMsg.second = pRecvMsg;
  m_recvMsgQueue.push(RecvMsg);
  m_conVar.notify_one(); // 这是在唤醒阻塞的线程
}

std::string CMsgWork::getBrokerMsgForRos(std::string &msg) { return msg; }