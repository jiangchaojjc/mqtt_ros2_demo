#ifndef _MQTT_H_
#define _MQTT_H_

#include "MQTTAsync.h"

#include "mqtt_test/msg_work.h"
#include "mqtt_test/ros2_mqtt_bridge.h"

using std::cout;
using std::endl;

class CMqtt {
public:
  CMqtt();
  ~CMqtt();

  void initMqttConnect(char *pAddress, int MqttClientFlag,
                       const char *SetClientId);
  void PublishMsg(std::string pTopic, int nQos, std::string sData);
  std::string GetMqttMsg() const { return m_sMqttMsg; }
  void SetMqttMsg(std::string MqttMsg) { m_sMqttMsg = MqttMsg; }
  void mqttBridgeThread();
  void startEcsMsgJobThread();

public:
  static bool m_bSubscribe;
  static int m_nConnectStatus;

private:
  void reConnect();

  void reSubscribe();

  std::string m_sMqttMsg; // 获取的消息队列
  int m_nMqttFlag;
  const char *m_cCLIENTID;
  MQTTAsync m_pMqttAsyncClient;
  char m_sAddress[32];
  // std::shared_ptr<CMsgWork> g_cMsgWork;
};

extern CMqtt m_pMqtt;

#endif // !_MQTT_H_