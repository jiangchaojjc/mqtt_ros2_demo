#include "mqtt_test/mqtt.h"

#include <iostream>
#include <stdlib.h>
#include <string>
#include <unistd.h>

// #include "base_log.h"
// #include "msg_common_define.h"
// #include "gate_way.h"
// #include "msg_regist.h"

// #define CLIENTID        "LocalMsgProxy"
#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

#define MQTT_CONNECT_STATUS 0
#define MQTT_NOT_CONNECT_STATUS 1
#define MQTT_LOST_CONNECT_STATUS 2
#define MQTT_DIS_CONNECT_STATUS 3
#define MQTT_EXCEPTION_CONNECT_STATUS 4

bool CMqtt::m_bSubscribe = false;
int CMqtt::m_nConnectStatus = MQTT_NOT_CONNECT_STATUS;

// mqtt pub topic
#define ROBOT_PUB_CONFIG_SET "/robot/config/set"
#define ROBOT_PUB_MAP_GET "/robot/map/get"
#define ROBOT_PUB_MAP_SET "/robot/map/set"
#define ROBOT_PUB_CMD "/robot/cmd"
// mqtt sub topic
#define ROBOT_SUB_ROBOT_INFO "/robot/info"
#define ROBOT_SUB_ROBOT_CONFIG_GET "/robot/config/get"
#define ROBOT_SUB_ROBOT_MSG "/robot/msg"
#define ROBOT_SUB_ROBOT_ERROR "/robot/error"
#define ROBOT_SUB_TEST "send_string"

#define LOG_MAG_LEN 2048

#define CONNECT_ROBOT_IP "localhost"
#define ROBOT_MQTT_PORT 1883
#define MQTT_BRIDGE_FLAG 1
#define MQTT_BRIDGE_ID "mqtt_ros_bridge"

// 创建订阅者
#define MQTTASYNC_SUB(client, opts)                                            \
  {                                                                            \
    /*if ((rc = MQTTAsync_subscribe(client, ROBOT_SUB_ROBOT_INFO,              \
   QOS_ONE_LEVEL, &opts)) != MQTTASYNC_SUCCESS)                                \
   {                                                                           \
    LogPrint(EN_LOG_LEVEL::ERR, "Failed to start subscribe(%s), return code    \
   %d", ROBOT_SUB_ROBOT_INFO, rc);                                             \
   }                                                                           \
    else                                                                       \
   {                                                                           \
    LogPrint(EN_LOG_LEVEL::DEBUG, "success subscribe(%s), return code %d",     \
   ROBOT_SUB_ROBOT_INFO, rc);                                                  \
   }\*/                                                                        \
    if ((rc = MQTTAsync_subscribe(client, ROBOT_SUB_ROBOT_CONFIG_GET,          \
                                  QOS_TWO_LEVEL, &opts)) !=                    \
        MQTTASYNC_SUCCESS) {                                                   \
      cout << "Error: Failed to start subscribe("                              \
           << ROBOT_SUB_ROBOT_CONFIG_GET << "), return code " << rc << endl;   \
    } else {                                                                   \
      cout << "DEBUG: success subscribe(" << ROBOT_SUB_ROBOT_CONFIG_GET        \
           << "), return code " << rc << endl;                                 \
    }                                                                          \
    if ((rc = MQTTAsync_subscribe(client, ROBOT_SUB_ROBOT_MSG, QOS_TWO_LEVEL,  \
                                  &opts)) != MQTTASYNC_SUCCESS) {              \
      cout << "Error: Failed to start subscribe(" << ROBOT_SUB_ROBOT_MSG       \
           << "), return code " << rc << endl;                                 \
    } else {                                                                   \
      cout << "DEBUG: success subscribe(" << ROBOT_SUB_ROBOT_MSG               \
           << "), return code " << rc << endl;                                 \
    }                                                                          \
    if ((rc = MQTTAsync_subscribe(client, ROBOT_SUB_ROBOT_ERROR,               \
                                  QOS_TWO_LEVEL, &opts)) !=                    \
        MQTTASYNC_SUCCESS) {                                                   \
      cout << "Error: Failed to start subscribe(" << ROBOT_SUB_ROBOT_ERROR     \
           << "), return code " << rc << endl;                                 \
    } else {                                                                   \
      cout << "DEBUG: success subscribe(" << ROBOT_SUB_ROBOT_ERROR             \
           << "), return code " << rc << endl;                                 \
    }                                                                          \
    if ((rc = MQTTAsync_subscribe(client, ROBOT_SUB_TEST, QOS_TWO_LEVEL,       \
                                  &opts)) != MQTTASYNC_SUCCESS) {              \
      cout << "Error: Failed to start subscribe(" << ROBOT_SUB_TEST            \
           << "), return code " << rc << endl;                                 \
    } else {                                                                   \
      cout << "DEBUG: success subscribe(" << ROBOT_SUB_TEST                    \
           << "), return code " << rc << endl;                                 \
    }                                                                          \
    if ((rc = MQTTAsync_subscribe(client, CHARGEBACK, QOS_TWO_LEVEL,           \
                                  &opts)) != MQTTASYNC_SUCCESS) {              \
      cout << "Error: Failed to start subscribe(" << CHARGEBACK                \
           << "), return code " << rc << endl;                                 \
    } else {                                                                   \
      cout << "DEBUG: success subscribe(" << CHARGEBACK << "), return code "   \
           << rc << endl;                                                      \
    }                                                                          \
    if ((rc = MQTTAsync_subscribe(client, MQTTBRIDGE_TO_BROKER, QOS_TWO_LEVEL, \
                                  &opts)) != MQTTASYNC_SUCCESS) {              \
      cout << "Error: Failed to start subscribe(" << MQTTBRIDGE_TO_BROKER      \
           << "), return code " << rc << endl;                                 \
    } else {                                                                   \
      cout << "DEBUG: success subscribe(" << MQTTBRIDGE_TO_BROKER              \
           << "), return code " << rc << endl;                                 \
    }                                                                          \
    if ((rc = MQTTAsync_subscribe(client, STOPCHARGE, QOS_TWO_LEVEL,           \
                                  &opts)) != MQTTASYNC_SUCCESS) {              \
      cout << "Error: Failed to start subscribe(" << STOPCHARGE                \
           << "), return code " << rc << endl;                                 \
    } else {                                                                   \
      cout << "DEBUG: success subscribe(" << STOPCHARGE << "), return code "   \
           << rc << endl;                                                      \
    }                                                                          \
    if ((rc = MQTTAsync_subscribe(client, NAV2TRIG, QOS_TWO_LEVEL, &opts)) !=  \
        MQTTASYNC_SUCCESS) {                                                   \
      cout << "Error: Failed to start subscribe(" << NAV2TRIG                  \
           << "), return code " << rc << endl;                                 \
    } else {                                                                   \
      cout << "DEBUG: success subscribe(" << NAV2TRIG << "), return code "     \
           << rc << endl;                                                      \
    }                                                                          \
    if ((rc = MQTTAsync_subscribe(client, NAV2PREEMPT, QOS_TWO_LEVEL,          \
                                  &opts)) != MQTTASYNC_SUCCESS) {              \
      cout << "Error: Failed to start subscribe(" << NAV2PREEMPT               \
           << "), return code " << rc << endl;                                 \
    } else {                                                                   \
      cout << "DEBUG: success subscribe(" << NAV2PREEMPT << "), return code "  \
           << rc << endl;                                                      \
    }                                                                          \
    if ((rc = MQTTAsync_subscribe(client, NAV2STOP, QOS_TWO_LEVEL, &opts)) !=  \
        MQTTASYNC_SUCCESS) {                                                   \
      cout << "Error: Failed to start subscribe(" << NAV2STOP                  \
           << "), return code " << rc << endl;                                 \
    } else {                                                                   \
      cout << "DEBUG: success subscribe(" << NAV2STOP << "), return code "     \
           << rc << endl;                                                      \
    }                                                                          \
  }

CMqtt m_pMqtt;

CMqtt::CMqtt() : m_pMqttAsyncClient(nullptr) {
  // g_cMsgWork = tsd::make_shared<CMsgWork>();
}

CMqtt::~CMqtt() {}

void onSubscribe(void *context, MQTTAsync_successData *response) {
  // LogPrint(EN_LOG_LEVEL::INFO, "Subscribe succeeded");
  cout << "INFO: Subscribe succeeded" << context << response << endl;
}

void onSubscribeFailure(void *context, MQTTAsync_failureData *response) {
  // LogPrint(EN_LOG_LEVEL::ERR, "Subscribe failed, rc %d", response->code);
  cout << "Error: Subscribe failed, rc " << context << response->code << endl;
}

void onConnect(void *context, MQTTAsync_successData *response) {
  // LogPrint(EN_LOG_LEVEL::INFO, "mqtt Successful connection");
  cout << "INFO: mqtt Successful connection" << context << response << endl;

  CMqtt::m_nConnectStatus = MQTT_CONNECT_STATUS;
  CMqtt::m_bSubscribe = true;
  // std::string m_pTopic="get_string";
  // int m_nQos=QOS_ONE_LEVEL;
  // std::string m_sData="test succeed!!!";
  // m_pMqtt.PublishMsg(m_pTopic,m_nQos,m_sData);

  // 这里先将ros的自定义消息转换为json格式消息
  // Json::Value root;
  // Json::Value Rtime;
  // Json::Value pose;
  // // 将stamp时间转换为字符串
  // root["frame_id"] = "map";

  // // 获取时间戳
  // auto rtime = std::chrono::system_clock::now().time_since_epoch().count();
  // int sec = rtime / lround(floor((pow(10, 9))));
  // int nanasec = rtime % lround(floor((pow(10, 9))));
  // // auto &&ros_time = rclcpp::Time(sec, nanasec);
  // // Rtime = rclcpp::Clock().now();
  // Rtime[0] = sec;
  // Rtime[1] = nanasec;
  // root["stamp"] = Rtime;
  // pose[0] = 0.48;
  // pose[1] = 1.74;
  // pose[2] = -0.707;
  // pose[3] = 0.707;
  // pose[4] = 0.0;
  // pose[5] = 0.0;
  // root["RobotPose"] = pose;
  // // json序列化为std::string类型数据，发送到broker
  // Json::StreamWriterBuilder builder;
  // std::string sJsonData = Json::writeString(builder, root);
  // m_pMqtt.PublishMsg(MQTTBRIDGE_TO_BROKER, QOS_TWO_LEVEL, sJsonData);
  // //QOS_TWO_LEVEL sender 尽力向 Receiver
  // 发送消息，如果发送失败，会继续重试，直到 Receiver 收到消息为止，同时保证
  // Receiver 不会因为消息重传而收到重复的消息 cout << "m_pMqtt.PublishMsg" <<
  // endl;
}

void publishTest() {
  Json::Value root;
  Json::Value Rtime;
  Json::Value pose;
  // 将stamp时间转换为字符串
  root["frame_id"] = "map";

  // 获取时间戳
  auto rtime = std::chrono::system_clock::now().time_since_epoch().count();
  int sec = rtime / lround(floor((pow(10, 9))));
  int nanasec = rtime % lround(floor((pow(10, 9))));
  // auto &&ros_time = rclcpp::Time(sec, nanasec);
  // Rtime = rclcpp::Clock().now();
  Rtime[0] = sec;
  Rtime[1] = nanasec;
  root["stamp"] = Rtime;
  pose[0] = 0.48;
  pose[1] = 1.74;
  pose[2] = -0.707;
  pose[3] = 0.707;
  pose[4] = 0.0;
  pose[5] = 0.0;
  root["RobotPose"] = pose;
  // json序列化为std::string类型数据，发送到broker
  Json::StreamWriterBuilder builder;
  std::string sJsonData = Json::writeString(builder, root);
  m_pMqtt.PublishMsg(MQTTBRIDGE_TO_BROKER, QOS_TWO_LEVEL, sJsonData);
  cout << "m_pMqtt.PublishMsg twice" << endl;
}
void onConnectFailure(void *context, MQTTAsync_failureData *response) {
  // LogPrint(EN_LOG_LEVEL::ERR, "Connect failed, rc %d", response ?
  // response->code : 0);
  cout << "Error: Connect failed, rc " << (response ? response->code : 0)
       << context << endl; // code:表示错误的数字代码
  CMqtt::m_nConnectStatus = MQTT_NOT_CONNECT_STATUS; // 设置为未连接状态
}

void connlost(void *context, char *cause) {
  // LogPrint(EN_LOG_LEVEL::ERR, "mqtt Connection lost, Reconnecting cause: %s",
  // cause);
  cout << "Error: mqtt Connection lost, Reconnecting cause: " << cause
       << context << endl;
  CMqtt::m_nConnectStatus = MQTT_LOST_CONNECT_STATUS;
}

int messageArrived(void *context, char *topicName, int topicLen,
                   MQTTAsync_message *message) {
  // 将获取的消息给成员变量赋值
  std::string sMsg = (char *)message->payload;
  std::string sTopic = topicName;
  g_cMsgWork.notifyMsg(sTopic, sMsg);
  // LogPrint(EN_LOG_LEVEL::DEBUG, "Message arrived");
  // LogPrint(EN_LOG_LEVEL::DEBUG, "topic: %s", topicName);
  // cout << "DEBUG: Message arrived" <<endl;
  cout << "DEBUG: topic " << topicName << endl;
  unsigned int uLen = message->payloadlen;
  if (uLen > LOG_MAG_LEN) {
    // LogPrint(EN_LOG_LEVEL::ERR, "err msg len(%d)", message->payloadlen);
    cout << "Error: err msg len(" << message->payloadlen << ")" << context
         << topicLen << endl;
    return 1;
  } else {
    //%.*s:
    //*指定输出宽度(第一个变量),.*指定必须输出这个宽度，如果所输出的字符串长度大于这个数，则按此宽度输出，如果小于，则输出实际长度。真正输出的内容是第二个变量
    // 下面代码意思：输出长度为message->payloadlen的内容(char*)message->payload)
    // LogPrint(EN_LOG_LEVEL::DEBUG, "message: %.*s", message->payloadlen,
    // (char*)message->payload); cout<<"DEBUG: message:
    // "<<(char*)message->payload<<endl;
  }

  // CPack *pMsg = g_msgRegist.getFunction(msg::E_ROBOT_PUBLISH_MSG, 0);
  // if (pMsg != nullptr)
  // {
  // 	if (pMsg->parseBufData(topicName, (char*)message->payload,
  // message->payloadlen))
  // 	{
  // 		g_gateWayServer.pushMsg(pMsg);
  // 	}
  // 	else
  // 	{
  // 		delete pMsg;
  // 	}

  // 	pMsg = nullptr;
  // }

  MQTTAsync_freeMessage(&message);
  MQTTAsync_free(topicName);

  return 1;
}

void CMqtt::initMqttConnect(
    char *pAddress, int MqttFlag,
    const char *SetClientId) // jc:SetClientId  agent name
{
  m_cCLIENTID = SetClientId;
  m_nMqttFlag = MqttFlag;
  memcpy(m_sAddress, pAddress, 32);
  // LogPrint(EN_LOG_LEVEL::INFO, "mqtt address(%s)", m_sAddress);
  cout << "INFO: mqtt address(" << m_sAddress << ")" << endl;
  MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;

  int rc = MQTTASYNC_FAILURE;
  // 创建mqtt client,连接到给定地址的broker
  if ((rc = MQTTAsync_create(&m_pMqttAsyncClient, m_sAddress, m_cCLIENTID,
                             MQTTCLIENT_PERSISTENCE_NONE, nullptr)) !=
      MQTTASYNC_SUCCESS) {
    // LogPrint(EN_LOG_LEVEL::ERR, "Failed to create mqtt client object, return
    // code %d\n", rc);
    cout << "Error: Failed to create mqtt client object, return code " << rc
         << "\n"
         << endl;
    exit(EXIT_FAILURE);
  }
  // 设置回调函数,包括:连接丢失、收到消息
  if ((rc = MQTTAsync_setCallbacks(m_pMqttAsyncClient, m_pMqttAsyncClient,
                                   connlost, messageArrived, nullptr)) !=
      MQTTASYNC_SUCCESS) {
    // LogPrint(EN_LOG_LEVEL::ERR, "Failed to set callback, return code %d\n",
    // rc);
    cout << "Error: Failed to set callback, return code " << rc << "\n" << endl;
    exit(EXIT_FAILURE);
  }
  // 设置部分连接选项
  conn_opts.keepAliveInterval = 20;
  conn_opts.cleansession = 1;
  conn_opts.onSuccess = onConnect;        // 连接成功的回调
  conn_opts.onFailure = onConnectFailure; // 连接失败的回调
  conn_opts.context = m_pMqttAsyncClient; // 传递给成功/失败的回调的客户端消息?
  // 开始尝试连接
  if ((rc = MQTTAsync_connect(m_pMqttAsyncClient, &conn_opts)) !=
      MQTTASYNC_SUCCESS) {
    // LogPrint(EN_LOG_LEVEL::ERR, "Failed to start connect, return code %d\n",
    // rc);
    cout << "Error: Failed to start connect, return code " << rc << "\n"
         << endl;
    exit(EXIT_FAILURE);
  }
  // 只要断开连接就尝试重新连接
  while (true) {
    sleep(3);
    if (CMqtt::m_nConnectStatus != MQTT_CONNECT_STATUS) {
      reConnect();
    }
    if (CMqtt::m_bSubscribe) //没有订阅话题，就没有消息，没有存值，但是有回调
    {
      cout << "jcjcjc Subscribe ........ " << endl;
      reSubscribe(); // 订阅消息
      CMqtt::m_bSubscribe = false;
      // publishTest();
    }
  }

  MQTTAsync_destroy(&m_pMqttAsyncClient);
}

void onSend(void *context, MQTTAsync_successData *response) {
  // This gets called when a message is acknowledged successfully.
  // LogPrint(EN_LOG_LEVEL::DEBUG, "Message send successfully token %d",
  // response->token);
  cout << "DEBUG: Message send successfully token " << response->token
       << context << endl;
}

void onDisconnectFailure(void *context, MQTTAsync_failureData *response) {
  // LogPrint(EN_LOG_LEVEL::ERR, "Disconnect failed");
  cout << "Error: Disconnect failed" << context << response << endl;
}

void onDisconnect(void *context, MQTTAsync_successData *response) {
  // LogPrint(EN_LOG_LEVEL::DEBUG, "Successful disconnection");
  cout << "DEBUG: Successful disconnection" << context << response << endl;

  CMqtt::m_nConnectStatus = MQTT_DIS_CONNECT_STATUS;
}

void onSendFailure(void *context, MQTTAsync_failureData *response) {
  MQTTAsync client = (MQTTAsync)context;
  MQTTAsync_disconnectOptions opts = MQTTAsync_disconnectOptions_initializer;
  int rc;
  // LogPrint(EN_LOG_LEVEL::DEBUG, "Message send failed token %d error code %d",
  // response->token, response->code);
  cout << "DEBUG: Message send failed token " << response->token
       << " error code " << response->code << endl;
  opts.onSuccess = onDisconnect;
  opts.onFailure = onDisconnectFailure;
  opts.context = client;
  if ((rc = MQTTAsync_disconnect(client, &opts)) != MQTTASYNC_SUCCESS) {
    // LogPrint(EN_LOG_LEVEL::ERR, "Failed to start disconnect, return code %d",
    // rc);
    cout << "Error: Failed to start disconnect, return code " << rc << endl;
    CMqtt::m_nConnectStatus = MQTT_EXCEPTION_CONNECT_STATUS;
    //	exit(EXIT_FAILURE);
  }
}

void CMqtt::PublishMsg(std::string pTopic, int nQos, std::string sData) {
  MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
  MQTTAsync_responseOptions pub_opts = MQTTAsync_responseOptions_initializer;
  pub_opts.onSuccess = onSend;
  pub_opts.onFailure = onSendFailure;
  pub_opts.context = m_pMqttAsyncClient;

  pubmsg.qos = nQos;
  pubmsg.retained = 0;
  pubmsg.payload = (void *)sData.c_str();
  pubmsg.payloadlen = sData.length();

  int rc = -1;
  if ((rc = MQTTAsync_sendMessage(m_pMqttAsyncClient, pTopic.c_str(), &pubmsg,
                                  &pub_opts)) != MQTTASYNC_SUCCESS) {
    // LogPrint(EN_LOG_LEVEL::ERR, "Failed to start sendMessage, return code
    // %d\n", rc);
    cout << "Error: Failed to start sendMessage, return code " << rc << "\n"
         << endl;
    CMqtt::m_nConnectStatus = MQTT_EXCEPTION_CONNECT_STATUS;
    //	exit(EXIT_FAILURE);
  }
}

// 尝试重新连接
void CMqtt::reConnect() {
  // LogPrint(EN_LOG_LEVEL::WARN, "mqtt lost Connection Reconnecting");
  cout << "WARN: mqtt lost Connection Reconnecting" << endl;
  MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
  conn_opts.keepAliveInterval = 20;
  conn_opts.cleansession = 1;
  conn_opts.onSuccess = onConnect;
  conn_opts.onFailure = onConnectFailure;
  int rc = -1;
  if ((rc = MQTTAsync_connect(m_pMqttAsyncClient, &conn_opts)) !=
      MQTTASYNC_SUCCESS) {
    // LogPrint(EN_LOG_LEVEL::ERR, "Failed to start connect, return code %d",
    // rc);
    cout << "Error: Failed to start connect, return code " << rc << endl;
  }
}

void CMqtt::reSubscribe() {
  MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
  opts.onSuccess = onSubscribe;
  opts.onFailure = onSubscribeFailure;
  opts.context = m_pMqttAsyncClient;
  int rc = -1;
  MQTTASYNC_SUB(m_pMqttAsyncClient, opts);
  // MQTTCLIENT_SUB(m_pMqttAsyncClient, opts)
  // if (m_nMqttFlag != 0)
  //   MQTTCLIENT_SUB(m_pMqttAsyncClient, opts)
  // else
  //   MQTTCBRIDGE_SUB(m_pMqttAsyncClient, opts); // mqtt_bridge
}

void CMqtt::mqttBridgeThread() {
  char sAddress[32] = {0};
  sprintf(sAddress, "tcp://localhost:1883");

  // CMqtt *pMqtt = new CMqtt;
  m_pMqtt.initMqttConnect(sAddress, MQTT_BRIDGE_FLAG, MQTT_BRIDGE_ID);
}

void CMqtt::startEcsMsgJobThread() {
  g_cMsgWork.startServer();
} // mqtt连接成功之后由mqtt底层函数调用
