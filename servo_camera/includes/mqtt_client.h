#ifndef _MQTT_H_
#define _MQTT_H_
#include <string>
#include <functional>

// 수신 콜백 타입: (topic, payload)
using MqttMsgCallback = std::function<void(const std::string &, const std::string &)>;

// MQTT 시작(구독 시작). 내부 스레드로 loop 돌리게 만들 수 있음.
bool mqtt_start_subscriber(const std::string &host, int port,
                           const std::string &topic,
                           MqttMsgCallback cb);

// MQTT 종료
void mqtt_stop();
#endif
