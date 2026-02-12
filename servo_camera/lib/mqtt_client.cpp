#include "mqtt_client.h"
#include <mosquitto.h>
#include <atomic>

static mosquitto *g_mosq = nullptr;
static std::atomic<bool> g_running{false};
static MqttMsgCallback g_cb;

static void on_message(struct mosquitto *, void *, const struct mosquitto_message *msg)
{
    if (!msg || !msg->topic || !msg->payload)
        return;

    std::string topic = msg->topic;
    std::string payload((char *)msg->payload, (size_t)msg->payloadlen);

    if (g_cb)
        g_cb(topic, payload);
}

bool mqtt_start_subscriber(const std::string &host, int port,
                           const std::string &topic,
                           MqttMsgCallback cb)
{
    mosquitto_lib_init();

    g_cb = cb;
    g_mosq = mosquitto_new(nullptr, true, nullptr);
    if (!g_mosq)
        return false;

    mosquitto_message_callback_set(g_mosq, on_message);

    if (mosquitto_connect(g_mosq, host.c_str(), port, 60) != MOSQ_ERR_SUCCESS)
        return false;

    if (mosquitto_subscribe(g_mosq, nullptr, topic.c_str(), 0) != MOSQ_ERR_SUCCESS)
        return false;

    // ✅ main 안 막히게 내부 스레드로 loop 돌림
    if (mosquitto_loop_start(g_mosq) != MOSQ_ERR_SUCCESS)
        return false;

    g_running = true;
    return true;
}

void mqtt_stop()
{
    if (!g_mosq)
        return;

    g_running = false;
    mosquitto_loop_stop(g_mosq, true);
    mosquitto_disconnect(g_mosq);
    mosquitto_destroy(g_mosq);
    g_mosq = nullptr;

    mosquitto_lib_cleanup();
}
