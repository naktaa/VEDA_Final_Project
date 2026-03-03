// mqtt_pub.hpp (혹은 rtsp_meta_forward.cpp 상단에 같이 넣어도 됨)
#pragma once
#include <mosquitto.h>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <iostream>

class MqttPublisher {
public:
    MqttPublisher(std::string host="127.0.0.1", int port=1883,
                  std::string topic="wiserisk/events",
                  std::string clientId="rtsp-meta-forwarder")
        : host_(std::move(host)), port_(port), topic_(std::move(topic)), clientId_(std::move(clientId)) {}

    bool start() {
        mosquitto_lib_init();

        mosq_ = mosquitto_new(clientId_.c_str(), true, this);
        if (!mosq_) {
            std::cerr << "[MQTT] mosquitto_new failed\n";
            return false;
        }

        mosquitto_connect_callback_set(mosq_, &MqttPublisher::on_connect_static);
        mosquitto_disconnect_callback_set(mosq_, &MqttPublisher::on_disconnect_static);

        // (선택) LWT: 프로세스 죽으면 offline 알림
        const std::string lwt = R"({"service":"rtsp_meta_forward","status":"offline"})";
        mosquitto_will_set(mosq_, topic_.c_str(), (int)lwt.size(), lwt.c_str(), 1, true);

        // loop thread
        running_ = true;
        loopThread_ = std::thread([this] { loop_worker(); });

        // 최초 연결
        request_reconnect();
        return true;
    }

    void stop() {
        running_ = false;
        if (loopThread_.joinable()) loopThread_.join();

        if (mosq_) {
            mosquitto_disconnect(mosq_);
            mosquitto_destroy(mosq_);
            mosq_ = nullptr;
        }
        mosquitto_lib_cleanup();
    }

    bool publish_json(const std::string& jsonPayload, int qos=0, bool retain=false) {
        if (!connected_) return false;

        int rc = mosquitto_publish(
            mosq_, nullptr, topic_.c_str(),
            (int)jsonPayload.size(), jsonPayload.c_str(),
            qos, retain
        );
        if (rc != MOSQ_ERR_SUCCESS) {
            // publish 실패하면 연결 문제일 가능성 높음 -> reconnect 시도
            connected_ = false;
            request_reconnect();
            return false;
        }
        return true;
    }

    bool is_connected() const { return connected_; }

private:
    void loop_worker() {
        while (running_) {
            // 연결이 필요하면 시도
            if (needReconnect_) {
                needReconnect_ = false;
                connect_with_backoff();
            }

            // mosquitto I/O loop
            if (mosq_) {
                int rc = mosquitto_loop(mosq_, 100 /*ms*/, 1);
                if (rc != MOSQ_ERR_SUCCESS) {
                    connected_ = false;
                    // 네트워크 끊김 등 -> 재연결 플래그
                    request_reconnect();
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        }
    }

    void connect_with_backoff() {
        int backoffMs = 200;
        while (running_ && !connected_) {
            int rc = mosquitto_connect(mosq_, host_.c_str(), port_, 30 /*keepalive*/);
            if (rc == MOSQ_ERR_SUCCESS) {
                // on_connect callback에서 connected_ true로 바뀜
                // 여기서는 loop가 돌아야 실제 연결 확정됨
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                return;
            }
            std::cerr << "[MQTT] connect failed rc=" << rc << " retry in " << backoffMs << "ms\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(backoffMs));
            backoffMs = std::min(backoffMs * 2, 5000);
        }
    }

    void request_reconnect() { needReconnect_ = true; }

    static void on_connect_static(struct mosquitto* m, void* obj, int rc) {
        auto* self = static_cast<MqttPublisher*>(obj);
        if (rc == 0) {
            self->connected_ = true;
            // (선택) online retain
            const std::string online = R"({"service":"rtsp_meta_forward","status":"online"})";
            mosquitto_publish(m, nullptr, self->topic_.c_str(), (int)online.size(), online.c_str(), 1, true);
            std::cerr << "[MQTT] connected\n";
        } else {
            self->connected_ = false;
            std::cerr << "[MQTT] connect callback rc=" << rc << "\n";
        }
    }

    static void on_disconnect_static(struct mosquitto*, void* obj, int) {
        auto* self = static_cast<MqttPublisher*>(obj);
        self->connected_ = false;
        self->request_reconnect();
        std::cerr << "[MQTT] disconnected\n";
    }

private:
    std::string host_;
    int port_;
    std::string topic_;
    std::string clientId_;

    struct mosquitto* mosq_ = nullptr;
    std::thread loopThread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> needReconnect_{false};
};
