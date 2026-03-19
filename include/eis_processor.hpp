#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

/**
 * @brief LK Optical Flow + Kalman Filter 기반 EIS 프로세서
 */
class EisProcessor {
public:
    struct Config {
        // LK parameters
        int max_features = 200;
        double feature_quality = 0.01;
        double feature_min_dist = 30.0;

        // Kalman parameters
        double kf_q = 0.004;
        double kf_r = 0.5;

        // Visuals
        int border_crop = 30;
        bool debug_overlay = true;

        void load_from_ini(const std::string& path);
    };

    EisProcessor();
    ~EisProcessor() = default;

    /**
     * @brief 프레임을 입력받아 EIS 보정을 수행하고 결과를 반환 (원본 프레임 수정)
     */
    void process(cv::Mat& frame);

    const Config& config() const { return config_; }

private:
    struct KalmanState {
        double x = 0.0;  // 상태 추정값
        double P = 1.0;  // 오차 공분산
        double Q = 0.004;
        double R = 0.5;
        double sum = 0.0; // 누적 궤적

        void init(double q, double r) {
            x = 0.0; P = 1.0; Q = q; R = r; sum = 0.0;
        }
        void update(double measurement) {
            sum += measurement;
            double x_pred = x;
            double P_pred = P + Q;
            double K = P_pred / (P_pred + R);
            x = x_pred + K * (sum - x_pred);
            P = (1.0 - K) * P_pred;
        }
        double diff() const { return x - sum; }
    };

    Config config_;
    KalmanState kf_theta_, kf_tx_, kf_ty_;
    
    cv::Mat prev_gray_;
    cv::Mat prev_frame_;
    int frame_count_ = 0;
};
