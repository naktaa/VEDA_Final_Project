#include "eis_capture.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <gst/gstmeta.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#include "eis_common.hpp"
#include "eis_globals.hpp"
#include "gyro_eis.hpp"
#include "libcamera_grabber.hpp"
#include "rtsp_server.hpp"

using namespace cv;

struct CapturedFrame {
    cv::Mat frame;
    double time_ms = 0.0;
    TsSource src = TsSource::ARRIVAL;
    int64_t sensor_ts_ns = 0;
    int exp_us = 0;
    uint64_t index = 0;
};

static cv::Mat centerCropAndResize(const cv::Mat& in, double cropPercent) {
    if (in.empty() || cropPercent <= 0.0) return in;
    int cw = (int)(in.cols * cropPercent / 200.0);
    int ch = (int)(in.rows * cropPercent / 200.0);
    cv::Rect roi(cw, ch, in.cols - 2 * cw, in.rows - 2 * ch);
    cv::Mat out;
    cv::resize(in(roi), out, in.size(), 0, 0, cv::INTER_LINEAR);
    return out;
}

static cv::Mat ensureBGR(const cv::Mat& in) {
    if (in.empty()) return cv::Mat();
    cv::Mat out = in;
    if (out.type() == CV_8UC4) {
        if (g_libcamera_xrgb.load()) {
            cv::Mat tmp(out.rows, out.cols, CV_8UC3);
            // XRGB -> BGR (X,R,G,B)
            int from_to[] = {3, 0, 2, 1, 1, 2};
            cv::mixChannels(&out, 1, &tmp, 1, from_to, 3);
            out = tmp;
        } else {
            // XBGR8888 in memory is typically B,G,R,X -> BGRA
            cv::cvtColor(out, out, cv::COLOR_BGRA2BGR);
        }
    }
    else if (out.type() == CV_8UC1)
        cv::cvtColor(out, out, cv::COLOR_GRAY2BGR);
    else if (out.type() != CV_8UC3) {
        cv::Mat tmp;
        out.convertTo(tmp, CV_8U);
        if (tmp.channels() == 1)
            cv::cvtColor(tmp, out, cv::COLOR_GRAY2BGR);
        else
            out = tmp;
    }
    if (out.cols != G_WIDTH || out.rows != G_HEIGHT)
        cv::resize(out, out, cv::Size(G_WIDTH, G_HEIGHT), 0, 0, cv::INTER_LINEAR);
    if (!out.isContinuous()) out = out.clone();
    return out;
}

static bool get_reference_timestamp_ns(GstBuffer* buffer, int64_t& out_ns) {
    gpointer state = nullptr;
    while (true) {
        GstMeta* meta = gst_buffer_iterate_meta(buffer, &state);
        if (!meta) break;
        if (meta->info->api == GST_REFERENCE_TIMESTAMP_META_API_TYPE) {
            GstReferenceTimestampMeta* r = (GstReferenceTimestampMeta*)meta;
            out_ns = (int64_t)r->timestamp;
            return true;
        }
    }
    return false;
}

static bool integrate_gyro_delta(const GyroBuffer& buf, double t0_ms, double t1_ms,
                                 cv::Vec3d& out_delta, GyroRangeInfo* info = nullptr) {
    out_delta = cv::Vec3d(0, 0, 0);
    if (t1_ms <= t0_ms) return false;
    std::vector<GyroSample> samples;
    GyroRangeInfo tmp;
    if (!buf.get_range(t0_ms, t1_ms, samples, &tmp)) {
        if (info) *info = tmp;
        return false;
    }
    double t_prev = t0_ms;
    cv::Vec3d w_prev = samples.front().w_rad;
    int used = 0;
    for (const auto& s : samples) {
        double t = std::clamp(s.t_ms, t0_ms, t1_ms);
        if (t <= t_prev) { w_prev = s.w_rad; continue; }
        double dt = (t - t_prev) / 1000.0;
        out_delta += w_prev * dt;
        t_prev = t;
        w_prev = s.w_rad;
        used++;
    }
    if (t_prev < t1_ms) {
        double dt = (t1_ms - t_prev) / 1000.0;
        out_delta += w_prev * dt;
    }
    if (info) {
        *info = tmp;
        info->used = used;
    }
    return true;
}

static cv::Vec3d quat_to_euler_deg(const Quaternion& q) {
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    double pitch = 0.0;
    if (std::abs(sinp) >= 1.0) pitch = std::copysign(CV_PI / 2.0, sinp);
    else pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    return cv::Vec3d(roll, pitch, yaw) * (180.0 / CV_PI);
}

static double compute_corr(const std::deque<std::pair<double, double>>& buf) {
    if (buf.size() < 2) return 0.0;
    double sumx = 0, sumy = 0, sumxx = 0, sumyy = 0, sumxy = 0;
    const double n = (double)buf.size();
    for (const auto& p : buf) {
        sumx += p.first;
        sumy += p.second;
        sumxx += p.first * p.first;
        sumyy += p.second * p.second;
        sumxy += p.first * p.second;
    }
    double denom = std::sqrt((n * sumxx - sumx * sumx) * (n * sumyy - sumy * sumy));
    if (denom < 1e-9) return 0.0;
    return (n * sumxy - sumx * sumy) / denom;
}

struct FlowDebugInfo {
    double dx = 0.0;
    double dy = 0.0;
    double da = 0.0;
    cv::Point2f mean_all{0,0};
    cv::Point2f mean_center{0,0};
    cv::Point2f mean_edge{0,0};
    int n_all = 0;
    int n_center = 0;
    int n_edge = 0;
};

static bool draw_optical_flow_overlay(const cv::Mat& prev_gray, const cv::Mat& curr_gray,
                                      cv::Mat& canvas, FlowDebugInfo& info) {
    if (prev_gray.empty() || curr_gray.empty()) return false;
    std::vector<cv::Point2f> feat_prev, feat_curr;
    std::vector<uchar> status;
    std::vector<float> err_vec;
    goodFeaturesToTrack(prev_gray, feat_prev,
                        (int)g_lk_max_features.load(),
                        g_lk_quality.load(),
                        g_lk_min_dist.load());
    if (feat_prev.size() < (size_t)g_lk_trans_min_features.load()) return false;
    calcOpticalFlowPyrLK(prev_gray, curr_gray, feat_prev, feat_curr, status, err_vec);
    std::vector<cv::Point2f> gp, gc;
    const cv::Point2f center(canvas.cols * 0.5f, canvas.rows * 0.5f);
    const float center_r = 0.25f * std::min(canvas.cols, canvas.rows);
    for (size_t i = 0; i < status.size(); ++i) {
        if (!status[i]) continue;
        cv::Point2f p0 = feat_prev[i];
        cv::Point2f p1 = feat_curr[i];
        gp.push_back(p0);
        gc.push_back(p1);
        cv::Point2f v = p1 - p0;
        info.mean_all += v;
        info.n_all++;
        float dist = cv::norm(p0 - center);
        if (dist < center_r) {
            info.mean_center += v;
            info.n_center++;
        } else {
            info.mean_edge += v;
            info.n_edge++;
        }
        cv::line(canvas, p0, p1, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        cv::circle(canvas, p0, 1, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
    }
    if (info.n_all > 0) {
        info.mean_all *= (1.0f / info.n_all);
    }
    if (info.n_center > 0) {
        info.mean_center *= (1.0f / info.n_center);
    }
    if (info.n_edge > 0) {
        info.mean_edge *= (1.0f / info.n_edge);
    }
    if (gp.size() >= 6) {
        cv::Mat affine = estimateAffinePartial2D(gp, gc);
        if (!affine.empty()) {
            info.dx = affine.at<double>(0, 2);
            info.dy = affine.at<double>(1, 2);
            info.da = std::atan2(affine.at<double>(1, 0), affine.at<double>(0, 0));
        }
    }
    return info.n_all > 0;
}

void capture_loop() {
    TsSourcePref pref = (TsSourcePref)g_ts_pref.load();
    bool want_libcamera = (pref == TsSourcePref::AUTO || pref == TsSourcePref::SENSOR);
    bool use_libcamera = false;
    LibcameraGrabber lc;

    GstElement* pipeline = nullptr;
    GstElement* sink = nullptr;
    GstAppSink* appsink = nullptr;
    GError* err = nullptr;

    bool ts_pref_warned = false;
    bool sensor_meta_warned = false;

    if (want_libcamera) {
        if (lc.init()) {
            use_libcamera = true;
        } else {
            fprintf(stderr, "[TS] libcamera init failed -> fallback GStreamer\n");
        }
    }

    if (!use_libcamera) {
        std::string cap_pipe =
            "libcamerasrc "
            "! video/x-raw,format=RGBx,width=" + std::to_string(G_WIDTH) +
            ",height=" + std::to_string(G_HEIGHT) +
            ",framerate=" + std::to_string(G_FPS) + "/1 "
            "! videoconvert ! video/x-raw,format=BGR "
            "! appsink name=appsink drop=true max-buffers=1 sync=false";

        pipeline = gst_parse_launch(cap_pipe.c_str(), &err);
        if (!pipeline) {
            fprintf(stderr, "[ERR] pipeline\n");
            if (err) g_error_free(err);
            g_running = false;
            return;
        }
        if (err) {
            fprintf(stderr, "[WARN] %s\n", err->message);
            g_error_free(err);
        }

        sink = gst_bin_get_by_name(GST_BIN(pipeline), "appsink");
        if (!sink) {
            fprintf(stderr, "[ERR] appsink\n");
            gst_object_unref(pipeline);
            g_running = false;
            return;
        }
        appsink = GST_APP_SINK(sink);
        gst_app_sink_set_emit_signals(appsink, FALSE);
        gst_app_sink_set_drop(appsink, TRUE);
        gst_app_sink_set_max_buffers(appsink, 1);

        if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            fprintf(stderr, "[ERR] pipeline start\n");
            gst_object_unref(sink);
            gst_object_unref(pipeline);
            g_running = false;
            return;
        }
    }

    auto pull_frame_gst = [&](CapturedFrame& out) -> bool {
        GstSample* sample = gst_app_sink_try_pull_sample(appsink, 100000000);
        if (!sample) return false;
        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstCaps* caps = gst_sample_get_caps(sample);
        if (!buffer || !caps) {
            gst_sample_unref(sample);
            return false;
        }
        GstVideoInfo info;
        bool info_ok = gst_video_info_from_caps(&info, caps);
        int w = info_ok ? (int)info.width : G_WIDTH;
        int h = info_ok ? (int)info.height : G_HEIGHT;
        int stride = info_ok ? (int)info.stride[0] : (w * 3);
        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return false;
        }
        cv::Mat frame(h, w, CV_8UC3, map.data, stride);
        out.frame = frame.clone();
        gst_buffer_unmap(buffer, &map);

        out.src = TsSource::ARRIVAL;
        out.time_ms = now_ms();
        int64_t ref_ns = 0;
        if (get_reference_timestamp_ns(buffer, ref_ns)) {
            out.src = TsSource::SENSOR;
            out.time_ms = (ref_ns - g_time_origin_raw_ns) / 1e6;
        } else if (GST_CLOCK_TIME_IS_VALID(GST_BUFFER_PTS(buffer))) {
            out.src = TsSource::PTS;
            out.time_ms = (int64_t)GST_BUFFER_PTS(buffer) / 1e6;
        }

        gst_sample_unref(sample);
        return !out.frame.empty();
    };

    auto pull_frame_lc = [&](CapturedFrame& out) -> bool {
        if (!lc.get_frame(out.frame, out.time_ms, out.src,
                          out.sensor_ts_ns, out.exp_us, out.index)) {
            return false;
        }
        if (out.src != TsSource::SENSOR && !sensor_meta_warned) {
            fprintf(stderr, "[TS] SensorTimestamp missing -> using ARRIVAL\n");
            sensor_meta_warned = true;
        }
        return !out.frame.empty();
    };

    auto pull_frame = [&](CapturedFrame& out) -> bool {
        return use_libcamera ? pull_frame_lc(out) : pull_frame_gst(out);
    };

    CameraIntrinsics intr = CameraIntrinsics::from_fov(G_WIDTH, G_HEIGHT, HFOV_DEG, VFOV_DEG);
    EISConfig cfg;
    cfg.smooth_alpha = g_smooth_alpha.load();
    cfg.roll_gain = ROLL_GAIN;
    cfg.pitch_gain = PITCH_GAIN;
    cfg.yaw_gain = YAW_GAIN;
    cfg.max_roll_rad = g_max_roll_rad.load();
    cfg.max_pitch_rad = g_max_pitch_rad.load();
    cfg.max_yaw_rad = g_max_yaw_rad.load();
    cfg.crop_percent = FIXED_CROP_PERCENT;
    cfg.enable_crop = false; // crop after all warps

    GyroEIS gyro_eis(intr, cfg, &g_gyro_buffer);

    RunProfile profile = (RunProfile)g_profile.load();
    const bool profile_run = (profile == RunProfile::RUN);
    const bool profile_calib = (profile == RunProfile::CALIB);
    const bool profile_debug = (profile == RunProfile::DEBUG);

    auto estimate_lk_transform = [&](const Mat& prev, const Mat& curr,
                                     double& out_dx, double& out_dy, double& out_da) -> bool {
        std::vector<Point2f> feat_prev, feat_curr;
        std::vector<uchar> status;
        std::vector<float> err_vec;
        goodFeaturesToTrack(prev, feat_prev,
                            (int)g_lk_max_features.load(),
                            g_lk_quality.load(),
                            g_lk_min_dist.load());
        if (feat_prev.size() < (size_t)g_lk_trans_min_features.load()) return false;
        calcOpticalFlowPyrLK(prev, curr, feat_prev, feat_curr, status, err_vec);
        std::vector<Point2f> gp, gc;
        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) { gp.push_back(feat_prev[i]); gc.push_back(feat_curr[i]); }
        }
        if (gp.size() < 6) return false;
        Mat affine = estimateAffinePartial2D(gp, gc);
        if (affine.empty()) return false;
        out_dx = affine.at<double>(0, 2);
        out_dy = affine.at<double>(1, 2);
        out_da = atan2(affine.at<double>(1, 0), affine.at<double>(0, 0));
        return true;
    };

    guint64 frameIdx = 0;
    Mat prev_lk_gray;
    bool prev_lk_ok = false;
    double prev_frame_time_ms = 0.0;
    bool prev_time_ok = false;

    struct TransState {
        Mat prev_gray;
        Mat prev_small;
        bool ok = false;
        double path_x = 0.0;
        double path_y = 0.0;
        double smooth_x = 0.0;
        double smooth_y = 0.0;
        bool smooth_init = false;
        int frame_count = 0;
        double last_corr_x = 0.0;
        double last_corr_y = 0.0;
        double scale = 0.5;
    };
    TransState trans;
    trans.scale = g_lk_trans_scale.load();
    Mat flow_prev_gray;
    bool flow_prev_ok = false;

    struct CalibFrame { double t_ms; double lk_da; };
    std::vector<CalibFrame> calib_frames;
    double calib_start_ms = -1.0;
    bool do_offset_sweep = profile_calib ? true : (profile_debug ? g_offset_sweep.load() : false);
    bool offset_calibrated = !do_offset_sweep;
    double time_offset_ms = g_manual_imu_offset_ms;

    const double SWEEP_MAX_ERR_MS = 8.0;
    const int SWEEP_MIN_SAMPLES = 5;

    double imu_err_sum = 0.0;
    double imu_err_max = 0.0;
    int imu_err_cnt = 0;
    TsSource last_ts_src = TsSource::ARRIVAL;

    std::deque<std::pair<double, double>> corr_buf;
    double corr_val = 0.0;
    const int corr_window = std::max(10, G_FPS);

    EisMode last_mode = (EisMode)g_mode.load();
    bool lk_disabled_warned = false;
    double last_cost_ms = 0.0;
    bool color_logged = false;

    while (g_running) {
        int64_t frame_start_ns = clock_ns(CLOCK_MONOTONIC_RAW);
        const double budget_ms = 1000.0 / std::max(1, G_FPS);
        bool skip_heavy = (last_cost_ms > budget_ms);
        CapturedFrame cap;
        if (!pull_frame(cap) || cap.frame.empty()) continue;

        Mat frame = ensureBGR(cap.frame);
        if (frame.empty()) continue;
        double frame_time_ms = cap.time_ms;
        TsSource ts_src = cap.src;
        int64_t sensor_ts_ns = cap.sensor_ts_ns;
        int exp_us = cap.exp_us;
        uint64_t cap_index = cap.index;

        if (FLIP_VERTICAL) {
            cv::flip(frame, frame, -1);
        }

        if (!color_logged && !frame.empty()) {
            cv::Scalar m = cv::mean(frame);
            fprintf(stderr, "[COLOR] mean B=%.1f G=%.1f R=%.1f\n", m[0], m[1], m[2]);
            color_logged = true;
        }

        if (ts_src != last_ts_src) {
            fprintf(stderr, "[TS] source=%s\n", ts_source_str(ts_src));
            last_ts_src = ts_src;
        }

        EisMode mode = (EisMode)g_mode.load();
        if (mode != last_mode) {
            gyro_eis.reset();
            prev_lk_gray.release();
            prev_lk_ok = false;
            trans = TransState{};
            trans.scale = g_lk_trans_scale.load();
            flow_prev_gray.release();
            flow_prev_ok = false;
            prev_time_ok = false;
            calib_frames.clear();
            calib_start_ms = -1.0;
            do_offset_sweep = profile_calib ? true : (profile_debug ? g_offset_sweep.load() : false);
            offset_calibrated = !do_offset_sweep;
            time_offset_ms = g_manual_imu_offset_ms;
            imu_err_sum = 0.0;
            imu_err_max = 0.0;
            imu_err_cnt = 0;
            corr_buf.clear();
            corr_val = 0.0;
            lk_disabled_warned = false;
            last_mode = mode;
            fprintf(stderr, "[MODE] switched to %s\n", mode_str(mode));
        }

        if (mode != EisMode::GYRO && !lk_disabled_warned) {
            fprintf(stderr,
                    "[MODE] %s requested -> gyro rotation + LK translation (if enabled)\n",
                    mode_str(mode));
            lk_disabled_warned = true;
        }

        int log_every = profile_debug ? g_log_every_frames.load() : 0;
        bool dbg = profile_debug && g_debug_overlay.load();
        bool need_lk = do_offset_sweep || dbg || (log_every > 0);

        Mat curr_gray;
        double lk_dx = 0.0;
        double lk_dy = 0.0;
        double lk_da = 0.0;
        bool lk_ok = false;
        if (need_lk) {
            cvtColor(frame, curr_gray, COLOR_BGR2GRAY);
            if (prev_lk_ok) {
                lk_ok = estimate_lk_transform(prev_lk_gray, curr_gray, lk_dx, lk_dy, lk_da);
            }
            prev_lk_gray = curr_gray.clone();
            prev_lk_ok = true;
        }

        if (g_imu_ready.load() && do_offset_sweep) {
            if (calib_start_ms < 0.0) calib_start_ms = frame_time_ms;

            if (!offset_calibrated && (frame_time_ms - calib_start_ms) <= OFFSET_CALIB_DURATION_MS) {
                if (lk_ok) calib_frames.push_back({frame_time_ms, lk_da});
            } else if (!offset_calibrated && (frame_time_ms - calib_start_ms) > OFFSET_CALIB_DURATION_MS) {
                if (calib_frames.size() >= 8 && g_gyro_buffer.size() >= 10) {
                    double base_off = g_manual_imu_offset_ms;
                    int pair_total = 0;
                    int pair_ok = 0;
                    int pair_low_samples = 0;
                    int pair_out_of_range = 0;
                    double lk_motion_sum = 0.0;
                    for (size_t i = 1; i < calib_frames.size(); ++i) {
                        pair_total++;
                        lk_motion_sum += std::abs(calib_frames[i].lk_da);
                        double t0 = calib_frames[i - 1].t_ms + base_off;
                        double t1 = calib_frames[i].t_ms + base_off;
                        cv::Vec3d delta;
                        GyroRangeInfo rinfo;
                        if (!integrate_gyro_delta(g_gyro_buffer, t0, t1, delta, &rinfo)) {
                            pair_low_samples++;
                            continue;
                        }
                        if (rinfo.used < SWEEP_MIN_SAMPLES) {
                            pair_low_samples++;
                            continue;
                        }
                        double err0 = std::abs(t0 - rinfo.min_ts);
                        double err1 = std::abs(t1 - rinfo.max_ts);
                        if (err0 > SWEEP_MAX_ERR_MS || err1 > SWEEP_MAX_ERR_MS) {
                            pair_out_of_range++;
                            continue;
                        }
                        pair_ok++;
                    }
                    auto cost_for = [&](double delta_ms) -> double {
                        double off_ms = base_off + delta_ms;
                        double sum = 0.0;
                        int n = 0;
                        for (size_t i = 1; i < calib_frames.size(); ++i) {
                            double t0 = calib_frames[i - 1].t_ms + off_ms;
                            double t1 = calib_frames[i].t_ms + off_ms;
                            cv::Vec3d delta;
                            GyroRangeInfo rinfo;
                            if (!integrate_gyro_delta(g_gyro_buffer, t0, t1, delta, &rinfo)) continue;
                            if (rinfo.used < SWEEP_MIN_SAMPLES) continue;
                            double err0 = std::abs(t0 - rinfo.min_ts);
                            double err1 = std::abs(t1 - rinfo.max_ts);
                            if (err0 > SWEEP_MAX_ERR_MS || err1 > SWEEP_MAX_ERR_MS) continue;
                            double imu_dyaw = delta[2];
                            double diff = calib_frames[i].lk_da - imu_dyaw;
                            sum += diff * diff;
                            n++;
                        }
                        if (n == 0) return 1e18;
                        return sum / n;
                    };

                    double best_off = 0.0;
                    double best_cost = 1e18;
                    for (double off = -OFFSET_COARSE_RANGE_MS; off <= OFFSET_COARSE_RANGE_MS; off += OFFSET_COARSE_STEP_MS) {
                        double c = cost_for(off);
                        if (c < best_cost) { best_cost = c; best_off = off; }
                    }
                    for (double off = best_off - OFFSET_FINE_RANGE_MS; off <= best_off + OFFSET_FINE_RANGE_MS; off += OFFSET_FINE_STEP_MS) {
                        double c = cost_for(off);
                        if (c < best_cost) { best_cost = c; best_off = off; }
                    }
                    if (best_cost < 1e17) {
                        time_offset_ms = base_off + best_off;
                    } else {
                        time_offset_ms = base_off;
                        fprintf(stderr,
                                "[SYNC] sweep failed (valid=%d/%d low_samples=%d out_of_range=%d lk_motion=%.4f)\n",
                                pair_ok, pair_total, pair_low_samples, pair_out_of_range, lk_motion_sum);
                    }
                } else {
                    time_offset_ms = g_manual_imu_offset_ms;
                }
                offset_calibrated = true;
                gyro_eis.reset();
                prev_time_ok = false;
                fprintf(stderr, "[SYNC] offset=%.3f ms (base=%.3f, frames=%zu)\n",
                        time_offset_ms, g_manual_imu_offset_ms, calib_frames.size());
            }
        }

        double target_curr_ms = frame_time_ms + time_offset_ms;
        double target_prev_ms = 0.0;
        bool have_target_prev = false;
        cv::Vec3d gyro_delta(0, 0, 0);
        GyroRangeInfo delta_range;
        bool gyro_delta_ok = false;
        if (prev_time_ok && g_imu_ready.load()) {
            target_prev_ms = prev_frame_time_ms + time_offset_ms;
            have_target_prev = true;
            gyro_delta_ok = integrate_gyro_delta(g_gyro_buffer, target_prev_ms, target_curr_ms, gyro_delta, &delta_range);
        }
        prev_frame_time_ms = frame_time_ms;
        prev_time_ok = true;

        gyro_eis.set_warp_mode((GyroWarpMode)g_gyro_warp_mode.load());
        GyroEISDebug dbginfo{};
        Mat gyro_out = frame;
        bool gyro_ok = false;
        if (!profile_calib && mode != EisMode::LK) {
            gyro_ok = gyro_eis.process(frame, frame_time_ms, time_offset_ms, gyro_out, &dbginfo);
        }

        Mat stabilized = gyro_out;
        bool trans_ok = false;
        double trans_dx = 0.0, trans_dy = 0.0, trans_da = 0.0;
        double corr_x = 0.0, corr_y = 0.0;
        bool lk_trans_enabled = g_lk_trans_enable.load();
        if (!profile_calib && lk_trans_enabled && (mode == EisMode::HYBRID || mode == EisMode::LK)) {
            const Mat& trans_base = (mode == EisMode::HYBRID) ? gyro_out : frame;
            Mat curr_trans_gray;
            cvtColor(trans_base, curr_trans_gray, COLOR_BGR2GRAY);
            Mat curr_small;
            trans.scale = g_lk_trans_scale.load();
            if (trans.scale > 0.0 && trans.scale < 1.0) {
                cv::resize(curr_trans_gray, curr_small, cv::Size(), trans.scale, trans.scale, cv::INTER_LINEAR);
            } else {
                curr_small = curr_trans_gray;
            }
            trans.frame_count++;
            bool do_lk = (trans.frame_count % std::max(1, g_lk_trans_every_n.load()) == 0);
            if (!skip_heavy && trans.ok && do_lk) {
                const Mat& prev_use = trans.prev_small.empty() ? trans.prev_gray : trans.prev_small;
                const Mat& curr_use = curr_small;
                trans_ok = estimate_lk_transform(prev_use, curr_use,
                                                 trans_dx, trans_dy, trans_da);
                if (trans_ok) {
                    if (!trans.prev_small.empty() && trans.scale > 0.0) {
                        trans_dx /= trans.scale;
                        trans_dy /= trans.scale;
                    }
                    trans.path_x += trans_dx;
                    trans.path_y += trans_dy;
                    if (!trans.smooth_init) {
                        trans.smooth_x = trans.path_x;
                        trans.smooth_y = trans.path_y;
                        trans.smooth_init = true;
                    } else {
                        double a = g_lk_trans_alpha.load();
                        trans.smooth_x = a * trans.smooth_x + (1.0 - a) * trans.path_x;
                        trans.smooth_y = a * trans.smooth_y + (1.0 - a) * trans.path_y;
                    }
                    double max_corr = g_lk_trans_max_corr_px.load();
                    corr_x = std::clamp(trans.smooth_x - trans.path_x, -max_corr, max_corr);
                    corr_y = std::clamp(trans.smooth_y - trans.path_y, -max_corr, max_corr);
                    trans.last_corr_x = corr_x;
                    trans.last_corr_y = corr_y;
                } else {
                    trans.last_corr_x = 0.0;
                    trans.last_corr_y = 0.0;
                }
            }

            corr_x = trans.last_corr_x;
            corr_y = trans.last_corr_y;
            if (std::abs(corr_x) > 1e-6 || std::abs(corr_y) > 1e-6) {
                Mat Ht = (Mat_<double>(2, 3) << 1, 0, corr_x,
                                               0, 1, corr_y);
                warpAffine(trans_base, stabilized, Ht, trans_base.size(), INTER_LINEAR, BORDER_REPLICATE);
            } else {
                stabilized = trans_base;
            }
            trans.prev_gray = curr_trans_gray.clone();
            trans.prev_small = curr_small.clone();
            trans.ok = true;
        }

        double imu_err_ms = 0.0;
        if (gyro_ok && dbginfo.range.used > 0) {
            imu_err_ms = target_curr_ms - dbginfo.range.max_ts;
            double err_abs = std::abs(imu_err_ms);
            imu_err_sum += err_abs;
            imu_err_max = std::max(imu_err_max, err_abs);
            imu_err_cnt++;
        }

        if (lk_ok && gyro_delta_ok) {
            corr_buf.emplace_back(lk_da, gyro_delta[2]);
            if ((int)corr_buf.size() > corr_window) corr_buf.pop_front();
            corr_val = compute_corr(corr_buf);
        }

        if (dbg) {
            char buf[256];
            const int font = FONT_HERSHEY_SIMPLEX;
            snprintf(buf, sizeof(buf), "TS:%s off:%+.2fms err:%+.2fms mode:%s out:%s gy:%s",
                     ts_source_str(ts_src), time_offset_ms, imu_err_ms,
                     mode_str(mode), output_mode_str((OutputMode)g_output_mode.load()),
                     gyro_warp_str((GyroWarpMode)g_gyro_warp_mode.load()));
            putText(stabilized, buf, Point(8, 18), font, 0.4, Scalar(0, 255, 0), 1, LINE_AA);

            cv::Vec3d e_corr = quat_to_euler_deg(dbginfo.q_corr);
            snprintf(buf, sizeof(buf), "Gyro corr R:%+.2f P:%+.2f Y:%+.2f  crop:%.1f%%",
                     e_corr[0], e_corr[1], e_corr[2], dbginfo.crop_percent);
            putText(stabilized, buf, Point(8, 33), font, 0.4, Scalar(255, 200, 0), 1, LINE_AA);

            // Optical flow overlay (debug only)
            Mat flow_curr_gray;
            cvtColor(frame, flow_curr_gray, COLOR_BGR2GRAY);
            FlowDebugInfo flowinfo;
            if (flow_prev_ok && draw_optical_flow_overlay(flow_prev_gray, flow_curr_gray, stabilized, flowinfo)) {
                snprintf(buf, sizeof(buf), "OF dx:%.2f dy:%.2f da:%.2f",
                         flowinfo.dx, flowinfo.dy, flowinfo.da * 180.0 / CV_PI);
                putText(stabilized, buf, Point(8, 48), font, 0.4, Scalar(0, 200, 255), 1, LINE_AA);
                snprintf(buf, sizeof(buf), "OF mean all:(%.1f,%.1f) center:(%.1f,%.1f) edge:(%.1f,%.1f)",
                         flowinfo.mean_all.x, flowinfo.mean_all.y,
                         flowinfo.mean_center.x, flowinfo.mean_center.y,
                         flowinfo.mean_edge.x, flowinfo.mean_edge.y);
                putText(stabilized, buf, Point(8, 63), font, 0.35, Scalar(0, 200, 255), 1, LINE_AA);
            }
            flow_prev_gray = flow_curr_gray.clone();
            flow_prev_ok = true;
        }

        if (log_every > 0 && (frameIdx % (guint64)log_every == 0)) {
            if (imu_err_cnt > 0) {
                double avg_err = imu_err_sum / imu_err_cnt;
                fprintf(stderr, "[SYNC] imu_err avg=%.3f ms max=%.3f ms offset=%.3f ms\n",
                        avg_err, imu_err_max, time_offset_ms);
            }

            if (ts_src == TsSource::SENSOR) {
                fprintf(stderr, "[TS] sensor_ts_ns=%lld exp_us=%d idx=%llu\n",
                        (long long)sensor_ts_ns, exp_us,
                        (unsigned long long)cap_index);
            }

            fprintf(stderr,
                    "[ALIGN] src=%s frame=%.2f target=%.2f win=[%.2f,%.2f] "
                    "range=[%.2f,%.2f] imu_err=%.2fms\n",
                    ts_source_str(ts_src),
                    frame_time_ms,
                    target_curr_ms,
                    dbginfo.range.t0,
                    dbginfo.range.t1,
                    dbginfo.range.min_ts,
                    dbginfo.range.max_ts,
                    imu_err_ms);

            GyroSample last = g_last_gyro_sample;
            cv::Vec3d e_phys = quat_to_euler_deg(dbginfo.q_phys);
            cv::Vec3d e_virt = quat_to_euler_deg(dbginfo.q_virtual);
            cv::Vec3d e_corr = quat_to_euler_deg(dbginfo.q_corr);
            fprintf(stderr,
                    "[GYRO] raw(gx,gy,gz)=%.1f %.1f %.1f  rate(rad/s)=%.3f %.3f %.3f  "
                    "phys(deg)=%.2f %.2f %.2f  virt(deg)=%.2f %.2f %.2f  corr(deg)=%.2f %.2f %.2f  hz=%.1f\n",
                    last.raw[0], last.raw[1], last.raw[2],
                    last.w_rad[0], last.w_rad[1], last.w_rad[2],
                    e_phys[0], e_phys[1], e_phys[2],
                    e_virt[0], e_virt[1], e_virt[2],
                    e_corr[0], e_corr[1], e_corr[2],
                    g_imu_actual_hz.load());

            fprintf(stderr,
                    "[CORR] warp=%s raw(deg)=%.2f %.2f %.2f  clamped(deg)=%.2f %.2f %.2f  "
                    "req_scale=%.2f req_crop=%.1f%% actual_crop=%.1f%%\n",
                    gyro_warp_str((GyroWarpMode)g_gyro_warp_mode.load()),
                    dbginfo.e_corr_raw[0] * 180.0 / CV_PI,
                    dbginfo.e_corr_raw[1] * 180.0 / CV_PI,
                    dbginfo.e_corr_raw[2] * 180.0 / CV_PI,
                    dbginfo.e_corr_clamped[0] * 180.0 / CV_PI,
                    dbginfo.e_corr_clamped[1] * 180.0 / CV_PI,
                    dbginfo.e_corr_clamped[2] * 180.0 / CV_PI,
                    dbginfo.required_scale,
                    dbginfo.required_crop_percent,
                    (!profile_calib && FIXED_CROP_PERCENT > 0.0) ? FIXED_CROP_PERCENT : 0.0);

            if (lk_ok && gyro_delta_ok) {
                fprintf(stderr,
                        "[CMP] lk_da=%.3fdeg gyro_dyaw=%.3fdeg diff=%.3fdeg corr=%.3f\n",
                        lk_da * 180 / CV_PI,
                        gyro_delta[2] * 180 / CV_PI,
                        (lk_da - gyro_delta[2]) * 180 / CV_PI,
                        corr_val);
            }

            if (gyro_ok) {
                fprintf(stderr,
                        "[WIN] t0=%.2f t1=%.2f used=%d range=[%.2f,%.2f]\n",
                        dbginfo.range.t0, dbginfo.range.t1, dbginfo.range.used,
                        dbginfo.range.min_ts, dbginfo.range.max_ts);
            } else if (have_target_prev) {
                fprintf(stderr,
                        "[WIN] t0=%.2f t1=%.2f used=%d range=[%.2f,%.2f]\n",
                        delta_range.t0, delta_range.t1, delta_range.used,
                        delta_range.min_ts, delta_range.max_ts);
            }

            if (trans_ok) {
                fprintf(stderr,
                        "[TRANS] dx=%.2f dy=%.2f corr=%.2f %.2f\n",
                        trans_dx, trans_dy, corr_x, corr_y);
            }
        }

        if (!profile_calib && FIXED_CROP_PERCENT > 0.0) {
            stabilized = centerCropAndResize(stabilized, FIXED_CROP_PERCENT);
        }

        GstAppSrc *rawsrc, *stabsrc;
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            rawsrc = g_rawsrc;
            stabsrc = g_stabsrc;
        }

        Mat raw_out = dbg ? frame.clone() : frame;
        if (dbg) {
            char buf[160];
            snprintf(buf, sizeof(buf), "mode:%s off:%+.1fms",
                     mode_str(mode), time_offset_ms);
            putText(raw_out, buf, Point(8, 18), FONT_HERSHEY_SIMPLEX, 0.35, Scalar(0, 255, 0), 1, LINE_AA);
        }

        // CAM only for now (raw disabled to reduce load)
        push_bgr(stabsrc, stabilized, frameIdx, "cam");
        frameIdx++;

        last_cost_ms = (clock_ns(CLOCK_MONOTONIC_RAW) - frame_start_ns) / 1e6;
    }

    if (use_libcamera) {
        lc.shutdown();
    } else {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(sink);
        gst_object_unref(pipeline);
    }
    fprintf(stderr, "[Capture] Thread exiting\n");
}
