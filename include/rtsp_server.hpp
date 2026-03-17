#pragma once

#include <opencv2/opencv.hpp>
#include <gst/app/gstappsrc.h>
#include <gst/rtsp-server/rtsp-server.h>

void set_appsrc_caps(GstAppSrc* appsrc);
void on_media_configure(GstRTSPMediaFactory*, GstRTSPMedia* media, gpointer user_data);
GstRTSPMediaFactory* make_factory(const char* appsrc_name);
bool push_bgr(GstAppSrc* appsrc, const cv::Mat& frame, guint64 idx, const char* tag);
