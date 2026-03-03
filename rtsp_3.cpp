#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <iostream>

static GstRTSPMediaFactory* make_relay_factory(const gchar* uri, int id) {
    GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();

    gchar* pipeline = g_strdup_printf(
        "( rtspsrc location=%s protocols=tcp latency=200 "
        "  do-rtsp-keep-alive=true tcp-timeout=5000000 retry=1000 ! "
        "  queue max-size-buffers=200 max-size-bytes=0 max-size-time=0 ! "
        "  rtph264depay ! h264parse ! queue ! "
        "  rtph264pay name=pay0 pt=96 config-interval=1 )",
        uri, id
    );

    gst_rtsp_media_factory_set_launch(factory, pipeline);
    gst_rtsp_media_factory_set_shared(factory, TRUE);
    gst_rtsp_media_factory_set_suspend_mode(factory, GST_RTSP_SUSPEND_MODE_NONE);
    gst_rtsp_media_factory_set_buffer_size(factory, 2 * 1024 * 1024);

    g_free(pipeline);
    return factory;
}

int main(int argc, char* argv[]) {
    system("fuser -k 8554/tcp 2>/dev/null");
    gst_init(&argc, &argv);

    GMainLoop*      loop   = g_main_loop_new(nullptr, FALSE);
    GstRTSPServer*  server = gst_rtsp_server_new();

    GstRTSPThreadPool* pool = gst_rtsp_thread_pool_new();
    gst_rtsp_thread_pool_set_max_threads(pool, 16);
    gst_rtsp_server_set_thread_pool(server, pool);
    g_object_unref(pool);

    gst_rtsp_server_set_service(server, "8554");
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);

    // ← 카메라 URL 목록 (콤마 꼭 넣기)
    const gchar* uris[] = {
        "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp",  // 고정 카메라
        "rtsp://192.168.100.8:8555/cam",   // Pi카메라 1
        // "rtsp://192.168.100.9:8555/cam",   // Pi카메라 2 추가 시
    };
    int cam_count = sizeof(uris) / sizeof(uris[0]);

    for (int i = 0; i < cam_count; i++) {
        gchar* path = g_strdup_printf("/cam%d", i);
        gst_rtsp_mount_points_add_factory(mounts, path, make_relay_factory(uris[i], i));
        g_print("Camera %d ready: rtsp://<서버IP>:8554%s\n", i, path);
        g_free(path);
    }

    g_object_unref(mounts);
    gst_rtsp_server_attach(server, nullptr);

    g_print("Server running on :8554\n");
    g_main_loop_run(loop);
    return 0;
}