/* tx_main.c
 * WiFi station + UDP traffic generator for ESP32-S3 (ESP-IDF v5.5.3)
 */

#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

/* ---------------------------------------------------------
 * WiFi config from menuconfig
 * --------------------------------------------------------- */
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* ---------------------------------------------------------
 * User config
 * --------------------------------------------------------- */
#define TX_UDP_TARGET_IP        CONFIG_RUVIEW_TX_UDP_TARGET_IP
#define TX_UDP_TARGET_PORT      CONFIG_RUVIEW_TX_UDP_TARGET_PORT
#define NODE_ID                 CONFIG_RUVIEW_TX_NODE_ID
#define TX_INTERVAL_MS          10
#define TX_PAYLOAD_SIZE         128

/* ---------------------------------------------------------
 * FreeRTOS event bits
 * --------------------------------------------------------- */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "tx_main";
static int s_retry_num = 0;

/* ---------------------------------------------------------
 * UDP socket globals
 * --------------------------------------------------------- */
static int g_udp_sock = -1;
static struct sockaddr_in g_dest_addr;
static bool g_udp_ready = false;
static uint32_t g_seq = 0;

typedef struct __attribute__((packed)) {
    char     node_id[16];
    uint32_t seq;
    int64_t  ts_us;
    uint8_t  payload[TX_PAYLOAD_SIZE];
} tx_udp_packet_t;

/* ---------------------------------------------------------
 * UDP init
 * --------------------------------------------------------- */
static void udp_client_init(void)
{
    if (g_udp_sock >= 0) {
        close(g_udp_sock);
        g_udp_sock = -1;
    }

    g_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (g_udp_sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        g_udp_ready = false;
        return;
    }

    memset(&g_dest_addr, 0, sizeof(g_dest_addr));
    g_dest_addr.sin_family = AF_INET;
    g_dest_addr.sin_port = htons(TX_UDP_TARGET_PORT);
    g_dest_addr.sin_addr.s_addr = inet_addr(TX_UDP_TARGET_IP);

    g_udp_ready = true;
    ESP_LOGI(TAG, "UDP ready -> %s:%d", TX_UDP_TARGET_IP, TX_UDP_TARGET_PORT);
}

/* ---------------------------------------------------------
 * TX traffic task
 * --------------------------------------------------------- */
static void tx_traffic_task(void *arg)
{
    tx_udp_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    strncpy(pkt.node_id, NODE_ID, sizeof(pkt.node_id) - 1);

    while (1) {
        if (g_udp_ready && g_udp_sock >= 0) {
            pkt.seq = g_seq++;
            pkt.ts_us = (int64_t)esp_log_timestamp() * 1000;

            for (int i = 0; i < TX_PAYLOAD_SIZE; i++) {
                pkt.payload[i] = (uint8_t)((pkt.seq + i) & 0xFF);
            }

            int err = sendto(g_udp_sock,
                             &pkt,
                             sizeof(pkt),
                             0,
                             (struct sockaddr *)&g_dest_addr,
                             sizeof(g_dest_addr));

            if (err < 0) {
                static uint32_t udp_err_cnt = 0;
                udp_err_cnt++;
                if ((udp_err_cnt % 100) == 1) {
                    ESP_LOGW(TAG, "UDP send failed, errno=%d", errno);
                }
            } else {
                if ((pkt.seq % 100) == 0) {
                    ESP_LOGI(TAG, "TX seq=%" PRIu32, pkt.seq);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL_MS));
    }
}

/* ---------------------------------------------------------
 * WiFi event handler
 * --------------------------------------------------------- */
static void event_handler(void *arg,
                          esp_event_base_t event_base,
                          int32_t event_id,
                          void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        g_udp_ready = false;

        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));

        s_retry_num = 0;
        udp_client_init();

        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ---------------------------------------------------------
 * WiFi station init
 * --------------------------------------------------------- */
static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &event_handler,
        NULL,
        &instance_any_id));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &event_handler,
        NULL,
        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };

    strncpy((char *)wifi_config.sta.ssid, EXAMPLE_ESP_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, EXAMPLE_ESP_WIFI_PASS, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY
    );

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to AP SSID:%s", EXAMPLE_ESP_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", EXAMPLE_ESP_WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

/* ---------------------------------------------------------
 * app_main
 * --------------------------------------------------------- */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (CONFIG_LOG_MAXIMUM_LEVEL > CONFIG_LOG_DEFAULT_LEVEL) {
        esp_log_level_set("wifi", CONFIG_LOG_MAXIMUM_LEVEL);
    }

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA + UDP traffic generator (TX)");
    wifi_init_sta();

    xTaskCreate(tx_traffic_task, "tx_traffic_task", 4096, NULL, 5, NULL);
}
