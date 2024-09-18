#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include <string.h>
#include "esp_cpu.h"
#include <inttypes.h>
#include "esp_task_wdt.h"

#define TARGET_MAC "aa:bb:cc:22:22:33"
#define MAX_PACKETS 10000

static QueueHandle_t packetQueue = NULL;

typedef enum {
    PACKET_QOS,
    PACKET_ACK,
    PACKET_UNKNOWN
} PacketType;

typedef struct {
    uint32_t timestamp;
    uint8_t frame_type;
    uint8_t frame_subtype;
    uint8_t addr1[6]; // 对于一些帧，目标MAC可能在addr1
    uint8_t addr2[6]; // 对于其他帧，目标MAC可能在addr2
    int rssi; // 添加RSSI字段
} packet_info_t;


typedef struct {
    uint16_t frame_control;
    uint16_t duration_id;
    uint8_t addr1[6];
    uint8_t addr2[6];
    uint8_t addr3[6];
    uint16_t seq_ctrl;
    // Add more fields if necessary, depending on the frame types you're dealing with
} ieee80211_hdr_t;

static const char *TAG = "wifi_sniffer";
static uint8_t target_mac[6];

void IRAM_ATTR parse_mac_address(const char *mac_str, uint8_t *mac) {
    int values[6];
    if (6 == sscanf(mac_str, "%x:%x:%x:%x:%x:%x", &values[0], &values[1], &values[2], &values[3], &values[4], &values[5])) {
        for (int i = 0; i < 6; ++i) {
            mac[i] = (uint8_t)values[i];
        }
    }
}

bool IRAM_ATTR compare_mac(const uint8_t *mac1, const uint8_t *mac2) {
    return memcmp(mac1, mac2, 6) == 0;
}

void IRAM_ATTR wifi_promiscuous_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
    uint32_t timestamp_cycles = esp_cpu_get_cycle_count();
    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
    const ieee80211_hdr_t *hdr = (const ieee80211_hdr_t *)pkt->payload;
    int rssi = pkt->rx_ctrl.rssi;

    packet_info_t packetInfo = {
        .timestamp = timestamp_cycles,
        .frame_type = (hdr->frame_control & 0x0C) >> 2,
        .frame_subtype = (hdr->frame_control & 0xF0) >> 4,
        .rssi = rssi,
    };
    memcpy(packetInfo.addr1, hdr->addr1, 6); // 捕获addr1
    memcpy(packetInfo.addr2, hdr->addr2, 6); // 捕获addr2

    if (xQueueSendFromISR(packetQueue, &packetInfo, NULL) != pdPASS) {
        ESP_LOGW(TAG, "Failed to send packet info to queue.");
    }
}



// Initialize Wi-Fi in promiscuous mode
void wifi_init_promiscuous() {
    

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Set Wi-Fi to promiscuous mode
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_cb));


    // Configure the promiscuous filter to capture all types of frames
    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_ALL
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));

    // Set control frame filter specifically
    wifi_promiscuous_filter_t ctrl_filter = {
        .filter_mask = WIFI_PROMIS_CTRL_FILTER_MASK_ALL
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_ctrl_filter(&ctrl_filter));

    

    

    ESP_LOGI(TAG, "Wi-Fi initialized in promiscuous mode");
    // Set the Wi-Fi channel
    ESP_ERROR_CHECK(esp_wifi_set_channel(10, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "Wi-Fi initialized in promiscuous mode, listening on channel 2 ");
}

void packet_handler_task(void *pvParameter) {
    packet_info_t packetInfo;
    while (1) {
        if (xQueueReceive(packetQueue, &packetInfo, portMAX_DELAY) == pdTRUE) {
            // 决定基于帧类型和子类型比较哪个地址
            const uint8_t *addr_to_compare = (packetInfo.frame_type == 1 && packetInfo.frame_subtype == 13) ? packetInfo.addr1 : packetInfo.addr2;

            if (compare_mac(addr_to_compare, target_mac)) {
                // 进行帧类型特定的处理
                if (packetInfo.frame_type == 2 && packetInfo.frame_subtype == 4) {
 //                   ESP_LOGI(TAG, "QoS,Timestamp: %" PRIu32, packetInfo.timestamp);
                    ESP_LOGI(TAG, "QoS, Timestamp: %" PRIu32 ", RSSI: %d", packetInfo.timestamp, packetInfo.rssi);
                } else if (packetInfo.frame_type == 1 && packetInfo.frame_subtype == 13) {
   //                 ESP_LOGI(TAG, "ACK,Timestamp: %" PRIu32, packetInfo.timestamp);
                    ESP_LOGI(TAG, "ACK, Timestamp: %" PRIu32 ", RSSI: %d", packetInfo.timestamp, packetInfo.rssi);
                }
                // 进一步处理...
            }
        }
    }
}

void app_main(void) {
    // Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    // Initialize the TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    // Create the default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 解析目标MAC地址
    parse_mac_address(TARGET_MAC, target_mac);

    // 创建数据包队列
    packetQueue = xQueueCreate(MAX_PACKETS, sizeof(packet_info_t));
    if (packetQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create packet queue");
        return;
    }
    // 配置和启动Wi-Fi
    wifi_init_promiscuous();
    // 创建并绑定任务到特定CPU核心
    xTaskCreatePinnedToCore(packet_handler_task, "PacketHandler", 2048, NULL, 10, NULL, 1);
    
}


