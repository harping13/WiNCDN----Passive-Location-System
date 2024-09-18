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

#define TARGET_MAC "aa:bb:cc:11:22:33"
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
    uint8_t addr[6]; // 存储源MAC或目的MAC，取决于帧的类型和子类型
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

    // 提取帧类型和子类型
    uint8_t frame_type = (hdr->frame_control & 0x0C) >> 2;
    uint8_t frame_subtype = (hdr->frame_control & 0xF0) >> 4;

    // 初始化packet_info_t结构，但只在符合特定条件时填充
    packet_info_t packetInfo = {
        .timestamp = timestamp_cycles,
        .frame_type = frame_type,
        .frame_subtype = frame_subtype,
    };

    // 检查帧类型和子类型，决定是否处理并如何填充addr字段
    if ((frame_type == 2 && frame_subtype == 4) || (frame_type == 1 && frame_subtype == 13)) {
        // 对于QoS数据帧（源MAC在addr2）或ACK帧（目的MAC在addr1）
        const uint8_t *src_or_dst_mac = (frame_type == 2 && frame_subtype == 4) ? hdr->addr2 : hdr->addr1;
        memcpy(packetInfo.addr, src_or_dst_mac, 6);

        // 将符合条件的帧信息加入队列
        if (xQueueSendFromISR(packetQueue, &packetInfo, NULL) != pdPASS) {
            ESP_LOGW(TAG, "Failed to send packet info to queue.");
        }
    }
    // 不处理其他帧类型和子类型
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
    ESP_ERROR_CHECK(esp_wifi_set_channel(2, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "Wi-Fi initialized in promiscuous mode, listening on channel 2 ");
}

void packet_handler_task(void *pvParameter) {
    packet_info_t packetInfo;
    while (1) {
        if (xQueueReceive(packetQueue, &packetInfo, portMAX_DELAY) == pdTRUE) {
            // 直接使用packetInfo中的addr与目标MAC地址比较
            if (compare_mac(packetInfo.addr, target_mac)) {
                // 根据帧的类型和子类型进行帧类型特定的处理
                if (packetInfo.frame_type == 2 && packetInfo.frame_subtype == 4) {
                    // QoS数据帧
                    ESP_LOGI(TAG, "QoS,Timestamp: %" PRIu32, packetInfo.timestamp);
                } else if (packetInfo.frame_type == 1 && packetInfo.frame_subtype == 13) {
                    // ACK帧
                    ESP_LOGI(TAG, "ACK,Timestamp: %" PRIu32, packetInfo.timestamp);
                }
                // 如果需要，可添加其他帧类型和子类型的处理逻辑
            }
            // 如果不满足条件，不执行任何操作
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

    // 配置和启动Wi-Fi
    wifi_init_promiscuous();

    // 解析目标MAC地址
    parse_mac_address(TARGET_MAC, target_mac);

    // 创建数据包队列
    packetQueue = xQueueCreate(MAX_PACKETS, sizeof(packet_info_t));
    if (packetQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create packet queue");
        return;
    }

    // 创建并绑定任务到特定CPU核心
    xTaskCreatePinnedToCore(packet_handler_task, "PacketHandler", 2048, NULL, 10, NULL, 1);
}

